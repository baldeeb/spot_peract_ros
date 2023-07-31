#!/usr/bin/env python

import os
import yaml
import numpy as np
from enum import Enum
from copy import deepcopy
from scipy.spatial.transform import Rotation as R
from quaternion import quaternion, as_rotation_matrix
import rospy
import actionlib
from cv_bridge import CvBridge
from tf2_ros import Buffer, TransformListener
from message_filters import Subscriber as MFSubscriber
from message_filters import ApproximateTimeSynchronizer
from std_msgs.msg import Duration, Empty
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped, PoseStamped
from spot_msgs.msg import TaskState, TrajectoryAction, TrajectoryGoal
from spot_peract.agents.peract_spot_bc.inference_agent import PeractAgentInterface
from pprint import pprint as pp

def _ros_image_to_np(image:Image, depth_to_meters=1e-1):
    H, W = image.height, image.width
    if image.encoding == 'rgb8':
        rgb = np.frombuffer(image.data, dtype=np.byte)
        img = rgb.reshape(H, W, 3).astype(np.uint8)
    elif image.encoding == 'rgba8':
        rgb = np.frombuffer(image.data, dtype=np.byte)
        img = rgb.reshape(H, W, 4).astype(np.uint8)
    elif image.encoding == 'bgra8':
        rgb = np.frombuffer(image.data, dtype=np.byte)
        img = rgb.reshape(H, W, 4)[:, :, (2,1,0)].astype(np.uint8)
    elif image.encoding == '16UC1':
        d = np.frombuffer(image.data, dtype=np.uint16).reshape(H, W)
        img = d.astype(np.float32) * depth_to_meters
    elif image.encoding == 'bgra8':
        rgbd = np.frombuffer(image.data, dtype=np.byte)
        rgbd = rgbd.reshape(H, W, 4)[:, :, (2,1,0)].astype(np.uint8)
        img = rgbd[:,:,3].astype(np.uint16).astype(np.float32) * depth_to_meters
    else: 
        raise RuntimeError(f'Image to Numpy is not setup to handle {image.encoding}.')
    return img

def _ros_camera_info_to_np_intrinsic(info:CameraInfo):
    '''
    From rosbag2dataset.py, Bahaa Aldeeb
    '''
    return np.array([
        [info.K[0], info.K[1], info.K[2]],
        [info.K[3], info.K[4], info.K[5]],
        [info.K[6], info.K[7], info.K[8]]
    ])

def _ros_pose_to_np_se3_matrix(pose:PoseWithCovarianceStamped):
    p, o = pose.pose.pose.position, pose.pose.pose.orientation
    return _position_orientation_to_np_se3_matrix(
                        np.array([p.x, p.y, p.z]), 
                        np.array([o.w, o.x, o.y, o.z]))

def _tf_to_se3_matrix(tf:TransformStamped):
    t, r = tf.transform.translation, tf.transform.rotation
    return _position_orientation_to_np_se3_matrix(
                        np.array([t.x, t.y, t.z]), 
                        np.array([r.w, r.x, r.y, r.z]))

def _position_orientation_to_np_se3_matrix(position:np.ndarray, 
                                           orientation:np.ndarray):
    '''
    Args:
        position: x, y, z
        orientation: w, x, y, z
    '''
    mat = np.eye(4)
    q = quaternion(*orientation)
    mat[:3,:3] = as_rotation_matrix(q)
    mat[:3,3] = position
    return mat

def _transform_pose(pose, rot_vec=[0, 0, np.pi/2], offset=[0.0, 0.0, 0.0]):
    rot = R.from_rotvec(rot_vec)
    quat = pose.orientation
    pose_rot = R.from_quat([quat.x, quat.y, quat.z, quat.w])
    pose_quat = (pose_rot * rot).as_quat()
    quat.x, quat.y, quat.z, quat.w = pose_quat
    
    offset = pose_rot.as_matrix() @ np.array(offset).T
    p = pose.position
    p.x += offset[0]; p.y += offset[1]; p.z += offset[2]
    pose.position = p
    return pose 

def _get_ros_stamped_pose(position, orientation, frame_id='body'):
    p, o = position, orientation
    return _get_ros_stamped_from_pose_ndarrays([p.x, p.y, p.z],
                                               [o.x, o.y, o.z, o.w],
                                               frame_id)

def _get_ros_stamped_from_pose_ndarrays(position:np.ndarray, orientation:np.ndarray, frame_id='body'):
    pose = PoseStamped()
    pose.header.stamp = rospy.Time.now()
    pose.header.frame_id = frame_id
    pi, oi = position, orientation
    p, o = pose.pose.position, pose.pose.orientation
    o.x, o.y, o.z, o.w = oi[0], oi[1], oi[2], oi[3]
    p.x, p.y, p.z = pi[0], pi[1], pi[2]
    return pose

################### CLASSES #####################

# subscribing to topics and storing in dictionary
class TopicData:

    # DATA STORED IN topicData.topic_data as dictionary

    def __init__(self, cfg):
        self.topics_info = cfg['topics_info']
        self._data = {}
        self.cv_bridge = CvBridge()

        # Agent 
        self.agent = PeractAgentInterface(cfg)

        # State Machine
        self.STATE = Enum('STATE', ('GET_LANGUAGE_CMD', 
                                    'GET_SENSOR_DATA',
                                    'DISPLAY_ACTION',
                                    'REQUEST_ACTION',))
        self._state = self.STATE.GET_LANGUAGE_CMD

        # ROS Listeners
        self.tf_buffer = Buffer(cache_time=rospy.Duration(25.0))
        self.listener = TransformListener(self.tf_buffer)
        self._subs, self._sync = self._register_callbacks()
        self._last_confirmation = rospy.Time.now()
        self._alloted_confirmation_time = 5.0
        self._confirm_action_sub = rospy.Subscriber('execute_action', Empty, 
                                                    self._action_confirmation_callback)
        # Publishers
        self._pred_pub = rospy.Publisher('pred_waypoint',
                                          TaskState,
                                          queue_size=1)
        self.pred_voxel_pub = rospy.Publisher('pred_voxel_render', 
                                              Image,
                                              latch=True, 
                                              queue_size=10)
        
        # Action Client
        self.request_action = ActionClient()

    
    def _action_confirmation_callback(self, _):
        self._last_confirmation = rospy.Time.now()
    
    
    @property
    def _action_confirmed(self):
        dt = (rospy.Time.now() - self._last_confirmation).to_sec()
        return dt < 1.0
    

    def _register_callbacks(self, 
                           allow_headerless:bool=True, 
                           queue_size:int=5, 
                           slop:float=1.5, # 0.25
                           ):
        ''' Creates subscribers, synchronizer, and sets up callbacks '''
        subs, sync = {}, None
        for info in self.topics_info:
            sub = MFSubscriber(info['topic'], globals()[info['type']])
            subs[info['name']] = sub
        sync = ApproximateTimeSynchronizer(list(subs.values()), 
                                           queue_size, slop,
                                           allow_headerless=allow_headerless)
        sync.registerCallback(self._synched_data_callback)
        return subs, sync


    def get_ros_data_as_dict(self, ros_data, reference_frame='body'):
        results = {}
        for i, v in enumerate(ros_data):
            k = self.topics_info[i]['name']
            if isinstance(v, CameraInfo):
                results[k] = _ros_camera_info_to_np_intrinsic(v)
            elif isinstance(v, Image):
                tf = self.tf_buffer.lookup_transform(reference_frame, 
                                                    v.header.frame_id, 
                                                    v.header.stamp)
                results[k] = {
                    'image': _ros_image_to_np(v, depth_to_meters=1e-3),
                    'extrinsic': _tf_to_se3_matrix(tf)
                }
        return results



    def _publish_pred_action(self, skill:str, pose:PoseStamped):
        if skill == 'goto':
            ts = TaskState()
            ts.task_name = 'goto'
            ts.target_pose = pose
            self._pred_pub.publish(ts)


    def _decipher_action(self, action):
        # returns action name and target pose
        rospy.loginfo(f"Action: {action['position']}")
        pose = _get_ros_stamped_from_pose_ndarrays(action['position'], 
                                                list(action['quaternion'].values()), 
                                                'body')
        return 'goto', pose


    def _visualize_rendered_voxel(self, render):
        img = self.cv_bridge.cv2_to_imgmsg(render,encoding='rgb8')
        self.pred_voxel_pub.publish(img)


    def _await_confirmation(self):
        t0 = rospy.Time.now()
        while not rospy.is_shutdown() and self._state == self.STATE.DISPLAY_ACTION:
            if self._action_confirmed:
                rospy.loginfo('Action confirmed, requesting action...')
                self._state = self.STATE.REQUEST_ACTION
            elif (rospy.Time.now() - t0).to_sec() > self._alloted_confirmation_time:
                rospy.loginfo('Action not confirmed, re-processing...')
                self._state = self.STATE.GET_SENSOR_DATA
            rospy.sleep(0.2)


    def _step(self):
        if self._state == self.STATE.GET_LANGUAGE_CMD:
            # Request Language Command
            self._data['language'] = input('Enter language command...')
            self._data['language'] = 'move to left of the chair' # TODO: remove
            self._state = self.STATE.GET_SENSOR_DATA
        elif self._state == self.STATE.DISPLAY_ACTION:
            # Get waypoint and visualize
            action, render = self.agent.step(self._peract_data)
            self._visualize_rendered_voxel(render)
            self._proposed_skill, self._proposed_pose = self._decipher_action(action)
            self._publish_pred_action(self._proposed_skill, self._proposed_pose)
            self._await_confirmation()
        elif self._state == self.STATE.REQUEST_ACTION:
            self.request_action('goto', self._proposed_pose)
            self._state = self.STATE.GET_SENSOR_DATA
    @property
    def _peract_data(self):
        d = {
            'lang_goal': self._data['language'],
            'cameras': {},
            'finger_positions': np.array([1.0, 1.0]),
            'step': 0,
            'episode_length': 20,
        }
        def _cam_data(cam_name):
            return {
                'rgb': self._data[f'{cam_name}_image']['image'],
                'depth': self._data[f'{cam_name}_depth']['image'],
                'camera_intrinsics': self._data[f'{cam_name}_camera_info'],
                'camera_extrinsics': self._data[f'{cam_name}_image']['extrinsic'],
            }
        assert all([f'{cam}_image' in self._data for cam in ['frontleft', 'frontright', 'left', 'right']]), 'wtf'
        if 'frontleft_image' in self._data:
            d['cameras']['left_shoulder'] = _cam_data('frontleft')
        if 'frontright_image' in self._data:
            d['cameras']['right_shoulder'] = _cam_data('frontright')
        if 'left_image' in self._data:
            d['cameras']['overhead'] = _cam_data('left')
        if 'right_image' in self._data:
            d['cameras']['front'] = _cam_data('right')
        return d


    def _synched_data_callback(self, *data):
        if self._state == self._state.GET_SENSOR_DATA:
            rospy.loginfo('Updating sensor data...')
            self._data.update(self.get_ros_data_as_dict(deepcopy(data)))
            self._state = self.STATE.DISPLAY_ACTION


# action client class for sending goals to the robot
class ActionClient:
    def __init__(self):
        self.services = {
            'goto':  actionlib.SimpleActionClient('spot/trajectory', TrajectoryAction),
            # 'grasp': 'spot/grasp',
            # 'drag': 'spot/drag'
        }
        for k, v in self.services.items():
            rospy.loginfo(f"Wating for {k} action server...")
            v.wait_for_server()

    def _goto(self, pose:PoseStamped):
        duration = Duration()
        duration.data.secs = 10
        goal = TrajectoryGoal(target_pose=pose, duration=duration)
        srv = self.services['goto']
        srv.send_goal(goal)
        srv.wait_for_result()
        result = srv.get_result()
        rospy.loginfo(f"Result: {result}")

    def __call__(self, service, pose:PoseStamped):
        if service == 'goto': self._goto(pose)
        elif service == 'grasp': self._grasp(pose)
        elif service == 'drag': self._drag(pose)



if __name__ == '__main__':

    rospy.init_node('peract_wrapper', anonymous=True)
    # Get directory of the current file 
    dir_path = os.path.dirname(os.path.realpath(__file__))
    config = open(f"{dir_path}/../config/base.yaml", "r")
    cfg = yaml.load(config, Loader=yaml.FullLoader)
    topicData = TopicData(cfg)
    rate = rospy.Rate(cfg['node_rate'])
    while not rospy.is_shutdown():
        topicData._step()
        rate.sleep()