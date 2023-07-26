#!/usr/bin/env bash

PROJECT_ROOT=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )
ENV_NAME=spot-peract-ros

# Pull spot_peract into src folder
git submodule update --init --recursive

# Create conda env called ros_peract
if conda info --envs | grep ${PROJECT_ROOT}/env/${ENV_NAME}; 
then echo "Env already exists"; 
else conda create --prefix ${PROJECT_ROOT}/env/${ENV_NAME} python=3.8 -y;
fi
eval "$(conda shell.bash hook)"
conda activate ${PROJECT_ROOT}/env/${ENV_NAME}
pip install --upgrade pip
pip install -r requirements.txt

# Setup PerAct
source ${PROJECT_ROOT}/peract_ros_wrapper/src/spot_peract/setup.sh

# Setup ROS
source /opt/ros/noetic/setup.bash
cd ../../
catkin build
source devel/setup.bash