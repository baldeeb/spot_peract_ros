# Pull spot_peract into src folder
git submodule update --init --recursive


# Clone spot_mesgs from RPM lab's spot_ros repo
# ref: https://stackoverflow.com/questions/600079/how-do-i-clone-a-subdirectory-only-of-a-git-repository
cd src
# git clone -n --depth=1 --filter=tree:0 git@github.com:RPM-lab-UMN/spot_ros.git
git update-index --add --cacheinfo 160000,695e3df21aaba6cda628ff6312efeff08acc0451,git@github.com:RPM-lab-UMN/spot_ros.git
cd spot_ros
git sparse-checkout init
git sparse-checkout set spot_msgs
git checkout HEAD
cd ../..


# Create conda env called ros_peract
conda create -n ros-peract python=3.8
conda activate ros-peract
pip install -r requirements.txt

