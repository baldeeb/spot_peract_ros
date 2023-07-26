#!/usr/bin/env bash

SCRIPT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )
ENV_NAME=spot-peract-ros

# Pull spot_peract into src folder
git submodule update --init --recursive


# Create conda env called ros_peract
if conda info --envs | grep ${SCRIPT_DIR}/env/${ENV_NAME}; 
then echo "env already exists"; 
else conda create --prefix ${SCRIPT_DIR}/env/${ENV_NAME} python=3.8 -y;
fi

# conda create --prefix ${SCRIPT_DIR}/env/${ENV_NAME} python=3.8 -y
eval "$(conda shell.bash hook)"
conda activate ${SCRIPT_DIR}/env/${ENV_NAME}
# pip install -r requirements.txt

# Setup PerAct
source ${SCRIPT_DIR}/peract_ros_wrapper/src/spot_peract/setup.sh