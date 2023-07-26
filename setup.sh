#!/usr/bin/env bash

SCRIPT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )
ENV_NAME=spot-ros-peract

# Pull spot_peract into src folder
git submodule update --init --recursive

# Create conda env called ros_peract
conda create --prefix ${SCRIPT_DIR}/env -n ${ENV_NAME} python=3.8
conda activate ${ENV_NAME}
pip install -r requirements.txt

# Setup PerAct
cd ${SCRIPT_DIR}/peract_ros_wrapper/src/spot_peract
./setup.sh