#!/bin/bash

cd /PX4Firmware
source /opt/ros/melodic/setup.bash
DONT_RUN=1 make px4_sitl_default gazebo
source /mission9/workspace/devel/setup.bash
source Tools/setup_gazebo.bash $(pwd) $(pwd)/build/px4_sitl_default
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd)
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd)/Tools/sitl_gazebo
export GAZEBO_MODEL_PATH=${GAZEBO_MODEL_PATH}:/mission9/workspace/src/avoidance/sim/models
export GAZEBO_MODEL_PATH=${GAZEBO_MODEL_PATH}:/mission9/sim/models
export GAZEBO_PLUGIN_PATH=${GAZEBO_PLUGIN_PATH}:/mission9/sim/plugins
export GAZEBO_RESOURCE_PATH=${GAZEBO_RESOURCE_PATH}:/mission9/sim/worlds


cd /mission9/workspace
roslaunch maav maav_mavros_posix_sitl.launch
#roslaunch px4 mavros_posix_sitl.launch
