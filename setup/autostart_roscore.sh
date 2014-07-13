#! /bin/bash

source /opt/ros/groovy/setup.bash
source ~/catkin_ws/devel/setup.sh

export ROS_PACKAGE_PATH=~/catkin_ws/src/iarrc:$ROS_PACKAGE_PATH
export PATH=$PATH:$ROS_ROOT/bin

roscore
