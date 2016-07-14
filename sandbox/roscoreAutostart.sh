#! /bin/bash
source /opt/ros/indigo/setup.bash
source /home/robojackets/catkin_ws/devel/setup.sh

export ROS_PACKAGE_PATH=/home/robojackets/catkin_ws/src/roboracing-software:$ROS_PACKAGE_PATH
export PATH=$PATH:$ROS_ROOT/bin
roscore
