#! /bin/bash

sleep 3

roslaunch rr_gazebo spawn_macaroni_platform.launch x:=$1 y:=$2 heading:=$3
