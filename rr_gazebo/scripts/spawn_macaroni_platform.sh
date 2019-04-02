#! /bin/bash

sleep 3

roslaunch rr_gazebo spawn_macaroni_platform.launch x:=$1 y:=$2 heading:=$3

# set gazebo physics step size so that it uses less CPU resources
sleep 2
gz physics --step-size 0.0025 --update-rate 400 --iters 30
