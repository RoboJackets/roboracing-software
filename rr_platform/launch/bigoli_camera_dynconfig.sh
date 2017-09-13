#!/usr/bin/env bash

# There is no sleeping/waiting in launch files, so
# unfortunately we're left with this

sleep 10
roslaunch rr_platform bigoli_camera_dynconfig.launch
