#!/usr/bin/env bash

# There is no sleeping/waiting in launch files, so
# unfortunately we're left with this. The dynamic
# reconfigure settings should be applied after all
# the defaults have been applied

sleep 10
roslaunch rr_platform bigoli_camera_dynconfig.launch
