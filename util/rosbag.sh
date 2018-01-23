sleep $1
roslaunch rr_platform rosbag.launch &
sleep $2
rosnode kill record
