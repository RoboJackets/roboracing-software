#!/usr/bin/env python
"""
Measures the response time of the evgp car by moving a box in
front of the car then measuring how long it takes the lidar scan, map,
and planned path to reflect the update.

@author Daniel Martin
"""


from __future__ import print_function

import time
import itertools
import signal
import sys

import rospy
from gazebo_msgs.srv import SetModelState, SetModelStateRequest

from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Path
import numpy as np

scan_empty_ = True
map_empty_ = True
path_straight_ = True

scan_pop_time_ = time.time()
map_pop_time_ = time.time()
path_pop_time_ = time.time()

car_inscribed_radius_ = 0
path_y_divergence_threshold_ = 0


def make_request(traj_point, model_name):
    x, y, z, q0, q1, q2, q3 = traj_point

    request = SetModelStateRequest()
    request.model_state.model_name = model_name
    request.model_state.reference_frame = "world"
    request.model_state.pose.position.x = x
    request.model_state.pose.position.y = y
    request.model_state.pose.position.z = z
    request.model_state.pose.orientation.x = q0
    request.model_state.pose.orientation.y = q1
    request.model_state.pose.orientation.z = q2
    request.model_state.pose.orientation.w = q3

    return request

def scan_callback(scan):
    # Scan is empty when all values detected are either within the car's footprint or infinity
    scan_empty = np.all(np.logical_or(np.array(scan.ranges) < car_inscribed_radius_, np.isinf(scan.ranges)))

    global scan_empty_, scan_pop_time_
    if scan_empty_ and not scan_empty:
        scan_pop_time_ = time.time()
    scan_empty_ = scan_empty


def map_callback(map):
    # Map is empty when there are only zero values
    map_empty = np.count_nonzero(map.data) == 0

    global map_empty_, map_pop_time_
    if map_empty_ and not map_empty:
        map_pop_time_ = time.time()
    map_empty_ = map_empty

def path_callback(path):
    # Path is straight when there are not obstacles in front of it
    path_straight = abs(path.poses[-1].pose.position.y) <= path_y_divergence_threshold_

    global path_straight_, path_pop_time_
    if path_straight_ and not path_straight:
        path_pop_time_ = time.time()
    path_straight_ = path_straight


def main():
    rospy.init_node("box_trajectory_controller", anonymous=True)

    def handler(signum, frame):
        print("shutting down", rospy.get_name())
        sys.exit(signum)

    signal.signal(signal.SIGINT, handler)  # set_model_state won't terminate without this

    global car_inscribed_radius_, path_y_divergence_threshold_
    model_name = rospy.get_param("~model_name", "LongBox")
    lidar_scan = rospy.get_param("~scan_topic", "/scan")
    costmap_topic = rospy.get_param("~costmap_topic", "/local_mapper/costmap/costmap")
    path_topic = rospy.get_param("~path_topic", "/plan/path")
    world_full_obstacle_pose = rospy.get_param("~world_full_obstacle_pose", "15 0 0.5 0 0 0 0")
    world_empty_obstacle_pose = rospy.get_param("~world_empty_obstacle_pose", "500 0 0.5 0 0 0 0")
    car_inscribed_radius_ = rospy.get_param("~car_inscribed_radius", 5)
    path_y_divergence_threshold_ = rospy.get_param("~path_y_divergence_threshold", 5)
    wait_time = rospy.get_param("~wait_time_between_states", 1)

    # Convert pose string to list of floats
    world_full_obstacle_pose = map(float, world_full_obstacle_pose.split(" "))
    world_empty_obstacle_pose = map(float, world_empty_obstacle_pose.split(" "))

    rospy.Subscriber(lidar_scan, LaserScan, scan_callback)
    rospy.Subscriber(costmap_topic, OccupancyGrid, map_callback)
    rospy.Subscriber(path_topic, Path, path_callback)

    rospy.wait_for_service("/gazebo/set_model_state")
    set_model_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)

    scan_pop_dur_sum = 0
    map_pop_dur_sum = 0
    path_pop_dur_sum = 0

    for loop in itertools.count(start=1):
        print("Blocked removed...")
        set_model_state(make_request(world_empty_obstacle_pose, model_name))
        time.sleep(wait_time)

        print("Waiting for all topics to declare state: WORLD_EMPTY...")
        while not scan_empty_ or not map_empty_ and not path_straight_:
            time.sleep(.1)

        # Moving object forwards and timing reaction
        print("Blocked added...")
        set_model_state(make_request(world_full_obstacle_pose, model_name))
        block_moved = time.time()

        while scan_empty_ or map_empty_ or path_straight_:
            time.sleep(.1)

        if time.time() - block_moved < 5:
            scan_pop_dur = scan_pop_time_ - block_moved
            map_pop_dur = map_pop_time_ - scan_pop_time_
            path_pop_dur = path_pop_time_ - map_pop_time_

            scan_pop_dur_sum += scan_pop_dur
            map_pop_dur_sum += map_pop_dur
            path_pop_dur_sum += path_pop_dur

            print("--- Loop {} ---".format(loop))
            print("Scan took: {} \nMap took:  {} \nPath took: {}".format(scan_pop_dur, map_pop_dur, path_pop_dur))
            print("Total: {}".format(path_pop_time_ - block_moved))
            print("Scan Avg:  {} \nMap Avg:   {} \nPath Avg:  {}".format(scan_pop_dur_sum / loop, map_pop_dur_sum / loop, path_pop_dur_sum / loop))
        time.sleep(wait_time)

    rospy.spin()


if __name__ == '__main__':
    main()
