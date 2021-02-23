#!/usr/bin/env python

from __future__ import print_function

import csv
import time
import itertools
import math
import signal
import sys

import rospy
from gazebo_msgs.srv import SetModelState, SetModelStateRequest
from rosgraph_msgs.msg import Clock
import tf.transformations


def smallest_angle(a0, a1):
    return min([a1 - a0, a1 - a0 - 2 * math.pi, a1 - a0 + 2 * math.pi], key=abs)


def make_request(traj_point, next_traj_point, model_name, time_warp, z_offset):
    t1, x1, y1, z1, q01, q11, q21, q31 = next_traj_point
    t0, x0, y0, z0, q00, q10, q20, q30 = traj_point

    request = SetModelStateRequest()
    request.model_state.model_name = model_name
    request.model_state.reference_frame = "world"
    request.model_state.pose.position.x = x0
    request.model_state.pose.position.y = y0
    request.model_state.pose.position.z = z0 + z_offset
    request.model_state.pose.orientation.x = q00
    request.model_state.pose.orientation.y = q10
    request.model_state.pose.orientation.z = q20
    request.model_state.pose.orientation.w = q30

    dt = (t1 - t0) / time_warp
    R0, P0, Y0 = tf.transformations.euler_from_quaternion([q00, q10, q20, q30])
    R1, P1, Y1 = tf.transformations.euler_from_quaternion([q01, q11, q21, q31])
    request.model_state.twist.linear.x = (x1 - x0) / dt
    request.model_state.twist.linear.y = (y1 - y0) / dt
    request.model_state.twist.linear.z = (z1 - z0) / dt
    request.model_state.twist.angular.x = smallest_angle(R0, R1) / dt
    request.model_state.twist.angular.y = smallest_angle(P0, P1) / dt
    request.model_state.twist.angular.z = smallest_angle(Y0, Y1) / dt

    return request


def main():
    rospy.init_node("opponent_trajectory_controller", anonymous=True)

    def handler(signum, frame):
        print("shutting down", rospy.get_name())
        sys.exit(signum)

    signal.signal(signal.SIGINT, handler)  # set_model_state won't terminate without this

    in_csv = rospy.get_param("~trajectory_csv")
    model_name = rospy.get_param("~model_name")
    time_warp = rospy.get_param("~time_warp", 1.0)
    start_delay = rospy.get_param("~start_delay", 0.0)
    z_offset = rospy.get_param("~z_offset", 0.0)  # Added later, Sometimes model are too low

    with open(in_csv) as f:
        reader = csv.reader(f, dialect='excel')
        trajectory_data = []
        next(reader)  # skip column labels
        for row in reader:
            trajectory_data.append([float(s) for s in row])

    rospy.wait_for_service("/gazebo/set_model_state")
    set_model_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)

    success = False
    while not success:  # it's that easy
        res = set_model_state(make_request(trajectory_data[0], trajectory_data[1], model_name, 1e-30, z_offset))
        success = res.success
        time.sleep(0.1)

    if start_delay > 0:
        print("opponent trajectory controller is waiting {} seconds...".format(start_delay))

    trajectory_data = sorted(trajectory_data)
    first_t = trajectory_data[0][0]
    last_t = trajectory_data[-1][0]
    start_t = rospy.get_time()
    for loop in itertools.count():
        for traj_point, next_traj_point in zip(trajectory_data[:-1], trajectory_data[1:]):
            t0 = traj_point[0]
            next_update_time = start_delay + t0 + start_t - first_t + ((last_t - first_t) * loop) # could prob just make this longer over the course of a minute
            while (rospy.get_time() - start_t) * time_warp < next_update_time:
                time.sleep(0.001)

            set_model_state(make_request(traj_point, next_traj_point, model_name, time_warp, z_offset))


if __name__ == '__main__':
    main()
