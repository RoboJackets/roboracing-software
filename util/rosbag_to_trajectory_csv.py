#!/usr/bin/env python

"""
rosbag_to_trajectory_csv.py
Populates a CSV file with timestamps and robot poses that were logged on the /tf and /tf_static topics. Example
usage for EVGP robot:
    ./rosbag_to_trajectory_csv.py --in-bag ~/Downloads/sim_lap_tf_2019-10-03-10-38-16.bag \
            --out-csv ~/catkin_ws/src/roboracing-software/rr_gazebo/config/opponent_trajectory_copycat.csv \
            --world-frame world --robot-frame base_footprint --timestep 0.1
"""

from __future__ import print_function

import copy
import argparse
import csv

import rospy
import rosbag
import tf
import tf.transformations


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--in-bag", type=str, required=True, help="path in input rosbag file")
    parser.add_argument("--out-csv", type=str, required=True, help="path to output csv file")
    parser.add_argument("--world-frame", type=str, default="world", help="world reference frame (default: 'world')")
    parser.add_argument("--robot-frame", type=str, default="base_footprint",
                        help="moving reference frame (default: 'base_footprint')")
    parser.add_argument("--timestep", default=0.1, type=float,
                        help="time delta between output trajectory waypoints in seconds (default 0.1)")
    args = parser.parse_args()

    bag = rosbag.Bag(args.in_bag, 'r')

    unordered_tfs = []
    for topic, tf_msg, t in bag.read_messages(topics=["/tf", "/tf_static"]):
        for transform_stamped in tf_msg.transforms:
            unordered_tfs.append(transform_stamped)

            # for static transforms, make sure transformer object doesn't forget about them
            if topic == "/tf_static":
                t = bag.get_start_time()
                while t <= bag.get_end_time():
                    ts2 = copy.deepcopy(transform_stamped)
                    ts2.header.stamp = rospy.Time.from_sec(t)
                    unordered_tfs.append(ts2)
                    t += args.timestep / 2.0

    ordered_tfs = sorted(unordered_tfs, key=lambda T: T.header.stamp.to_sec())

    transformer = tf.Transformer(interpolate=True, cache_time=rospy.Duration.from_sec(args.timestep * 2.0))

    stamped_poses = []
    next_time = 0
    for T in ordered_tfs:
        transformer.setTransform(T)
        msg_time = T.header.stamp.to_sec()
        if msg_time >= next_time:
            if transformer.canTransform(args.world_frame, args.robot_frame, rospy.Time(0)):
                trans, rot = transformer.lookupTransform(args.world_frame, args.robot_frame, rospy.Time(0))
                stamped_poses.append((trans, rot, msg_time - bag.get_start_time()))

            while next_time <= msg_time:
                next_time += args.timestep

    trajectory_data = [["t", "x", "y", "z", "q0", "q1", "q2", "q3"]]
    for trans, rot, t in stamped_poses:
        trajectory_data.append([str(round(v, 5)) for v in [t] + list(trans) + list(rot)])

    with open(args.out_csv, 'w') as f:
        writer = csv.writer(f, dialect='excel')
        writer.writerows(trajectory_data)


if __name__ == '__main__':
    main()
