#!/usr/bin/env python

import rospy
import time
import numpy as np
from geometry_msgs.msg import Point
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt

xs = []
ys = []
zs = []

def callback(data):
    xs.append(data.x)
    ys.append(data.y)
    zs.append(data.z)

if __name__ == '__main__':
    rospy.init_node('planner_plotter', anonymous=True)

    count = 0
    rospy.Subscriber('/plan_cost', Point, callback)

    rate = rospy.Rate(30)
    while not rospy.is_shutdown() and len(xs) < 2000:
        rate.sleep()

    rospy.signal_shutdown("Stopping listening for data")

    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.plot_trisurf(xs, ys, zs, cmap=plt.cm.viridis)
    ax.set_xlabel("Steering 1")
    ax.set_ylabel("Steering 2")
    ax.set_zlabel("Weight (1/cost)")
    plt.show()

    # plt.plot(xs, zs, 'bo')
    # plt.show()
