#!/usr/bin/env python

import rospy
import time
import numpy as np
from std_msgs.msg import Float32MultiArray
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt


labels = []
xs = []
ys = []
weights = []

global DONE

def callback(arrayMsg):
    data = arrayMsg.data
    for i in range(0, len(data), 4):
        labels.append(int(data[i]))
        xs.append(data[i+1])
        ys.append(data[i+2])
        weights.append(data[i+3])
    global DONE
    DONE = True


if __name__ == '__main__':
    rospy.init_node('planner_plotter', anonymous=True)

    rospy.Subscriber('/steer_groups', Float32MultiArray, callback)

    DONE = False

    rate = rospy.Rate(30)
    while not rospy.is_shutdown() and not DONE:
        rate.sleep()

    rospy.signal_shutdown("Stopping listening for data")

    fig = plt.figure()
    # ax = fig.add_subplot(111, projection='3d')
    # ax.plot_trisurf(xs, ys, zs, cmap=plt.cm.viridis)
    # ax.set_xlabel("Steering 1")
    # ax.set_ylabel("Steering 2")
    # ax.set_zlabel("Weight")
    # plt.show()

    colors = 'rgbk'
    shapes = 'os^'
    for i in range(len(labels)):
        shapecode = colors[labels[i] % len(colors)] \
                    + shapes[labels[i] // len(colors) % len(shapes)]
        plt.plot(xs[i], ys[i], shapecode)
    plt.show()
