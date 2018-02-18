#!/usr/bin/env python

import rospy
from cv_bridge import CvBridge, CvBridgeError
from keras.models import load_model
from sensor_msgs.msg import Image
from rr_platform.msg import steering as Steering
from rr_platform.msg import speed as Speed
import cv2
import numpy as np
import os
from tensorflow import get_default_graph


path_to_model = os.path.join(os.path.dirname(__file__), 'nn_2018-01-14.h5')
model = load_model(path_to_model)
tf_graph = get_default_graph()

steering_angle = 0

def plan(image_message):
    bridge = CvBridge()
    try:
        cv_image = bridge.imgmsg_to_cv2(image_message)
    except CvBridgeError as e:
        print e

    X = np.zeros((1,90,120,3))
    X[0] = cv2.resize(cv_image, (120,90))

    with tf_graph.as_default():
        Y = model.predict(X, batch_size=1)
        # print '  '.join('%.2f' % y for y in Y.flat)

    i = np.argmax(Y)
    global steering_angle
    steering_angle = [-0.25, -0.1, 0, 0.1, 0.25][i]


if __name__ == "__main__":
    rospy.init_node('nn_planner')
    publisher = rospy.Publisher('/steering', Steering, queue_size=1)
    speed_publisher = rospy.Publisher('/speed', Speed, queue_size=1)
    rospy.Subscriber('/camera/image_rect', Image, plan)
    # rospy.Subscriber('/camera/image_raw', Image, plan)

    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        msg = Steering()
        speed_msg = Speed()
        msg.angle = steering_angle
        speed_msg.speed = 2 - 3*abs(steering_angle)
        publisher.publish(msg)
        speed_publisher.publish(speed_msg)
        rate.sleep()
