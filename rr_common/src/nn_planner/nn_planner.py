#!/usr/bin/env python

import rospy
from cv_bridge import CvBridge, CvBridgeError
from keras.models import load_model
from sensor_msgs.msg import Image
from rr_platform.msg import steering as Steering
import cv2
import numpy as np
import os
from tensorflow import get_default_graph


path_to_model = os.path.join(os.path.dirname(__file__), 'nn_2017-12-30.h5')
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
    steering_angle = [-0.15, -0.05, 0, 0.05, 0.15][i]


if __name__ == "__main__":
    rospy.init_node('nn_planner')
    publisher = rospy.Publisher('/steering', Steering, queue_size=1)
    rospy.Subscriber('/camera/image_rect', Image, plan)
    
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        msg = Steering()
        msg.angle = steering_angle
        publisher.publish(msg)
        rate.sleep()
