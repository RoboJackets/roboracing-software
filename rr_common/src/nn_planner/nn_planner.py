#!/usr/bin/env python

import rospy
from cv_bridge import CvBridge, CvBridgeError
from keras.models import load_model
from sensor_msgs.msg import Image
from rr_platform.msg import steering as Steering
from rr_platform.msg import speed as Speed
import cv2
import numpy as np
import os, sys
from tensorflow import get_default_graph

path_to_model = ''
model = None
input_shape = None

# print "********", input_shape
tf_graph = get_default_graph()

steering_angle = 0
has_rect_message = False

def plan(image_message):
    bridge = CvBridge()
    try:
        cv_image = bridge.imgmsg_to_cv2(image_message)
    except CvBridgeError as e:
        print e

    # use bottom half only
    halfHeight = cv_image.shape[0] // 2
    cv_image = cv_image[halfHeight:, :]

    X = np.zeros((1,) + input_shape)
    X[0] = cv2.resize(cv_image, (input_shape[1], input_shape[0]))

    with tf_graph.as_default():
        Y = model.predict(X, batch_size=1)
        print '  '.join('%.2f' % y for y in Y.flat)

    i = np.argmax(Y)
    global steering_angle
    steering_angle = [-0.3, -0.1, 0, 0.1, 0.3][i]
    # steering_angle *= -1

def plan_rect(image_message):
    has_rect_message = True
    plan(image_message)

def plan_raw(image_message):
    if not has_rect_message:
        plan(image_message)


if __name__ == "__main__":
    rospy.init_node('nn_planner')

    path_to_model = os.path.join(os.path.dirname(__file__), rospy.get_param('~model_path'))
    model = load_model(path_to_model)
    input_shape = model.get_layer(index=0).get_input_shape_at(0)[1:]

    image_topic = rospy.get_param('~image_topic')
    is_rectified = rospy.get_param('~is_rectified')

    publisher = rospy.Publisher('/steering', Steering, queue_size=1)
    speed_publisher = rospy.Publisher('/speed', Speed, queue_size=1)

    print("Subscribing to " + image_topic)

    rospy.Subscriber(image_topic, Image, plan_rect if is_rectified else plan_raw, buff_size=10**8)

    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        msg = Steering()
        speed_msg = Speed()
        msg.angle = steering_angle
        speed_msg.speed = 0.5 - 0.1*abs(steering_angle)
        publisher.publish(msg)
        speed_publisher.publish(speed_msg)
        rate.sleep()
