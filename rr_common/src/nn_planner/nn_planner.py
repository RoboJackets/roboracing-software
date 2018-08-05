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


steering_angle = 0

def plan(image_message):
    bridge = CvBridge()
    try:
        cv_image = bridge.imgmsg_to_cv2(image_message)
    except CvBridgeError as e:
        print e

    # use bottom half only
    halfHeight = cv_image.shape[0] // 2
    cv_image = cv_image[halfHeight:, :]

    X = cv2.resize(cv_image, (input_shape[1], input_shape[0]))
    X = X.astype('float32') / 255.0

    with tf_graph.as_default():
        Y = model.predict_on_batch(X[np.newaxis]).reshape(-1)

    # i = np.argmax(Y)
    global steering_angle
    steering_angle = np.dot(steer_categories, Y)

    print '  '.join('%.2f' % y for y in Y.flat), "->", round(steering_angle,2)


if __name__ == "__main__":
    tf_graph = get_default_graph()

    rospy.init_node('nn_planner')

    path_to_model = rospy.get_param('~model_path')
    model = load_model(path_to_model)
    input_shape = model.get_layer(index=0).get_input_shape_at(0)[1:]

    image_topic = rospy.get_param('~image_topic')
    speed_topic = rospy.get_param('~speed_topic')
    steer_topic = rospy.get_param('~steer_topic')
    speed_straight = rospy.get_param('~speed_straight')
    speed_turn_prop = rospy.get_param('~speed_turn_prop')

    categories_str = rospy.get_param('~steer_categories')
    half_cats = np.array([float(s) for s in categories_str.split(' ')])
    steer_categories = np.concatenate([-half_cats[::-1], [0], half_cats])
    print "[nn_planner] Steering categories:", steer_categories

    steer_publisher = rospy.Publisher(steer_topic, Steering, queue_size=1)
    speed_publisher = rospy.Publisher(speed_topic, Speed, queue_size=1)

    print "[nn_planner] Subscribing to", image_topic

    rospy.Subscriber(image_topic, Image, plan, buff_size=10**8)

    rate = rospy.Rate(30)
    while not rospy.is_shutdown():
        steer_msg = Steering()
        steer_msg.angle = steering_angle
        steer_publisher.publish(steer_msg)

        speed_msg = Speed()
        speed_msg.speed = speed_straight - speed_turn_prop*abs(steering_angle)
        speed_publisher.publish(speed_msg)

        rate.sleep()
