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


if len(sys.argv) < 2:
    print "Error: specify a model file"
    sys.exit(-1)


path_to_model = os.path.join(os.path.dirname(__file__), sys.argv[1])
model = load_model(path_to_model)
input_shape = model.get_layer(index=0).get_input_shape_at(0)[1:]
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
    steering_angle = [-0.2, -0.05, 0, 0.05, 0.2][i]
    # steering_angle *= -1

def plan_rect(image_message):
    has_rect_message = True
    plan(image_message)

def plan_raw(image_message):
    if not has_rect_message:
        plan(image_message)


if __name__ == "__main__":
    rospy.init_node('nn_planner')
    publisher = rospy.Publisher('/steering', Steering, queue_size=1)
    speed_publisher = rospy.Publisher('/speed', Speed, queue_size=1)
    rospy.Subscriber('/camera/image_rect', Image, plan_rect)
    rospy.Subscriber('/camera/image_raw', Image, plan_raw)

    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        msg = Steering()
        speed_msg = Speed()
        msg.angle = steering_angle
        speed_msg.speed = 1.5 - 0.3*abs(steering_angle)
        publisher.publish(msg)
        speed_publisher.publish(speed_msg)
        rate.sleep()
