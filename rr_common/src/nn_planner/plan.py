#!/usr/bin/env python

import rospy
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import Float32
from keras.models import load_model
from sensor_msgs.msg import Image

model = load_model('model.h5')
publisher = None

def plan(image_message):
    try:
        cv_image = bridge.imgmsg_to_cv2(image_message)
    except CvBridgeError as e:
        print (e)
    output = model.predict(cv_image)
    publisher.publish(output)


if __name__ == "__main__":
    rospy.init_node('planner')
    publisher = rospy.Publisher('/steer', Float32, queue_size=10)
    rospy.Subscriber('/image_rect', Image, plan)
    rospy.spin()
