import os
import random

import rospy
import rosbag
from cv_bridge import CvBridge
import cv2

from . import util


def main():
    rospy.init_node("bag_image_extractor")

    image_dir = rospy.get_param("~image_dir")
    image_format = rospy.get_param("~image_format")
    bag_file = rospy.get_param("~bag_file")
    image_topic = rospy.get_param("~image_topic")
    n_images = rospy.get_param("~num_images")

    if not image_format.startswith("."):
        image_format = "." + image_format

    assert image_format in util.acceptable_image_formats

    print "using image dir:", image_dir

    if not os.path.exists(image_dir):
        os.makedirs(image_dir)
        print "made image dir because it did not exist"

    cv_bridge = CvBridge()

    bag = rosbag.Bag(bag_file)

    indices = range(bag.get_message_count(topic_filters=[image_topic]))
    random.shuffle(indices)

    if n_images <= 0:
        n_images = len(indices)

    save_indices = set(indices[:n_images])

    for i, (topic, msg, t) in enumerate(bag.read_messages(topics=[image_topic])):
        if i in save_indices:
            try:
                image = cv_bridge.imgmsg_to_cv2(msg, "bgr8")
                fname = os.path.join(image_dir, str(t) + image_format)
                cv2.imwrite(fname, image)
            except Exception as e:
                print "Failed during extraction:", e
