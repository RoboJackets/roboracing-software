#https://answers.ros.org/question/27713/how-to-recover-the-saved-images-in-a-bagfile-to-jpg-or-png/

import rosbag
import sys
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import itertools
import cPickle as pickle

bag = rosbag.Bag(sys.argv[1])
bridge = CvBridge()
data = list()
iter_obj = iter(bag.read_messages(topics=["/chassis_state"]))
topic2, msg, t2 = iter_obj.next()

for topic1, img, t1 in bag.read_messages(topics=['/camera/image_rect']):
    try:
        cv_image = bridge.imgmsg_to_cv2(img, 'bgr8')
        while (t1 > t2):
            topic2, msg, t2 = iter_obj.next()
        data.append((cv_image, msg))
    except CvBridgeError, e:
        print e

with open('data.pickle', 'wb') as pickle_file:
    pickle.dump(data, pickle_file)
bag.close()
