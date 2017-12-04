import rosbag
import sys
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np

bag = rosbag.Bag(sys.argv[1])
bridge = CvBridge()
data = list()
iter_obj = iter(bag.read_messages(topics=["/plan/steering"]))
topic2, msg, t2 = iter_obj.next()

for topic1, img, t1 in bag.read_messages(topics=['/camera/image_rect']):
    try:
        cv_image = bridge.imgmsg_to_cv2(img, 'bgr8')
        while (t1 > t2):
            try:
                topic2, msg, t2 = iter_obj.next()
            except StopIteration, e:
                print str(t2) + " " + str(t1)
                break
        data.append((cv_image, float(str(msg).split()[-1])))
    except CvBridgeError, e:
        print e

np.save('data.npy', data)
bag.close()
