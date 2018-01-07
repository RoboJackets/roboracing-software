import rosbag
import sys
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import random
from example_set import Example, ExampleSet, ExampleSetArchiver


if len(sys.argv) < 3:
    print "Usage: python bag_to_data.py [data folder] [bagfile1] [bagfile2] ..."
    sys.exit(1)


test_proportion = 0.3
examples_per_archive = 10**5

bridge = CvBridge()
archiver = ExampleSetArchiver(sys.argv[1])


def process_bag_file(bag_file_path):
    example_set = ExampleSet()
    bag = rosbag.Bag(bag_file_path)
    iter_obj = iter(bag.read_messages(topics=["/steering"]))
    topic2, msg, t2 = iter_obj.next()

    count = 0
    for topic1, img, t1 in bag.read_messages(topics=['/camera/image_rect']):
        try:
            count += 1
            sys.stdout.write("\rImage %d        " % count)
            sys.stdout.flush()

            cv_image = bridge.imgmsg_to_cv2(img, 'bgr8')
            # cv_image = cv_image.astype('uint8')
            
            while (t1 > t2):
                try:
                    topic2, msg, t2 = iter_obj.next()
                except StopIteration, e:
                    print str(t2) + " " + str(t1)
                    break
            
            angle = float(str(msg).split()[-1])
            ex = Example(cv_image, angle)
            
            L = example_set.test if random.random() < test_proportion else example_set.train
            L.append(ex)

            if len(example_set.train) >= examples_per_archive:
                random.shuffle(example_set.train)
                random.shuffle(example_set.test)
                archiver.store(example_set, examples_per_archive)
                example_set = ExampleSet()
        
        except CvBridgeError, e:
            print e

    print ""
    bag.close()

    random.shuffle(example_set.train)
    random.shuffle(example_set.test)
    archiver.store(example_set, examples_per_archive)


for file_path in sys.argv[2:]:
    process_bag_file(file_path)
