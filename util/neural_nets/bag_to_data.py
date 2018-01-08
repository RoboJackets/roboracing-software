import rosbag
import os, sys
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import random
from example_set import Example, ExampleSet

def save_next_filename(examples, output_dir):
    fname = lambda i: "data%d.pkl.lz4" % i
    random.shuffle(examples.train)
    random.shuffle(examples.test)
    for i in range(len(os.listdir(output_dir)) + 1):
        if fname(i) not in os.listdir(output_dir):
            fpath = os.path.join(output_dir, fname(i))
            print "saving %s..." % fpath
            examples.save(fpath)
            break

def convert_bag_file(bag_file_path, output_dir, examples_per_file):
    print "Processing bag file:", bag_file_path

    test_proportion = 0.3

    examples = ExampleSet()
    bridge = CvBridge()
    bag = rosbag.Bag(bag_file_path)
    iter_obj = iter(bag.read_messages(topics=["/steering"]))
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
            
            angle = float(str(msg).split()[-1])
            ex = Example(cv_image, angle)

            if len(examples.train) >= examples_per_file:
                save_next_filename(examples, output_dir)
                examples = ExampleSet()

            L = examples.test if random.random() < test_proportion else examples.train
            L.append(ex)

        except CvBridgeError, e:
            print e

    bag.close()
    save_next_filename(examples, output_dir)



if __name__ == '__main__':

    if len(sys.argv) < 3:
        print "Usage: python bag_to_data.py [output directory] [bagfile1] [bagfile2] ..."
        sys.exit(1)

    if not os.path.exists(sys.argv[1]):
        os.makedirs(sys.argv[1])

    for file_path in sys.argv[2:]:
        convert_bag_file(file_path, sys.argv[1], 1024)
