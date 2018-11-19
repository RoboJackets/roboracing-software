import rosbag
import rospy
import os
import sys
# from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge, CvBridgeError
import random
from example_set import Example, ExampleSet
from params import input_shape


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

    test_proportion = 0.05

    examples = ExampleSet()
    bridge = CvBridge()
    bag = rosbag.Bag(bag_file_path)

    topics = bag.get_type_and_topic_info()[1].keys()
    if '/camera/image_color_rect_flipped' in topics:
        camera_topic = '/camera/image_color_rect_flipped'
    elif '/camera/image_rect' in topics:
        camera_topic = '/camera/image_rect'
    elif '/camera/image_raw' in topics:
        camera_topic = '/camera/image_raw'
    else:
        print "ERROR: camera topic not recognized"
        sys.exit(2)

    if "/chassis_state" in topics:
        steering_topic = '/chassis_state'
        steering_msg_parse_idx = -3
    elif "/steering" in topics:
        steering_topic = '/steering'
        steering_msg_parse_idx = -1
    else:
        print "ERROR: steering topic not recognized"
        sys.exit(2)

    iter_obj = iter(bag.read_messages(topics=[steering_topic]))
    topic2, msg, t2 = iter_obj.next()

    for topic1, img, t1 in bag.read_messages(topics=[camera_topic]):
        try:
            cv_image = bridge.imgmsg_to_cv2(img, 'bgr8')
        except CvBridgeError as e:
            print e
            continue

        # use lower half of picture
        halfHeight = cv_image.shape[0] // 2
        cv_image = cv_image[halfHeight:, :]

        # resize
        cv_image = cv2.resize(cv_image, (input_shape[1], input_shape[0]))

        while (t1 >= t2):
            try:
                topic2, msg, t2 = iter_obj.next()
            except StopIteration:
                print "caught StopIteration for steering message iterator"
                break

        angle = float(str(msg).split()[steering_msg_parse_idx])
        ex = Example(cv_image, angle)

        if len(examples.train) >= examples_per_file:
            save_next_filename(examples, output_dir)
            examples = ExampleSet()

        L = examples.test if random.random() < test_proportion else examples.train
        L.append(ex)

    bag.close()
    save_next_filename(examples, output_dir)


def main():
    rospy.init_node("convert_end_to_end")

    output_dir = rospy.get_param("~output_dir", None)
    if not os.path.exists(output_dir):
        os.makedirs(output_dir)

    bag_file_paths = []
    for i in range(50):
        f = rospy.get_param("~bag_file_%d" % i, None)
        if f not in (None, ""):
            bag_file_paths.append(f)

    for file_path in bag_file_paths:
        convert_bag_file(file_path, output_dir, 256)
