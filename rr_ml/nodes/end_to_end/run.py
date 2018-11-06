import rospy
from cv_bridge import CvBridge, CvBridgeError
from keras.models import load_model
from sensor_msgs.msg import Image
from rr_platform.msg import steering as Steering
from rr_platform.msg import speed as Speed
import cv2
import numpy as np
from tensorflow import get_default_graph
from params import input_shape, expand_categories


last_msg = None


def image_msg_callback(msg):
    global last_msg
    last_msg = msg


def msg_to_input(msg, input_shape):
    bridge = CvBridge()
    try:
        image = bridge.imgmsg_to_cv2(msg)
    except CvBridgeError as e:
        print e
        return None

    # use bottom half only
    half_height = image.shape[0] // 2
    cv_image = image[half_height:, :]

    X = cv2.resize(cv_image, (input_shape[1], input_shape[0]))
    X = X.astype('float32') / 255.0

    return X


def main():
    rospy.init_node('nn_planner')

    path_to_model = rospy.get_param('~model_path')
    model = load_model(path_to_model)

    image_topic = rospy.get_param('~image_topic')
    speed_topic = rospy.get_param('~speed_topic')
    steer_topic = rospy.get_param('~steer_topic')
    speed_straight = rospy.get_param('~speed_straight')
    speed_turn_prop = rospy.get_param('~speed_turn_prop')

    categories_str = rospy.get_param('~steer_categories')
    half_cats = [float(s) for s in categories_str.split(' ')]
    steer_categories = np.array(expand_categories(half_cats))
    print "[nn_planner] Steering categories:", steer_categories

    steer_publisher = rospy.Publisher(steer_topic, Steering, queue_size=1)
    speed_publisher = rospy.Publisher(speed_topic, Speed, queue_size=1)

    print "[nn_planner] Subscribing to", image_topic

    rospy.Subscriber(image_topic, Image, image_msg_callback, buff_size=10**8)

    rate = rospy.Rate(10)

    tf_graph = get_default_graph()
    with tf_graph.as_default():
        while not rospy.is_shutdown():
            rate.sleep()

            if last_msg is None:
                continue

            X = msg_to_input(last_msg, input_shape)
            if X is None:
                continue

            Y = model.predict_on_batch(X[np.newaxis]).reshape(-1)

            steering_angle = float(np.dot(steer_categories, Y))
            print '  '.join('%.2f' % y for y in Y.flat), "->", round(steering_angle, 2)

            steer_msg = Steering()
            steer_msg.angle = steering_angle
            steer_publisher.publish(steer_msg)

            speed_msg = Speed()
            speed_msg.speed = speed_straight - speed_turn_prop*abs(steering_angle)
            speed_publisher.publish(speed_msg)
