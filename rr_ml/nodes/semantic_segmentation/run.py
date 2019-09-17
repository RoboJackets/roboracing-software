import time

import keras
import numpy as np
import cv2
import tensorflow as tf

import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image

from . import model_utils


last_msg = None
disable_gpu = True


def image_msg_callback(msg):
    global last_msg
    last_msg = msg


def main():
    global last_msg

    config = tf.ConfigProto()
    if disable_gpu:
        config.device_count['GPU'] = 0  # disables GPU if testing system has one
    keras.backend.set_session(tf.Session(config=config))

    rospy.init_node('segnet_labeler')

    path_to_model = rospy.get_param('~model_path')
    image_topic = rospy.get_param('~image_topic')
    detection_topic = rospy.get_param('~detection_topic')
    polling_rate = rospy.get_param("~polling_rate")

    detect_pub = rospy.Publisher(detection_topic, Image, queue_size=1)
    visualizer_pub = rospy.Publisher("~detect_vis", Image, queue_size=1)

    rospy.Subscriber(image_topic, Image, image_msg_callback, queue_size=1, buff_size=10**8)

    unet_helper = model_utils.UNetModelUtils()
    cv_bridge = CvBridge()

    model = keras.models.load_model(path_to_model)

    while not rospy.is_shutdown():
        if last_msg is None:
            time.sleep(1.0 / polling_rate)
            continue

        try:
            # CvBridge makes img read-only without copy
            cam_img = cv_bridge.imgmsg_to_cv2(last_msg, desired_encoding="bgr8").copy()
        except CvBridgeError as e:
            print e
            continue
        else:
            last_msg = None

        img = unet_helper.crop(cam_img)
        cropped_height, cam_width = img.shape[:2]

        X = unet_helper.prepare_input(img)

        t0 = time.time()
        y = model.predict_on_batch(X[np.newaxis])[0]
        t1 = time.time()
        print "neural net latency:", (t1 - t0)

        output_img = unet_helper.prediction_image_mono8(y)  # convert to mask
        output_img = cv2.resize(output_img, (cam_width, cropped_height))  # back to original half dimensions
        output_img = unet_helper.uncrop(output_img)

        n_labels, labels_img = cv2.connectedComponents(output_img, 8, cv2.CV_32S)
        for i in range(n_labels):
            component_size = np.count_nonzero(labels_img == i)
            if component_size < 500:
                output_img[labels_img == i] = 0

        out_msg = cv_bridge.cv2_to_imgmsg(output_img, encoding="mono8")
        detect_pub.publish(out_msg)

        obstacle_mask = (output_img > 0)
        highlight_color = np.array([0, 255, 0], dtype=np.uint8)
        cam_img[obstacle_mask] /= 2
        cam_img[obstacle_mask] += (highlight_color / 2)

        vis_msg = cv_bridge.cv2_to_imgmsg(cam_img, encoding="bgr8")
        visualizer_pub.publish(vis_msg)
