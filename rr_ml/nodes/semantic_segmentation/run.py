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


def image_msg_callback(msg):
    global last_msg
    last_msg = msg


def main():
    global last_msg

    config = tf.ConfigProto()
    config.gpu_options.per_process_gpu_memory_fraction = 0.3
    keras.backend.set_session(tf.Session(config=config))

    rospy.init_node('segnet_labeler')

    path_to_model = rospy.get_param('~model_path')
    image_topic = rospy.get_param('~image_topic')
    detection_topic = rospy.get_param('~detection_topic')

    detect_pub = rospy.Publisher(detection_topic, Image, queue_size=1)
    visualizer_pub = rospy.Publisher("~detect_vis", Image, queue_size=1)

    rospy.Subscriber(image_topic, Image, image_msg_callback, queue_size=1, buff_size=10**8)

    unet_helper = model_utils.UNetModelUtils()

    cv_bridge = CvBridge()

    # limit tensorflow from using GPU
    sess = tf.Session(config=tf.ConfigProto(device_count={'GPU': 0}))

    with sess.as_default():
        model = keras.models.load_model(path_to_model)

        while not rospy.is_shutdown():
            if last_msg is None:
                time.sleep(0.01)
                continue

            try:
                cam_img = cv_bridge.imgmsg_to_cv2(last_msg).copy()  # CvBridge makes img read-only without copy
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

            # keypoints = blob_detector.detect(output_img)
            # pub_img = np.zeros_like(output_img)
            # cv2.drawKeypoints(output_img, keypoints, pub_img, color=255, flags=cv2.DRAW_MATCHES_FLAGS_DEFAULT)
            # print keypoints

            # image, contours, hierarchy = cv2.findContours(output_img, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
            # pub_img = np.zeros_like(output_img)
            # cv2.drawContours(pub_img, contours, -1, (255,), 1)

            out_msg = cv_bridge.cv2_to_imgmsg(output_img, encoding="mono8")
            detect_pub.publish(out_msg)

            obstacle_mask = (output_img > 0)
            highlight_color = np.array([0, 255, 0], dtype=np.uint8)
            cam_img[obstacle_mask] /= 2
            cam_img[obstacle_mask] += (highlight_color / 2)

            vis_msg = cv_bridge.cv2_to_imgmsg(cam_img, encoding="bgr8")
            visualizer_pub.publish(vis_msg)
