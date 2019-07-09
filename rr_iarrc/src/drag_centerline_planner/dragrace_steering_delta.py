import numpy as np
import pandas as pd
import cv2
import os
import glob
import matplotlib.pyplot as plt
import pickle
import math
from sklearn.metrics import r2_score
from sklearn.metrics import mean_squared_error
import imutils
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import time
import warnings
from rr_platform.msg import steering as Steering
from rr_platform.msg import speed as Speed

warnings.simplefilter('ignore', np.RankWarning)

min_contour_area = 400

right_line_present = False
left_line_present = False
size = 500
anchor = (size / 2, size - 50)
prev_angle = 0


def draw_connected_points(img, points, color=(0, 255, 0)):
    combined = points.reshape((-1, 1, 2))
    cv2.polylines(img, [combined], False, color, 2)
    return img


def prepare_img(img):
    img = cv2.resize(img, (size, size))
    img = cv2.morphologyEx(img, cv2.MORPH_CLOSE, np.ones((5, 5), np.uint8))
    ret, img = cv2.threshold(img, 40, 255, cv2.THRESH_BINARY)
    img_debug = np.stack([img] * 3, axis=-1)
    return img, img_debug


def get_linear_endpoints(points):
    row_end_pnts = [min(points[:, 1]), max(points[:, 1])]
    coefs = np.polyfit(points[:, 1], points[:, 0], 1)
    new_cols = np.polyval(coefs, row_end_pnts)

    points = np.vstack((new_cols, row_end_pnts)).T
    points = np.array(points, np.int32)
    return points


def calc_midline(img, left_endpoints, right_endpoints):
    left_endpoints = left_endpoints[left_endpoints[:, 1].argsort()]
    right_endpoints = right_endpoints[right_endpoints[:, 1].argsort()]

    midx = [(left_endpoints[i][0] + right_endpoints[i][0]) / 2 for i in range(2)]
    midy = [(left_endpoints[i][1] + right_endpoints[i][1]) / 2 for i in range(2)]
    mid_endpoints = np.array(np.vstack((midx, midy)).T, np.int32)

    img = draw_connected_points(img, mid_endpoints, (0, 50, 0))
    return img, mid_endpoints


def convert_endpoints_to_angle(endpoints, midline=False):
    if midline:
        if endpoints[0][0] == anchor[0]:
            return 0
        slope = -(endpoints[0][1] - anchor[1]) / float(endpoints[0][0] - anchor[0])
    else:
        if endpoints[0][0] == endpoints[-1][0]:
            return 0
        slope = -(endpoints[0][1] - endpoints[-1][1]) / float(endpoints[0][0] - endpoints[-1][0])
        print(slope, (endpoints[0][0] - endpoints[-1][0]))
    angle = math.atan(slope)
    angle = math.pi/2 - angle if angle > 0 else -math.pi / 2 - angle
    return angle


def draw_angle_direction(img, angle):
    # x_1 = 1/m * dy + x_0
    height = 100
    angle = math.pi/2 - angle if angle > 0 else -math.pi / 2 - angle
    x = 1 / math.tan(angle) * (anchor[1] - height) + anchor[0] if angle != 0 else anchor[0]

    cv2.line(img, anchor, (int(x), height), (0, 100, 255), 2)
    return img


def process_steering_angle(steering_angle):
    steering_angle = max(-.3, min(.3, steering_angle))
    # steering_angle *= 1
    global prev_angle
    prev_angle -= max(-.05, min(.05, prev_angle - steering_angle))
    return prev_angle


def print_slope(img, slope):
    font, color, line_type = cv2.FONT_HERSHEY_SIMPLEX, (255, 255, 255), cv2.LINE_AA
    cv2.putText(img, 'deg: % .3f' % math.degrees(slope), (200, 40), font, 1, color, 2, line_type)
    cv2.putText(img, 'rad: % .3f' % slope, (200, 80), font, 1, color, 2, line_type)
    return img


def get_side_contours(img):
    right_cnt, left_cnt = None, None
    max_left_row, max_right_row = 0, 0
    im, contours, hierarchy = cv2.findContours(img, 1, 2)
    for cnt in contours:
        if cv2.contourArea(cnt) > min_contour_area:
            cnt_bottom_most_pnt = tuple(cnt[cnt[:, :, 1].argmax()][0])
            if size / 2 < cnt_bottom_most_pnt[0] and cnt_bottom_most_pnt[1] > max_right_row:
                max_right_row = cnt_bottom_most_pnt[1]
                right_cnt = cnt
            elif cnt_bottom_most_pnt[0] < size / 2 and cnt_bottom_most_pnt[1] > max_left_row:
                max_left_row = cnt_bottom_most_pnt[1]
                left_cnt = cnt

    global left_line_present, right_line_present
    left_line_present = left_cnt is not None
    right_line_present = right_cnt is not None

    return left_cnt, right_cnt


def get_contours_linear_endpoints(left_cnt, right_cnt, img_debug):
    left_endpoints, right_endpoints = None, None
    if left_line_present:
        cv2.drawContours(img_debug, [left_cnt], -1, (0, 0, 100), -1)
        left_endpoints = get_linear_endpoints(left_cnt[:, 0])
        img_debug = draw_connected_points(img_debug, left_endpoints, (100, 0, 200))

    if right_line_present:
        cv2.drawContours(img_debug, [right_cnt], -1, (200, 100), -1)
        right_endpoints = get_linear_endpoints(right_cnt[:, 0])
        img_debug = draw_connected_points(img_debug, right_endpoints, (200))
    cv2.circle(img_debug, anchor, 4, (255, 0, 255), -1)
    return left_endpoints, right_endpoints, img_debug


def determine_steering_angle(left_endpoints, right_endpoints, img_debug):
    if left_line_present and right_line_present:
        img_debug, mid_endpoints = calc_midline(img_debug, left_endpoints, right_endpoints)
        angle = convert_endpoints_to_angle(mid_endpoints, midline=True)
    elif left_line_present:
        angle = convert_endpoints_to_angle(left_endpoints)
    elif right_line_present:
        angle = convert_endpoints_to_angle(right_endpoints)
    else:
        angle = 0

    angle = process_steering_angle(angle)
    img_debug = print_slope(img_debug, angle)
    img_debug = draw_angle_direction(img_debug, angle)
    return img_debug, angle


def callback(data):
    bridge = CvBridge()
    img = bridge.imgmsg_to_cv2(data)

    # Prepare Image (resize, dilate, threshold)
    img, img_debug = prepare_img(img)

    # Get left and right contours based on image position
    left_cnt, right_cnt = get_side_contours(img)

    # Get linear end points of side contours
    left_endpoints, right_endpoints, img_debug = get_contours_linear_endpoints(left_cnt, right_cnt, img_debug)

    # Determine path
    img_debug, steering_angle = determine_steering_angle(left_endpoints, right_endpoints, img_debug)

    # Publish Image, Steering, Speed
    print("Steering_angle: ", steering_angle)
    msg = bridge.cv2_to_imgmsg(img_debug, encoding="bgr8")
    debug_publisher.publish(msg)

    steer_msg = Steering()
    steer_msg.angle = steering_angle
    steer_publisher.publish(steer_msg)

    speed_msg = Speed()
    speed_msg.speed = 1
    speed_publisher.publish(speed_msg)


def listener():
    global debug_publisher, steer_publisher, speed_publisher

    rospy.init_node("lane_detector")
    rospy.Subscriber("camera_center/image_color_rect/lines/detection_img_transformed", Image, callback, buff_size=10 ** 8)
    debug_publisher = rospy.Publisher("dragrace/steering_debug_image", Image, queue_size=1)

    steer_publisher = rospy.Publisher("/plan/steering", Steering, queue_size=1)
    speed_publisher = rospy.Publisher("/plan/speed", Speed, queue_size=1)

    rospy.spin()


if __name__ == '__main__':
    listener()
