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

size = 500
anchor = (size / 2, size - 50)
LANE_WIDTH = 1


def getEquestions(img, a1, degree=1, color=(0, 255, 0), inputVisual=False):
    min_a1_x, max_a1_x = min(a1[:, 0]), max(a1[:, 0])
    new_a1_x = np.linspace(min_a1_x, max_a1_x, 200)
    a1_coefs = np.polyfit(a1[:, 0], a1[:, 1], degree)
    new_a1_y = np.polyval(a1_coefs, new_a1_x)

    img = putArrayOnImg(img, new_a1_x, new_a1_y, color)
    return img


def getEquestionsY(img, a1, degree=1, color=(0, 255, 0), inputVisual=False):
    a1 = np.fliplr(a1)
    min_a1_x, max_a1_x = min(a1[:, 0]), max(a1[:, 0])
    new_a1_x = np.linspace(min_a1_x, max_a1_x, 100)
    a1_coefs = np.polyfit(a1[:, 0], a1[:, 1], degree)
    new_a1_y = np.polyval(a1_coefs, new_a1_x)

    img = putArrayOnImg(img, new_a1_y, new_a1_x, color)

    slope = -(new_a1_x[0] - new_a1_x[-1]) / (new_a1_y[0] - new_a1_y[-1])
    angle = math.degrees(math.atan(slope))

    if angle < 0:
        angle += 90
    else:
        angle -= 90

    points = np.vstack((new_a1_y, new_a1_x)).T
    points = np.array(points, np.int32)

    return img, angle, points


def getMidlineY(img, points1, points2, poly_deg=1, n_points=100, plot=True, color=(0, 255, 0)):
    combined1 = points1[points1[:, 1].argsort()]
    combined2 = points2[points2[:, 1].argsort()]

    midx = [(combined1[i][0] + combined2[i][0]) / 2 for i in range(n_points)]
    midy = [(combined1[i][1] + combined2[i][1]) / 2 for i in range(n_points)]
    mid = np.array(np.vstack((midx, midy)).T, np.int32)

    img = putArrayOnImg(img, midx, midy, (0, 255, 0))

    point = makeAnchorBody()
    mid = np.vstack((mid, point))

    img, slope, points = getEquestionsY(img, mid, 1, (0, 0, 255))
    return img, slope


def makeAnchorBody():
    point = np.array([size / 2, size - 50])
    point = np.vstack((point, [size / 2, size - 50 + 1]))
    point = np.vstack((point, [size / 2, size - 50 - 1]))
    point = np.vstack((point + 1, [size / 2, size - 50]))
    point = np.vstack((point + 1, [size / 2, size - 50]))
    point = np.vstack((point + 1, [size / 2, size - 50 + 1]))
    for i in range(45, 100):
        point = np.vstack((point, [size / 2, size - i]))
    return point


def putArrayOnImg(img, new_a1_x, new_a1_y, color=(0, 255, 0)):
    combined = np.vstack((new_a1_x, new_a1_y)).T
    combined = np.array(combined, np.int32)
    combined = combined.reshape((-1, 1, 2))
    cv2.polylines(img, [combined], False, color, 2)
    return img


def printSlope(img, slope):
    cv2.putText(img, 'deg: % .3f' % (slope), (200, 40), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2,
                cv2.LINE_AA)
    cv2.putText(img, 'rad: % .3f' % (math.radians(slope)), (200, 80), cv2.FONT_HERSHEY_SIMPLEX, 1,
                (255, 255, 255), 2, cv2.LINE_AA)
    return img

def placeLineWithOffset(img, points, offset, color = (255, 100, 100)):
    offset *= 140
    cv2.line(img, tuple(points[0] + [offset, 0]), tuple(points[-1] + [offset, 0]), color, 2)
    return img


def callback(data):
    bridge = CvBridge()
    try:
        img = bridge.imgmsg_to_cv2(data)
    except CvBridgeError as e:
        print(e)
        return None

    t0 = time.time()

    # resize image
    r1 = time.time()
    height, width = np.size(img, 0), np.size(img, 1)
    img = cv2.resize(img, (size, size))
    img = cv2.morphologyEx(img, cv2.MORPH_CLOSE, np.ones((5, 5), np.uint8))
    ret, img = cv2.threshold(img, 40, 255, cv2.THRESH_BINARY)

    # Three channels
    im_debug = np.stack([img] * 3, axis=-1)
    cv2.circle(im_debug, anchor, 4, (255, 0, 255), -1)
    r2 = time.time()

    # Get main Right and Left
    h1 = time.time()
    rightCnt, leftCnt = [], []
    lowestLeftPnt, lowestRightPnt = 0, 0
    im2, contours, hierarchy = cv2.findContours(img, 1, 2)
    minArea = 1000
    for cnt in contours:
        bottommost = tuple(cnt[cnt[:, :, 1].argmax()][0])
        if bottommost[0] > size / 2 and bottommost[1] > lowestRightPnt and cv2.contourArea(cnt) > minArea:
            lowestRightPnt = bottommost[1]
            rightCnt = cnt
        elif bottommost[0] < size / 2 and bottommost[1] > lowestLeftPnt and cv2.contourArea(cnt) > minArea:
            lowestLeftPnt = bottommost[1]
            leftCnt = cnt

    # Get Equation
    leftSlope, rightSlope = 0, 0
    leftLinePoints, rightLinePoints = [], []
    if len(leftCnt) != 0:
        cv2.drawContours(im_debug, [leftCnt], -1, (0, 0, 255), 1)
        im_debug, leftSlope, leftLinePoints = getEquestionsY(im_debug, leftCnt[:, 0])
        im_debug = placeLineWithOffset(im_debug, leftLinePoints, LANE_WIDTH, (100, 100, 200))

    if len(rightCnt) != 0:
        cv2.drawContours(im_debug, [rightCnt], -1, 255, 1)
        im_debug, rightSlope, rightLinePoints = getEquestionsY(im_debug, rightCnt[:, 0])
        im_debug = placeLineWithOffset(im_debug, rightLinePoints, -LANE_WIDTH,(200, 100, 100))
    h2 = time.time()



    # Make Path
    steering_angle = 1
    o1 = time.time()
    if len(leftLinePoints) != 0 and len(rightLinePoints) != 0:
        im_debug, slope = getMidlineY(im_debug, leftLinePoints, rightLinePoints)
        im_debug = printSlope(im_debug, slope)
        steering_angle = slope
    elif len(leftLinePoints) != 0:
        im_debug = printSlope(im_debug, leftSlope)
        x = 1 / math.tan((math.radians(leftSlope + 90))) * (250 - 45) + 250
        cv2.line(im_debug, anchor, (int(x), 250), (0, 100, 255), 2)
        steering_angle = leftSlope

    elif len(rightLinePoints) != 0:
        im_debug = printSlope(im_debug, rightSlope)
        x = 1 / math.tan(math.radians(rightSlope - 90)) * (250 - 45) + 250
        cv2.line(im_debug, anchor, (int(x), 250), (0, 100, 255), 2)
        steering_angle = rightSlope

    else:
        im_debug = printSlope(im_debug, 0)
        cv2.line(im_debug, anchor, (250, 250), (0, 100, 255), 2)
        steering_angle = 0

    o2 = time.time()
    steering_angle = -math.radians(steering_angle)

    if steering_angle < -.4:
        steering_angle = -.4
    elif steering_angle > .4:
        steering_angle = .4


    t1 = time.time()
    # print "Part 0? Time: %f Hz: %f" % (r2 - r1, 1 / (r2 - r1))
    # print "Part 1? Time: %f Hz: %f" % (h2 - h1, 1 / (h2 - h1))
    # print "Part 2? Time: %f Hz: %f" % (o2 - o1, 1 / (o2 - o1))
    print "Time: %f Hz: %f" % (t1 - t0, 1 / (t1 - t0))
    print("-----------------------------------------------")
    msg = bridge.cv2_to_imgmsg(im_debug, encoding="bgr8")
    debug_publisher.publish(msg)

    steer_msg = Steering()
    steer_msg.angle = steering_angle
    steer_publisher.publish(steer_msg)

    speed_msg = Speed()
    speed_msg.speed = 2
    speed_publisher.publish(speed_msg)


def listener():
    global debug_publisher
    global steer_publisher
    global speed_publisher
    rospy.init_node("lane_detector")
    rospy.Subscriber("/lines/detection_img_transformed", Image, callback, buff_size=10 ** 8)
    debug_publisher = rospy.Publisher("de_image", Image, queue_size=1)

    steer_publisher = rospy.Publisher("/steering", Steering, queue_size=1)
    speed_publisher = rospy.Publisher("/speed", Speed, queue_size=1)

    rospy.spin()


if __name__ == '__main__':
    listener()
