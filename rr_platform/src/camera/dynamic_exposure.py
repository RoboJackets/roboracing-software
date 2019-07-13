#!/usr/bin/env python
import rospy
import dynamic_reconfigure.client
import numpy as npy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

brightnessTolerance = 5#2
goalBrightness = -1
brightness = -1
shutter_speed = -1
lastShutter = 0
currentUpage = 0.0
upageAmount = 0.0001#0.005

def img_callback(data):
    global brightness, goalBrightness
    bridge = CvBridge()
    img = bridge.imgmsg_to_cv2(data)
    #get roi
    r1 = int(img.shape[0]/2)
    r2 = int(img.shape[0] * 5.0/6.0)
    roi = img[r1:r2,:] #rows, cols #we want the middle 3rd of image

    #calc brightness level
    gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
    brightness = npy.average(gray)

    if (goalBrightness < 0):
        goalBrightness = brightness
        #print("Changed goal brightness to " + str(goalBrightness))

    msg = bridge.cv2_to_imgmsg(gray, encoding="mono8")
    debug_publisher.publish(msg)


def dynamic_callback(config):
    global shutter_speed, brightness, goalBrightness, lastShutter, currentUpage, upageAmount
    rospy.loginfo("Camera shutter speed set to {shutter_speed}".format(**config))
    shutter_speed = config['shutter_speed']
    if (shutter_speed < 0):
        #first run setup
        goalBrightness = brightness #setpoint
        print("Our goal brightness is: " + str(goalBrightness))
        shutter_speed = config['shutter_speed']
        lastShutter = shutter_speed

    # if shutter speed actually changed (speed != lastShutter): reset upage
    if (shutter_speed != lastShutter): #TODO: might need to compare for possible small change
        #print("WE DONE A CHANGE!")
        lastShutter = shutter_speed
        currentUpage = 0.0

if __name__ == "__main__":
    rospy.init_node("dynamic_exposure_client")

    rospy.Subscriber("camera_center/image_color_rect", Image, img_callback, buff_size=10 ** 8)
    global debug_publisher
    debug_publisher = rospy.Publisher("dyanamic_exposure", Image, queue_size=1)

    client = dynamic_reconfigure.client.Client("/camera_center/camera_nodelet_pointgrey", timeout=30, config_callback=dynamic_callback)

    r = rospy.Rate(2)
    while not rospy.is_shutdown():
	
        if (brightness < goalBrightness and abs(goalBrightness-brightness) > brightnessTolerance):
            #up shutter_speed
            currentUpage += upageAmount
            #print("UP!! " + str(currentUpage))
            newValue = shutter_speed + currentUpage
            #print("SEND: " + str(newValue))
            client.update_configuration({"shutter_speed":newValue})

        elif (brightness > goalBrightness and abs(goalBrightness-brightness) > brightnessTolerance):
            #lower shutter_speed
            currentUpage -= upageAmount
            #print("DOWN " + str(currentUpage))
            newValue = shutter_speed + currentUpage
            client.update_configuration({"shutter_speed":newValue})
        else:
            #do nothing
            pass
	    print(brightness)
        r.sleep()
