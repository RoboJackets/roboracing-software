#!/usr/bin/env python

import rospy
import socket

from rr_platform.msg import steering as Steering, speed as Speed


ECHO_SERVER_ADDRESS = "192.168.2.2"
ECHO_SERVER_PORT = 7

speed = 0.0
steering = 0.0


def speed_callback(speed_msg):
    global speed
    speed = float(speed_msg.speed)

def steering_callback(steering_msg):
    global steering
    steering = float(steering_msg.angle)


rospy.init_node("bigoli_ethernet_drive_relay")
rospy.Subscriber('/speed', Speed, speed_callback)
rospy.Subscriber('/steering', Steering, steering_callback)

print "setting up socket..."
sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
print "connecting socket..."
sock.connect((ECHO_SERVER_ADDRESS, ECHO_SERVER_PORT))

rate = rospy.Rate(10)

try:
    while not rospy.is_shutdown():
        msg_out = bytes(str(speed))
        sock.sendall(msg_out)
        msg_in = str(sock.recv(1024)).split(" ")[:2]
        print msg_in
        rate.sleep()

finally:
    sock.close()
