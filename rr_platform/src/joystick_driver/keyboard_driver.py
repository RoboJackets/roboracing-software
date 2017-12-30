#!/usr/bin/env python

import rospy
from rr_platform.msg import steering as Steering, speed as Speed
import pynput
import time


speed_limits = (-0.5, 1.5)
turn_limits = (-0.35, 0.35)
turn_delta = 0.05
speed_delta = 0.07
refresh_rate = 10
pressed_set = set()


def on_key_press(key):
    if 'char' in dir(key):
        pressed_set.add(key.char)
    
    elif 'esc' in dir(key) and key.esc:
        # ends key listener loop
        return False


def on_key_release(key):
    if 'char' in dir(key):
        pressed_set.discard(key.char)


if __name__ == '__main__':
    rospy.init_node('keyboard_driver')
    steering_pub = rospy.Publisher('/steering', Steering, queue_size=1)
    speed_pub = rospy.Publisher('/speed', Speed, queue_size=1)

    steering_rad = 0.0
    speed_mps = 0.0

    rate = rospy.Rate(refresh_rate)

    with pynput.keyboard.Listener(on_press=on_key_press,
            on_release=on_key_release) as kb_listener:
        while not rospy.is_shutdown():
            if 'a' in pressed_set: 
                steering_rad -= turn_delta
            if 'd' in pressed_set:
                steering_rad += turn_delta
            if 'w' in pressed_set:
                speed_mps += speed_delta
            if 's' in pressed_set:
                speed_mps -= speed_delta

            if speed_mps < speed_limits[0]:
                speed_mps = speed_limits[0]
            elif speed_mps > speed_limits[1]: 
                speed_mps = speed_limits[1]

            if steering_rad < turn_limits[0]:
                steering_rad = turn_limits[0]
            elif steering_rad > turn_limits[1]: 
                steering_rad = turn_limits[1]
            
            if not {'a','d'} & pressed_set: # {turning keys} ^ {pressed keys} is null
                steering_rad *= 0.85 # steer back to center
            if not {'w','s'} & pressed_set:
                speed_mps *= 0.95

            if abs(steering_rad) < 0.01:
                steering_rad = 0
            if abs(speed_mps) < 0.01:
                speed_mps = 0

            steering_msg = Steering()
            steering_msg.angle = steering_rad
            steering_pub.publish(steering_msg)

            speed_msg = Speed()
            speed_msg.speed = speed_mps
            speed_pub.publish(speed_msg)

            print "speed: %1.3f     steering: %1.3f" % (speed_mps, steering_rad)

            rate.sleep()
