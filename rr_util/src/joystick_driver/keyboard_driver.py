#!/usr/bin/env python

import rclpy
from rr_msgs.msg import steering as Steering, speed as Speed
import pynput
import time


pressed_set = set()


def on_key_press(key):
    try:
        pressed_set.add(key.char)
    except Exception as e:
        print (f'Keyboard driver error: {e}')

    if 'esc' in dir(key) and key.esc:
        # ends key listener loop
        return False


def on_key_release(key):
    if 'char' in dir(key):
        pressed_set.discard(key.char)


if __name__ == '__main__':
    rospy.init_node('keyboard_driver')
    steering_pub = rospy.Publisher('/steering', Steering, queue_size=1)
    speed_pub = rospy.Publisher('/speed', Speed, queue_size=1)

    reverse_speed = rospy.get_param("~reverse_speed")
    forward_speed = rospy.get_param("~forward_speed")
    turn_limit = rospy.get_param("~turn_limit")
    speed_delta = rospy.get_param("~speed_delta")
    turn_delta = rospy.get_param("~turn_delta")
    refresh_rate = rospy.get_param("~refresh_rate")
    speed_decay = rospy.get_param("~speed_decay")
    turn_decay = rospy.get_param("~turn_decay")

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

            speed_mps = min(max(speed_mps, -abs(reverse_speed)), forward_speed)

            steering_rad = min(max(steering_rad, -turn_limit), turn_limit)

            if not {'a','d'} & pressed_set: # {turning keys} ^ {pressed keys} is null
                steering_rad *= turn_decay # steer back to center
            if not {'w','s'} & pressed_set:
                speed_mps *= speed_decay

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

            print ("speed: %1.3f     steering: %1.3f" % (speed_mps, steering_rad))

            rate.sleep()
