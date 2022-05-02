#!/usr/bin/env python
import rospy
import socket
from rr_msgs.msg import speed
from rr_msgs.msg import steering
from time import sleep
  
speed_val = 0.0
steering_val = 0.0
buffersize = 1024
port = 8888

# todo make these the right ips
steeringAddressPort = ("192.168.20.3", port)
speedAddressPort = ("192.168.20.4", port)
myAddressPort = ("192.168.20.2", port)

  
def speed_callback(data: speed):
    global speed_val
    speed_val = data.speed
    # print the actual message in its raw format
    rospy.loginfo("Got speed: %s", data.speed)
  
def steering_callback(data: steering):
    global steering_val
    steering_val = data.angle
    # print the actual message in its raw format
    rospy.loginfo("Got steering: %s", data.angle)
  
def main():
    # subscribers
    rospy.init_node('udp_pub', anonymous=True)
    rospy.Subscriber("/speed", speed, speed_callback)
    rospy.Subscriber("/steering", steering, steering_callback)
      
    # create the UDP cient
    socket_client = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    while not rospy.is_shutdown():
        # send the data
        speed_msg = "v={:.2f}".format(speed_val)
        steer_msg = "A={:.2f}".format(steering_val)

        socket_client.sendto(speed_msg.encode(), speedAddressPort)
        socket_client.sendto(steer_msg.encode(), steeringAddressPort)

        rospy.loginfo("Sent speed: %s", speed_msg)
        rospy.loginfo("Sent steering: %s", steer_msg)
        sleep(0.1) # send data at 10Hz

  
if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass