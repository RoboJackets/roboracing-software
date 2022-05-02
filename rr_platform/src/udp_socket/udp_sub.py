#!/usr/bin/env python
import rospy
import socket
from udp_pub import buffersize, myAddressPort

def main():
    # todo add some publishers here for the data stuff
    rospy.init_node('udp_sub', anonymous=True)

    # create UDP server
    socket_server = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    socket_server.bind(myAddressPort)

    while not rospy.is_shutdown():
        # receive the data
        message, address = socket_server.recvfrom(buffersize)
        rospy.loginfo("Received message: %s from %s", message.decode(), address)

        # todo decode and publish data here

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass