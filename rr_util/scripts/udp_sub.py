#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

import socket


buffersize = 1024
port = 8888

# todo make these the right ips
steeringAddressPort = ("192.168.20.3", port)
speedAddressPort = ("192.168.20.4", port)
myAddressPort = ("192.168.20.2", port)

class UDPSubscriber(Node):

    def __init__(self):
        super().__init__('udp_subscriber')
        self.socket_server = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.socket_server.bind(myAddressPort)
        self.speed_pub_ = self.create_publisher(Twist, '/actual_speed', 10)
        self.steer_pub_ = self.create_publisher(Twist, '/actual_ackermann', 10)

    def read_data(self):
        while not rclpy.is_shutdown():
            # receive the data
            message, address = self.socket_server.recvfrom(buffersize)
            odom_message = Twist()
            if address == steeringAddressPort[0]:
                odom_message.angular.z = float(message[2:])
                self.steer_pub_.publish(odom_message)
            elif address == speedAddressPort[0]:
                odom_message.linear.x = float(message[2:])
                self.speed_pub_.publish(odom_message)
    
def main(args=None):
    rclpy.init(args=args)

    udp_subscriber = UDPSubscriber()

    rclpy.spin(udp_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    udp_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()