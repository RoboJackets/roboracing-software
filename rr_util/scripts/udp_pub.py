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

class UDPPublisher(Node):

    def __init__(self):
        super().__init__('udp_publisher')
        self.socket_client = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.plan_speed_sub_ = self.create_subscription(Twist, '/tricycle_controller/cmd_vel', self.speed_callback, 10)
        self.plan_steer_sub_ = self.create_subscription(Twist, '/tricycle_controller/cmd_ackermann', self.steering_callback, 10)
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0
        self.speed = 0
        self.steer = 0

    def timer_callback(self):
        speed_msg = "v={:.2f}".format(self.speed)
        steer_msg = "A={:.2f}".format(self.steer)   
        self.socket_client.sendto(speed_msg.encode(), speedAddressPort)
        self.socket_client.sendto(steer_msg.encode(), steeringAddressPort)

    def speed_callback(self, msg):
        self.speed = msg.linear.x

    def steering_callback(self, msg):
        self.steer = msg.angular.z


def main(args=None):
    rclpy.init(args=args)

    udp_publisher = UDPPublisher()

    rclpy.spin(udp_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    udp_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()