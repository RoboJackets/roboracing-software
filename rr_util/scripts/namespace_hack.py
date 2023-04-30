#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class Namespaces(Node):

    def __init__(self):
        super().__init__('udp_publisher')
        self.speed_pub_ = self.create_publisher(Twist, '/tricycle_controller/cmd_vel', 10)
        self.plan_speed_sub_ = self.create_subscription(Twist, '/cmd_vel', self.speed_callback, 10)
        self.previous_z = 0
    def speed_callback(self, msg):
        if abs(msg.angular.z) < 0.1:
            msg.linear.x = 5.0
            msg.angular.z = 0.0
        else:
            msg.linear.x = 2.5
            msg.angular.z = (msg.angular.z + self.previous_z) / 2
        self.previous_z = msg.angular.z
        self.speed_pub_.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    names = Namespaces()

    rclpy.spin(names)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    names.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()