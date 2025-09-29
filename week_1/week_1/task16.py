#!/usr/bin/env python3
#
# Sample solution to Task 16 of Practical 2.

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from std_msgs.msg import String
from nav_msgs.msg import Odometry

class Task16Subscriber(Node):

    def __init__(self):
        super().__init__('task16_subscriber')

        self.create_subscription(
            Twist,
            'cmd_vel_1',
            self.cmd_vel_1_callback,
            10)

        self.create_subscription(
            Twist,
            'cmd_vel_2',
            self.cmd_vel_2_callback,
            10)

        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        self.time_since_cmd_vel_1 = None

    def cmd_vel_1_callback(self, msg):
        # This one is always published
        self.time_since_cmd_vel_1 = self.get_clock().now()
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing cmd_vel_1: {msg}\n')

    def cmd_vel_2_callback(self, msg):

        # We publish if either cmd_vel_1 has never been received, or if it was received more than 1.0 second ago.
        if self.time_since_cmd_vel_1 is None or (self.get_clock().now() - self.time_since_cmd_vel_1).nanoseconds > 1.0*1e9:
            self.publisher_.publish(msg)
            self.get_logger().info(f'Publishing cmd_vel_2: {msg}\n')


def main(args=None):
    rclpy.init(args=args)

    node = Task16Subscriber()

    try:
        # Spin the node so the callbacks are processed.
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()

if __name__ == '__main__':
    main()