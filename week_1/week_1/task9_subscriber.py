#!/usr/bin/env python3
#
# Sample solutions to Task 9 of Practical 2.
#

import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from nav_msgs.msg import Odometry

class Task9Subscriber(Node):

    def __init__(self):
        super().__init__('task9_subscriber')

        self.subscription = self.create_subscription(
            Odometry,
            'odom',
            self.odometry_callback,
            10)

        self.subscription  # prevent unused variable warning

    def odometry_callback(self, msg):
        self.get_logger().info(f'Position: {msg.pose.pose.position}\n')


def main(args=None):
    rclpy.init(args=args)

    node = Task9Subscriber()

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