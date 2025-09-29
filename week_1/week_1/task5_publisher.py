#!/usr/bin/env python3
#
# Sample solutions to Task 5 of Practical 2.
#

import rclpy
import signal

from rclpy.node import Node
from geometry_msgs.msg import Twist

class Task5Publisher(Node):

    def __init__(self):

        # Initialise teh node with the name 'task5_publisher'
        super().__init__('task5_publisher')

        # Create a publisher for the 'cmd_vel' topic with message type Twist
        # Notice how we use 'cmd_vel' rather than '/cmd_vel' as the topic name.
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)

        # Period for the timer in seconds
        timer_period = 1  # seconds

        # Create a timer to call the timer_callback function every 1 seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):

        # Create the Twist message
        msg = Twist()
        msg.angular.z = 1.0
        
        # Publish the message
        self.publisher_.publish(msg)

        # Log the message to the console
        self.get_logger().info(f"Publishing: '{msg}'")

def main(args=None):

    # Pass command line arguments to the rclpy library
    rclpy.init(args=args)

    # Create the node
    node = Task5Publisher()

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