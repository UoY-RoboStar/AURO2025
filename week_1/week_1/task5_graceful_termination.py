#!/usr/bin/env python3
#
# Sample solutions to Task 5 of Practical 2, but with graceful handling of
# SIGINT kill signal, stoping the robot before terminating.
#

import rclpy
import signal

from rclpy.node import Node
from std_msgs.msg import String
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

        # Create a guard condition to be used to trigger the executor
        self.guard_condition = self.create_guard_condition(self.shutdown_hook)

        # Save old signal handler for SIGINT
        self.old_handler = signal.signal(signal.SIGINT, self.sigint_handler)

    def timer_callback(self):

        # Create the Twist message
        msg = Twist()
        msg.angular.z = 1.0
        
        # Publish the message
        self.publisher_.publish(msg)

        # Log the message to the console
        self.get_logger().info(f"Publishing: '{msg}'")

    def shutdown_hook(self):

        # Log the shutdown message to console
        self.get_logger().info('Shutting down node...')

        # Create a Twist message to stop the robot
        msg = Twist()
        msg.linear.x = 0.0
        msg.angular.z = 0.0

        # Publish the Twist message with 0 velocity for linear/angular components
        self.publisher_.publish(msg)

        # Log the message to the console
        self.get_logger().info(f"Stopping robot by publishing: '{msg}'")

        # rclpy.shutdown()
        #
        # This call to shutdown should be enough on its own, however it can get stuck because of the following issue:
        # https://github.com/ros2/rclpy/pull/1492
        #
        # Until that issue is fixed in ROS2 Humble, we can use the following workaround 
        # to raise a KeyboardInterrupt exception that is caught by rlcpy and in our main.
        raise KeyboardInterrupt

    # This is a custom handler for the SIGINT signal that catches Ctrl+C.
    def sigint_handler(self, signum, frame):

        # Trigger the guard condition to wake up the executor
        self.guard_condition.trigger()

        # Restore the old signal handler for SIGINT
        signal.signal(signal.SIGINT, self.old_handler)

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