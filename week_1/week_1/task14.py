#!/usr/bin/env python3
#
# Sample solution to Task 14 of Practical 2.
# A node that uses a proportional controller to drive a robot to a goal (x,y) position.
#
# Implementation gracefully handles SIGINT (Ctrl+C) to stop the robot before terminating,
# using a similar pattern to that used in task5_graceful_termination.py.
#

import rclpy
import math
import signal

from rclpy.node import Node

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from tf_transformations import euler_from_quaternion

class Task14Subscriber(Node):

    def __init__(self):
        super().__init__('task14_node')

        # Declare parameters used by the node, with default values
        # You can change these on the command line, e.g.:
        # ros2 run week_1 task14 --ros-args -p goal_x:=2.0 -p goal_y:=3.0
        self.declare_parameter('goal_x', 0.0)   
        self.declare_parameter('goal_y', 0.0)
        self.declare_parameter('linear_p', 0.35)
        self.declare_parameter('angular_p', 0.2)

        # Proportional co-efficients for the controller
        self.linear_p = self.get_parameter('linear_p').value
        self.angular_p = self.get_parameter('angular_p').value

        # Desired position
        self.goal_x = self.get_parameter('goal_x').value
        self.goal_y = self.get_parameter('goal_y').value

        # Create a subscription to the 'odom' topic with message type Odometry
        self.subscription = self.create_subscription(
            Odometry,
            'odom',
            self.odometry_callback,
            10)

        # Create a publisher for the 'cmd_vel' topic with message type Twist
        # Notice how we use 'cmd_vel' rather than '/cmd_vel' as the topic name here.
        #
        # Question: does it make a difference? Yes, topics with a leading '/' are fully qualified,
        # thus they could not be renamed. 
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)

        self.subscription  # prevent unused variable warning

        # Create a guard condition to be used to trigger the executor
        self.guard_condition = self.create_guard_condition(self.shutdown_hook)

        # Save old signal handler for SIGINT
        self.old_handler = signal.signal(signal.SIGINT, self.sigint_handler)

    def odometry_callback(self, odom_msg):

        # Extract the yaw from the odometry message.
        # Note that the orientation in the odometry message is given as a quaternion.
        # We use the tf_transformations library to convert the quaternion to roll, pitch, yaw.
        (roll, pitch, yaw) = euler_from_quaternion([
            odom_msg.pose.pose.orientation.x,
            odom_msg.pose.pose.orientation.y,
            odom_msg.pose.pose.orientation.z,
            odom_msg.pose.pose.orientation.w])

        v_x = (self.goal_x - odom_msg.pose.pose.position.x)
        v_y = (self.goal_y - odom_msg.pose.pose.position.y)

        # We rotate the vector pointing from the robot to the goal
        # into the robot's frame of reference using the yaw as the
        # angle between the frames.
        (des_x, des_y) = rotate_vector_2d(v_x, v_y, -yaw)

        # The magnitude of the X component of the vector in this frame represents the distance to the goal, so the
        # and so the difference across that axis between the robot and goal_x.
        delta_vec = des_x

        # We use the angle between the vector (v_x, v_y) and the origin to compute the desired yaw,
        # which we then use to compute the error (delta_yaw) in yaw. Note that we use the angle_diff function
        # to ensure that we get the smallest angle difference (i.e., we can turn either clockwise or counter-clockwise).
        delta_yaw = angle_diff(math.atan2(v_y, v_x), yaw)

        twist_msg = Twist()

        # To prevent oscillations, we stop moving if we are close enough to the goal.
        if abs(delta_vec) < 0.05:
            twist_msg.linear.x = 0.0
        else:
            # Here, linear_p is a proportional constant.
            twist_msg.linear.x = self.linear_p*delta_vec

        if abs(delta_yaw) < 0.5:
            twist_msg.angular.z = 0.0
        else:
            # Here, angular_p is a proportional constant.
            twist_msg.angular.z = self.angular_p*delta_yaw

        # So the above is in fact a form of proportional control.
        # More advanced control strategies (e.g., PID control) are possible.

        self.publisher_.publish(twist_msg)
        self.get_logger().info(f'Position: ({odom_msg.pose.pose.position.x:.2f},{odom_msg.pose.pose.position.y:.2f},{yaw:.2f}); deltas:({delta_vec:.2f},{delta_yaw:.2f})')
        self.get_logger().info(f'Publishing: (linear: {twist_msg.linear.x}, angular: {twist_msg.angular.z})')

    def shutdown_hook(self):

        # Log the shutdown message to console
        self.get_logger().info('Shutting down node...')

        # Create a Twist message to stop the robot
        twist_msg = Twist()
        twist_msg.linear.x = 0.0
        twist_msg.angular.z = 0.0

        # Publish the Twist message with 0 velocity for linear/angular components
        self.publisher_.publish(twist_msg)

        # Log the message to the console
        self.get_logger().info(f'Publishing: (linear: {twist_msg.linear.x}, angular: {twist_msg.angular.z})')

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

# The following defines a 2D rotation of a vector (x, y) by theta radians CCW.
def rotate_vector_2d(x, y, theta):
    """Rotate (x, y) CCW by theta radians."""
    x_new = x * math.cos(theta) - y * math.sin(theta)
    y_new = x * math.sin(theta) + y * math.cos(theta)
    return x_new, y_new

def angle_diff(theta1: float, theta2: float) -> float:
    """Return smallest absolute angle difference in radians."""
    diff = (theta1 - theta2 + math.pi) % (2 * math.pi) - math.pi
    return diff

def main(args=None):
    rclpy.init(args=args)

    node = Task14Subscriber()

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