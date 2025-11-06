#
# Copyright (c) 2025 University of York and others
#
# This program and the accompanying materials are made available under the
# terms of the Eclipse Public License 2.0 which is available at
# http://www.eclipse.org/legal/epl-2.0.
# 
# SPDX-License-Identifier: EPL-2.0
#
# Contributors:
#   * Alan Millard - initial contributor
#   * Pedro Ribeiro - revised implementation
#

import sys
import argparse
from collections import deque

import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from tf2_ros import StaticTransformBroadcaster
import tf_transformations

from geometry_msgs.msg import PoseStamped, TransformStamped
from nav_msgs.msg import Odometry, Path

BUFFER_SIZE = 1000

class PathPublisher(Node):

    def __init__(self):
        super().__init__('path_publisher')

        self.declare_parameter('x_pose', 0.0)
        self.declare_parameter('y_pose', 0.0)
        self.declare_parameter('yaw', 0.0)
        self.declare_parameter('odometry_source', 'WORLD')

        self.odometry_source = self.get_parameter('odometry_source').value
        self.initial_x = self.get_parameter('x_pose').value
        self.initial_y = self.get_parameter('y_pose').value
        self.initial_yaw = self.get_parameter('yaw').value

        self.odom_pose_history = deque(maxlen = BUFFER_SIZE)
        self.true_pose_history = deque(maxlen = BUFFER_SIZE)

        # First we need to publish a transform from world -> odom,
        # so that RViz will know where to look for world frame.
        #
        # This is not automatically published by Gazebo.
        self.static_broadcaster = StaticTransformBroadcaster(self)

        x = 0.0
        y = 0.0
        yaw = 0.0
        
        if self.odometry_source == "ENCODER":
            x = self.initial_x
            y = self.initial_y
            yaw = self.initial_yaw

        # Convert yaw to quaternion (roll=pitch=0)
        quat = tf_transformations.quaternion_from_euler(0, 0, yaw)

        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = "world"
        t.child_frame_id = "odom"
        t.transform.translation.x = x
        t.transform.translation.y = y
        t.transform.translation.z = 0.0
        t.transform.rotation.x = quat[0]
        t.transform.rotation.y = quat[1]
        t.transform.rotation.z = quat[2]
        t.transform.rotation.w = quat[3]

        # Send transform once — static transforms do not change
        self.static_broadcaster.sendTransform(t)
        self.get_logger().info("Published static transform world to odom")

        self.odom_subscriber = self.create_subscription(
            Odometry,
            'odom',
            self.odom_callback,
            10)
        
        self.true_pose_subscriber = self.create_subscription(
            Odometry,
            'world',
            self.true_pose_callback,
            10)
        
        self.odom_path_publisher = self.create_publisher(Path, 'odom_path', 10)
        self.true_path_publisher = self.create_publisher(Path, 'true_path', 10)

    
    def odom_callback(self, msg):
        
        pose = msg.pose.pose

        # pose.position.x += self.initial_x
        # pose.position.y += self.initial_y

        pose_stamped = PoseStamped()
        pose_stamped.header.stamp = self.get_clock().now().to_msg()
        pose_stamped.pose = pose

        self.odom_pose_history.append(pose_stamped)

        path = Path()
        path.header.frame_id = 'odom'
        path.header.stamp = self.get_clock().now().to_msg()
        path.poses = self.odom_pose_history

        self.odom_path_publisher.publish(path)


    def true_pose_callback(self, msg):
        pose = msg.pose.pose

        pose_stamped = PoseStamped()
        pose_stamped.header.stamp = self.get_clock().now().to_msg()
        pose_stamped.pose = pose

        self.true_pose_history.append(pose_stamped)

        path = Path()
        path.header.frame_id = 'world'
        path.header.stamp = self.get_clock().now().to_msg()
        path.poses = self.true_pose_history

        self.true_path_publisher.publish(path)


def main(args=None):

    rclpy.init(args=args)

    node = PathPublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except ExternalShutdownException:
        sys.exit(1)
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()