#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose2D, PoseStamped
from std_msgs.msg import Float64MultiArray

from transforms3d.euler import euler2quat

class PlanarFKEuler(Node):
    def __init__(self):
        super().__init__('planar_fk_euler')
        
        # parameters
        self.L1 = float(self.declare_parameter('L1', 0.4).value)
        self.L2 = float(self.declare_parameter('L2', 0.3).value)

        # pubs/subs
        self.pose2d_pub = self.create_publisher(Pose2D, 'ee_pose2d', 10)
        self.stamped_pub = self.create_publisher(Float64MultiArray, 'ee_pose2d_stamped', 10)  # [stamp_sec, stamp_nanosec, x, y, phi]
        self.sub = self.create_subscription(JointState, 'joint_states', self.cb, 10)
        self.poseStamped_pub = self.create_publisher(PoseStamped, 'ee_pose_stamped', 10)

        self.idx1 = None
        self.idx2 = None

    def cb(self, msg: JointState):
        # Find indices by name once
        if self.idx1 is None or self.idx2 is None:
            try:
                self.idx1 = msg.name.index('joint1')
                self.idx2 = msg.name.index('joint2')
            except ValueError:
                self.get_logger().warn("Expected joint names 'joint1' and 'joint2' in /joint_states")
                return
                
        # print("q1: ", msg.position[self.idx1])
        # print("q2: ", msg.position[self.idx2])

        q1 = float(msg.position[self.idx1])
        q2 = float(msg.position[self.idx2])

        x = (self.L1)*math.cos(q1) + self.L2*math.cos(q1 + q2)
        y = (self.L1)*math.sin(q1) + self.L2*math.sin(q1 + q2)
        phi = q1 + q2  # yaw (Euler), radians

        # Pose2D
        p = Pose2D()
        p.x = x
        p.y = y
        p.theta = phi
        self.pose2d_pub.publish(p)

        # Stamped array (simple for logging/plots)
        arr = Float64MultiArray()
        arr.data = [float(msg.header.stamp.sec) if msg.header.stamp else 0.0,
                     float(msg.header.stamp.nanosec) if msg.header.stamp else 0.0,
                     x, y, phi]

        self.stamped_pub.publish(arr)

        pose = PoseStamped()
        pose.header.frame_id = 'base_link'
        pose.header.stamp = msg.header.stamp
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = 0.0
        q = euler2quat(0, 0, phi)  # yaw,
        pose.pose.orientation.x = q[1]
        pose.pose.orientation.y = q[2]
        pose.pose.orientation.z = q[3]
        pose.pose.orientation.w = q[0]
        self.poseStamped_pub.publish(pose)        

def main():
    rclpy.init()
    rclpy.spin(PlanarFKEuler())
    rclpy.shutdown()

if __name__ == '__main__':
    main()
