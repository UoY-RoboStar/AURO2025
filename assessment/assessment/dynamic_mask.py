#!/usr/bin/env python3
#
# Copyright (c) 2025 University of York and others
#
# Dynamic filter for LiDAR scan data
# 
# Contributors:
#   * Pedro Ribeiro - initial implementation

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import math

class DynamicMask(Node):

    def __init__(self):
        super().__init__('dynamic_mask')
        self.declare_parameter('ignore_sector_start', 145)  # start of behind sector
        self.declare_parameter('ignore_sector_end', 206)    # end of behind sector
        self.declare_parameter('mask_enabled', False)
        self.sub = self.create_subscription(LaserScan, 'scan', self.callback, 10)
        self.pub = self.create_publisher(LaserScan, 'scan_filtered', 10)

    def callback(self, msg):
        start = self.get_parameter('ignore_sector_start').value
        end = self.get_parameter('ignore_sector_end').value
        new_msg = LaserScan()
        new_msg.header = msg.header
        new_msg.angle_min = msg.angle_min
        new_msg.angle_max = msg.angle_max
        new_msg.angle_increment = msg.angle_increment
        new_msg.time_increment = msg.time_increment
        new_msg.scan_time = msg.scan_time
        new_msg.range_min = msg.range_min
        new_msg.range_max = msg.range_max
        new_msg.ranges = list(msg.ranges)
        new_msg.intensities = list(msg.intensities)

        if self.get_parameter('mask_enabled').value:
            for i in range(len(msg.ranges)):
                if start <= i <= end:
                    new_msg.ranges[i] = float('inf')  # or msg.range_max

        self.pub.publish(new_msg)

def main(args=None):
    rclpy.init(args=args)
    node = DynamicMask()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
