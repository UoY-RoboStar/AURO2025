#!/usr/bin/env python3

import math
import rclpy
import csv

from rclpy.node import Node
from nav_msgs.msg import Odometry
from assessment_interfaces.msg import BarrelList, Barrel
from rclpy.executors import ExternalShutdownException

class DataCollector(Node):

    def __init__(self):

        super().__init__('data_collector')

        # We need the position of the single barrel we'll be tracking as an input to this node.
        self.declare_parameter('barrel_x', 0.0)
        self.declare_parameter('barrel_y', 0.0)

        self.barrel_x = self.get_parameter('barrel_x').value
        self.barrel_y = self.get_parameter('barrel_y').value

        # Radius of barrels
        self.barrel_radius = 0.2

        # There are several ways to capture the pose of the robot in the world,
        # either using the TF from world to odom, or assuming that odom is ground-truth
        # (WORLD for odometry_source). Here, we follow the latter approach.
        self.odom_subscriber = self.create_subscription(Odometry, 'odom', self.odom_callback, 10)

        # Subscriber for the 'barrels' topic published by the visual_sensor.
        self.barrels_subscriber = self.create_subscription(BarrelList, 'barrels', self.barrels_callback, 10)

        self.robot_x = None
        self.robot_y = None
        self.distance = None

        self.log_file = open('task7_data_collection.csv', 'w')

        self.log_file.write('robot_x,robot_y,size,distance\n')
        self.log_file.flush()

    def odom_callback(self, msg):

        # We do not need yaw here, just (x, y)
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y

        # Calculate distance to obstacle and update it
        pure_distance = math.sqrt((self.robot_x - self.barrel_x)**2 + (self.robot_y - self.barrel_y)**2)

        if pure_distance >= self.barrel_radius:
            self.distance = pure_distance - self.barrel_radius
        else: # Somehow the robot is too close!
            self.distance = None

    def barrels_callback(self, msg):

        self.get_logger().info('Received barrels message.')

        # Ensure there are barrels in the message's data field
        if len(msg.data) > 0:

            # We take the first barrel -- this experiment is to be carried out with just one barrel visible
            barrel = msg.data[0]
            
            # We assume here that the position has been adjusted sufficiently close in time
            # to the publishing of the barrel information. 
            self.write_out_data(barrel.size)

    def write_out_data(self, size):

        if not (self.robot_x is None or self.robot_y is None or self.distance is None):
            self.log_file.write(f"{self.robot_x},{self.robot_y},{size},{self.distance}\n")

    def destroy_node(self):
        self.log_file.close()
        super().destroy_node()

def main(args=None):

    rclpy.init(args=args)

    node = DataCollector()

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