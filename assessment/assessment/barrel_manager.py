#!/usr/bin/env python3
#
# Copyright (c) 2025 University of York and others
#
# This node provides management of picking up, towing, and dropping off of barrels, as well
# as a simple simulation of radiation while picking up contaminated barrels.
#
# It also publishes logs about how many barrels have been deposited in collection zones,
# and level of radiation.
# 
# Contributors:
#   * Pedro Ribeiro - initial implementation

import os
import sys
import math
import random
import argparse
import multiprocessing
import rclpy
import asyncio

from ament_index_python.packages import get_package_share_directory
from enum import Enum
import xml.etree.ElementTree as ET

from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.node import Node
from rclpy.time import Duration

from gazebo_msgs.msg import EntityState, ModelStates
from gazebo_msgs.srv import SpawnEntity, GetModelList, GetEntityState, SetEntityState

import tf2_geometry_msgs
from tf_transformations import quaternion_matrix

import numpy as np

from auro_interfaces.srv import ItemRequest

# ROS2 types for positioning
from geometry_msgs.msg import Pose, Point, Quaternion

# Attachment
from gazebo_ros_link_attacher.srv import Attach

# Assessment interfaces
from assessment_interfaces.msg import Barrel, BarrelHolder, BarrelHolders, BarrelList, BarrelLog, Radiation, RadiationList, Zone, ZoneList

class Zone(Enum):
    ZONE_A = 1
    ZONE_B = 2

class ZoneData():

    def __init__(self, x, y, size):
        self.x = x
        self.y = y
        self.items_returned = set()
        self.size = size

    def inside(self, x, y):
        '''Returns where the point (x,y) is within the zone'''
        # Assumes (self.x,self.y) is the origin of a square
        return (self.x-(self.size/2) <= x and x <= self.x+(self.size/2) and self.y-(self.size/2) <= y and self.y+(self.size/2))

    def items(self):
        return self.items_returned

    def deposit(self, barrel):
        self.items_returned.add(barrel)

    def __repr__(self):
        return f'(x: {self.x}, y: {self.y}, items_returned: {self.items_returned})'


class BarrelType(Enum):
    CONTAMINATED = 1
    OTHER = 2

    def name(self):
        if self == BarrelType.CONTAMINATED:
            return "contaminated"
        else: 
            return "other"
        return self.name

class Barrel():

    def __init__(self, name, x, y, type: BarrelType, cluster_id):
        self.x = x
        self.y = y
        self.name = name
        self.type = type
        self.cluster_id = cluster_id

    def barrel_holder_colour(self):

        match self.type:
            case BarrelType.CONTAMINATED:
                return BarrelHolder.RED
            case BarrelType.OTHER:
                return BarrelHolder.BLUE
            case _:
                return None

    def has_radiation(self):
        return self.type == BarrelType.CONTAMINATED

    def __repr__(self):
        return f"Barrel(type={self.type}, x={self.x}, y={self.y})"

class Cluster():
    
    def __init__(self, x, y, func = None):
        self.x = x
        self.y = y
        self.func = func

    def valid(self, x, y):
        if self.func is None:
            return True
        else:
            return self.func(x,y)

    def __repr__(self):
        return f'({self.x}, {self.y})'

class Robot():

    def __init__(self, robot_id='', contamination_level=0):
        self.robot_id = robot_id
        self.item_held = None
        self.previous_item_held = None
        self.contamination_level = contamination_level

    def __repr__(self):
        return f'({self.robot_id}, {self.item_held})'

    def contamination_step(self):
        self.contamination_level += 1

    def decontaminate(self):
        self.contamination_level = 0

class BarrelManager(Node):

    def __init__(self):
        super().__init__('barrel_manager')

        # ROS2 parameters for node
        self.declare_parameter('random_seed', 0)
        random.seed(self.get_parameter('random_seed').value)

        # Load and configure barrel models
        self.item_models = {}
        self.load_and_configure_models()

        # Barrels
        self.barrels = {}

        # Central locations of barrel clusters
        self.clusters = {}
        self.clusters[0] = Cluster(-2.0, 4.15, lambda x, y: (-2.75 <= x and x <= -1.24))

        # First execution?
        self.first_execution = True

        # Number of items
        self.item_counter = 0

        # Keep track of items returned per zone
        self.zones = {}
        self.zones[Zone.ZONE_A] = ZoneData(x=13.5, y=9.4, size=3.0)
        self.zones[Zone.ZONE_B] = ZoneData(x=19.5, y=9.4, size=3.0)

        # Decontamination zone
        self.decontamination_zone = ZoneData(x=7.5, y=9.4, size=3.0)

        # Barrel positions
        self.filtered_barrel_names = []
        self.filtered_barrel_positions = np.empty((0,3))

        # Robot positions
        self.robot_positions = {}
        self.robot_names = []

        # Internal tracking of robots and their held items
        self.robots = {}

        # Mutually exclusive callback groups
        client_callback_group = MutuallyExclusiveCallbackGroup()
        timer_callback_group = MutuallyExclusiveCallbackGroup()
        service_callback_group = MutuallyExclusiveCallbackGroup()

        # ROS2 service clients and servers
        self.spawn_entity_client = self.create_client(SpawnEntity, '/spawn_entity', callback_group=client_callback_group)
        self.get_model_list_client = self.create_client(GetModelList, '/get_model_list', callback_group=client_callback_group)
        self.get_entity_state_client = self.create_client(GetEntityState, '/get_entity_state', callback_group=client_callback_group)
        self.set_entity_state_client = self.create_client(SetEntityState, '/set_entity_state', callback_group=client_callback_group)

        # Gazebo attachment service clients
        self.attach_client = self.create_client(Attach, '/attach', callback_group=client_callback_group)
        self.detach_client = self.create_client(Attach, '/detach', callback_group=client_callback_group)

        # Provide services to collect item or offload item, but ensure no concurrency with regular control loop
        self.pick_up_service = self.create_service(ItemRequest, '/pick_up_item', self.pick_up_item, callback_group=timer_callback_group)
        self.offload_service = self.create_service(ItemRequest, '/offload_item', self.offload_item, callback_group=timer_callback_group)
        self.decontamination_service = self.create_service(ItemRequest, '/decontaminate', self.decontaminate, callback_group=timer_callback_group)

        # ROS2 subscribers
        self.model_states_subscriber = self.create_subscription(ModelStates, '/model_states', self.model_states_callback, 10, callback_group=timer_callback_group)

        # ROS2 publishers
        self.item_holders_publisher = self.create_publisher(BarrelHolders, '/barrel_holders', 10)
        self.barrel_log_publisher = self.create_publisher(BarrelLog, '/barrel_log', 10)
        self.radiation_publisher = self.create_publisher(RadiationList, '/radiation_levels', 10)

        # Control loop timer
        self.timer = self.create_timer(1.0, self.control_loop, callback_group=timer_callback_group)

    def model_states_callback(self, msg):

        # Save and update the position of all barrels
        names = np.array(msg.name)

        # We ignore 'z' coordinate
        positions = np.array([[p.position.x, p.position.y, 0] for p in msg.pose])

        mask = np.array([n.startswith('barrel_') for n in names])

        self.filtered_barrel_names = names[mask]
        self.filtered_barrel_positions = positions[mask]

        robot_positions = np.array([[p.position.x, p.position.y, p.position.z, p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w] for p in msg.pose])

        mask = np.array([n.startswith('robot') for n in names])

        filtered_robot_positions = robot_positions[mask]
        self.robot_names = names[mask]

        # self.get_logger().info(f"Updated robot positions: {filtered_robot_positions} at {self.robot_names}")

        for i, robot_name in enumerate(self.robot_names):
            pose_list = filtered_robot_positions[i]
            robot_pose = self.pose_from_list(pose_list)
            self.robot_positions[robot_name] = robot_pose

    def pose_from_list(self, pose_list):
        """
        Convert a 7-element list [x, y, z, qx, qy, qz, qw] into a geometry_msgs Pose.
        """
        if len(pose_list) != 7:
            raise ValueError("Pose list must have 7 elements: [x,y,z,qx,qy,qz,qw]")

        pose = Pose()
        pose.position = Point(x=pose_list[0], y=pose_list[1], z=pose_list[2])
        pose.orientation = Quaternion(x=pose_list[3], y=pose_list[4],
                                    z=pose_list[5], w=pose_list[6])
        return pose


    def transform_points(self, points_world, robot_pose):

        # Extract translation
        t = np.array([robot_pose.position.x,
                    robot_pose.position.y,
                    robot_pose.position.z])

        # Extract rotation (as matrix)
        q = [robot_pose.orientation.x,
             robot_pose.orientation.y,
             robot_pose.orientation.z,
             robot_pose.orientation.w]
             
        R = quaternion_matrix(q)[:3, :3]  # Convert quaternion to 3x3 rotation

        # Apply transformation to all points
        points_robot = ((points_world - t) @ R).T  # Actually need R.T
        points_robot = (R.T @ (points_world - t).T).T
        return points_robot

    def load_and_configure_models(self):

        models_path = os.path.join(get_package_share_directory("assessment"), "models")
        barrel_model_path = os.path.join(models_path, "barrel", "model.sdf")

        tree = ET.parse(barrel_model_path)
        root = tree.getroot()

        for visual in root.findall(".//model/link/visual/material/script/name"):
            visual.text = "red_outlined"  # Red colour for contaminated barrels

        self.item_models[BarrelType.CONTAMINATED] = ET.tostring(root, encoding='unicode')

        for visual in root.findall(".//model/link/visual/material/script/name"):
            visual.text = "blue_outlined"  # Blue colour for other barrels

        self.get_logger().info(f"Loaded and configured barrel models: '{visual}'")

        self.item_models[BarrelType.OTHER] = ET.tostring(root, encoding='unicode')
        
    def add_barrel(self, x, y, cluster_id, type: BarrelType):
        name = "barrel_" + str(self.item_counter)
        barrel = Barrel(name, x, y, type, cluster_id)
        self.barrels[name] = barrel
        self.item_counter += 1
        self.get_logger().info(f"Added {barrel}")

    def get_barrels(self):
        return self.barrels

    def generate_cluster_location(self):
        '''Generate new cluster locations within defined zones, avoiding obstacles and existing clusters'''

        while True:
            # Randomly generate a candidate cluster location
            y = random.randint(5, 11)
            x = random.randint(-15, -8)

            # Invalid if it clashes with an existing cluster
            for cluster in self.clusters.values():
                if (cluster.x == x and cluster.x == y) or math.dist((cluster.x, cluster.y), (x, y)) <= 1.0:
                    break
                # Invalid if it clashes with an obstacle
                if (cluster.x == -14 and cluster.y == 6) or (cluster.x == -10 and cluster.y == 6) or (cluster.x == -14 and cluster.y == 10) or (cluster.x == -10 and cluster.y == 10):
                    break
            else:                    
                return Cluster(x, y)

    def generate_barrel_position(self, cluster_id):

        while True:
            if cluster_id == 0:
                radius = random.uniform(0.4, 1.0)
            else:
                radius = random.uniform(1.0, 1.85)

            angle = math.radians(random.uniform(0, 360))

            x = self.clusters[cluster_id].x + round(radius * math.cos(angle), 2)
            y = self.clusters[cluster_id].y + round(radius * math.sin(angle), 2)

            if not self.clusters[cluster_id].valid(x,y):
                continue

            for b in self.barrels.values():
                if b.cluster_id == cluster_id and math.dist((b.x, b.y), (x, y)) < 0.4:
                    break
            else:
                return x, y
    
    def generate_barrels_in_cluster(self, cluster_id, num_barrels):
        '''Generate barrels within a cluster'''

        for _ in range(num_barrels):
            x, y = self.generate_barrel_position(cluster_id)
            barrel_type = BarrelType.CONTAMINATED if random.random() < 0.6 else BarrelType.OTHER
            self.add_barrel(x, y, cluster_id, barrel_type)

    def filter_points_behind(self, points, max_dist=2.0, alpha=np.deg2rad(60)):
        """
        Filters 2D points that are within a distance threshold and
        within an angular sector behind the robot (+X forward).

        Args:
            points (Nx2 np.ndarray): points in robot frame
            max_dist (float): maximum distance from origin
            alpha (float): total angular width (radians) for the "behind" sector

        Returns:
            mask (np.ndarray[bool]): True where point satisfies both conditions
            filtered_points (Nx2 np.ndarray): subset of points behind & close
        """
        # --- Compute polar coordinates ---
        dist = np.hypot(points[:, 0], points[:, 1])
        angles = np.arctan2(points[:, 1], points[:, 0])  # radians in [-π, π]

        # --- Distance condition ---
        mask_dist = dist <= max_dist

        # --- Angular sector centered at π (behind the robot) ---
        lower = np.pi - alpha / 2.0
        upper = -np.pi + alpha / 2.0  # wraps around negative side

        # Angles near +π or –π are “behind”
        mask_angle = (angles >= lower) | (angles <= upper)

        # --- Combine conditions ---
        mask = mask_dist & mask_angle

        return mask, points[mask]

    def get_barrel_position(self, barrel_name):
        '''Get the position of a barrel, given its name, based on the positions received, so far, from Gazebo.'''

        if len(self.filtered_barrel_positions) == 0 or len(self.filtered_barrel_names) == 0:
            return None

        if barrel_name in self.filtered_barrel_names:

            idx = np.where(self.filtered_barrel_names == barrel_name)[0]
            if len(idx) > 0:
                index = idx[0]
                return self.filtered_barrel_positions[index]
            else:
                return None
        else:
            return None       

    # ROS2 callbacks
    async def decontaminate(self, request, response):
        '''
        Resets the decontamination value for a robot.
        '''
        response.success = False

        if request.robot_id not in self.robot_positions.keys():
            response.message = f"Unable to find robot_id '{request.robot_id}'"
            return response

        if request.robot_id not in self.robots.keys():
            self.robots[request.robot_id] = Robot(request.robot_id)
        else:
            robot = self.robot_positions[request.robot_id]

            if self.decontamination_zone.inside(robot.position.x, robot.position.y):
                self.get_logger().info(f"Initiating decontamination of robot with id '{request.robot_id}'")
                self.robots[request.robot_id].decontaminate()
                response.success = True
                response.message = f"Decontaminated robot_id '{request.robot_id}'"
            else:
                self.get_logger().info(f"Robot with id '{request.robot_id}' asked for decontamination, but is outside the decontamination zone.")

        return response

    async def pick_up_item(self, request, response):
        '''
        Makes the robot whose 'robot_id' passed in the request to pick an item behind it.
        The 'success' field of response is set to True if pick up succeeded, and otherwise
        is set to False. The 'message' field provides further details.
        '''

        self.get_logger().info(f"Incoming request on pick_up_item from robot_id '{request.robot_id}'")

        if request.robot_id not in self.robot_positions.keys():
            response.success = False
            response.message = f"Unable to find robot_id '{request.robot_id}'"
            return response

        if len(self.filtered_barrel_positions) == 0:
            response.success = False
            response.message = "No barrels available to pick up"
            return response

        # Need to check whether the robot is already holding an item.
        if request.robot_id not in self.robots.keys():
            self.robots[request.robot_id] = Robot(request.robot_id)
        elif self.robots[request.robot_id].item_held is not None:
            response.success = False
            response.message = f"Robot '{request.robot_id}' is already holding an item"
            return response

        # We do have the robot's position and some barrels in the world, so transform the barrel positions
        # into the robot's frame of reference.
        points_robot = self.transform_points(self.filtered_barrel_positions, self.robot_positions[request.robot_id])

        # self.get_logger().info(f"All barrels: {self.filtered_barrel_names} \n Positions: {self.filtered_barrel_positions}")
        # self.get_logger().info(f"Points in range: {points_robot}")

        if points_robot is not None:
            
            mask2, filtered_points_2d = self.filter_points_behind(points_robot[:, :2], max_dist=0.45, alpha=np.deg2rad(30))
            final_names = self.filtered_barrel_names[mask2]

            if len(final_names) > 0:
                # Pick the first barrel in the list
                picked_barrel_name = final_names[0]

                # Now attach barrel behind robot using gazebo_ros_link_attacher.
                req = Attach.Request()
                req.model_name_1 = request.robot_id
                req.link_name_1 = "base_link"
                req.model_name_2 = picked_barrel_name
                req.link_name_2 = "body"

                attach_result = await self.attach_client.call_async(req)

                if attach_result.ok:
                    self.robots[request.robot_id].item_held = self.barrels[picked_barrel_name]
                    self.get_logger().info(f"Robot '{request.robot_id}' picked up barrel '{picked_barrel_name}'")
                    response.success = True
                    response.message = f"Robot '{request.robot_id}' picked up barrel '{picked_barrel_name}' successfully"
                    return response
                else:
                    response.success = False
                    response.message = f"Failed to attach barrel '{picked_barrel_name}' to robot '{request.robot_id}'"
                    return response
            else:
                response.success = False
                response.message = f"No barrels behind robot '{request.robot_id}' within range"
                return response

    async def offload_item(self, request, response):
        '''
        Makes the robot whose 'robot_id' passed in the request to offload its held item.
        The 'success' field of response is set to True if offload succeeded, and otherwise
        is set to False. The 'message' field provides further details.
        '''     

        if request.robot_id not in self.robots.keys():
            response.success = False
            response.message = f"Unable to find robot_id '{request.robot_id}'"
            return response
        elif self.robots[request.robot_id].item_held is None:
            response.success = False
            response.message = f"Robot '{request.robot_id}' is not holding any item"
            return response
        else:
            req = Attach.Request()
            barrel = self.robots[request.robot_id].item_held
            req.model_name_1 = request.robot_id
            req.link_name_1 = "base_link"
            req.model_name_2 = barrel.name
            req.link_name_2 = "body"

            detach_result = await self.detach_client.call_async(req)

            if detach_result.ok:

                self.get_logger().info(f"Robot '{request.robot_id}' offloaded barrel '{barrel.name}'.")
                self.robots[request.robot_id].item_held = None
                response.success = True
                response.message = f"Robot '{request.robot_id}' offloaded barrel '{barrel.name}' successfully."

                # Check whether offload is taking place within a collection zone and it's the first time the
                # item is dropped in it.
                accountable = True

                for  _, zone in self.zones.items():
                    if barrel in zone.items():
                        accountable = False

                if accountable:
                    barrel_position = self.get_barrel_position(barrel.name)

                    if barrel_position is not None:
                        for _, zone in self.zones.items():
                            if zone.inside(barrel_position[0], barrel_position[1]):
                                zone.deposit(barrel)
                                response.message = f"Robot '{request.robot_id}' offloaded barrel '{barrel.name}' successfully in a collection zone."
                                return response
                    else:
                        self.get_logger().info(f"Could not find position for barrel '{barrel.name}'.")
                else:
                    self.get_logger().info(f"Barrel '{barrel.name}' has already previously been accounted for in a collection zone.")

                return response

            else:
                response.success = False
                response.message = f"Failed to detach barrel '{barrel.name}' from robot '{request.robot_id}'."
                return response

    async def initialise(self):

        self.generate_barrels_in_cluster(0, 10)
        self.get_logger().info('Generating clusters...')

        # Generate initial clusters and barrels
        for cluster_id in range(1,5):
            self.get_logger().info(f'Generating cluster id: {str(cluster_id)}')
            self.clusters[cluster_id] = self.generate_cluster_location()
            self.generate_barrels_in_cluster(cluster_id, 5)

        # Spawn barrels in Gazebo
        for p, b in self.barrels.items():
            self.get_logger().info(f'Spawning barrel {p} at x: {b.x}, y: {b.y}, type: {b.type}')
            await self.spawn_item(p, b.x, b.y, b.type)

        await self.spawn_item("ready", 0.0, 2.0, BarrelType.OTHER)

    async def control_loop(self):

        if self.first_execution:
            await self.initialise()
            self.first_execution = False

        # Publish item holders
        item_holders = BarrelHolders()

        for robot_id, robot in self.robots.items():
            if robot.item_held is not None:
                if robot.item_held.name in self.barrels:
                    item_holder = BarrelHolder()
                    item_holder.robot_id = robot_id
                    item_holder.colour = robot.item_held.barrel_holder_colour()
                    item_holders.data.append(item_holder)

                    # Update contamination level
                    if robot.item_held.has_radiation():
                        robot.contamination_step()

        self.item_holders_publisher.publish(item_holders)

        # Publish barrel collection log
        barrel_log = BarrelLog()

        for _, zone in self.zones.items():
            for barrel in zone.items_returned:
                match barrel.type:
                    case BarrelType.CONTAMINATED:
                        barrel_log.red_count += 1
                    case BarrelType.OTHER:
                        barrel_log.blue_count +=1

        barrel_log.total_count = barrel_log.red_count + barrel_log.blue_count

        self.barrel_log_publisher.publish(barrel_log)

        # Publish radiation levels

        radiation_levels = RadiationList()

        for robot_id, robot in self.robots.items():
            radiation = Radiation()
            radiation.robot_id = robot_id
            radiation.level = robot.contamination_level
            radiation_levels.data.append(radiation)

        self.radiation_publisher.publish(radiation_levels)

    def spawn_item(self, name, x, y, type, z = 0.0):

        while not self.spawn_entity_client.wait_for_service():
            pass

        request = SpawnEntity.Request()
        request.name = name
        request.xml = self.item_models[type]
        request.initial_pose.position.x = x
        request.initial_pose.position.y = y
        request.initial_pose.position.z = z
        request.reference_frame = "world"

        return self.spawn_entity_client.call_async(request)

def main(args=None):

    rclpy.init(args=args)

    # args_without_ros = rclpy.utilities.remove_ros_args(args)

    node = BarrelManager()

    if multiprocessing.cpu_count() >= 3:
        executor = MultiThreadedExecutor()
    else:
        executor = MultiThreadedExecutor(num_threads=3)
    
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    except ExternalShutdownException:
        sys.exit(1)
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.try_shutdown()

if __name__ == '__main__':
    main()