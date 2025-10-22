#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from gazebo_msgs.srv import SpawnEntity
from geometry_msgs.msg import Pose
from tf_transformations import quaternion_from_euler
import time

sdf_cube = """<?xml version="1.0" ?>
<sdf version="1.6">
  <model name="MODELNAME">
    <static>0</static>
    <link name="link">
      <inertial>
        <mass>1.0</mass>
        <inertia>
          <ixx>0.01</ixx>
          <ixy>0.0</ixy>
          <ixz>0.0</ixz>
          <iyy>0.01</iyy>
          <iyz>0.0</iyz>
          <izz>0.01</izz>
        </inertia>
      </inertial>
      <collision name="stairs_collision0">
        <pose>0 0 0 0 0 0</pose>
        <geometry>
          <box>
            <size>SIZEXYZ</size>
          </box>
        </geometry>
        <surface>
          <bounce />
          <friction>
            <ode>
              <mu>1.0</mu>
              <mu2>1.0</mu2>
            </ode>
          </friction>
          <contact>
            <ode>
              <kp>10000000.0</kp>
              <kd>1.0</kd>
              <min_depth>0.0</min_depth>
              <max_vel>0.0</max_vel>
            </ode>
          </contact>
        </surface>
      </collision>
      <visual name="stairs_visual0">
        <pose>0 0 0 0 0 0</pose>
        <geometry>
          <box>
            <size>SIZEXYZ</size>
          </box>
        </geometry>
        <material>
          <script>
            <uri>file://media/materials/scripts/gazebo.material</uri>
            <name>Gazebo/Wood</name>
          </script>
        </material>
      </visual>
      <velocity_decay>
        <linear>0.000000</linear>
        <angular>0.000000</angular>
      </velocity_decay>
      <self_collide>0</self_collide>
      <kinematic>0</kinematic>
      <gravity>1</gravity>
    </link>
  </model>
</sdf>
"""


class ModelSpawner(Node):
    def __init__(self):
        super().__init__('spawn_models')
        self.spawn_client = self.create_client(SpawnEntity, '/spawn_entity')
        while not self.spawn_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /spawn_entity service...')
        self.get_logger().info('Connected to service!')

    def create_cube_request(self, modelname, px, py, pz, rr, rp, ry, sx, sy, sz):
        cube = sdf_cube.replace('SIZEXYZ', f"{sx:.3f} {sy:.3f} {sz:.3f}")
        cube = cube.replace('MODELNAME', str(modelname))

        req = SpawnEntity.Request()
        req.name = modelname
        req.xml = cube
        req.robot_namespace = ""

        pose = Pose()
        pose.position.x = px
        pose.position.y = py
        pose.position.z = pz

        q = quaternion_from_euler(rr, rp, ry)
        pose.orientation.x = q[0]
        pose.orientation.y = q[1]
        pose.orientation.z = q[2]
        pose.orientation.w = q[3]

        req.initial_pose = pose
        return req
    def spawn_cubes(self):
        # Spawn cube1
        self.get_logger().info('Spawning cube1')
        req1 = self.create_cube_request("cube1",
                                      0.0, 0.0, 0.51,  # position
                                      0.0, 0.0, 0.0,  # rotation
                                      1.0, 1.0, 1.0)  # size
        fut = self.spawn_client.call_async(req1)
        rclpy.spin_until_future_complete(self, fut)
        time.sleep(1.0)

        # Spawn cube2
        self.get_logger().info('Spawning cube2')
        req2 = self.create_cube_request("cube2",
                                      0.0, 1.1, 0.41,  # position
                                      0.0, 0.0, 0.0,  # rotation
                                      0.8, 0.8, 0.8)  # size
        fut = self.spawn_client.call_async(req2)
        rclpy.spin_until_future_complete(self, fut)
        time.sleep(1.0)

        # Spawn cube3
        self.get_logger().info('Spawning cube3')
        req3 = self.create_cube_request("cube3",
                                      0.0, 2.1, 0.41,  # position
                                      0.0, 0.0, 0.0,  # rotation
                                      0.4, 0.4, 0.4)  # size
        fut = self.spawn_client.call_async(req3)
        rclpy.spin_until_future_complete(self, fut)

def main():
    rclpy.init()
    spawner = ModelSpawner()
    rclpy.spin_once(spawner, timeout_sec=1.0)
    spawner.spawn_cubes()
    spawner.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
