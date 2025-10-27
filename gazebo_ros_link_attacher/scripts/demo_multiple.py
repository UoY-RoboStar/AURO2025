#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from gazebo_ros_link_attacher.srv import Attach
from gazebo_msgs.srv import SpawnEntity
from copy import deepcopy
from tf_transformations import quaternion_from_euler

sdf_cube = """<?xml version="1.0" ?>
<sdf version="1.4">
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


class DemoMultipleNode(Node):
    def __init__(self):
        super().__init__('demo_multiple_attach_links')
        self.attach_srv = self.create_client(Attach, '/link_attacher_node/attach')
        self.spawn_srv = self.create_client(SpawnEntity, '/spawn_entity')

        while not self.spawn_srv.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /spawn_entity service...')
        self.get_logger().info('Connected to spawn service!')

        while not self.attach_srv.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /link_attacher_node/attach service...')
        self.get_logger().info('Connected to attach service!')

    def create_cube_request(self, modelname, px, py, pz, rr, rp, ry, sx, sy, sz):
        """Create a SpawnEntityRequest with the parameters of the cube given."""
        cube = deepcopy(sdf_cube)
        # Replace size of model
        size_str = str(round(sx, 3)) + " " + \
            str(round(sy, 3)) + " " + str(round(sz, 3))
        cube = cube.replace('SIZEXYZ', size_str)
        # Replace modelname
        cube = cube.replace('MODELNAME', str(modelname))

        req = SpawnEntity()
        req.name = modelname
        req.xml = cube
        req.robot_namespace = modelname
        req.initial_pose.position.x = px
        req.initial_pose.position.y = py
        req.initial_pose.position.z = pz

        q = quaternion_from_euler(rr, rp, ry)
        req.initial_pose.orientation.x = q[0]
        req.initial_pose.orientation.y = q[1]
        req.initial_pose.orientation.z = q[2]
        req.initial_pose.orientation.w = q[3]

        return req

    async def run_demo(self):
        # Spawn cube1
        self.get_logger().info('Spawning cube1')
        req1 = self.create_cube_request("cube1",
                                      0.0, 0.0, 0.51,
                                      0.0, 0.0, 0.0,
                                      1.0, 1.0, 1.0)
        await self.spawn_srv.call_async(req1)
        await rclpy.sleep(1.0)

        # Spawn cube2
        self.get_logger().info('Spawning cube2')
        req2 = self.create_cube_request("cube2",
                                      0.0, 1.1, 0.41,
                                      0.0, 0.0, 0.0,
                                      0.8, 0.8, 0.8)
        await self.spawn_srv.call_async(req2)
        await rclpy.sleep(1.0)

        # Spawn cube3
        self.get_logger().info('Spawning cube3')
        req3 = self.create_cube_request("cube3",
                                      0.0, 2.1, 0.41,
                                      0.0, 0.0, 0.0,
                                      0.4, 0.4, 0.4)
        await self.spawn_srv.call_async(req3)
        await rclpy.sleep(1.0)

        # Attach cube1 and cube2
        self.get_logger().info('Attaching cube1 and cube2')
        req = Attach.Request()
        req.model_name_1 = "cube1"
        req.link_name_1 = "link"
        req.model_name_2 = "cube2"
        req.link_name_2 = "link"
        await self.attach_srv.call_async(req)
        await rclpy.sleep(1.0)

        # Attach cube2 and cube3
        self.get_logger().info('Attaching cube2 and cube3')
        req = Attach.Request()
        req.model_name_1 = "cube2"
        req.link_name_1 = "link"
        req.model_name_2 = "cube3"
        req.link_name_2 = "link"
        await self.attach_srv.call_async(req)
        await rclpy.sleep(1.0)

        # Attach cube3 and cube1
        self.get_logger().info('Attaching cube3 and cube1')
        req = Attach.Request()
        req.model_name_1 = "cube3"
        req.link_name_1 = "link"
        req.model_name_2 = "cube1"
        req.link_name_2 = "link"
        await self.attach_srv.call_async(req)
        await rclpy.sleep(2.0)


def main():
    rclpy.init()
    node = DemoMultipleNode()

    try:
        rclpy.spin_once(node)
        future = node.run_demo()
        rclpy.spin_until_future_complete(node, future)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
