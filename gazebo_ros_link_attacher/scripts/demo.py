#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from gazebo_ros_link_attacher.srv import Attach
from gazebo_msgs.srv import SpawnEntity
from copy import deepcopy
from tf_transformations import quaternion_from_euler

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
      <collision name="collision">
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
      <visual name="visual">
        <pose>0 0 0 0 0 0</pose>
        <geometry>
          <box>
            <size>SIZEXYZ</size>
          </box>
        </geometry>
        <material>
          <ambient>0.7 0.5 0.3 1</ambient>
          <diffuse>0.7 0.5 0.3 1</diffuse>
        </material>
      </visual>
    </link>
  </model>
</sdf>
"""

class DemoNode(Node):
    def __init__(self):
        super().__init__('demo_attach_links')
        self.attach_srv = self.create_client(Attach, '/attach')
        self.spawn_srv = self.create_client(SpawnEntity, '/spawn_entity')
        
        while not self.spawn_srv.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /spawn_entity service...')
        self.get_logger().info('Connected to spawn service!')

        while not self.attach_srv.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /attach service...')
        self.get_logger().info('Connected to attach service!')

    def create_cube_request(self, modelname, px, py, pz, rr, rp, ry, sx, sy, sz):
        """Create a SpawnEntityRequest with the parameters of the cube given."""
        cube = deepcopy(sdf_cube)
        size_str = f"{sx:.3f} {sy:.3f} {sz:.3f}"
        cube = cube.replace('SIZEXYZ', size_str)
        cube = cube.replace('MODELNAME', str(modelname))

        req = SpawnEntity.Request()
        req.name = modelname
        req.xml = cube
        req.robot_namespace = ""
        
        q = quaternion_from_euler(rr, rp, ry)
        req.initial_pose.position.x = px
        req.initial_pose.position.y = py
        req.initial_pose.position.z = pz
        req.initial_pose.orientation.x = q[0]
        req.initial_pose.orientation.y = q[1]
        req.initial_pose.orientation.z = q[2]
        req.initial_pose.orientation.w = q[3]

        return req

    def run_demo(self):
        # Spawn cube1
        self.get_logger().info('Spawning cube1')
        req1 = self.create_cube_request("cube1",
                                      0.0, 0.0, 0.51,  # position
                                      0.0, 0.0, 0.0,  # rotation
                                      1.0, 1.0, 1.0)  # size
        fut = self.spawn_srv.call_async(req1)
        rclpy.spin_until_future_complete(self, fut)

        # Spawn cube2
        self.get_logger().info('Spawning cube2')
        req2 = self.create_cube_request("cube2",
                                      0.0, 1.1, 0.41,  # position
                                      0.0, 0.0, 0.0,  # rotation
                                      0.8, 0.8, 0.8)  # size
        fut = self.spawn_srv.call_async(req2)
        rclpy.spin_until_future_complete(self, fut)

        # Attach cubes
        self.get_logger().info('Attaching cube1 and cube2')
        attach_req = Attach.Request()
        attach_req.model_name_1 = "cube1"
        attach_req.link_name_1 = "link"
        attach_req.model_name_2 = "cube2"
        attach_req.link_name_2 = "link"

        fut = self.attach_srv.call_async(attach_req)
        rclpy.spin_until_future_complete(self, fut)
        self.get_logger().info('Attachment request sent!')

def main():
    rclpy.init()
    node = DemoNode()
    
    try:
        rclpy.spin_once(node, timeout_sec=1.0)
        node.run_demo()
        #rclpy.spin_until_future_complete(node, future)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
