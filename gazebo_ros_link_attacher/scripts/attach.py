#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from gazebo_ros_link_attacher.srv import Attach


class AttachClient(Node):
    def __init__(self):
        super().__init__('demo_attach_links')
        self.attach_client = self.create_client(Attach, '/attach')
        while not self.attach_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for attach service...')
        self.get_logger().info('Connected to service!')

    def attach_cubes(self):
        # Attach cube1 and cube2
        self.get_logger().info('Attaching cube1 and cube2')
        req = Attach.Request()
        req.model_name_1 = "cube1"
        req.link_name_1 = "link"
        req.model_name_2 = "cube2"
        req.link_name_2 = "link"
        fut = self.attach_client.call_async(req)
        rclpy.spin_until_future_complete(self, fut)

        # Attach cube2 and cube3
        self.get_logger().info('Attaching cube2 and cube3')
        req = Attach.Request()
        req.model_name_1 = "cube2"
        req.link_name_1 = "link"
        req.model_name_2 = "cube3"
        req.link_name_2 = "link"
        fut = self.attach_client.call_async(req)
        rclpy.spin_until_future_complete(self, fut)

        # Attach cube3 and cube1
        self.get_logger().info('Attaching cube3 and cube1')
        req = Attach.Request()
        req.model_name_1 = "cube3"
        req.link_name_1 = "link"
        req.model_name_2 = "cube1"
        req.link_name_2 = "link"
        fut = self.attach_client.call_async(req)
        rclpy.spin_until_future_complete(self, fut)


def main():
    rclpy.init()
    attacher = AttachClient()
    rclpy.spin_once(attacher, timeout_sec=1.0)
    attacher.attach_cubes()
    attacher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
