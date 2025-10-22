#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from gazebo_ros_link_attacher.srv import Attach


class DetachClient(Node):
    def __init__(self):
        super().__init__('demo_detach_links')
        self.detach_client = self.create_client(Attach, '/detach')
        while not self.detach_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for detach service...')
        self.get_logger().info('Connected to service!')

    def detach_cubes(self):
        # Detach cube1 and cube2
        self.get_logger().info('Detaching cube1 and cube2')
        req = Attach.Request()
        req.model_name_1 = "cube1"
        req.link_name_1 = "link"
        req.model_name_2 = "cube2"
        req.link_name_2 = "link"
        fut = self.detach_client.call_async(req)
        rclpy.spin_until_future_complete(self, fut)


def main():
    rclpy.init()
    detacher = DetachClient()
    rclpy.spin_once(detacher, timeout_sec=1.0)
    detacher.detach_cubes()
    detacher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
