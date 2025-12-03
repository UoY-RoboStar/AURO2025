import rclpy
from rclpy.node import Node
from rcl_interfaces.srv import SetParameters

class Task8Node(Node):
    def __init__(self):
        super().__init__('task8_node')

        self.cli = self.create_client(SetParameters, '/local_costmap/local_costmap/set_parameters')

        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for service...')

        # Build request
        from rclpy.parameter import Parameter
        req = SetParameters.Request()
        req.parameters = [
            Parameter('inflation_layer.inflation_radius', Parameter.Type.DOUBLE, 5.0).to_parameter_msg()
        ]

        # Call service
        future = self.cli.call_async(req)
        future.add_done_callback(self.callback)

    def callback(self, future):
        result = future.result()
        self.get_logger().info(f"Set result: {result.results[0].successful}")
        self.destroy_node()
        raise KeyboardInterrupt  # To exit the spin loop

def main():
    rclpy.init()
    node = Task8Node()
    
    try:
        # Spin the node so the callbacks are processed.
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()

if __name__ == '__main__':
    main()
