import sys
import argparse

import rclpy
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from rclpy.executors import ExternalShutdownException

from assessment_interfaces.msg import BarrelLog, Radiation, RadiationList

class DataLogger(Node):

    def __init__(self, args):
        super().__init__('data_logger')

        parser = argparse.ArgumentParser()
        group = parser.add_argument_group()
        group.add_argument('--path', type=str, metavar='PATH', help='Path')
        group.add_argument('--filename', type=str, metavar='FILENAME', help='Filename')
        group.add_argument('--random_seed', type=str, metavar='RANDOM_SEED', help='Random seed')
        self.args = parser.parse_args(args[1:])

        full_filepath = self.args.path + self.args.filename + '_' + self.args.random_seed + '.csv'
        self.get_logger().info(f"Logging data to file: {full_filepath}")

        self.counter = 0
        self.log_file = open(full_filepath, 'w')

        self.log_file.write('counter,')
        self.log_file.write('red_count,blue_count,total_count,average_radiation_level,total_radiation_level\n')
        self.log_file.flush()

        self.average_radiation_level = 0
        self.total_radiation_level = 0  

        self.barrel_log_subscriber = self.create_subscription(
            BarrelLog,
            '/barrel_log',
            self.barrel_log_callback,
            10)

        self.radiation_level_subscriber = self.create_subscription(
            RadiationList,
            '/radiation_levels',
            self.radiation_level_callback,
            10)

    def radiation_level_callback(self, msg):

        radiation = 0
        num_robots = 0

        for r in msg.data:
            radiation += r.level 
            num_robots += 1

        if num_robots == 0:
            self.average_radiation_level = 0
        else:
            self.average_radiation_level = radiation / num_robots

        self.total_radiation_level = radiation

    def barrel_log_callback(self, msg):
        self.log_file.write(f'{self.counter},')
        self.log_file.write(f'{msg.red_count},{msg.blue_count},{msg.total_count},{self.average_radiation_level},{self.total_radiation_level}\n')
        self.log_file.flush()
        self.counter += 1

    def destroy_node(self):
        self.log_file.close()
        super().destroy_node()


def main(args=sys.argv):

    rclpy.init(args = args, signal_handler_options = SignalHandlerOptions.NO)

    args_without_ros = rclpy.utilities.remove_ros_args(args)

    node = DataLogger(args_without_ros)

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