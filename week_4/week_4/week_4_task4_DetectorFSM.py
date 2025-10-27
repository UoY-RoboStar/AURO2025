#!/usr/bin/env python3

# Sample solution for Week4's task 4.
#
# This Python program implements the TurtleBot3FSMC.

import rclpy
import week_4.robochart_fsm2
import math

# RoboChart specific implementations
from week_4.robochart_fsm2 import Machine, StateMachine, Context, Clock, TypedEvent, Event, MachineStatus, Generator
from week_4.robochart_sequence_toolkit import Seq, extract, ran
from week_4.robochart_set_toolkit import union

from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from typing import Callable
from enum import Enum

from auro_interfaces.msg import Obstacle

# Constant defined in DetectorFSM.
SCAN_THRESHOLD = 0.4

class Direction(Enum):
    LEFT = 0
    RIGHT = 1
    FRONT = 2

    @classmethod
    def default(cls):
        return cls.LEFT # Provide a default value

# We define DetectorFSM by inheriting from StateMachine
class DetectorFSM(StateMachine):

    class MachineContext(Context):
        data: Seq[float]

    def __init__(self, logger, timer_period, time_unit, context):
        super().__init__(logger, timer_period, time_unit, DetectorFSM.MachineContext(parent = context, data=Seq[float]([])))

    def initial(self): 
        return self.Idle

    def Idle(self):
        
        # No entry action
        self.logger.info("In state 'Idle'")
        
        while True:
            if self.context.scan.is_triggered():

                self.logger.info("Transition on 'scan' event triggered to junction")
                
                # Implementation of RoboChart trigger: scan?date
                # Save value communicated via the event 'obstacle' into obs variable and clear event.
                self.context.data = self.context.scan.value
                self.context.scan.clear()

                # Execute exit action, if any. There are none in this case.

                # Junction is a decision point, so evaluate the guards and perform the action.
                if min(ran(extract(union(set(range(345, 360)),set(range(1,15))),self.context.data))) < SCAN_THRESHOLD:
                    self.context.obstacle(Direction.FRONT)
                elif min(ran(extract(set(range(16,44)),self.context.data))) < SCAN_THRESHOLD:
                    self.context.obstacle(Direction.LEFT)
                elif min(ran(extract(set(range(315,344)),self.context.data))) < SCAN_THRESHOLD:
                    self.context.obstacle(Direction.RIGHT)
                else:
                    return self.Idle # We stay in the same state
            else:
                # Wait till next execution step
                yield

class DetectorFSMNode(Node):

    class PlatformContext(Context):
        obstacle: Callable[[Direction], None]
        scan: TypedEvent[Seq[float]]

    def __init__(self):
        super().__init__('DetectorFSMNode')
        
        # ROS2 Outputs
        self.publisher = self.create_publisher(Obstacle, 'obstacle', 1)

        # ROS2 Inputs
        self.subscriber = self.create_subscription(LaserScan, 'scan', self.scan_callback, 1)
        
        # Periodic callback for executing FSM
        self.timer_period = 0.1 # 100 milliseconds = 10 Hz for the control loop
        self.time_unit = 1 # 1 second is the value chosen for one 'time unit'

        self.timer = self.create_timer(self.timer_period, self.control_loop)

        # Setup of robotic platform context
        self.platform_context = DetectorFSMNode.PlatformContext(obstacle=self.send_obstacle, scan=TypedEvent[Seq[float]]())

        # Setup the state machine itself, passing the machine_context
        self.stm = DetectorFSM(self.get_logger(), self.timer_period, self.time_unit, context=self.platform_context)

    def send_obstacle(self, value):
        msg = Obstacle()

        # Mapping between model types and ROS2 Obstacle message type
        match value:
            case Direction.FRONT:
                msg.data = Obstacle.FRONT
            case Direction.LEFT:
                msg.data = Obstacle.LEFT
            case Direction.RIGHT:
                msg.data = Obstacle.RIGHT
            
        self.get_logger().info(f"Publishing obstacle: {msg}")
        self.publisher.publish(msg)

    def scan_callback(self, msg):
        self.get_logger().info(f"Received message on topic 'scan'")
        
        # We update the event in the context, setting and passing the data,
        # mapping from a ROS message type, in this case, LaserScan,
        # to a type that corresponds to the sequence of reals in RoboChart.
        self.platform_context.scan.set(Seq(msg.ranges))

        # Attempt to step the machine in response to the event. This step could be
        # omitted if we only step the machine at the periods set out by the control_loop callback.
        self.stm.execute(timed = False)

    def control_loop(self):

        # We step the machine
        self.get_logger().debug(f"-- Executing control_loop. --")
        self.stm.execute()

def main(args=None):

    rclpy.init(args=args)

    node = DetectorFSMNode()

    rclpy.spin(node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()