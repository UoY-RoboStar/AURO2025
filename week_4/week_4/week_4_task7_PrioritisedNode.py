#!/usr/bin/env python3

import rclpy
import week_4.robochart_fsm2
from week_4.robochart_fsm2 import StateMachine, Machine, Context, Clock, TypedOutputEvent, TypedEvent
from rclpy.node import Node
from geometry_msgs.msg import Twist, Vector3
from typing import Callable

lvel = 0.1  # Linear velocity
avel = 0.5  # Angular velocity

class PrioritisedNodeMachine(StateMachine):

    class MachineContext(Context):
        clock_C: Clock
        msg : Twist

    def __init__(self, logger, timer_period, time_unit, context):
        super().__init__(logger, timer_period, time_unit, PrioritisedNodeMachine.MachineContext(parent = context, clock_C=Clock(timer_period, time_unit), msg = Twist()))

    def initial(self): 
        return self.NotReceived1

    def NotReceived1(self):
        
        # No entry action
        self.logger.info("In state 'NotReceived1'")

        # Check transitions out of state NotReceived1
        while True:
            if self.context.cmd_vel_1.is_triggered():
                self.context.msg = self.context.cmd_vel_1.value
                self.context.cmd_vel_1.clear()
                self.context.clock_C.reset()

                self.logger.info("Transitioning to state 'Received'")
                return self.Received
            elif self.context.cmd_vel_2.is_triggered():
                self.context.msg = self.context.cmd_vel_2.value
                self.context.cmd_vel_2.clear()

                self.context.cmd_vel.set(self.context.msg)
                # Stay in the same state
                return self.NotReceived1
            else:
                yield

    def Received(self):

        # Entry action
        self.context.cmd_vel.set(self.context.msg)

        # Check transitions out of state Received
        while True:
            if self.context.cmd_vel_1.is_triggered():
                self.context.msg = self.context.cmd_vel_1.value
                self.context.cmd_vel_1.clear()
                self.context.clock_C.reset()
                return self.Received
            elif self.context.cmd_vel_2.is_triggered() and self.context.clock_C.since() > 1:
                self.context.msg = self.context.cmd_vel_2.value
                self.context.cmd_vel_2.clear()
                # Stay in the same state
                return self.Received
            else:
                yield

class PrioritisedNode(Node):

    class PlatformContext(Context):
        cmd_vel: TypedOutputEvent[Twist]
        cmd_vel_1: TypedEvent[Twist]
        cmd_vel_2: TypedEvent[Twist]       

    def __init__(self):
        super().__init__('PrioritisedNode')

        # ROS2 Inputs
        self.cmd_vel_1_sub = self.create_subscription(Twist, 'cmd_vel_1', self.cmd_vel_1_callback, 1)
        self.cmd_vel_2_sub = self.create_subscription(Twist, 'cmd_vel_2', self.cmd_vel_2_callback, 1)

        # ROS2 Outputs
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 1)

        # Periodic callback for executing FSM
        self.timer_period = 0.1 # 100 milliseconds = 10 Hz for the control loop
        self.time_unit = 1 # 1 second is the value chosen for one 'time unit'

        self.timer = self.create_timer(self.timer_period, self.control_loop)

        # Setup of robotic platform context
        self.platform_context = PrioritisedNode.PlatformContext(cmd_vel_1=TypedEvent[Twist](), cmd_vel_2=TypedEvent[Twist](), cmd_vel=TypedOutputEvent[Twist](impl=self.cmd_vel))

        # Setup the state machine itself, passing the platform_context
        self.stm = PrioritisedNodeMachine(self.get_logger(), self.timer_period, self.time_unit, context=self.platform_context)

    def cmd_vel(self, twist):
        self.get_logger().info(f"Publishing cmd_vel: linear.x={twist.linear.x}, angular.z={twist.angular.z}")
        self.publisher.publish(twist)

    def control_loop(self):

        # We step the machine
        self.get_logger().debug(f"-- Executing control_loop. --")
        self.stm.execute()

    def cmd_vel_1_callback(self, msg):
        self.platform_context.cmd_vel_1.set(msg)

        # We can step the machine outside the control_loop
        self.stm.execute(timed = False)

    def cmd_vel_2_callback(self, msg):
        self.platform_context.cmd_vel_2.set(msg)

        # We can step the machine outside the control_loop
        self.stm.execute(timed = False)


def main(args=None):

    rclpy.init(args=args)

    node = PrioritisedNode()

    rclpy.spin(node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
