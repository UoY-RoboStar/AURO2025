#!/usr/bin/env python3

import rclpy
import week_4.robochart_fsm2
from week_4.robochart_fsm2 import StateMachine, Machine, Context, Clock
from rclpy.node import Node
from geometry_msgs.msg import Twist, Vector3
from typing import Callable

lvel = 0.1  # Linear velocity
avel = 0.5  # Angular velocity

class TurtleBot3FSMC(StateMachine):

    class MachineContext(Context):
        clock_C: Clock

    def __init__(self, logger, timer_period, time_unit, context):
        super().__init__(logger, timer_period, time_unit, TurtleBot3FSMC.MachineContext(parent = context, clock_C=Clock(timer_period, time_unit)))

    def initial(self): 
        return self.Forward

    def Forward(self):
        
        # Entry action
        self.context.cmd_vel(Twist(linear=Vector3(x=lvel, y=0.0, z=0.0), angular=Vector3(x=0.0, y=0.0, z=0.0)))
        self.logger.info("In state 'FORWARD'")

        # Example of using a clock to trigger a transition after 4 time units:        
        while True:
            if self.context.clock_C.since() >= self.context.T:
                self.logger.info("Transition from 'FORWARD' to 'TURNING")
                self.context.clock_C.reset()
                return self.Turning
            else:
                yield

    def Turning(self):

        # Entry action
        self.context.cmd_vel(Twist(linear=Vector3(x=0.0, y=0.0, z=0.0), angular=Vector3(x=0.0, y=0.0, z=avel)))
        self.logger.info("In state 'TURNING'")

        # Check transitions out of state Turning
        while True:
            if self.context.clock_C.since() > self.context.F:
                self.logger.info("Transition from 'TURNING' to 'FORWARD")
                # Transition's action
                self.context.clock_C.reset()
                return self.Forward
            else:
                yield

class Move(Machine):

    def move(self, lvel, avel):
        self.lvel = lvel
        self.avel = avel
        yield from self.execute()

    def initial(self):
        self.context.cmd_vel(Twist(linear=Vector3(x=self.lvel, y=0.0, z=0.0), angular=Vector3(x=0.0, y=0.0, z=self.avel)))
        return self.Final

    def Final(self):
        return

class TurtleBot3FSMCNode(Node):

    class PlatformContext(Context):
        cmd_vel: Callable[[Twist], None]

    class MachineContext(Context):
        move: Callable
        F: int
        T: int

    def __init__(self):
        super().__init__('TurtleBot3FSMC')

        # ROS2 Outputs
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 1)

        # Periodic callback for executing FSM
        self.timer_period = 0.1 # 100 milliseconds = 10 Hz for the control loop
        self.time_unit = 1 # 1 second is the value chosen for one 'time unit'

        self.timer = self.create_timer(self.timer_period, self.control_loop)

        # Setup of robotic platform context
        self.platform_context = TurtleBot3FSMCNode.PlatformContext(cmd_vel=self.cmd_vel)

        # Setup of move operation, defined above as a state machine
        self.move_op = Move(self.get_logger(), self.timer_period, self.time_unit, context=self.platform_context)

        # Setup of the RoboChart controller/state machine context, passing the definition of the operation move above
        self.machine_context = TurtleBot3FSMCNode.MachineContext(parent=self.platform_context, move=self.move_op.move, F=0, T=0)

        self.machine_context.F = int(self.declare_parameter('F', 1).value)
        self.machine_context.T = int(self.declare_parameter('T', 4).value)

        # Setup the state machine itself, passing the machine_context
        self.stm = TurtleBot3FSMC(self.get_logger(), self.timer_period, self.time_unit, context=self.machine_context)

    def cmd_vel(self, twist):
        self.get_logger().info(f"Publishing cmd_vel: linear.x={twist.linear.x}, angular.z={twist.angular.z}")
        self.publisher.publish(twist)

    def control_loop(self):

        # We step the machine
        self.get_logger().debug(f"-- Executing control_loop. --")
        self.stm.execute()

def main(args=None):

    rclpy.init(args=args)

    node = TurtleBot3FSMCNode()

    rclpy.spin(node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()