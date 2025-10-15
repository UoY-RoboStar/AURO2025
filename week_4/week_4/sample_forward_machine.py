#!/usr/bin/env python3

import rclpy
import week_4.robochart_fsm
from week_4.robochart_fsm import Machine, Context, Clock
from rclpy.node import Node
from geometry_msgs.msg import Twist, Vector3
from typing import Callable

lvel = 0.1  # Linear velocity
avel = 0.5  # Angular velocity

class SampleForwardMachine(Machine):

    class MachineContext(Context):
        clock_C: Clock

    def __init__(self, logger, timer_period, time_unit, context):
        super().__init__(logger, timer_period, time_unit, SampleForwardMachine.MachineContext(parent = context, clock_C=Clock(timer_period, time_unit)))

    def initial(self): yield from self.Forward()

    def Forward(self):
        
        # Entry action
        self.context.cmd_vel(Twist(linear=Vector3(x=lvel, y=0.0, z=0.0), angular=Vector3(x=0.0, y=0.0, z=0.0)))
        self.logger.info("In state 'FORWARD'")
        
        # Because there are no transitions out of this state, we stay here forever
        while True:
            yield

        # Example of using a clock to trigger a transition after 4 time units:
        #
        # while True:
        #     if self.context.clock_C.since() > 4:
        #         self.context.clock_C.reset()
        #         yield from TargetState()
        #     else:
        #         yield

class ForwardMachineNode(Node):

    class NodeContext(Context):
        cmd_vel: Callable[[Twist], None]

    def __init__(self):
        super().__init__('sample_forward_machine')
        
        # Output
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Periodic callback for executing FSM
        self.timer_period = 0.1 # 100 milliseconds = 10 Hz for the control loop
        self.time_unit = 1 # 1 second is the value chosen for one 'time unit'

        self.timer = self.create_timer(self.timer_period, self.control_loop)
        self.exec = None
        self.stm = SampleForwardMachine(self.get_logger(), self.timer_period, self.time_unit, context=ForwardMachineNode.NodeContext(cmd_vel=self.cmd_vel))

    def cmd_vel(self, twist):
        self.get_logger().info(f"Publishing cmd_vel: linear.x={twist.linear.x}, angular.z={twist.angular.z}")
        self.publisher.publish(twist)

    def control_loop(self):
        self.stm.execute()
        # try:
        #     if self.exec is None:
        #         self.exec = self.stm.execute()
        #     else:
        #         next(self.exec)
        # except StopIteration:
        #     pass

def main(args=None):

    rclpy.init(args=args)

    node = ForwardMachineNode()

    rclpy.spin(node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()