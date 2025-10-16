#!/usr/bin/env python3

import rclpy
import week_4.robochart_fsm
import math

from week_4.robochart_fsm import Machine, StateMachine, Context, Clock, TypedEvent, Event, MachineStatus, Generator
from rclpy.node import Node
from geometry_msgs.msg import Twist, Vector3
from typing import Callable
from enum import Enum

from auro_interfaces.msg import Obstacle

lvel = 0.1  # Linear velocity
avel = 1.0  # Angular velocity

class Direction(Enum):
    LEFT = 0
    RIGHT = 1
    FRONT = 2

    @classmethod
    def default(cls):
        return cls.LEFT

# We define SampleTurtleBot3FSMC by inheriting from StateMachine
class SampleTurtleBot3FSMC(StateMachine):

    class MachineContext(Context):
        clock_C: Clock
        obs: Direction

    def __init__(self, logger, timer_period, time_unit, context):
        super().__init__(logger, timer_period, time_unit, SampleTurtleBot3FSMC.MachineContext(parent = context, clock_C=Clock(timer_period, time_unit), obs=Direction.default()))

    def initial(self): 
        yield from self.Driving()

    def Driving(self):

        # Composite state, therefore has its own Machine
        class Composite(Machine):

            def initial(self):
                self.logger.info("In substate 'Initial'")
                yield from self.Forward()

            def Forward(self):
                self.logger.info("In substate 'Forward'")

                # An operation defined by a state machine will have its own Python generator.
                # However, here, when we call it, we want to delegate to its generator,
                # as its execution could, in general, take a number of steps/time.
                yield from self.context.move(lvel, 0.0)

                # Need to let the parent machine know that we have entered. Here,
                # there are no further substates, otherwise they would need to
                # be entered before we did.
                yield MachineStatus.ENTERED

                exit = False

                # Monitor the transitions
                while not exit:
                    if self.context.clock_C.since() > 5:
                        self.context.clock_C.reset()
                        yield from self.Turning()
                    else:
                        # We check whether the exit flag has been set, if so we quit the loop.
                        (_, exit) = yield
                    
            def Turning(self):
                self.logger.info("In substate 'Turning'")
                yield from self.context.move(0.0, avel)

                exit = False

                while not exit:
                    if self.context.clock_C.since() > 5:
                        self.context.clock_C.reset()
                        yield from self.Forward()
                    else:
                        # We check whether the exit flag has been set, and if so we return.
                        (_, exit) = yield
                        # Need to be able to process an exit action in here by handling an input to the
                        # generator, which means likely breaking out of the while, and then exiting
                        # cleanly.

        self.state = Composite(self.logger, self.timer_period, self.time_unit, self.context)

        # No entry action
        self.logger.info("In state 'Driving'")

        # Wait for substates to enter
        yield from self.state.enter()
        
        exit = False

        while not exit:
            if self.context.obstacle.is_triggered():

                self.logger.info("Transition on 'obstacle' event triggered to junction")
                
                # Implementation of RoboChart trigger: obstacle?obs
                # Save value communicated via the event 'obstacle' into obs variable and clear event.
                self.context.obs = self.context.obstacle.value
                self.context.obstacle.clear()

                # Exit the composite state.
                self.logger.info("Exiting state 'Driving'")
                yield from self.state.exit()
                self.logger.info("Exited state 'Driving'")

                # Execute exit action, if any
                # There is none.

                # Execute the action on the transition: obstacle?obs/#C, so here we reset clock C
                self.context.clock_C.reset()
                
                # Junction is a decision point, so evaluate the guards and perform the action, calling move accordingly
                if self.context.obs == Direction.RIGHT or self.context.obs == Direction.FRONT:
                    yield from self.context.move(0.0, avel)
                elif self.context.obs == Direction.LEFT or self.context.obs == Direction.FRONT:
                    yield from self.context.move(0.0, -avel)

                # Both transitions lead to Rotating
                yield from self.Rotating()
            else:
                # Here we need to pass the value that can come into the generator into
                # the generator handling execution of the state.
                (timed, exit) = yield
                self.state.execute(timed = timed, exit = exit)

    def Rotating(self):

        self.logger.info("In state 'Rotating'")

        # No entry, during or exit actions. Just a transition, so we can ommit checking for exit in yield.
        while True:
            if self.context.clock_C.since() > math.pi/(5*avel):
                yield from self.Driving()
            else:
                yield 

class Move(StateMachine):

    # Move is not a generator, so we need to make it one using the decorator Generator, defined in robochart_fsm.py
    @Generator
    def move(self, lvel, avel):
        self.lvel = lvel
        self.avel = avel
        self.execute()

    # Initial junction leads to final state
    def initial(self):
        self.context.cmd_vel(Twist(linear=Vector3(x=self.lvel, y=0.0, z=0.0), angular=Vector3(x=0.0, y=0.0, z=self.avel)))
        yield from self.Final()

    @Generator
    def Final(self):
        return

class ForwardMachineNode(Node):

    class PlatformContext(Context):
        cmd_vel: Callable[[Twist], None]
        obstacle: TypedEvent[Obstacle]

    class MachineContext(Context):
        move: Callable

    def __init__(self):
        super().__init__('sample_forward_machine')
        
        # ROS2 Outputs
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        # ROS2 Inputs
        self.subscriber = self.create_subscription(Obstacle, 'obstacle', self.obstacle_detected, 10)
        
        # Periodic callback for executing FSM
        self.timer_period = 0.1 # 100 milliseconds = 10 Hz for the control loop
        self.time_unit = 1 # 1 second is the value chosen for one 'time unit'

        self.timer = self.create_timer(self.timer_period, self.control_loop)

        # Setup of robotic platform context
        self.platform_context = ForwardMachineNode.PlatformContext(cmd_vel=self.cmd_vel, obstacle=TypedEvent[Obstacle]())

        # Setup of move operation, defined above as a state machine
        self.move_op = Move(self.get_logger(), self.timer_period, self.time_unit, context=self.platform_context)

        # Setup of the RoboChart controller/state machine context, passing the definition of the operation move above
        self.machine_context = ForwardMachineNode.MachineContext(parent=self.platform_context, move=self.move_op.move)

        # Setup the state machine itself, passing the machine_context
        self.stm = SampleTurtleBot3FSMC(self.get_logger(), self.timer_period, self.time_unit, context=self.machine_context)

    def cmd_vel(self, twist):
        self.get_logger().info(f"Publishing cmd_vel: linear.x={twist.linear.x}, angular.z={twist.angular.z}")
        self.publisher.publish(twist)

    def obstacle_detected(self, msg):
        self.get_logger().info(f"Received message on topic 'obstacle'")
        
        # We update the event in the context, setting and passing the data,
        # mapping from a ROS message type, in this case, Obstacle,
        # to a type that corresponds to that used by the implementation
        # of the RoboChart machine above in SampleTurtleBot3FSMC.
        #
        # Note that we could have used the same ROS type instead.
        match msg.data:
            case Obstacle.LEFT:
                self.platform_context.obstacle.set(Direction.LEFT)
            case Obstacle.FRONT:
                self.platform_context.obstacle.set(Direction.FRONT)
            case Obstacle.RIGHT:
                self.platform_context.obstacle.set(Direction.RIGHT)

        # Attempt to step the machine in response to the event. This step could be
        # omitted if we only step the machine at the periods set out by the control_loop callback.
        self.stm.execute(timed = False)

    def control_loop(self):

        # We step the machine
        self.get_logger().debug(f"-- Executing control_loop. --")
        self.stm.execute()

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