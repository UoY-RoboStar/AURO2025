#!/usr/bin/env python3

# Sample solution for Week4's task 4.
#
# This Python program implements the TurtleBot3FSMC.

import rclpy
import week_4.robochart_fsm2
import math
import types


from week_4.robochart_fsm2 import Machine, StateMachine, Context, Clock, TypedEvent, Event, MachineStatus, Generator, State
from rclpy.node import Node
from geometry_msgs.msg import Twist, Vector3
from nav_msgs.msg import Odometry
from typing import Callable
from enum import Enum

from auro_interfaces.msg import Obstacle
from tf_transformations import euler_from_quaternion

lvel = 0.3  # Linear velocity
avel = 0.1  # Angular velocity

class Direction(Enum):
    LEFT = 0
    RIGHT = 1
    FRONT = 2

    @classmethod
    def default(cls):
        return cls.LEFT # Provide a default value

# We define SampleTurtleBot3FSMC by inheriting from StateMachine
class SampleTurtleBot3FSMC(StateMachine):

    class MachineContext(Context):
        clock_C: Clock
        obs: Direction

    def __init__(self, logger, timer_period, time_unit, context):
        super().__init__(logger, timer_period, time_unit, SampleTurtleBot3FSMC.MachineContext(parent = context, clock_C=Clock(timer_period, time_unit), obs=Direction.default()))

    def initial(self): 
        return self.Driving

    def Driving(self):

        # Composite state, therefore has its own StateMachine
        class Composite(StateMachine):

            def initial(self):
                self.context.clock_C.reset()
                return self.Forward

            def Forward(self):
                self.logger.info("In substate 'Forward'")

                # An operation defined by a state machine will have its own Python generator.
                # However, here, when we call it, we want to delegate to its generator,
                # as its execution could, in general, take a number of steps/time.
                yield from self.context.move(lvel, 0.0)

                # self.logger.info("In substate 'Forward' after calling move")

                # Need to let the parent machine know that we have entered. Here,
                # there are no further substates, otherwise they would need to
                # be entered before we did.
                yield MachineStatus.ENTERED

                # self.logger.info("In substate 'Forward' after entering")

                exit = False

                # Monitor the transitions
                while not exit:
                    if self.context.clock_C.since() > 5:
                        self.context.clock_C.reset()
                        return self.Turning
                    else:
                        # We check whether the exit flag has been set, if so we quit the loop.
                        # (_, exit) = yield
                        #self.logger.info("Waiting in 'Forward' for transition")
                        data = yield
                        if data is not None:
                            #self.logger.info(f"From yield: {data}")
                            _, exit = data

                #self.logger.info("Exiting 'Forward'")
                return MachineStatus.FINISHED
                    
            def Turning(self):
                self.logger.info("In substate 'Turning'")
                yield from self.context.move(0.0, avel)

                exit = False

                while not exit:
                    if self.context.clock_C.since() > 1:
                        self.context.clock_C.reset()
                        return self.Forward
                    else:
                        # We check whether the exit flag has been set, and if so we return.
                        data = yield
                        if data is not None:
                            _, exit = data
                        # Need to be able to process an exit action in here by handling an input to the
                        # generator, which means likely breaking out of the while, and then exiting
                        # cleanly.

                #self.logger.info("Exiting 'Turning'")
                return MachineStatus.FINISHED

        self.state = Composite(self.logger, self.timer_period, self.time_unit, self.context)

        # No entry action, but need to wait for substate to enter
        yield from self.state.enter()

        self.logger.info("In state 'Driving'")

        exit = False
        timed = False

        while not exit:
            if self.context.obstacle.is_triggered():

                # Implementation of RoboChart trigger: obstacle?obs
                # Save value communicated via the event 'obstacle' into obs variable and clear event.
                self.context.obs = self.context.obstacle.value
                self.context.obstacle.clear()
                self.logger.info(f"Transition on 'obstacle' event triggered to junction with value '{self.context.obs}'")

                # Exit the composite state.
                # self.logger.debug("Exiting state 'Driving'")
                yield from self.state.exit()
                # self.logger.debug("Exited state 'Driving'")

                # Execute exit action, if any. There are none in this case.

                # Execute the action on the transition: obstacle?obs/#C, so here we reset clock C
                self.context.clock_C.reset()
                
                # Junction is a decision point, so evaluate the guards and perform the action, calling move accordingly.
                #
                # Notice how here we have made the decision deterministic, whereas in the model if obs==Direction.FRONT,
                # either transition could be taken, so any is correct with respecto the model.
                #
                if self.context.obs == Direction.RIGHT or self.context.obs == Direction.FRONT:
                    yield from self.context.move(0.0, avel)
                elif self.context.obs == Direction.LEFT or self.context.obs == Direction.FRONT:
                    yield from self.context.move(0.0, -avel)

                # self.logger.info("Going to state rotating")
                # Both transitions lead to Rotating
                return self.Rotating
            else:
                # Here we need to pass the value that can come into the generator into
                # the generator handling execution of the state.

                 # First, execute substate
                self.state.execute(timed = timed, exit = exit)

                # Then yield back control
                data = yield
                if data is not None:
                    timed, exit = data

    def Rotating(self):

        self.logger.info(f"In state 'Rotating'")

        # No entry, during or exit actions. Just a transition with no guards.
        # Need to continue executing rotate till it finishes, so yield from it.
        yield from self.context.rotate(math.pi/5)

        return self.Driving

class Rotate(Machine):

    class MachineContext(Context):
        init: float
        current: float

    def __init__(self, logger, timer_period, time_unit, context):
        super().__init__(logger, timer_period, time_unit, Rotate.MachineContext(parent = context, init = 0.0, current = 0.0))

    # This pattern of sharing the generator with the callee only works for
    # use of this operation in an action other than a during action!

    def rotate(self, angle):
        self.angle = angle
        yield from self.execute() 
       
    # Initial junction
    def initial(self):
        
        # Execute transition action that waits on input yaw?current
        while not self.context.yaw.is_triggered():
            yield

        self.context.init = self.context.yaw.value
        self.context.yaw.clear()

        # This finishes the generator, despite there being more work to be done.
        return self.s0

    # See below for an alternative using a class decorator that encapsulates
    # the control flow for yielding when no transitions are enabled.
    def s0(self):
        
        while True:
            if self.context.yaw.is_triggered():
                self.context.current = self.context.yaw.value
                self.context.yaw.clear()

                # Transitions out of junction
                if self.angle - abs(((self.context.current-self.context.init) + math.pi) % (2 * math.pi) - math.pi) <= 0.5:                
                    return self.Final
                else:
                    return self.s0
            else:
                yield

    # @State
    # def s0(self):

    #     if self.context.yaw.is_triggered():
    #         self.context.current = self.context.yaw.value
    #         self.context.yaw.clear()

    #         # Transitions out of junction
    #         if self.angle - abs(((self.context.current-self.context.init) + math.pi) % (2 * math.pi) - math.pi) <= 0.5:                
    #             return self.Final
    #         else:
    #             return self.s0

    def Final(self):
        return

class Move(Machine):

    def move(self, lvel, avel):
        self.lvel = lvel
        self.avel = avel
        yield from self.execute()

    # Initial junction leads to final state
    def initial(self):
        self.context.cmd_vel(Twist(linear=Vector3(x=self.lvel, y=0.0, z=0.0), angular=Vector3(x=0.0, y=0.0, z=self.avel)))
        return self.Final

    def Final(self):
        return

class TurtleBot3FSMCNode(Node):

    class PlatformContext(Context):
        cmd_vel: Callable[[Twist], None]
        yaw: TypedEvent[float]
        obstacle: TypedEvent[Obstacle]

    class MachineContext(Context):
        move: Callable
        rotate: Callable

    def __init__(self):
        super().__init__('TurtleBot3FSMC')
        
        # ROS2 Outputs
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 1)

        # ROS2 Inputs
        self.subscriber = self.create_subscription(Obstacle, 'obstacle', self.obstacle_detected, 1)
        self.odom_subscriber = self.create_subscription(Odometry, 'odom', self.odom_callback, 1)
        
        # Periodic callback for executing FSM
        self.timer_period = 0.1 # 100 milliseconds = 10 Hz for the control loop
        self.time_unit = 1 # 1 second is the value chosen for one 'time unit'

        self.timer = self.create_timer(self.timer_period, self.control_loop)

        # Setup of robotic platform context
        self.platform_context = TurtleBot3FSMCNode.PlatformContext(cmd_vel=self.cmd_vel, obstacle=TypedEvent[Obstacle](), yaw=TypedEvent[float]())

        # Setup of move operation, defined above as a state machine
        self.move_op = Move(self.get_logger(), self.timer_period, self.time_unit, context=self.platform_context)

        # Setup of rotate operation, defined above as a state machine
        self.rotate_op = Rotate(self.get_logger(), self.timer_period, self.time_unit, context=self.platform_context)

        # Setup of the RoboChart controller/state machine context, passing the definition of the operation move above
        self.machine_context = TurtleBot3FSMCNode.MachineContext(parent=self.platform_context, move=self.move_op.move, rotate=self.rotate_op.rotate)

        # Setup the state machine itself, passing the machine_context
        self.stm = SampleTurtleBot3FSMC(self.get_logger(), self.timer_period, self.time_unit, context=self.machine_context)

        self.iter = None

    def cmd_vel(self, twist):
        self.get_logger().info(f"Publishing cmd_vel: linear.x={twist.linear.x}, angular.z={twist.angular.z}")
        self.publisher.publish(twist)

    def obstacle_detected(self, msg):
        self.get_logger().debug(f"Received message on topic 'obstacle'")
        
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

    def odom_callback(self, msg):

        self.get_logger().debug(f"Received message on topic 'odom'")

        q = [msg.pose.pose.orientation.x,
             msg.pose.pose.orientation.y,
             msg.pose.pose.orientation.z,
             msg.pose.pose.orientation.w]
        
        (_, _, yaw) = euler_from_quaternion(q)

        # Convert to RoboChar type 'real', here float in the Python implementation.
        self.platform_context.yaw.set(yaw)

        # Attempt to step the machine in response to the event. This step could be
        # omitted if we only step the machine at the periods set out by the control_loop callback.
        self.stm.execute(timed = False)

    def control_loop(self):

        # We step the machine
        self.get_logger().debug(f"-- Executing control_loop. --")
        self.stm.execute()
        self.get_logger().debug(f"-- End control_loop --")

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