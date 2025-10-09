
import rclpy
import time
from rclpy.node import Node
from rclpy.logging import get_logger
from moveit.planning import (MoveItPy,MultiPipelinePlanRequestParameters)
from moveit.core.planning_scene import PlanningScene
from geometry_msgs.msg import Pose

def plan_and_execute(
    robot,
    planning_component,
    logger,
    single_plan_parameters=None,
    multi_plan_parameters=None,
    sleep_time=0.0,
):
    """Helper function to plan and execute a motion."""
    # plan to goal
    logger.info("Planning trajectory")
    if multi_plan_parameters is not None:
        plan_result = planning_component.plan(
            multi_plan_parameters=multi_plan_parameters
        )
    elif single_plan_parameters is not None:
        plan_result = planning_component.plan(
            single_plan_parameters=single_plan_parameters
        )
    else:
        plan_result = planning_component.plan()

    # execute the plan
    if plan_result:
        logger.info("Executing plan")
        robot_trajectory = plan_result.trajectory
        robot.execute(robot_trajectory, controllers=[])
    else:
        logger.error("Planning failed")

    time.sleep(sleep_time)


def main():
    rclpy.init()
    logger = get_logger("moveit_example")
    node = rclpy.create_node('moveit_example')

    # Initialize MoveItPy
    panda = MoveItPy(node_name="moveit_py")
    panda_arm = panda.get_planning_component("panda_arm")
    logger.info("MoveItPy instance created")

    # set plan start state to current state
    panda_arm.set_start_state_to_current_state()

    # set pose goal with PoseStamped message
    from geometry_msgs.msg import PoseStamped

    pose_goal = PoseStamped()
    pose_goal.header.frame_id = "panda_link0"
    pose_goal.pose.orientation.w = 1.0
    pose_goal.pose.position.x = 0.28
    pose_goal.pose.position.y = -0.2
    pose_goal.pose.position.z = 1.0
    panda_arm.set_goal_state(pose_stamped_msg=pose_goal, pose_link="panda_link8")

    time.sleep(5.0)

    # plan to goal
    plan_and_execute(panda, panda_arm, logger, sleep_time=3.0)

if __name__ == '__main__':
    main()
