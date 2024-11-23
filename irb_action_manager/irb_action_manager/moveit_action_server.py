#!/usr/bin/env python3
"""Action server for the GoTo action."""
import time
from threading import Lock

import rclpy
from moveit.planning import MoveItPy
from rclpy.action import ActionServer
from rclpy.node import Node

from irb_interfaces.action import DropScrew, GoTo  # type: ignore


def plan_and_execute(
    robot,
    planning_component,
    logger,
    sleep_time=0.0,
):
    """Helper function to plan and execute a motion."""
    # plan to goal
    logger.info("Planning trajectory")
    plan_result = planning_component.plan()

    # execute the plan
    if plan_result:
        logger.info("Executing plan")
        robot_trajectory = plan_result.trajectory
        robot.execute(robot_trajectory, controllers=[])

        return True
    else:
        logger.error("Planning failed")

        return False

    time.sleep(sleep_time)


class GoToActionServer(Node):
    """Action server for the GoTo action."""

    def __init__(self):
        super().__init__("goto_action_server")
        self.logger = self.get_logger()

        self.irb = MoveItPy(node_name="goto_action_server")
        self.arm = self.irb.get_planning_component("arm")
        self.logger.info("Arm component loaded")
        self.action_lock = Lock()

        self.arm.set_start_state("home")

        self.action_server_1 = ActionServer(
            self, GoTo, "goto", self.goto_execute_callback
        )

        self.action_server_2 = ActionServer(
            self, DropScrew, "drop_screw", self.drop_screw_execute_callback
        )

        self.arm.set_start_state("home")

    def goto_execute_callback(self, goal_handle):
        """GoTo action server callback."""
        with self.action_lock:
            return self._goto_execute_callback(goal_handle)

    def _goto_execute_callback(self, goal_handle):
        """GoTo action server callback."""

        self.logger.info("Executing goal for GoTo action server")

        self.arm.set_start_state_to_current_state()

        goal = goal_handle.request.goal_pose
        self.arm.set_goal_state(pose_stamped_msg=goal, pose_link="screw_driver")

        result = GoTo.Result()
        result.success = plan_and_execute(self.irb, self.arm, self.logger)

        return result

    def drop_screw_execute_callback(self, goal_handle):
        """DropScrew action server callback."""
        with self.action_lock:
            return self._drop_screw_execute_callback(goal_handle)

    def _drop_screw_execute_callback(self, goal_handle):
        """DropScrew action server callback."""
        self.logger.info("Executing goal for DropScrew action server")

        # TODO: Add logic to drop the screw through simple way points

        self.arm.set_start_state_to_current_state()

        goal = goal_handle.request.goal_pose
        self.arm.set_goal_state(pose_stamped_msg=goal, pose_link="screw_driver")

        result = GoTo.Result()
        result.success = plan_and_execute(self.irb, self.arm, self.logger)

        return result


def main():
    """Main function."""
    rclpy.init()
    action_server = GoToActionServer()
    rclpy.spin(action_server)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
