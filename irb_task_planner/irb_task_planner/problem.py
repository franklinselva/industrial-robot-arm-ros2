""" Problem definition for unscrew task planning.

A simple representation of the task planning for unscrewing screws.
"""

import time
from typing import List, Optional

from up_esb.bridge import Bridge  # type: ignore

from irb_task_planner.action_clients import (
    ChangeToolClient,
    DetectScrewClient,
    DropScrewClient,
    GoToClient,
    UnscrewClient,
)


####################################################################################################
## Object representation
####################################################################################################
class Pose:
    """Pose representation."""

    def __init__(self, name):
        self.name = name

    def __repr__(self) -> str:
        return self.name

    def __eq__(self, other):
        return self.name == other.name


####################################################################################################
## Robot arm representation
####################################################################################################
class RobotArm:
    """Robot arm representation."""

    pose = Pose("home")
    has_screw = False
    are_screws_detected = False
    has_screwdriver = False
    screws: List[Pose] = []
    unscrews: List[Pose] = []

    change_tool_client: Optional[ChangeToolClient] = None
    detect_screw_client: Optional[DetectScrewClient] = None
    drop_screw_client: Optional[DropScrewClient] = None
    goto_client: Optional[GoToClient] = None
    unscrew_client: Optional[UnscrewClient] = None

    @classmethod
    def set_change_tool_client(cls, client):
        """Set change tool client."""
        RobotArm.change_tool_client = client

    @classmethod
    def set_detect_screw_client(cls, client):
        """Set detect screw client."""
        RobotArm.detect_screw_client = client

    @classmethod
    def set_drop_screw_client(cls, client):
        """Set drop screw client."""
        RobotArm.drop_screw_client = client

    @classmethod
    def set_goto_client(cls, client):
        """Set goto client."""
        RobotArm.goto_client = client

    @classmethod
    def set_unscrew_client(cls, client):
        """Set unscrew client."""
        RobotArm.unscrew_client = client

    @classmethod
    def goto(cls, pose_from: Pose, pose_to: Pose):
        """Move robot arm to a pose."""
        if pose_from != cls.pose:
            return False

        if cls.goto_client is None:
            raise ChildProcessError("Goto client is not set")

        print(f"Move from {pose_from} to {pose_to}")
        result = cls.goto_client.send_goal(pose_to.name)

        if result.success:
            cls.pose = pose_to
            return True

        return False

    @classmethod
    def detect_screw(cls):
        """Detect screw."""
        cls.screws = [
            Pose("screw_pose_1"),
            Pose("screw_pose_2"),
            Pose("screw_pose_3"),
            Pose("screw_pose_4"),
            Pose("screw_pose_5"),
        ]

        if cls.detect_screw_client is None:
            raise ChildProcessError("Detect screw client is not set")

        print("Detecting screws")
        result = cls.detect_screw_client.send_goal()

        if result.success:
            cls.are_screws_detected = True
            return True

        return False

    @classmethod
    def change_tool(cls):
        """Change tool."""

        if cls.change_tool_client is None:
            raise ChildProcessError("Change tool client is not set")

        print("Changing to screwdriver")
        result = cls.change_tool_client.send_goal()

        if result.success:
            cls.has_screwdriver = True
            return True
        return False

    @classmethod
    def unscrew(cls, screw: Pose):
        """Unscrew."""
        if cls.unscrew_client is None:
            raise ChildProcessError("Unscrew client is not set")

        print(f"Unscrewing {screw}")
        time.sleep(5)

        # Drop screw
        result = cls.drop_screw_client.send_goal()  # type: ignore

        if result.success:
            cls.unscrews.append(screw)
            return True

        return False


####################################################################################################
## Fluent functions
####################################################################################################
def robot_at_fn(pose: Pose):
    """Check if robot is at a pose."""
    return RobotArm.pose == pose


def are_screws_detected_fn():
    """Check if screws are detected."""
    return RobotArm.are_screws_detected


def is_unscrewed_fn(pose: Pose):
    """Check if screw is unscrewed."""
    return pose in RobotArm.unscrews


def has_screwdriver_fn():
    """Check if screwdriver is available."""
    return RobotArm.has_screwdriver


def has_no_tool_fn():
    """Check if screwdriver is available."""
    return not RobotArm.has_screwdriver


def has_screw_at_tool_fn():
    """Check if screw is at tool."""
    return RobotArm.has_screw


####################################################################################################
## Problem definition
####################################################################################################
def define_problem():
    """UP Problem representation."""

    bridge = Bridge()
    bridge.create_types([Pose, RobotArm])

    robot_at = bridge.create_fluent_from_function(robot_at_fn)
    are_screws_detected = bridge.create_fluent_from_function(are_screws_detected_fn)
    is_unscrewed = bridge.create_fluent_from_function(is_unscrewed_fn)
    has_screwdriver = bridge.create_fluent_from_function(has_screwdriver_fn)
    has_no_tool = bridge.create_fluent_from_function(has_no_tool_fn)
    has_screw_at_tool = bridge.create_fluent_from_function(has_screw_at_tool_fn)

    home_pose = bridge.create_object("home", Pose("home"))
    detect_screws_pose = bridge.create_object("detect_screws", Pose("detect_screws"))
    change_tool_pose = bridge.create_object("change_tool", Pose("change_tool"))
    drop_screw_pose = bridge.create_object("drop_screw", Pose("drop_screw"))
    screw1_pose = bridge.create_object("screw_pose_1", Pose("screw_pose_1"))
    screw2_pose = bridge.create_object("screw_pose_2", Pose("screw_pose_2"))
    screw3_pose = bridge.create_object("screw_pose_3", Pose("screw_pose_3"))
    screw4_pose = bridge.create_object("screw_pose_4", Pose("screw_pose_4"))
    screw5_pose = bridge.create_object("screw_pose_5", Pose("screw_pose_5"))

    goto, [pose_from, pose_to] = bridge.create_action(
        "Goto", _callable=RobotArm.goto, pose_from=Pose, pose_to=Pose
    )
    goto.add_precondition(robot_at(pose_from))
    goto.add_effect(robot_at(pose_to), True)
    goto.add_effect(robot_at(pose_from), False)

    detect_screw, _ = bridge.create_action(
        "DetectScrew", _callable=RobotArm.detect_screw
    )
    detect_screw.add_precondition(robot_at(detect_screws_pose))
    detect_screw.add_effect(are_screws_detected, True)

    change_tool, _ = bridge.create_action("ChangeTool", _callable=RobotArm.change_tool)
    change_tool.add_precondition(robot_at(change_tool_pose))
    change_tool.add_precondition(has_no_tool)
    change_tool.add_effect(has_screwdriver, True)

    unscrew, [screw] = bridge.create_action(
        "Unscrew", _callable=RobotArm.unscrew, screw=Pose
    )
    unscrew.add_precondition(robot_at(screw))
    unscrew.add_precondition(has_screwdriver)
    unscrew.add_precondition(are_screws_detected)
    unscrew.add_effect(is_unscrewed(screw), True)
    unscrew.add_effect(has_screw_at_tool, True)

    problem = bridge.define_problem()
    problem.set_initial_value(robot_at(home_pose), True)
    problem.set_initial_value(robot_at(detect_screws_pose), False)
    problem.set_initial_value(robot_at(change_tool_pose), False)
    problem.set_initial_value(robot_at(drop_screw_pose), False)
    problem.set_initial_value(robot_at(screw1_pose), False)
    problem.set_initial_value(robot_at(screw2_pose), False)
    problem.set_initial_value(robot_at(screw3_pose), False)
    problem.set_initial_value(robot_at(screw4_pose), False)
    problem.set_initial_value(robot_at(screw5_pose), False)
    problem.set_initial_value(are_screws_detected, False)
    problem.set_initial_value(is_unscrewed(screw1_pose), False)
    problem.set_initial_value(is_unscrewed(screw2_pose), False)
    problem.set_initial_value(is_unscrewed(screw3_pose), False)
    problem.set_initial_value(is_unscrewed(screw4_pose), False)
    problem.set_initial_value(is_unscrewed(screw5_pose), False)
    problem.set_initial_value(is_unscrewed(home_pose), True)
    problem.set_initial_value(is_unscrewed(detect_screws_pose), False)
    problem.set_initial_value(is_unscrewed(change_tool_pose), False)
    problem.set_initial_value(is_unscrewed(drop_screw_pose), False)
    problem.set_initial_value(has_screwdriver, False)
    problem.set_initial_value(has_no_tool, True)

    problem.set_initial_value(has_screwdriver, False)
    problem.set_initial_value(has_screw_at_tool, False)

    problem.add_goal(is_unscrewed(screw1_pose))
    problem.add_goal(is_unscrewed(screw2_pose))
    problem.add_goal(is_unscrewed(screw3_pose))
    problem.add_goal(is_unscrewed(screw4_pose))
    problem.add_goal(is_unscrewed(screw5_pose))
    problem.add_goal(robot_at(home_pose))

    return problem, bridge
