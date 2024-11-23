"""Mock action server for detecting screws.
Reads a json file from the configs/ directory and returns the screw poses.
"""

import json
import time

import rclpy
from ament_index_python.packages import get_package_share_directory  # type: ignore
from geometry_msgs.msg import Pose  # type: ignore
from rclpy.action import ActionClient, ActionServer
from rclpy.node import Node

from irb_interfaces.action import DetectScrews, GoTo  # type: ignore

CONFIG_PATH = (
    get_package_share_directory("irb_action_manager") + "/config/screw_poses.json"
)


class DetectScrewsActionServer(Node):
    """Action server for detecting screws."""

    def __init__(self):
        super().__init__("detect_screws_action_server")
        self._action_server = ActionServer(
            self,
            DetectScrews,
            "detect_screws",
            self.execute_callback,
        )

        self._goto_action_client = ActionClient(self, GoTo, "goto")
        self._goto_action_client.wait_for_server()

        self.future = None

    def execute_callback(self, _):
        """Execute the action."""
        self.get_logger().info("Executing goal...")

        # Move to detect screws position
        goal = GoTo.Goal()
        goal.goal_configuration = "detect_screws"
        self.future = self._goto_action_client.send_goal_async(goal)
        self.future.add_done_callback(self.goto_response_callback)

        time.sleep(3)
        # FIXME: Potential deadlock if the goal is not accepted

        # result = self.future.result().result
        # if not result.success:
        #     self.get_logger().warn("Failed to move to detect screws position")
        #     return DetectScrews.Result()

        # Read the screw poses from the json file
        with open(CONFIG_PATH, "r", encoding="utf-8") as f:
            screw_poses = json.load(f)

        # Create the result message
        result = DetectScrews.Result()
        result.success = True
        result.screw_poses = self._process_poses(screw_poses["screws"]["poses"])

        return result

    def goto_response_callback(self, future):
        """Callback for goto response."""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info("Goal rejected")
            return

        self.get_logger().info("Goal accepted")

        goal_handle.get_result_async().add_done_callback(self.goto_result_callback)

    def goto_result_callback(self, future):
        """Callback for goto result."""
        result = future.result().result
        if result.success:
            self.get_logger().info("Goal reached successfully")
        else:
            self.get_logger().info("Goal failed")

    def feedback_callback(self, feedback_msg):
        """Print feedback message."""
        self.get_logger().info(f"Feedback received: {feedback_msg}")

    def _process_poses(self, screw_poses):
        """Process the screw poses."""
        screw_pose_msgs = []
        for screw_pose in screw_poses:
            pose = Pose()
            pose.position.x = screw_pose["position"][0]
            pose.position.y = screw_pose["position"][1]
            pose.position.z = screw_pose["position"][2]
            pose.orientation.x = screw_pose["orientation"][0]
            pose.orientation.y = screw_pose["orientation"][1]
            pose.orientation.z = screw_pose["orientation"][2]
            pose.orientation.w = screw_pose["orientation"][3]
            screw_pose_msgs.append(pose)
        return screw_pose_msgs


def main(args=None):
    """Main entry point."""

    rclpy.init(args=args)
    action_server = DetectScrewsActionServer()
    rclpy.spin(action_server)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
