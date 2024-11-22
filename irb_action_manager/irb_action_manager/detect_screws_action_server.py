"""Mock action server for detecting screws.
Reads a json file from the configs/ directory and returns the screw poses.
"""

import json

import rclpy
from ament_index_python.packages import get_package_share_directory  # type: ignore
from geometry_msgs.msg import Pose  # type: ignore
from rclpy.action import ActionServer
from rclpy.node import Node

from irb_interfaces.action import DetectScrews  # type: ignore

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

    def execute_callback(self, goal_handle):
        """Execute the action."""
        self.get_logger().info("Executing goal...")

        # Read the screw poses from the json file
        with open(CONFIG_PATH, "r", encoding="utf-8") as f:
            screw_poses = json.load(f)

        # Create the result message
        result = DetectScrews.Result()
        result.screw_poses = [Pose(**pose) for pose in screw_poses]

        # Return the result
        goal_handle.succeed(result)


def main(args=None):
    """Main entry point."""

    rclpy.init(args=args)
    action_server = DetectScrewsActionServer()
    rclpy.spin(action_server)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
