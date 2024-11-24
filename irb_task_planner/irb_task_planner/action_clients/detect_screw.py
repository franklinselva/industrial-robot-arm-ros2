"""Detect screw action client as a module."""

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from irb_interfaces.action import DetectScrews  # type: ignore


class DetectScrewClient:
    """Action client for detecting screws."""

    def __init__(self, node: Node):
        """Initialize the action client."""
        self.node = node
        self._action_client = ActionClient(self.node, DetectScrews, "detect_screws")

        self._action_client.wait_for_server()
        self.future = None
        self.node.get_logger().info("Detect Screw Action server is up...")

    def send_goal(self):
        """Send the goal to the server."""
        self.node.get_logger().info("Sending goal...")

        goal_msg = DetectScrews.Goal()
        goal_msg.no_screws = 5

        self.future = self._action_client.send_goal_async(goal_msg)
        self.future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        """Callback for goal response."""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.node.get_logger().info("Goal rejected")
            return

        self.node.get_logger().info("Goal accepted")

        goal_handle.get_result_async().add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        """Callback for goal result."""
        result = future.result().result
        self.node.get_logger().info(f"Result: {result}")

    def feedback_callback(self, feedback_msg):
        """Print feedback message."""
        self.node.get_logger().info(f"Feedback received: {feedback_msg}")


def main(args=None):
    """Detect screw action client main function."""
    rclpy.init(args=args)
    node = Node("detect_screw_client")
    client = DetectScrewClient(node)
    client.send_goal()
    rclpy.spin(node)


if __name__ == "__main__":
    main()
