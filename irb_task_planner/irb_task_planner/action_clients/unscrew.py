"""Unscrew action client."""

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from irb_interfaces.action import Unscrew  # type: ignore


class UnscrewClient:
    """Unscrew action client class."""

    def __init__(self, node: Node):
        """Initialize the action client."""
        self.node = node
        self._client = ActionClient(self.node, Unscrew, "unscrew")

        self._client.wait_for_server()
        self.future = None
        self.node.get_logger().info("Unscrew Action server is up...")

    def send_goal(self):
        """Send goal to the action server."""
        goal_msg = Unscrew.Goal()
        goal_msg.no_turns = 0
        self.future = self._client.send_goal_async(goal_msg)
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
        if result.success:
            self.node.get_logger().info("Screw unscrewed successfully")
        else:
            self.node.get_logger().info("Screw unscrew failed")

    def feedback_callback(self, feedback_msg):
        """Print feedback message."""
        self.node.get_logger().info(f"Feedback received: {feedback_msg}")


def main(args=None):
    """Unscrew action client main function."""
    rclpy.init(args=args)
    node = Node("unscrew_client")
    client = UnscrewClient(node)
    client.send_goal()
    rclpy.spin(node)


if __name__ == "__main__":
    main()
