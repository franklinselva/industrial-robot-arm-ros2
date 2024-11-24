"""Goto action client as a library."""

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from irb_interfaces.action import GoTo  # type: ignore


class GoToClient:
    """GoTo action client class."""

    def __init__(self, node: Node):
        """Initialize the action client."""
        self.node = node
        self._client = ActionClient(self.node, GoTo, "goto")

        self._client.wait_for_server()
        self.future = None
        self._goal_completed = False
        self.node.get_logger().info("GoTo Action server is up...")

    @property
    def goal_completed(self):
        """Return goal completion status."""
        return self._goal_completed

    def send_goal(self, goal_configuration: str):
        """Send goal to the action server."""
        self._goal_completed = False
        goal_msg = GoTo.Goal()
        goal_msg.goal_configuration = goal_configuration
        self.future = self._client.send_goal_async(goal_msg)
        self.future.add_done_callback(self.goal_response_callback)  # type: ignore

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
            self._goal_completed = True
            self.node.get_logger().info("Goal reached successfully")
        else:
            self.node.get_logger().info("Goal failed")

    def feedback_callback(self, feedback_msg):
        """Print feedback message."""
        self.node.get_logger().info(f"Feedback received: {feedback_msg}")


def main(args=None):
    """GoTo action client main function."""
    rclpy.init(args=args)
    node = Node("goto_client")
    client = GoToClient(node)
    goal_configuration = "detect_screws"
    client.send_goal(goal_configuration)
    rclpy.spin(node)


if __name__ == "__main__":
    main()
