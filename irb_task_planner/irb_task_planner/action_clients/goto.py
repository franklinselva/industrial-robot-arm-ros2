"""Goto action client as a library."""

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from irb_interfaces.action import GoTo  # type: ignore


class GoToClient(Node):
    """GoTo action client class."""

    def __init__(self):
        """Initialize the action client."""
        super().__init__("goto_client")
        self._client = ActionClient(self, GoTo, "goto")

        self._client.wait_for_server()
        self.future = None
        self.get_logger().info("GoTo Action server is up...")

    def send_goal(self, goal_configuration: str):
        """Send goal to the action server."""
        goal_msg = GoTo.Goal()
        goal_msg.goal_configuration = goal_configuration
        self.future = self._client.send_goal_async(goal_msg)
        self.future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        """Callback for goal response."""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info("Goal rejected")
            return

        self.get_logger().info("Goal accepted")

        goal_handle.get_result_async().add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        """Callback for goal result."""
        result = future.result().result
        if result.success:
            self.get_logger().info("Goal reached successfully")
        else:
            self.get_logger().info("Goal failed")

    def feedback_callback(self, feedback_msg):
        """Print feedback message."""
        self.get_logger().info(f"Feedback received: {feedback_msg}")


def main(args=None):
    """GoTo action client main function."""
    rclpy.init(args=args)
    client = GoToClient()
    goal_configuration = "detect_screws"
    client.send_goal(goal_configuration)
    rclpy.spin(client)


if __name__ == "__main__":
    main()
