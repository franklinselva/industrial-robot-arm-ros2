"""Change tool action client as a library."""

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from irb_interfaces.action import ChangeTool  # type: ignore


class ChangeToolClient:
    """Change tool action client class."""

    def __init__(self, node: Node):
        """Initialize the action client."""
        self._node = node
        self._client = ActionClient(self._node, ChangeTool, "change_tool")

        self._client.wait_for_server()
        self.future = None
        self._node.get_logger().info("Change Tool Action server is up...")

    def send_goal(self):
        """Send goal to the action server."""
        goal_msg = ChangeTool.Goal()
        goal_msg.tool_size = 0
        goal_msg.tool_type = 0
        self.future = self._client.send_goal_async(goal_msg)
        self.future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        """Callback for goal response."""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self._node.get_logger().info("Goal rejected")
            return

        self._node.get_logger().info("Goal accepted")

        goal_handle.get_result_async().add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        """Callback for goal result."""
        result = future.result().result
        if result.success:
            self._node.get_logger().info("Tool changed successfully")
        else:
            self._node.get_logger().info("Tool change failed")

    def feedback_callback(self, feedback_msg):
        """Print feedback message."""
        self._node.get_logger().info(f"Feedback received: {feedback_msg}")


def main(args=None):
    """Change tool action client main function."""
    rclpy.init(args=args)
    node = Node("change_tool_client")
    client = ChangeToolClient(node)
    client.send_goal()
    rclpy.spin(node)


if __name__ == "__main__":
    main()
