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
        self._node.get_logger().info("Change Tool Action server is up...")

    def send_goal(self):
        """Send goal to the action server."""
        goal_msg = ChangeTool.Goal()
        goal_msg.tool_size = 0
        goal_msg.tool_type = 0
        future = self._client.send_goal_async(goal_msg)

        rclpy.spin_until_future_complete(self._node, future)
        goal_handle = future.result()

        if not goal_handle.accepted:
            self._node.get_logger().info("Goal rejected")
            return ChangeTool.Result()

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self._node, result_future)
        result = result_future.result().result

        return result


def main(args=None):
    """Change tool action client main function."""
    rclpy.init(args=args)
    node = Node("change_tool_client")
    client = ChangeToolClient(node)
    print(client.send_goal())


if __name__ == "__main__":
    main()
