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
        self._result = None
        self.node.get_logger().info("GoTo Action server is up...")

    def send_goal(self, goal_configuration: str):
        """Send goal to the action server."""
        self._result = None
        goal_msg = GoTo.Goal()
        goal_msg.goal_configuration = goal_configuration
        future = self._client.send_goal_async(goal_msg)

        rclpy.spin_until_future_complete(self.node, future)
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.node.get_logger().info("Goal rejected")
            return GoTo.Result()

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self.node, result_future)
        self._result = result_future.result().result

        return self._result


def main(args=None):
    """GoTo action client main function."""
    rclpy.init(args=args)
    node = Node("goto_client")
    client = GoToClient(node)
    goal_configuration = "detect_screws"
    print(client.send_goal(goal_configuration))


if __name__ == "__main__":
    main()
