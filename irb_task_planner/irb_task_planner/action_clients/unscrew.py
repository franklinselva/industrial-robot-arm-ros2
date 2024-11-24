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
        self.node.get_logger().info("Unscrew Action server is up...")

    def send_goal(self):
        """Send goal to the action server."""
        goal_msg = Unscrew.Goal()
        goal_msg.no_turns = 0
        future = self._client.send_goal_async(goal_msg)

        rclpy.spin_until_future_complete(self.node, future)
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.node.get_logger().info("Goal rejected")
            return Unscrew.Result()

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self.node, result_future)

        result = result_future.result().result
        return result


def main(args=None):
    """Unscrew action client main function."""
    rclpy.init(args=args)
    node = Node("unscrew_client")
    client = UnscrewClient(node)
    client.send_goal()
    rclpy.spin(node)


if __name__ == "__main__":
    main()
