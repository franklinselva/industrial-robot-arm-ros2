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
        self.node.get_logger().info("Detect Screw Action server is up...")

    def send_goal(self):
        """Send the goal to the server."""
        self.node.get_logger().info("Sending goal...")

        goal_msg = DetectScrews.Goal()
        goal_msg.no_screws = 5

        future = self._action_client.send_goal_async(goal_msg)

        rclpy.spin_until_future_complete(self.node, future)
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.node.get_logger().info("Goal rejected")
            return DetectScrews.Result()

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self.node, result_future)

        result = result_future.result().result
        return result


def main(args=None):
    """Detect screw action client main function."""
    rclpy.init(args=args)
    node = Node("detect_screw_client")
    client = DetectScrewClient(node)
    print(client.send_goal())


if __name__ == "__main__":
    main()
