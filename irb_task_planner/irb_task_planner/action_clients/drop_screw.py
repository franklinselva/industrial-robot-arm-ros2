"""Drop screw action client."""

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from irb_interfaces.action import DropScrew  # type: ignore


class DropScrewClient:
    """DropScrew action client class."""

    def __init__(self, node: Node):
        """Initialize the action client."""
        self.node = node
        self._client = ActionClient(self.node, DropScrew, "drop_screw")

        self._client.wait_for_server()
        self.node.get_logger().info("DropScrew Action server is up...")

    def send_goal(self):
        """Send goal to the action server."""
        goal_msg = DropScrew.Goal()

        future = self._client.send_goal_async(goal_msg)

        rclpy.spin_until_future_complete(self.node, future)
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.node.get_logger().info("Goal rejected")
            return DropScrew.Result()

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self.node, result_future)

        result = result_future.result().result
        return result


def main(args=None):
    """DropScrew action client main function."""
    rclpy.init(args=args)
    node = Node("drop_screw_client")
    client = DropScrewClient(node)
    print(client.send_goal())


if __name__ == "__main__":
    main()
