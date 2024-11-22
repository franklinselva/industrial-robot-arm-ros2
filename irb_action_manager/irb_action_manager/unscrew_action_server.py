"""Unscrew action server.

Mock action server for unscrewing screws.

Waits for a request, sleeps for 5 seconds, and returns a success response.
"""

import time

import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node

from irb_interfaces.action import Unscrew  # type: ignore


class UnscrewActionServer(Node):
    """Action server for unscrewing screws."""

    def __init__(self):
        super().__init__("unscrew_action_server")
        self._action_server = ActionServer(
            self,
            Unscrew,
            "unscrew",
            self.execute_callback,
        )

    def execute_callback(self, _):
        """Execute the action."""
        self.get_logger().info("Executing goal...")

        # Sleep for 5 seconds
        time.sleep(5)

        # Return the result
        result = Unscrew.Result()
        result.success = True

        return result


def main(args=None):
    """Main entry point."""

    rclpy.init(args=args)
    action_server = UnscrewActionServer()
    rclpy.spin(action_server)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
