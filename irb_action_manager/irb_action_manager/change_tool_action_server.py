"""Change tool action server.

Mock action server for changing the tool of the robot.

waits for a request, sleeps for 5 seconds, and returns a success response.
"""

import time

import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node

from irb_interfaces.action import ChangeTool  # type: ignore


class ChangeToolActionServer(Node):
    """Action server for changing the tool."""

    def __init__(self):
        super().__init__("change_tool_action_server")
        self._action_server = ActionServer(
            self,
            ChangeTool,
            "change_tool",
            self.execute_callback,
        )

    def execute_callback(self, _):
        """Execute the action."""
        self.get_logger().info("Executing goal...")

        # Sleep for 5 seconds
        time.sleep(5)

        # Return the result
        result = ChangeTool.Result()
        result.success = True

        return result


def main(args=None):
    """Main entry point."""

    rclpy.init(args=args)
    action_server = ChangeToolActionServer()
    rclpy.spin(action_server)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
