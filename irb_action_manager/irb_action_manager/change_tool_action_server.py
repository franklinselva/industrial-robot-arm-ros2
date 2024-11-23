"""Change tool action server.

Mock action server for changing the tool of the robot.

waits for a request, moves to the change tool location, sleeps for 5 seconds, and returns a success response.
"""

import time

import rclpy
from rclpy.action import ActionClient, ActionServer
from rclpy.node import Node

from irb_interfaces.action import ChangeTool, GoTo  # type: ignore


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

        self._goto_action_client = ActionClient(self, GoTo, "goto")
        self._goto_action_client.wait_for_server()

        self.future = None

    def execute_callback(self, _):
        """Execute the action."""
        self.get_logger().info("Executing goal...")

        # Move to change tool position
        goal = GoTo.Goal()
        goal.goal_configuration = "change_to_screwdriver"
        self.future = self._goto_action_client.send_goal_async(goal)
        self.future.add_done_callback(self.goto_response_callback)

        # Sleep for 5 seconds
        time.sleep(5)

        # Return the result
        result = ChangeTool.Result()
        result.success = True

        return result

    def goto_response_callback(self, future):
        """Callback for goto response."""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn("Failed to move to change tool position")
            return

        self.get_logger().info("Moved to change tool position")
        goal_handle.get_result_async().add_done_callback(self.goto_result_callback)

    def goto_result_callback(self, future):
        """Callback for goto result."""
        result = future.result().result
        if not result.success:
            self.get_logger().warn("Failed to move to change tool position")
            return

        self.get_logger().info("Moved to change tool position")

    def feedback_callback(self, feedback):
        """Feedback callback."""
        self.get_logger().info(f"Received feedback: {feedback}")


def main(args=None):
    """Main entry point."""

    rclpy.init(args=args)
    action_server = ChangeToolActionServer()
    rclpy.spin(action_server)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
