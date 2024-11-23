"""Launch file for the action servers."""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """Launch the action servers."""

    change_tool_action_server = Node(
        name="change_tool_action_server",
        package="irb_action_manager",
        executable="change_tool_action_server",
        output="both",
    )

    detect_screws_action_server = Node(
        name="detect_screws_action_server",
        package="irb_action_manager",
        executable="detect_screws_action_server",
        output="both",
    )

    unscrew_action_server = Node(
        name="unscrew_action_server",
        package="irb_action_manager",
        executable="unscrew_action_server",
        output="both",
    )

    ## Drop screws and goto action servers are a part of moveit_action_server node
    ## and declared in the other launch file

    return LaunchDescription(
        [
            change_tool_action_server,
            detect_screws_action_server,
            unscrew_action_server,
        ]
    )
