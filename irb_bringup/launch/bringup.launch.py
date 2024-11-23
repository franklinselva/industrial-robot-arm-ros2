"""Launch file for bringup."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription  # type: ignore
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    """Launch the bringup nodes."""

    actions_manager = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("irb_action_manager"),
                "launch",
                "start_scene.launch.py",
            )
        )
    )

    return LaunchDescription([actions_manager])
