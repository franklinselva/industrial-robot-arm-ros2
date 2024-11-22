"""ROS2 launch file to spawn controllers for the IRB 6640 205 robot."""

from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_spawn_controllers_launch


def generate_launch_description():
    """Launch the spawn_controllers launch file."""
    moveit_config = MoveItConfigsBuilder(
        "irb6640_205", package_name="irb_moveit_config"
    ).to_moveit_configs()
    return generate_spawn_controllers_launch(moveit_config)
