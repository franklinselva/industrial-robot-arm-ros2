"""MoveIt RViz launch file."""

from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_moveit_rviz_launch


def generate_launch_description():
    """Launch the moveit_rviz launch file."""
    moveit_config = MoveItConfigsBuilder(
        "irb6640_205", package_name="irb_moveit_config"
    ).to_moveit_configs()
    return generate_moveit_rviz_launch(moveit_config)
