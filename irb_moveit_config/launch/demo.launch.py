"""Demo launch file for the IRB 6640 205 MoveIt configuration."""

from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_demo_launch


def generate_launch_description():
    """Launch the demo launch file."""
    moveit_config = MoveItConfigsBuilder(
        "irb6640_205", package_name="irb_moveit_config"
    ).to_moveit_configs()
    return generate_demo_launch(moveit_config)
