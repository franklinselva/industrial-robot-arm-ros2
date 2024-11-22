"""Setup assistant launch file for the irb6640_205."""

from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_setup_assistant_launch


def generate_launch_description():
    """Launch the setup_assistant launch file."""
    moveit_config = MoveItConfigsBuilder(
        "irb6640_205", package_name="irb_moveit_config"
    ).to_moveit_configs()
    return generate_setup_assistant_launch(moveit_config)
