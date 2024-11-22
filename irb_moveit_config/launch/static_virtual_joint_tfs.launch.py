"""Static virtual joint TFs launch file."""

from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_static_virtual_joint_tfs_launch


def generate_launch_description():
    """Static virtual joint TFs launch file."""
    moveit_config = MoveItConfigsBuilder(
        "irb6640_205", package_name="irb_moveit_config"
    ).to_moveit_configs()
    return generate_static_virtual_joint_tfs_launch(moveit_config)
