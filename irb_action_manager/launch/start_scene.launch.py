"""Launch the MoveIt2 demo."""

import os

from ament_index_python.packages import get_package_share_directory  # type: ignore
from launch import LaunchDescription  # type: ignore
from launch.actions import ExecuteProcess, IncludeLaunchDescription  # type: ignore
from launch.launch_description_sources import (
    PythonLaunchDescriptionSource,  # type: ignore
)
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder  # type: ignore


def generate_launch_description():
    """Launch the MoveIt2 demo."""

    planning_api_config = os.path.join(
        get_package_share_directory("irb_action_manager"),
        "config",
        "planning_api.yaml",
    )
    rviz_config_file = os.path.join(
        get_package_share_directory("irb_action_manager"),
        "config",
        "moveit.rviz",
    )

    ros_controller_file = os.path.join(
        get_package_share_directory("irb_moveit_config"),
        "config",
        "ros2_controllers.yaml",
    )

    irb_moveit_config = os.path.join(
        get_package_share_directory("irb_moveit_config"),
        "config",
    )

    moveit_config = (
        MoveItConfigsBuilder("irb6640_205", package_name="irb_moveit_config")
        .moveit_cpp(file_path=planning_api_config)
        .robot_description_kinematics(
            file_path=os.path.join(irb_moveit_config, "kinematics.yaml")
        )
        .to_moveit_configs()
    )

    moveit_action_server = Node(
        name="moveit_action_server",
        package="irb_action_manager",
        executable="moveit_action_server",
        output="both",
        parameters=[moveit_config.to_dict()],
    )

    rviz_node = Node(
        name="rviz2",
        package="rviz2",
        executable="rviz2",
        output="both",
        arguments=["-d", rviz_config_file],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
        ],
    )

    static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher",
        output="log",
        arguments=["--frame-id", "world", "--child-frame-id", "base_link"],
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="log",
        parameters=[moveit_config.robot_description],
    )

    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[ros_controller_file],
        remappings=[("/controller_manager/robot_description", "/robot_description")],
        output="both",
    )

    action_servers = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("irb_action_manager"),
                "launch",
                "action_servers.launch.py",
            )
        )
    )

    load_controllers = []
    for controller in [
        "arm_controller",
        "joint_state_broadcaster",
    ]:
        load_controllers += [
            ExecuteProcess(
                cmd=[f"ros2 run controller_manager spawner {controller}"],
                shell=True,
                output="log",
            )
        ]

    ld = LaunchDescription(
        [
            moveit_action_server,
            rviz_node,
            static_tf,
            robot_state_publisher,
            ros2_control_node,
            action_servers,
        ]
        + load_controllers
    )

    return ld
