### IRB6640 Robot Arm for Unscrewing task in a battery pack assembly line

This repository is a demonstration of the use of the IRB6640 robot arm for unscrewing tasks in a battery pack.

### Technologies Used

 - ROS2 Jazzy
 - rclpy
 - MoveIt2
 - [Embedded Systems Bridge](https://github.com/aiplan4eu/embedded-systems-bridge) for task planning


### Packages

 - **irb_action_manager**: Contains the action server related to the unscrewing task. Some functionalities are mocked for demonstration purposes.
 - **irb_bringup**: Contains the launch files for bringing up the IRB6640 robot arm.
 - **irb_description**: Contains the URDF and meshes for the IRB6640 robot arm
 - **irb_interfaces**: Contains the interface for the IRB6640 robot arm. It uses the ROS2 action server for communication.
 - **irb_moveit_config**: Contains the MoveIt configuration for the IRB6640 robot arm
 - **irb_task_planner**: Contains the task planner for the unscrewing task. It uses the Embedded Systems Bridge for task planning.


### Setup

To run this project, make sure you have ROS2 Jazzy and Moveit2 installed. Then, clone this repository and run the following commands:

```bash
git clone https://github.com/franklinselva/industrial-robot-arm-ros2.git
cd industrial-robot-arm-ros2
pip3 install -r requirements.txt # Make source the python environment is linked to ROS2
rosdep update && rosdep install --from-paths src --ignore-src -r -y --rosdistro $ROS_DISTRO
colcon build
source install/setup.bash
```

### Running the project

To run the project, you can use the following commands:

```bash
# Make sure you have sourced the workspace
source install/setup.bash

# Bring up the robot arm
ros2 launch irb_bringup bringup.launch.py

# On a new terminal, run the task planner
ros2 run irb_task_planner unscrew_task_planner
```

If you want to run only the task planner, you can use the following command:

```bash
python3 scripts/unscrew_task_planning.py
```

### Known Issues

 - Moveit2 on ROS2 Jazzy might face issues when running `demo.launch.py` of `irb_moveit_config` package. You can refer to the issue [moveit/moveit2#2734](https://github.com/moveit/moveit2/issues/2734)

### Improvements

 - The task planner can be improved to handle more complex scenarios and optimizations.
 - The mocked action servers can be replaced with actual implementations.
