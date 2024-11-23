### IRB6640 Robot Arm for Unscrewing task in a battery pack assembly line

This repository is a demonstration of the use of the IRB6640 robot arm for unscrewing task in a battery pack assembly line.

### Technologies Used

 - ROS2 Jazzy
 - rclpy
 - MoveIt2
 - [Embedded Systems Bridge](https://github.com/aiplan4eu/embedded-systems-bridge) for task planning


### Packages

 - **irb6640_action_manager**: Contains the action server related the unscrewing task. Some functionalities are mocked for demonstration purposes.
 - **irb6640_bringup**: Contains the launch files for bringing up the IRB6640 robot arm.
 - **irb6640_description**: Contains the URDF and meshes for the IRB6640 robot arm
 - **irb6640_moveit_config**: Contains the MoveIt configuration for the IRB6640 robot arm
 - **irb6640_task_planner**: Contains the task planner for the unscrewing task. It uses the Embedded Systems Bridge for task planning.


## Setup

To run this project, make sure you have ROS2 Jazzy and Moveit2 installed. Then, clone this repository and run the following commands:

```bash
git clone https://github.com/franklinselva/industrial-robot-arm-ros2.git
cd industrial-robot-arm-ros2
rosdep update && rosdep install --from-paths src --ignore-src -r -y --rosdistro $ROS_DISTRO
colcon build
source install/setup.bash
```

### Running the project

| TODO: Add instructions on how to run the project
