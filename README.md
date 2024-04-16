# movegroup_interface_demo
A package designed to help learning ROS MoveIt Move Group C++ Interface.

The underlying idea for this package is to provide easy-to-understand coding examples to illustrate the capabilities of [Move Group C++ Interface](https://github.com/ros-planning/moveit2/blob/main/moveit_ros/planning_interface/move_group_interface/include/moveit/move_group_interface/move_group_interface.h "moveit::planning_interface::MoveGroupInterface Class Reference"). For that reason:
+ the coding aims to be as minimalistic as possible with intuitive variable names;
+ different features are exemplified individually in separate ROS nodes to make code readable for beginners in computer programming.

## Prerequisites
1. If you don't have the MoveIt on your ROS system, you can easily install it with the following command.
```bash
sudo apt install ros-$ROS_DISTRO-moveit
```
2. You also need a MoveIt configuration package to use this demo. Although this package should be usable with any properly setup MoveIt configuration package, the development and testing has been conducted using xArm7 ROS support available at https://github.com/xArm-Developer/xarm_ros2

## Running with xArm7 (simulated)

Clone repositories for xarm_ros and movegroup_interface_demo to your catkin workspace and compile it.

To run MoveIt and visualize the ***fake*** xArm7 manipulator in RViz
```bash
ros2 launch xarm_moveit_config xarm7_moveit_fake.launch.py add_gripper:=true
```

Demonstration of planning to a ***pose*** goal
```bash
ros2 run movegroup_interface_demo pose_goal --ros-args -p use_sim_time:=true
```

## Other robots

In order to use movegroup_interface_demo with other MoveIt planning groups, adjust hard-coded planning group names and pose values in the .cpp files located in `src` folder.

