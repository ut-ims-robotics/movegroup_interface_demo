# movegroup_interface_demo
A package designed to help learning ROS MoveIt Move Group C++ Interface.

The underlying idea for this package is to provide easy-to-understand coding examples to illustrate the capabilities of [Move Group C++ Interface](http://docs.ros.org/kinetic/api/moveit_ros_planning_interface/html/classmoveit_1_1planning__interface_1_1MoveGroupInterface.html "moveit::planning_interface::MoveGroupInterface Class Reference"). For that reason:
+ the coding aims to be as minimalistic as possible with intuitive variable names;
+ different features are exemplified individually in separate ROS nodes to make code readable for beginners in computer programming.

## Prerequisites
1. If you don't have the MoveIt Move Group Interface package on your ROS system, you can easily install it with the following command.
```bash
sudo apt install ros-$ROS_DISTRO-moveit-ros-planning-interface
```
2. You also need a MoveIt configuration package to use this demo. Although this package should be usable with any properly setup MoveIt configuration package, the development and testing has been conducted using xArm7 ROS support available at https://github.com/xArm-Developer/xarm_ros

## Running with xArm7

Clone repositories for xarm_ros and movegroup_interface_demo to your catkin workspace and compile it.

Using `demo.launch` to run MoveIt and visualize the xArm7 manipulator in RViz
```bash
roslaunch xarm7_moveit_config demo.launch
```

Demonstration of planning to a ***pose*** goal
```bash
rosrun movegroup_interface_demo pose_goal
```

Demonstration of planning to a ***named*** goal
```bash
rosrun movegroup_interface_demo named_goal
```

Demonstration of planning to a ***joint value*** goal
```bash
rosrun movegroup_interface_demo joint_value_goal
```

Demonstration of computing a ***Cartesian path***
```bash
rosrun movegroup_interface_demo cartesian_path
```

## Running with xArm7 and its gripper

Clone repositories for xarm_ros and movegroup_interface_demo to your catkin workspace and compile it.

Using `demo.launch` to run MoveIt and visualize the xArm7 manipulator with gripper in RViz
```bash
roslaunch xarm7_gripper_moveit_config demo.launch
```

Demonstration of closing the gripper by planning to a ***named*** goal
```bash
rosrun movegroup_interface_demo gripper_close
```

## Other robots

In order to use movegroup_interface_demo with other MoveIt planning groups, adjust hard-coded planning group names and pose values in the .cpp files located in `src` folder.

