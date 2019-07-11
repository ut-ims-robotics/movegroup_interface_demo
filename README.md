# movegroup_interface_demo
A package designed to help learning ROS MoveIt Move Group C++ Interface.

Although this package should be usable with any properly setup MoveIt configuration package, the development and testing has been conducted using xArm7 ROS support available at https://github.com/xArm-Developer/xarm_ros

## Running with xArm7

Clone xarm_ros and movegroup_interface_demo repositories to your catkin workspace and compile it.

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

## Other robots

In order to use movegroup_interface_demo with other MoveIt planning groups, adjust hard-coded planning group names and pose values in the .cpp files located in src folder.


