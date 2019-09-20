cmake_minimum_required(VERSION 2.8.3)
project(movegroup_interface_demo)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  moveit_ros_planning_interface
)


###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES movegroup_interface_tutorial
  CATKIN_DEPENDS geometry_msgs roscpp moveit_ros_planning_interface
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(pose_goal src/pose_goal.cpp)
target_link_libraries(pose_goal ${catkin_LIBRARIES})

add_executable(named_goal src/named_goal.cpp)
target_link_libraries(named_goal ${catkin_LIBRARIES})

add_executable(joint_value_goal src/joint_value_goal.cpp)
target_link_libraries(joint_value_goal ${catkin_LIBRARIES})

add_executable(cartesian_path src/cartesian_path.cpp)
target_link_libraries(cartesian_path ${catkin_LIBRARIES})

add_executable(dual_pose_goal src/dual_pose_goal.cpp)
target_link_libraries(dual_pose_goal ${catkin_LIBRARIES})
