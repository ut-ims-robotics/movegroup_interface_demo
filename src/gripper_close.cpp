/*
Copyright 2022, University of Tartu

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the
following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following
disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following
disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products
derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

/* Author: Karl Kruusamäe */
/* E-mail: karl.kruusamae@ut.ee */

#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>

int main(int argc, char** argv)
{
  // Basic ROS setup
  // ^^^^^^^^^^^^^^^
  ros::init(argc, argv, "gripper_close");
  ros::NodeHandle node_handle;
  ros::AsyncSpinner spinner(1);
  spinner.start();

  // MoveGroupInterface API can be found at:
  // http://docs.ros.org/noetic/api/moveit_ros_planning_interface/html/classmoveit_1_1planning__interface_1_1MoveGroupInterface.html

  // Controlling a gripper within makes use of the same concepts as controlling the manipulator robots.
  // We need to create an instance of MoveGroupInterface, set target states for the gripper,
  // and then plan & execute motions for the gripper.
  // In this coding example the gripper is controlled by using named states, so it is analogous to named_goal.cpp

  // MoveIt setup
  // ^^^^^^^^^^^^
  // A MoveGroupInterface instance (a client for the MoveGroup action) can be easily setup using just the name of the
  // planning group we would like to control and plan for.
  moveit::planning_interface::MoveGroupInterface move_group("xarm_gripper");

  // Planning to a predefined named pose
  // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
  // To close and open a gripper, we can often use named states which have been 
  // defined for the gripper planning group using the MoveIt Setup Assistant.

  // Setting the named pose "close" as target
  move_group.setNamedTarget("close");

  // Calling the planner to compute the motion plan, which is then stored in my_plan.
  // Note that we are just planning, not asking MoveGroupInterface to actually move the robot.
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  moveit::core::MoveItErrorCode success = move_group.plan(my_plan);
  if (success)
  {
    ROS_INFO("[movegroup_interface_demo/gripper_close] Planning OK. Proceeding.");
  }
  else
  {
    ROS_WARN("[movegroup_interface_demo/gripper_close] Planning failed. Shutting Down.");
    ros::shutdown();
    return 0;
  }

  // Executing the computed plan
  // ^^^^^^^^^^^^^^^^^^^^^^^^^^^
  // Uncomment the following line to make the robot move according to the computed plan
  // move_group.execute(my_plan);

  ros::shutdown();
  return 0;
}
