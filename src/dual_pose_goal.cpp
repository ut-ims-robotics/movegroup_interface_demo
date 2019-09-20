/*
Copyright 2019, University of Tartu

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
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>

int main(int argc, char** argv)
{
  // Basic ROS setup
  // ^^^^^^^^^^^^^^^
  ros::init(argc, argv, "dual_pose_goal");
  ros::NodeHandle node_handle;
  ros::AsyncSpinner spinner(1);
  spinner.start();

  // MoveGroupInterface API can be found at:
  // http://docs.ros.org/kinetic/api/moveit_ros_planning_interface/html/classmoveit_1_1planning__interface_1_1MoveGroupInterface.html

  // MoveIt setup
  // ^^^^^^^^^^^^
  // A MoveGroupInterface instance (a client for the MoveGroup action) can be easily setup using just the name of the
  // planning group you would like to control and plan for.
  moveit::planning_interface::MoveGroupInterface move_group("dual_xarm7");
  // Printing on screen the coordinate reference frame and the palnner for the selected planning group
  ROS_INFO("[movegroup_interface_demo/dual_pose_goal] Planning frame: %s", move_group.getPlanningFrame().c_str());
  move_group.setPlannerId("RRT");
  ROS_INFO("[movegroup_interface_demo/dual_pose_goal] Planner ID: %s", move_group.getPlannerId().c_str());

  // Planning to a pose goal
  // ^^^^^^^^^^^^^^^^^^^^^^^
  // We can plan a motion for this group by defining target poses for any of its end-effectors.
  // As pose requires 7 variables - x, y, and z for the position and x, y, z, and w for the orientation - to be defined,
  // it is convenient to first get the current end-effector pose and then modify it to a suitable target pose.

  // Since our planning group consists of two manipulators, we have up to two end-effectors to plan for.
  // Let's start with the first manipulator.

  // Getting the current pose of the end-effector named right_link7.
  geometry_msgs::PoseStamped current_pose_right;
  current_pose_right = move_group.getCurrentPose("right_link7");

  // Modifying the current pose into a target pose.
  // Feel free to change the following position values to see their influence on the motion planning
  geometry_msgs::Pose target_pose = current_pose_right.pose;
  target_pose.position.x += 0.1;
  target_pose.position.y += -0.2;
  target_pose.position.z += -0.0;

  // Setting the target pose for the end-effector named right_link7
  move_group.setPoseTarget(target_pose, "right_link7");

  // Now let's do the same steps for the second manipulator.
  // Getting the current pose of the end-effector named left_link7.
  geometry_msgs::PoseStamped current_pose_left;
  current_pose_left = move_group.getCurrentPose("left_link7");

  // Modifying the current pose into a target pose.
  // Feel free to change the following position values to see their influence on the motion planning
  target_pose = current_pose_left.pose;
  target_pose.position.x += 0.1;
  target_pose.position.y += 0.0;
  target_pose.position.z += 0.3;

  // Setting the target pose for the end-effector named left_link7
  move_group.setPoseTarget(target_pose, "left_link7");

  // Calling the planner to compute the motion plan, which is then stored in my_plan.
  // Note that we are just planning, not asking MoveGroupInterface to actually move the robots.
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  moveit::planning_interface::MoveItErrorCode success = move_group.plan(my_plan);
  if (success)
  {
    ROS_INFO("[movegroup_interface_demo/dual_pose_goal] Planning OK. Proceeding.");
  }
  else
  {
    ROS_WARN("[movegroup_interface_demo/dual_pose_goal] Planning failed. Shutting Down.");
    ros::shutdown();
    return 0;
  }

  // Executing the computed plan
  // ^^^^^^^^^^^^^^^^^^^^^^^^^^^
  // Uncomment the following line to make the robots move according to the computed plan
  // move_group.execute(my_plan);

  ros::shutdown();
  return 0;
}
