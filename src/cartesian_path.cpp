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
#include <vector>

int main(int argc, char** argv)
{
  // Basic ROS setup
  // ^^^^^^^^^^^^^^^
  ros::init(argc, argv, "cartesian_path");
  ros::NodeHandle node_handle;
  ros::AsyncSpinner spinner(1);
  spinner.start();

  // MoveGroupInterface API can be found at:
  // http://docs.ros.org/kinetic/api/moveit_ros_planning_interface/html/classmoveit_1_1planning__interface_1_1MoveGroupInterface.html

  // MoveIt setup
  // ^^^^^^^^^^^^
  // A MoveGroupInterface instance (a client for the MoveGroup action) can be easily setup using just the name of the
  // planning group you would like to control and plan for.
  moveit::planning_interface::MoveGroupInterface move_group("xarm7");

  // Planning a Cartesian path
  // ^^^^^^^^^^^^^^^^^^^^^^^^^
  // We can plan a Cartesian path by specifying a series of waypoints for the end-effector to go through.
  // As each waypoint will be defined as a pose, which requires 7 variables - x, y, and z for the position and x, y, z,
  // and w for the orientation - to be defined, it is convenient to first get the current end-effector pose and then
  // modify it to a suitable waypoint.

  // Getting the current pose of the end-effector.
  geometry_msgs::PoseStamped current_pose;
  current_pose = move_group.getCurrentPose();

  // Creating a vector that contains all the waypoints of the desired Cartesian path.
  std::vector<geometry_msgs::Pose> waypoints;

  // Modifying the current pose into a next waypoint and then each waypoint into a subsequent one.
  // Feel free to change the following position values to see their influence on the motion planning.
  geometry_msgs::Pose next_waypoint = current_pose.pose;

  // Move diagonally in xy-plane
  next_waypoint.position.x += 0.2;
  next_waypoint.position.y -= 0.2;
  waypoints.push_back(next_waypoint);

  // Move only along y-axis
  next_waypoint.position.y += 0.2;
  waypoints.push_back(next_waypoint);

  // Move diagonally in xy-plane
  next_waypoint.position.x -= 0.2;
  next_waypoint.position.y += 0.2;
  waypoints.push_back(next_waypoint);

  // Calling the planner to compute the Cartesian path, which is stored in my_plan.trajectory_.
  // Note that we are just planning, not asking MoveGroupInterface to actually move the robot.
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  double eef_step = 0.01;
  double jump_threshold = 0;
  double fraction = move_group.computeCartesianPath(waypoints, eef_step, jump_threshold, my_plan.trajectory_);

  if (fraction == 1)
  {
    ROS_INFO("[movegroup_interface_demo/cartesian_path] Planning OK. Proceeding.");
  }
  else
  {
    ROS_WARN("[movegroup_interface_demo/cartesian_path] The whole Cartesian path was not planned for (%.2f%% "
             "successfully computed). Shutting Down.",
             fraction * 100.0);
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
