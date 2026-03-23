#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose.hpp>

int main(int argc, char * argv[])
{
  // Basic ROS setup
  // ^^^^^^^^^^^^^^^
  rclcpp::init(argc, argv);
  auto const node = std::make_shared<rclcpp::Node>("pose_goal");

  // Create a ROS logger
  auto const logger = rclcpp::get_logger("pose_goal");

  // MoveIt setup
  // ^^^^^^^^^^^^
  // A MoveGroupInterface instance (a client for the MoveGroup action) can be easily setup using just the name of the
  // planning group you would like to control and plan for.
  using moveit::planning_interface::MoveGroupInterface;
  auto move_group_interface = MoveGroupInterface(node, "xarm7");

  // Getting the current pose of the end-effector.
  geometry_msgs::msg::PoseStamped current_pose;
  current_pose = move_group_interface.getCurrentPose();

  // Modifying the current pose into a target pose.
  // Feel free to change the following position values to see their influence on the motion planning
  geometry_msgs::msg::Pose target_pose = current_pose.pose;
  target_pose.position.x += 0.1;
  target_pose.position.y += -0.2;
  target_pose.position.z += -0.0;
  // Setting the target pose for the end-effector

  // Set the target pose
  move_group_interface.setPoseTarget(target_pose);

  // Calling the planner to compute the motion plan, which is then stored in my_plan.
  // Note that we are just planning, not asking MoveGroupInterface to actually move the robot.
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  bool success = (move_group_interface.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);

  // If the plan is successful, execute the plan
  if(success) {
    move_group_interface.execute(my_plan);
  } else {
    RCLCPP_ERROR(logger, "Planing failed!");
  }

  // Shutdown
  rclcpp::shutdown();
  return 0;
}
