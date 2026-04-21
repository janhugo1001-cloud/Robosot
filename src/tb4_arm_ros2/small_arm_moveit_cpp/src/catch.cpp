#include <memory>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <std_msgs/msg/bool.hpp>

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto const node = std::make_shared<rclcpp::Node>("catch_prepare");
  auto const logger = rclcpp::get_logger("catch_prepare");
  auto start_detection_pub = node->create_publisher<std_msgs::msg::Bool>("start_detection", 10);

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  auto spinner = std::thread([&executor]() { executor.spin(); });

  using moveit::planning_interface::MoveGroupInterface;

  auto arm_group = MoveGroupInterface(node, "small_arm");
  arm_group.setEndEffectorLink("link6");
  arm_group.setPlanningTime(10.0);
  arm_group.setNumPlanningAttempts(10);
  arm_group.setStartStateToCurrentState();
  arm_group.setNamedTarget("take1");

  moveit::planning_interface::MoveGroupInterface::Plan take1_plan;
  const bool take1_success =
    (arm_group.plan(take1_plan) == moveit::core::MoveItErrorCode::SUCCESS);

  if (!take1_success) {
    RCLCPP_ERROR(logger, "Planning to take1 failed!");
    rclcpp::shutdown();
    spinner.join();
    return 0;
  }

  RCLCPP_INFO(logger, "Move small_arm to take1...");
  arm_group.execute(take1_plan);

  auto gripper_group = MoveGroupInterface(node, "gripper");
  gripper_group.setPlanningTime(10.0);
  gripper_group.setNumPlanningAttempts(10);
  gripper_group.setStartStateToCurrentState();
  gripper_group.setNamedTarget("close");

  moveit::planning_interface::MoveGroupInterface::Plan close_plan;
  const bool close_success =
    (gripper_group.plan(close_plan) == moveit::core::MoveItErrorCode::SUCCESS);
  if (!close_success) {
    RCLCPP_ERROR(logger, "Planning gripper close failed!");
    rclcpp::shutdown();
    spinner.join();
    return 0;
  }

  RCLCPP_INFO(logger, "Close gripper...");
  gripper_group.execute(close_plan);

  std_msgs::msg::Bool start_msg;
  start_msg.data = true;
  start_detection_pub->publish(start_msg);
  RCLCPP_INFO(logger, "Sent start_detection=True");
  RCLCPP_INFO(logger, "Prepare stage finished. Launch catch_grasp after coordinates are ready.");

  rclcpp::shutdown();
  spinner.join();
  return 0;
}
