#include <chrono>
#include <memory>
#include <thread>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_msgs/msg/constraints.hpp>
#include <moveit_msgs/msg/joint_constraint.hpp>
#include <moveit_msgs/msg/robot_trajectory.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/int32.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/time.h>

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto const node = std::make_shared<rclcpp::Node>("place_down");
  auto const logger = rclcpp::get_logger("place_down");
  auto start_detection_pub = node->create_publisher<std_msgs::msg::Bool>("start_detection", 10);

  // int arm_to_detect = 0;
  // bool arm_to_detect_received = false;

  // auto arm_to_detect_sub = node->create_subscription<std_msgs::msg::Int32>(
  //   "arm_to_detect", 10,
  //   [&](const std_msgs::msg::Int32::SharedPtr msg) {
  //     arm_to_detect = msg->data;
  //     arm_to_detect_received = true;
  //     RCLCPP_INFO(logger, "Received arm_to_detect=%d", arm_to_detect);
  //   });

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  auto spinner = std::thread([&executor]() { executor.spin(); });

  using moveit::planning_interface::MoveGroupInterface;

  auto arm_group = MoveGroupInterface(node, "small_arm");
  arm_group.setEndEffectorLink("link6");
  arm_group.setPlanningTime(10.0);
  arm_group.setNumPlanningAttempts(10);

  auto gripper_group = MoveGroupInterface(node, "gripper");
  gripper_group.setPlanningTime(10.0);
  gripper_group.setNumPlanningAttempts(10);

  // RCLCPP_INFO(logger, "Waiting for arm_to_detect...");
  // while (rclcpp::ok() && !arm_to_detect_received) {
  //   rclcpp::sleep_for(std::chrono::milliseconds(100));
  // }

  // if (!rclcpp::ok()) {
  //   rclcpp::shutdown();
  //   spinner.join();
  //   return 0;
  // }

  // if (arm_to_detect == 1) {
  //   auto joint_values = arm_group.getCurrentJointValues();
  //   if (!joint_values.empty()) {
  //     // joint1 右轉 90 度
  //     joint_values[0] -= 1.5708;
  //     arm_group.setStartStateToCurrentState();
  //     arm_group.setJointValueTarget(joint_values);

  //     moveit::planning_interface::MoveGroupInterface::Plan turn_plan;
  //     const bool turn_success =
  //       (arm_group.plan(turn_plan) == moveit::core::MoveItErrorCode::SUCCESS);

  //     if (!turn_success) {
  //       RCLCPP_ERROR(logger, "Planning joint1 right turn failed!");
  //       rclcpp::shutdown();
  //       spinner.join();
  //       return 0;
  //     }

  //     RCLCPP_INFO(logger, "Rotate joint1 right by 90 degrees.");
  //     arm_group.execute(turn_plan);
  //   }
  // } else {
  //   RCLCPP_INFO(logger, "arm_to_detect is 0, keep current pose.");
  // }

  // (void)arm_to_detect_sub;

  std_msgs::msg::Bool start_msg;
  start_msg.data = true;
  start_detection_pub->publish(start_msg);
  RCLCPP_INFO(logger, "Sent start_detection=True");

  tf2_ros::Buffer tf_buffer(node->get_clock());
  tf2_ros::TransformListener tf_listener(tf_buffer);
  geometry_msgs::msg::TransformStamped tf_result;

  // 發送啟動訊號後，等待 YOLO 經由 TF2 提供 object_frame。
  RCLCPP_INFO(logger, "Waiting for object_frame from TF...");
  while (rclcpp::ok()) {
    try {
      tf_result = tf_buffer.lookupTransform("base_link", "object_frame", tf2::TimePointZero);
      break;
    } catch (const tf2::TransformException &) {
      rclcpp::sleep_for(std::chrono::milliseconds(200));
    }
  }

  if (!rclcpp::ok()) {
    rclcpp::shutdown();
    spinner.join();
    return 0;
  }

  const double obj_x = tf_result.transform.translation.x;
  const double obj_y = tf_result.transform.translation.y;
  const double obj_z = tf_result.transform.translation.z;

  RCLCPP_INFO(logger,
              "object_frame in base_link: x=%.3f y=%.3f z=%.3f",
              obj_x, obj_y, obj_z);

  const double approach_z = obj_z + 0.12;
  const double release_z = obj_z + 0.09;

  RCLCPP_INFO(logger,
              "Place approach: x=%.3f y=%.3f z=%.3f",
              obj_x, obj_y, approach_z);
  RCLCPP_INFO(logger,
              "Place release : x=%.3f y=%.3f z=%.3f",
              obj_x, obj_y, release_z);

  // 保持目前 link6 姿態，並限制 joint1，避免底座繞太多。
  const auto preferred_orientation = arm_group.getCurrentPose("link6").pose.orientation;
  const auto current_joint_values = arm_group.getCurrentJointValues();

  moveit_msgs::msg::Constraints path_constraints;
  moveit_msgs::msg::JointConstraint joint1_constraint;
  joint1_constraint.joint_name = "joint1";
  joint1_constraint.position = current_joint_values.at(0);
  joint1_constraint.tolerance_above = 1.0;
  joint1_constraint.tolerance_below = 1.0;
  joint1_constraint.weight = 1.0;
  path_constraints.joint_constraints.push_back(joint1_constraint);

  geometry_msgs::msg::Pose approach_pose;
  approach_pose.position.x = obj_x;
  approach_pose.position.y = obj_y;
  approach_pose.position.z = approach_z;
  approach_pose.orientation = preferred_orientation;

  arm_group.setStartStateToCurrentState();
  arm_group.setPoseReferenceFrame("base_link");
  arm_group.setGoalPositionTolerance(0.01);
  arm_group.setGoalOrientationTolerance(0.10);
  arm_group.setPathConstraints(path_constraints);
  arm_group.setPoseTarget(approach_pose);

  moveit::planning_interface::MoveGroupInterface::Plan approach_plan;
  const bool approach_success =
    (arm_group.plan(approach_plan) == moveit::core::MoveItErrorCode::SUCCESS);

  if (!approach_success) {
    RCLCPP_ERROR(logger, "Place approach planning failed!");
    rclcpp::shutdown();
    spinner.join();
    return 0;
  }

  RCLCPP_INFO(logger, "Move to place pose above target...");
  arm_group.execute(approach_plan);

  // 到放置點正上方後，保持姿態直線下降。
  arm_group.clearPathConstraints();
  geometry_msgs::msg::Pose current_pose = arm_group.getCurrentPose("link6").pose;
  geometry_msgs::msg::Pose release_pose = current_pose;
  release_pose.position.x = obj_x;
  release_pose.position.y = obj_y;
  release_pose.position.z = release_z;

  arm_group.setStartStateToCurrentState();
  std::vector<geometry_msgs::msg::Pose> release_waypoints;
  release_waypoints.push_back(release_pose);
  moveit_msgs::msg::RobotTrajectory release_trajectory;
  const double release_fraction = arm_group.computeCartesianPath(
    release_waypoints, 0.01, 0.0, release_trajectory);

  if (release_fraction <= 0.89) {
    RCLCPP_ERROR(logger, "Cartesian move down for place failed! fraction=%.2f", release_fraction);
    rclcpp::shutdown();
    spinner.join();
    return 0;
  }

  moveit::planning_interface::MoveGroupInterface::Plan release_plan;
  release_plan.trajectory_ = release_trajectory;
  RCLCPP_INFO(logger, "Move down to place pose...");
  arm_group.execute(release_plan);

  // 到定位後打開夾爪放下物體。
  gripper_group.setStartStateToCurrentState();
  gripper_group.setNamedTarget("open");

  moveit::planning_interface::MoveGroupInterface::Plan open_plan;
  const bool open_success =
    (gripper_group.plan(open_plan) == moveit::core::MoveItErrorCode::SUCCESS);

  if (!open_success) {
    RCLCPP_ERROR(logger, "Planning gripper open failed!");
    rclcpp::shutdown();
    spinner.join();
    return 0;
  }

  RCLCPP_INFO(logger, "Open gripper to place object...");
  gripper_group.execute(open_plan);

  rclcpp::shutdown();
  spinner.join();
  return 0;
}
