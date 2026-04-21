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
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/time.h>

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto const node = std::make_shared<rclcpp::Node>("catch_grasp");
  auto const logger = rclcpp::get_logger("catch_grasp");

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

  tf2_ros::Buffer tf_buffer(node->get_clock());
  tf2_ros::TransformListener tf_listener(tf_buffer);
  geometry_msgs::msg::TransformStamped tf_result;

  // 等主控把物體座標發布到 TF，拿到 object_frame 後才繼續。
  RCLCPP_INFO(logger, "Waiting for object_frame from main controller...");
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

  // 這裡再打開一次夾爪，讓後半段 node 可以獨立執行。
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

  RCLCPP_INFO(logger, "Open gripper after receiving object_frame...");
  gripper_group.execute(open_plan);

  const double obj_x = tf_result.transform.translation.x;
  const double obj_y = tf_result.transform.translation.y;
  const double obj_z = tf_result.transform.translation.z;

  // 先套用目前實測調過的 XY 偏移，再做後續抓取規劃。
  double target_x = obj_x - 0.015;
  const double target_y = obj_y - 0.04;

  RCLCPP_INFO(logger,
              "object_frame in base_link: x=%.3f y=%.3f z=%.3f",
              obj_x, obj_y, obj_z);

  const double approach_z = obj_z + 0.12;
  const double grasp_z = obj_z + 0.06;

  RCLCPP_INFO(logger,
              "Approach pose: x=%.3f y=%.3f z=%.3f",
              target_x, target_y, approach_z);
  RCLCPP_INFO(logger,
              "Grasp pose   : x=%.3f y=%.3f z=%.3f",
              target_x, target_y, grasp_z);

  // 保持目前 link6 的姿態，並限制 joint1 / joint5，避免底座繞太多、第五軸抬起來。
  const auto preferred_orientation = arm_group.getCurrentPose("link6").pose.orientation;
  const auto current_joint_values = arm_group.getCurrentJointValues();

  moveit_msgs::msg::Constraints path_constraints;
  moveit_msgs::msg::JointConstraint joint1_constraint;
  joint1_constraint.joint_name = "joint1";
  joint1_constraint.position = current_joint_values.at(0);
  joint1_constraint.tolerance_above = 2.0;
  joint1_constraint.tolerance_below = 2.0;
  joint1_constraint.weight = 1.0;
  path_constraints.joint_constraints.push_back(joint1_constraint);

  moveit_msgs::msg::JointConstraint joint5_constraint;
  joint5_constraint.joint_name = "joint5";
  joint5_constraint.position = current_joint_values.at(4);
  joint5_constraint.tolerance_above = 0.35;
  joint5_constraint.tolerance_below = 0.35;
  joint5_constraint.weight = 1.0;
  path_constraints.joint_constraints.push_back(joint5_constraint);
  

  geometry_msgs::msg::Pose approach_pose;
  approach_pose.position.x = target_x;
  approach_pose.position.y = target_y;
  approach_pose.position.z = approach_z;
  // 鎖住目前末端姿態，避免接近物體時先把夾爪轉掉。
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
    RCLCPP_ERROR(logger, "Approach planning failed!");
    rclcpp::shutdown();
    spinner.join();
    return 0;
  }

  RCLCPP_INFO(logger, "Move to pose above object...");
  arm_group.execute(approach_plan);

  // 到物體正上方後，保持手腕姿態不變，直接垂直往下。
  //arm_group.clearPathConstraints();
  geometry_msgs::msg::Pose current_pose = arm_group.getCurrentPose("link6").pose;
  geometry_msgs::msg::Pose grasp_pose = current_pose;
  // Keep current x/y and orientation, move down only on Z axis.
  grasp_pose.position.z = grasp_z;
  approach_pose.position.z = approach_z;


  arm_group.setStartStateToCurrentState();
  std::vector<geometry_msgs::msg::Pose> grasp_waypoints;
  grasp_waypoints.push_back(grasp_pose);
  moveit_msgs::msg::RobotTrajectory grasp_trajectory;
  const double grasp_fraction = arm_group.computeCartesianPath(
    grasp_waypoints, 0.01, 0.0, grasp_trajectory);

  if (grasp_fraction <= 0.99) {
    RCLCPP_ERROR(logger, "Cartesian move down failed! fraction=%.2f", grasp_fraction);
    rclcpp::shutdown();
    spinner.join();
    return 0;
  }

  moveit::planning_interface::MoveGroupInterface::Plan grasp_plan;
  grasp_plan.trajectory_ = grasp_trajectory;
  RCLCPP_INFO(logger, "Move down to grasp pose...");
  arm_group.execute(grasp_plan);

  // 確定直線下探成功後，再關夾爪。
  gripper_group.setStartStateToCurrentState();
  gripper_group.setNamedTarget("catch_g");

  moveit::planning_interface::MoveGroupInterface::Plan catch_g_plan;
  const bool catch_g_success =
    (gripper_group.plan(catch_g_plan) == moveit::core::MoveItErrorCode::SUCCESS);

  if (!catch_g_success) {
    RCLCPP_ERROR(logger, "Planning gripper catch failed!");
    rclcpp::shutdown();
    spinner.join();
    return 0;
  }

  RCLCPP_INFO(logger, "Catch object...");
  gripper_group.execute(catch_g_plan);

  // 抓取完成後收回 home，避免維持前面的路徑限制影響 named target 規劃。
  arm_group.clearPathConstraints();
  arm_group.clearPoseTargets();
  arm_group.setStartStateToCurrentState();
  arm_group.setNamedTarget("home");

  moveit::planning_interface::MoveGroupInterface::Plan home_plan;
  const bool home_success =
    (arm_group.plan(home_plan) == moveit::core::MoveItErrorCode::SUCCESS);

  if (!home_success) {
    RCLCPP_ERROR(logger, "Planning return to home failed!");
    rclcpp::shutdown();
    spinner.join();
    return 0;
  }

  RCLCPP_INFO(logger, "Return small_arm to home...");
  arm_group.execute(home_plan);

  rclcpp::shutdown();
  spinner.join();
  return 0;
}
