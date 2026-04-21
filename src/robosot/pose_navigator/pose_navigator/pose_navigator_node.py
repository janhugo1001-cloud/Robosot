#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Quaternion, Twist
from std_msgs.msg import String
import json
import os
import time
import math
from ament_index_python.packages import get_package_share_directory
from turtlebot4_navigation.turtlebot4_navigator import TurtleBot4Navigator
from nav2_msgs.action import NavigateToPose
from action_msgs.msg import GoalStatus

# Define TaskResult enum to match the navigation result values
class TaskResult:
    SUCCEEDED = GoalStatus.STATUS_SUCCEEDED
    CANCELED = GoalStatus.STATUS_CANCELED
    FAILED = GoalStatus.STATUS_ABORTED

# 定義方向類別
class Direction:
    FORWARD = 1.0      # 前方 - 1弧度
    trunLEFT = 2.571       # 正左 - 1 + π/2 弧度 
    trunRIGHT = -0.571     # 正右 - 1 - π/2 弧度
    BACKWARD = 4.142   # 正後 - 1 + π 弧度
    
    @staticmethod
    def to_quaternion(yaw_radians):
        """將弧度轉換為四元數"""
        # 將弧度正規化到 [-π, π] 範圍
        while yaw_radians > math.pi:
            yaw_radians -= 2 * math.pi
        while yaw_radians <= -math.pi:
            yaw_radians += 2 * math.pi
            
        # 計算四元數 (只有 z 和 w 分量用於 2D 旋轉)
        z = math.sin(yaw_radians / 2.0)
        w = math.cos(yaw_radians / 2.0)
        return z, w

class PoseNavigator(Node):
    def __init__(self):
        super().__init__('pose_navigator')
        
        # Create subscriber for pose name
        self.pose_sub = self.create_subscription(
            String,
            'pose_name',
            self.pose_callback,
            10)
            
        # Create publisher for navigation completion
        self.nav_done_pub = self.create_publisher(
            String,
            'nav_done',
            10)

        # Create publisher for cmd_vel
        self.cmd_vel_pub = self.create_publisher(
            Twist,
            'cmd_vel',
            10)

        # Create publisher for initial pose
        # Initialize navigator
        self.navigator = TurtleBot4Navigator()
        
        # Load poses from config file
        self.poses = self.load_poses()
        


    def load_poses(self):
        """Load poses from config file using package path"""
        try:
            # Get the package share directory
            package_share_dir = get_package_share_directory('pose_navigator')
            config_path = os.path.join(package_share_dir, 'config', 'poses.json')
            
            with open(config_path, 'r') as f:
                return json.load(f)
        except FileNotFoundError:
            self.get_logger().error(f'Pose config file not found at {config_path}')
            return {}
        except Exception as e:
            self.get_logger().error(f'Error loading poses: {str(e)}')
            return {}

    def navigate_to_pose(self, pose_name, direction=None):
        """Navigate to a specific pose with optional direction override
        
        Args:
            pose_name (str): 目標位置名稱，若為 "now" 則使用當前位置
            direction (float, optional): 目標方向弧度值，若為 "now" 則使用當前角度，若未指定則使用預設方向
        """
        # Get current pose for "now" functionality
        current_pose = None
        if pose_name == "now" or direction == "now":
            try:
                current_pose = self.navigator.getCurrentPose()
                if current_pose is None:
                    self.get_logger().error('Could not get current robot pose for "now" functionality')
                    return
            except Exception as e:
                self.get_logger().error(f'Error getting current pose: {str(e)}')
                return
        
        # Handle position
        if pose_name == "now":
            # Use current position
            target_x = current_pose.pose.position.x
            target_y = current_pose.pose.position.y
            self.get_logger().info('Using current position')
        else:
            # Use specified pose
            if pose_name not in self.poses:
                self.get_logger().error(f'Pose {pose_name} not found in configuration')
                return
            pose_data = self.poses[pose_name]
            target_x = pose_data['x']
            target_y = pose_data['y']
        
        # Create pose stamped message
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = self.get_clock().now().to_msg()
        
        # Set position
        goal_pose.pose.position.x = target_x
        goal_pose.pose.position.y = target_y
        goal_pose.pose.position.z = 0.0
        
        # Handle orientation
        if direction == "now":
            # Use current orientation
            goal_pose.pose.orientation.x = current_pose.pose.orientation.x
            goal_pose.pose.orientation.y = current_pose.pose.orientation.y
            goal_pose.pose.orientation.z = current_pose.pose.orientation.z
            goal_pose.pose.orientation.w = current_pose.pose.orientation.w
            self.get_logger().info('Using current orientation')
        elif direction is not None:
            # Use specified direction
            z, w = Direction.to_quaternion(direction)
            goal_pose.pose.orientation.z = z
            goal_pose.pose.orientation.w = w
            self.get_logger().info(f'Using custom direction: {direction} radians')
        else:
            # Use default pose orientation (only if pose_name is not "now")
            if pose_name != "now":
                pose_data = self.poses[pose_name]
                goal_pose.pose.orientation.z = pose_data['z']
                goal_pose.pose.orientation.w = pose_data['w']
                self.get_logger().info('Using default pose orientation')
            else:
                # If pose is "now" but no direction specified, use current orientation
                goal_pose.pose.orientation.x = current_pose.pose.orientation.x
                goal_pose.pose.orientation.y = current_pose.pose.orientation.y
                goal_pose.pose.orientation.z = current_pose.pose.orientation.z
                goal_pose.pose.orientation.w = current_pose.pose.orientation.w
                self.get_logger().info('Using current orientation (default for "now")')
        
        goal_pose.pose.orientation.x = 0.0 if direction != "now" else goal_pose.pose.orientation.x
        goal_pose.pose.orientation.y = 0.0 if direction != "now" else goal_pose.pose.orientation.y
        
        # Navigate to pose
        direction_str = f' with direction {direction}' if direction is not None else ''
        pose_str = f'current position' if pose_name == "now" else pose_name
        self.get_logger().info(f'Navigating to pose: {pose_str}{direction_str}')
        self.navigator.startThroughPoses([goal_pose])
        
        self.get_logger().info(f'Navigation to {pose_str} succeeded!')
        # Publish navigation completion
        done_msg = String()
        done_msg.data = "True"
        self.nav_done_pub.publish(done_msg)


    def turn_right_180_degrees(self):
        """使用 cmd_vel 原地右轉 180 度"""
        self.get_logger().info('開始右轉 180 度')
        
        # 創建 Twist 訊息
        twist_msg = Twist()
        twist_msg.linear.x = 0.0
        twist_msg.linear.y = 0.0
        twist_msg.linear.z = 0.0
        twist_msg.angular.x = 0.0
        twist_msg.angular.y = 0.0
        twist_msg.angular.z = -1.0  # 負值為右轉 (rad/s)
        
        # 計算旋轉時間：180度 = π 弧度，角速度 1.0 rad/s
        rotation_time = math.pi  # π 秒 = 180度 / 1.0 rad/s
        
        # 發布旋轉命令
        start_time = time.time()
        rate = self.create_rate(10)  # 10 Hz
        
        while (time.time() - start_time) < rotation_time:
            self.cmd_vel_pub.publish(twist_msg)
            rate.sleep()
        
        # 停止旋轉
        stop_msg = Twist()
        self.cmd_vel_pub.publish(stop_msg)
        
        self.get_logger().info('右轉 180 度完成')


    def pose_callback(self, msg):
        """Callback for pose name subscription"""
        pose_name = msg.data
        self.get_logger().info(f'Received pose name: {pose_name}')
        self.navigate_to_pose(pose_name)
        
def main(args=None):
    rclpy.init(args=args)
    node = PoseNavigator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
