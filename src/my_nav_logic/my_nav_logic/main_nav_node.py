import rclpy
import math
import threading
from rclpy.node import Node
from std_msgs.msg import Empty, String, Bool
from std_msgs.msg import Int32
from geometry_msgs.msg import PoseStamped
from turtlebot4_navigation.turtlebot4_navigator import TurtleBot4Navigator
from nav2_simple_commander.robot_navigator import TaskResult
from rclpy.executors import MultiThreadedExecutor
import time


class MainNavigator(Node):
    def __init__(self):
        super().__init__('main_navigator')

        self.navigator = TurtleBot4Navigator()
        self._moving = False  # 防止重複執行
        self.current_pose = 'home'  # 追蹤當前點位
        
        # 設定初始位置（對應地圖座標）
        initial_pose = PoseStamped()
        initial_pose.header.frame_id = 'map'
        initial_pose.header.stamp = self.navigator.get_clock().now().to_msg()
        initial_pose.pose.position.x = -0.9268927301114445   # 改成你的初始 x
        initial_pose.pose.position.y = 1.2852939856101835     # 改成你的初始 y
        initial_pose.pose.orientation.z = -0.33640448449212706  # 改成你的初始方向 z
        initial_pose.pose.orientation.w = 0.9417175918573393 # 改成你的初始方向 w
        self.navigator.setInitialPose(initial_pose)

        self.poses = {
            # 'zone_C': {'x': -0.25545442913713464, 'y': 1.2628589580352938, 'z': 0.4538412574197056, 'w': 0.8910825512059478},  # 收集區 C
            'zone_C': {'x': -0.26967466286678334, 'y': 1.256694463051006, 'z': 0.4538412574197056, 'w': 0.8910825512059478},  # 收集區 C
            'zone_D': {'x': -0.553839378973781,   'y': 0.4016348298561835, 'z': -0.32784161624025865, 'w': 0.9447327001120344},  # 收集區 D
            'zone_E': {'x': -1.3161517262450566,  'y': -0.0762408066354556, 'z': -0.906931073924803, 'w': 0.42127903715898757},  # 收集區 E
            'drop':   {'x': -1.092222432590772,   'y': 0.2193842132133244,  'z': 0.9366166378165709, 'w': 0.35035592440428687},  # 卸貨區 B
            'home':   {'x': -0.9486732805262716,  'y': 1.3108478091748508, 'z': 0.935114065832073, 'w': 0.354346841220589},  # 出發區
        }
        self.command_map = {
            0: 'home',
            1: 'drop',
            2: 'zone_C',
            3: 'zone_D',
            4: 'zone_E',
        }
        # 收集區順序
        # self.collection_zones = ['zone_C', 'zone_D', 'zone_E']

        # 訂閱main status
        self.create_subscription(
            Int32,
            'move_command',
            self.move_command_callback,
            10)

        # 發布底盤抵達通知給main
        self.arrived_pub = self.create_publisher(
            Bool,
            'area_arrived',
            10)

        self.get_logger().info('等待 Nav2...')
        self.navigator.waitUntilNav2Active()
        self.get_logger().info('系統就緒！')

    # ─────────────────────────────────────────
    # 收到
    # ─────────────────────────────────────────
    def move_command_callback(self, msg):
        command = msg.data
        pose_name = self.command_map.get(command)

        if pose_name is None:
            self.get_logger().warn(f'未知指令: {command}')
            return

        self._execute_move(pose_name)

    # ─────────────────────────────────────────
    # 離開當前點位前旋轉
    # ─────────────────────────────────────────
    def _spin(self, angle_rad):
        direction = '左' if angle_rad > 0 else '右'
        self.get_logger().info(f'旋轉 {math.degrees(angle_rad):.0f}° ({direction})')
        self.navigator.spin(spin_dist=angle_rad)
        while not self.navigator.isTaskComplete():
            rclpy.spin_once(self, timeout_sec=0.1)

    # ─────────────────────────────────────────
    # 執行移動並回報結果
    # ─────────────────────────────────────────
    def _execute_move(self, pose_name):
        self._moving = True

        # 離開收集區 → 旋轉 180°
        if self.current_pose in ['zone_C', 'zone_D', 'zone_E']:
            self._spin(math.pi)
        # 離開卸貨區 → 依下一目標決定轉向
        elif self.current_pose == 'drop':
            if pose_name == 'zone_E':
                self._spin(math.pi / 2)   # 向左轉 90°
            elif pose_name in ['zone_C', 'zone_D', 'home']:
                self._spin(-math.pi / 2)  # 向右轉 90°

        success = self.go_to(pose_name)
        if success:
            self.current_pose = pose_name
        self._moving = False

        msg = Bool()
        msg.data = success
        self.arrived_pub.publish(msg)
        self.get_logger().info(f'回報抵達 {pose_name}：{"成功" if success else "失敗"}')

    # ─────────────────────────────────────────
    # 前往目標
    # ─────────────────────────────────────────
    def go_to(self, pose_name):
        if pose_name not in self.poses:
            self.get_logger().error(f'找不到點位: {pose_name}')
            return False

        data = self.poses[pose_name]
        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.pose.position.x = data['x']
        goal.pose.position.y = data['y']
        goal.pose.position.z = 0.0
        goal.pose.orientation.x = 0.0
        goal.pose.orientation.y = 0.0
        goal.pose.orientation.z = data['z']
        goal.pose.orientation.w = data['w']

        self.get_logger().info(f'前往: {pose_name}')
        self.navigator.startToPose(goal)

        while not self.navigator.isTaskComplete():
            rclpy.spin_once(self, timeout_sec=0.1)

        result = self.navigator.getResult()
        if result == TaskResult.SUCCEEDED:
            self.get_logger().info(f'抵達: {pose_name}')
            return True
        else:
            self.get_logger().error(f'前往 {pose_name} 失敗')
            return False



def main(args=None):
    rclpy.init(args=args)
    node = MainNavigator()
        
    executor = MultiThreadedExecutor()  
    executor.add_node(node)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()


if __name__ == '__main__':
    main()