import rclpy
from rclpy.node import Node
from std_msgs.msg import Empty, String
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, Twist
from turtlebot4_navigation.turtlebot4_navigator import TurtleBot4Navigator
from nav2_simple_commander.robot_navigator import TaskResult
import time
import math
from irobot_create_msgs.msg import InterfaceButtons
from rclpy.qos import qos_profile_sensor_data

class RobosotNavigator(Node):
    def __init__(self):
        super().__init__('color_navigator')

        self.navigator = TurtleBot4Navigator()
        self.detected_color = None
        self.is_running = False
        self._button_1_prev = False  # 用來偵測「按下瞬間」，避免持續觸發
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
            'Yellow':   {'x': -0.09738835252602348, 'y':  1.476164056506394, 'z': 0.41802214667533977, 'w': 0.9084368359379428},
            'Blue':     {'x': -0.35678440989914656,  'y':  0.2430117012679988, 'z': -0.327538265792149, 'w': 0.944837914375726},
            'Green':    {'x': -1.4647906883153903,  'y': -0.2941443871648671,'z': -0.8781641960626001, 'w': 0.47835932598176356},
            'home':     {'x': -0.9488927301114445,  'y': 1.3108478091748508,  'z': 0.935114065832073,   'w': 0.354346841220589},  # 出發區
            'mid':      {'x': -0.587605107252285, 'y':  1.0100325684944989, 'z':  -0.3050154752707838, 'w':  0.9523473945180603},
        }

        self.sub_buttons = self.create_subscription(
            InterfaceButtons,
            '/interface_buttons',
            self.interface_buttons_callback,
            qos_profile_sensor_data
        )

        self.sub_color = self.create_subscription(
            String, 'color_topic', self.color_callback, 10)

        self.current_yaw = 0.0

        self.cmd_vel_pub = self.create_publisher(
            Twist, '/cmd_vel', 10)

        self.sub_odom = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10)

        self.reset_pub = self.create_publisher(
            Empty, '/reset_color_detection', 10)

        self.get_logger().info('正在等待 Nav2 激活...')
        self.navigator.waitUntilNav2Active()
        self.get_logger().info('系統就緒！請開始任務。')

    def interface_buttons_callback(self, msg: InterfaceButtons):
        # 偵測 Button 1 的「上升沿」（按下瞬間才觸發，不會一直重複）
        pressed_now = msg.button_1.is_pressed
        if pressed_now and not self._button_1_prev:
            self.get_logger().info('Button 1 按下，觸發導航任務！')
            self.start_task_callback(None)   # 直接呼叫原本的任務函式
        self._button_1_prev = pressed_now

    def build_goal_pose(self, pose_name):
        if pose_name not in self.poses:
            self.get_logger().error(f'找不到點位名稱: {pose_name}')
            return None

        data = self.poses[pose_name]
        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.pose.position.x = data['x']
        goal.pose.position.y = data['y']
        goal.pose.orientation.z = data['z']
        goal.pose.orientation.w = data['w']
        return goal

    def odom_callback(self, msg):
        q = msg.pose.pose.orientation
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.current_yaw = math.atan2(siny, cosy)

    def rotate_180(self, speed=0.5, direction='left'):
        self.get_logger().info('開始原地旋轉 180°')
        
        # 180° = π rad，時間 = π / speed
        duration = math.pi / speed
        twist = Twist()
        twist.angular.z = speed
        twist.angular.z = speed if direction == 'left' else -speed

        start = self.get_clock().now().nanoseconds / 1e9
        while True:
            elapsed = self.get_clock().now().nanoseconds / 1e9 - start
            if elapsed >= duration:
                break
            self.cmd_vel_pub.publish(twist)
            rclpy.spin_once(self, timeout_sec=0.05)

        # 停止
        twist.angular.z = 0.0
        self.cmd_vel_pub.publish(twist)
        self.get_logger().info('旋轉 180° 完成')

    def color_callback(self, msg):
        self.detected_color = msg.data.split(',')
        self.get_logger().info(f'收到顏色順序：{self.detected_color}')

    def wait_for_color(self, timeout=10.0):
        self.get_logger().info('等待顏色...')
        start = self.get_clock().now().nanoseconds / 1e9

        while self.detected_color is None:
            elapsed = self.get_clock().now().nanoseconds / 1e9 - start
            if elapsed > timeout:
                self.get_logger().warn('等待顏色超時！使用預設順序')
                return ['Green', 'Yellow', 'Blue']
            rclpy.spin_once(self, timeout_sec=0.1)  # 這裡需要 spin_once 才能收到顏色

        self.get_logger().info(f'顏色確認：{self.detected_color}')
        return self.detected_color

    def go_to_and_wait(self, pose_name, wait_sec=0.0, rotate=True ,direction='left'):  
        goal = self.build_goal_pose(pose_name)
        if goal is None:
            return False

        self.get_logger().info(f'正在前往: {pose_name}')
        self.navigator.startToPose(goal)

        while not self.navigator.isTaskComplete():
            rclpy.spin_once(self, timeout_sec=0.1)

        result = self.navigator.getResult()
        if result == TaskResult.SUCCEEDED:
            self.get_logger().info(f'成功抵達: {pose_name}')
            if rotate:                  
                self.rotate_180(direction=direction)
            if wait_sec > 0:
                self.get_logger().info(f'停留 {wait_sec} 秒...')
                time.sleep(wait_sec)
                self.get_logger().info('停留結束，繼續任務')
            return True
        else:
            self.get_logger().error(f'前往 {pose_name} 失敗，任務中止。')
            return False

    def start_task_callback(self, msg):
        if self.is_running:
            self.get_logger().warn('任務執行中，忽略此次指令。')
            return

        self.is_running = True
        self.get_logger().info('收到指令！開始任務...')
        rotation_dir = {
            'Yellow': 'left',
            'Blue':   'right',
            'Green':  'right',
        }
        try:
            if not self.go_to_and_wait('mid', wait_sec=0.0, rotate=False):  
                return  
           
            self.get_logger().info('抵達 mid，開始偵測顏色...')
            color_order = self.wait_for_color(timeout=10.0)
            self.reset_pub.publish(Empty())

            for color in color_order:
                direction = rotation_dir.get(color, 'left')  # 預設左轉
                if not self.go_to_and_wait(color, wait_sec=2.0, direction=direction):
                    return

            self.go_to_and_wait('home', wait_sec=2.0, rotate=False)
            self.get_logger().info('任務完成！')

        finally:
            self.detected_color = None
            self.get_logger().info('任務結束。')


def main(args=None):
    rclpy.init(args=args)
    node = RobosotNavigator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()