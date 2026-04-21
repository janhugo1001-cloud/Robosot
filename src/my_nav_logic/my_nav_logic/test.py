import rclpy
from rclpy.node import Node
from std_msgs.msg import Empty  # 使用 Empty 訊息作為啟動觸發
from geometry_msgs.msg import PoseStamped
from turtlebot4_navigation.turtlebot4_navigator import TurtleBot4Navigator
from nav2_simple_commander.robot_navigator import TaskResult

class RobosotNavigator(Node):
    def __init__(self):
        # 初始化節點名稱
        super().__init__('robosot_navigator')
        
        # 建立 TurtleBot 4 官方導航物件
        self.navigator = TurtleBot4Navigator()

        # 設定初始位置（對應地圖座標）
        initial_pose = PoseStamped()
        initial_pose.header.frame_id = 'map'
        initial_pose.header.stamp = self.navigator.get_clock().now().to_msg()
        initial_pose.pose.position.x = -0.9386494045386677   # 改成你的初始 x
        initial_pose.pose.position.y = 1.277936789986896     # 改成你的初始 y
        initial_pose.pose.orientation.z = -0.33640448449212706  # 改成你的初始方向 z
        initial_pose.pose.orientation.w = 0.9417175918573393 # 改成你的初始方向 w
        self.navigator.setInitialPose(initial_pose)

        # 等待 Nav2 啟動完成
        # self.navigator.waitUntilNav2Active()

        # 1. 儲存你提供的精確座標點位
        self.poses = {
            # 'collection': {'x': -1.2805948571953192, 'y': 0.15720600931875012, 'z': 0.6734428773529703, 'w': 0.7392392650167822},
            'Yellow':     {'x': -0.10172362347859836, 'y':  1.4667550656249191, 'z': -0.8668696785422898, 'w':  0.4985348136529556},
            'Blue':       {'x': -0.3466040623667577,  'y': 0.3069565634427828, 'z': 0.965692539349921,  'w': 0.2596881195663389},
            'Green':      {'x': -1.4166267959127819,  'y': -0.24598032484752137,  'z': 0.779861840495959, 'w': 0.6259516832298283},
            'home':       {'x': -0.9386494045386677, 'y': 1.277936789986896,   'z': -0.33640448449212706,  'w': 0.9417175918573393},
            'mid_home':   {'x': -0.45256698414358604, 'y': 0.1169877319935348,   'z': 0.7280504404627775,  'w': 0.6847411085703849},
            'mid_blue':   {'x': -0.9002539005133665 , 'y': 0.13337177017734375,   'z': -0.8433048666257228,  'w': 0.5374354862914915}
        }
        

        # 2. 設定訂閱器：收到 /start_navigation 指令才開始跑
        self.subscription = self.create_subscription(
            Empty,
            '/start_navigation',
            self.start_task_callback,
            10)

        self.get_logger().info('正在等待 Nav2 激活 (TurtleBot 4 模式)...')
        self.navigator.waitUntilNav2Active()
        self.get_logger().info('系統就緒！請發佈 /start_navigation 開始任務。')

    def build_goal_pose(self, pose_name):
        """將字典中的座標轉換為 PoseStamped 格式"""
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

    def start_task_callback(self, msg):
        """收到 Topic 訊號後執行的導航流程"""
        self.get_logger().info('收到指令！開始執行定點巡邏...')
        
        # 定義要跑的順序
        mission_list = [
            # 'collection', 
            'Yellow',
            # 'Blue', 
            # 'Green',
            # 'home'
        ]

        for target in mission_list:
            goal = self.build_goal_pose(target)
            if goal:
                self.get_logger().info(f'正在前往: {target}')
                self.navigator.startToPose(goal) # TB4 模組的發送指令方法

                # 監控任務是否完成
                while not self.navigator.isTaskComplete():
                    # 可以在這裡加入回饋資訊
                    rclpy.spin_once(self, timeout_sec=0.1)

                result = self.navigator.getResult()
                if result == TaskResult.SUCCEEDED:
                    self.get_logger().info(f'成功抵達: {target}')
                else:
                    self.get_logger().error(f'前往 {target} 失敗，任務中止。')
                    break

def main(args=None):
    rclpy.init(args=args)
    node = RobosotNavigator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()

if __name__ == '__main__':
    main()