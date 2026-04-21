import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Int32
from std_msgs.msg import Float32MultiArray

RESPOND_DELAY = 0.5  # 模擬偵測時間（秒）

# 每個收集區的假物件，分散在 3 個 zone 確保都會跑到
# 格式：[count, id, X, Y, Z, ...]
ZONE_DATA = {
    2: [2.0,
        5.0,   0.10,  0.05, 0.30,   # F
        17.0, -0.08,  0.12, 0.35],  # R
    3: [1.0,
        19.0,  0.08,  0.08, 0.30],  # I
    4: [1.0,
        0.0,   0.02,  0.15, 0.28],  # A
}

ZONE_EMPTY = [0.0]

# 放置區假資料（固定）
PLACEMENT_DATA = [
    4.0,
    5.0,   0.05,  0.03, 0.25,   # F
    19.0,  0.10,  0.03, 0.25,   # I
    17.0,  0.15,  0.03, 0.25,   # R
    0.0,   0.20,  0.03, 0.25,   # A
]


class FakeCamera(Node):

    def __init__(self):
        super().__init__('fake_camera')

        self._current_zone = 2

        self.pub_task = self.create_publisher(Float32MultiArray, 'task_topic', 10)
        self.pub_detect_done = self.create_publisher(Bool, 'detection_done', 10)
        self.pub_placement = self.create_publisher(Float32MultiArray, 'placement_topic', 10)
        self.pub_placement_done = self.create_publisher(Bool, 'placement_done', 10)

        self.create_subscription(Int32, 'move_command', self._on_move_command, 10)
        self.create_subscription(Bool, 'start_detection', self._on_start_detection, 10)
        self.create_subscription(
            Bool, 'start_placement_detection', self._on_start_placement, 10)

        self.get_logger().info('假相機啟動，等待偵測指令...')

    def _fire_once(self, delay, fn):
        holder = [None]

        def cb():
            holder[0].cancel()
            holder[0] = None
            fn()

        holder[0] = self.create_timer(delay, cb)

    def _on_move_command(self, msg):
        self._current_zone = msg.data

    def _on_start_detection(self, msg):
        if not msg.data:
            return
        zone = self._current_zone
        data = ZONE_DATA.get(zone, ZONE_EMPTY)
        count = int(data[0])
        self.get_logger().info(
            f'[相機] 收集區偵測，zone={zone}，物件數={count}，{RESPOND_DELAY}s 後回傳')

        def reply():
            self.pub_task.publish(Float32MultiArray(data=[float(x) for x in data]))
            self.pub_detect_done.publish(Bool(data=True))
            self.get_logger().info(f'[相機] zone={zone} 偵測完成，已發布 task_topic + detection_done')

        self._fire_once(RESPOND_DELAY, reply)

    def _on_start_placement(self, msg):
        if not msg.data:
            return
        self.get_logger().info(f'[相機] 放置區偵測，{RESPOND_DELAY}s 後回傳')

        def reply():
            self.pub_placement.publish(Float32MultiArray(data=PLACEMENT_DATA))
            self.pub_placement_done.publish(Bool(data=True))
            self.get_logger().info('[相機] 放置區偵測完成，已發布 placement_topic + placement_done')

        self._fire_once(RESPOND_DELAY, reply)


def main(args=None):
    rclpy.init(args=args)
    node = FakeCamera()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()