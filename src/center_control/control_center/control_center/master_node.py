import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Int32
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import PointStamped


class MasterNode(Node):

    def __init__(self, timeout_multiplier=1.0):
        super().__init__('master_node')

        self.state = 'INIT'
        self.picked_list = []
        self.holding = None
        self.zone_order = [2, 3, 4]
        self.zone_index = 0
        self.current_zone = 2
        self.detection_result = None
        self.placement_result = None

        self._timeout_multiplier = timeout_multiplier
        self._timeout_timer = None
        self._placement_retry = 0
        self._last_grab_target = None
        self._last_place_target = None
        self._initialized = False
        self._repeat_timer = None

        # Publishers
        self.pub_move = self.create_publisher(Int32, 'move_command', 10)
        self.pub_arm_detect = self.create_publisher(Int32, 'arm_to_detect', 10)
        self.pub_grab = self.create_publisher(PointStamped, 'grab_target', 10)
        self.pub_place = self.create_publisher(PointStamped, 'place_target', 10)
        self.pub_start_detect = self.create_publisher(Bool, 'start_detection', 10)
        self.pub_start_placement = self.create_publisher(Bool, 'start_placement_detection', 10)

        # Subscribers
        self.create_subscription(Bool, 'area_arrived', self.area_arrived_callback, 10)
        self.create_subscription(Bool, 'arm_ready', self.arm_ready_callback, 10)
        self.create_subscription(Bool, 'grab_done', self.grab_done_callback, 10)
        self.create_subscription(Bool, 'place_done', self.place_done_callback, 10)
        self.create_subscription(
            Float32MultiArray, 'task_topic', self.task_topic_callback, 10)
        self.create_subscription(Bool, 'detection_done', self.detection_done_callback, 10)
        self.create_subscription(
            Float32MultiArray, 'placement_topic', self.placement_topic_callback, 10)
        self.create_subscription(Bool, 'placement_done', self.placement_done_callback, 10)

        # 延遲啟動，讓 publisher/subscriber 連線完成
        self._init_timer = self.create_timer(0.5, self._init_once)

    # ── 初始化 ───────────────────────────────────────────────

    def _init_once(self):
        if self._initialized:
            return
        self._initialized = True
        self._init_timer.cancel()

        self.current_zone = self.zone_order[self.zone_index]
        self.get_logger().info(f'狀態切換: BOOT → INIT')
        self._publish_move(self.current_zone)
        self._transition('WAIT_ARRIVE_COLLECT')
        self._start_timeout(60)

    # ── 狀態轉移 ─────────────────────────────────────────────

    def _transition(self, new_state):
        old_state = self.state
        self.state = new_state
        self.get_logger().info(f'狀態切換: {old_state} → {new_state}')

    # ── 超時機制 ─────────────────────────────────────────────

    def _start_timeout(self, seconds):
        self._cancel_timeout()
        actual = seconds * self._timeout_multiplier
        self._timeout_timer = self.create_timer(actual, self._on_timeout)

    def _cancel_timeout(self):
        if self._timeout_timer:
            self._timeout_timer.cancel()
            self._timeout_timer = None

    def _on_timeout(self):
        self.get_logger().warn(f'狀態 {self.state} 超時！')
        self._cancel_timeout()

        if self.state == 'WAIT_DETECTION':
            self.get_logger().warn('偵測超時，視為此區無物件，前往下一區')
            self._next_zone_or_home()

        elif self.state == 'WAIT_PLACEMENT_DETECT':
            if self._placement_retry < 1:
                self._placement_retry += 1
                self.get_logger().warn(f'放置偵測超時，重試第 {self._placement_retry} 次')
                self.pub_start_placement.publish(Bool(data=True))
                self._start_timeout(15)
            else:
                self.get_logger().error('放置偵測超時，放棄此次放置')
                self._placement_retry = 0
                self.holding = None
                self._transition('CHECK_DONE')
                self._run_check_done()

        elif self.state == 'WAIT_GRAB':
            self.get_logger().error('抓取超時，重新發送抓取指令')
            if self._last_grab_target:
                self._publish_grab(*self._last_grab_target)
                self._start_timeout(30)

        elif self.state == 'WAIT_PLACE':
            self.get_logger().error('放置超時，重新發送放置指令')
            if self._last_place_target:
                self._publish_place(*self._last_place_target)
                self._start_timeout(30)

        elif self.state in ('WAIT_ARRIVE_COLLECT', 'WAIT_ARRIVE_PLACE', 'WAIT_HOME'):
            self.get_logger().error(f'底盤移動超時（狀態: {self.state}），底盤可能卡住')

        elif self.state == 'WAIT_ARM_COLLECT' or self.state == 'WAIT_ARM_PLACE':
            self.get_logger().error(f'手臂移動超時（狀態: {self.state}）')

    # ── Publishers ───────────────────────────────────────────

    def _publish_move(self, zone):
        msg = Int32()
        msg.data = zone
        self.pub_move.publish(msg)
        self.get_logger().info(f'發佈 move_command = {zone}')

    def _publish_arm_to_detect(self, pos):
        msg = Int32()
        msg.data = pos
        self.pub_arm_detect.publish(msg)
        label = '收集區' if pos == 0 else '放置區'
        self.get_logger().info(f'發佈 arm_to_detect = {pos} ({label})')

    def _publish_grab(self, x, y, z):
        msg = PointStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'camera_frame'
        msg.point.x = float(x)
        msg.point.y = float(y)
        msg.point.z = float(z)
        self.pub_grab.publish(msg)
        self.get_logger().info(f'發佈 grab_target = [{x:.3f}, {y:.3f}, {z:.3f}] (camera_frame)')

    def _publish_place(self, x, y, z):
        msg = PointStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'camera_frame'
        msg.point.x = float(x)
        msg.point.y = float(y)
        msg.point.z = float(z)
        self.pub_place.publish(msg)
        self.get_logger().info(f'發佈 place_target = [{x:.3f}, {y:.3f}, {z:.3f}] (camera_frame)')

    def _start_repeat_grab(self, x, y, z):
        self._cancel_repeat()
        self._publish_grab(x, y, z)
        self._repeat_timer = self.create_timer(0.5, lambda: self._publish_grab(x, y, z))

    def _start_repeat_place(self, x, y, z):
        self._cancel_repeat()
        self._publish_place(x, y, z)
        self._repeat_timer = self.create_timer(0.5, lambda: self._publish_place(x, y, z))

    def _cancel_repeat(self):
        if self._repeat_timer:
            self._repeat_timer.cancel()
            self._repeat_timer = None

    # ── Subscriber Callbacks ─────────────────────────────────

    def area_arrived_callback(self, msg):
        if not msg.data:
            return
        self._cancel_timeout()

        if self.state == 'WAIT_ARRIVE_COLLECT':
            self.get_logger().info(f'到達收集區 {self.current_zone}')
            self._publish_arm_to_detect(0)
            self._transition('WAIT_ARM_COLLECT')
            self._start_timeout(30)

        elif self.state == 'WAIT_ARRIVE_PLACE':
            self.get_logger().info('到達放置區 B')
            self._publish_arm_to_detect(1)
            self._transition('WAIT_ARM_PLACE')
            self._start_timeout(30)

        elif self.state == 'WAIT_HOME':
            self.get_logger().info('回到出發區 A，任務完成！')
            self._transition('DONE')

    def arm_ready_callback(self, msg):
        if not msg.data:
            return
        self._cancel_timeout()

        if self.state == 'WAIT_ARM_COLLECT':
            self.get_logger().info('手臂到位（收集區），開始偵測')
            self.pub_start_detect.publish(Bool(data=True))
            self._transition('WAIT_DETECTION')
            self._start_timeout(15)

        elif self.state == 'WAIT_ARM_PLACE':
            self.get_logger().info('手臂到位（放置區），開始偵測放置位置')
            self._placement_retry = 0
            self.pub_start_placement.publish(Bool(data=True))
            self._transition('WAIT_PLACEMENT_DETECT')
            self._start_timeout(15)

    def task_topic_callback(self, msg):
        self.detection_result = list(msg.data)

    def detection_done_callback(self, msg):
        if not msg.data:
            return
        if self.state != 'WAIT_DETECTION':
            return
        self._cancel_timeout()
        self.get_logger().info('收集區偵測完成，決定目標')
        self._transition('DECIDE_TARGET')
        self._run_decide_target()

    def placement_topic_callback(self, msg):
        self.placement_result = list(msg.data)

    def placement_done_callback(self, msg):
        if not msg.data:
            return
        if self.state != 'WAIT_PLACEMENT_DETECT':
            return
        self._cancel_timeout()
        self._placement_retry = 0
        self.get_logger().info('放置區偵測完成，決定放置位置')
        self._transition('DECIDE_PLACE')
        self._run_decide_place()

    def grab_done_callback(self, msg):
        if not msg.data:
            return
        if self.state != 'WAIT_GRAB':
            return
        self._cancel_timeout()
        self._cancel_repeat()
        self.get_logger().info(f'抓取完成（ID={self.holding}），前往放置區 B')
        self._publish_move(1)
        self._transition('WAIT_ARRIVE_PLACE')
        self._start_timeout(60)

    def place_done_callback(self, msg):
        if not msg.data:
            return
        if self.state != 'WAIT_PLACE':
            return
        self._cancel_timeout()
        self._cancel_repeat()
        self.get_logger().info(f'放置完成（ID={self.holding}）')
        self.holding = None
        self._transition('CHECK_DONE')
        self._run_check_done()

    # ── 立即執行的邏輯狀態 ────────────────────────────────────

    def _run_decide_target(self):
        detections = self.parse_detection(self.detection_result or [])
        target = self.select_target(detections)

        if target:
            obj_id, x, y, z = target
            self.holding = obj_id
            self.picked_list.append(obj_id)
            self._last_grab_target = (x, y, z)
            self.get_logger().info(
                f'選中目標: ID={obj_id}, X={x:.3f}, Y={y:.3f}, Z={z:.3f}')
            self._start_repeat_grab(x, y, z)
            self._transition('WAIT_GRAB')
            self._start_timeout(30)
        else:
            self.get_logger().info(f'區 {self.current_zone} 無可抓取目標，前往下一區')
            self._next_zone_or_home()

    def _run_decide_place(self):
        detections = self.parse_detection(self.placement_result or [])
        coords = self.find_placement(detections, self.holding)

        if coords:
            x, y, z = coords
            self._last_place_target = coords
            self.get_logger().info(
                f'放置位置: ID={self.holding}, X={x:.3f}, Y={y:.3f}, Z={z:.3f}')
            self._start_repeat_place(x, y, z)
            self._transition('WAIT_PLACE')
            self._start_timeout(30)
        else:
            self.get_logger().error(f'找不到 ID={self.holding} 的放置位置，跳過')
            self.holding = None
            self._transition('CHECK_DONE')
            self._run_check_done()

    def _run_check_done(self):
        self.get_logger().info(
            f'已抓取: {self.picked_list}（共 {len(self.picked_list)} 個）')

        if len(self.picked_list) >= 4:
            self.get_logger().info('全部完成，返回出發區 A')
            self._publish_move(0)
            self._transition('WAIT_HOME')
            self._start_timeout(60)
        else:
            self.get_logger().info(f'繼續收集，回到區 {self.current_zone}')
            self._publish_move(self.current_zone)
            self._transition('WAIT_ARRIVE_COLLECT')
            self._start_timeout(60)

    def _next_zone_or_home(self):
        self.zone_index += 1
        if self.zone_index < len(self.zone_order):
            self.current_zone = self.zone_order[self.zone_index]
            self.get_logger().info(f'移往下一收集區 {self.current_zone}')
            self._publish_move(self.current_zone)
            self._transition('WAIT_ARRIVE_COLLECT')
            self._start_timeout(60)
        else:
            self.get_logger().warn('所有收集區均已巡訪，返回出發區 A')
            self._publish_move(0)
            self._transition('WAIT_HOME')
            self._start_timeout(60)

    # ── 解析與選擇輔助方法 ────────────────────────────────────

    def parse_detection(self, data):
        """
        輸入: [count, id0, X0, Y0, Z0, id1, X1, Y1, Z1, ...]
        輸出: [(id, X, Y, Z), ...]
        """
        if len(data) < 1:
            return []
        count = int(data[0])
        result = []
        for i in range(count):
            idx = 1 + i * 4
            if idx + 3 >= len(data):
                break
            result.append((int(data[idx]), data[idx + 1], data[idx + 2], data[idx + 3]))
        return result

    def select_target(self, detections):
        """
        排除 picked_list，優先 Y 最小（距手臂近），Y 相同取 X 最小
        """
        available = [d for d in detections if d[0] not in self.picked_list]
        if not available:
            return None
        available.sort(key=lambda d: (d[2], d[1]))  # (Y, X)
        return available[0]

    def find_placement(self, detections, target_id):
        """
        從放置區偵測結果中找 target_id 對應的座標
        """
        for obj_id, x, y, z in detections:
            if obj_id == target_id:
                return (x, y, z)
        return None


def main(args=None):
    rclpy.init(args=args)
    node = MasterNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
