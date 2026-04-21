"""
測試主控 Node 狀態機

場景 1：基本流程      C=F,R  D=I  E=A  → 依序抓 4 個，回 A
場景 2：一區多個      C=F,I,R  D=A  → C 抓 3 個、D 抓 1 個，回 A
場景 3：空區          C=空  D=F,I,R,A → 跳過 C，D 抓 4 個，回 A
場景 4：偵測超時      C 視覺不回應 → 15s 超時後跳到 D
"""

import threading
import time

import pytest
import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import Bool, Int32
from std_msgs.msg import Float32MultiArray

from control_center.master_node import MasterNode

# ── 放置區假資料（固定）──────────────────────────────────────

PLACEMENT_DATA = [
    4.0,
    5.0,  0.05, 0.03, 0.25,   # F
    19.0, 0.10, 0.03, 0.25,   # I
    17.0, 0.15, 0.03, 0.25,   # R
    0.0,  0.20, 0.03, 0.25,   # A
]

# zone_id → Float32MultiArray.data（空區用 [0.0]）
ZONE_EMPTY = [0.0]

def make_zone_data(*objects):
    """
    objects: [(id, X, Y, Z), ...]
    回傳 Float32MultiArray.data 格式
    """
    data = [float(len(objects))]
    for obj_id, x, y, z in objects:
        data += [float(obj_id), float(x), float(y), float(z)]
    return data


# ── FakeNodes ────────────────────────────────────────────────

class FakeNodes(Node):
    """
    模擬底盤、手臂、視覺，收到主控指令後延遲回應。
    zone_detect_data: dict[int, list[float]]  zone_id → 偵測資料
    respond_detection: bool  False = 不回應 start_detection（測超時用）
    """

    def __init__(self, zone_detect_data, respond_detection=True):
        super().__init__('fake_nodes')

        self._zone_data = zone_detect_data
        self._respond_detection = respond_detection
        self._current_zone = 0

        # 訂閱主控指令
        self.create_subscription(Int32, 'move_command', self._on_move, 10)
        self.create_subscription(Int32, 'arm_to_detect', self._on_arm_detect, 10)
        self.create_subscription(
            Float32MultiArray, 'grab_target', self._on_grab, 10)
        self.create_subscription(
            Float32MultiArray, 'place_target', self._on_place, 10)
        self.create_subscription(Bool, 'start_detection', self._on_start_detect, 10)
        self.create_subscription(
            Bool, 'start_placement_detection', self._on_start_placement, 10)

        # 回應主控的 publisher
        self.pub_arrived = self.create_publisher(Bool, 'area_arrived', 10)
        self.pub_arm_ready = self.create_publisher(Bool, 'arm_ready', 10)
        self.pub_grab_done = self.create_publisher(Bool, 'grab_done', 10)
        self.pub_place_done = self.create_publisher(Bool, 'place_done', 10)
        self.pub_task = self.create_publisher(Float32MultiArray, 'task_topic', 10)
        self.pub_detect_done = self.create_publisher(Bool, 'detection_done', 10)
        self.pub_placement = self.create_publisher(Float32MultiArray, 'placement_topic', 10)
        self.pub_placement_done = self.create_publisher(Bool, 'placement_done', 10)

    # ── 一次性 timer helper ───────────────────────────────────

    def _fire_once(self, delay, fn):
        holder = [None]

        def cb():
            if holder[0] is not None:
                holder[0].cancel()
                holder[0] = None
            fn()

        holder[0] = self.create_timer(delay, cb)

    # ── 回應邏輯 ──────────────────────────────────────────────

    def _on_move(self, msg):
        self._current_zone = msg.data
        self.get_logger().info(f'[底盤] 收到移動指令: 區 {msg.data}')
        self._fire_once(0.3, lambda: self.pub_arrived.publish(Bool(data=True)))

    def _on_arm_detect(self, msg):
        label = '收集區' if msg.data == 0 else '放置區'
        self.get_logger().info(f'[手臂] 移至{label}偵測位')
        self._fire_once(0.2, lambda: self.pub_arm_ready.publish(Bool(data=True)))

    def _on_grab(self, msg):
        self.get_logger().info(f'[手臂] 抓取目標: {list(msg.data)}')
        self._fire_once(0.3, lambda: self.pub_grab_done.publish(Bool(data=True)))

    def _on_place(self, msg):
        self.get_logger().info(f'[手臂] 放置目標: {list(msg.data)}')
        self._fire_once(0.3, lambda: self.pub_place_done.publish(Bool(data=True)))

    def _on_start_detect(self, msg):
        if not msg.data:
            return
        if not self._respond_detection:
            self.get_logger().warn('[視覺] 故意不回應（測超時用）')
            return

        zone = self._current_zone
        data = self._zone_data.get(zone, ZONE_EMPTY)
        self.get_logger().info(f'[視覺] 收集區偵測，區={zone}，物件數={int(data[0])}')

        def reply():
            self.pub_task.publish(Float32MultiArray(data=[float(x) for x in data]))
            self.pub_detect_done.publish(Bool(data=True))

        self._fire_once(0.3, reply)

    def _on_start_placement(self, msg):
        if not msg.data:
            return
        self.get_logger().info('[視覺] 放置區偵測')

        def reply():
            self.pub_placement.publish(
                Float32MultiArray(data=[float(x) for x in PLACEMENT_DATA]))
            self.pub_placement_done.publish(Bool(data=True))

        self._fire_once(0.3, reply)


# ── 測試輔助 ─────────────────────────────────────────────────

def _wait_for_done(master: MasterNode, timeout_sec: float) -> bool:
    """等待主控進入 DONE 狀態，回傳是否在時限內完成"""
    deadline = time.time() + timeout_sec
    while time.time() < deadline:
        if master.state == 'DONE':
            return True
        time.sleep(0.05)
    return False


def _run_scenario(zone_data, timeout_multiplier=1.0,
                  respond_detection=True, wait_sec=30.0):
    """
    建立主控 + FakeNodes，執行完整場景，回傳 (master, success)
    """
    rclpy.init()
    try:
        master = MasterNode(timeout_multiplier=timeout_multiplier)
        fake = FakeNodes(zone_data, respond_detection=respond_detection)

        executor = MultiThreadedExecutor()
        executor.add_node(master)
        executor.add_node(fake)

        spin_thread = threading.Thread(target=executor.spin, daemon=True)
        spin_thread.start()

        success = _wait_for_done(master, wait_sec)

        executor.shutdown(timeout_sec=2)
        spin_thread.join(timeout=3)
        return master, success
    finally:
        rclpy.shutdown()


# ── 場景 1：基本流程 ─────────────────────────────────────────

def test_scenario_1_basic_flow():
    """
    C=F(5),R(17)  D=I(19)  E=A(0)
    預期：依序抓 F→R→I→A，picked_list=[5,17,19,0]，最後回 A
    """
    zone_data = {
        2: make_zone_data((5, 0.10, 0.05, 0.30), (17, -0.08, 0.12, 0.35)),  # C
        3: make_zone_data((19, 0.05, 0.08, 0.30)),                            # D
        4: make_zone_data((0, 0.02, 0.03, 0.28)),                             # E
    }
    master, success = _run_scenario(zone_data, wait_sec=30.0)

    assert success, f'未在時限內完成，最終狀態: {master.state}'
    assert master.state == 'DONE'
    assert set(master.picked_list) == {5, 17, 19, 0}, \
        f'picked_list 不符: {master.picked_list}'
    # 確認 Y 最小的 F(Y=0.05) 比 R(Y=0.12) 先抓
    assert master.picked_list.index(5) < master.picked_list.index(17), \
        'F 應比 R 先被抓（Y 值較小）'


# ── 場景 2：一區多個 ─────────────────────────────────────────

def test_scenario_2_multiple_in_one_zone():
    """
    C=F(5),I(19),R(17)  D=A(0)
    預期：C 抓 3 個，D 抓 1 個，共 4 個後回 A
    """
    zone_data = {
        2: make_zone_data(
            (5,  0.10, 0.05, 0.30),
            (19, 0.08, 0.08, 0.30),
            (17, -0.08, 0.12, 0.35),
        ),  # C
        3: make_zone_data((0, 0.02, 0.03, 0.28)),   # D
        4: ZONE_EMPTY,                               # E（不會到達）
    }
    master, success = _run_scenario(zone_data, wait_sec=40.0)

    assert success, f'未在時限內完成，最終狀態: {master.state}'
    assert master.state == 'DONE'
    assert len(master.picked_list) == 4, f'應抓 4 個: {master.picked_list}'
    assert set(master.picked_list) == {5, 19, 17, 0}


# ── 場景 3：空區 ─────────────────────────────────────────────

def test_scenario_3_empty_zone():
    """
    C=空  D=F(5),I(19),R(17),A(0)
    預期：C 偵測無物件 → 跳到 D → 抓 4 個 → 回 A
    """
    zone_data = {
        2: ZONE_EMPTY,                               # C 空
        3: make_zone_data(
            (5,  0.10, 0.05, 0.30),
            (19, 0.08, 0.08, 0.30),
            (17, -0.08, 0.12, 0.35),
            (0,  0.02, 0.15, 0.28),
        ),  # D
        4: ZONE_EMPTY,
    }
    master, success = _run_scenario(zone_data, wait_sec=60.0)

    assert success, f'未在時限內完成，最終狀態: {master.state}'
    assert master.state == 'DONE'
    assert len(master.picked_list) == 4, f'應抓 4 個: {master.picked_list}'
    # zone_index 應已推進到 D (index=1)
    assert master.zone_index >= 1, '應已跳過 C 區'


# ── 場景 4：偵測超時 ─────────────────────────────────────────

def test_scenario_4_detection_timeout():
    """
    視覺不回應 start_detection
    使用 timeout_multiplier=0.1 → 15s → 1.5s
    預期：C 區超時後跳到 D，D 有 F,I,R,A 並完成任務
    """
    zone_data = {
        2: ZONE_EMPTY,   # C：即使假節點有資料也不會發（respond_detection=False）
        3: make_zone_data(
            (5,  0.10, 0.05, 0.30),
            (19, 0.08, 0.08, 0.30),
            (17, -0.08, 0.12, 0.35),
            (0,  0.02, 0.15, 0.28),
        ),
        4: ZONE_EMPTY,
    }

    rclpy.init()
    try:
        # timeout_multiplier=0.1：15s→1.5s, 30s→3s, 60s→6s
        master = MasterNode(timeout_multiplier=0.1)

        # C 區不回應偵測，D 區正常
        class SelectiveFake(FakeNodes):
            def _on_start_detect(self, msg):
                if self._current_zone == 2:
                    self.get_logger().warn('[視覺] C 區故意不回應')
                    return
                super()._on_start_detect(msg)

        fake = SelectiveFake(zone_data)

        executor = MultiThreadedExecutor()
        executor.add_node(master)
        executor.add_node(fake)

        spin_thread = threading.Thread(target=executor.spin, daemon=True)
        spin_thread.start()

        success = _wait_for_done(master, timeout_sec=30.0)

        executor.shutdown(timeout_sec=2)
        spin_thread.join(timeout=3)
    finally:
        rclpy.shutdown()

    assert success, f'未在時限內完成，最終狀態: {master.state}'
    assert master.state == 'DONE'
    assert master.zone_index >= 1, '應已因超時跳過 C 區'
    assert len(master.picked_list) == 4


# ── 單元測試：parse / select / find ──────────────────────────

class TestHelpers:
    """直接測試 MasterNode 的輔助方法，不需要 ROS2 spin"""

    def setup_method(self):
        rclpy.init()
        self.node = MasterNode()

    def teardown_method(self):
        self.node.destroy_node()
        rclpy.shutdown()

    def test_parse_detection_basic(self):
        data = [2.0, 5.0, 0.1, 0.05, 0.3, 17.0, -0.08, 0.12, 0.35]
        result = self.node.parse_detection(data)
        assert result == [(5, 0.1, 0.05, 0.3), (17, -0.08, 0.12, 0.35)]

    def test_parse_detection_empty(self):
        assert self.node.parse_detection([0.0]) == []
        assert self.node.parse_detection([]) == []

    def test_select_target_priority(self):
        """Y 最小優先；Y 相同取 X 最小"""
        detections = [
            (17, -0.08, 0.12, 0.35),
            (5,   0.10, 0.05, 0.30),
        ]
        target = self.node.select_target(detections)
        assert target[0] == 5  # Y=0.05 < Y=0.12

    def test_select_target_excludes_picked(self):
        self.node.picked_list = [5]
        detections = [(5, 0.1, 0.05, 0.3), (17, -0.08, 0.12, 0.35)]
        target = self.node.select_target(detections)
        assert target[0] == 17

    def test_select_target_all_picked(self):
        self.node.picked_list = [5, 17]
        detections = [(5, 0.1, 0.05, 0.3), (17, -0.08, 0.12, 0.35)]
        assert self.node.select_target(detections) is None

    def test_find_placement_found(self):
        detections = [(5, 0.05, 0.03, 0.25), (17, 0.15, 0.03, 0.25)]
        assert self.node.find_placement(detections, 17) == (0.15, 0.03, 0.25)

    def test_find_placement_not_found(self):
        detections = [(5, 0.05, 0.03, 0.25)]
        assert self.node.find_placement(detections, 99) is None
