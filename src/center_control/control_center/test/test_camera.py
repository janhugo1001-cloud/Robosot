"""
相機 topic 測試

驗證主控觸發偵測的時機、次數，以及正確解析偵測資料。

訂閱（主控發給相機）: start_detection (Bool), start_placement_detection (Bool)
發布（相機回給主控）: task_topic (Float32MultiArray), detection_done (Bool),
                       placement_topic (Float32MultiArray), placement_done (Bool)

Stub 節點自動處理底盤與手臂 topic。
"""

import threading
import time
from dataclasses import dataclass, field

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import Bool, Int32
from std_msgs.msg import Float32MultiArray

from control_center.master_node import MasterNode

# ── 測試資料 ─────────────────────────────────────────────────

ZONE_C = [2.0, 5.0, 0.10, 0.05, 0.30, 17.0, -0.08, 0.12, 0.35]   # F, R
ZONE_D = [1.0, 19.0, 0.05, 0.08, 0.30]                              # I
ZONE_E = [1.0, 0.0,  0.02, 0.03, 0.28]                              # A
ZONE_EMPTY = [0.0]

PLACEMENT = [
    4.0,
    5.0,  0.05, 0.03, 0.25,
    19.0, 0.10, 0.03, 0.25,
    17.0, 0.15, 0.03, 0.25,
    0.0,  0.20, 0.03, 0.25,
]


# ── 相機 Capture Node ────────────────────────────────────────

@dataclass
class DetectEvent:
    kind: str       # 'collect' | 'placement'
    zone: int       # 觸發時的 zone（collect 用）


class CameraCapture(Node):
    """
    記錄偵測觸發事件，並依 zone_data 回傳對應資料。
    """

    def __init__(self, zone_data: dict):
        super().__init__('camera_capture')
        self._zone_data = zone_data
        self._current_zone = 0
        self.events: list[DetectEvent] = []
        self._lock = threading.Lock()

        self.pub_task = self.create_publisher(Float32MultiArray, 'task_topic', 10)
        self.pub_detect_done = self.create_publisher(Bool, 'detection_done', 10)
        self.pub_placement = self.create_publisher(
            Float32MultiArray, 'placement_topic', 10)
        self.pub_placement_done = self.create_publisher(Bool, 'placement_done', 10)

        self.create_subscription(Int32, 'move_command', self._on_move, 10)
        self.create_subscription(Bool, 'start_detection', self._on_detect, 10)
        self.create_subscription(
            Bool, 'start_placement_detection', self._on_placement, 10)

    def _fire_once(self, delay, fn):
        holder = [None]

        def cb():
            if holder[0]:
                holder[0].cancel()
                holder[0] = None
            fn()

        holder[0] = self.create_timer(delay, cb)

    def _on_move(self, msg: Int32):
        self._current_zone = msg.data

    def _on_detect(self, msg: Bool):
        if not msg.data:
            return
        zone = self._current_zone
        with self._lock:
            self.events.append(DetectEvent('collect', zone))
        self.get_logger().info(f'[相機] start_detection，區={zone}')

        data = self._zone_data.get(zone, ZONE_EMPTY)

        def reply():
            self.pub_task.publish(Float32MultiArray(data=[float(x) for x in data]))
            self.pub_detect_done.publish(Bool(data=True))

        self._fire_once(0.3, reply)

    def _on_placement(self, msg: Bool):
        if not msg.data:
            return
        with self._lock:
            self.events.append(DetectEvent('placement', self._current_zone))
        self.get_logger().info('[相機] start_placement_detection')

        def reply():
            self.pub_placement.publish(
                Float32MultiArray(data=[float(x) for x in PLACEMENT]))
            self.pub_placement_done.publish(Bool(data=True))

        self._fire_once(0.3, reply)


# ── 底盤 Stub ────────────────────────────────────────────────

class ChassisStub(Node):
    def __init__(self):
        super().__init__('chassis_stub')
        self.pub_arrived = self.create_publisher(Bool, 'area_arrived', 10)
        self.create_subscription(Int32, 'move_command', self._on_move, 10)

    def _fire_once(self, delay, fn):
        holder = [None]

        def cb():
            if holder[0]:
                holder[0].cancel()
                holder[0] = None
            fn()

        holder[0] = self.create_timer(delay, cb)

    def _on_move(self, msg):
        self._fire_once(0.3, lambda: self.pub_arrived.publish(Bool(data=True)))


# ── 手臂 Stub ────────────────────────────────────────────────

class ArmStub(Node):
    def __init__(self):
        super().__init__('arm_stub')
        self.pub_arm_ready = self.create_publisher(Bool, 'arm_ready', 10)
        self.pub_grab_done = self.create_publisher(Bool, 'grab_done', 10)
        self.pub_place_done = self.create_publisher(Bool, 'place_done', 10)

        self.create_subscription(Int32, 'arm_to_detect', self._on_arm_detect, 10)
        self.create_subscription(Float32MultiArray, 'grab_target', self._on_grab, 10)
        self.create_subscription(Float32MultiArray, 'place_target', self._on_place, 10)

    def _fire_once(self, delay, fn):
        holder = [None]

        def cb():
            if holder[0]:
                holder[0].cancel()
                holder[0] = None
            fn()

        holder[0] = self.create_timer(delay, cb)

    def _on_arm_detect(self, msg):
        self._fire_once(0.2, lambda: self.pub_arm_ready.publish(Bool(data=True)))

    def _on_grab(self, msg):
        self._fire_once(0.3, lambda: self.pub_grab_done.publish(Bool(data=True)))

    def _on_place(self, msg):
        self._fire_once(0.3, lambda: self.pub_place_done.publish(Bool(data=True)))


# ── 輔助 ─────────────────────────────────────────────────────

def _run(zone_data, wait_sec=30.0):
    rclpy.init()
    try:
        master = MasterNode()
        camera = CameraCapture(zone_data)
        chassis = ChassisStub()
        arm = ArmStub()

        ex = MultiThreadedExecutor()
        for n in (master, camera, chassis, arm):
            ex.add_node(n)

        t = threading.Thread(target=ex.spin, daemon=True)
        t.start()

        deadline = time.time() + wait_sec
        while time.time() < deadline:
            if master.state == 'DONE':
                break
            time.sleep(0.05)

        ex.shutdown(timeout_sec=2)
        t.join(timeout=3)
        return master, camera
    finally:
        rclpy.shutdown()


# ── 測試案例 ─────────────────────────────────────────────────

def test_detection_triggered_once_per_zone_visit():
    """
    每次到達收集區都應觸發一次 start_detection。
    C 區去了 3 次（抓 F、抓 R、確認空），D 去 1 次，E 去 1 次 → 共 5 次 collect 事件。
    """
    zone_data = {2: ZONE_C, 3: ZONE_D, 4: ZONE_E}
    master, camera = _run(zone_data)
    assert master.state == 'DONE'

    collect_events = [e for e in camera.events if e.kind == 'collect']
    # C 去 3 次（抓 F、抓 R、空了跳 D），D 去 2 次（抓 I、空了跳 E），E 去 1 次（抓 A）
    assert len(collect_events) >= 4, \
        f'collect 觸發次數應 >= 4，實際: {len(collect_events)}'


def test_placement_detection_triggered_after_each_grab():
    """每次抓取後前往 B 區，應觸發一次 start_placement_detection，共 4 次"""
    zone_data = {2: ZONE_C, 3: ZONE_D, 4: ZONE_E}
    master, camera = _run(zone_data)
    assert master.state == 'DONE'

    placement_events = [e for e in camera.events if e.kind == 'placement']
    assert len(placement_events) == 4, \
        f'placement 觸發次數應為 4，實際: {len(placement_events)}'


def test_collect_before_placement_always():
    """事件序列應為 collect/placement 交替：collect → placement → collect → ..."""
    zone_data = {2: ZONE_C, 3: ZONE_D, 4: ZONE_E}
    _, camera = _run(zone_data)

    # 第一個 placement 之前必定有至少一個 collect
    kinds = [e.kind for e in camera.events]
    assert kinds[0] == 'collect', f'第一個事件應為 collect，實際: {kinds[0]}'

    # 每個 placement 前面的上一個相機事件應為 collect
    for i, ev in enumerate(camera.events):
        if ev.kind == 'placement':
            assert i > 0 and camera.events[i - 1].kind == 'collect', \
                f'placement 前一個相機事件應為 collect，事件序列: {kinds}'


def test_collect_detects_correct_zone():
    """
    start_detection 觸發時，記錄的 zone 應對應主控 current_zone。
    首次觸發應在 zone=2（C 區）。
    """
    zone_data = {2: ZONE_C, 3: ZONE_D, 4: ZONE_E}
    _, camera = _run(zone_data)

    collect_events = [e for e in camera.events if e.kind == 'collect']
    assert collect_events[0].zone == 2, \
        f'第一次偵測應在 C(2)，實際: {collect_events[0].zone}'


def test_empty_zone_still_triggers_detection():
    """即使區域是空的，主控仍應觸發偵測（結果為空才知道要跳區）"""
    zone_data = {2: ZONE_EMPTY, 3: ZONE_D, 4: ZONE_E}
    _, camera = _run(zone_data)

    c_events = [e for e in camera.events if e.kind == 'collect' and e.zone == 2]
    assert len(c_events) >= 1, 'C 區空時仍應觸發至少一次偵測'


def test_target_selection_uses_lowest_y():
    """
    C 區有 F(Y=0.05) 與 R(Y=0.12)，主控應先抓 F。
    確認方式：第一次 collect 後，master.picked_list[0] == 5（F 的 ID）。
    """
    zone_data = {2: ZONE_C, 3: ZONE_D, 4: ZONE_E}
    master, _ = _run(zone_data)
    assert master.state == 'DONE'

    assert master.picked_list[0] == 5, \
        f'應先抓 F(ID=5)，實際 picked_list: {master.picked_list}'


def test_placement_data_maps_to_correct_id():
    """
    主控根據 placement_topic 找到 self.holding 對應的座標。
    驗證 picked_list 中每個 ID 都能在 PLACEMENT 裡找到對應位置。
    """
    zone_data = {2: ZONE_C, 3: ZONE_D, 4: ZONE_E}
    master, _ = _run(zone_data)
    assert master.state == 'DONE'

    # 建立 PLACEMENT id→coords 對照
    placement_map = {}
    count = int(PLACEMENT[0])
    for i in range(count):
        idx = 1 + i * 4
        placement_map[int(PLACEMENT[idx])] = PLACEMENT[idx + 1:idx + 4]

    for obj_id in master.picked_list:
        assert obj_id in placement_map, \
            f'ID={obj_id} 在 PLACEMENT 中找不到對應位置'
