"""
手臂 topic 測試

驗證主控對手臂的指令內容是否正確。

訂閱（主控發給手臂）: arm_to_detect (Int32), grab_target (Float32MultiArray),
                       place_target (Float32MultiArray)
發布（手臂回給主控）: arm_ready (Bool), grab_done (Bool), place_done (Bool)

Stub 節點自動處理底盤與相機 topic。
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

ZONE_C = [2.0, 5.0, 0.10, 0.05, 0.30, 17.0, -0.08, 0.12, 0.35]   # F Y=0.05, R Y=0.12
ZONE_D = [1.0, 19.0, 0.05, 0.08, 0.30]                              # I
ZONE_E = [1.0, 0.0,  0.02, 0.03, 0.28]                              # A

PLACEMENT = [
    4.0,
    5.0,  0.05, 0.03, 0.25,   # F 放置位置
    19.0, 0.10, 0.03, 0.25,   # I 放置位置
    17.0, 0.15, 0.03, 0.25,   # R 放置位置
    0.0,  0.20, 0.03, 0.25,   # A 放置位置
]


# ── 手臂 Capture Node ────────────────────────────────────────

@dataclass
class ArmCommand:
    kind: str        # 'detect_pos' | 'grab' | 'place'
    value: object    # Int32.data 或 [X, Y, Z]


class ArmCapture(Node):
    """
    記錄所有手臂指令（依發送順序），並自動回應。
    """

    def __init__(self):
        super().__init__('arm_capture')
        self.commands: list[ArmCommand] = []
        self._lock = threading.Lock()

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

    def _on_arm_detect(self, msg: Int32):
        with self._lock:
            self.commands.append(ArmCommand('detect_pos', msg.data))
        label = '收集區(0)' if msg.data == 0 else '放置區(1)'
        self.get_logger().info(f'[手臂] arm_to_detect = {msg.data} ({label})')
        self._fire_once(0.2, lambda: self.pub_arm_ready.publish(Bool(data=True)))

    def _on_grab(self, msg: Float32MultiArray):
        coords = list(msg.data)
        with self._lock:
            self.commands.append(ArmCommand('grab', coords))
        self.get_logger().info(f'[手臂] grab_target = {coords}')
        self._fire_once(0.3, lambda: self.pub_grab_done.publish(Bool(data=True)))

    def _on_place(self, msg: Float32MultiArray):
        coords = list(msg.data)
        with self._lock:
            self.commands.append(ArmCommand('place', coords))
        self.get_logger().info(f'[手臂] place_target = {coords}')
        self._fire_once(0.3, lambda: self.pub_place_done.publish(Bool(data=True)))


# ── 底盤 Stub ────────────────────────────────────────────────

class ChassisStub(Node):
    """自動回應 move_command，追蹤當前區以提供 zone_data。"""

    def __init__(self):
        super().__init__('chassis_stub')
        self.current_zone = 0
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
        self.current_zone = msg.data
        self._fire_once(0.3, lambda: self.pub_arrived.publish(Bool(data=True)))


# ── 相機 Stub ────────────────────────────────────────────────

class CameraStub(Node):
    """自動回應偵測，依 zone_data 回傳資料。"""

    def __init__(self, zone_data: dict):
        super().__init__('camera_stub')
        self._zone_data = zone_data
        self._current_zone = 0

        self.pub_task = self.create_publisher(Float32MultiArray, 'task_topic', 10)
        self.pub_detect_done = self.create_publisher(Bool, 'detection_done', 10)
        self.pub_placement = self.create_publisher(Float32MultiArray, 'placement_topic', 10)
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

    def _on_move(self, msg):
        self._current_zone = msg.data

    def _on_detect(self, msg):
        if not msg.data:
            return
        data = self._zone_data.get(self._current_zone, [0.0])

        def reply():
            self.pub_task.publish(Float32MultiArray(data=[float(x) for x in data]))
            self.pub_detect_done.publish(Bool(data=True))

        self._fire_once(0.3, reply)

    def _on_placement(self, msg):
        if not msg.data:
            return

        def reply():
            self.pub_placement.publish(
                Float32MultiArray(data=[float(x) for x in PLACEMENT]))
            self.pub_placement_done.publish(Bool(data=True))

        self._fire_once(0.3, reply)


# ── 輔助 ─────────────────────────────────────────────────────

def _run(zone_data, wait_sec=30.0):
    rclpy.init()
    try:
        master = MasterNode()
        arm = ArmCapture()
        chassis = ChassisStub()
        camera = CameraStub(zone_data)

        ex = MultiThreadedExecutor()
        for n in (master, arm, chassis, camera):
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
        return master, arm
    finally:
        rclpy.shutdown()


# ── 測試案例 ─────────────────────────────────────────────────

def test_arm_detect_pos_alternates_0_and_1():
    """
    arm_to_detect 規則：
    - 每次放置（1）一定有 4 次，對應 4 次抓取
    - 收集（0）次數 >= 4（空區造訪也會產生一次 0）
    - 每個 1 的前一次必定是 0（不可能連續出現兩個 1）
    """
    zone_data = {2: ZONE_C, 3: ZONE_D, 4: ZONE_E}
    _, arm = _run(zone_data)

    detect_cmds = [c.value for c in arm.commands if c.kind == 'detect_pos']

    # 放置側（1）恰好 4 次
    assert detect_cmds.count(1) == 4, \
        f'arm_to_detect=1 應有 4 次，實際: {detect_cmds}'

    # 收集側（0）至少 4 次
    assert detect_cmds.count(0) >= 4, \
        f'arm_to_detect=0 應至少 4 次，實際: {detect_cmds}'

    # 每個 1 的前一個必為 0（放置前一定先做收集偵測）
    for i, val in enumerate(detect_cmds):
        if val == 1:
            assert i > 0 and detect_cmds[i - 1] == 0, \
                f'第 {i} 個 arm_to_detect=1 的前一個應為 0，序列: {detect_cmds}'


def test_grab_target_lowest_y_first():
    """
    C 區有 F(Y=0.05) 和 R(Y=0.12)，第一次 grab_target 應為 F 的座標。
    """
    zone_data = {2: ZONE_C, 3: ZONE_D, 4: ZONE_E}
    _, arm = _run(zone_data)

    grabs = [c.value for c in arm.commands if c.kind == 'grab']
    assert len(grabs) >= 1

    first_grab = grabs[0]
    # F 的座標：X=0.10, Y=0.05, Z=0.30
    assert abs(first_grab[1] - 0.05) < 1e-3, \
        f'第一次抓取 Y 應為 0.05（F），實際: {first_grab}'


def test_grab_count_equals_4():
    """總共應發出 4 次 grab_target"""
    zone_data = {2: ZONE_C, 3: ZONE_D, 4: ZONE_E}
    master, arm = _run(zone_data)
    assert master.state == 'DONE'

    grabs = [c for c in arm.commands if c.kind == 'grab']
    assert len(grabs) == 4, f'應有 4 次抓取，實際: {len(grabs)}'


def test_place_count_equals_4():
    """每次抓取後應對應一次 place_target，共 4 次"""
    zone_data = {2: ZONE_C, 3: ZONE_D, 4: ZONE_E}
    master, arm = _run(zone_data)
    assert master.state == 'DONE'

    places = [c for c in arm.commands if c.kind == 'place']
    assert len(places) == 4, f'應有 4 次放置，實際: {len(places)}'


def test_grab_before_place_always():
    """每一次 place 前必定有一次 grab（抓放交替）"""
    zone_data = {2: ZONE_C, 3: ZONE_D, 4: ZONE_E}
    _, arm = _run(zone_data)

    action_seq = [c.kind for c in arm.commands if c.kind in ('grab', 'place')]
    # 應為 grab, place, grab, place, ...
    for i, act in enumerate(action_seq):
        expected = 'grab' if i % 2 == 0 else 'place'
        assert act == expected, \
            f'動作序列第 {i} 步應為 {expected}，實際: {act}，序列: {action_seq}'


def test_place_target_matches_placement_data():
    """
    放置座標應對應 PLACEMENT 中該物件 ID 的座標。
    例如 F(ID=5) → place_target ≈ [0.05, 0.03, 0.25]
    """
    zone_data = {2: ZONE_C, 3: ZONE_D, 4: ZONE_E}
    master, arm = _run(zone_data)
    assert master.state == 'DONE'

    # 建立 PLACEMENT 的 id→coords 對照表
    placement_map = {}
    count = int(PLACEMENT[0])
    for i in range(count):
        idx = 1 + i * 4
        obj_id = int(PLACEMENT[idx])
        placement_map[obj_id] = (PLACEMENT[idx + 1], PLACEMENT[idx + 2], PLACEMENT[idx + 3])

    # 依照 picked_list 順序驗證每次 place_target
    places = [c.value for c in arm.commands if c.kind == 'place']
    picked = master.picked_list
    assert len(places) == len(picked)

    for obj_id, coords in zip(picked, places):
        expected = placement_map[obj_id]
        for j in range(3):
            assert abs(coords[j] - expected[j]) < 1e-3, \
                f'ID={obj_id} 放置座標第 {j} 維應為 {expected[j]:.3f}，實際: {coords[j]:.3f}'
