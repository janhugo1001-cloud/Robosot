"""
底盤 topic 測試

驗證主控對底盤的 move_command 發送順序與時機是否正確。

訂閱（主控發給底盤）: move_command (Int32)
發布（底盤回給主控）: area_arrived (Bool)

Stub 節點自動處理手臂與相機 topic，使狀態機可以正常推進。
"""

import threading
import time

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import Bool, Int32
from std_msgs.msg import Float32MultiArray

from control_center.master_node import MasterNode

# ── 測試資料 ─────────────────────────────────────────────────

# C 區：F(5) Y=0.05, R(17) Y=0.12
ZONE_C = [2.0, 5.0, 0.10, 0.05, 0.30, 17.0, -0.08, 0.12, 0.35]
# D 區：I(19)
ZONE_D = [1.0, 19.0, 0.05, 0.08, 0.30]
# E 區：A(0)
ZONE_E = [1.0, 0.0, 0.02, 0.03, 0.28]

PLACEMENT = [
    4.0,
    5.0,  0.05, 0.03, 0.25,
    19.0, 0.10, 0.03, 0.25,
    17.0, 0.15, 0.03, 0.25,
    0.0,  0.20, 0.03, 0.25,
]


# ── 底盤 Capture Node ────────────────────────────────────────

class ChassisCapture(Node):
    """
    記錄所有 move_command，收到後延遲回應 area_arrived。
    """

    def __init__(self, zone_data: dict):
        super().__init__('chassis_capture')
        self._zone_data = zone_data
        self.commands_received: list[int] = []   # 依序記錄所有 move_command 值
        self._lock = threading.Lock()

        self.pub_arrived = self.create_publisher(Bool, 'area_arrived', 10)
        self.create_subscription(Int32, 'move_command', self._on_move, 10)

    def _on_move(self, msg: Int32):
        with self._lock:
            self.commands_received.append(msg.data)
        self.get_logger().info(f'[底盤] move_command = {msg.data}')
        self._fire_once(0.3, lambda: self.pub_arrived.publish(Bool(data=True)))

    def _fire_once(self, delay, fn):
        holder = [None]

        def cb():
            if holder[0]:
                holder[0].cancel()
                holder[0] = None
            fn()

        holder[0] = self.create_timer(delay, cb)


# ── 手臂 Stub ────────────────────────────────────────────────

class ArmStub(Node):
    """自動回應手臂相關 topic，不做任何記錄。"""

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


# ── 相機 Stub ────────────────────────────────────────────────

class CameraStub(Node):
    """自動回應偵測 topic，依 zone_data 回傳對應資料。"""

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
        chassis = ChassisCapture(zone_data)
        arm = ArmStub()
        camera = CameraStub(zone_data)

        ex = MultiThreadedExecutor()
        for n in (master, chassis, arm, camera):
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
        return master, chassis
    finally:
        rclpy.shutdown()


# ── 測試案例 ─────────────────────────────────────────────────

def test_first_command_is_zone_c():
    """主控啟動後第一個 move_command 必須是 2（C 區）"""
    zone_data = {2: ZONE_C, 3: ZONE_D, 4: ZONE_E}
    _, chassis = _run(zone_data)
    assert chassis.commands_received[0] == 2, \
        f'首個指令應為 2(C)，實際: {chassis.commands_received}'


def test_move_to_place_after_grab():
    """每次抓取後必須發出 move_command=1（B 放置區）"""
    zone_data = {2: ZONE_C, 3: ZONE_D, 4: ZONE_E}
    _, chassis = _run(zone_data)
    cmds = chassis.commands_received
    # 抓 4 個 → 應有 4 次 move_command=1
    place_count = cmds.count(1)
    assert place_count == 4, f'應有 4 次前往 B(1)，實際: {place_count}，完整序列: {cmds}'


def test_return_to_same_zone_after_partial_pickup():
    """
    C 區有 2 個物件，抓第一個放完後，CHECK_DONE 應再回 C(2)，
    不應直接跳到 D(3)。
    """
    zone_data = {2: ZONE_C, 3: ZONE_D, 4: ZONE_E}
    _, chassis = _run(zone_data)
    cmds = chassis.commands_received

    # 找第一個 move_command=1（前往 B 放置）的後一個指令
    for i, c in enumerate(cmds):
        if c == 1 and i + 1 < len(cmds):
            next_cmd = cmds[i + 1]
            assert next_cmd == 2, \
                f'放完第一個後應回 C(2)，實際下一個指令: {next_cmd}，序列: {cmds}'
            return
    assert False, f'未找到前往 B 的指令，序列: {cmds}'


def test_advance_zone_when_empty():
    """C 區空，應跳到 D(3) 而非停留在 C(2)"""
    zone_data = {2: [0.0], 3: ZONE_D, 4: ZONE_E}
    _, chassis = _run(zone_data)
    cmds = chassis.commands_received

    # 確認有發出 move_command=3（D 區）
    assert 3 in cmds, f'C 區空時應前往 D(3)，實際序列: {cmds}'
    # D 必須在 C 之後出現
    idx_c = cmds.index(2)
    idx_d = cmds.index(3)
    assert idx_d > idx_c, f'D(3) 應在 C(2) 之後，序列: {cmds}'


def test_final_return_home():
    """抓滿 4 個後最後一個 move_command 必須是 0（A 出發區）"""
    zone_data = {2: ZONE_C, 3: ZONE_D, 4: ZONE_E}
    master, chassis = _run(zone_data)
    assert master.state == 'DONE'
    assert chassis.commands_received[-1] == 0, \
        f'最後指令應為 0(A)，實際: {chassis.commands_received[-1]}，序列: {chassis.commands_received}'


def test_full_move_sequence():
    """
    驗證完整 move_command 序列符合預期模式：
    C→(B→C)*→(B→D)→B→(D→)B→E→B→A
    即: 2,1,2,1,2,1,3,1,3,1,4,1,0  （C抓2次D抓1次E抓1次）
    """
    zone_data = {2: ZONE_C, 3: ZONE_D, 4: ZONE_E}
    master, chassis = _run(zone_data)
    assert master.state == 'DONE'

    cmds = chassis.commands_received
    # 必要特徵：開頭 2，結尾 0，中間有 4 個 1
    assert cmds[0] == 2,  f'開頭應為 2: {cmds}'
    assert cmds[-1] == 0, f'結尾應為 0: {cmds}'
    assert cmds.count(1) == 4, f'應有 4 次前往 B(1): {cmds}'
