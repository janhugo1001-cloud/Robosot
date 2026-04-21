import rclpy
from rclpy.node import Node
import cv2
import numpy as np
import os
import easyocr

from ament_index_python.packages import get_package_share_directory
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
from std_msgs.msg import Bool, Float32MultiArray
from message_filters import ApproximateTimeSynchronizer, Subscriber
from ultralytics import YOLO
from rclpy.qos import qos_profile_sensor_data
from tf2_ros import Buffer, TransformBroadcaster, TransformListener
from geometry_msgs.msg import TransformStamped

# YOLO 目標 ID（只用來確認是否偵測到木塊，分類由 OCR 決定）
TARGET_IDS = {0: 'A', 5: 'F', 19: 'T', 17: 'R'}

# OCR 字母對應表
LETTER_TO_ID = {'F': 5, 'I': 19, 'R': 17, 'A': 0}
ID_TO_LETTER = {5: 'F', 19: 'I', 17: 'R', 0: 'A'}
VALID_LETTERS = {'F', 'I', 'R', 'A'}

# 夾取順序 F→I→R→A
PICKUP_ORDER = [5, 19, 17, 0]

# 穩定判斷參數
STABLE_FRAMES    = 8
STABLE_THRESHOLD = 0.015  # 座標變化範圍 (m)

MODE_IDLE      = 'idle'
MODE_COLLECT   = 'collect'
MODE_PLACEMENT = 'placement'

# 這裡用針孔模型由影像像素回推 3D，因此座標是 optical frame 慣例：
# X 向右為正、Y 向下為正、Z 向前為正。
# 若實機 TF tree 使用 camera_color_optical_frame，可直接改這個常數。
CAMERA_SOURCE_FRAME = 'camera_frame'
TASK_TARGET_FRAME = 'base_link'


class OcrPuberNode(Node):
    def __init__(self):
        super().__init__('ocr_puber_node')

        # 相機內參（預設值，會被 camera_info 覆蓋）
        self.fx = 1314.8191
        self.fy = 1312.7450
        self.cx = 978.7547
        self.cy = 556.5297
        self.camera_info_received = False

        # YOLO 模型載入
        package_share_directory = get_package_share_directory('ocr_puber_node')
        model_path = os.path.join(package_share_directory, 'weights', 'best.pt')
        self.model = YOLO(model_path)

        # EasyOCR 初始化
        self.get_logger().info('初始化 EasyOCR（第一次執行需下載模型）...')
        self.ocr_reader = easyocr.Reader(['en'], gpu=False)
        self.get_logger().info('EasyOCR 初始化完成。')

        self.bridge = CvBridge()
        self.tf_broadcaster = TransformBroadcaster(self)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.camera_source_frame = CAMERA_SOURCE_FRAME
        self.task_target_frame = TASK_TARGET_FRAME

        # 狀態控制
        self.mode           = MODE_IDLE
        self.task_published = False
        self.coord_history  = {obj_id: [] for obj_id in LETTER_TO_ID.values()}
        self.locked_targets = {}

        # 固定 0.5s 持續發布 TF timer
        self.create_timer(0.5, self._timer_publish_callback)

        # Subscribers
        self.rgb_sub = Subscriber(
            self, Image, '/camera/camera/color/image_raw',
            qos_profile=qos_profile_sensor_data
        )
        self.depth_sub = Subscriber(
            self, Image, '/camera/camera/aligned_depth_to_color/image_raw',
            qos_profile=qos_profile_sensor_data
        )
        self.sync = ApproximateTimeSynchronizer(
            [self.rgb_sub, self.depth_sub],
            queue_size=10,
            slop=0.2,
        )
        self.sync.registerCallback(self.sync_callback)

        # 訂閱相機內參（動態取得真實內參，比寫死的準確）
        self.create_subscription(
            CameraInfo, '/camera/camera/color/camera_info',
            self.camera_info_callback, 10
        )

        # 接收主控的開始信號
        self.create_subscription(Bool, 'start_detection',           self.start_collect_callback,   10)
        self.create_subscription(Bool, 'start_placement_detection', self.start_placement_callback, 10)

        # Publishers
        self.publisher_done    = self.create_publisher(Bool, 'detection_done',  10)
        self.publisher_pl_done = self.create_publisher(Bool, 'placement_done',  10)
        self.publisher_task    = self.create_publisher(Float32MultiArray, 'task_topic', 10)

        self.get_logger().info('OcrPuberNode 已啟動，等待 camera_info 與 start_detection 信號...')
        self.get_logger().info(
            f'相機座標使用 optical frame 慣例：{self.camera_source_frame} '
            f'(X右/Y下/Z前)，task_topic 轉換目標 frame：{self.task_target_frame}'
        )

    # --------------------------------------------------
    # 相機內參 callback（只更新一次）
    # --------------------------------------------------
    def camera_info_callback(self, msg: CameraInfo):
        if self.camera_info_received:
            return
        self.fx = msg.k[0]
        self.fy = msg.k[4]
        self.cx = msg.k[2]
        self.cy = msg.k[5]
        self.camera_info_received = True
        self.get_logger().info(
            f'[CameraInfo] 取得相機內參: '
            f'fx={self.fx:.2f}, fy={self.fy:.2f}, '
            f'cx={self.cx:.2f}, cy={self.cy:.2f}'
        )

    # --------------------------------------------------
    # 重置偵測狀態
    # --------------------------------------------------
    def _reset_state(self):
        self.task_published = False
        self.locked_targets = {}
        self.coord_history  = {obj_id: [] for obj_id in LETTER_TO_ID.values()}

    # --------------------------------------------------
    # 接收開始信號
    # --------------------------------------------------
    def start_collect_callback(self, msg: Bool):
        if msg.data:
            self._reset_state()
            self.mode = MODE_COLLECT
            self.get_logger().info('收到開始信號，開始收集區偵測...')
        elif not msg.data:
            self.mode = MODE_IDLE
            self.get_logger().info('收到停止信號，暫停偵測。')

    def start_placement_callback(self, msg: Bool):
        if msg.data:
            self._reset_state()
            self.mode = MODE_PLACEMENT
            self.get_logger().info('收到開始信號，開始卸貨區偵測...')
        elif not msg.data:
            self.mode = MODE_IDLE
            self.get_logger().info('收到停止信號，暫停偵測。')

    # --------------------------------------------------
    # 穩定判斷
    # --------------------------------------------------
    def is_stable(self, coords):
        if len(coords) < STABLE_FRAMES:
            return False
        xs = [c[0] for c in coords]
        ys = [c[1] for c in coords]
        zs = [c[2] for c in coords]
        return (np.std(xs) < STABLE_THRESHOLD and
                np.std(ys) < STABLE_THRESHOLD and
                np.std(zs) < STABLE_THRESHOLD)

    # --------------------------------------------------
    # 主要 callback
    # --------------------------------------------------
    def sync_callback(self, rgb_msg: Image, depth_msg: Image):

        frame       = self.bridge.imgmsg_to_cv2(rgb_msg,   'bgr8')
        depth_image = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding='passthrough')
        annotated   = frame.copy()

        # 尚未收到開始信號或已完成，只顯示畫面不處理
        if self.mode == MODE_IDLE or self.task_published:
            status_text = '等待開始...' if self.mode == MODE_IDLE else '偵測完成'
            cv2.putText(annotated, status_text, (30, 50),
                        cv2.FONT_HERSHEY_SIMPLEX, 1.2, (0, 165, 255), 3)
            return

        # 如果還沒收到 camera_info，log 警告（每 5 秒一次）
        if not self.camera_info_received:
            self.get_logger().warn(
                '尚未收到 camera_info，使用預設內參（可能不準確）',
                throttle_duration_sec=5.0
            )

        results   = self.model(frame, conf=0.4, verbose=False, imgsz=640)
        boxes     = results[0].boxes
        annotated = results[0].plot()

        if boxes is not None and len(boxes) > 0:
            xywh    = boxes.xywh.cpu().numpy()
            cls_ids = boxes.cls.cpu().numpy().astype(int)

            for i in range(len(cls_ids)):
                u, v, w, h  = xywh[i]
                u, v        = int(u), int(v)
                yolo_cls_id = int(cls_ids[i])

                # YOLO 只確認是否偵測到木塊類別
                if yolo_cls_id not in TARGET_IDS:
                    continue

                # bounding box 座標
                x1 = max(int(u - w // 2), 0)
                x2 = min(int(u + w // 2), frame.shape[1])
                y1 = max(int(v - h // 2), 0)
                y2 = min(int(v + h // 2), frame.shape[0])

                # 裁出 ROI，做前處理後送 OCR
                roi = frame[y1:y2, x1:x2]
                if roi.size == 0:
                    continue

                gray_roi = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
                _, binary_roi = cv2.threshold(
                    gray_roi, 0, 255,
                    cv2.THRESH_BINARY + cv2.THRESH_OTSU
                )

                ocr_results = self.ocr_reader.readtext(
                    binary_roi, detail=0, allowlist='FIRA'
                )

                if len(ocr_results) == 0:
                    continue

                letter = ocr_results[0].strip().upper()
                if letter not in VALID_LETTERS:
                    continue

                obj_id = LETTER_TO_ID[letter]

                # 深度計算
                cx_roi   = (x1 + x2) // 2
                cy_roi   = (y1 + y2) // 2
                margin_x = max(1, (x2 - x1) // 4)
                margin_y = max(1, (y2 - y1) // 4)

                depth_roi   = depth_image[
                    cy_roi - margin_y : cy_roi + margin_y,
                    cx_roi - margin_x : cx_roi + margin_x,
                ]
                depth_valid = depth_roi[depth_roi > 0]

                if len(depth_valid) == 0:
                    continue

                # 單位：mm → m
                # 這裡得到的是 optical frame 座標：X右 / Y下 / Z前
                Z = float(np.median(depth_valid)) / 1000.0
                X = (u - self.cx) * Z / self.fx
                Y = (v - self.cy) * Z / self.fy

                # 更新座標歷史
                history = self.coord_history[obj_id]
                history.append((X, Y, Z))
                if len(history) > STABLE_FRAMES:
                    history.pop(0)

                # 判斷是否穩定並鎖定
                if obj_id not in self.locked_targets and self.is_stable(history):
                    avg_x = sum(c[0] for c in history) / STABLE_FRAMES
                    avg_y = sum(c[1] for c in history) / STABLE_FRAMES
                    avg_z = sum(c[2] for c in history) / STABLE_FRAMES
                    self.locked_targets[obj_id] = (avg_x, avg_y, avg_z)
                    self.get_logger().info(
                        f'鎖定 {letter}(OCR): '
                        f'X={avg_x:.3f} Y={avg_y:.3f} Z={avg_z:.3f} m'
                    )

                # 標註畫面
                mode_tag = '[collect]' if self.mode == MODE_COLLECT else '[placement]'
                status   = '(鎖定)' if obj_id in self.locked_targets else f'({len(self.coord_history[obj_id])}/{STABLE_FRAMES}幀)'
                label    = f'{mode_tag}{letter}{status} X:{X:.2f} Y:{Y:.2f} Z:{Z:.2f}m'
                cv2.putText(annotated, label, (u, v - 15),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 0), 2)

        # 所有出現過的 ID 都鎖定才發布任務
        detected_ids = [oid for oid, h in self.coord_history.items() if len(h) > 0]
        if len(detected_ids) > 0 and all(oid in self.locked_targets for oid in detected_ids):
            self._publish_result()


    # --------------------------------------------------
    # TF 發布
    # --------------------------------------------------
    def _publish_tf(self, obj_id: int, X: float, Y: float, Z: float):
        letter = ID_TO_LETTER[obj_id]
        t = TransformStamped()
        t.header.stamp    = self.get_clock().now().to_msg()
        t.header.frame_id = self.camera_source_frame
        t.child_frame_id  = f'object_frame{letter}'
        t.transform.translation.x = float(X)
        t.transform.translation.y = float(Y)
        t.transform.translation.z = float(Z)
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 1.0
        self.tf_broadcaster.sendTransform(t)

    def _quaternion_to_matrix(self, x: float, y: float, z: float, w: float):
        return np.array([
            [1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w),       2.0 * (x * z + y * w)],
            [2.0 * (x * y + z * w),       1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - x * w)],
            [2.0 * (x * z - y * w),       2.0 * (y * z + x * w),       1.0 - 2.0 * (x * x + y * y)],
        ], dtype=float)

    def _transform_point_to_task_frame(self, X: float, Y: float, Z: float):
        try:
            transform = self.tf_buffer.lookup_transform(
                self.task_target_frame,
                self.camera_source_frame,
                rclpy.time.Time()
            )
        except Exception as exc:
            self.get_logger().warn(
                f'無法取得 {self.camera_source_frame} -> {self.task_target_frame} 的 TF: {exc}',
                throttle_duration_sec=5.0
            )
            return None

        q = transform.transform.rotation
        t = transform.transform.translation
        rotation = self._quaternion_to_matrix(q.x, q.y, q.z, q.w)
        point_in_target = rotation @ np.array([X, Y, Z], dtype=float) + np.array([t.x, t.y, t.z], dtype=float)
        return tuple(point_in_target.tolist())

    def _build_task_data(self):
        task_data = []
        count = 0
        for obj_id in PICKUP_ORDER:
            if obj_id in self.locked_targets:
                X, Y, Z = self.locked_targets[obj_id]
                transformed_point = self._transform_point_to_task_frame(X, Y, Z)
                if transformed_point is None:
                    continue
                tx, ty, tz = transformed_point
                task_data.extend([float(obj_id), float(tx), float(ty), float(tz)])
                count += 1
        return [float(count)] + task_data

    # --------------------------------------------------
    # 0.5s timer：持續發布已鎖定目標的 TF 與 task_topic
    # --------------------------------------------------
    def _timer_publish_callback(self):
        if not self.locked_targets:
            return
        for obj_id, (X, Y, Z) in self.locked_targets.items():
            self._publish_tf(obj_id, X, Y, Z)

        if self.task_published:
            task_data = self._build_task_data()
            if task_data[0] > 0.0:
                self.publisher_task.publish(Float32MultiArray(data=task_data))

    # --------------------------------------------------
    # 鎖定完成：儲存結果、通知主控（done 只發一次）
    # --------------------------------------------------
    def _publish_result(self):
        if self.mode == MODE_COLLECT:
            self.publisher_done.publish(Bool(data=True))
            self.get_logger().info('=== 收集區偵測完成，通知主控 ===')
        elif self.mode == MODE_PLACEMENT:
            self.publisher_pl_done.publish(Bool(data=True))
            self.get_logger().info('=== 卸貨區偵測完成，通知主控 ===')

        self.task_published = True
        task_data = self._build_task_data()
        if task_data[0] > 0.0:
            self.publisher_task.publish(Float32MultiArray(data=task_data))
        self.mode           = MODE_IDLE

        for obj_id in PICKUP_ORDER:
            if obj_id in self.locked_targets:
                X, Y, Z = self.locked_targets[obj_id]
                self.get_logger().info(
                    f'  {ID_TO_LETTER[obj_id]}: X={X:.3f} Y={Y:.3f} Z={Z:.3f} m'
                )


def main(args=None):
    rclpy.init(args=args)
    node = OcrPuberNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
