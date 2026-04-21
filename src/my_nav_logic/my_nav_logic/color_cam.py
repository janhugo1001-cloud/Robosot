import rclpy
from rclpy.node import Node
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from std_msgs.msg import String, Empty

STABLE_FRAMES = 5  # 連續幾幀相同才發送

class ColorDetector(Node):
    def __init__(self):
        super().__init__('color_detector')

        self.publisher_ = self.create_publisher(String, 'color_topic', 10)

        self.subscription = self.create_subscription(
            Image,
            '/camera/camera/color/image_raw',
            self.image_callback,
            10
        )

        self.bridge = CvBridge()

        # 綠色
        self.green_lower = np.array([35, 60, 60], np.uint8)
        self.green_upper = np.array([95, 255, 255], np.uint8)

        # 藍色
        self.blue_lower = np.array([85, 60, 40], np.uint8)
        self.blue_upper = np.array([135, 255, 255], np.uint8)

        # 黃色
        self.yellow_lower = np.array([15, 80, 80], np.uint8)
        self.yellow_upper = np.array([35, 255, 255], np.uint8)

        # 穩定判斷狀態
        self.result_history = []
        self.result_published = False

        self.get_logger().info('Color Detector Node 已啟動')

        self.reset_sub = self.create_subscription(
            Empty, '/reset_color_detection', self.reset_callback, 10)
            
    def reset_callback(self, msg):
        self.result_history = []
        self.result_published = False
        self.get_logger().info('顏色偵測已重置，重新開始偵測')

    def image_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

        # 已發送過就只顯示畫面
        if self.result_published:
            cv2.imshow("Color Detection Result", frame)
            cv2.waitKey(1)
            return

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        color_map = {
            "Yellow": (self.yellow_lower, self.yellow_upper),
            "Blue":   (self.blue_lower,   self.blue_upper),
            "Green":  (self.green_lower,  self.green_upper)
        }

        detected_objects = []

        for color_name, (lower, upper) in color_map.items():
            mask = cv2.inRange(hsv, lower, upper)

            kernel = np.ones((5, 5), np.uint8)
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
            mask = cv2.dilate(mask, kernel, iterations=1)

            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            for cnt in contours:
                area = cv2.contourArea(cnt)
                if area > 3500:
                    x, y, w, h = cv2.boundingRect(cnt)
                    aspect_ratio = float(w) / h
                    if 0.4 < aspect_ratio < 2.5:
                        cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
                        cv2.putText(frame, color_name, (x, y - 10),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.9, (36, 255, 12), 2)
                        detected_objects.append((color_name, x))

        if detected_objects:
            detected_objects.sort(key=lambda obj: obj[1])
            current_result = ', '.join([obj[0] for obj in detected_objects])
        else:
            current_result = ''

        # 更新歷史
        self.result_history.append(current_result)
        if len(self.result_history) > STABLE_FRAMES:
            self.result_history.pop(0)

        # 連續 STABLE_FRAMES 幀相同且有結果才發送一次
        if (len(self.result_history) == STABLE_FRAMES
                and all(r == current_result for r in self.result_history)
                and current_result != ''):
            res_msg = String()
            res_msg.data = current_result
            self.publisher_.publish(res_msg)
            self.get_logger().info('發布穩定辨識結果: "%s"' % res_msg.data)
            self.result_published = True

        cv2.imshow("Color Detection Result", frame)
        cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    node = ColorDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
