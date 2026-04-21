import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class ColorPublisher(Node):
    def __init__(self):
        super().__init__('color_publisher')

        self.publisher = self.create_publisher(String, '/detected_color', 10)

        # 等 1 秒確保訂閱者準備好
        self.timer = self.create_timer(1.0, self.publish_once)
        self.published = False

        self.get_logger().info('顏色發布器啟動！')
        self.get_logger().info('請稍候，1 秒後發布...')

    def publish_once(self):
        if self.published:
            return
        self.published = True

        msg = String()
        msg.data = 'Yellow,Blue,Green'  # ← 這裡修改測試的顏色順序
        self.publisher.publish(msg)
        self.get_logger().info(f'已發布顏色順序：{msg.data}')

        self.timer.cancel()


def main(args=None):
    rclpy.init(args=args)
    node = ColorPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()


if __name__ == '__main__':
    main()