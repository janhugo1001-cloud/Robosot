import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Int32
from geometry_msgs.msg import PointStamped

RESPOND_DELAY = 1.0  # 模擬手臂動作時間（秒）


class FakeArm(Node):

    def __init__(self):
        super().__init__('fake_arm')

        self.pub_arm_ready = self.create_publisher(Bool, 'arm_ready', 10)
        self.pub_grab_done = self.create_publisher(Bool, 'grab_done', 10)
        self.pub_place_done = self.create_publisher(Bool, 'place_done', 10)

        self.create_subscription(Int32, 'arm_to_detect', self._on_arm_to_detect, 10)
        self.create_subscription(PointStamped, 'grab_target', self._on_grab_target, 10)
        self.create_subscription(PointStamped, 'place_target', self._on_place_target, 10)

        self.get_logger().info('假手臂啟動，等待指令...')

    def _fire_once(self, delay, fn):
        holder = [None]

        def cb():
            holder[0].cancel()
            holder[0] = None
            fn()

        holder[0] = self.create_timer(delay, cb)

    def _on_arm_to_detect(self, msg):
        label = '收集區' if msg.data == 0 else '放置區'
        self.get_logger().info(f'[手臂] 移至{label}偵測位，{RESPOND_DELAY}s 後回報就緒')
        self._fire_once(RESPOND_DELAY, lambda: self.pub_arm_ready.publish(Bool(data=True)))

    def _on_grab_target(self, msg):
        p = msg.point
        self.get_logger().info(
            f'[手臂] 抓取 [{p.x:.3f}, {p.y:.3f}, {p.z:.3f}] '
            f'({msg.header.frame_id})，{RESPOND_DELAY}s 後回報完成')
        self._fire_once(RESPOND_DELAY, lambda: self.pub_grab_done.publish(Bool(data=True)))

    def _on_place_target(self, msg):
        p = msg.point
        self.get_logger().info(
            f'[手臂] 放置 [{p.x:.3f}, {p.y:.3f}, {p.z:.3f}] '
            f'({msg.header.frame_id})，{RESPOND_DELAY}s 後回報完成')
        self._fire_once(RESPOND_DELAY, lambda: self.pub_place_done.publish(Bool(data=True)))


def main(args=None):
    rclpy.init(args=args)
    node = FakeArm()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
