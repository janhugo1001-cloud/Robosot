import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Int32


ZONE_NAME = {
    0: '出發區 A',
    1: '放置區 B',
    2: '收集區 2',
    3: '收集區 3',
    4: '收集區 4',
}

TRAVEL_TIME = 2.0  # 模擬移動時間（秒）


class FakeChassis(Node):

    def __init__(self):
        super().__init__('fake_chassis')

        self.pub_arrived = self.create_publisher(Bool, 'area_arrived', 10)
        self.create_subscription(Int32, 'move_command', self.move_command_callback, 10)

        self._pending_timer = None
        self.get_logger().info('假底盤啟動，等待 move_command...')

    def move_command_callback(self, msg):
        zone = msg.data
        name = ZONE_NAME.get(zone, f'未知區 {zone}')
        self.get_logger().info(f'收到 move_command = {zone}，前往 {name}（{TRAVEL_TIME} 秒後到達）')

        if self._pending_timer:
            self._pending_timer.cancel()

        self._pending_timer = self.create_timer(TRAVEL_TIME, lambda: self._arrive(name))

    def _arrive(self, name):
        self._pending_timer.cancel()
        self._pending_timer = None
        self.get_logger().info(f'到達 {name}，發佈 area_arrived = True')
        self.pub_arrived.publish(Bool(data=True))


def main(args=None):
    rclpy.init(args=args)
    node = FakeChassis()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
