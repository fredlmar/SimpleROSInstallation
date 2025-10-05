import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class CustomNode(Node):
    def __init__(self):
        super().__init__('custom_node')
        self.declare_parameter('timeout_seconds', 3.0)
        self.declare_parameter('check_period_seconds', 1.0)
        self.timeout_duration = float(self.get_parameter('timeout_seconds').get_parameter_value().double_value)
        check_period = float(self.get_parameter('check_period_seconds').get_parameter_value().double_value)
        self.last_message_time = self.get_clock().now()
        self.talker_active = True
        self.status_pub = self.create_publisher(String, 'talker_status', 10)
        self.chatter_sub = self.create_subscription(
            String,
            'chatter',
            self.chatter_callback,
            10
        )
        self.timer = self.create_timer(check_period, self.check_timeout)
        self.get_logger().info('CustomNode started, monitoring /chatter')

    def chatter_callback(self, msg):
        self.last_message_time = self.get_clock().now()
        if not self.talker_active:
            self.talker_active = True
            self.get_logger().info('Talker resumed - message received')
            status_msg = String()
            status_msg.data = 'talker_active'
            self.status_pub.publish(status_msg)

    def check_timeout(self):
        elapsed = (self.get_clock().now() - self.last_message_time).nanoseconds / 1e9
        if elapsed > self.timeout_duration and self.talker_active:
            self.talker_active = False
            self.get_logger().warn(f'Talker stopped - no messages for {elapsed:.2f} seconds')
            status_msg = String()
            status_msg.data = 'talker_stopped'
            self.status_pub.publish(status_msg)

def main(args=None):
    rclpy.init(args=args)
    node = CustomNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
