import rclpy
from rclpy.node import Node
from std_msgs.msg import String

NANOSECONDS_TO_SECONDS = 1e9
STATUS_ACTIVE = 'talker_active'
STATUS_STOPPED = 'talker_stopped'

class CustomNode(Node):
    def __init__(self):
        super().__init__('custom_node')
        self.declare_parameter('timeout_seconds', 3.0)
        self.declare_parameter('check_period_seconds', 1.0)
        self.timeout_duration = self.get_parameter_value('timeout_seconds')
        check_period = self.get_parameter_value('check_period_seconds')
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
        self.get_logger().info('CustomNode started, monitoring "chatter" topic.')

    def get_parameter_value(self, name: str) -> float:
        return self.get_parameter(name).get_parameter_value().double_value

    def chatter_callback(self, msg):
        if not self.talker_active:
            self.get_logger().info('Talker resumed - message received')
            status_msg = String()
            status_msg.data = STATUS_ACTIVE
            self.status_pub.publish(status_msg)
            self.talker_active = True
        self.last_message_time = self.get_clock().now()

    def check_timeout(self):
        # Check if talker has timed out
        elapsed = (self.get_clock().now() - self.last_message_time).nanoseconds / NANOSECONDS_TO_SECONDS
        if elapsed > self.timeout_duration and self.talker_active:
            self.get_logger().warn(f'Talker stopped - no messages for {elapsed:.2f} seconds')
            status_msg = String()
            status_msg.data = STATUS_STOPPED
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
