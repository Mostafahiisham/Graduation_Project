import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

class EncoderSubscriber(Node):
    def __init__(self):
        super().__init__('encoder_subscriber')
        self.subscription = self.create_subscription(
            Float32MultiArray,
            'motor_rpms',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info(f'Motor RPMs: {msg.data}')
        # Access individual motor RPMs:
        # motor1 = msg.data[0]
        # motor2 = msg.data[1]
        # etc.

def main(args=None):
    rclpy.init(args=args)
    encoder_subscriber = EncoderSubscriber()
    rclpy.spin(encoder_subscriber)
    encoder_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()