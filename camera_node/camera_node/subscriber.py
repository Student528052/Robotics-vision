import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

class FloatArraySubscriber(Node):
    def __init__(self):
        super().__init__('float_array_subscriber')
        self.subscription = self.create_subscription(
            Float32MultiArray,
            'float_array_topic',
            self.listener_callback,
            10
        )

    def listener_callback(self, msg):
        self.get_logger().info(f'Received: {msg}')

def main(args=None):
    rclpy.init(args=args)
    node = FloatArraySubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()