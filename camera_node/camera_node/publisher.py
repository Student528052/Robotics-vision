import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

class FloatArrayPublisher(Node):
    def __init__(self):
        super().__init__('float_array_publisher')
        self.publisher_ = self.create_publisher(Float32MultiArray, 'float_array_topic', 10)
        self.timer = self.create_timer(1.0, self.publish_array)

    def publish_array(self, data):
        self.publisher_.publish(data)
        self.get_logger().info(f'Sent: {data}')

def main(args=None):
    rclpy.init(args=args)
    node = FloatArrayPublisher()
    node.publish_array([1, 2, 3])
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()