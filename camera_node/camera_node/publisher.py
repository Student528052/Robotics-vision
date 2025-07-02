import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
-
class FloatArrayPublisher(Node):
    def __init__(self):
        super().__init__('float_array_publisher')
        self.publisher_ = self.create_publisher(Float32MultiArray, 'float_array_topic', 10)
        self.timer = self.create_timer(1.0, self.publish_array)

    def publish_array(self, data):
        self.publisher_.publish(data)
        self.get_logger().info(f'Sent: {data}')