import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Int32MultiArray
data = 0 
class FloatArraySubscriber(Node):
    def __init__(self):
        super().__init__('int_reciever')
        self.subscription = self.create_subscription(
        Float32MultiArray,
            'int_topic',
            self.listener_callback,
            10
        )

    def listener_callback(self, msg):
        self.get_logger().info(f'Received: {msg}')
        data = msg.data
        self.get_logger().info(f'Stored data: {data}')

def main(args=None):
    rclpy.init(args=args)
    node = FloatArraySubscriber()

    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()