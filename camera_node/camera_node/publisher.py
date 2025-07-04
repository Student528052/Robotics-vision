import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Int32MultiArray

from main_camera import CameraDetection
class Sender(Node):
    def __init__(self):
        super().__init__('int_sender')
        self.publisher_ = self.create_publisher(Int32MultiArray, 'position_topic', 10)
        self.timer = self.create_timer(1.0, self.publish_array)
        self.camera = CameraDetection()

    def publish_array(self):
        test = Int32MultiArray()
        test.data = [1,2,3,4,5]
        camera_data = self.camera.test_print()
        temp = Int32MultiArray()
        temp.data = self.camera.get_angle_length()
        self.publisher_.publish(test)
        self.get_logger().info(f'Sent: {test}')

def main(args=None):
    rclpy.init(args=args)
    node = Sender()
    node.publish_array()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()