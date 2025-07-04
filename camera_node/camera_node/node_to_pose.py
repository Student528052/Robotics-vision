import sys
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray

from moveit_msgs.action import MoveGroup
from geometry_msgs.msg import PoseStamped
from moveit_msgs.msg import Constraints, OrientationConstraint, PositionConstraint
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import Pose

recieved_position = 0

class MoveToPoseClient(Node):

    def __init__(self):
        super().__init__('move_to_pose_client')
        self._action_client = ActionClient(self, MoveGroup, 'move_action')
        self.subscription = self.create_subscription(Int32MultiArray, 'position_topic', self.store_position, 10)

    def store_position(self, data):
        self.get_logger().info(f'Recieved: {data}')
        recieved_position = data



    def send_goal(self, pose: PoseStamped):
        self.get_logger().info('Waiting for action server...')
        if not self._action_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error('Action server not available!')
            return

        goal_msg = MoveGroup.Goal()
        goal_msg.request.group_name = 'ur_manipulator'  # ✅ Belangrijk!
        goal_msg.request.workspace_parameters.header.frame_id = 'base_link'
        goal_msg.request.goal_constraints.append(self.create_goal_constraints(pose))

        self.get_logger().info(f'Sending goal pose:\n{pose}')
        self._send_goal_future = self._action_client.send_goal_async(goal_msg)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def create_goal_constraints(self, pose: PoseStamped):
        constraints = Constraints()

        # ✅ PositionConstraint
        pc = PositionConstraint()
        pc.header = pose.header
        pc.link_name = 'tool0'
        pc.target_point_offset.x = 0.0
        pc.target_point_offset.y = 0.0
        pc.target_point_offset.z = 0.0
        pc.constraint_region.primitives.append(SolidPrimitive(type=SolidPrimitive.BOX, dimensions=[0.001, 0.001, 0.001]))
        region_pose = Pose()
        region_pose.position = pose.pose.position
        region_pose.orientation.w = 1.0  # default orientation
        pc.constraint_region.primitive_poses.append(region_pose)
        pc.weight = 1.0
        constraints.position_constraints.append(pc)

        # ✅ OrientationConstraint
        oc = OrientationConstraint()
        oc.header = pose.header
        oc.orientation = pose.pose.orientation
        oc.link_name = 'tool0'
        oc.absolute_x_axis_tolerance = 0.1
        oc.absolute_y_axis_tolerance = 0.1
        oc.absolute_z_axis_tolerance = 0.1
        oc.weight = 1.0
        constraints.orientation_constraints.append(oc)

        return constraints

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            rclpy.shutdown()
            return

        self.get_logger().info('Goal accepted :)')
        self._result_future = goal_handle.get_result_async()
        self._result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f'Result received: {result}')
        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = MoveToPoseClient()

    pose = PoseStamped()
    pose.header.frame_id = 'base_link'  # ✅ Correct TF-frame
    pose.pose.position.x = 0.3
    pose.pose.position.y = 0.3
    pose.pose.position.z = 0.3
    pose.pose.orientation.x = 0.0
    pose.pose.orientation.y = 0.0
    pose.pose.orientation.z = 0.0
    pose.pose.orientation.w = 1.0

    node.send_goal(pose)
    rclpy.spin(node)

if __name__ == '__main__':
    main()
