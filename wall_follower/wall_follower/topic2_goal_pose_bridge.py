import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node


class Topic2GoalPoseBridge(Node):
    """Forward RViz /goal_pose messages to Nav2's NavigateToPose action."""

    def __init__(self):
        super().__init__('topic2_goal_pose_bridge')

        self.declare_parameter('goal_pose_topic', '/goal_pose')
        self.declare_parameter('action_name', 'navigate_to_pose')

        goal_pose_topic = self.get_parameter('goal_pose_topic').value
        action_name = self.get_parameter('action_name').value

        self.action_client = ActionClient(self, NavigateToPose, action_name)
        self.goal_sub = self.create_subscription(
            PoseStamped,
            goal_pose_topic,
            self.goal_pose_callback,
            10,
        )

        self.get_logger().info(
            f'Forwarding {goal_pose_topic} to NavigateToPose action {action_name}'
        )

    def goal_pose_callback(self, msg):
        if not self.action_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().warning(
                'NavigateToPose action server is not available yet',
                throttle_duration_sec=2.0,
            )
            return

        goal = NavigateToPose.Goal()
        goal.pose = msg
        self.get_logger().info(
            f'Sending Nav2 goal: frame={msg.header.frame_id} '
            f'x={msg.pose.position.x:.2f} y={msg.pose.position.y:.2f}'
        )
        future = self.action_client.send_goal_async(goal)
        future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warning('Nav2 goal was rejected')
            return

        self.get_logger().info('Nav2 goal accepted')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.goal_result_callback)

    def goal_result_callback(self, future):
        result = future.result()
        self.get_logger().info(f'Nav2 goal finished with status {result.status}')


def main(args=None):
    rclpy.init(args=args)
    node = Topic2GoalPoseBridge()

    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
