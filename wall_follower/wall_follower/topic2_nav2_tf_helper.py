import rclpy
from geometry_msgs.msg import Point, TransformStamped
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from tf2_ros import TransformBroadcaster
from visualization_msgs.msg import Marker


class Topic2Nav2TfHelper(Node):
    """Expose the official atlas odometry as a Nav2-friendly TF chain."""

    def __init__(self):
        super().__init__('topic2_nav2_tf_helper')

        self.declare_parameter('odom_in_topic', '/atlas/odom_ground_truth')
        self.declare_parameter('odom_out_topic', '/odom')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'atlas/base_link')
        self.declare_parameter('footprint_topic', '/atlas/footprint_marker')
        self.declare_parameter('footprint_length', 0.34)
        self.declare_parameter('footprint_width', 0.28)
        self.declare_parameter('footprint_height', 0.08)
        self.declare_parameter('footprint_publish_rate', 5.0)
        self.declare_parameter('tf_publish_rate', 20.0)

        self.odom_in_topic = self.get_parameter('odom_in_topic').value
        self.odom_out_topic = self.get_parameter('odom_out_topic').value
        self.map_frame = self.get_parameter('map_frame').value
        self.odom_frame = self.get_parameter('odom_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        self.footprint_topic = self.get_parameter('footprint_topic').value
        self.footprint_length = float(self.get_parameter('footprint_length').value)
        self.footprint_width = float(self.get_parameter('footprint_width').value)
        self.footprint_height = float(self.get_parameter('footprint_height').value)
        marker_rate = float(self.get_parameter('footprint_publish_rate').value)
        tf_rate = float(self.get_parameter('tf_publish_rate').value)
        self.latest_odom = None

        reliable_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self.odom_pub = self.create_publisher(
            Odometry,
            self.odom_out_topic,
            reliable_qos,
        )
        self.marker_pub = self.create_publisher(
            Marker,
            self.footprint_topic,
            reliable_qos,
        )
        self.odom_sub = self.create_subscription(
            Odometry,
            self.odom_in_topic,
            self.odom_callback,
            reliable_qos,
        )

        self.tf_broadcaster = TransformBroadcaster(self)

        marker_period = 1.0 / max(marker_rate, 1.0)
        tf_period = 1.0 / max(tf_rate, 1.0)
        self.marker_timer = self.create_timer(marker_period, self.publish_marker)
        self.tf_timer = self.create_timer(tf_period, self.publish_latest_tf)

        self.get_logger().info(
            f'Publishing TF: {self.map_frame} -> {self.odom_frame}'
        )
        self.get_logger().info(
            f'Publishing TF: {self.odom_frame} -> {self.base_frame}'
        )
        self.get_logger().info(
            f'Republishing odom: {self.odom_in_topic} -> '
            f'{self.odom_out_topic}'
        )
        self.get_logger().info(
            f'Publishing atlas footprint marker on {self.footprint_topic}'
        )

    def odom_callback(self, msg):
        nav_odom = Odometry()
        nav_odom.header = msg.header
        nav_odom.header.frame_id = self.odom_frame
        nav_odom.child_frame_id = self.base_frame
        nav_odom.pose = msg.pose
        nav_odom.twist = msg.twist
        self.odom_pub.publish(nav_odom)
        self.latest_odom = msg

        self.broadcast_tf(msg)

    def publish_latest_tf(self):
        if self.latest_odom is None:
            return
        self.broadcast_tf(self.latest_odom)

    def broadcast_tf(self, msg):
        self.tf_broadcaster.sendTransform(self.map_to_odom_transform(msg))
        self.tf_broadcaster.sendTransform(self.odom_to_base_transform(msg))

    def map_to_odom_transform(self, msg):
        transform = TransformStamped()
        transform.header.stamp = msg.header.stamp
        transform.header.frame_id = self.map_frame
        transform.child_frame_id = self.odom_frame
        transform.transform.rotation.w = 1.0
        return transform

    def odom_to_base_transform(self, msg):
        transform = TransformStamped()
        transform.header.stamp = msg.header.stamp
        transform.header.frame_id = self.odom_frame
        transform.child_frame_id = self.base_frame
        transform.transform.translation.x = msg.pose.pose.position.x
        transform.transform.translation.y = msg.pose.pose.position.y
        transform.transform.translation.z = msg.pose.pose.position.z
        transform.transform.rotation = msg.pose.pose.orientation
        return transform

    def publish_marker(self):
        marker = Marker()
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.header.frame_id = self.base_frame
        marker.ns = 'atlas_nav2_footprint'
        marker.id = 0
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.scale.x = 0.035
        marker.color.r = 0.05
        marker.color.g = 0.85
        marker.color.b = 1.0
        marker.color.a = 1.0
        marker.pose.orientation.w = 1.0

        half_l = self.footprint_length * 0.5
        half_w = self.footprint_width * 0.5
        z = self.footprint_height
        corners = [
            (half_l, half_w, z),
            (half_l, -half_w, z),
            (-half_l, -half_w, z),
            (-half_l, half_w, z),
            (half_l, half_w, z),
        ]
        marker.points = [Point(x=x, y=y, z=z_val) for x, y, z_val in corners]
        self.marker_pub.publish(marker)


def main(args=None):
    rclpy.init(args=args)
    node = Topic2Nav2TfHelper()

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
