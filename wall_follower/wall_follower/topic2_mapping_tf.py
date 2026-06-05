import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster


class Topic2MappingTf(Node):
    """Publish the TF chain needed to map atlas RGB-D clouds in the map frame."""

    def __init__(self):
        super().__init__('topic2_mapping_tf')

        self.declare_parameter('odom_topic', '/atlas/odom_ground_truth')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('base_frame', 'atlas/base_link')
        self.declare_parameter('camera_frame', 'atlas/realsense')
        self.declare_parameter('camera_x', 0.10)
        self.declare_parameter('camera_y', 0.0)
        self.declare_parameter('camera_z', 0.19)
        self.declare_parameter('camera_roll', 0.0)
        self.declare_parameter('camera_pitch', 0.0)
        self.declare_parameter('camera_yaw', 0.0)

        self.odom_topic = self.get_parameter('odom_topic').value
        self.map_frame = self.get_parameter('map_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        self.camera_frame = self.get_parameter('camera_frame').value

        self.tf_broadcaster = TransformBroadcaster(self)
        self.static_broadcaster = StaticTransformBroadcaster(self)

        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self.odom_sub = self.create_subscription(
            Odometry,
            self.odom_topic,
            self.odom_callback,
            qos,
        )

        self.publish_camera_static_tf()
        self.get_logger().info(
            f'Publishing odom TF from {self.odom_topic}: '
            f'{self.map_frame} -> {self.base_frame}'
        )
        self.get_logger().info(
            f'Publishing static camera TF: {self.base_frame} -> '
            f'{self.camera_frame}'
        )

    def publish_camera_static_tf(self):
        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = self.base_frame
        transform.child_frame_id = self.camera_frame
        transform.transform.translation.x = float(
            self.get_parameter('camera_x').value
        )
        transform.transform.translation.y = float(
            self.get_parameter('camera_y').value
        )
        transform.transform.translation.z = float(
            self.get_parameter('camera_z').value
        )

        roll = float(self.get_parameter('camera_roll').value)
        pitch = float(self.get_parameter('camera_pitch').value)
        yaw = float(self.get_parameter('camera_yaw').value)
        qx, qy, qz, qw = self.quaternion_from_euler(roll, pitch, yaw)
        transform.transform.rotation.x = qx
        transform.transform.rotation.y = qy
        transform.transform.rotation.z = qz
        transform.transform.rotation.w = qw

        self.static_broadcaster.sendTransform(transform)

    def odom_callback(self, msg):
        transform = TransformStamped()
        transform.header.stamp = msg.header.stamp
        transform.header.frame_id = self.map_frame
        transform.child_frame_id = self.base_frame
        transform.transform.translation.x = msg.pose.pose.position.x
        transform.transform.translation.y = msg.pose.pose.position.y
        transform.transform.translation.z = msg.pose.pose.position.z
        transform.transform.rotation = msg.pose.pose.orientation
        self.tf_broadcaster.sendTransform(transform)

    @staticmethod
    def quaternion_from_euler(roll, pitch, yaw):
        import math

        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)

        qw = cr * cp * cy + sr * sp * sy
        qx = sr * cp * cy - cr * sp * sy
        qy = cr * sp * cy + sr * cp * sy
        qz = cr * cp * sy - sr * sp * cy
        return qx, qy, qz, qw


def main(args=None):
    rclpy.init(args=args)
    node = Topic2MappingTf()

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
