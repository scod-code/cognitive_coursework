import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2


class Topic2PointCloudFilter(Node):
    """Filter atlas RGB-D points before OctoMap insertion."""

    def __init__(self):
        super().__init__('topic2_pointcloud_filter')

        self.declare_parameter('input_topic', '/atlas/rgbd_camera/points')
        self.declare_parameter(
            'output_topic',
            '/atlas/rgbd_camera/points_filtered',
        )
        self.declare_parameter('min_x', 0.15)
        self.declare_parameter('max_x', 4.0)
        self.declare_parameter('min_y', -2.5)
        self.declare_parameter('max_y', 2.5)
        self.declare_parameter('min_z', -0.10)
        self.declare_parameter('max_z', 1.50)
        self.declare_parameter('floor_z', 0.05)
        self.declare_parameter('downsample_step', 4)

        self.input_topic = self.get_parameter('input_topic').value
        self.output_topic = self.get_parameter('output_topic').value
        self.min_x = float(self.get_parameter('min_x').value)
        self.max_x = float(self.get_parameter('max_x').value)
        self.min_y = float(self.get_parameter('min_y').value)
        self.max_y = float(self.get_parameter('max_y').value)
        self.min_z = float(self.get_parameter('min_z').value)
        self.max_z = float(self.get_parameter('max_z').value)
        self.floor_z = float(self.get_parameter('floor_z').value)
        self.downsample_step = max(
            1,
            int(self.get_parameter('downsample_step').value),
        )

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.publisher = self.create_publisher(
            PointCloud2,
            self.output_topic,
            sensor_qos,
        )
        self.subscription = self.create_subscription(
            PointCloud2,
            self.input_topic,
            self.cloud_callback,
            sensor_qos,
        )

        self.get_logger().info(
            f'Filtering {self.input_topic} -> {self.output_topic}'
        )
        self.get_logger().info(
            'Limits: '
            f'x=[{self.min_x:.2f}, {self.max_x:.2f}], '
            f'y=[{self.min_y:.2f}, {self.max_y:.2f}], '
            f'z=[{self.min_z:.2f}, {self.max_z:.2f}], '
            f'floor_z={self.floor_z:.2f}, '
            f'downsample_step={self.downsample_step}'
        )

    def cloud_callback(self, msg):
        try:
            points = self.read_xyz_points(msg)
            raw_count = points.shape[0]

            if raw_count == 0:
                self.publish_filtered(msg, points)
                self.log_counts(raw_count, 0)
                return

            valid_mask = np.isfinite(points).all(axis=1)
            points = points[valid_mask]

            bounds_mask = (
                (points[:, 0] >= self.min_x) &
                (points[:, 0] <= self.max_x) &
                (points[:, 1] >= self.min_y) &
                (points[:, 1] <= self.max_y) &
                (points[:, 2] >= self.min_z) &
                (points[:, 2] <= self.max_z) &
                (points[:, 2] >= self.floor_z)
            )
            points = points[bounds_mask]

            if self.downsample_step > 1 and points.shape[0] > 0:
                points = points[::self.downsample_step]

            self.publish_filtered(msg, points)
            self.log_counts(raw_count, points.shape[0])

        except Exception as exc:
            self.get_logger().error(f'Point cloud filter failed: {exc}')

    def read_xyz_points(self, msg):
        raw_points = point_cloud2.read_points(
            msg,
            field_names=('x', 'y', 'z'),
            skip_nans=True,
        )

        if isinstance(raw_points, np.ndarray) and raw_points.dtype.names:
            xs = raw_points['x'].astype(np.float64)
            ys = raw_points['y'].astype(np.float64)
            zs = raw_points['z'].astype(np.float64)
            return np.column_stack((xs, ys, zs))

        points = np.asarray(list(raw_points), dtype=np.float64)
        if points.size == 0:
            return np.empty((0, 3), dtype=np.float64)
        return points.reshape((-1, 3))

    def publish_filtered(self, source_msg, points):
        cloud = point_cloud2.create_cloud_xyz32(
            source_msg.header,
            points.astype(np.float32, copy=False).tolist(),
        )
        self.publisher.publish(cloud)

    def log_counts(self, raw_count, filtered_count):
        self.get_logger().info(
            f'point cloud filter: raw={raw_count} filtered={filtered_count}',
            throttle_duration_sec=3.0,
        )


def main(args=None):
    rclpy.init(args=args)
    node = Topic2PointCloudFilter()

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
