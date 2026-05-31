import math

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import LaserScan, PointCloud2
from sensor_msgs_py import point_cloud2


class ScanToPointCloud(Node):
    def __init__(self):
        super().__init__('scan_to_pointcloud')

        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('cloud_topic', '/scan_cloud')
        self.declare_parameter('fixed_z', 0.0)

        scan_topic = self.get_parameter('scan_topic').value
        cloud_topic = self.get_parameter('cloud_topic').value
        self.fixed_z = float(self.get_parameter('fixed_z').value)

        self.publisher = self.create_publisher(
            PointCloud2,
            cloud_topic,
            qos_profile_sensor_data
        )

        self.subscription = self.create_subscription(
            LaserScan,
            scan_topic,
            self.scan_callback,
            qos_profile_sensor_data
        )

        self.get_logger().info(
            f'Converting LaserScan {scan_topic} to PointCloud2 {cloud_topic}'
        )

    def scan_callback(self, msg: LaserScan):
        points = []

        angle = msg.angle_min

        for distance in msg.ranges:
            if math.isfinite(distance) and msg.range_min <= distance <= msg.range_max:
                x = distance * math.cos(angle)
                y = distance * math.sin(angle)
                z = self.fixed_z
                points.append((x, y, z))

            angle += msg.angle_increment

        cloud = point_cloud2.create_cloud_xyz32(msg.header, points)
        self.publisher.publish(cloud)


def main(args=None):
    rclpy.init(args=args)
    node = ScanToPointCloud()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
