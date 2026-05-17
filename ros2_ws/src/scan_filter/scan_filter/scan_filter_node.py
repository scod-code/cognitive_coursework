import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan


class ScanFilter(Node):

    def __init__(self):
        super().__init__('scan_filter')

        self.publisher_ = self.create_publisher(
            LaserScan,
            '/scan_filtered',
            10
        )

        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )

        self.previous_ranges = None
        self.alpha = 0.6

        self.get_logger().info('Scan filter node started')

    def scan_callback(self, msg):

        filtered_msg = LaserScan()
        filtered_msg.header = msg.header
        filtered_msg.angle_min = msg.angle_min
        filtered_msg.angle_max = msg.angle_max
        filtered_msg.angle_increment = msg.angle_increment
        filtered_msg.time_increment = msg.time_increment
        filtered_msg.scan_time = msg.scan_time
        filtered_msg.range_min = msg.range_min
        filtered_msg.range_max = msg.range_max
        filtered_msg.intensities = msg.intensities

        if self.previous_ranges is None:
            self.previous_ranges = list(msg.ranges)
            filtered_msg.ranges = list(msg.ranges)
        else:
            filtered_ranges = []

            for i, current_value in enumerate(msg.ranges):
                previous_value = self.previous_ranges[i]

                if math.isinf(current_value) or math.isnan(current_value):
                    filtered_value = previous_value
                else:
                    filtered_value = (
                        self.alpha * current_value
                        + (1.0 - self.alpha) * previous_value
                    )

                filtered_ranges.append(filtered_value)

            self.previous_ranges = filtered_ranges
            filtered_msg.ranges = filtered_ranges

        self.publisher_.publish(filtered_msg)


def main(args=None):
    rclpy.init(args=args)

    node = ScanFilter()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()