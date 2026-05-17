import rclpy
from rclpy.node import Node

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist


class WallFollower(Node):

    def __init__(self):
        super().__init__('wall_follower')

        self.publisher_ = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )

        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )

        self.get_logger().info('Wall follower node started')

    def scan_callback(self, msg):

        front_distance = min(msg.ranges[0:20])
        right_distance = min(msg.ranges[80:100])

        move = Twist()

        # obstacle in front
        if front_distance < 0.6:
            move.linear.x = 0.0
            move.angular.z = 0.8

        # too far from right wall
        elif right_distance > 0.7:
            move.linear.x = 0.12
            move.angular.z = 0.2

        # too close to right wall
        elif right_distance < 0.4:
            move.linear.x = 0.12
            move.angular.z = -0.2

        # correct distance
        else:
            move.linear.x = 0.2
            move.angular.z = 0.0

        self.publisher_.publish(move)


def main(args=None):

    rclpy.init(args=args)

    node = WallFollower()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()