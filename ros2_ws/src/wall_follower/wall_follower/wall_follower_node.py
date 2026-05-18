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

        # PID target distance from wall
        self.target_distance = 0.6

        # PID proportional gain
        self.kp = 1.0

        self.get_logger().info('PID Wall follower node started')

    def scan_callback(self, msg):

        # front obstacle detection
        front_distance = min(msg.ranges[0:20])

        # right wall detection
        right_distance = min(msg.ranges[80:100])

        move = Twist()

        # obstacle in front
        if front_distance < 0.6:

            move.linear.x = 0.0
            move.angular.z = 0.8

        else:

            # PID error
            error = self.target_distance - right_distance

            # proportional controller
            correction = self.kp * error

            # forward speed
            move.linear.x = 0.15

            # steering correction
            move.angular.z = correction

        self.publisher_.publish(move)


def main(args=None):

    rclpy.init(args=args)

    node = WallFollower()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()