import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from tf2_msgs.msg import TFMessage


class Topic2RelayAdapters(Node):
    """Relay official atlas topics onto standard navigation topic names."""

    def __init__(self):
        super().__init__('topic2_relay_adapters')

        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )

        self._odom_pub = self.create_publisher(Odometry, '/odom', qos)
        self._tf_pub = self.create_publisher(TFMessage, '/tf', qos)

        self.create_subscription(
            Odometry,
            '/atlas/odom_ground_truth',
            self._relay_odom,
            qos,
        )
        self.create_subscription(
            TFMessage,
            '/model/atlas/tf',
            self._relay_tf,
            qos,
        )

        self.get_logger().info('Relaying /atlas/odom_ground_truth to /odom')
        self.get_logger().info('Relaying /model/atlas/tf to /tf')

    def _relay_odom(self, msg):
        self._odom_pub.publish(msg)

    def _relay_tf(self, msg):
        self._tf_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = Topic2RelayAdapters()

    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
