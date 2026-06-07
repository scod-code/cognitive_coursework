import json
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

try:
    from nav2_msgs.msg import SpeedLimit
    USE_NAV2_SPEED_LIMIT = True
except Exception:
    from std_msgs.msg import Float64
    USE_NAV2_SPEED_LIMIT = False


class Topic2Nav2TrafficRules(Node):
    def __init__(self):
        super().__init__('topic2_nav2_traffic_rules')

        self.normal_speed = 0.18
        self.slow_speed = 0.05
        self.fast_speed = 0.28
        self.stop_speed = 0.01
        self.timeout_sec = 2.5

        self.current_rule = 'NORMAL'
        self.last_detection_time = 0.0

        self.sub = self.create_subscription(
            String,
            '/yolo/detections_json',
            self.detections_cb,
            10,
        )

        self.pub_state = self.create_publisher(
            String,
            '/traffic_rule_state',
            10,
        )

        if USE_NAV2_SPEED_LIMIT:
            self.pub_speed = self.create_publisher(
                SpeedLimit,
                '/speed_limit',
                10,
            )
        else:
            self.pub_speed = self.create_publisher(
                Float64,
                '/speed_limit',
                10,
            )

        self.timer = self.create_timer(0.2, self.timer_cb)

        self.publish_rule('NORMAL', self.normal_speed)
        self.get_logger().info(
            'Topic 2 Nav2 traffic rules started: '
            '/yolo/detections_json -> /traffic_rule_state + /speed_limit'
        )

    def detections_cb(self, msg):
        try:
            detections = json.loads(msg.data)
        except Exception:
            return

        labels = []
        for det in detections:
            label = str(det.get('label', '')).lower()
            conf = float(det.get('conf', 0.0))
            if conf >= 0.40:
                labels.append((label, conf))

        if not labels:
            return

        # Support both old 3-class labels and new 6-class labels.
        for label, conf in labels:
            if label in ['stopsign', 'stop']:
                self.last_detection_time = time.time()
                self.publish_rule('STOP', self.stop_speed)
                self.get_logger().info(
                    f'STOP detected ({label}, conf={conf:.2f}): '
                    f'speed_limit={self.stop_speed}'
                )
                return

        for label, conf in labels:
            if label in ['slowsign', 'slow']:
                self.last_detection_time = time.time()
                self.publish_rule('SLOW', self.slow_speed)
                self.get_logger().info(
                    f'SLOW detected ({label}, conf={conf:.2f}): '
                    f'speed_limit={self.slow_speed}'
                )
                return

        for label, conf in labels:
            if label in ['fastsign', 'fast']:
                self.last_detection_time = time.time()
                self.publish_rule('FAST', self.fast_speed)
                self.get_logger().info(
                    f'FAST detected ({label}, conf={conf:.2f}): '
                    f'speed_limit={self.fast_speed}'
                )
                return

    def timer_cb(self):
        if self.current_rule != 'NORMAL':
            elapsed = time.time() - self.last_detection_time
            if elapsed > self.timeout_sec:
                self.publish_rule('NORMAL', self.normal_speed)
                self.get_logger().info(
                    f'Traffic rule expired: restored speed_limit={self.normal_speed}'
                )

    def publish_rule(self, rule, speed):
        self.current_rule = rule

        state_msg = String()
        state_msg.data = json.dumps({
            'rule': rule,
            'speed_limit': speed,
        })
        self.pub_state.publish(state_msg)

        if USE_NAV2_SPEED_LIMIT:
            speed_msg = SpeedLimit()
            speed_msg.percentage = False
            speed_msg.speed_limit = float(speed)
            self.pub_speed.publish(speed_msg)
        else:
            speed_msg = Float64()
            speed_msg.data = float(speed)
            self.pub_speed.publish(speed_msg)


def main(args=None):
    rclpy.init(args=args)
    node = Topic2Nav2TrafficRules()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
