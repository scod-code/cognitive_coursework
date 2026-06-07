"""
Wall-Following Navigation Node.

Reactive wall-following + open-area wandering for the detection arena.

Strategy:
    1. Subscribe to /scan (LiDAR, 360 degrees, 0.12-8.0 m range).
    2. Divide scan into left, centre, right sectors.
    3. If walls are detected (valid readings exist in range):
       - Use PID control to balance left/right distance (wall-following).
       - Slow down if obstacle ahead.
    4. If NO walls are detected (robot is in open space):
       - Drive forward with a gentle turn to eventually reach a wall.
    5. Publish to /wall_follower/cmd_vel (sign_controller forwards to /cmd_vel).

Topics:
    Subscriptions:
        /scan: LaserScan (sensor_msgs/LaserScan)
    Publications:
        /wall_follower/cmd_vel: Twist commands (geometry_msgs/Twist)
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

import numpy as np
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

from .kalman_filter import KalmanFilter1D


class WallFollowerNode(Node):
    """
    Reactive wall-following with open-area wandering.

    In the detection arena (30 m × 30 m), the robot starts at the centre
    and initially has no walls within LiDAR range.  The node will drive
    forward (with a slight angular bias) until it reaches a wall, then
    switch to PID-based wall-following.
    """

    def __init__(self):
        super().__init__('wall_follower')

        # Declare parameters
        self.declare_parameter('target_wall_distance', 1.0)
        self.declare_parameter('linear_speed', 0.25)
        self.declare_parameter('wander_speed', 0.3)
        self.declare_parameter('wander_turn', 0.15)
        self.declare_parameter('pid_kp', 0.8)
        self.declare_parameter('pid_ki', 0.0)
        self.declare_parameter('pid_kd', 0.2)
        self.declare_parameter('angular_max', 1.0)
        self.declare_parameter('use_kalman_filter', True)

        # Get parameters
        self.target_wall_distance = self.get_parameter('target_wall_distance').value
        self.linear_speed = self.get_parameter('linear_speed').value
        self.wander_speed = self.get_parameter('wander_speed').value
        self.wander_turn = self.get_parameter('wander_turn').value
        self.pid_kp = self.get_parameter('pid_kp').value
        self.pid_ki = self.get_parameter('pid_ki').value
        self.pid_kd = self.get_parameter('pid_kd').value
        self.angular_max = self.get_parameter('angular_max').value
        self.use_kalman = self.get_parameter('use_kalman_filter').value

        # Sensor range limits (must match LiDAR config in URDF: 0.12 - 8.0 m)
        self.range_min = 0.12
        self.range_max = 7.9  # slightly below sensor max to exclude inf

        # QoS profiles
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5
        )
        cmd_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # Publisher
        self.twist_pub = self.create_publisher(Twist, '/wall_follower/cmd_vel', cmd_qos)

        # Subscriber (LiDAR only — primary sensor, always available)
        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, qos_profile=sensor_qos
        )

        # PID state
        self.prev_error = 0.0
        self.integral_error = 0.0

        # Kalman filters for left/centre/right regions
        self.kalman_left = KalmanFilter1D(process_noise=0.01, measurement_noise=0.05)
        self.kalman_right = KalmanFilter1D(process_noise=0.01, measurement_noise=0.05)
        self.kalman_center = KalmanFilter1D(process_noise=0.01, measurement_noise=0.05)

        # Publish a steady command even before first scan arrives
        self.timer = self.create_timer(0.1, self._heartbeat)
        self._last_twist = Twist()
        self._last_twist.linear.x = self.wander_speed
        self._last_twist.angular.z = self.wander_turn

        self.get_logger().info(
            f'Wall-Follower ready. target={self.target_wall_distance}m, '
            f'speed={self.linear_speed}m/s, wander={self.wander_speed}m/s'
        )

    # ----------------------------------------------------------
    # Heartbeat: always publish so sign_controller always has data
    # ----------------------------------------------------------
    def _heartbeat(self):
        self.twist_pub.publish(self._last_twist)

    # ----------------------------------------------------------
    # Main sensor callback
    # ----------------------------------------------------------
    def scan_callback(self, msg):
        """Process LaserScan from Gazebo LiDAR."""
        ranges = np.array(msg.ranges, dtype=np.float64)
        n = len(ranges)
        if n == 0:
            return

        # Replace invalid readings with NaN
        invalid = (ranges < self.range_min) | (ranges > self.range_max) | np.isinf(ranges)
        ranges[invalid] = np.nan

        # Divide into three sectors (front-left, front-centre, front-right)
        # LiDAR convention: index 0 = front, going counter-clockwise
        # Front sector: -60° to +60° (indices roughly 0..60 and 300..360)
        # Left sector: 30° to 150° (indices 30..150)
        # Right sector: 210° to 330° (indices 210..330)
        sector_left = ranges[30:150]
        sector_center = np.concatenate([ranges[0:30], ranges[330:]])
        sector_right = ranges[210:330]

        left_valid = sector_left[~np.isnan(sector_left)]
        center_valid = sector_center[~np.isnan(sector_center)]
        right_valid = sector_right[~np.isnan(sector_right)]

        has_left = len(left_valid) > 5
        has_center = len(center_valid) > 5
        has_right = len(right_valid) > 5

        # If we have almost no valid readings in any sector, wander
        if not has_left and not has_right and not has_center:
            self._last_twist = Twist()
            self._last_twist.linear.x = self.wander_speed
            self._last_twist.angular.z = self.wander_turn
            return

        # Compute mean distances (use large fallback for missing sectors)
        depth_left = float(np.mean(left_valid)) if has_left else 8.0
        depth_center = float(np.mean(center_valid)) if has_center else 8.0
        depth_right = float(np.mean(right_valid)) if has_right else 8.0

        # Kalman filtering
        if self.use_kalman:
            depth_left = self.kalman_left.filter_measurement(depth_left)
            depth_center = self.kalman_center.filter_measurement(depth_center)
            depth_right = self.kalman_right.filter_measurement(depth_right)

        # Compute control
        self._compute_control(depth_left, depth_center, depth_right)

    # ----------------------------------------------------------
    # PID wall-following control
    # ----------------------------------------------------------
    def _compute_control(self, depth_left, depth_center, depth_right):
        """PID-based wall-following with obstacle avoidance."""
        # Error: positive means right is closer, steer left (positive angular.z)
        error = (depth_right - depth_left) / 2.0

        # Front obstacle check
        if depth_center < self.target_wall_distance * 0.7:
            # Something very close ahead — stop forward, turn away
            forward_speed = 0.0
            # Turn toward the side with more space
            if depth_left > depth_right:
                angular_velocity = self.angular_max * 0.7
            else:
                angular_velocity = -self.angular_max * 0.7
        else:
            # Normal wall-following
            if depth_center < self.target_wall_distance * 1.5:
                # Wall ahead but not critical — slow down
                forward_speed = self.linear_speed * 0.5
            else:
                forward_speed = self.linear_speed

            # PID
            self.integral_error += error * 0.1
            self.integral_error = float(np.clip(self.integral_error, -1.0, 1.0))

            derivative = (error - self.prev_error) / 0.1
            self.prev_error = error

            angular_velocity = (
                self.pid_kp * error +
                self.pid_ki * self.integral_error +
                self.pid_kd * derivative
            )
            angular_velocity = float(np.clip(angular_velocity, -self.angular_max, self.angular_max))

        twist = Twist()
        twist.linear.x = float(forward_speed)
        twist.angular.z = float(angular_velocity)
        self._last_twist = twist


def main(args=None):
    rclpy.init(args=args)
    node = WallFollowerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == '__main__':
    main()