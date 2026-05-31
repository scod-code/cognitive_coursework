#!/usr/bin/env python3
"""
AMCL Initializer Node
Automatically publishes an initial pose to AMCL upon startup.
This bypasses the need for manual "2D Pose Estimate" in RViz.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import OccupancyGrid
import time
import math


class AMCLInitializer(Node):
    def __init__(self):
        super().__init__('amcl_initializer')
        
        # Parameters
        self.declare_parameter('initial_pose_x', 0.0)
        self.declare_parameter('initial_pose_y', 0.0)
        self.declare_parameter('initial_pose_yaw', 0.0)
        self.declare_parameter('pose_covariance_linear', 0.25)
        self.declare_parameter('pose_covariance_angular', 0.785)
        self.declare_parameter('max_wait_time', 10.0)
        
        self.initial_x = self.get_parameter('initial_pose_x').value
        self.initial_y = self.get_parameter('initial_pose_y').value
        self.initial_yaw = self.get_parameter('initial_pose_yaw').value
        self.linear_cov = self.get_parameter('pose_covariance_linear').value
        self.angular_cov = self.get_parameter('pose_covariance_angular').value
        self.max_wait = self.get_parameter('max_wait_time').value
        
        # Publisher for initial pose
        self.initialpose_pub = self.create_publisher(
            PoseWithCovarianceStamped,
            '/initialpose',
            10
        )
        
        # Subscription to map to detect when it's ready
        self.map_sub = self.create_subscription(
            OccupancyGrid,
            '/map',
            self.map_callback,
            10
        )
        
        self.map_received = False
        self.pose_published = False
        self.start_time = time.time()
        
        # Create a timer to check if we should publish anyway (fallback)
        self.timer = self.create_timer(1.0, self.timer_callback)
        
        self.get_logger().info(
            f'AMCL Initializer started. Will set initial pose to '
            f'({self.initial_x}, {self.initial_y}, {math.degrees(self.initial_yaw):.1f}°)'
        )
    
    def map_callback(self, msg):
        """Called when map is received."""
        if not self.pose_published:
            self.get_logger().info('Map received. Publishing initial pose...')
            self.publish_initial_pose()
            self.map_received = True
            self.pose_published = True
    
    def timer_callback(self):
        """Fallback timer to publish pose even if map isn't received."""
        elapsed = time.time() - self.start_time
        
        # Publish pose after 2 seconds even if map not ready
        if not self.pose_published and elapsed > 2.0:
            self.get_logger().warn(
                'Map not received after 2s. Publishing initial pose anyway...'
            )
            self.publish_initial_pose()
            self.pose_published = True
        
        # Stop after max wait time
        if elapsed > self.max_wait:
            self.get_logger().info('Max wait time reached. Shutting down initializer.')
            self.timer.cancel()
            rclpy.shutdown()
    
    def publish_initial_pose(self):
        """Publish the initial pose estimate."""
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()
        
        # Set position
        msg.pose.pose.position.x = self.initial_x
        msg.pose.pose.position.y = self.initial_y
        msg.pose.pose.position.z = 0.0
        
        # Convert yaw to quaternion
        half_yaw = self.initial_yaw / 2.0
        msg.pose.pose.orientation.x = 0.0
        msg.pose.pose.orientation.y = 0.0
        msg.pose.pose.orientation.z = math.sin(half_yaw)
        msg.pose.pose.orientation.w = math.cos(half_yaw)
        
        # Set covariance (6x6 matrix, row-major)
        # Diagonal elements: [x, y, z, rx, ry, rz]
        msg.pose.covariance[0] = self.linear_cov ** 2      # x
        msg.pose.covariance[7] = self.linear_cov ** 2      # y
        msg.pose.covariance[14] = 0.01                     # z (small, fixed height)
        msg.pose.covariance[21] = 0.01                     # rx
        msg.pose.covariance[28] = 0.01                     # ry
        msg.pose.covariance[35] = self.angular_cov ** 2    # rz (yaw)
        
        # Publish
        self.initialpose_pub.publish(msg)
        self.get_logger().info(
            f'Published initial pose: x={self.initial_x:.2f}, '
            f'y={self.initial_y:.2f}, yaw={math.degrees(self.initial_yaw):.1f}°'
        )


def main(args=None):
    rclpy.init(args=args)
    node = AMCLInitializer()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
