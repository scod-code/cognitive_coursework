"""
Curiosity-inspired exploration goal selector.

This node compares a classic nearest-frontier policy with a lightweight ICM-style
intrinsic reward proxy. It is intentionally small enough for coursework demos:
frontiers are extracted from an OccupancyGrid, then scored either by
distance-to-robot frontier selection or by novelty relative to previously
selected goals.

Map source: by default /projected_map, the 2D projection OctoMap publishes
while the robot explores (topic2_octomap_with_nav2.launch.py sets
incremental_2D_projection). Unknown cells there are genuinely unexplored, so
frontiers are meaningful. The static Nav2 map on /map can be used instead with
-p map_topic:=/map, but its only unknown cells are outside the maze.

Robot pose comes from /odom (republished by topic2_nav2_tf_helper), so the
distance weighting follows the robot instead of a fixed start point.
Decisions run on a timer against the latest map, so a map that is published
only once (latched) still yields repeated goal selection.
"""

import math

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, OccupancyGrid
from std_msgs.msg import String


class CuriosityExplorer(Node):
    def __init__(self):
        super().__init__('curiosity_explorer')

        self.declare_parameter('mode', 'icm')
        self.declare_parameter('map_topic', '/projected_map')
        self.declare_parameter('map_transient_local', True)
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('robot_x', 0.0)   # initial pose until /odom arrives
        self.declare_parameter('robot_y', 0.0)
        self.declare_parameter('min_goal_separation', 0.5)
        self.declare_parameter('decision_period', 5.0)
        self.declare_parameter('publish_goals', True)

        self.mode = self.get_parameter('mode').value
        self.map_topic = self.get_parameter('map_topic').value
        self.odom_topic = self.get_parameter('odom_topic').value
        self.robot_x = float(self.get_parameter('robot_x').value)
        self.robot_y = float(self.get_parameter('robot_y').value)
        self.min_goal_separation = float(self.get_parameter('min_goal_separation').value)
        self.decision_period = float(self.get_parameter('decision_period').value)
        self.publish_goals = bool(self.get_parameter('publish_goals').value)
        self.have_odom = False

        if self.mode not in ('icm', 'frontier'):
            self.get_logger().warn(f"Unknown mode '{self.mode}', defaulting to 'icm'")
            self.mode = 'icm'

        self.latest_map = None
        self.last_goal = None
        self.visited_goals = []

        # Latched map publishers (nav2 map_server, octomap_server) use
        # TRANSIENT_LOCAL; a volatile subscriber that starts late never
        # receives the map. Match the durability so the map arrives.
        map_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            durability=(
                DurabilityPolicy.TRANSIENT_LOCAL
                if bool(self.get_parameter('map_transient_local').value)
                else DurabilityPolicy.VOLATILE
            ),
        )
        self.map_sub = self.create_subscription(
            OccupancyGrid, self.map_topic, self.map_callback, map_qos)
        self.odom_sub = self.create_subscription(
            Odometry, self.odom_topic, self.odom_callback, 10)
        self.goal_pub = self.create_publisher(PoseStamped, '/goal_pose', 10)
        self.metrics_pub = self.create_publisher(String, '/curiosity_explorer/metrics', 10)
        self.decision_timer = self.create_timer(self.decision_period, self.decision_step)

        self.get_logger().info(
            f'CuriosityExplorer started in {self.mode} mode: map={self.map_topic}, '
            f'odom={self.odom_topic}, decision every {self.decision_period:.1f}s, '
            f'publish_goals={self.publish_goals}')

    def odom_callback(self, msg):
        self.robot_x = float(msg.pose.pose.position.x)
        self.robot_y = float(msg.pose.pose.position.y)
        self.have_odom = True

    def map_callback(self, msg):
        self.latest_map = msg

    def decision_step(self):
        msg = self.latest_map
        if msg is None:
            self.get_logger().info(
                f'Waiting for a map on {self.map_topic}', throttle_duration_sec=10.0)
            return
        if not self.have_odom:
            self.get_logger().info(
                f'No odometry yet on {self.odom_topic}; using initial pose',
                throttle_duration_sec=10.0)

        frontiers = self.extract_frontiers(msg)
        explored = sum(1 for cell in msg.data if cell == 0)
        unknown = sum(1 for cell in msg.data if cell == -1)

        if not frontiers:
            self.publish_metrics(explored, unknown, 0, 0.0)
            return

        if self.mode == 'frontier':
            goal_cell, score = self.select_nearest_frontier(frontiers, msg)
        else:
            goal_cell, score = self.select_curiosity_frontier(frontiers, msg)

        if self.publish_goals:
            self.publish_goal(goal_cell, msg)
        self.publish_metrics(explored, unknown, len(frontiers), score)

    def extract_frontiers(self, msg):
        width = msg.info.width
        height = msg.info.height
        data = msg.data
        frontiers = []

        for y in range(1, height - 1):
            row = y * width
            for x in range(1, width - 1):
                idx = row + x
                if data[idx] != -1:
                    continue
                neighbours = (
                    data[idx - 1],
                    data[idx + 1],
                    data[idx - width],
                    data[idx + width],
                )
                if any(value == 0 for value in neighbours):
                    frontiers.append((x, y))

        return frontiers

    def select_nearest_frontier(self, frontiers, msg):
        robot_cell = self.world_to_cell(self.robot_x, self.robot_y, msg)
        return min(
            ((cell, self.cell_distance(cell, robot_cell)) for cell in frontiers),
            key=lambda item: item[1],
        )

    def select_curiosity_frontier(self, frontiers, msg):
        robot_cell = self.world_to_cell(self.robot_x, self.robot_y, msg)
        best_cell = frontiers[0]
        best_score = -1.0

        for cell in frontiers:
            travel_cost = self.cell_distance(cell, robot_cell)
            novelty = self.goal_novelty(cell, msg)
            score = novelty / (1.0 + travel_cost)
            if score > best_score:
                best_cell = cell
                best_score = score

        return best_cell, best_score

    def goal_novelty(self, cell, msg):
        if not self.visited_goals:
            return 1.0

        world = self.cell_to_world(cell, msg)
        nearest = min(
            math.hypot(world[0] - old[0], world[1] - old[1])
            for old in self.visited_goals
        )
        return max(nearest, self.min_goal_separation)

    def publish_goal(self, cell, msg):
        x, y = self.cell_to_world(cell, msg)
        if self.last_goal is not None:
            if math.hypot(x - self.last_goal[0], y - self.last_goal[1]) < self.min_goal_separation:
                return

        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.pose.position.x = float(x)
        goal.pose.position.y = float(y)
        goal.pose.orientation.w = 1.0

        self.goal_pub.publish(goal)
        self.last_goal = (x, y)
        self.visited_goals.append((x, y))
        self.get_logger().info(f'Published {self.mode} exploration goal at ({x:.2f}, {y:.2f})')

    def publish_metrics(self, explored, unknown, frontier_count, score):
        msg = String()
        msg.data = (
            f'mode={self.mode},explored={explored},unknown={unknown},'
            f'frontiers={frontier_count},score={score:.3f}'
        )
        self.metrics_pub.publish(msg)

    @staticmethod
    def cell_distance(a, b):
        return math.hypot(a[0] - b[0], a[1] - b[1])

    @staticmethod
    def cell_to_world(cell, msg):
        x = msg.info.origin.position.x + (cell[0] + 0.5) * msg.info.resolution
        y = msg.info.origin.position.y + (cell[1] + 0.5) * msg.info.resolution
        return x, y

    @staticmethod
    def world_to_cell(x, y, msg):
        cx = int((x - msg.info.origin.position.x) / msg.info.resolution)
        cy = int((y - msg.info.origin.position.y) / msg.info.resolution)
        cx = max(0, min(msg.info.width - 1, cx))
        cy = max(0, min(msg.info.height - 1, cy))
        return cx, cy


def main(args=None):
    rclpy.init(args=args)
    node = CuriosityExplorer()
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
