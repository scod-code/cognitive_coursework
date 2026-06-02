"""
Curiosity-inspired exploration goal selector.

This node compares a classic nearest-frontier policy with a lightweight ICM-style
intrinsic reward proxy. It is intentionally small enough for coursework demos:
frontiers are extracted from /map, then scored either by distance-to-robot
frontier selection or by novelty relative to previously selected goals.
"""

import math

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import String


class CuriosityExplorer(Node):
    def __init__(self):
        super().__init__('curiosity_explorer')

        self.declare_parameter('mode', 'icm')
        self.declare_parameter('robot_x', 0.0)
        self.declare_parameter('robot_y', 0.0)
        self.declare_parameter('min_goal_separation', 0.5)
        self.declare_parameter('publish_every_n_maps', 5)

        self.mode = self.get_parameter('mode').value
        self.robot_x = float(self.get_parameter('robot_x').value)
        self.robot_y = float(self.get_parameter('robot_y').value)
        self.min_goal_separation = float(self.get_parameter('min_goal_separation').value)
        self.publish_every_n_maps = int(self.get_parameter('publish_every_n_maps').value)

        if self.mode not in ('icm', 'frontier'):
            self.get_logger().warn(f"Unknown mode '{self.mode}', defaulting to 'icm'")
            self.mode = 'icm'

        self.map_count = 0
        self.last_goal = None
        self.visited_goals = []

        self.map_sub = self.create_subscription(OccupancyGrid, '/map', self.map_callback, 10)
        self.goal_pub = self.create_publisher(PoseStamped, '/goal_pose', 10)
        self.metrics_pub = self.create_publisher(String, '/curiosity_explorer/metrics', 10)

        self.get_logger().info(f'CuriosityExplorer started in {self.mode} mode')

    def map_callback(self, msg):
        self.map_count += 1
        if self.publish_every_n_maps > 1 and self.map_count % self.publish_every_n_maps != 0:
            return

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
