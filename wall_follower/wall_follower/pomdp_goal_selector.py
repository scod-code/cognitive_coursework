import json
import math
import re
import time
from typing import Dict, List, Tuple

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from std_msgs.msg import String
from nav2_msgs.action import NavigateToPose


def yaw_to_quaternion(yaw):
    return 0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0)


class PomdpGoalSelector(Node):
    def __init__(self):
        super().__init__("pomdp_goal_selector")

        self.declare_parameter("dry_run", True)
        self.declare_parameter("belief_threshold", 0.65)
        self.declare_parameter("decision_period", 3.0)
        self.declare_parameter("map_frame", "map")
        # YOLO integration: detections on this topic are turned into
        # observations about which corridor/sector the robot is looking at.
        self.declare_parameter("detections_topic", "/yolo/detections_json")
        self.declare_parameter("detection_min_confidence", 0.45)
        # Minimum seconds between belief updates for the same label, so a
        # 10 Hz detection stream does not saturate the belief in one second.
        self.declare_parameter("detection_update_interval", 1.0)
        # Sector poster -> goal hypothesis. Matches the official maze layout
        # used in DEMO_INSTRUCTIONS.md (oranges west, trees east, vehicles north).
        self.declare_parameter(
            "label_to_goal",
            ["orange:left_corridor", "tree:right_corridor", "vehicle:top_corridor"],
        )

        self.dry_run = self.get_parameter("dry_run").value
        self.belief_threshold = float(self.get_parameter("belief_threshold").value)
        self.map_frame = self.get_parameter("map_frame").value
        self.detection_min_confidence = float(
            self.get_parameter("detection_min_confidence").value
        )
        self.detection_update_interval = float(
            self.get_parameter("detection_update_interval").value
        )
        self.label_to_goal: Dict[str, str] = {}
        for entry in self.get_parameter("label_to_goal").value:
            if ":" in entry:
                label, goal = entry.split(":", 1)
                self.label_to_goal[label.strip().lower()] = goal.strip()
        self.last_update_by_label: Dict[str, float] = {}

        self.goals: Dict[str, Dict] = {
            "left_corridor":  {"label": "Explore left corridor",  "goal": (-3.5, 0.0, 0.0), "reward": 1.0},
            "top_corridor":   {"label": "Explore top corridor",   "goal": (0.0, 4.0, 0.0),  "reward": 1.1},
            "right_corridor": {"label": "Explore right corridor", "goal": (3.5, 0.0, 0.0),  "reward": 1.0},
            "exit_area":      {"label": "Move toward exit area",  "goal": (0.0, -3.5, 0.0), "reward": 1.2},
        }

        self.beliefs = {name: 0.50 for name in self.goals}
        self.visited = set()
        self.active_goal = None
        self.is_navigating = False

        self.nav_client = ActionClient(self, NavigateToPose, "/navigate_to_pose")

        # Manual / scripted observation channel (kept for testing).
        self.create_subscription(String, "/landmark_observation", self.observation_callback, 10)
        # Live perception channel from yolo_node.
        detections_topic = self.get_parameter("detections_topic").value
        self.create_subscription(String, detections_topic, self.detections_callback, 10)
        self.create_timer(float(self.get_parameter("decision_period").value), self.decision_step)

        self.get_logger().info("POMDP-inspired goal selector started.")
        self.get_logger().info(f"Dry run: {self.dry_run} (set -p dry_run:=false to dispatch Nav2 goals)")
        self.get_logger().info(f"Subscribed to {detections_topic} and /landmark_observation")
        self.get_logger().info(f"Label -> goal mapping: {self.label_to_goal}")
        self.get_logger().info("Manual observations may also be published like: left_corridor:0.82")

    def parse_observation(self, text):
        match = re.match(r"^([a-zA-Z0-9_]+)\s*[:=\s]\s*([0-9]*\.?[0-9]+)$", text.strip())
        if not match:
            return None, None
        name = match.group(1)
        confidence = max(0.01, min(0.99, float(match.group(2))))
        return name, confidence

    def bayes_update(self, prior, confidence):
        numerator = prior * confidence
        denominator = numerator + (1.0 - prior) * (1.0 - confidence)
        if denominator == 0:
            return prior
        return numerator / denominator

    def apply_observation(self, name, confidence, source):
        old = self.beliefs[name]
        new = self.bayes_update(old, confidence)
        self.beliefs[name] = new
        self.get_logger().info(
            f"Observation ({source}): {name}, confidence={confidence:.2f}, "
            f"belief {old:.2f} -> {new:.2f}"
        )

    def observation_callback(self, msg):
        name, confidence = self.parse_observation(msg.data)

        if name not in self.goals:
            self.get_logger().warn(f"Unknown observation: {msg.data}")
            return

        self.apply_observation(name, confidence, "manual")

    def detections_callback(self, msg):
        """Turn YOLO detections (JSON list of {label, conf, xyxy}) into observations."""
        try:
            detections = json.loads(msg.data)
        except Exception:
            return
        if not isinstance(detections, list):
            return

        # Highest-confidence detection per mapped label in this frame.
        best_by_label: Dict[str, float] = {}
        for det in detections:
            label = str(det.get("label", "")).strip().lower()
            if label not in self.label_to_goal:
                continue
            conf = float(det.get("conf", 0.0))
            if conf < self.detection_min_confidence:
                continue
            if conf > best_by_label.get(label, 0.0):
                best_by_label[label] = conf

        now = time.monotonic()
        for label, conf in best_by_label.items():
            if now - self.last_update_by_label.get(label, 0.0) < self.detection_update_interval:
                continue
            goal_name = self.label_to_goal[label]
            if goal_name not in self.goals:
                self.get_logger().warn(
                    f"label_to_goal maps '{label}' to unknown goal '{goal_name}'"
                )
                continue
            confidence = max(0.01, min(0.99, conf))
            self.apply_observation(goal_name, confidence, f"yolo:{label}")
            self.last_update_by_label[label] = now

    def build_priority_queue(self) -> List[Tuple[float, str]]:
        queue = []
        for name, belief in self.beliefs.items():
            reward = self.goals[name]["reward"]
            visited_penalty = 0.35 if name in self.visited else 0.0
            score = belief * reward - visited_penalty
            queue.append((score, name))
        queue.sort(reverse=True)
        return queue

    def decision_step(self):
        queue = self.build_priority_queue()

        self.get_logger().info("----- POMDP Belief State / Priority Queue -----")
        for rank, (score, name) in enumerate(queue, 1):
            self.get_logger().info(
                f"{rank}. {name:15s} belief={self.beliefs[name]:.2f} "
                f"score={score:.2f} label='{self.goals[name]['label']}'"
            )

        best_score, best_name = queue[0]
        best_belief = self.beliefs[best_name]

        if best_belief < self.belief_threshold:
            self.get_logger().info(
                f"No goal selected. Best belief {best_belief:.2f} is below threshold {self.belief_threshold:.2f}"
            )
            return

        if self.is_navigating:
            self.get_logger().info(f"Already navigating to {self.active_goal}")
            return

        self.select_goal(best_name)

    def select_goal(self, name):
        x, y, yaw = self.goals[name]["goal"]

        self.get_logger().info(
            f"Selected goal: {name} -> x={x:.2f}, y={y:.2f}, yaw={yaw:.2f}"
        )

        if self.dry_run:
            self.get_logger().info("Dry run enabled: goal not sent to Nav2.")
            return

        if not self.nav_client.wait_for_server(timeout_sec=3.0):
            self.get_logger().error("Nav2 /navigate_to_pose server not available.")
            return

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = self.map_frame
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = float(x)
        goal_msg.pose.pose.position.y = float(y)

        qx, qy, qz, qw = yaw_to_quaternion(yaw)
        goal_msg.pose.pose.orientation.x = qx
        goal_msg.pose.pose.orientation.y = qy
        goal_msg.pose.pose.orientation.z = qz
        goal_msg.pose.pose.orientation.w = qw

        self.active_goal = name
        self.is_navigating = True

        future = self.nav_client.send_goal_async(goal_msg)
        future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn("Goal rejected.")
            self.is_navigating = False
            return

        self.get_logger().info(f"Goal accepted: {self.active_goal}")
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.result_callback)

    def result_callback(self, future):
        self.get_logger().info(f"Goal finished: {self.active_goal}")
        self.visited.add(self.active_goal)
        self.active_goal = None
        self.is_navigating = False


def main(args=None):
    rclpy.init(args=args)
    node = PomdpGoalSelector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
