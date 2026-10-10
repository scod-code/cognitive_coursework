"""
Autonomous mission supervisor: the only goal source for the Topic 2 robot.

Thin rclpy adapter around ``MissionCore``.  It never publishes ``/goal_pose`` or any
velocity; it talks to Nav2 only through the ``navigate_to_pose`` and
``compute_path_to_pose`` actions and waits for each terminal result.
"""

import dataclasses
import datetime
import json
import math
import os
import sys
import time

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from lifecycle_msgs.msg import State as LifecycleState
from lifecycle_msgs.srv import GetState
from nav2_msgs.action import ComputePathToPose, NavigateToPose
from nav_msgs.msg import OccupancyGrid
from rcl_interfaces.msg import ParameterDescriptor
from rcl_interfaces.srv import GetParameters
from rclpy.action import ActionClient
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener

from wall_follower import candidate_generator as cg
from wall_follower.goal_selection import check_source_name, make_selector
from wall_follower.mission_core import (
    MissionCore, MissionParams, ParamError, READY_ORDER, Reason)
from wall_follower.mission_metrics import (
    MissionLog, estimate_fps, find_competitors, parse_frame, perception_ready)

_STATUS = {GoalStatus.STATUS_SUCCEEDED: 'SUCCEEDED', GoalStatus.STATUS_ABORTED: 'ABORTED',
           GoalStatus.STATUS_CANCELED: 'CANCELED'}


def yaw_to_quat(yaw):
    """Return (z, w) of a planar yaw quaternion."""
    return math.sin(yaw / 2.0), math.cos(yaw / 2.0)


def quat_to_yaw(q):
    """Yaw from a quaternion message."""
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class AutonomousMission(Node):
    """rclpy adapter feeding ``MissionCore`` and implementing its navigator port."""

    def __init__(self):
        """Declare parameters, create interfaces and start the steady-clock tick."""
        super().__init__('autonomous_mission')
        self.params = self._load_params()
        make_selector(self.params.selector)
        check_source_name(self.params.candidate_source)
        stamp = datetime.datetime.now(datetime.timezone.utc).strftime('%Y%m%dT%H%M%SZ')
        log_dir = os.path.join(os.path.expanduser(self.params.log_dir), stamp)
        self.log_file = MissionLog(log_dir, warn=lambda m: self.get_logger().error(m))
        self.core = MissionCore(
            self.params, make_selector(self.params.selector), self,
            now_sim=lambda: self.get_clock().now().nanoseconds * 1e-9,
            now_wall=time.monotonic, log=self.log_file, on_outcome=self._publish_outcome,
            logger=self._say)
        self._handle = None
        self._det_msgs = 0
        self._det_stamps = []
        self._last_det_wall = None
        self._silent_warned = False
        self._fps_done = False
        self._model_ok = not self.params.perception_model_check
        self._model_pending = False
        self._lifecycle_active = {n: False for n in self.params.lifecycle_nodes}
        self._lifecycle_pending = {}
        self._last_wait_log = 0.0
        self._qos_logged = False

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.plan_client = ActionClient(self, ComputePathToPose, 'compute_path_to_pose')
        self.state_clients = {
            n: self.create_client(GetState, f'/{n}/get_state')
            for n in self.params.lifecycle_nodes}
        self.yolo_params = self.create_client(
            GetParameters, f'/{self.params.yolo_node_name}/get_parameters')

        map_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(OccupancyGrid, '/map', self._on_map, map_qos)
        self.create_subscription(String, '/yolo/detections_json', self._on_detections, 10)
        self.create_subscription(String, '/traffic_rule_state', self._on_traffic, 10)
        self.create_subscription(PoseStamped, '/goal_pose',
                                 lambda _m: self.core.on_external_goal_seen(), 10)
        self.outcome_pub = self.create_publisher(
            String, '/autonomous_mission/outcome',
            QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                       durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.create_timer(1.0 / self.params.tick_hz, self._on_tick,
                          clock=Clock(clock_type=ClockType.STEADY_TIME))
        self.get_logger().info(
            f'autonomous_mission started; logs in {log_dir}; waiting for readiness')

    # ---------------------------------------------------------------- params
    def _load_params(self) -> MissionParams:
        defaults = MissionParams()
        desc = ParameterDescriptor(dynamic_typing=True)
        values = {}
        for f in dataclasses.fields(MissionParams):
            default = getattr(defaults, f.name)
            self.declare_parameter(f.name, default, desc)
            values[f.name] = self.get_parameter(f.name).value
        return MissionParams.from_dict(values)

    def _say(self, level, msg):
        getattr(self.get_logger(), {'warn': 'warning'}.get(level, level))(msg)

    # ---------------------------------------------------------------- inputs
    def _on_map(self, msg):
        if self.core.map_spec is not None:
            return
        if msg.header.frame_id != 'map':
            self.core.fail(Reason.BAD_MAP_FRAME, f'map frame {msg.header.frame_id!r}')
            return
        q = msg.info.origin.orientation
        spec = cg.GridSpec(msg.info.width, msg.info.height, msg.info.resolution,
                           msg.info.origin.position.x, msg.info.origin.position.y,
                           quat_to_yaw(q))
        self.core.on_map(spec, list(msg.data))

    def _on_detections(self, msg):
        self._det_msgs += 1
        self._last_det_wall = time.monotonic()
        self._silent_warned = False
        now_sim = self.get_clock().now().nanoseconds * 1e-9
        if now_sim > 0.0:                      # ignore frames seen before /clock flows
            self._det_stamps.append(now_sim)
            del self._det_stamps[:-max(self.params.perception_min_msgs, 5)]
        self.core.on_detection_frame(parse_frame(msg.data))

    def _on_traffic(self, msg):
        try:
            self.core.on_traffic_rule(str(json.loads(msg.data).get('rule', 'NORMAL')))
        except (ValueError, AttributeError):
            pass

    # ------------------------------------------------------------------ tick
    def _on_tick(self):
        self._poll_inputs()
        self.core.tick()

    def _poll_inputs(self):
        p = self.params
        tf_ok = False
        try:
            tr = self.tf_buffer.lookup_transform('map', 'atlas/base_link', rclpy.time.Time())
            stamp = tr.header.stamp.sec + tr.header.stamp.nanosec * 1e-9
            now = self.get_clock().now().nanoseconds * 1e-9
            tf_ok = stamp == 0.0 or (now - stamp) <= p.tf_stale_s
            t = tr.transform
            self.core.on_pose(t.translation.x, t.translation.y, quat_to_yaw(t.rotation))
        except Exception:   # tf2 raises several lookup exception types
            tf_ok = False
        nav_ok = self.nav_client.server_is_ready()
        plan_ok = self.plan_client.server_is_ready()
        self.core.on_input_status(tf_ok, nav_ok and plan_ok, plan_ok)
        if not self.core.state.value == 'WAIT_READY':
            self._check_perception_silence()
            return
        self._poll_lifecycle()
        perception_ok = self._poll_perception()
        competitors = find_competitors(
            n if not ns.rstrip('/') else f'{ns}/{n}'
            for n, ns in self.get_node_names_and_namespaces())
        if competitors:
            self._warn_every(f'competing goal sources running: {competitors}')
        self.core.on_ready_checks({
            'tf': tf_ok, 'nav_server': nav_ok, 'planner_server': plan_ok,
            'lifecycle': all(self._lifecycle_active.values()),
            'perception': perception_ok, 'sole_owner': not competitors})
        missing = [n for n in READY_ORDER if not self.core.ready[n]]
        if missing:
            self._warn_every(f'waiting for readiness: {missing}')
        elif not self._qos_logged:
            self._qos_logged = True
            pubs = [i.node_name for i in self.get_publishers_info_by_topic('/atlas/cmd_vel')]
            self.get_logger().info(f'/atlas/cmd_vel publishers: {pubs}')

    def _warn_every(self, msg, period=5.0):
        now = time.monotonic()
        if now - self._last_wait_log >= period:
            self._last_wait_log = now
            self.get_logger().warning(msg)

    def _poll_lifecycle(self):
        for name, client in self.state_clients.items():
            if self._lifecycle_active[name]:
                continue
            sent = self._lifecycle_pending.get(name)
            if sent is not None and time.monotonic() - sent < 3.0:
                continue                       # a response may still arrive; else retry
            if not client.service_is_ready():
                continue
            self._lifecycle_pending[name] = time.monotonic()
            fut = client.call_async(GetState.Request())
            fut.add_done_callback(lambda f, n=name: self._on_lifecycle(n, f))

    def _on_lifecycle(self, name, fut):
        self._lifecycle_pending.pop(name, None)
        try:
            self._lifecycle_active[name] = (
                fut.result().current_state.id == LifecycleState.PRIMARY_STATE_ACTIVE)
        except Exception:
            self._lifecycle_active[name] = False

    def _poll_perception(self) -> bool:
        p = self.params
        if not p.require_perception:
            return True
        if not self._model_ok and not self._model_pending and self.yolo_params.service_is_ready():
            self._model_pending = True
            req = GetParameters.Request()
            req.names = ['model_path']
            self.yolo_params.call_async(req).add_done_callback(self._on_model_param)
        ok, _detail = perception_ready(self._det_msgs, p.perception_min_msgs, self._model_ok)
        if ok and not self._fps_done:
            fps = estimate_fps(self._det_stamps)
            if fps > 0.0:                      # wait for a usable sim-time estimate
                self._fps_done = True
                self.core.set_perception_fps(fps)
        return ok

    def _on_model_param(self, fut):
        self._model_pending = False
        try:
            value = fut.result().values[0].string_value
            self._model_ok = bool(value) and os.path.isfile(os.path.expanduser(value))
            if not self._model_ok:
                self.get_logger().error(f'YOLO model file missing: {value!r}')
        except Exception as exc:
            self.get_logger().warning(f'could not read YOLO model_path: {exc}')

    def _check_perception_silence(self):
        if (self.params.require_perception and self._last_det_wall is not None
                and not self._silent_warned
                and time.monotonic() - self._last_det_wall > self.params.perception_silence_s):
            self._silent_warned = True
            self.get_logger().warning('PERCEPTION_SILENT: no detection messages')

    # ----------------------------------------------------------- navigator port
    def send_goal(self, goal_id, x, y, yaw):
        """Send a NavigateToPose goal (non-blocking)."""
        goal = NavigateToPose.Goal()
        goal.pose = self._pose(x, y, yaw)
        self._handle = None
        fut = self.nav_client.send_goal_async(goal)
        fut.add_done_callback(lambda f, gid=goal_id: self._on_goal_response(gid, f))

    def _on_goal_response(self, goal_id, fut):
        try:
            handle = fut.result()
        except Exception as exc:
            self.get_logger().error(f'send_goal failed: {exc}')
            self.core.on_goal_response(goal_id, False)
            return
        if handle.accepted:
            self._handle = handle
            handle.get_result_async().add_done_callback(
                lambda f, gid=goal_id: self._on_goal_result(gid, f))
        self.core.on_goal_response(goal_id, bool(handle.accepted))

    def _on_goal_result(self, goal_id, fut):
        try:
            status = _STATUS.get(fut.result().status, f'STATUS_{fut.result().status}')
        except Exception as exc:
            self.get_logger().error(f'result failed: {exc}')
            status = 'ABORTED'
        self.core.on_goal_result(goal_id, status)

    def cancel_goal(self):
        """Ask Nav2 to cancel the current goal (terminal status arrives via the result)."""
        if self._handle is not None:
            self._handle.cancel_goal_async()

    def request_path(self, req_id, x, y, yaw):
        """Request a ComputePathToPose plan from the current robot pose."""
        goal = ComputePathToPose.Goal()
        goal.goal = self._pose(x, y, yaw)
        goal.use_start = False
        fut = self.plan_client.send_goal_async(goal)
        fut.add_done_callback(lambda f, rid=req_id: self._on_plan_response(rid, f))

    def _on_plan_response(self, req_id, fut):
        try:
            handle = fut.result()
        except Exception:
            self.core.on_plan_result(req_id, None)
            return
        if not handle.accepted:
            self.core.on_plan_result(req_id, None)
            return
        handle.get_result_async().add_done_callback(
            lambda f, rid=req_id: self._on_plan_result(rid, f))

    def _on_plan_result(self, req_id, fut):
        try:
            res = fut.result()
            poses = res.result.path.poses
            if res.status != GoalStatus.STATUS_SUCCEEDED or len(poses) == 0:
                self.core.on_plan_result(req_id, None)
                return
            length = sum(
                math.hypot(b.pose.position.x - a.pose.position.x,
                           b.pose.position.y - a.pose.position.y)
                for a, b in zip(poses, poses[1:]))
            self.core.on_plan_result(req_id, length)
        except Exception:
            self.core.on_plan_result(req_id, None)

    def _pose(self, x, y, yaw):
        msg = PoseStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.position.x, msg.pose.position.y = float(x), float(y)
        msg.pose.orientation.z, msg.pose.orientation.w = yaw_to_quat(yaw)
        return msg

    # --------------------------------------------------------------- outcome
    def _publish_outcome(self, message):
        self.outcome_pub.publish(String(data=json.dumps(message)))


def main(args=None):
    """Run the autonomous mission node."""
    rclpy.init(args=args)
    try:
        node = AutonomousMission()
    except (ParamError, ValueError) as exc:
        print(f'[autonomous_mission] invalid configuration: {exc}', file=sys.stderr)
        rclpy.shutdown()
        sys.exit(2)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.core.on_shutdown()
            for _ in range(10):
                rclpy.spin_once(node, timeout_sec=0.1)
        except Exception:
            pass
        node.log_file.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
