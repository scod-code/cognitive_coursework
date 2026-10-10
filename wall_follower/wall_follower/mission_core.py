"""
Mission state machine (ROS-free).

``MissionCore`` receives events from a thin rclpy adapter and issues commands
through a ``NavigatorPort``.  It owns the single-in-flight-goal invariant, the
two clocks (sim time for robot-time logic, wall time for liveness and deadlines)
and the closed set of mission outcomes.
"""

import dataclasses
import math
import traceback
from dataclasses import dataclass, field
from enum import Enum
from typing import Callable, Dict, List, Optional, Protocol

from wall_follower import candidate_generator as cg
from wall_follower.goal_selection import SelectionContext
from wall_follower.mission_metrics import (
    KNOWN_CLASSES, EvidenceTracker, NullLog, effective_dwell, effective_window)


class State(str, Enum):
    """Supervisor states."""

    WAIT_READY = 'WAIT_READY'
    SELECT_GOAL = 'SELECT_GOAL'
    VALIDATE_PATH = 'VALIDATE_PATH'
    NAVIGATE = 'NAVIGATE'
    OBSERVE = 'OBSERVE'
    UPDATE_MEMORY = 'UPDATE_MEMORY'
    RETURN_HOME = 'RETURN_HOME'
    CANCEL_AND_WAIT = 'CANCEL_AND_WAIT'
    PAUSED = 'PAUSED'
    COMPLETE = 'COMPLETE'
    PARTIAL = 'PARTIAL'
    FAILED_SAFE = 'FAILED_SAFE'
    ABORTED_BY_OPERATOR = 'ABORTED_BY_OPERATOR'


TERMINAL = (State.COMPLETE, State.PARTIAL, State.FAILED_SAFE, State.ABORTED_BY_OPERATOR)


class Reason(str, Enum):
    """Closed set of terminal reasons."""

    ALL_REQUIRED_CONFIRMED_AT_HOME = 'ALL_REQUIRED_CONFIRMED_AT_HOME'
    BUDGET_EXHAUSTED = 'BUDGET_EXHAUSTED'
    CANDIDATES_EXHAUSTED_EVIDENCE_MISSING = 'CANDIDATES_EXHAUSTED_EVIDENCE_MISSING'
    EVIDENCE_COMPLETE_HOME_UNREACHABLE = 'EVIDENCE_COMPLETE_HOME_UNREACHABLE'
    REPEATED_NAV_FAILURES = 'REPEATED_NAV_FAILURES'
    YAW_CONTROL_UNAVAILABLE = 'YAW_CONTROL_UNAVAILABLE'
    READINESS_TIMEOUT_CLOCK = 'READINESS_TIMEOUT_CLOCK'
    READINESS_TIMEOUT_TF = 'READINESS_TIMEOUT_TF'
    READINESS_TIMEOUT_MAP = 'READINESS_TIMEOUT_MAP'
    READINESS_TIMEOUT_NAV_SERVER = 'READINESS_TIMEOUT_NAV_SERVER'
    READINESS_TIMEOUT_PLANNER_SERVER = 'READINESS_TIMEOUT_PLANNER_SERVER'
    READINESS_TIMEOUT_LIFECYCLE = 'READINESS_TIMEOUT_LIFECYCLE'
    READINESS_TIMEOUT_PERCEPTION = 'READINESS_TIMEOUT_PERCEPTION'
    READINESS_TIMEOUT_POSE = 'READINESS_TIMEOUT_POSE'
    COMPETING_GOAL_SOURCE = 'COMPETING_GOAL_SOURCE'
    INPUT_LOST_CLOCK = 'INPUT_LOST_CLOCK'
    INPUT_LOST_TF = 'INPUT_LOST_TF'
    INPUT_LOST_NAV_SERVER = 'INPUT_LOST_NAV_SERVER'
    PLANNER_UNRESPONSIVE = 'PLANNER_UNRESPONSIVE'
    BAD_MAP = 'BAD_MAP'
    BAD_MAP_FRAME = 'BAD_MAP_FRAME'
    NO_SAFE_START = 'NO_SAFE_START'
    START_OUTSIDE_MAP = 'START_OUTSIDE_MAP'
    NO_CANDIDATES = 'NO_CANDIDATES'
    CANCEL_NOT_ACKNOWLEDGED = 'CANCEL_NOT_ACKNOWLEDGED'
    GOAL_RESPONSE_TIMEOUT = 'GOAL_RESPONSE_TIMEOUT'
    INTERNAL_ERROR = 'INTERNAL_ERROR'
    OPERATOR_SHUTDOWN = 'OPERATOR_SHUTDOWN'


_R = Reason
OUTCOME_FOR: Dict[Reason, State] = {
    _R.ALL_REQUIRED_CONFIRMED_AT_HOME: State.COMPLETE,
    _R.BUDGET_EXHAUSTED: State.PARTIAL,
    _R.CANDIDATES_EXHAUSTED_EVIDENCE_MISSING: State.PARTIAL,
    _R.EVIDENCE_COMPLETE_HOME_UNREACHABLE: State.PARTIAL,
    _R.REPEATED_NAV_FAILURES: State.PARTIAL,
    _R.YAW_CONTROL_UNAVAILABLE: State.PARTIAL,
    _R.OPERATOR_SHUTDOWN: State.ABORTED_BY_OPERATOR,
}
for _r in Reason:
    OUTCOME_FOR.setdefault(_r, State.FAILED_SAFE)


class PauseCause(str, Enum):
    """Why the supervisor is PAUSED."""

    NAV_SERVER_LOST = 'NAV_SERVER_LOST'
    CLOCK_STALLED = 'CLOCK_STALLED'
    CLOCK_RESET = 'CLOCK_RESET'
    TF_STALE = 'TF_STALE'
    PLANNER_UNRESPONSIVE = 'PLANNER_UNRESPONSIVE'
    START_UNPLANNABLE = 'START_UNPLANNABLE'


PAUSE_TIMEOUT_REASON = {
    PauseCause.CLOCK_STALLED: Reason.INPUT_LOST_CLOCK,
    PauseCause.CLOCK_RESET: Reason.READINESS_TIMEOUT_CLOCK,
    PauseCause.TF_STALE: Reason.INPUT_LOST_TF,
    PauseCause.NAV_SERVER_LOST: Reason.INPUT_LOST_NAV_SERVER,
    PauseCause.PLANNER_UNRESPONSIVE: Reason.PLANNER_UNRESPONSIVE,
    PauseCause.START_UNPLANNABLE: Reason.PLANNER_UNRESPONSIVE,
}

READY_ORDER = ['clock', 'tf', 'map', 'nav_server', 'planner_server', 'lifecycle',
               'perception', 'pose', 'sole_owner']
READY_REASON = {
    'clock': Reason.READINESS_TIMEOUT_CLOCK, 'tf': Reason.READINESS_TIMEOUT_TF,
    'map': Reason.READINESS_TIMEOUT_MAP, 'nav_server': Reason.READINESS_TIMEOUT_NAV_SERVER,
    'planner_server': Reason.READINESS_TIMEOUT_PLANNER_SERVER,
    'lifecycle': Reason.READINESS_TIMEOUT_LIFECYCLE,
    'perception': Reason.READINESS_TIMEOUT_PERCEPTION,
    'pose': Reason.READINESS_TIMEOUT_POSE, 'sole_owner': Reason.COMPETING_GOAL_SOURCE,
}


class ParamError(ValueError):
    """An invalid mission parameter."""


@dataclass
class MissionParams:
    """All tunables.  Every numeric default is an untuned starting point."""

    required_classes: List[str] = field(default_factory=lambda: ['orange', 'tree', 'vehicle'])
    min_conf: float = 0.45
    confirm_frames: int = 3
    confirm_window_s: float = 3.0
    dwell_s: float = 3.0
    dwell_extend_s: float = 1.5
    dwell_extend_max: int = 2
    yaw_count: int = 4
    yaw_check_tol_rad: float = 0.5
    yaw_fail_systemic_n: int = 3
    same_location_m: float = 0.15
    robot_radius_m: float = 0.18
    safety_margin_m: float = 0.17
    coverage_min: float = 0.95
    spacing_m: float = 0.9
    max_locations: int = 40
    min_locations: int = 6
    validate_k: int = 8
    no_path_systemic_n: int = 4
    max_planner_pauses: int = 3
    max_attempts_per_candidate: int = 2
    max_consecutive_failures: int = 5
    timeout_base_s: float = 30.0
    timeout_min_speed_mps: float = 0.08
    stall_window_s: float = 45.0
    stall_min_move_m: float = 0.10
    stop_hold_grace_s: float = 4.0
    max_stop_hold_extension_s: float = 60.0
    plan_timeout_s: float = 5.0
    cancel_timeout_s: float = 5.0
    cancel_retries: int = 2
    goal_response_timeout_s: float = 10.0
    external_window_s: float = 3.0
    external_settle_s: float = 5.0
    mission_budget_s: float = 2400.0
    max_goals: int = 0
    ready_timeout_clock_s: float = 60.0
    ready_timeout_tf_s: float = 60.0
    ready_timeout_map_s: float = 60.0
    ready_timeout_pose_s: float = 60.0
    ready_timeout_nav_server_s: float = 90.0
    ready_timeout_planner_server_s: float = 90.0
    ready_timeout_lifecycle_s: float = 90.0
    ready_timeout_perception_s: float = 90.0
    ready_timeout_sole_owner_s: float = 30.0
    paused_timeout_s: float = 60.0
    tf_stale_s: float = 2.0
    clock_stall_s: float = 5.0
    require_perception: bool = True
    perception_min_msgs: int = 5
    perception_silence_s: float = 15.0
    perception_model_check: bool = True
    yolo_node_name: str = 'yolo_node'
    lifecycle_nodes: List[str] = field(default_factory=lambda: [
        'map_server', 'planner_server', 'controller_server', 'bt_navigator',
        'behavior_server'])
    selector: str = 'nearest_path'
    candidate_source: str = 'lattice'
    log_dir: str = '~/ros2_coursework_ws/mission_logs'
    tick_hz: float = 5.0

    def validate(self):
        """Raise ``ParamError`` naming the first invalid parameter."""
        if not self.required_classes or any(c not in KNOWN_CLASSES
                                            for c in self.required_classes):
            raise ParamError(f'required_classes must be a non-empty subset of {KNOWN_CLASSES}')
        checks = {
            'min_conf': 0 < self.min_conf <= 1,
            'confirm_frames': 1 <= self.confirm_frames <= 30,
            'confirm_window_s': 0.5 <= self.confirm_window_s <= 30,
            'dwell_s': 0 <= self.dwell_s <= 30,
            'yaw_count': 1 <= self.yaw_count <= 8,
            'spacing_m': 0.3 <= self.spacing_m <= 5,
            'max_locations': self.max_locations >= 1,
            'min_locations': 1 <= self.min_locations <= max(1, self.max_locations),
            'validate_k': 1 <= self.validate_k <= 30,
            'max_attempts_per_candidate': self.max_attempts_per_candidate >= 1,
            'max_consecutive_failures': self.max_consecutive_failures >= 1,
            'no_path_systemic_n': self.no_path_systemic_n >= 2,
            'max_planner_pauses': self.max_planner_pauses >= 1,
            'mission_budget_s': self.mission_budget_s > 0,
            'max_goals': self.max_goals >= 0,
            'tick_hz': 1 <= self.tick_hz <= 20,
            'plan_timeout_s': self.plan_timeout_s > 0,
            'cancel_timeout_s': self.cancel_timeout_s > 0,
            'clock_stall_s': self.clock_stall_s > 0,
            'paused_timeout_s': self.paused_timeout_s > 0,
            'lifecycle_nodes': bool(self.lifecycle_nodes),
        }
        for name, ok in checks.items():
            if not ok:
                raise ParamError(f'invalid parameter {name}={getattr(self, name)!r}')

    @property
    def goal_budget(self) -> int:
        """Effective ``max_goals`` (computed when the parameter is 0)."""
        if self.max_goals > 0:
            return self.max_goals
        return self.max_locations * self.yaw_count + 2 * self.max_locations + 2

    @classmethod
    def from_dict(cls, values: dict) -> 'MissionParams':
        """Build from a dict, casting each value to the field type, and validate."""
        kwargs = {}
        for f in dataclasses.fields(cls):
            if f.name not in values:
                continue
            v = values[f.name]
            try:
                if f.type is float:
                    v = float(v)
                elif f.type is int:
                    v = int(v)
                elif f.type is bool:
                    v = bool(v)
                elif f.type is str:
                    v = str(v)
                else:
                    v = list(v)
            except (TypeError, ValueError) as exc:
                raise ParamError(f'invalid parameter {f.name}: {exc}') from exc
            kwargs[f.name] = v
        params = cls(**kwargs)
        params.validate()
        return params


class NavigatorPort(Protocol):
    """Commands the core issues; all are non-blocking."""

    def send_goal(self, goal_id: int, x: float, y: float, yaw: float) -> None:
        """Send a NavigateToPose goal."""

    def cancel_goal(self) -> None:
        """Cancel the current NavigateToPose goal."""

    def request_path(self, req_id: int, x: float, y: float, yaw: float) -> None:
        """Request a ComputePathToPose plan."""


def angle_diff(a: float, b: float) -> float:
    """Smallest signed difference a - b in radians."""
    return math.atan2(math.sin(a - b), math.cos(a - b))


class MissionCore:
    """The supervisor state machine."""

    def __init__(self, params: MissionParams, selector, port: NavigatorPort,
                 now_sim: Callable[[], float], now_wall: Callable[[], float],
                 log=None, on_outcome: Optional[Callable[[dict], None]] = None,
                 evidence: Optional[EvidenceTracker] = None,
                 logger: Optional[Callable[[str, str], None]] = None):
        """Wire the core to its selector, port, clocks, log and outcome callback."""
        self.p = params
        self.selector = selector
        self.port = port
        self.now_sim = now_sim
        self.now_wall = now_wall
        self.log = log or NullLog()
        self.on_outcome = on_outcome
        self._say = logger or (lambda level, msg: None)
        self.evidence = evidence or EvidenceTracker(
            params.min_conf, params.confirm_frames, params.confirm_window_s)
        self.state = State.WAIT_READY
        self.outcome: Optional[State] = None
        self.reason: Optional[Reason] = None
        self.detail = ''
        self.last_error = ''
        # Inputs
        self.pose = None            # (x, y, yaw) in map
        self.map_spec = None
        self.map_data = None
        self.ready: Dict[str, bool] = {n: False for n in READY_ORDER}
        self.tf_ok = True
        self.server_ok = True
        self.planner_ok = True
        self.traffic_rule = 'NORMAL'
        self._stop_exit_sim = None
        # Clocks
        self._last_sim = None
        self._last_sim_change_wall = None
        self._last_tick_sim = None
        self.clock_ok = False
        self._wait_entry_wall = self.now_wall()
        self._sim_at_entry = None
        self.t0_wall = None
        # Mission memory
        self.candidates = []
        self.gen_info = {}
        self.home = None
        self.visited = set()
        self.suppressed: Dict[int, str] = {}
        self.attempts: Dict[int, int] = {}
        self.no_path_count: Dict[int, int] = {}
        self.recent_no_path: List[tuple] = []
        self.yaw_fail_locations = set()
        self.consecutive_failures = 0
        self.consecutive_no_path = 0
        self.consecutive_plan_timeouts = 0
        self.planner_pauses = 0
        self.goals_sent = 0
        self.totals = {'succeeded': 0, 'aborted': 0, 'timeouts': 0, 'stalls': 0,
                       'rejected': 0, 'preempted': 0, 'interventions': 0,
                       'yaw_not_reached': 0}
        self.goal_records: List[dict] = []
        self.fps_info = {}
        self.dwell_eff = params.dwell_s
        # Selection cycle
        self.path_lengths: Dict[int, float] = {}
        self.validated = set()
        self._cycle_cands: List = []
        self._batch: List = []
        self._cycle_skipped = 0
        self._retry_cycles = 0
        self._cycle_open = False
        self._settle_until = 0.0
        self._plan_req = None
        self._plan_cand = None
        self._plan_sent_wall = 0.0
        self._req_seq = 0
        # Navigation
        self.nav_state = 'IDLE'
        self.goal = None
        self._goal_seq = 0
        self._after_idle = None
        self._pending_cancel = False
        self._cancel_wall = 0.0
        self._cancel_tries = 0
        self._send_wall = 0.0
        self._sending_expired_wall = None
        self._ext_seen_wall = None
        self.home_attempts = 0
        # Observe
        self._obs_start = 0.0
        self._obs_ext = 0
        self._obs_cand = None
        # Pause
        self.pause_cause = None
        self._pause_wall = 0.0
        self._paused_from = State.WAIT_READY
        self.goal_kind = 'candidate'

    # ------------------------------------------------------------------ inputs
    def on_pose(self, x, y, yaw):
        """Record the latest map-frame robot pose."""
        self.pose = (x, y, yaw)

    def on_map(self, spec, flat_data):
        """Store the static map (validated here)."""
        try:
            cg.grid_array(spec, flat_data)
        except cg.MapError as exc:
            self.fail(Reason.BAD_MAP, str(exc))
            return
        self.map_spec, self.map_data = spec, flat_data

    def on_ready_checks(self, checks: Dict[str, bool]):
        """Adapter-evaluated readiness checks (clock, map and pose are derived here)."""
        for name, ok in checks.items():
            if name in self.ready and name not in ('clock', 'map', 'pose'):
                self.ready[name] = bool(ok)

    def on_input_status(self, tf_ok: bool, server_ok: bool, planner_ok: bool = True):
        """Adapter-evaluated liveness of TF and the action servers."""
        self.tf_ok, self.server_ok, self.planner_ok = tf_ok, server_ok, planner_ok

    def set_perception_fps(self, fps: float):
        """Apply the measured detection rate (frames per sim second) to the windows."""
        window, feasible = effective_window(
            self.p.confirm_frames, self.p.confirm_window_s, fps)
        self.evidence.window_s = window
        self.dwell_eff = effective_dwell(self.p.dwell_s, self.p.confirm_frames, fps)
        self.fps_info = {'fps_est_per_sim_s': fps,
                         'confirm_window_s_configured': self.p.confirm_window_s,
                         'confirm_window_s_effective': window,
                         'dwell_s_effective': self.dwell_eff,
                         'confirmation_feasible': feasible}
        if window != self.p.confirm_window_s or not feasible:
            self._say('warn', f'perception rate {fps:.2f}/s: window {window}s '
                              f'feasible={feasible}')

    def on_detection_frame(self, frame):
        """Feed one parsed YOLO frame (None means malformed); only used while OBSERVE."""
        if frame is None:
            self.evidence.count_bad()
            return
        if self.state != State.OBSERVE:
            return
        new = self.evidence.observe(frame, self.now_sim(), self.pose)
        for label in new:
            self.log.event(self.now_sim(), 'CLASS_CONFIRMED', self.state.value, detail=label)
            self._say('info', f'confirmed {label}')

    def on_traffic_rule(self, rule: str):
        """Track STOP-hold so a deliberate crawl is not treated as a stall."""
        if self.traffic_rule == 'STOP' and rule != 'STOP':
            self._stop_exit_sim = self.now_sim()
        self.traffic_rule = rule

    def on_external_goal_seen(self):
        """Note that a /goal_pose message arrived (someone else is sending goals)."""
        if self.nav_state in ('SENDING', 'ACTIVE', 'CANCELING'):
            self._ext_seen_wall = self.now_wall()

    def on_shutdown(self):
        """Operator shutdown."""
        self._finish(Reason.OPERATOR_SHUTDOWN)

    def fail(self, reason: Reason, detail: str = ''):
        """End the mission with a given reason (public for adapter-detected faults)."""
        self._finish(reason, detail)

    # --------------------------------------------------------------- nav events
    def on_goal_response(self, goal_id: int, accepted: bool):
        """Handle the server accepting or rejecting goal ``goal_id``."""
        if self.goal is None or goal_id != self.goal['id'] or self.nav_state != 'SENDING':
            self._say('warn', f'ignoring stale goal response {goal_id}')
            return
        self._sending_expired_wall = None
        if not accepted:
            self.nav_state = 'IDLE'
            self._pending_cancel = False
            if self.state in TERMINAL:
                return
            self.totals['rejected'] += 1
            self.log.event(self.now_sim(), 'GOAL_REJECTED', self.state.value,
                           cid=self.goal['cid'])
            if self._after_idle:
                self._run_after_idle()
            else:
                self._record_failure('REJECTED')
            return
        self.nav_state = 'ACTIVE'
        s = self.now_sim()
        self.goal['start_sim'] = s
        self.goal['ref_pose'] = self.pose
        self.goal['ref_sim'] = s
        self.log.event(self.now_sim(), 'GOAL_ACCEPTED', self.state.value, cid=self.goal['cid'])
        if self._pending_cancel:
            self._pending_cancel = False
            self._issue_cancel()

    def on_goal_result(self, goal_id: int, status: str):
        """Terminal status of goal ``goal_id`` (SUCCEEDED, ABORTED, CANCELED, other)."""
        if self.goal is None or goal_id != self.goal['id'] or self.nav_state == 'IDLE':
            self._say('warn', f'ignoring stale goal result {goal_id}')
            return
        if status not in ('SUCCEEDED', 'ABORTED', 'CANCELED'):
            self._say('error', f'unexpected terminal status {status}, treating as ABORTED')
            status = 'ABORTED'
        self.nav_state = 'IDLE'
        self._pending_cancel = False
        self.log.event(self.now_sim(), 'GOAL_RESULT', self.state.value,
                       cid=self.goal['cid'], status=status)
        if self.state in TERMINAL:
            return
        if self._after_idle:
            self._run_after_idle()
            return
        if self.state == State.PAUSED:
            return
        wall = self.now_wall()
        if (status in ('ABORTED', 'CANCELED') and self._ext_seen_wall is not None
                and wall - self._ext_seen_wall <= self.p.external_window_s):
            self.totals['preempted'] += 1
            self.totals['interventions'] += 1
            self._settle_until = wall + self.p.external_settle_s
            self.goal = None
            self._say('warn', 'goal preempted by an external goal; not counted as a failure')
            self._enter_select()
            return
        if status == 'SUCCEEDED':
            self._on_success()
        else:
            self._record_failure('ABORTED')

    def on_plan_result(self, req_id: int, length: Optional[float]):
        """Planned path length in metres, or None if planning failed."""
        if req_id != self._plan_req or self.state != State.VALIDATE_PATH:
            self._say('warn', f'ignoring stale plan result {req_id}')
            return
        cand = self._plan_cand
        self._plan_req = None
        self.validated.add(cand.location_id)
        if length is None:
            self._on_no_path(cand)
            return
        self.consecutive_no_path = 0
        self.consecutive_plan_timeouts = 0
        self.path_lengths[cand.location_id] = float(length)
        self.log.event(self.now_sim(), 'PLAN_OK', self.state.value, cid=cand.cid,
                       path_length_m=f'{length:.2f}')

    # ------------------------------------------------------------------- tick
    def tick(self):
        """Advance timers and the state machine; call from a steady-clock timer."""
        try:
            self._tick()
        except Exception:
            self.last_error = traceback.format_exc()
            self._say('error', self.last_error)
            self._finish(Reason.INTERNAL_ERROR, self.last_error.splitlines()[-1])

    def _tick(self):
        if self.state in TERMINAL:
            return
        s, w = self.now_sim(), self.now_wall()
        self._update_clock(s, w)
        if self.state in TERMINAL:
            return
        self._supervise_nav(w)
        if self.state in TERMINAL:
            return
        if self.t0_wall is not None and w - self.t0_wall > self.p.mission_budget_s:
            self._finish(
                Reason.EVIDENCE_COMPLETE_HOME_UNREACHABLE if self.state == State.RETURN_HOME
                else Reason.BUDGET_EXHAUSTED)
            return
        if self.state not in (State.WAIT_READY, State.PAUSED) and not self._monitor_inputs():
            return
        st = self.state
        if st == State.WAIT_READY:
            self._tick_wait_ready(s, w)
        elif st == State.PAUSED:
            self._tick_paused(w)
        elif st == State.SELECT_GOAL:
            self._tick_select(w)
        elif st == State.VALIDATE_PATH:
            self._tick_validate(w)
        elif st in (State.NAVIGATE, State.RETURN_HOME):
            self._tick_navigate(s)
        elif st == State.OBSERVE:
            self._tick_observe(s)
        elif st == State.UPDATE_MEMORY:
            self._update_memory()
        self._last_tick_sim = s

    # ------------------------------------------------------------------ clocks
    def _update_clock(self, s, w):
        if self._last_sim is None:
            self._last_sim, self._last_sim_change_wall = s, w
            self._sim_at_entry = s
        if s < self._last_sim - 1.0:
            self._say('error', 'sim clock moved backwards (reset)')
            self._last_sim, self._last_sim_change_wall = s, w
            self.clock_ok = True
            self._rebase_after_reset(s)
            if self.state == State.WAIT_READY:
                self._wait_entry_wall = w
            elif self.state not in (State.PAUSED,):
                self._pause(PauseCause.CLOCK_RESET)
            else:
                self.pause_cause = PauseCause.CLOCK_RESET
            return
        if s != self._last_sim:
            self._last_sim_change_wall = w
        self._last_sim = s
        self.clock_ok = (w - self._last_sim_change_wall) <= self.p.clock_stall_s

    def _rebase_after_reset(self, s):
        self._sim_at_entry = s
        self._last_tick_sim = None
        self._stop_exit_sim = None
        self.evidence.reset_window()
        self.path_lengths.clear()
        self.validated.clear()
        self._cycle_open = False
        if self.goal:
            self.goal['start_sim'] = s
            self.goal['ref_sim'] = s
        self._obs_start = s

    def _monitor_inputs(self) -> bool:
        if not self.clock_ok:
            self._pause(PauseCause.CLOCK_STALLED)
        elif not self.tf_ok:
            self._pause(PauseCause.TF_STALE)
        elif not self.server_ok:
            self._pause(PauseCause.NAV_SERVER_LOST)
        else:
            return True
        return False

    # -------------------------------------------------------------- readiness
    def _tick_wait_ready(self, s, w):
        self.ready['clock'] = self.clock_ok and (s - self._sim_at_entry >= 0.5)
        self.ready['map'] = self.map_spec is not None
        self.ready['pose'] = self.pose is not None
        if not self.p.require_perception:
            self.ready['perception'] = True
        elapsed = w - self._wait_entry_wall
        missing = [n for n in READY_ORDER if not self.ready[n]]
        for name in missing:
            limit = getattr(self.p, f'ready_timeout_{name}_s')
            if elapsed > limit:
                self._finish(READY_REASON[name],
                             f'{name} not ready after {limit:.0f}s; also missing {missing}')
                return
        if not missing:
            self._begin_mission(w)

    def _begin_mission(self, w):
        if not self.candidates:
            if self.home is None:
                self.home = self.pose
            try:
                self.candidates, self.gen_info = cg.generate_candidates(
                    self.map_spec, self.map_data, self.pose[:2],
                    robot_radius_m=self.p.robot_radius_m,
                    safety_margin_m=self.p.safety_margin_m, spacing_m=self.p.spacing_m,
                    max_locations=self.p.max_locations, min_locations=self.p.min_locations,
                    yaw_count=self.p.yaw_count, coverage_min=self.p.coverage_min)
            except cg.NoSafeStartError as exc:
                reason = (Reason.START_OUTSIDE_MAP if 'outside' in str(exc)
                          else Reason.NO_SAFE_START)
                self._finish(reason, str(exc))
                return
            except cg.NoCandidatesError as exc:
                self._finish(Reason.NO_CANDIDATES, str(exc))
                return
            except cg.MapError as exc:
                self._finish(Reason.BAD_MAP, str(exc))
                return
            self._say('info', f'generated {self.gen_info}')
        if self.t0_wall is None:
            self.t0_wall = w
        self._enter_select()

    # ----------------------------------------------------------------- pausing
    def _pause(self, cause: PauseCause):
        self._say('warn', f'PAUSED: {cause.value}')
        self.log.event(self.now_sim(), 'PAUSED', self.state.value, detail=cause.value)
        self._paused_from = self.state
        self.state = State.PAUSED
        self.pause_cause = cause
        self._pause_wall = self.now_wall()
        self._plan_req = None
        self._after_idle = None
        if cause == PauseCause.NAV_SERVER_LOST:
            if self.nav_state != 'IDLE':
                self.nav_state = 'UNKNOWN'
        elif cause in (PauseCause.PLANNER_UNRESPONSIVE, PauseCause.START_UNPLANNABLE):
            pass
        else:
            self._cancel_then(lambda: None)

    def _tick_paused(self, w):
        cause = self.pause_cause
        waited = w - self._pause_wall
        if waited > self.p.paused_timeout_s:
            self._finish(PAUSE_TIMEOUT_REASON[cause], f'paused {waited:.0f}s in {cause.value}')
            return
        if self.nav_state == 'UNKNOWN' and self.server_ok:
            if waited > self.p.cancel_timeout_s:
                self.nav_state = 'IDLE'
                self.goal = None
        if self.nav_state != 'IDLE':
            return
        ok = {
            PauseCause.CLOCK_STALLED: self.clock_ok,
            PauseCause.CLOCK_RESET: self.clock_ok,
            PauseCause.TF_STALE: self.tf_ok,
            PauseCause.NAV_SERVER_LOST: self.server_ok,
            PauseCause.PLANNER_UNRESPONSIVE:
                waited >= self.p.external_settle_s and self.planner_ok and self.server_ok,
            PauseCause.START_UNPLANNABLE:
                waited >= self.p.external_settle_s and self.planner_ok and self.server_ok,
        }[cause]
        if not ok:
            return
        self.log.event(self.now_sim(), 'RESUMED', self.state.value, detail=cause.value)
        self.consecutive_no_path = 0
        self.consecutive_plan_timeouts = 0
        self.pause_cause = None
        if cause == PauseCause.CLOCK_RESET:
            self.state = State.WAIT_READY
            self._wait_entry_wall = w
            self._sim_at_entry = self.now_sim()
            for n in READY_ORDER:
                self.ready[n] = False
        elif self._paused_from == State.RETURN_HOME and self.evidence_complete():
            self.goal = None
            self.state = State.RETURN_HOME
        else:
            self._enter_select()

    # ------------------------------------------------------------ nav protocol
    def _supervise_nav(self, w):
        if self.nav_state == 'CANCELING' and w - self._cancel_wall > self.p.cancel_timeout_s:
            if self._cancel_tries <= self.p.cancel_retries:
                self._issue_cancel()
            else:
                self._finish(Reason.CANCEL_NOT_ACKNOWLEDGED, 'no terminal status after cancel')
        elif self.nav_state == 'SENDING' and self.state not in TERMINAL:
            if self._sending_expired_wall is None:
                if w - self._send_wall > self.p.goal_response_timeout_s:
                    self._sending_expired_wall = w
                    self.state = State.CANCEL_AND_WAIT
                    self._cancel_then(lambda: self._record_failure('ABORTED'))
                    self._say('error', 'no goal response; will cancel when it arrives')
            elif w - self._sending_expired_wall > self.p.cancel_timeout_s:
                self._finish(Reason.GOAL_RESPONSE_TIMEOUT, 'no response to send_goal')

    def _issue_cancel(self):
        self.nav_state = 'CANCELING'
        self._cancel_wall = self.now_wall()
        self._cancel_tries += 1
        self.log.event(self.now_sim(), 'CANCEL_SENT', self.state.value)
        self.port.cancel_goal()

    def _cancel_then(self, fn):
        """Run ``fn`` once no goal is in flight, cancelling the current one if needed."""
        self._after_idle = fn
        if self.nav_state == 'IDLE':
            self._run_after_idle()
        elif self.nav_state == 'ACTIVE':
            self._cancel_tries = 0
            self._issue_cancel()
        elif self.nav_state == 'SENDING':
            self._pending_cancel = True

    def _run_after_idle(self):
        fn, self._after_idle = self._after_idle, None
        if fn:
            fn()

    def _send(self, kind, cid, loc, x, y, yaw, timeout_s):
        assert self.nav_state == 'IDLE', 'single in-flight goal violated'
        self._goal_seq += 1
        self.goals_sent += 1
        self.goal_kind = kind
        self.goal = {'id': self._goal_seq, 'kind': kind, 'cid': cid, 'loc': loc,
                     'x': x, 'y': y, 'yaw': yaw, 'timeout_s': timeout_s,
                     'start_sim': self.now_sim(), 'ref_pose': self.pose,
                     'ref_sim': self.now_sim(), 'stop_ext': 0.0}
        self.nav_state = 'SENDING'
        self._send_wall = self.now_wall()
        self._sending_expired_wall = None
        self._ext_seen_wall = None
        self._cancel_tries = 0
        self.log.event(self.now_sim(), 'GOAL_SENT', self.state.value, cid=cid,
                       x=f'{x:.2f}', y=f'{y:.2f}', yaw=f'{yaw:.2f}')
        self.port.send_goal(self._goal_seq, x, y, yaw)

    # ------------------------------------------------------------- selection
    def _ctx(self) -> SelectionContext:
        return SelectionContext(
            robot_xy=self.pose[:2], visited=set(self.visited),
            suppressed=dict(self.suppressed), confirmed=set(self.evidence.confirmed),
            required=set(self.p.required_classes), path_lengths=dict(self.path_lengths),
            validated=set(self.validated))

    def _enter_select(self):
        self.state = State.SELECT_GOAL
        self.goal = None
        self._cycle_open = False

    def _tick_select(self, w):
        if self.nav_state != 'IDLE' or w < self._settle_until or self.pose is None:
            return
        if self.goals_sent >= self.p.goal_budget:
            self._finish(Reason.BUDGET_EXHAUSTED, 'max_goals reached')
            return
        if not self._cycle_open:
            self.path_lengths.clear()
            self.validated.clear()
            self._cycle_cands = []
            self._cycle_skipped = 0
            self._cycle_open = True
        if self._cycle_cands:
            choice = self.selector.choose(self._cycle_cands, self._ctx())
            if choice is not None:
                self._retry_cycles = 0
                self._send_candidate(choice)
                return
        batch = self.selector.shortlist(self.candidates, self._ctx(), self.p.validate_k)
        if batch:
            self._batch = list(batch)
            self._cycle_cands.extend(batch)
            self.state = State.VALIDATE_PATH
            return
        if self._cycle_skipped > 0 and self._retry_cycles < 2:
            self._retry_cycles += 1
            self._cycle_open = False
            return
        detail = f'confirmed={sorted(self.evidence.confirmed)}'
        self._finish(Reason.CANDIDATES_EXHAUSTED_EVIDENCE_MISSING, detail)

    def _tick_validate(self, w):
        if self._plan_req is not None:
            if w - self._plan_sent_wall > self.p.plan_timeout_s:
                cand = self._plan_cand
                self._plan_req = None
                self.validated.add(cand.location_id)
                self._cycle_skipped += 1
                self.consecutive_plan_timeouts += 1
                self.log.event(self.now_sim(), 'PLAN_FAIL', self.state.value, cid=cand.cid,
                               detail='PLAN_TIMEOUT')
                if self.consecutive_plan_timeouts >= 3:
                    self._planner_pause(PauseCause.PLANNER_UNRESPONSIVE)
            return
        if self.state != State.VALIDATE_PATH:
            return
        while self._batch:
            cand = self._batch.pop(0)
            if cand.location_id in self.validated:
                continue
            if self.pose and (math.hypot(cand.x - self.pose[0], cand.y - self.pose[1])
                              <= self.p.same_location_m):
                self.path_lengths[cand.location_id] = 0.0
                self.validated.add(cand.location_id)
                continue
            self._req_seq += 1
            self._plan_req, self._plan_cand = self._req_seq, cand
            self._plan_sent_wall = w
            self.port.request_path(self._req_seq, cand.x, cand.y, cand.yaw)
            return
        self.state = State.SELECT_GOAL

    def _on_no_path(self, cand):
        loc = cand.location_id
        self.no_path_count[loc] = self.no_path_count.get(loc, 0) + 1
        self.consecutive_no_path += 1
        self._cycle_skipped += 1
        suppressed = self.no_path_count[loc] >= 2
        if suppressed:
            self._suppress_location(loc, 'NO_PATH')
        self.recent_no_path.append((loc, suppressed))
        self.log.event(self.now_sim(), 'PLAN_FAIL', self.state.value, cid=cand.cid,
                       detail='NO_PATH')
        if self.consecutive_no_path >= self.p.no_path_systemic_n:
            for lc, was_suppressed in self.recent_no_path[-self.p.no_path_systemic_n:]:
                self.no_path_count[lc] = max(0, self.no_path_count.get(lc, 1) - 1)
                if was_suppressed:
                    self._unsuppress_location(lc, 'NO_PATH')
            self.recent_no_path.clear()
            self.log.event(self.now_sim(), 'NO_PATH_REVERTED', self.state.value)
            self._planner_pause(PauseCause.START_UNPLANNABLE)

    def _planner_pause(self, cause):
        self.planner_pauses += 1
        if self.planner_pauses > self.p.max_planner_pauses:
            self._finish(Reason.PLANNER_UNRESPONSIVE,
                         f'{self.planner_pauses - 1} planner pauses did not clear the fault')
        else:
            self._pause(cause)

    def _suppress_location(self, loc, reason):
        for c in self.candidates:
            if c.location_id == loc and c.cid not in self.visited:
                self.suppressed[c.cid] = reason
        self.log.event(self.now_sim(), 'CANDIDATE_SUPPRESSED', self.state.value,
                       detail=f'location {loc}: {reason}')

    def _unsuppress_location(self, loc, reason):
        for c in self.candidates:
            if c.location_id == loc and self.suppressed.get(c.cid) == reason:
                del self.suppressed[c.cid]

    def _send_candidate(self, cand):
        length = self.path_lengths.get(cand.location_id, 0.0)
        timeout = min(300.0, max(60.0, self.p.timeout_base_s
                                 + length / self.p.timeout_min_speed_mps))
        self.state = State.NAVIGATE
        self._send('candidate', cand.cid, cand.location_id, cand.x, cand.y, cand.yaw, timeout)

    # -------------------------------------------------------------- navigating
    def _tick_navigate(self, s):
        if self.state == State.RETURN_HOME and self.goal is None:
            if self.nav_state == 'IDLE' and self.pose is not None:
                if self.goals_sent >= self.p.goal_budget:
                    self._finish(Reason.EVIDENCE_COMPLETE_HOME_UNREACHABLE, 'max_goals reached')
                    return
                hx, hy, hyaw = self.home
                dist = math.hypot(hx - self.pose[0], hy - self.pose[1])
                self._send('home', -1, -1, hx, hy, hyaw,
                           min(300.0, max(60.0, self.p.timeout_base_s
                                          + 1.5 * dist / self.p.timeout_min_speed_mps)))
            return
        if self.nav_state != 'ACTIVE' or self.goal is None or self._after_idle:
            return
        g = self.goal
        stop_active = self.traffic_rule == 'STOP' or (
            self._stop_exit_sim is not None
            and s - self._stop_exit_sim < self.p.stop_hold_grace_s)
        if stop_active:
            if self._last_tick_sim is not None:
                g['stop_ext'] = min(self.p.max_stop_hold_extension_s,
                                    g['stop_ext'] + max(0.0, s - self._last_tick_sim))
            g['ref_pose'], g['ref_sim'] = self.pose, s
        if s - g['start_sim'] - g['stop_ext'] > g['timeout_s']:
            self.totals['timeouts'] += 1
            self._fail_goal('TIMEOUT')
            return
        if stop_active or self.pose is None or g['ref_pose'] is None:
            return
        moved = math.hypot(self.pose[0] - g['ref_pose'][0], self.pose[1] - g['ref_pose'][1])
        turned = abs(angle_diff(self.pose[2], g['ref_pose'][2]))
        if moved >= self.p.stall_min_move_m or turned >= 0.2:
            g['ref_pose'], g['ref_sim'] = self.pose, s
        elif s - g['ref_sim'] > self.p.stall_window_s:
            self.totals['stalls'] += 1
            self._fail_goal('STALL')

    def _fail_goal(self, reason):
        self.state = State.CANCEL_AND_WAIT
        self._cancel_then(lambda: self._record_failure(reason))

    def _record_failure(self, reason):
        g = self.goal
        self.goal = None
        self.consecutive_failures += 1
        if reason == 'ABORTED':
            self.totals['aborted'] += 1
        self.goal_records.append({'cid': g['cid'], 'kind': g['kind'], 'result': reason})
        self._say('warn', f"goal {g['cid']} failed: {reason}")
        if g['kind'] == 'home':
            self.home_attempts += 1
            if (reason == 'REJECTED' or self.home_attempts >= self.p.max_attempts_per_candidate
                    or self.consecutive_failures >= self.p.max_consecutive_failures):
                self._finish(Reason.EVIDENCE_COMPLETE_HOME_UNREACHABLE,
                             f'home goal failed: {reason}')
            else:
                self.state = State.RETURN_HOME
            return
        loc = g['loc']
        self.attempts[loc] = self.attempts.get(loc, 0) + 1
        if reason == 'REJECTED' or self.attempts[loc] >= self.p.max_attempts_per_candidate:
            self._suppress_location(loc, reason)
        if self.consecutive_failures >= self.p.max_consecutive_failures:
            self._finish(Reason.REPEATED_NAV_FAILURES, f'{self.consecutive_failures} in a row')
            return
        self._enter_select()

    def _on_success(self):
        g = self.goal
        self.totals['succeeded'] += 1
        self.goal_records.append({'cid': g['cid'], 'kind': g['kind'], 'result': 'SUCCEEDED'})
        if g['kind'] == 'home':
            self.goal = None
            self._finish(Reason.ALL_REQUIRED_CONFIRMED_AT_HOME)
            return
        self.consecutive_failures = 0
        cand = next(c for c in self.candidates if c.cid == g['cid'])
        if self.pose is not None and abs(angle_diff(self.pose[2], cand.yaw)) > \
                self.p.yaw_check_tol_rad:
            self.totals['yaw_not_reached'] += 1
            self.log.event(self.now_sim(), 'YAW_NOT_REACHED', self.state.value, cid=cand.cid)
            self._say('warn', f'yaw not reached at cid {cand.cid}')
            self.suppressed[cand.cid] = 'YAW_NOT_REACHED'
            self.yaw_fail_locations.add(cand.location_id)
            self.goal = None
            if len(self.yaw_fail_locations) >= self.p.yaw_fail_systemic_n:
                self._finish(Reason.YAW_CONTROL_UNAVAILABLE,
                             'goals succeed without reaching the requested heading')
            else:
                self._enter_select()
            return
        self.state = State.OBSERVE
        self._obs_start = self.now_sim()
        self._obs_ext = 0
        self._obs_cand = cand

    # ----------------------------------------------------------------- observe
    def _tick_observe(self, s):
        end = self._obs_start + self.dwell_eff + self._obs_ext * self.p.dwell_extend_s
        if s < end:
            return
        if (self._obs_ext < self.p.dwell_extend_max
                and self.evidence.partial(s, self.p.required_classes)):
            self._obs_ext += 1
            return
        self.visited.add(self._obs_cand.cid)
        self.goal = None
        self.state = State.UPDATE_MEMORY

    def evidence_complete(self) -> bool:
        """Return True when every required class has been confirmed."""
        return set(self.p.required_classes) <= set(self.evidence.confirmed)

    def _update_memory(self):
        self.log.write_json(self.snapshot())
        if self.evidence_complete():
            self.log.event(self.now_sim(), 'STATE_CHANGE', 'RETURN_HOME')
            self.goal_kind = 'home'
            self.home_attempts = 0
            self.state = State.RETURN_HOME
        elif self.goals_sent >= self.p.goal_budget:
            self._finish(Reason.BUDGET_EXHAUSTED, 'max_goals reached')
        else:
            self._enter_select()

    # ----------------------------------------------------------------- outcome
    def _finish(self, reason: Reason, detail: str = ''):
        if self.state in TERMINAL:
            return
        outcome = OUTCOME_FOR[reason]
        if self.nav_state == 'ACTIVE':
            self.nav_state = 'CANCELING'
            self._cancel_wall = self.now_wall()
            self.port.cancel_goal()
        elif self.nav_state == 'SENDING':
            self._pending_cancel = True
        self._after_idle = None
        self.state, self.outcome, self.reason, self.detail = outcome, outcome, reason, detail
        rec = self.snapshot()
        self.log.event(self.now_sim(), 'OUTCOME', outcome.value, detail=f'{reason.value} {detail}')
        self.log.write_json(rec)
        self._say('info' if outcome in (State.COMPLETE, State.ABORTED_BY_OPERATOR) else 'warn',
                  f'MISSION {outcome.value}: {reason.value} {detail}')
        if self.on_outcome:
            self.on_outcome(self.outcome_message())

    def outcome_message(self) -> dict:
        """Compact machine-readable outcome (``success`` only for COMPLETE)."""
        return {'outcome': self.outcome.value if self.outcome else 'RUNNING',
                'reason': self.reason.value if self.reason else '',
                'missing_classes': sorted(set(self.p.required_classes)
                                          - set(self.evidence.confirmed)),
                'confirmed': sorted(self.evidence.confirmed),
                'success': self.outcome == State.COMPLETE}

    def snapshot(self) -> dict:
        """Full mission record for ``mission.json``."""
        msg = self.outcome_message()
        msg.update({
            'state': self.state.value, 'detail': self.detail,
            'params': dataclasses.asdict(self.p), 'home': self.home,
            'generation': self.gen_info, 'perception': self.fps_info,
            'confirmed_detail': self.evidence.confirmed,
            'reconfirmations': self.evidence.reconfirmations,
            'goals': self.goal_records,
            'totals': dict(self.totals, goals_sent=self.goals_sent,
                           bad_frames=self.evidence.bad_frames,
                           planner_pauses=self.planner_pauses),
            'suppressed': {str(k): v for k, v in self.suppressed.items()},
        })
        return msg
