"""State-machine tests with a mocked navigator and two fake clocks (ROS-free)."""

import math
import os
import random
import subprocess
import sys

import numpy as np
import pytest

from wall_follower import candidate_generator as cg
from wall_follower.goal_selection import NearestByPathSelector
from wall_follower.mission_core import (
    OUTCOME_FOR, PAUSE_TIMEOUT_REASON, TERMINAL, MissionCore, MissionParams, ParamError,
    PauseCause, Reason, State)

ALL_OK = {'tf': True, 'nav_server': True, 'planner_server': True, 'lifecycle': True,
          'perception': True, 'sole_owner': True}
FRAMES = [('orange', 0.9), ('tree', 0.9), ('vehicle', 0.9)]


def make_map():
    data = np.zeros((60, 60), dtype=np.int16)
    data[0, :] = data[-1, :] = 100
    data[:, 0] = data[:, -1] = 100
    return cg.GridSpec(60, 60, 0.08, -2.4, -2.4, 0.0), data.flatten()


class FakePort:
    """Mock navigator: records commands and asserts one goal in flight."""

    def __init__(self):
        self.goals, self.plans = [], []
        self.cancels = 0
        self.in_flight = False
        self.sent_total = 0

    def send_goal(self, gid, x, y, yaw):
        assert not self.in_flight, 'second goal sent while one is in flight'
        self.in_flight = True
        self.sent_total += 1
        self.goals.append((gid, x, y, yaw))

    def cancel_goal(self):
        self.cancels += 1

    def request_path(self, rid, x, y, yaw):
        self.plans.append((rid, x, y, yaw))


class Rig:
    """Core + mock port + fake clocks."""

    def __init__(self, **over):
        self.params = MissionParams(**over)
        self.params.validate()
        self.sim_t, self.wall_t = 100.0, 1000.0
        self.port = FakePort()
        self.outcomes = []
        self.core = MissionCore(
            self.params, NearestByPathSelector(), self.port, lambda: self.sim_t,
            lambda: self.wall_t, on_outcome=self.outcomes.append)
        self.spec, self.flat = make_map()

    def push_checks(self, **override):
        self.core.on_ready_checks(dict(ALL_OK, **override))

    def step(self, wall_dt=0.5, sim_dt=None):
        self.wall_t += wall_dt
        self.sim_t += wall_dt if sim_dt is None else sim_dt
        self.core.tick()
        assert not self.core.last_error, self.core.last_error

    def ready(self):
        self.core.on_map(self.spec, self.flat)
        self.core.on_pose(0.0, 0.0, 0.0)
        for _ in range(20):
            self.push_checks()
            self.step()
            if self.core.state != State.WAIT_READY:
                return
        raise AssertionError('never became ready')

    def answer_plans(self, plan=lambda x, y: math.hypot(x, y) + 0.5):
        while self.port.plans:
            rid, x, y, _yaw = self.port.plans.pop(0)
            self.core.on_plan_result(rid, plan(x, y))

    def take_goal(self, accept=True):
        gid, x, y, yaw = self.port.goals.pop(0)
        self.core.on_goal_response(gid, accept)
        if not accept:
            self.port.in_flight = False
        return gid, x, y, yaw

    def finish_goal(self, gid, status, pose=None):
        self.port.in_flight = False
        if pose:
            self.core.on_pose(*pose)
        self.core.on_goal_result(gid, status)

    def until(self, pred, max_steps=600, **step_kw):
        for _ in range(max_steps):
            if pred():
                return
            self.answer_plans()
            self.step(**step_kw)
        raise AssertionError(f'condition not reached; state={self.core.state}')

    def to_active_goal(self):
        """Ready the rig and leave one accepted, hanging goal. Returns its id."""
        self.ready()
        self.until(lambda: self.port.goals)
        gid, *_ = self.take_goal()
        assert self.core.nav_state == 'ACTIVE'
        return gid

    def autopilot(self, result=lambda rig, gid, x, y, yaw: 'SUCCEEDED', plan=None, frames=None,
                  yaw_offset=0.0, max_steps=6000):
        for _ in range(max_steps):
            if self.core.state in TERMINAL:
                return
            self.answer_plans(*([plan] if plan else []))
            while self.port.goals:
                gid, x, y, yaw = self.port.goals.pop(0)
                res = result(self, gid, x, y, yaw)
                if res == 'REJECT':
                    self.core.on_goal_response(gid, False)
                    self.port.in_flight = False
                    continue
                self.core.on_goal_response(gid, True)
                if res == 'HANG':
                    continue
                pose = (x, y, yaw + yaw_offset) if res == 'SUCCEEDED' else None
                self.finish_goal(gid, res, pose)
            if frames and self.core.state == State.OBSERVE:
                self.core.on_detection_frame(frames)
            self.step()
        raise AssertionError(f'no terminal state; state={self.core.state}')


# ------------------------------------------------------------------ happy path
def test_happy_path_completes_at_home():
    rig = Rig()
    rig.ready()
    rig.autopilot(frames=FRAMES)
    assert rig.core.state == State.COMPLETE
    assert rig.core.reason == Reason.ALL_REQUIRED_CONFIRMED_AT_HOME
    assert rig.core.goal_records[-1]['kind'] == 'home'
    assert rig.outcomes[-1]['success'] is True
    assert rig.outcomes[-1]['missing_classes'] == []
    assert rig.port.sent_total == 2           # one survey goal, then home


def test_two_sequential_goals_without_evidence_then_partial():
    rig = Rig(max_locations=2, yaw_count=1, min_locations=1)
    rig.ready()
    rig.autopilot()
    assert rig.core.state == State.PARTIAL
    assert rig.core.reason == Reason.CANDIDATES_EXHAUSTED_EVIDENCE_MISSING
    assert rig.port.sent_total >= 2 and rig.core.totals['succeeded'] == rig.port.sent_total
    assert rig.outcomes[-1]['success'] is False
    assert sorted(rig.outcomes[-1]['missing_classes']) == ['orange', 'tree', 'vehicle']


def test_random_results_never_break_single_in_flight():
    rng = random.Random(7)

    def result(rig, gid, x, y, yaw):
        return rng.choice(['SUCCEEDED', 'SUCCEEDED', 'ABORTED', 'REJECT', 'CANCELED'])

    for seed in range(5):
        rng.seed(seed)
        rig = Rig(max_attempts_per_candidate=3, max_consecutive_failures=50,
                  mission_budget_s=900.0)
        rig.ready()
        rig.autopilot(result=result, frames=FRAMES if seed % 2 else None,
                      plan=lambda x, y: None if rng.random() < 0.1 else 1.0)
        assert rig.core.state in TERMINAL
        assert (rig.core.state != State.FAILED_SAFE
                or rig.core.reason == Reason.PLANNER_UNRESPONSIVE)


# ------------------------------------------------------------------- readiness
def test_frozen_clock_in_wait_ready_times_out_on_wall_clock():
    rig = Rig()
    rig.core.on_map(rig.spec, rig.flat)
    rig.core.on_pose(0.0, 0.0, 0.0)
    for _ in range(130):
        rig.push_checks()
        rig.step(0.5, sim_dt=0.0)
        if rig.core.state in TERMINAL:
            break
    assert rig.core.reason == Reason.READINESS_TIMEOUT_CLOCK
    assert rig.core.state == State.FAILED_SAFE


def test_slow_real_time_factor_is_not_a_clock_fault():
    rig = Rig()
    rig.core.on_map(rig.spec, rig.flat)
    rig.core.on_pose(0.0, 0.0, 0.0)
    for _ in range(40):
        rig.push_checks()
        rig.answer_plans()
        if rig.port.goals:
            rig.take_goal()                   # goal accepted, then runs for a long time
        rig.step(1.0, sim_dt=0.2)             # real-time factor 0.2
    assert rig.core.clock_ok
    assert rig.core.state not in (State.PAUSED, State.WAIT_READY) + TERMINAL


def test_perception_without_model_never_becomes_ready():
    rig = Rig()
    rig.core.on_map(rig.spec, rig.flat)
    rig.core.on_pose(0.0, 0.0, 0.0)
    for _ in range(200):
        rig.push_checks(perception=False)
        rig.step()
        if rig.core.state in TERMINAL:
            break
    assert rig.core.reason == Reason.READINESS_TIMEOUT_PERCEPTION


def test_competing_goal_source_blocks_start():
    rig = Rig()
    rig.core.on_map(rig.spec, rig.flat)
    rig.core.on_pose(0.0, 0.0, 0.0)
    for _ in range(100):
        rig.push_checks(sole_owner=False)
        rig.step()
        if rig.core.state in TERMINAL:
            break
    assert rig.core.reason == Reason.COMPETING_GOAL_SOURCE
    assert rig.port.sent_total == 0


def test_bad_map_and_start_outside_map():
    rig = Rig()
    rig.core.on_map(cg.GridSpec(10, 10, 0.1, 0, 0), [0] * 5)
    assert rig.core.reason == Reason.BAD_MAP
    rig = Rig()
    rig.core.on_map(rig.spec, rig.flat)
    rig.core.on_pose(99.0, 99.0, 0.0)
    for _ in range(10):
        rig.push_checks()
        rig.step()
    assert rig.core.reason == Reason.START_OUTSIDE_MAP


# ---------------------------------------------------------- goal failure paths
def test_reject_suppresses_immediately_abort_needs_two():
    rig = Rig()
    rig.ready()
    rig.until(lambda: rig.port.goals)
    gid, *_ = rig.take_goal(accept=False)
    loc = rig.core.candidates[0].location_id
    assert rig.core.suppressed and rig.core.totals['rejected'] == 1
    rig2 = Rig()
    rig2.ready()
    rig2.until(lambda: rig2.port.goals)
    gid, *_ = rig2.take_goal()
    rig2.finish_goal(gid, 'ABORTED')
    assert not rig2.core.suppressed and rig2.core.consecutive_failures == 1
    rig2.until(lambda: rig2.port.goals)
    gid, *_ = rig2.take_goal()
    rig2.finish_goal(gid, 'ABORTED')
    assert rig2.core.suppressed
    assert loc >= 0


def test_unexpected_status_is_treated_as_aborted():
    rig = Rig()
    gid = rig.to_active_goal()
    rig.finish_goal(gid, 'STATUS_6')
    assert rig.core.totals['aborted'] == 1


def test_timeout_cancels_and_waits_for_terminal_status_before_next_send():
    rig = Rig(stall_window_s=1e6)
    gid = rig.to_active_goal()
    rig.until(lambda: rig.core.state == State.CANCEL_AND_WAIT, max_steps=400)
    assert rig.port.cancels == 1 and rig.core.nav_state == 'CANCELING'
    for _ in range(6):
        rig.step()
        assert not rig.port.goals            # nothing new while canceling
    rig.finish_goal(gid, 'CANCELED')
    assert rig.core.totals['timeouts'] == 1 and rig.core.consecutive_failures == 1
    rig.until(lambda: rig.port.goals)
    assert rig.core.state == State.NAVIGATE


def test_stall_detected_without_motion():
    rig = Rig()
    rig.to_active_goal()
    rig.until(lambda: rig.core.state == State.CANCEL_AND_WAIT, max_steps=400)
    assert rig.core.totals['stalls'] == 1 and rig.core.totals['timeouts'] == 0


def test_stop_hold_is_not_a_stall_but_plain_standstill_is():
    rig = Rig()
    rig.to_active_goal()
    rig.core.on_traffic_rule('STOP')
    for _ in range(100):                      # 50 s of crawling at a STOP sign
        rig.step()
    assert rig.port.cancels == 0 and rig.core.state == State.NAVIGATE
    rig.core.on_traffic_rule('NORMAL')
    for _ in range(4):                        # inside the 4 s grace
        rig.step()
    assert rig.port.cancels == 0


def test_yaw_only_motion_is_not_a_stall():
    rig = Rig()
    rig.to_active_goal()
    yaw = 0.0
    for _ in range(100):                      # 50 s, rotating 0.2 rad/s
        yaw += 0.1
        rig.core.on_pose(0.0, 0.0, yaw)
        rig.step()
        if rig.port.cancels:
            break
    assert rig.core.totals['stalls'] == 0


def test_cancel_not_acknowledged_escalates():
    rig = Rig(stall_window_s=1e6)
    rig.to_active_goal()
    rig.until(lambda: rig.core.state in TERMINAL, max_steps=600)
    assert rig.core.reason == Reason.CANCEL_NOT_ACKNOWLEDGED
    assert rig.port.cancels == rig.params.cancel_retries + 1


def test_goal_response_timeout_defers_cancel_and_sends_nothing_else():
    rig = Rig()
    rig.ready()
    rig.until(lambda: rig.port.goals)
    gid, *_ = rig.port.goals.pop(0)
    for _ in range(26):                       # 13 s without a response
        rig.step()
    assert rig.core.nav_state == 'SENDING' and rig.port.cancels == 0
    assert rig.port.sent_total == 1 and rig.core.state not in TERMINAL
    rig.core.on_goal_response(gid, True)      # late acceptance: cancel at once
    assert rig.port.cancels == 1
    rig.finish_goal(gid, 'CANCELED')
    assert rig.core.state != State.FAILED_SAFE


def test_goal_response_never_arrives_is_failed_safe():
    rig = Rig()
    rig.ready()
    rig.until(lambda: rig.port.goals)
    rig.until(lambda: rig.core.state in TERMINAL, max_steps=100)
    assert rig.core.reason == Reason.GOAL_RESPONSE_TIMEOUT
    assert rig.port.sent_total == 1


def test_repeated_nav_failures_is_partial_with_evidence_kept():
    rig = Rig()
    rig.ready()
    rig.autopilot(result=lambda r, g, x, y, yaw: 'ABORTED')
    assert rig.core.reason == Reason.REPEATED_NAV_FAILURES
    assert rig.core.state == State.PARTIAL


def test_external_preemption_is_not_a_failure():
    rig = Rig()
    gid = rig.to_active_goal()
    rig.core.on_external_goal_seen()
    rig.finish_goal(gid, 'ABORTED')
    assert rig.core.totals['preempted'] == 1 and rig.core.consecutive_failures == 0
    assert not rig.core.suppressed
    rig.step()
    assert not rig.port.goals                 # settling before the next goal


def test_unrequested_abort_outside_external_window_is_a_failure():
    rig = Rig()
    gid = rig.to_active_goal()
    rig.core.on_external_goal_seen()
    for _ in range(10):                       # 5 s > external_window_s
        rig.step()
    rig.finish_goal(gid, 'ABORTED')
    assert rig.core.consecutive_failures == 1


def test_yaw_not_reached_then_systemic_partial():
    rig = Rig()
    rig.ready()
    rig.autopilot(yaw_offset=1.5)
    assert rig.core.totals['yaw_not_reached'] >= 3
    assert rig.core.reason == Reason.YAW_CONTROL_UNAVAILABLE


# ----------------------------------------------------------------- end states
def test_budget_exhausted_cancels_active_goal():
    rig = Rig(mission_budget_s=30.0, stall_window_s=1e6)
    rig.to_active_goal()
    rig.until(lambda: rig.core.state in TERMINAL, max_steps=200)
    assert rig.core.reason == Reason.BUDGET_EXHAUSTED and rig.core.state == State.PARTIAL
    assert rig.port.cancels == 1


@pytest.mark.parametrize('attempts,consecutive', [(2, 5), (10, 5)])
def test_home_failure_is_partial_home_unreachable(attempts, consecutive):
    rig = Rig(max_attempts_per_candidate=attempts, max_consecutive_failures=consecutive)
    rig.ready()

    def result(r, gid, x, y, yaw):
        return 'ABORTED' if r.core.goal['kind'] == 'home' else 'SUCCEEDED'

    rig.autopilot(result=result, frames=FRAMES)
    assert rig.core.reason == Reason.EVIDENCE_COMPLETE_HOME_UNREACHABLE
    assert rig.core.state == State.PARTIAL
    assert rig.outcomes[-1]['missing_classes'] == []
    assert rig.outcomes[-1]['success'] is False


def test_operator_shutdown_is_aborted_not_success():
    rig = Rig()
    rig.to_active_goal()
    rig.core.on_shutdown()
    assert rig.core.state == State.ABORTED_BY_OPERATOR
    assert rig.port.cancels == 1 and rig.outcomes[-1]['success'] is False


# ------------------------------------------------------------- planner faults
def test_always_failing_planner_is_failed_safe_not_a_survey_result():
    rig = Rig()
    rig.ready()
    start = rig.wall_t
    rig.autopilot(plan=lambda x, y: None)
    assert rig.core.state == State.FAILED_SAFE
    assert rig.core.reason == Reason.PLANNER_UNRESPONSIVE
    assert rig.wall_t - start < rig.params.mission_budget_s / 2
    assert rig.core.planner_pauses == rig.params.max_planner_pauses + 1
    assert rig.port.sent_total == 0


def test_isolated_no_path_neither_pauses_nor_suppresses():
    rig = Rig()
    rig.ready()
    count = {'n': 0}

    def plan(x, y):
        count['n'] += 1
        return None if count['n'] <= 3 else 1.0

    rig.until(lambda: rig.core.state == State.VALIDATE_PATH or rig.port.plans)
    for _ in range(40):
        rig.answer_plans(plan)
        rig.step()
        assert rig.core.state != State.PAUSED
        if rig.port.goals:
            break
    assert rig.port.goals and not rig.core.suppressed


def test_plan_timeouts_pause_the_planner():
    rig = Rig()
    rig.ready()
    for _ in range(80):
        rig.step()                            # plan requests are never answered
        if rig.core.state == State.PAUSED:
            break
    assert rig.core.pause_cause == PauseCause.PLANNER_UNRESPONSIVE


# ------------------------------------------------------- pauses / clock reset
def test_server_loss_pauses_without_cancel_and_resumes_after_result():
    rig = Rig()
    gid = rig.to_active_goal()
    rig.core.on_input_status(True, False, True)
    rig.step()
    assert rig.core.state == State.PAUSED and rig.core.pause_cause == PauseCause.NAV_SERVER_LOST
    assert rig.port.cancels == 0 and rig.core.nav_state == 'UNKNOWN'
    rig.core.on_input_status(True, True, True)
    rig.step()
    assert rig.core.state == State.PAUSED     # old goal still unresolved
    rig.finish_goal(gid, 'ABORTED')
    rig.until(lambda: rig.core.state != State.PAUSED)
    assert rig.core.consecutive_failures == 0


def test_tf_loss_pauses_then_resumes_or_fails_safe():
    rig = Rig()
    gid = rig.to_active_goal()
    rig.core.on_input_status(False, True, True)
    rig.step()
    assert rig.core.state == State.PAUSED and rig.port.cancels == 1
    rig.finish_goal(gid, 'CANCELED')
    rig.core.on_input_status(True, True, True)
    rig.until(lambda: rig.core.state != State.PAUSED)
    assert rig.core.consecutive_failures == 0
    rig.core.on_input_status(False, True, True)
    rig.until(lambda: rig.core.state in TERMINAL, max_steps=300)
    assert rig.core.reason == Reason.INPUT_LOST_TF


def test_clock_stall_pauses_then_input_lost_clock():
    rig = Rig()
    rig.ready()
    rig.until(lambda: rig.core.state == State.PAUSED, max_steps=100, sim_dt=0.0)
    assert rig.core.pause_cause == PauseCause.CLOCK_STALLED
    rig.until(lambda: rig.core.state in TERMINAL, max_steps=300, sim_dt=0.0)
    assert rig.core.reason == Reason.INPUT_LOST_CLOCK


def test_clock_reset_reruns_readiness_and_keeps_memory():
    rig = Rig()
    gid = rig.to_active_goal()
    rig.core.visited.add(0)
    rig.core.evidence.confirmed['orange'] = {'pose': None}
    rig.core.suppressed[5] = 'REJECTED'
    home = rig.core.home
    rig.sim_t -= 500.0
    rig.step(sim_dt=0.0)
    assert rig.core.state == State.PAUSED and rig.core.pause_cause == PauseCause.CLOCK_RESET
    assert rig.port.cancels == 1
    rig.finish_goal(gid, 'CANCELED')
    seen = []
    for _ in range(10):
        rig.push_checks()
        rig.step()
        seen.append(rig.core.state)
    assert State.WAIT_READY in seen and seen[-1] != State.WAIT_READY
    assert seen.index(State.WAIT_READY) < len(seen) - 1
    assert rig.core.visited == {0} and 'orange' in rig.core.evidence.confirmed
    assert rig.core.suppressed == {5: 'REJECTED'} and rig.core.home == home


# ---------------------------------------------------------- tables / contract
def test_every_reason_maps_to_exactly_one_outcome():
    assert set(OUTCOME_FOR) == set(Reason)
    assert all(v in TERMINAL for v in OUTCOME_FOR.values())
    assert OUTCOME_FOR[Reason.ALL_REQUIRED_CONFIRMED_AT_HOME] == State.COMPLETE
    assert OUTCOME_FOR[Reason.OPERATOR_SHUTDOWN] == State.ABORTED_BY_OPERATOR
    assert OUTCOME_FOR[Reason.COMPETING_GOAL_SOURCE] == State.FAILED_SAFE
    assert set(PAUSE_TIMEOUT_REASON) == set(PauseCause)


@pytest.mark.parametrize('reason', list(Reason))
def test_success_true_only_for_complete(reason):
    rig = Rig()
    rig.core.fail(reason)
    msg = rig.outcomes[-1]
    assert msg['success'] == (OUTCOME_FOR[reason] == State.COMPLETE)
    assert msg['outcome'] == OUTCOME_FOR[reason].value
    assert 'missing_classes' in msg and 'confirmed' in msg


def test_param_validation():
    assert MissionParams.from_dict({'yaw_count': 4}).yaw_count == 4
    with pytest.raises(ParamError):
        MissionParams.from_dict({'yaw_count': 99})
    with pytest.raises(ParamError):
        MissionParams.from_dict({'required_classes': ['dragon']})
    with pytest.raises(ParamError):
        MissionParams.from_dict({'min_conf': 'abc'})
    assert MissionParams(max_locations=40, yaw_count=4).goal_budget == 242


def test_pure_modules_do_not_import_ros():
    root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    code = ("import sys\nfor m in ('rclpy', 'nav2_msgs', 'tf2_ros', 'geometry_msgs'):\n"
            "    sys.modules[m] = None\n"
            "import wall_follower.mission_core, wall_follower.candidate_generator\n"
            "import wall_follower.goal_selection, wall_follower.mission_metrics\n")
    env = dict(os.environ, PYTHONPATH=root + os.pathsep + os.environ.get('PYTHONPATH', ''))
    out = subprocess.run([sys.executable, '-c', code], env=env, capture_output=True, text=True)
    assert out.returncode == 0, out.stderr
