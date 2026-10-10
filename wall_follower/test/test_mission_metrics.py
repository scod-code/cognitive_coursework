"""Tests for detection confirmation, perception helpers and the mission log."""

import json
import os

from wall_follower.mission_metrics import (
    EvidenceTracker, MissionLog, effective_dwell, effective_window, estimate_fps,
    find_competitors, parse_frame, perception_ready)


def frame(*labels, conf=0.9):
    return [(label, conf) for label in labels]


def test_fewer_than_n_hits_not_confirmed():
    t = EvidenceTracker(0.45, 3, 3.0)
    assert t.observe(frame('orange'), 0.0) == []
    assert t.observe(frame('orange'), 1.0) == []
    assert 'orange' not in t.confirmed


def test_n_hits_inside_window_confirms():
    t = EvidenceTracker(0.45, 3, 3.0)
    t.observe(frame('orange'), 0.0)
    t.observe(frame('orange'), 1.0)
    assert t.observe(frame('orange'), 2.0, pose=(1, 2, 0)) == ['orange']
    assert t.confirmed['orange']['pose'] == [1, 2, 0]


def test_hits_spread_beyond_window_do_not_confirm():
    t = EvidenceTracker(0.45, 3, 3.0)
    for k in range(6):
        assert t.observe(frame('tree'), 4.0 * k) == []


def test_many_boxes_in_one_frame_count_once():
    t = EvidenceTracker(0.45, 3, 3.0)
    assert t.observe(frame('vehicle', 'vehicle', 'vehicle', 'vehicle'), 0.0) == []
    assert 'vehicle' not in t.confirmed


def test_low_confidence_unknown_labels_and_sticky():
    t = EvidenceTracker(0.45, 2, 3.0)
    t.observe(frame('orange', conf=0.2), 0.0)
    t.observe(frame('orange', conf=0.2), 0.5)
    assert not t.confirmed
    t.observe(frame('dragon'), 0.6)
    assert 'dragon' in t.unknown_labels and not t.confirmed
    t.observe(frame('tree'), 1.0)
    assert t.observe(frame('tree'), 1.5) == ['tree']
    t.observe(frame('tree'), 20.0)
    t.observe(frame('tree'), 20.5)
    assert list(t.confirmed) == ['tree'] and t.reconfirmations['tree'] == 1


def test_partial_and_reset():
    t = EvidenceTracker(0.45, 3, 3.0)
    t.observe(frame('orange'), 0.0)
    assert t.partial(1.0) == {'orange'}
    t.reset_window()
    assert t.partial(1.0) == set()


def test_parse_frame_and_bad_counter():
    assert parse_frame('[{"label":"tree","conf":0.8,"xyxy":[0,0,1,1]}]') == [('tree', 0.8)]
    assert parse_frame('[]') == []
    assert parse_frame('not json') is None
    assert parse_frame('{"a": 1}') is None
    t = EvidenceTracker()
    t.count_bad()
    assert t.bad_frames == 1


def test_effective_window_rules():
    assert effective_window(3, 3.0, 2.0) == (3.0, True)
    assert effective_window(3, 3.0, 0.5) == (9.0, True)           # ceil(1.5 * 6)
    window, feasible = effective_window(3, 3.0, 0.05)             # needs 60 s > cap
    assert window == 30.0 and not feasible
    assert effective_window(3, 3.0, 0.0) == (3.0, False)


def test_effective_dwell_and_fps():
    assert effective_dwell(3.0, 3, 2.0) == 3.0
    assert effective_dwell(3.0, 3, 0.5) == 9.0
    assert estimate_fps([0.0, 1.0, 2.0, 3.0, 4.0]) == 1.0
    assert estimate_fps([1.0]) == 0.0
    assert estimate_fps([1.0, 1.0]) == 0.0


def test_perception_ready_requires_model_and_messages():
    assert perception_ready(0, 5, True)[0] is False
    ok, detail = perception_ready(10, 5, False)
    assert not ok and detail == 'model file missing'
    assert perception_ready(10, 5, True) == (True, 'ok')


def test_find_competitors():
    names = ['/yolo_node', '/autonomous_mission', '/pomdp_goal_selector', '/ns/sign_controller']
    assert find_competitors(names) == ['pomdp_goal_selector', 'sign_controller']
    assert find_competitors(['/autonomous_mission', '/autonomous_mission']) == [
        'autonomous_mission']
    assert find_competitors(['/autonomous_mission', '/controller_server']) == []


def test_log_writes_json_and_csv(tmp_path):
    log = MissionLog(str(tmp_path / 'run'))
    log.event(1.5, 'GOAL_SENT', 'NAVIGATE', cid=3, x='1.0')
    log.write_json({'outcome': 'PARTIAL'})
    assert json.load(open(tmp_path / 'run' / 'mission.json'))['outcome'] == 'PARTIAL'
    rows = open(tmp_path / 'run' / 'events.csv').read().splitlines()
    assert rows[0].startswith('sim_time_s,event') and 'GOAL_SENT' in rows[1]
    log.close()


def test_log_disables_itself_on_unwritable_path(tmp_path):
    blocker = tmp_path / 'file'
    blocker.write_text('x')
    warnings = []
    log = MissionLog(os.path.join(str(blocker), 'sub'), warn=warnings.append)
    assert not log.ok and len(warnings) == 1
    log.event(0.0, 'X')          # must not raise
    log.write_json({})
