"""Detection confirmation, perception helpers and mission logging (ROS-free)."""

import csv
import json
import math
import os
from collections import deque
from typing import Dict, Iterable, List, Optional, Sequence, Tuple

KNOWN_CLASSES = ('fastsign', 'orange', 'slowsign', 'stopsign', 'tree', 'vehicle')

# Nodes that would compete with the mission node as a goal or velocity source.
COMPETING_NODES = ('pomdp_goal_selector', 'goal_publisher', 'curiosity_explorer',
                   'topic2_goal_pose_bridge', 'sign_controller', 'wall_follower_node')


def find_competitors(node_names: Iterable[str], own_name: str = 'autonomous_mission') -> List[str]:
    """Return names of nodes that violate the single-goal-owner rule."""
    found = []
    own_seen = 0
    for name in node_names:
        base = name.rsplit('/', 1)[-1]
        if base in COMPETING_NODES:
            found.append(base)
        elif base == own_name:
            own_seen += 1
    if own_seen > 1:
        found.append(own_name)
    return sorted(set(found))


def perception_ready(msgs: int, min_msgs: int, model_ok: bool) -> Tuple[bool, str]:
    """Perception is ready only if frames flow AND the model file exists."""
    if msgs < min_msgs:
        return False, f'only {msgs}/{min_msgs} detection messages received'
    if not model_ok:
        return False, 'model file missing'
    return True, 'ok'


def estimate_fps(stamps: Sequence[float]) -> float:
    """Frames per second from node-clock stamps: ``(n - 1) / (t_last - t_first)``."""
    if len(stamps) < 2 or stamps[-1] <= stamps[0]:
        return 0.0
    return (len(stamps) - 1) / (stamps[-1] - stamps[0])


def effective_window(confirm_frames: int, confirm_window_s: float, fps_est: float,
                     cap_s: float = 30.0) -> Tuple[float, bool]:
    """Return ``(window_s, feasible)`` so a slow detector can still confirm a class."""
    if fps_est <= 0.0:
        return confirm_window_s, False
    need = confirm_frames / fps_est
    if need <= confirm_window_s:
        return confirm_window_s, True
    return min(cap_s, float(math.ceil(1.5 * need))), need <= cap_s


def effective_dwell(dwell_s: float, confirm_frames: int, fps_est: float,
                    cap_s: float = 30.0) -> float:
    """Raise the dwell if it is too short to collect ``confirm_frames`` frames."""
    if fps_est <= 0.0 or dwell_s * fps_est >= confirm_frames:
        return dwell_s
    return min(cap_s, float(math.ceil(1.5 * confirm_frames / fps_est)))


def parse_frame(text: str) -> Optional[List[Tuple[str, float]]]:
    """Parse ``/yolo/detections_json``; None if malformed."""
    try:
        items = json.loads(text)
        return [(str(d['label']), float(d.get('conf', 0.0))) for d in items]
    except (ValueError, TypeError, KeyError, AttributeError):
        return None


class EvidenceTracker:
    """
    Confirm a class after ``confirm_frames`` frames with a hit inside a time window.

    A frame is one hit per class however many boxes it contains, so hits are never
    summed per detection.  Confirmation is sticky.
    """

    def __init__(self, min_conf=0.45, confirm_frames=3, window_s=3.0):
        """Create a tracker with the given (untuned) thresholds."""
        self.min_conf = min_conf
        self.confirm_frames = confirm_frames
        self.window_s = window_s
        self.hits: Dict[str, deque] = {}
        self.max_conf: Dict[str, float] = {}
        self.confirmed: Dict[str, dict] = {}
        self.reconfirmations: Dict[str, int] = {}
        self.bad_frames = 0
        self.unknown_labels = set()

    def reset_window(self):
        """Discard pending hits (used after a clock reset); confirmed classes stay."""
        self.hits.clear()

    def count_bad(self):
        """Record a malformed frame."""
        self.bad_frames += 1

    def observe(self, frame, t, pose=None) -> List[str]:
        """Add one frame at time ``t``; return labels newly confirmed by it."""
        new = []
        labels = set()
        for label, conf in frame:
            if label not in KNOWN_CLASSES:
                self.unknown_labels.add(label)
                continue
            if conf >= self.min_conf:
                labels.add(label)
                self.max_conf[label] = max(self.max_conf.get(label, 0.0), conf)
        for label in labels:
            q = self.hits.setdefault(label, deque())
            q.append(t)
            while q and t - q[0] > self.window_s:
                q.popleft()
            if len(q) >= self.confirm_frames:
                if label in self.confirmed:
                    self.reconfirmations[label] = self.reconfirmations.get(label, 0) + 1
                    q.clear()
                else:
                    self.confirmed[label] = {
                        'pose': list(pose) if pose else None, 'confirmed_at': t,
                        'hits': len(q), 'max_conf': self.max_conf.get(label, 0.0)}
                    new.append(label)
        return new

    def partial(self, t, labels: Optional[Iterable[str]] = None) -> set:
        """Unconfirmed classes with some but not enough recent hits at time ``t``."""
        out = set()
        for label, q in self.hits.items():
            if label in self.confirmed or (labels is not None and label not in labels):
                continue
            recent = [h for h in q if t - h <= self.window_s]
            if 0 < len(recent) < self.confirm_frames:
                out.add(label)
        return out


EVENT_COLUMNS = ['sim_time_s', 'event', 'state', 'cid', 'x', 'y', 'yaw', 'status',
                 'path_length_m', 'duration_s', 'detail']


class NullLog:
    """Log sink that does nothing (tests and disabled logging)."""

    def event(self, *args, **kwargs):
        """Ignore the event."""

    def write_json(self, data):
        """Ignore the record."""


class MissionLog:
    """Write ``events.csv`` (flushed per event) and an atomic ``mission.json``."""

    def __init__(self, directory: str, warn=None):
        """Create ``directory``; on any I/O error disable logging and warn once."""
        self.ok = True
        self.dir = directory
        self._warn = warn or (lambda msg: None)
        self._csv = None
        self._writer = None
        try:
            os.makedirs(directory, exist_ok=True)
            self._csv = open(os.path.join(directory, 'events.csv'), 'w', newline='')
            self._writer = csv.DictWriter(self._csv, fieldnames=EVENT_COLUMNS)
            self._writer.writeheader()
            self._csv.flush()
        except OSError as exc:
            self._disable(exc)

    def _disable(self, exc):
        if self.ok:
            self._warn(f'mission log disabled: {exc}')
        self.ok = False

    def event(self, sim_time_s, event, state='', **fields):
        """Append one event row."""
        if not self.ok:
            return
        row = {'sim_time_s': f'{sim_time_s:.3f}', 'event': event, 'state': state}
        row.update({k: v for k, v in fields.items() if k in EVENT_COLUMNS})
        try:
            self._writer.writerow(row)
            self._csv.flush()
        except OSError as exc:
            self._disable(exc)

    def write_json(self, data):
        """Atomically rewrite ``mission.json``."""
        if not self.ok:
            return
        path = os.path.join(self.dir, 'mission.json')
        try:
            with open(path + '.tmp', 'w') as fh:
                json.dump(data, fh, indent=2, default=str)
            os.replace(path + '.tmp', path)
        except OSError as exc:
            self._disable(exc)

    def close(self):
        """Close the CSV file."""
        if self._csv:
            try:
                self._csv.close()
            except OSError:
                pass
