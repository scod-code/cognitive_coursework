"""Tests for grid maths and safe-candidate generation (ROS-free)."""

import math
import time

import numpy as np
import pytest
from scipy import ndimage

from wall_follower import candidate_generator as cg


def room(w=60, h=60, res=0.08, ox=-2.4, oy=-2.4, yaw=0.0):
    """Return (spec, 2-D data) of an empty room with a one-cell wall border."""
    data = np.zeros((h, w), dtype=np.int16)
    data[0, :] = data[-1, :] = 100
    data[:, 0] = data[:, -1] = 100
    return cg.GridSpec(w, h, res, ox, oy, yaw), data


@pytest.mark.parametrize('yaw', [0.0, math.pi / 2, math.radians(30)])
def test_round_trip_with_rotated_origin(yaw):
    spec = cg.GridSpec(40, 30, 0.1, 1.5, -2.0, yaw)
    for col, row in [(0, 0), (39, 29), (10, 5), (3, 25)]:
        x, y = cg.grid_to_world(spec, col, row)
        assert cg.world_to_grid(spec, x, y) == (col, row)


def test_rotated_origin_known_values():
    spec = cg.GridSpec(40, 30, 0.1, 1.0, 2.0, math.pi / 2)
    x, y = cg.grid_to_world(spec, 10, 0)
    # local (1.05, 0.05) rotated by 90 degrees about the origin (1, 2)
    assert x == pytest.approx(1.0 - 0.05)
    assert y == pytest.approx(2.0 + 1.05)
    assert cg.world_to_grid(spec, x, y) == (10, 0)


def test_out_of_bounds_is_none():
    spec = cg.GridSpec(10, 10, 0.1, 0.0, 0.0, 0.0)
    assert cg.world_to_grid(spec, -0.01, 0.5) is None
    assert cg.world_to_grid(spec, 0.5, 1.01) is None


def test_bad_map_data_raises():
    spec = cg.GridSpec(10, 10, 0.1, 0.0, 0.0, 0.0)
    with pytest.raises(cg.MapError):
        cg.grid_array(spec, [0] * 99)
    with pytest.raises(cg.MapError):
        cg.grid_array(cg.GridSpec(10, 10, 0.0, 0.0, 0.0), [0] * 100)


def test_unknown_is_not_free_and_border_is_not_safe():
    spec, data = room()
    data[30:35, 30:35] = -1
    free = cg.free_mask(data)
    assert not free[32, 32]
    assert not free[0, 0]
    safe = cg.safe_mask(free, spec.resolution, 0.35)
    assert not safe[1, 1]          # touches the wall
    assert not safe[32, 32]
    assert safe[10, 10]


def test_safe_candidates_are_free_clear_and_connected_rotated_map():
    spec, data = room(yaw=math.radians(30), ox=1.0, oy=-1.0)
    data[:, 30] = 100              # wall splitting the room, no door
    data[10:14, 10:14] = -1        # unknown patch
    flat = data.flatten()
    robot = cg.grid_to_world(spec, 10, 40)
    cands, info = cg.generate_candidates(spec, flat, robot, max_locations=40, min_locations=1)
    assert cands and info['clearance_m'] >= 0.23
    free = cg.free_mask(data)
    dist = ndimage.distance_transform_edt(np.pad(free, 1), sampling=spec.resolution)[1:-1, 1:-1]
    for c in cands:
        cell = cg.world_to_grid(spec, c.x, c.y)
        assert cell is not None
        col, row = cell
        assert col < 30                                    # robot's side of the wall only
        assert dist[row, col] >= info['clearance_m'] - 1e-6
        assert -math.pi < c.yaw <= math.pi
    assert len({c.location_id for c in cands}) * 4 == len(cands)


def test_generation_is_deterministic_and_honours_max_locations():
    spec, data = room()
    robot = (0.0, 0.0)
    a, _ = cg.generate_candidates(spec, data.flatten(), robot, max_locations=5, min_locations=1)
    b, _ = cg.generate_candidates(spec, data.flatten(), robot, max_locations=5, min_locations=1)
    assert a == b
    assert len({c.location_id for c in a}) <= 5


def test_snap_and_no_safe_start():
    spec, data = room()
    free = cg.free_mask(data)
    comp = cg.component_containing(free, 30, 30)
    safe = cg.safe_mask(free, spec.resolution, 0.35)
    assert cg.snap_to_safe(comp, safe, 1, 1, spec.resolution, 1.0) != (1, 1)
    with pytest.raises(cg.NoSafeStartError):
        cg.snap_to_safe(comp, np.zeros_like(safe), 1, 1, spec.resolution, 1.0)


def test_components_use_four_connectivity():
    mask = np.zeros((6, 6), dtype=bool)
    mask[1, 1] = mask[2, 2] = True            # diagonal neighbours only
    comp = cg.component_containing(mask, 1, 1)
    assert comp[1, 1] and not comp[2, 2]
    assert not cg.component_containing(mask, 0, 0).any()


def test_narrow_corridor_forces_smaller_clearance():
    w, h = 140, 100
    data = np.full((h, w), 100, dtype=np.int16)
    data[20:86, 1:30] = 0                      # left room
    data[20:86, 111:139] = 0                   # right room
    data[50:57, 30:111] = 0                    # 7-cell (0.56 m) corridor
    spec = cg.GridSpec(w, h, 0.08, 0.0, 0.0, 0.0)
    robot = cg.grid_to_world(spec, 70, 53)
    cands, info = cg.generate_candidates(spec, data.flatten(), robot, min_locations=1)
    assert info['clearance_m'] < 0.35
    cols = {cg.world_to_grid(spec, c.x, c.y)[0] for c in cands}
    assert any(40 < c < 100 for c in cols)     # corridor cells are covered


def test_start_outside_map_and_no_candidates():
    spec, data = room()
    with pytest.raises(cg.NoSafeStartError):
        cg.generate_candidates(spec, data.flatten(), (50.0, 50.0))
    full = np.full_like(data, 100)
    with pytest.raises(cg.NoSafeStartError):
        cg.generate_candidates(spec, full.flatten(), (0.0, 0.0))


def test_large_map_is_fast_enough():
    spec, data = room(140, 140)
    start = time.time()
    cg.generate_candidates(spec, data.flatten(), (0.0, 0.0))
    assert time.time() - start < 5.0
