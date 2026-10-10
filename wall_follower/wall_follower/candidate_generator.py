"""Occupancy-grid maths and safe observation-pose generation (ROS-free)."""

import math
from dataclasses import dataclass
from typing import List, Optional, Tuple

import numpy as np
from scipy import ndimage

from wall_follower.goal_selection import Candidate


class MapError(ValueError):
    """The occupancy grid is malformed."""


class NoSafeStartError(RuntimeError):
    """No safe cell connected to the robot could be found."""


class NoCandidatesError(RuntimeError):
    """No observation location could be generated."""


@dataclass(frozen=True)
class GridSpec:
    """Geometry of a ROS ``OccupancyGrid`` (origin is the pose of cell (0, 0))."""

    width: int
    height: int
    resolution: float
    origin_x: float
    origin_y: float
    origin_yaw: float = 0.0


def grid_array(spec: GridSpec, flat) -> np.ndarray:
    """Validate geometry and reshape the flat row-major data to ``[row, col]``."""
    if spec.width <= 0 or spec.height <= 0 or spec.resolution <= 0.0:
        raise MapError('map width, height and resolution must be positive')
    arr = np.asarray(flat, dtype=np.int16)
    if arr.size != spec.width * spec.height:
        raise MapError(f'map data has {arr.size} cells, expected {spec.width * spec.height}')
    return arr.reshape(spec.height, spec.width)


def grid_to_world(spec: GridSpec, col: int, row: int) -> Tuple[float, float]:
    """Return the map-frame centre of cell (col, row), honouring the origin rotation."""
    lx = (col + 0.5) * spec.resolution
    ly = (row + 0.5) * spec.resolution
    c, s = math.cos(spec.origin_yaw), math.sin(spec.origin_yaw)
    return (spec.origin_x + c * lx - s * ly, spec.origin_y + s * lx + c * ly)


def world_to_grid(spec: GridSpec, x: float, y: float) -> Optional[Tuple[int, int]]:
    """Return (col, row) containing the map-frame point, or None if outside the grid."""
    dx, dy = x - spec.origin_x, y - spec.origin_y
    c, s = math.cos(spec.origin_yaw), math.sin(spec.origin_yaw)
    lx = c * dx + s * dy
    ly = -s * dx + c * dy
    col = int(math.floor(lx / spec.resolution))
    row = int(math.floor(ly / spec.resolution))
    if 0 <= col < spec.width and 0 <= row < spec.height:
        return (col, row)
    return None


def free_mask(data: np.ndarray, free_thresh: int = 25) -> np.ndarray:
    """Cells that are known and free; unknown (-1) is never free."""
    return (data >= 0) & (data < free_thresh)


def safe_mask(free: np.ndarray, resolution: float, clearance_m: float) -> np.ndarray:
    """Free cells at least ``clearance_m`` from any non-free cell (border counts as wall)."""
    padded = np.pad(free, 1, mode='constant', constant_values=False)
    dist = ndimage.distance_transform_edt(padded, sampling=resolution)
    return (dist[1:-1, 1:-1] >= clearance_m) & free


def component_containing(mask: np.ndarray, col: int, row: int) -> np.ndarray:
    """4-connected component of ``mask`` containing (col, row), else an empty mask."""
    if not (0 <= row < mask.shape[0] and 0 <= col < mask.shape[1]) or not mask[row, col]:
        return np.zeros_like(mask, dtype=bool)
    labels, _ = ndimage.label(mask)
    return labels == labels[row, col]


def snap_to_safe(free_comp: np.ndarray, safe: np.ndarray, col: int, row: int,
                 resolution: float, snap_radius_m: float) -> Tuple[int, int]:
    """Nearest safe cell inside the robot's free component, within the snap radius."""
    allowed = safe & free_comp
    if 0 <= row < allowed.shape[0] and 0 <= col < allowed.shape[1] and allowed[row, col]:
        return (col, row)
    rows, cols = np.nonzero(allowed)
    if rows.size == 0:
        raise NoSafeStartError('no safe cell in the robot component')
    d = np.hypot(cols - col, rows - row) * resolution
    i = int(np.argmin(d))
    if d[i] > snap_radius_m:
        raise NoSafeStartError(f'nearest safe cell is {d[i]:.2f} m away (> {snap_radius_m} m)')
    return (int(cols[i]), int(rows[i]))


def _coverage(free_comp, safe_comp, resolution, reach_m=1.0) -> int:
    """Count free-component cells within ``reach_m`` of the safe component."""
    if not safe_comp.any():
        return 0
    dist = ndimage.distance_transform_edt(~safe_comp, sampling=resolution)
    return int(np.count_nonzero(free_comp & (dist <= reach_m)))


def clearance_levels(max_clearance: float, min_clearance: float) -> List[float]:
    """Descending clearances in 0.05 m steps from max down to min (inclusive)."""
    levels, c = [], max_clearance
    while c >= min_clearance - 1e-9:
        levels.append(round(c, 4))
        c -= 0.05
    if not levels or levels[-1] > min_clearance + 1e-9:
        levels.append(round(min_clearance, 4))
    return levels


def choose_clearance(free, free_comp, col, row, resolution, max_clearance, min_clearance,
                     coverage_min=0.95, snap_radius_m=1.0):
    """
    Pick the largest clearance that keeps ``coverage_min`` of the minimum-clearance reach.

    Returns ``(clearance, safe_comp)``.  Raises ``NoSafeStartError`` when even the
    minimum clearance has no safe cell connected to the robot.
    """
    results = []
    for c in clearance_levels(max_clearance, min_clearance):
        safe = safe_mask(free, resolution, c)
        try:
            sc, sr = snap_to_safe(free_comp, safe, col, row, resolution, snap_radius_m)
        except NoSafeStartError:
            results.append((c, None, 0))
            continue
        comp = component_containing(safe, sc, sr)
        results.append((c, comp, _coverage(free_comp, comp, resolution)))
    base = results[-1][2]
    if base == 0:
        raise NoSafeStartError('no safe start even at the minimum clearance')
    for c, comp, cov in results:
        if comp is not None and cov >= coverage_min * base:
            return c, comp
    return results[-1][0], results[-1][1]


def sample_viewpoints(mask: np.ndarray, resolution: float, spacing_m: float,
                      max_locations: int, start_cell: Tuple[int, int]) -> List[Tuple[int, int]]:
    """Deterministic lattice sampling of ``mask`` cells, thinned by farthest-point sampling."""
    step = max(1, int(round(spacing_m / resolution)))
    half = max(1, step // 2)
    chosen: List[Tuple[int, int]] = []
    h, w = mask.shape
    for r0 in range(step // 2, h, step):
        for c0 in range(step // 2, w, step):
            best, best_d = None, None
            for r in range(max(0, r0 - half), min(h, r0 + half + 1)):
                for c in range(max(0, c0 - half), min(w, c0 + half + 1)):
                    if mask[r, c]:
                        d = (r - r0) ** 2 + (c - c0) ** 2
                        if best_d is None or d < best_d:
                            best, best_d = (c, r), d
            if best is not None and best not in chosen:
                chosen.append(best)
    min_sep = 0.5 * spacing_m / resolution
    kept: List[Tuple[int, int]] = []
    for cell in chosen:
        if all(math.hypot(cell[0] - k[0], cell[1] - k[1]) >= min_sep for k in kept):
            kept.append(cell)
    if len(kept) > max_locations:
        picked = [min(kept, key=lambda k: math.hypot(k[0] - start_cell[0], k[1] - start_cell[1]))]
        while len(picked) < max_locations:
            nxt = max((k for k in kept if k not in picked),
                      key=lambda k: min(math.hypot(k[0] - p[0], k[1] - p[1]) for p in picked))
            picked.append(nxt)
        kept = sorted(picked, key=lambda k: (k[1], k[0]))
    return kept


def generate_candidates(spec: GridSpec, flat_data, robot_xy: Tuple[float, float], *,
                        robot_radius_m: float = 0.18, safety_margin_m: float = 0.17,
                        spacing_m: float = 0.9, max_locations: int = 40,
                        min_locations: int = 6, yaw_count: int = 4,
                        coverage_min: float = 0.95, snap_radius_m: float = 1.0,
                        free_thresh: int = 25):
    """
    Build safe, robot-connected observation poses.

    Returns ``(candidates, info)``.  Raises ``MapError``, ``NoSafeStartError`` or
    ``NoCandidatesError``.  Poses sit at least ``clearance`` from any wall or unknown
    cell, in the same free component as the robot, with ``yaw_count`` headings each.
    """
    data = grid_array(spec, flat_data)
    free = free_mask(data, free_thresh)
    cell = world_to_grid(spec, robot_xy[0], robot_xy[1])
    if cell is None:
        raise NoSafeStartError('robot is outside the map')
    free_comp = component_containing(free, cell[0], cell[1])
    if not free_comp.any():
        # Robot sits on a non-free cell: use the nearest free cell as the component seed.
        rows, cols = np.nonzero(free)
        if rows.size == 0:
            raise NoSafeStartError('map has no free cells')
        i = int(np.argmin(np.hypot(cols - cell[0], rows - cell[1])))
        free_comp = component_containing(free, int(cols[i]), int(rows[i]))
    max_c = robot_radius_m + safety_margin_m
    min_c = robot_radius_m + 0.05
    clearance, comp = choose_clearance(
        free, free_comp, cell[0], cell[1], spec.resolution, max_c, min_c,
        coverage_min, snap_radius_m)
    cells = sample_viewpoints(comp, spec.resolution, spacing_m, max_locations, cell)
    # Relax clearance further if too few locations were found.
    while len(cells) < min_locations and clearance - 0.05 >= min_c - 1e-9:
        clearance = round(clearance - 0.05, 4)
        safe = safe_mask(free, spec.resolution, clearance)
        sc, sr = snap_to_safe(free_comp, safe, cell[0], cell[1], spec.resolution, snap_radius_m)
        comp = component_containing(safe, sc, sr)
        cells = sample_viewpoints(comp, spec.resolution, spacing_m, max_locations, cell)
    if not cells:
        raise NoCandidatesError('no observation locations found')
    cands: List[Candidate] = []
    for loc_id, (c, r) in enumerate(cells):
        x, y = grid_to_world(spec, c, r)
        for k in range(max(1, yaw_count)):
            yaw = math.atan2(math.sin(2 * math.pi * k / max(1, yaw_count)),
                             math.cos(2 * math.pi * k / max(1, yaw_count)))
            cands.append(Candidate(cid=len(cands), location_id=loc_id, x=x, y=y, yaw=yaw))
    info = {'clearance_m': clearance, 'locations': len(cells), 'poses': len(cands),
            'min_locations_met': len(cells) >= min_locations}
    return cands, info
