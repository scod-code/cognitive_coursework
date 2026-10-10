"""
Candidate data types and the goal-selection extension point (ROS-free).

``CandidateSelector`` and ``CandidateSource`` are the only interfaces a later
optimiser or learner (PSO, GA, RL) has to implement.  Nothing of that kind is
implemented here: the baseline is a deterministic nearest-by-path heuristic.
"""

import math
from dataclasses import dataclass, field
from typing import Callable, Dict, List, Optional, Protocol, Set, Tuple


@dataclass(frozen=True)
class Candidate:
    """One observation pose (x, y, yaw) in the map frame."""

    cid: int
    location_id: int
    x: float
    y: float
    yaw: float
    source: str = 'lattice'


@dataclass
class SelectionContext:
    """Mission state a selector may read (it must not mutate it)."""

    robot_xy: Tuple[float, float]
    visited: Set[int] = field(default_factory=set)
    suppressed: Dict[int, str] = field(default_factory=dict)
    confirmed: Set[str] = field(default_factory=set)
    required: Set[str] = field(default_factory=set)
    path_lengths: Dict[int, float] = field(default_factory=dict)
    validated: Set[int] = field(default_factory=set)


class CandidateSelector(Protocol):
    """Two-step selector: cheap shortlist, then choice using planned path lengths."""

    def shortlist(self, candidates: List[Candidate], ctx: SelectionContext,
                  k: int) -> List[Candidate]:
        """Return up to ``k`` candidates (distinct locations) worth validating."""

    def choose(self, shortlisted: List[Candidate],
               ctx: SelectionContext) -> Optional[Candidate]:
        """Return the candidate to navigate to, or None if none is reachable."""


class CandidateSource(Protocol):
    """Produces the candidate set (lattice today, frontiers in a later phase)."""

    def generate(self, ctx: SelectionContext) -> List[Candidate]:
        """Return the candidates for the current context."""


class NearestByPathSelector:
    """Nearest unvisited location by planned path length (baseline heuristic)."""

    def shortlist(self, candidates, ctx, k):
        """Pick the ``k`` Euclidean-nearest eligible locations (lowest cid per location)."""
        best: Dict[int, Candidate] = {}
        for cand in candidates:
            if cand.cid in ctx.visited or cand.cid in ctx.suppressed:
                continue
            if cand.location_id in ctx.validated:
                continue
            cur = best.get(cand.location_id)
            if cur is None or cand.cid < cur.cid:
                best[cand.location_id] = cand
        ranked = sorted(
            best.values(),
            key=lambda c: (math.hypot(c.x - ctx.robot_xy[0], c.y - ctx.robot_xy[1]), c.cid))
        return ranked[:k]

    def choose(self, shortlisted, ctx):
        """Return the candidate with the smallest finite planned path length."""
        scored = []
        for cand in shortlisted:
            length = ctx.path_lengths.get(cand.location_id)
            if length is not None and math.isfinite(length):
                scored.append((length, cand.cid, cand))
        if not scored:
            return None
        scored.sort(key=lambda t: (t[0], t[1]))
        return scored[0][2]


class LatticeSource:
    """Wrap a generator function so it satisfies ``CandidateSource``."""

    def __init__(self, generate_fn: Callable[[Tuple[float, float]], List[Candidate]]):
        """Store ``generate_fn(robot_xy) -> candidates`` and cache its first result."""
        self._fn = generate_fn
        self._cache: Optional[List[Candidate]] = None

    def generate(self, ctx):
        """Return the (cached) lattice candidates."""
        if self._cache is None:
            self._cache = self._fn(ctx.robot_xy)
        return self._cache


SELECTORS = {'nearest_path': NearestByPathSelector}
SOURCES = {'lattice': LatticeSource}


def make_selector(name: str):
    """Build a registered selector or raise ``ValueError`` for an unknown name."""
    if name not in SELECTORS:
        raise ValueError(f"unknown selector '{name}', registered: {sorted(SELECTORS)}")
    return SELECTORS[name]()


def check_source_name(name: str) -> str:
    """Return ``name`` if it is a registered candidate source, else raise ``ValueError``."""
    if name not in SOURCES:
        raise ValueError(f"unknown candidate_source '{name}', registered: {sorted(SOURCES)}")
    return name
