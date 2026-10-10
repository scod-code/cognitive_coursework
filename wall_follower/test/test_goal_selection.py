"""Tests for the selector interface and baseline selector (ROS-free)."""

import pytest

from wall_follower.goal_selection import (
    Candidate, NearestByPathSelector, SelectionContext, check_source_name, make_selector)


def cands():
    return [
        Candidate(0, 0, 1.0, 0.0, 0.0), Candidate(1, 0, 1.0, 0.0, 1.57),
        Candidate(2, 1, 2.0, 0.0, 0.0), Candidate(3, 2, 3.0, 0.0, 0.0),
        Candidate(4, 3, -1.0, 0.0, 0.0),
    ]


def ctx(**kw):
    return SelectionContext(robot_xy=(0.0, 0.0), **kw)


def test_shortlist_nearest_distinct_locations_ties_by_cid():
    sel = NearestByPathSelector()
    out = sel.shortlist(cands(), ctx(), 3)
    # locations 0 and 3 are both 1.0 m away: tie broken by lowest cid
    assert [c.cid for c in out] == [0, 4, 2]


def test_visited_suppressed_and_validated_are_excluded():
    sel = NearestByPathSelector()
    out = sel.shortlist(cands(), ctx(visited={0}, suppressed={4: 'x'}, validated={1}), 5)
    assert [c.cid for c in out] == [1, 3]       # cid 1 is the next heading at location 0


def test_choose_prefers_path_length_over_euclidean():
    sel = NearestByPathSelector()
    short = sel.shortlist(cands(), ctx(), 3)
    c = ctx(path_lengths={0: 9.0, 3: 12.0, 1: 2.5})
    assert sel.choose(short, c).cid == 2        # wall between robot and location 0


def test_choose_none_when_all_infinite_or_missing():
    sel = NearestByPathSelector()
    short = sel.shortlist(cands(), ctx(), 3)
    assert sel.choose(short, ctx(path_lengths={0: float('inf')})) is None


def test_choose_tie_breaks_by_cid():
    sel = NearestByPathSelector()
    short = [cands()[2], cands()[0]]
    assert sel.choose(short, ctx(path_lengths={0: 1.0, 1: 1.0})).cid == 0


def test_registry_rejects_unknown_names():
    assert isinstance(make_selector('nearest_path'), NearestByPathSelector)
    with pytest.raises(ValueError):
        make_selector('pso')
    assert check_source_name('lattice') == 'lattice'
    with pytest.raises(ValueError):
        check_source_name('frontier')


def test_interface_methods_exist():
    sel = make_selector('nearest_path')
    assert callable(sel.shortlist) and callable(sel.choose)
