"""Tests for SectorMap — online angular clustering that discovers launch sectors."""
import math
from sector_map import SectorMap


def test_first_bearing_opens_cluster_zero():
    sm = SectorMap(tol_rad=0.1)
    assert sm.observe(1.0) == 0
    assert len(sm) == 1


def test_near_bearing_joins_same_cluster():
    sm = SectorMap(tol_rad=0.1)
    a = sm.observe(1.00)
    b = sm.observe(1.05)  # within tol
    assert a == b
    assert len(sm) == 1


def test_far_bearing_opens_new_cluster():
    sm = SectorMap(tol_rad=0.1)
    sm.observe(1.0)
    assert sm.observe(2.0) == 1  # beyond tol → new id
    assert len(sm) == 2


def test_count_emerges_from_data_no_k():
    """Three well-separated bearings → exactly three discovered sectors."""
    sm = SectorMap(tol_rad=0.2)
    seq = [0.0, 0.02, 2.0, 1.99, -2.0, -2.01, 0.01]
    ids = [sm.observe(b) for b in seq]
    assert len(sm) == 3
    assert ids[0] == ids[1] == ids[6]   # all near 0.0
    assert ids[2] == ids[3]             # near 2.0
    assert ids[4] == ids[5]             # near -2.0


def test_wraparound_pi_is_one_cluster():
    """Bearings near +π and −π are the same direction → one cluster."""
    sm = SectorMap(tol_rad=0.2)
    a = sm.observe(math.pi - 0.05)
    b = sm.observe(-math.pi + 0.05)
    assert a == b
    assert len(sm) == 1


def test_bearing_returns_cluster_centre():
    sm = SectorMap(tol_rad=0.2)
    sm.observe(1.0)
    sm.observe(1.1)
    # Verify the nudge MOVES the centre toward the second observation
    assert 1.0 < sm.bearing(0) < 1.1
