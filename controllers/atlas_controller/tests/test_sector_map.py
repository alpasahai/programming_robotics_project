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


def test_capacity_folds_overflow_into_nearest_cluster():
    """With a cap, a bearing beyond capacity joins the nearest existing sector."""
    sm = SectorMap(tol_rad=0.2, max_clusters=2)
    a = sm.observe(0.0)
    b = sm.observe(2.0)        # opens the 2nd (and last) cluster
    c = sm.observe(-2.0)       # no room for a 3rd → folds into nearest existing
    assert len(sm) == 2
    assert {a, b} == {0, 1}
    assert c in (0, 1)         # reused an existing id, no new cluster


def test_capacity_never_returns_id_at_or_above_cap():
    """Every returned id stays in [0, max_clusters) no matter the bearings."""
    sm = SectorMap(tol_rad=0.05, max_clusters=3)
    ids = [sm.observe(b) for b in [0.0, 1.0, 2.0, 3.0, -1.0, -2.0, 0.5, 2.5]]
    assert len(sm) == 3
    assert all(0 <= i < 3 for i in ids)


def test_uncapped_default_unchanged():
    """Default (no cap) still opens unbounded clusters (back-compat)."""
    sm = SectorMap(tol_rad=0.05)
    ids = [sm.observe(b) for b in [0.0, 1.0, 2.0, 3.0]]
    assert len(sm) == 4
    assert ids == [0, 1, 2, 3]
