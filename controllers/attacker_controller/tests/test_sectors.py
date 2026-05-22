"""Tests for sector launch geometry."""
import math
from sectors import sector_launch


def test_due_south_matches_legacy_corridor():
    """Azimuth π (due south, ENU pan = atan2(dx,dy)) reproduces the old corridor."""
    pos, vel = sector_launch(math.pi, ground_range=10.0, height=0.5,
                             h_speed=2.0, v_speed=10.0)
    assert pos[0] == 0.0 or abs(pos[0]) < 1e-9   # dx ≈ 0
    assert abs(pos[1] - (-10.0)) < 1e-9          # dy ≈ -10 (south)
    assert abs(pos[2] - 0.5) < 1e-9
    assert abs(vel[0]) < 1e-9                    # vx ≈ 0
    assert abs(vel[1] - 2.0) < 1e-9              # vy toward turret (+north)
    assert abs(vel[2] - 10.0) < 1e-9


def test_distinct_azimuths_give_distinct_positions():
    p1, _ = sector_launch(0.0, 10.0, 0.5, 2.0, 10.0)
    p2, _ = sector_launch(math.pi / 2, 10.0, 0.5, 2.0, 10.0)
    assert p1[:2] != p2[:2]


def test_velocity_points_back_at_origin():
    """Horizontal velocity is anti-parallel to the spawn's horizontal offset."""
    az = 1.0
    pos, vel = sector_launch(az, 10.0, 0.5, 2.0, 10.0)
    # pos horizontal points away from origin at bearing az; vel horizontal opposes it
    assert vel[0] * pos[0] <= 0 and vel[1] * pos[1] <= 0
