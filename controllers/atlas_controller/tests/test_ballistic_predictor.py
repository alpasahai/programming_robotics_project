"""Tests for BallisticTrajectoryPredictor — closed-form ballistic intercept.

Verifies:
  - Horizontal axes (x, y) extrapolate linearly with time.
  - Z axis is parabolic: z + vz*T - 0.5*g*T².
  - Lookahead time T = lookahead_steps * timestep_ms / 1000.0.
  - Doubling lookahead_steps doubles T, scaling results correctly.
  - Doubling timestep_ms doubles T, scaling results correctly.
  - The predictor is stateless: repeated calls return the same result.
"""
import pytest
from stubs import StubTrackFilter
from ballistic_trajectory_predictor import BallisticTrajectoryPredictor

GRAVITY = 9.81  # m/s²


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def make_predictor(position, velocity, timestep_ms=32):
    """Return a BallisticTrajectoryPredictor with injected stub filter."""
    stub = StubTrackFilter(position, velocity)
    return BallisticTrajectoryPredictor(stub, timestep_ms)


# ---------------------------------------------------------------------------
# Slice 1: horizontal linear extrapolation (x and y)
# ---------------------------------------------------------------------------

def test_x_extrapolates_linearly():
    """x_pred = x + vx * T; no gravity on horizontal axes."""
    pos = [1.0, 2.0, 3.0]
    vel = [4.0, 5.0, 6.0]
    timestep_ms = 100       # 0.1 s per step
    lookahead = 3           # T = 3 * 0.1 = 0.3 s

    predictor = make_predictor(pos, vel, timestep_ms)
    result = predictor.get_intercept(lookahead)

    T = lookahead * timestep_ms / 1000.0   # 0.3 s
    expected_x = pos[0] + vel[0] * T       # 1.0 + 4.0*0.3 = 2.2
    assert result[0] == pytest.approx(expected_x)


def test_y_extrapolates_linearly():
    """y_pred = y + vy * T; no gravity on horizontal axes."""
    pos = [1.0, 2.0, 3.0]
    vel = [4.0, 5.0, 6.0]
    timestep_ms = 100
    lookahead = 3

    predictor = make_predictor(pos, vel, timestep_ms)
    result = predictor.get_intercept(lookahead)

    T = lookahead * timestep_ms / 1000.0
    expected_y = pos[1] + vel[1] * T       # 2.0 + 5.0*0.3 = 3.5
    assert result[1] == pytest.approx(expected_y)


# ---------------------------------------------------------------------------
# Slice 2: Z axis is parabolic (gravity drop)
# ---------------------------------------------------------------------------

def test_z_applies_gravity_drop():
    """z_pred = z + vz*T - 0.5*g*T²; gravity acts downward on Z."""
    pos = [0.0, 0.0, 10.0]
    vel = [0.0, 0.0, 0.0]
    timestep_ms = 1000       # 1.0 s per step
    lookahead = 1            # T = 1.0 s

    predictor = make_predictor(pos, vel, timestep_ms)
    result = predictor.get_intercept(lookahead)

    T = 1.0
    expected_z = 10.0 + 0.0 * T - 0.5 * GRAVITY * T ** 2   # 10 - 4.905 = 5.095
    assert result[2] == pytest.approx(expected_z)


def test_z_with_nonzero_vz_and_gravity():
    """z_pred = z + vz*T - 0.5*g*T² with upward initial vz."""
    pos = [0.0, 0.0, 0.0]
    vel = [0.0, 0.0, 20.0]   # thrown upward at 20 m/s
    timestep_ms = 500        # 0.5 s per step
    lookahead = 4            # T = 2.0 s

    predictor = make_predictor(pos, vel, timestep_ms)
    result = predictor.get_intercept(lookahead)

    T = lookahead * timestep_ms / 1000.0   # 2.0 s
    expected_z = 0.0 + 20.0 * T - 0.5 * GRAVITY * T ** 2   # 40 - 19.62 = 20.38
    assert result[2] == pytest.approx(expected_z)


# ---------------------------------------------------------------------------
# Slice 3: T = lookahead_steps * timestep_ms / 1000.0
# ---------------------------------------------------------------------------

def test_zero_lookahead_returns_current_position():
    """With lookahead_steps=0, T=0, result equals current filter state."""
    pos = [3.0, 7.0, -2.0]
    vel = [10.0, -5.0, 8.0]

    predictor = make_predictor(pos, vel, timestep_ms=64)
    result = predictor.get_intercept(0)

    assert result == pytest.approx([3.0, 7.0, -2.0])


def test_doubling_lookahead_steps_doubles_T():
    """Doubling lookahead_steps must double T, scaling all displacement terms.

    For a horizontal axis with nonzero velocity, displacement = v*T, so
    doubling T should double the displacement.
    """
    pos = [0.0, 0.0, 0.0]
    vel = [2.0, 0.0, 0.0]    # only vx; gravity irrelevant for x
    timestep_ms = 100

    pred_1 = make_predictor(pos, vel, timestep_ms)
    pred_2 = make_predictor(pos, vel, timestep_ms)

    r1 = pred_1.get_intercept(5)    # T = 0.5 s → x = 1.0
    r2 = pred_2.get_intercept(10)   # T = 1.0 s → x = 2.0

    # x displacement must double
    assert r2[0] == pytest.approx(2.0 * r1[0])


def test_doubling_timestep_ms_doubles_T():
    """Doubling timestep_ms (same lookahead_steps) must double T."""
    pos = [0.0, 0.0, 0.0]
    vel = [3.0, 0.0, 0.0]    # only vx
    lookahead = 5

    pred_slow = make_predictor(pos, vel, timestep_ms=100)   # T = 0.5 s
    pred_fast = make_predictor(pos, vel, timestep_ms=200)   # T = 1.0 s

    r_slow = pred_slow.get_intercept(lookahead)
    r_fast = pred_fast.get_intercept(lookahead)

    assert r_fast[0] == pytest.approx(2.0 * r_slow[0])


# ---------------------------------------------------------------------------
# Slice 4: statelessness — repeated calls return the same result
# ---------------------------------------------------------------------------

def test_get_intercept_is_stateless():
    """Calling get_intercept() twice with the same args returns the same values.

    The predictor must not mutate the filter or accumulate any internal state.
    """
    pos = [1.0, 2.0, 5.0]
    vel = [1.0, -1.0, 3.0]
    timestep_ms = 64
    lookahead = 10

    predictor = make_predictor(pos, vel, timestep_ms)
    r1 = predictor.get_intercept(lookahead)
    r2 = predictor.get_intercept(lookahead)

    assert r1 == pytest.approx(r2)


# ---------------------------------------------------------------------------
# Slice 5: returns a list of exactly 3 floats
# ---------------------------------------------------------------------------

def test_get_intercept_returns_list_of_3_floats():
    """get_intercept() must return a list of exactly 3 elements."""
    predictor = make_predictor([0.0, 0.0, 0.0], [1.0, 2.0, 3.0])
    result = predictor.get_intercept(5)

    assert isinstance(result, list)
    assert len(result) == 3
