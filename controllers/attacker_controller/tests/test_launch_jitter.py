"""Tests for next_launch_point — pure jitter/drift helper for the attacker.

Spec:
  - returned point x,y are within a few jitter_std of the nominal;
  - returned point z equals nominal[2] (z is kept fixed);
  - returned new_nominal equals nominal + drift (element-wise);
  - the function is pure (does not mutate the input nominal list);
  - seeded rng makes results deterministic.
"""

import numpy as np
import pytest
from launch_jitter import next_launch_point


NOMINAL = [0.0, -10.0, 0.5]
JITTER_STD = 0.3
DRIFT = [0.02, 0.0, 0.0]


def _rng(seed=42):
    return np.random.default_rng(seed)


# ---------------------------------------------------------------------------
# Basic structure of the return value
# ---------------------------------------------------------------------------

def test_returns_point_and_new_nominal():
    point, new_nominal = next_launch_point(NOMINAL, JITTER_STD, DRIFT, _rng())
    assert len(point) == 3
    assert len(new_nominal) == 3


def test_z_is_unchanged():
    """z-coordinate is kept fixed (only x,y are jittered)."""
    point, _ = next_launch_point(NOMINAL, JITTER_STD, DRIFT, _rng())
    assert point[2] == NOMINAL[2]


# ---------------------------------------------------------------------------
# Jitter magnitude
# ---------------------------------------------------------------------------

def test_point_within_a_few_jitter_std_of_nominal():
    """With a seeded rng, x and y are within 3σ of the nominal."""
    rng = _rng(0)
    for _ in range(100):
        point, _ = next_launch_point(NOMINAL, JITTER_STD, DRIFT, rng)
        assert abs(point[0] - NOMINAL[0]) < 4 * JITTER_STD, f"x out of range: {point[0]}"
        assert abs(point[1] - NOMINAL[1]) < 4 * JITTER_STD, f"y out of range: {point[1]}"


def test_seeded_rng_gives_deterministic_result():
    """Same seed → identical point."""
    p1, _ = next_launch_point(NOMINAL, JITTER_STD, DRIFT, _rng(7))
    p2, _ = next_launch_point(NOMINAL, JITTER_STD, DRIFT, _rng(7))
    assert p1 == p2


# ---------------------------------------------------------------------------
# Drift behaviour
# ---------------------------------------------------------------------------

def test_new_nominal_equals_nominal_plus_drift():
    """Returned new_nominal is element-wise nominal + drift."""
    point, new_nominal = next_launch_point(NOMINAL, JITTER_STD, DRIFT, _rng())
    expected = [NOMINAL[i] + DRIFT[i] for i in range(3)]
    assert new_nominal == pytest.approx(expected)


def test_drift_zero_leaves_nominal_unchanged():
    """Zero drift → new_nominal equals original nominal."""
    _, new_nominal = next_launch_point(NOMINAL, JITTER_STD, [0.0, 0.0, 0.0], _rng())
    assert new_nominal == pytest.approx(NOMINAL)


def test_drift_accumulates_over_multiple_calls():
    """Chaining calls: nominal advances by drift each step."""
    rng = _rng(3)
    nominal = list(NOMINAL)
    drift = [0.05, 0.01, 0.0]
    for step in range(5):
        _, nominal = next_launch_point(nominal, JITTER_STD, drift, rng)
    expected_x = NOMINAL[0] + 5 * drift[0]
    expected_y = NOMINAL[1] + 5 * drift[1]
    assert nominal[0] == pytest.approx(expected_x)
    assert nominal[1] == pytest.approx(expected_y)


# ---------------------------------------------------------------------------
# Purity: input nominal is not mutated
# ---------------------------------------------------------------------------

def test_input_nominal_not_mutated():
    """next_launch_point must not mutate the nominal list passed to it."""
    nominal = [1.0, -5.0, 0.5]
    nominal_copy = list(nominal)
    next_launch_point(nominal, JITTER_STD, DRIFT, _rng())
    assert nominal == nominal_copy
