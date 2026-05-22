"""Tests for LaunchPointEstimator — recursive estimate of the launch origin."""
import math
import numpy as np
from launch_point_estimator import LaunchPointEstimator


def test_no_estimate_before_any_observation():
    est = LaunchPointEstimator(alpha=0.2)
    assert est.get_estimate() is None
    assert est.get_ready_aim([0.0, 0.0, 0.0]) is None
    assert est.samples == 0


def test_first_observation_sets_estimate():
    est = LaunchPointEstimator(alpha=0.2)
    est.observe([1.0, 2.0, 3.0])
    assert est.get_estimate() == [1.0, 2.0, 3.0]
    assert est.samples == 1


def test_converges_to_true_mean_under_noise():
    """Noisy samples around a true point → estimate close to truth (≪ noise)."""
    true = np.array([2.0, -9.0, 0.5])
    rng = np.random.default_rng(0)
    est = LaunchPointEstimator(alpha=0.15)
    for _ in range(300):
        est.observe((true + rng.normal(0, 0.3, size=3)).tolist())
    err = np.linalg.norm(np.array(est.get_estimate()) - true)
    assert err < 0.15  # well below the 0.3 per-sample noise


def test_adapts_after_drift():
    """After the true point shifts, the estimate moves toward the new point."""
    rng = np.random.default_rng(1)
    est = LaunchPointEstimator(alpha=0.2)
    for _ in range(200):
        est.observe((np.array([0.0, -10.0, 0.5]) + rng.normal(0, 0.2, 3)).tolist())
    before = np.array(est.get_estimate())
    for _ in range(200):
        est.observe((np.array([5.0, -6.0, 0.5]) + rng.normal(0, 0.2, 3)).tolist())
    after = np.array(est.get_estimate())
    assert np.linalg.norm(after - np.array([5.0, -6.0, 0.5])) < np.linalg.norm(before - np.array([5.0, -6.0, 0.5]))
    assert after[0] > before[0]  # moved toward the new point


def test_ready_aim_points_at_estimate_relative_to_turret():
    est = LaunchPointEstimator(alpha=0.2)
    est.observe([3.0, 4.0, 1.0])      # estimate = [3,4,1]
    turret = [0.0, 0.0, 0.0]
    pan, tilt = est.get_ready_aim(turret)
    assert abs(pan - math.atan2(3.0, 4.0)) < 1e-9
    assert abs(tilt - math.atan2(1.0, math.hypot(3.0, 4.0))) < 1e-9
