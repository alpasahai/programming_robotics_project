"""Tests for PreAimMonitor — scores predicted-vs-actual launch sectors for telemetry."""
import os, sys
sys.path.insert(0, os.path.join(
    os.path.dirname(__file__), "..", "..", "atlas_controller"))

import math
from preaim_monitor import PreAimMonitor


def test_cold_start_predictions_not_scored():
    """A None prediction (cold start) is not counted as a scored prediction."""
    m = PreAimMonitor()
    m.record(predicted_sector=None, actual_sector=0,
             predicted_bearing=None, actual_bearing=0.0)
    assert m.n == 0
    assert m.accuracy is None


def test_accuracy_counts_hits():
    m = PreAimMonitor()
    m.record(0, 0, 0.0, 0.0)   # hit
    m.record(1, 2, 1.5, 3.0)   # miss
    m.record(2, 2, 3.0, 3.0)   # hit
    assert m.n == 3
    assert abs(m.accuracy - 2/3) < 1e-9


def test_predict_last_baseline():
    """Baseline = predict the previous actual sector; scored over the same set."""
    m = PreAimMonitor()
    m.record(0, 0, 0.0, 0.0)   # prev_actual None → baseline can't score, but n counts
    m.record(1, 0, 1.5, 0.0)   # baseline predicts prev_actual=0, actual=0 → baseline hit
    m.record(1, 1, 1.5, 1.5)   # baseline predicts prev_actual=0, actual=1 → baseline miss
    assert m.n == 3
    assert abs(m.baseline_accuracy - 1/3) < 1e-9   # 1 baseline hit / 3 scored


def test_angular_error_uses_wraparound_and_degrees():
    m = PreAimMonitor()
    # predicted bearing near +pi, actual near -pi → tiny true angular error
    m.record(0, 0, math.pi - 0.05, -math.pi + 0.05)
    assert m.last_error_deg < 6.0          # ~0.1 rad ≈ 5.7°, NOT ~360°
    assert m.mean_error_deg < 6.0


def test_error_only_accumulates_when_bearings_present():
    m = PreAimMonitor()
    m.record(0, 1, None, 1.5)   # no predicted bearing → no error sample
    assert m.last_error_deg is None
    assert m.mean_error_deg is None


def test_summary_is_a_string_with_accuracy_and_baseline():
    m = PreAimMonitor()
    m.record(0, 0, 0.0, 0.0)
    m.record(1, 1, 1.5, 1.5)
    s = m.summary()
    assert isinstance(s, str)
    assert "acc" in s.lower()
    assert "%" in s
