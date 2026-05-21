"""Unit tests for TrackTelemetry's prediction-accuracy bookkeeping.

Only record_intercept_and_error is exercised — the rolling-history logic that
was previously inline in the controller loop and untestable. The report()
formatting is left to the live Webots run.
"""

import logging

from telemetry import TrackTelemetry


def _make(lookahead_steps):
    return TrackTelemetry(
        logging.getLogger("test"),
        lookahead_steps=lookahead_steps,
        max_range=10.0,
        ground_threshold=0.1,
        turret_position=[0.0, 0.0, 0.0],
        timestep_ms=32,
    )


def test_warming_up_returns_none_until_history_fills():
    # maxlen = lookahead_steps + 1 = 3, so the first two calls warm up.
    tel = _make(lookahead_steps=2)

    assert tel.record_intercept_and_error([1, 0, 0], [9, 9, 9]) is None
    assert tel.record_intercept_and_error([2, 0, 0], [9, 9, 9]) is None
    # Third call: history full → compares the oldest intercept ([1,0,0]) to truth.
    assert tel.record_intercept_and_error([3, 0, 0], [9, 9, 9]) is not None


def test_error_compares_intercept_predicted_lookahead_steps_ago():
    tel = _make(lookahead_steps=2)  # maxlen 3

    tel.record_intercept_and_error([0, 0, 0], [0, 0, 0])  # warm
    tel.record_intercept_and_error([5, 5, 5], [0, 0, 0])  # warm
    # Now full: the prediction targeting THIS step is the oldest, [0,0,0].
    # true_rel is [3,0,0] → error = distance([3,0,0], [0,0,0]) = 3.0
    err = tel.record_intercept_and_error([9, 9, 9], [3, 0, 0])
    assert err == 3.0


def test_lookahead_zero_compares_current_step_immediately():
    # maxlen = 1: no warm-up; each intercept is graded against the same step.
    tel = _make(lookahead_steps=0)

    err = tel.record_intercept_and_error([0, 4, 0], [0, 0, 0])
    assert err == 4.0
