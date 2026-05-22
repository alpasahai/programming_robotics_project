"""Tests for Scoreboard — operational shot-down vs ground-hit tally."""
from scoreboard import Scoreboard


def test_starts_empty():
    s = Scoreboard()
    assert s.shot_down == 0 and s.ground_hits == 0 and s.total == 0
    assert s.shoot_down_rate is None          # undefined with no engagements


def test_counts_shootdowns_and_ground_hits():
    s = Scoreboard()
    s.record_shoot_down()
    s.record_shoot_down()
    s.record_ground_hit()
    assert s.shot_down == 2
    assert s.ground_hits == 1
    assert s.total == 3
    assert abs(s.shoot_down_rate - 2/3) < 1e-9


def test_summary_contains_counts_and_rate():
    s = Scoreboard()
    s.record_shoot_down()
    s.record_ground_hit()
    out = s.summary()
    assert "1" in out and "%" in out
    assert "shot down" in out.lower()


def test_summary_handles_no_engagements():
    assert isinstance(Scoreboard().summary(), str)  # no crash, no division by zero
