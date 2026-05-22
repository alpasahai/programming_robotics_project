"""Tests for LaunchObservationGate — arm/disarm latch for launch-origin observations.

Spec:
  - Starts DISARMED: no observation is produced before the first resolution.
  - Resolution step (resolved=True): arms the gate but does NOT observe the
    stale cue present on that step (stale-cue guard).
  - First cue on a later non-resolution step: yields exactly ONE observation
    and disarms.
  - Mid-engagement re-acquisition does NOT produce a second observation until
    the next resolution (debounce holds).
  - None cue is handled gracefully.
"""

import pytest
from launch_observation_gate import LaunchObservationGate


# ---------------------------------------------------------------------------
# Starts DISARMED — no observation before a resolution arms it
# ---------------------------------------------------------------------------

def test_starts_disarmed_no_cue_returns_none():
    gate = LaunchObservationGate()
    result = gate.observation(resolved=False, cue=[1.0, 2.0, 3.0])
    assert result is None


def test_starts_disarmed_no_cue_when_cue_is_none():
    gate = LaunchObservationGate()
    result = gate.observation(resolved=False, cue=None)
    assert result is None


# ---------------------------------------------------------------------------
# Resolution step arms but does NOT observe (stale-cue guard)
# ---------------------------------------------------------------------------

def test_resolution_step_arms_but_returns_none():
    """On the resolution step the gate arms — but the stale cue is NOT returned."""
    gate = LaunchObservationGate()
    result = gate.observation(resolved=True, cue=[9.0, 8.0, 7.0])
    assert result is None


def test_resolution_step_with_none_cue_arms_but_returns_none():
    gate = LaunchObservationGate()
    result = gate.observation(resolved=True, cue=None)
    assert result is None


# ---------------------------------------------------------------------------
# First fresh cue after arming yields ONE observation and disarms
# ---------------------------------------------------------------------------

def test_first_cue_after_resolution_yields_observation():
    """After resolution arms the gate, the first non-None cue is returned."""
    gate = LaunchObservationGate()
    gate.observation(resolved=True, cue=None)       # arm (resolution step)
    cue = [1.0, -10.0, 0.5]
    result = gate.observation(resolved=False, cue=cue)
    assert result == cue


def test_first_cue_after_resolution_disarms_gate():
    """After yielding the first observation, the gate disarms (debounce)."""
    gate = LaunchObservationGate()
    gate.observation(resolved=True, cue=None)       # arm
    gate.observation(resolved=False, cue=[1.0, -10.0, 0.5])  # observe + disarm
    # Second cue on the same engagement must NOT yield another observation.
    result = gate.observation(resolved=False, cue=[1.1, -10.1, 0.5])
    assert result is None


# ---------------------------------------------------------------------------
# Mid-engagement re-acquisition: debounce holds until the next resolution
# ---------------------------------------------------------------------------

def test_mid_engagement_none_cue_after_first_observation_returns_none():
    """After disarming, None cue on a later step returns None (not armed)."""
    gate = LaunchObservationGate()
    gate.observation(resolved=True, cue=None)
    gate.observation(resolved=False, cue=[1.0, -10.0, 0.5])  # disarms
    result = gate.observation(resolved=False, cue=None)
    assert result is None


def test_second_resolution_re_arms_for_next_engagement():
    """A second resolution re-arms so the gate responds to the next engagement."""
    gate = LaunchObservationGate()
    # First engagement.
    gate.observation(resolved=True, cue=None)
    gate.observation(resolved=False, cue=[1.0, -10.0, 0.5])  # consume + disarm
    gate.observation(resolved=False, cue=[1.1, -10.1, 0.5])  # still disarmed
    # Second resolution.
    gate.observation(resolved=True, cue=None)
    # Fresh cue for second engagement.
    cue2 = [2.0, -9.5, 0.5]
    result = gate.observation(resolved=False, cue=cue2)
    assert result == cue2


def test_second_engagement_also_disarms_after_one_observation():
    """The debounce resets properly: second engagement also yields only one obs."""
    gate = LaunchObservationGate()
    gate.observation(resolved=True, cue=None)
    gate.observation(resolved=False, cue=[1.0, -10.0, 0.5])
    gate.observation(resolved=True, cue=None)          # re-arm
    gate.observation(resolved=False, cue=[2.0, -9.5, 0.5])   # observe + disarm
    # Third cue — still same engagement, must be suppressed.
    result = gate.observation(resolved=False, cue=[2.1, -9.4, 0.5])
    assert result is None


# ---------------------------------------------------------------------------
# Cold-start: gate never armed, multiple cues → all suppressed
# ---------------------------------------------------------------------------

def test_cold_start_multiple_cues_all_suppressed():
    """Without any prior resolution, no cue is ever observed (cold-start guard)."""
    gate = LaunchObservationGate()
    for i in range(5):
        result = gate.observation(resolved=False, cue=[float(i), -10.0, 0.5])
        assert result is None, f"step {i}: expected None, got {result}"
