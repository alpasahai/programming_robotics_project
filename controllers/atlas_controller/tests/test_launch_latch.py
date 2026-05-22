"""Tests for LaunchLatch — segments the radar cue stream into one launch
observation per engagement (first fresh cue after the prior cue clears)."""
from launch_latch import LaunchLatch


def test_no_observation_before_first_resolution():
    latch = LaunchLatch()
    assert latch.update(resolved=False, cue=[1.0, 2.0, 0.5]) is None


def test_resolution_step_only_arms_never_observes():
    latch = LaunchLatch()
    assert latch.update(resolved=True, cue=[9.0, 9.0, 0.5]) is None


def test_does_not_observe_stale_cue_before_clear():
    """After arming, a non-None cue that has NOT yet cleared is the stale cue → ignored."""
    latch = LaunchLatch()
    latch.update(resolved=True, cue=[9.0, 9.0, 0.5])     # arm
    # cue still non-None (RESET hasn't cleared it yet) → must NOT observe
    assert latch.update(resolved=False, cue=[9.0, 9.0, 0.5]) is None
    assert latch.update(resolved=False, cue=[9.1, 9.0, 0.5]) is None


def test_observes_first_fresh_cue_after_clear():
    """Once the cue has gone None (RESET cleared it), the next non-None cue is observed."""
    latch = LaunchLatch()
    latch.update(resolved=True, cue=[9.0, 9.0, 0.5])     # arm
    assert latch.update(resolved=False, cue=[9.0, 9.0, 0.5]) is None  # stale, pre-clear
    assert latch.update(resolved=False, cue=None) is None            # RESET cleared it
    assert latch.update(resolved=False, cue=None) is None            # respawn gap, still none
    fresh = [3.0, 7.0, 0.5]
    assert latch.update(resolved=False, cue=fresh) == fresh          # new projectile → observe


def test_one_observation_per_engagement():
    """After observing, it disarms until the next resolution re-arms it."""
    latch = LaunchLatch()
    latch.update(resolved=True, cue=[9, 9, 0.5])
    latch.update(resolved=False, cue=None)
    assert latch.update(resolved=False, cue=[3, 7, 0.5]) == [3, 7, 0.5]  # observed
    # subsequent cues without a new resolution must NOT observe again
    assert latch.update(resolved=False, cue=[3, 7, 0.5]) is None
    assert latch.update(resolved=False, cue=[4, 7, 0.5]) is None


def test_rearms_on_next_resolution():
    latch = LaunchLatch()
    latch.update(resolved=True, cue=[9, 9, 0.5])
    latch.update(resolved=False, cue=None)
    latch.update(resolved=False, cue=[3, 7, 0.5])        # observed, disarmed
    latch.update(resolved=True, cue=[8, 8, 0.5])         # re-arm
    latch.update(resolved=False, cue=None)               # cleared
    assert latch.update(resolved=False, cue=[2, 6, 0.5]) == [2, 6, 0.5]  # next engagement observed
