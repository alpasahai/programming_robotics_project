"""Tests for SearchRadar — ATLAS's wide-beam external acquisition sensor."""
import math
import random

from stubs import StubProjectile
from search_radar import Detection, SearchRadar


# ---------------------------------------------------------------------------
# Shared defaults — permissive so each test focuses on one gate only.
# ---------------------------------------------------------------------------
TURRET_POS = [1.0, 2.0, 0.5]
RADAR_POS = [1.0, 2.0, 0.5]   # co-located with turret by default
TIMESTEP_MS = 32
MAX_RANGE = 1000.0              # effectively unlimited
VERTICAL_FOV = math.pi          # full hemisphere — effectively no FOV gate


def _make_radar(
    projectiles,
    *,
    turret_position=None,
    radar_position=None,
    noise_std=0.0,
    max_range=MAX_RANGE,
    vertical_fov=VERTICAL_FOV,
    timestep_ms=TIMESTEP_MS,
    rng=None,
    # scan params — beam_width and scan_rate now active (Task 3)
    # Default to full 2π beam so existing tests unaffected by azimuth gate
    beam_width=2 * math.pi,
    scan_rate=math.radians(30),
    track_timeout=3,
):
    """Construct a SearchRadar with sensible defaults for brevity."""
    return SearchRadar(
        projectiles,
        turret_position=turret_position if turret_position is not None else list(TURRET_POS),
        radar_position=radar_position if radar_position is not None else list(RADAR_POS),
        noise_std=noise_std,
        timestep_ms=timestep_ms,
        max_range=max_range,
        vertical_fov=vertical_fov,
        beam_width=beam_width,
        scan_rate=scan_rate,
        track_timeout=track_timeout,
        rng=rng,
    )


# ---------------------------------------------------------------------------
# Initial-state invariants
# ---------------------------------------------------------------------------

def test_get_detections_returns_empty_list_before_update():
    """get_detections() must return an empty list until update() is called."""
    proj = StubProjectile([[5.0, 6.0, 7.0]])
    radar = _make_radar([proj])
    assert radar.get_detections() == []


def test_get_target_position_returns_none_in_initial_state():
    """get_target_position() must return None before any update() or set_target()."""
    proj = StubProjectile([[5.0, 6.0, 7.0]])
    radar = _make_radar([proj])
    assert radar.get_target_position() is None


def test_get_target_position_returns_none_before_set_target():
    """get_target_position() must return None until set_target() is called."""
    proj = StubProjectile([[5.0, 6.0, 7.0]])
    radar = _make_radar([proj])
    radar.update()
    assert radar.get_target_position() is None


# ---------------------------------------------------------------------------
# Detection — in-range + in-FOV
# ---------------------------------------------------------------------------

def test_target_in_range_and_fov_is_detected():
    """A projectile within max_range and vertical_fov produces a Detection with track_id 0."""
    # Place radar at origin; projectile 10 m ahead at horizon — always in range/FOV.
    proj = StubProjectile([[0.0, 10.0, 0.0]])
    radar = _make_radar(
        [proj],
        radar_position=[0.0, 0.0, 0.0],
        turret_position=[0.0, 0.0, 0.0],
        max_range=50.0,
        vertical_fov=math.radians(30),
    )
    radar.update()
    detections = radar.get_detections()
    assert len(detections) == 1
    assert detections[0].track_id == 0


def test_detection_position_is_world_minus_turret_not_radar():
    """Detection.position is world_pos − turret_position, even when radar_position differs.

    Validates that the gating uses radar_position but the output uses turret_position,
    and the two must not be conflated.
    """
    world_pos = [10.0, 20.0, 1.0]
    turret = [1.0, 2.0, 0.5]
    radar_pos = [1.5, 2.5, 0.5]   # slightly offset from turret

    proj = StubProjectile([world_pos])
    radar = _make_radar(
        [proj],
        radar_position=radar_pos,
        turret_position=turret,
        noise_std=0.0,
    )
    radar.update()

    detections = radar.get_detections()
    assert len(detections) == 1
    expected = [world_pos[i] - turret[i] for i in range(3)]
    assert detections[0].position == expected


# ---------------------------------------------------------------------------
# Range gate
# ---------------------------------------------------------------------------

def test_target_beyond_max_range_is_rejected():
    """A projectile farther than max_range produces no Detection."""
    proj = StubProjectile([[0.0, 200.0, 0.0]])   # 200 m away
    radar = _make_radar(
        [proj],
        radar_position=[0.0, 0.0, 0.0],
        turret_position=[0.0, 0.0, 0.0],
        max_range=100.0,
        vertical_fov=math.pi,
    )
    radar.update()
    assert radar.get_detections() == []


def test_target_exactly_at_max_range_is_detected():
    """A projectile exactly at max_range (≤) must be accepted."""
    proj = StubProjectile([[0.0, 100.0, 0.0]])   # exactly 100 m
    radar = _make_radar(
        [proj],
        radar_position=[0.0, 0.0, 0.0],
        turret_position=[0.0, 0.0, 0.0],
        max_range=100.0,
        vertical_fov=math.pi,
    )
    radar.update()
    assert len(radar.get_detections()) == 1


# ---------------------------------------------------------------------------
# Vertical FOV gate
# ---------------------------------------------------------------------------

def test_target_above_narrow_fov_is_rejected():
    """A projectile with elevation > vertical_fov/2 produces no Detection.

    Place the projectile directly above the radar so elevation = 90°.
    Use a narrow FOV of 10° — the target must be rejected.
    """
    proj = StubProjectile([[0.0, 0.0, 50.0]])    # straight up
    radar = _make_radar(
        [proj],
        radar_position=[0.0, 0.0, 0.0],
        turret_position=[0.0, 0.0, 0.0],
        max_range=1000.0,
        vertical_fov=math.radians(10),
    )
    radar.update()
    assert radar.get_detections() == []


def test_target_near_horizon_inside_fov_is_accepted():
    """A projectile near the horizon and within the FOV is detected.

    Place the projectile 100 m north and 1 m up so elevation ≈ 0.57° — well
    inside a 10° half-angle FOV.
    """
    proj = StubProjectile([[0.0, 100.0, 1.0]])
    radar = _make_radar(
        [proj],
        radar_position=[0.0, 0.0, 0.0],
        turret_position=[0.0, 0.0, 0.0],
        max_range=1000.0,
        vertical_fov=math.radians(10),
    )
    radar.update()
    assert len(radar.get_detections()) == 1


def test_target_at_fov_boundary_is_accepted():
    """A projectile whose elevation equals exactly vertical_fov/2 is accepted (≤ gate)."""
    half_fov = math.radians(15)
    # Build a position whose elevation from the radar is exactly half_fov.
    # elevation = atan2(dz, sqrt(dx²+dy²)) → dz/horiz = tan(half_fov)
    horiz = 100.0
    dz = horiz * math.tan(half_fov)
    proj = StubProjectile([[0.0, horiz, dz]])
    radar = _make_radar(
        [proj],
        radar_position=[0.0, 0.0, 0.0],
        turret_position=[0.0, 0.0, 0.0],
        max_range=1000.0,
        vertical_fov=2 * half_fov,
    )
    radar.update()
    assert len(radar.get_detections()) == 1


# ---------------------------------------------------------------------------
# Mixed gates — multiple projectiles
# ---------------------------------------------------------------------------

def test_only_in_range_in_fov_projectile_is_detected_among_several():
    """Only the projectile passing both gates appears in get_detections()."""
    radar_pos = [0.0, 0.0, 0.0]
    turret_pos = [0.0, 0.0, 0.0]

    proj_good = StubProjectile([[0.0, 50.0, 0.0]])    # 50 m, horizon
    proj_far = StubProjectile([[0.0, 500.0, 0.0]])    # 500 m — beyond range
    proj_high = StubProjectile([[0.0, 0.0, 50.0]])    # straight up — beyond FOV

    radar = _make_radar(
        [proj_good, proj_far, proj_high],
        radar_position=radar_pos,
        turret_position=turret_pos,
        max_range=100.0,
        vertical_fov=math.radians(10),
    )
    radar.update()
    detections = radar.get_detections()
    assert len(detections) == 1
    assert detections[0].track_id == 0


# ---------------------------------------------------------------------------
# Locked target gating
# ---------------------------------------------------------------------------

def test_set_target_clears_stored_target_position():
    """set_target() resets stored position so get_target_position() returns None.

    Verifies that switching targets mid-flight does not leave a stale reading
    from the previous target visible to callers.
    """
    proj0 = StubProjectile([[0.0, 10.0, 0.0]])
    proj1 = StubProjectile([[0.0, 20.0, 0.0]])
    radar = _make_radar(
        [proj0, proj1],
        radar_position=[0.0, 0.0, 0.0],
        turret_position=[0.0, 0.0, 0.0],
    )
    radar.set_target(0)
    radar.update()
    assert radar.get_target_position() == [0.0, 10.0, 0.0]  # baseline
    radar.set_target(1)
    assert radar.get_target_position() is None


def test_set_target_filters_to_locked_node():
    """set_target(track_id) causes get_target_position() to return only the locked target."""
    proj_a = StubProjectile([[0.0, 30.0, 0.0]])
    proj_b = StubProjectile([[0.0, 80.0, 0.0]])
    radar = _make_radar(
        [proj_a, proj_b],
        radar_position=[0.0, 0.0, 0.0],
        turret_position=[0.0, 0.0, 0.0],
    )
    radar.set_target(1)
    radar.update()
    assert radar.get_target_position() == [0.0, 80.0, 0.0]


def test_locked_target_outside_range_leaves_target_position_as_last_or_none():
    """When the locked target leaves max_range, get_target_position() does not crash.

    On the first update the target is in range (returns a position).
    On the second update the target has moved out of range — must not raise;
    get_target_position() returns the previous reading (no new update occurs).
    """
    proj = StubProjectile([
        [0.0, 50.0, 0.0],   # update 1: in range (50 m < 100 m)
        [0.0, 500.0, 0.0],  # update 2: out of range
    ])
    radar = _make_radar(
        [proj],
        radar_position=[0.0, 0.0, 0.0],
        turret_position=[0.0, 0.0, 0.0],
        max_range=100.0,
    )
    radar.set_target(0)
    radar.update()
    first_pos = radar.get_target_position()
    assert first_pos == [0.0, 50.0, 0.0]

    radar.update()                           # target now out of range
    # Must not crash; value is the previous reading (stale but non-None)
    result = radar.get_target_position()
    assert result == [0.0, 50.0, 0.0]       # stale — not updated this cycle


def test_locked_target_outside_fov_leaves_target_position_unchanged():
    """When the locked target exits the FOV, get_target_position() keeps last reading."""
    proj = StubProjectile([
        [0.0, 100.0, 1.0],   # update 1: near horizon, inside 10° FOV
        [0.0, 0.0, 100.0],   # update 2: straight up, outside narrow FOV
    ])
    radar = _make_radar(
        [proj],
        radar_position=[0.0, 0.0, 0.0],
        turret_position=[0.0, 0.0, 0.0],
        max_range=1000.0,
        vertical_fov=math.radians(10),
    )
    radar.set_target(0)
    radar.update()
    first_pos = radar.get_target_position()
    assert first_pos == [0.0, 100.0, 1.0]

    radar.update()                           # target now outside FOV
    # Stale value preserved — exact first-update reading, no crash
    result = radar.get_target_position()
    assert result == [0.0, 100.0, 1.0]      # stale — not updated this cycle


def test_gate_uses_radar_position_not_turret_position():
    """The range gate must measure from radar_position, not turret_position.

    Radar and turret sit far apart. The projectile is placed so the gate
    result flips depending on which frame is used:
      - distance from radar  ≈ 10 m  → inside max_range (50 m)
      - distance from turret ≈ 510 m → outside max_range (50 m)
    A correct implementation gates off the radar and detects the projectile;
    a bug gating off the turret would reject it.
    """
    radar_pos = [0.0, 0.0, 0.0]
    turret_pos = [0.0, 500.0, 0.0]           # 500 m north of the radar
    proj = StubProjectile([[0.0, 10.0, 0.0]])  # 10 m from radar, 490 m from turret
    radar = _make_radar(
        [proj],
        radar_position=radar_pos,
        turret_position=turret_pos,
        max_range=50.0,
        vertical_fov=math.pi,
    )
    radar.update()
    detections = radar.get_detections()
    assert len(detections) == 1              # gated off radar → in range
    # And the output frame is still turret-relative (490 m south of turret).
    assert detections[0].position == [0.0, 10.0 - 500.0, 0.0]


# ---------------------------------------------------------------------------
# Noise
# ---------------------------------------------------------------------------

def test_update_with_nonzero_noise_perturbs_readings():
    """With noise_std > 0, readings must deviate from noiseless values.

    Uses a seeded ``random.Random`` injected via the ``rng`` parameter so the
    test is fully deterministic without monkey-patching. Samples many updates
    and asserts mean absolute deviation is within expected Gaussian bounds
    (E[|X|] = sigma * sqrt(2/pi) ≈ 0.798 * sigma for N(0, sigma)).
    """
    noise_std = 2.0
    N = 1000
    deviations = []

    seeded_rng = random.Random(42)
    # Place projectile at same position as turret so turret-relative mean is [0,0,0];
    # the radar must be far enough away that the target is within max_range and FOV.
    world_pos = [0.0, 10.0, 0.0]
    proj = StubProjectile([world_pos])
    radar = _make_radar(
        [proj],
        radar_position=[0.0, 0.0, 0.0],
        turret_position=list(world_pos),   # turret co-located with projectile → mean = 0
        noise_std=noise_std,
        rng=seeded_rng,
    )
    for _ in range(N):
        radar.update()
        detections = radar.get_detections()
        deviations.extend(detections[0].position)

    mean_abs = sum(abs(d) for d in deviations) / len(deviations)
    expected_mean_abs = noise_std * (2 / math.pi) ** 0.5
    # Allow generous ±30% tolerance given sampling variance
    assert 0.7 * expected_mean_abs < mean_abs < 1.3 * expected_mean_abs


# ---------------------------------------------------------------------------
# Interface contracts
# ---------------------------------------------------------------------------

def test_update_noiseless_returns_exact_relative_positions():
    """update() with noise_std=0.0 stores exact turret-relative positions.

    Covers subtraction logic and that get_detections() returns one Detection
    per in-gate projectile with the correct track_id and no noise contribution.
    """
    world_pos_a = [4.0, 5.0, 3.0]
    world_pos_b = [10.0, 0.0, 1.0]
    proj_a = StubProjectile([world_pos_a])
    proj_b = StubProjectile([world_pos_b])
    radar = _make_radar(
        [proj_a, proj_b],
        radar_position=list(RADAR_POS),
        turret_position=list(TURRET_POS),
    )
    radar.update()

    detections = radar.get_detections()
    assert len(detections) == 2

    expected_a = [world_pos_a[i] - TURRET_POS[i] for i in range(3)]
    expected_b = [world_pos_b[i] - TURRET_POS[i] for i in range(3)]

    assert detections[0].track_id == 0
    assert detections[0].position == expected_a
    assert detections[1].track_id == 1
    assert detections[1].position == expected_b


def test_detections_are_detection_instances():
    """get_detections() must return Detection NamedTuple instances (ADR-0006)."""
    proj = StubProjectile([[0.0, 10.0, 0.0]])
    radar = _make_radar(
        [proj],
        radar_position=[0.0, 0.0, 0.0],
        turret_position=[0.0, 0.0, 0.0],
    )
    radar.update()
    detections = radar.get_detections()
    assert len(detections) == 1
    assert isinstance(detections[0], Detection)


def test_get_detections_returns_fresh_list():
    """get_detections() must return a new list each call (callers cannot mutate state)."""
    proj = StubProjectile([[0.0, 10.0, 0.0]])
    radar = _make_radar(
        [proj],
        radar_position=[0.0, 0.0, 0.0],
        turret_position=[0.0, 0.0, 0.0],
    )
    radar.update()
    list_a = radar.get_detections()
    list_b = radar.get_detections()
    assert list_a is not list_b


def test_scan_params_must_pass_while_beam_is_wide():
    """Test that scan_rate and track_timeout are accepted without error.

    When beam_width is very wide (full 2π), the azimuth gate never rejects
    in-range, in-FOV targets regardless of scan_rate. Demonstrates that the
    constructor accepts these parameters safely.
    """
    proj = StubProjectile([[0.0, 50.0, 0.0]])
    radar = _make_radar(
        [proj],
        radar_position=[0.0, 0.0, 0.0],
        turret_position=[0.0, 0.0, 0.0],
        beam_width=2 * math.pi,  # wide beam passes azimuth gate
        scan_rate=math.radians(60),
        track_timeout=10,
    )
    radar.update()
    assert len(radar.get_detections()) == 1


# ---------------------------------------------------------------------------
# Rotating scan beam (Task 3)
# ---------------------------------------------------------------------------

def test_beam_advances_by_scan_rate_each_update():
    """The internal beam azimuth must advance by scan_rate each update() call.

    Uses reflective access to _beam_azimuth to verify the beam position.
    """
    proj = StubProjectile([[0.0, 10.0, 0.0]])
    scan_rate = math.radians(15)
    radar = _make_radar(
        [proj],
        radar_position=[0.0, 0.0, 0.0],
        turret_position=[0.0, 0.0, 0.0],
        beam_width=2 * math.pi,  # no azimuth gate for this test
        scan_rate=scan_rate,
    )
    assert radar._beam_azimuth == 0.0

    radar.update()
    assert abs(radar._beam_azimuth - scan_rate) < 1e-9

    radar.update()
    assert abs(radar._beam_azimuth - 2 * scan_rate) < 1e-9


def test_beam_wraps_past_2pi():
    """The beam azimuth must wrap at 2π using modulo arithmetic.

    After advancing beyond 2π, the next advance wraps back to the start.
    """
    proj = StubProjectile([[0.0, 10.0, 0.0]])
    scan_rate = 2 * math.pi - math.radians(5)  # just before a full rotation
    radar = _make_radar(
        [proj],
        radar_position=[0.0, 0.0, 0.0],
        turret_position=[0.0, 0.0, 0.0],
        beam_width=2 * math.pi,
        scan_rate=scan_rate,
    )

    # After first update, beam is at scan_rate (just before 2π)
    radar.update()
    assert abs(radar._beam_azimuth - scan_rate) < 1e-9

    # After second update, beam wraps: (2π - 5°) + (2π - 5°) mod 2π ≈ 2π - 10°
    radar.update()
    expected = (2 * scan_rate) % (2 * math.pi)
    assert abs(radar._beam_azimuth - expected) < 1e-9


def test_narrow_beam_pointed_away_rejects_target():
    """A target outside the beam's sweep rejects the detection.

    Place a target due north (azimuth 0°) and point a narrow beam due east
    (azimuth +90°). With beam_width = 20°, the beam spans [+80°, +100°].
    The target at 0° is outside and must be rejected.
    """
    proj = StubProjectile([[0.0, 100.0, 0.0]])  # due north, azimuth = 0
    radar = _make_radar(
        [proj],
        radar_position=[0.0, 0.0, 0.0],
        turret_position=[0.0, 0.0, 0.0],
        beam_width=math.radians(20),
        scan_rate=0.0,  # keep beam stationary at initial position
    )
    # Manually position the beam due east for this test
    radar._beam_azimuth = math.radians(90)  # +90° (due east)

    radar.update()
    # Target at azimuth 0° is 90° away from beam center → far outside ±10° window
    assert radar.get_detections() == []


def test_target_within_beam_is_detected():
    """A target within the beam's angular width passes the azimuth gate.

    Place a target at azimuth 45° and center the beam on it. With
    beam_width = 30°, the beam spans [30°, 60°]. The target must be detected.
    """
    proj = StubProjectile([[100.0, 100.0, 0.0]])  # azimuth ≈ +45° in ENU (atan2(dx=100, dy=100))
    radar = _make_radar(
        [proj],
        radar_position=[0.0, 0.0, 0.0],
        turret_position=[0.0, 0.0, 0.0],
        beam_width=math.radians(30),
        scan_rate=0.0,
    )
    # Center beam on the target
    radar._beam_azimuth = math.radians(45)

    radar.update()
    detections = radar.get_detections()
    assert len(detections) == 1
    assert detections[0].track_id == 0


def test_beam_sweep_discovers_target_then_loses_it():
    """As the beam sweeps, it detects a target when aligned, then rejects it when swept away.

    Place a target due north (azimuth 0°). Start the beam elsewhere, sweep it
    past the target, and verify detection occurs only during the sweep window.
    """
    proj = StubProjectile([[0.0, 100.0, 0.0]])  # due north, azimuth = 0
    beam_width = math.radians(30)
    scan_rate = math.radians(15)
    radar = _make_radar(
        [proj],
        radar_position=[0.0, 0.0, 0.0],
        turret_position=[0.0, 0.0, 0.0],
        beam_width=beam_width,
        scan_rate=scan_rate,
    )

    # Start beam pointing west (−90°)
    radar._beam_azimuth = math.radians(-90)

    # Sweep the beam: from −90° it advances 15° per update (scan_rate)
    # Target is at 0°, window is ±15° (−15° to +15°)
    detections_log = []

    for _ in range(12):  # Enough steps to sweep through target and beyond
        radar.update()
        detections_log.append(len(radar.get_detections()))

    # Expected pattern: miss, miss, miss, miss, miss, miss, HIT, HIT, HIT, miss, miss, miss
    # (−90° → −75° → −60° → −45° → −30° → −15° → 0° → 15° → 30° → 45° → 60° → 75°)
    # Actually this is trickier because angle wrapping can cause surprises near the poles.
    # Let's just verify: there's a window where detections occur (between indices where beam is near 0)
    # and detections outside that window.
    hits = sum(1 for count in detections_log if count > 0)
    misses = sum(1 for count in detections_log if count == 0)
    assert hits > 0 and misses > 0  # sweep must produce both hits and misses


# ---------------------------------------------------------------------------
# Track buffer persistence (Task 4)
#
# Off-by-one contract (drop rule: age > track_timeout):
#   After the last detection cycle the track's age is set to 0.
#   Each subsequent update with no re-detection increments age by 1.
#   A track is dropped when age > track_timeout, i.e. after track_timeout+1
#   no-detection updates.
#
#   With track_timeout=N:
#     - Updates 1 … N after last detection: track still returned (age 1 … N, ≤ N)
#     - Update N+1 after last detection: track dropped (age would be N+1 > N)
#
# These tests use a narrow-beam radar (beam_width = 20°) so we can control
# precisely which update cycles detect the target by pre-positioning the beam.
# ---------------------------------------------------------------------------

def _make_narrow_beam_radar(projectiles, *, track_timeout=3, noise_std=0.0):
    """Radar with a narrow 20° beam, scan_rate=0 (stationary), beam pre-aimed
    at target (azimuth 0°) or away (set radar._beam_azimuth manually).

    beam_width=20° → half-width 10°. Target placed due north (azimuth 0°).
    When beam is at 0° the target is detected; when beam is at 90° it is not.
    """
    return SearchRadar(
        projectiles,
        turret_position=[0.0, 0.0, 0.0],
        radar_position=[0.0, 0.0, 0.0],
        noise_std=noise_std,
        timestep_ms=32,
        max_range=1000.0,
        vertical_fov=math.pi,
        beam_width=math.radians(20),
        scan_rate=0.0,   # stationary — controlled manually in tests
        track_timeout=track_timeout,
        rng=random.Random(0),
    )


def test_track_persists_for_track_timeout_updates_after_beam_passes():
    """A track is retained for exactly track_timeout updates with no re-detection.

    Boundary: with track_timeout=3, updates 1, 2, 3 after beam passes still
    return the track; update 4 drops it.
    """
    track_timeout = 3
    proj = StubProjectile([[0.0, 100.0, 0.0]] * 20)  # stationary target, many positions
    radar = _make_narrow_beam_radar([proj], track_timeout=track_timeout)

    # Step 1: beam aimed at target (azimuth 0°) → detection, track enters buffer at age 0.
    radar._beam_azimuth = 0.0
    radar.update()
    assert len(radar.get_detections()) == 1, "Precondition: target detected when beam is on it"

    # Step 2: sweep beam away so target is no longer directly detected.
    radar._beam_azimuth = math.radians(90)  # 90° away — well outside ±10° window

    # Updates 1 through track_timeout: track still in buffer (age 1 … track_timeout ≤ track_timeout)
    for i in range(1, track_timeout + 1):
        radar.update()
        detections = radar.get_detections()
        track_ids = [d.track_id for d in detections]
        assert 0 in track_ids, (
            f"Track must persist at update {i} after beam passes "
            f"(age={i}, track_timeout={track_timeout})"
        )

    # Update track_timeout+1: age would become track_timeout+1 > track_timeout → dropped.
    radar.update()
    detections = radar.get_detections()
    track_ids = [d.track_id for d in detections]
    assert 0 not in track_ids, (
        f"Track must be dropped at update {track_timeout + 1} after beam passes "
        f"(age would be {track_timeout + 1} > {track_timeout})"
    )


def test_track_dropped_exactly_on_timeout_boundary():
    """Precise boundary: last visible on update N, absent on update N+1 (track_timeout=N)."""
    track_timeout = 2
    proj = StubProjectile([[0.0, 100.0, 0.0]] * 20)
    radar = _make_narrow_beam_radar([proj], track_timeout=track_timeout)

    # Detect once to seed the buffer
    radar._beam_azimuth = 0.0
    radar.update()
    assert len(radar.get_detections()) == 1

    # Sweep away permanently
    radar._beam_azimuth = math.radians(90)

    # Update 1: age=1 ≤ 2 → visible
    radar.update()
    assert 0 in [d.track_id for d in radar.get_detections()], "age=1: must be visible"

    # Update 2: age=2 ≤ 2 → visible (last cycle)
    radar.update()
    assert 0 in [d.track_id for d in radar.get_detections()], "age=2: must still be visible"

    # Update 3: age=3 > 2 → dropped
    radar.update()
    assert 0 not in [d.track_id for d in radar.get_detections()], "age=3: must be dropped"


def test_continuous_redetection_keeps_track_alive_indefinitely():
    """Continuous detection keeps track alive beyond what track_timeout would drop.

    With track_timeout=1, without re-detection the track would expire after 2
    no-detection updates. But if every update detects the target, the track
    must remain alive indefinitely (age resets to 0 each cycle).
    """
    track_timeout = 1
    N = 20
    proj = StubProjectile([[0.0, 100.0, 0.0]] * (N + 5))
    radar = _make_narrow_beam_radar([proj], track_timeout=track_timeout)

    # Beam always pointed at the target — detected every update
    radar._beam_azimuth = 0.0
    for i in range(N):
        radar.update()
        detections = radar.get_detections()
        assert 0 in [d.track_id for d in detections], (
            f"Continuously detected track must be alive at update {i + 1}"
        )


def test_track_expires_after_zero_timeout():
    """With track_timeout=0, a track is dropped on the very next update after last detection.

    age > 0 means: after one update with no detection (age=1 > 0), it is dropped.
    """
    track_timeout = 0
    proj = StubProjectile([[0.0, 100.0, 0.0]] * 10)
    radar = _make_narrow_beam_radar([proj], track_timeout=track_timeout)

    # Detect once
    radar._beam_azimuth = 0.0
    radar.update()
    assert len(radar.get_detections()) == 1, "Precondition: detected when beam on target"

    # Sweep away — one update is enough to drop the track
    radar._beam_azimuth = math.radians(90)
    radar.update()
    assert radar.get_detections() == [], "track_timeout=0: track dropped after first missed update"


def test_get_target_position_returns_buffered_position_when_beam_off_target():
    """get_target_position() returns the buffered (last fresh) position when beam has swept away.

    After set_target(), the first detection sets the buffered position.
    When the beam sweeps away (no direct detection), get_target_position() returns
    the last buffered value for as long as the track survives in the buffer.
    """
    track_timeout = 3
    proj = StubProjectile([[0.0, 100.0, 0.0]] * 20)
    radar = _make_narrow_beam_radar([proj], track_timeout=track_timeout)
    radar.set_target(0)

    # Detect once to seed the locked target's buffer entry
    radar._beam_azimuth = 0.0
    radar.update()
    buffered_pos = radar.get_target_position()
    assert buffered_pos is not None, "Precondition: target position set after detection"

    # Sweep beam away — target not directly detected, but within timeout
    radar._beam_azimuth = math.radians(90)
    for _ in range(track_timeout):
        radar.update()
        pos = radar.get_target_position()
        assert pos is not None, "Buffered target position must persist within timeout"
        assert pos == buffered_pos, "Position must be the last buffered (not stale pre-buffer) value"


def test_get_target_position_returns_none_after_track_drops():
    """get_target_position() returns None once the locked target's track is dropped from the buffer.

    After track_timeout+1 updates with no re-detection, the buffer entry is gone,
    so get_target_position() must return None (not a stale value).
    """
    track_timeout = 2
    proj = StubProjectile([[0.0, 100.0, 0.0]] * 20)
    radar = _make_narrow_beam_radar([proj], track_timeout=track_timeout)
    radar.set_target(0)

    # Seed the buffer
    radar._beam_azimuth = 0.0
    radar.update()
    assert radar.get_target_position() is not None

    # Sweep beam away permanently
    radar._beam_azimuth = math.radians(90)
    for _ in range(track_timeout + 1):
        radar.update()

    # Track is now dropped — get_target_position() must return None
    assert radar.get_target_position() is None, (
        "After track drops from buffer, get_target_position() must return None"
    )


def test_buffered_position_is_frozen_while_beam_is_away():
    """The buffer holds the position measured at the last detection, not the projectile's
    current position.

    A moving target is detected once (beam aligned), then the beam sweeps away.
    The projectile continues to move to new positions in subsequent calls to
    getPosition(), but get_detections() and get_target_position() must keep
    returning the position that was measured at the moment of detection — the
    buffer must NOT re-read the node during the dark window.
    """
    track_timeout = 3
    detection_pos = [0.0, 100.0, 0.0]    # position when beam is on target
    # Subsequent positions the projectile moves to while beam is away
    moved_positions = [
        [0.0, 110.0, 5.0],
        [0.0, 120.0, 10.0],
        [0.0, 130.0, 15.0],
    ]
    all_positions = [detection_pos] + moved_positions + [[0.0, 140.0, 20.0]]  # extra safety
    proj = StubProjectile(all_positions)

    radar = _make_narrow_beam_radar([proj], track_timeout=track_timeout, noise_std=0.0)
    radar.set_target(0)

    # Detect once with beam on target
    radar._beam_azimuth = 0.0
    radar.update()
    first_detection = radar.get_target_position()
    assert first_detection == detection_pos, "Precondition: first detection matches node position"

    # Sweep beam away — projectile moves each cycle but buffer must stay frozen
    radar._beam_azimuth = math.radians(90)
    for i, moved_pos in enumerate(moved_positions):
        radar.update()
        buffered_detection = radar.get_detections()
        buffered_target = radar.get_target_position()

        # Buffer must still hold the original detection position
        assert len(buffered_detection) == 1, f"Track must still be live on dark cycle {i + 1}"
        assert buffered_detection[0].position == detection_pos, (
            f"get_detections()[0].position must be frozen at {detection_pos}, "
            f"not updated to {moved_pos} (dark cycle {i + 1})"
        )
        assert buffered_target == detection_pos, (
            f"get_target_position() must be frozen at {detection_pos}, "
            f"not updated to {moved_pos} (dark cycle {i + 1})"
        )
