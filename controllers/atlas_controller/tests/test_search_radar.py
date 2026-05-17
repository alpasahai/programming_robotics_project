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
    # deferred scan params — must be accepted, must not affect detection
    beam_width=math.radians(10),
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
    assert radar.get_target_position() is not None

    radar.update()
    # Stale value preserved — no crash
    result = radar.get_target_position()
    assert result is not None               # still the old reading


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


def test_deferred_scan_params_accepted_without_effect():
    """beam_width, scan_rate, track_timeout are stored but do not affect detection output.

    Passing non-default values must not raise and must not change whether an
    in-range, in-FOV target is detected.
    """
    proj = StubProjectile([[0.0, 50.0, 0.0]])
    radar = _make_radar(
        [proj],
        radar_position=[0.0, 0.0, 0.0],
        turret_position=[0.0, 0.0, 0.0],
        beam_width=math.radians(5),
        scan_rate=math.radians(60),
        track_timeout=10,
    )
    radar.update()
    assert len(radar.get_detections()) == 1
