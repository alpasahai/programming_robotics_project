# Search Radar Controller — Thin Orchestration Refactor Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Reduce `search_radar_controller.py` from a fat control loop carrying real logic to a thin orchestration layer, mirroring `atlas_controller.py`, while closing the ADR-0006 sensor-membrane leaks it currently commits.

**Architecture:** Lift every piece of per-step logic out of the loop into focused, unit-tested modules: two new public methods on `SearchRadar` (so the controller stops reaching into its private fields), a controller-local `beam_alignment` module (pure geometry + a thin Webots-node wrapper), a producer-side `cue_emitter` module (the mirror of `SearchRadarLink`), and a `cue_telemetry` module (the mirror of `atlas_controller`'s `TrackTelemetry`). The loop then collapses to ~6 orchestration lines.

**Tech Stack:** Python 3.13, Webots `controller` API (Supervisor, Motor, PositionSensor, Emitter), `struct` for the radio payload, `pytest` for tests. Shared code lives in `lib/` (already on the test `sys.path` via `tests/conftest.py`); controller-specific modules sit beside `search_radar_controller.py`.

---

## Background & Context (read before starting)

`atlas_controller.py` is the model: it constructs components, wires them, and runs a numbered per-step loop where **every line is a one-call delegation** to a tested module (`cue_link.update()`, `fcr.update(...)`, `track_filter.predict()`, `fsm.step()`, `telemetry.report(...)`). No decision logic lives in the loop. `TrackTelemetry` (in `controllers/atlas_controller/telemetry.py`) owns all per-step logging and is unit-tested.

`search_radar_controller.py` today violates this in three ways:

1. **Membrane leaks (ADR-0006).** The loop reaches into `SearchRadar` private state every step:
   - `search_radar._radar_position = ...` and `search_radar._beam_azimuth = ...` (overwriting the beam pose from the rendered FOV node);
   - `search_radar._track_buffer[detection.track_id][1] == 0` (filtering to freshly-detected tracks);
   - `search_radar._max_range` (read for the beam-range geometry).
   ADR-0006 makes the sensor membrane a *structural* invariant. Reaching past the public API is exactly the leak it warns against. We close these with public methods.

2. **Geometry helpers in the controller.** `_joint_angle_to_search_azimuth` and `_visual_beam_pose_to_search_frame` are pure functions stuck in the controller file, untested.

3. **Triplicated logging.** Three ~25-line `log` blocks format the same `beam_az` / `visual_origin` / `visual_dir` fields for the no-cue, buffer-held, and cue-sent paths.

**What is the "SR process membrane"?** Per ADR-0006 the Search Radar controller is one process that *owns* projectile node handles, `Detection` values, and `track_id`s internally. Only a bare world-frame `[x, y, z]` may cross the radio link to ATLAS. So `track_id`, `Detection`, and selection logic are all legitimately controller-side here — the leak we are fixing is reaching into `SearchRadar`'s **private fields**, not using `track_id` at all.

**Deliberate scope decision — per-track ages dropped from logs.** The old `[SR BUFFER HELD]` line logged each buffered track's age, read straight from `_track_buffer`. The refactored telemetry logs the buffered `track_id`s (from the public `get_detections()`) but **not** their ages, because exposing ages would require either another private read or a new public accessor whose only consumer is a debug log. This is an intentional, minor reduction in debug detail in exchange for closing the leak. Call it out in the commit message.

**How tests are run:** from `controllers/search_radar_controller/`, run `python -m pytest tests/ -q`. `tests/conftest.py` puts the controller dir, the `tests/` dir, and `lib/` on `sys.path`, so production modules import bare (`from search_radar import ...`), stubs import bare (`from stubs import ...`), and shared lib imports bare (`import geometry`). 36 tests currently pass.

**Out of scope:** `sweep_control.py` (an unused pure helper — leave it), the one-time FOV-visual-cone proto-field configuration block (it is setup wiring, like `atlas_controller`'s `telemetry.log_legend()`, and stays inline in the controller), and the existing `search_radar.py` tests that poke `_beam_azimuth` directly (they keep working against the field; do not rewrite them).

---

## File Structure

**New files:**
- `controllers/search_radar_controller/beam_alignment.py` — pure azimuth/pose geometry functions + a thin `BeamAlignment` wrapper that reads the Webots FOV node / angle sensor and applies the pose to a radar. Returns a `BeamInfo` value for telemetry.
- `controllers/search_radar_controller/cue_emitter.py` — `CueResult` dataclass + `CueEmitter` class (producer-side mirror of `SearchRadarLink`): select fresh detection → world frame → `struct.pack` → send.
- `controllers/search_radar_controller/cue_telemetry.py` — `CueTelemetry` class (mirror of `TrackTelemetry`): owns every per-step log line.
- `controllers/search_radar_controller/tests/test_beam_alignment.py`
- `controllers/search_radar_controller/tests/test_cue_emitter.py`
- `controllers/search_radar_controller/tests/test_cue_telemetry.py`

**Modified files:**
- `controllers/search_radar_controller/search_radar.py` — add public `get_fresh_detections()` and `set_beam_pose()`.
- `controllers/search_radar_controller/tests/test_search_radar.py` — add tests for the two new methods.
- `controllers/search_radar_controller/search_radar_controller.py` — rewrite into thin orchestration.

---

## Task 1: `SearchRadar.get_fresh_detections()`

Replaces the loop's `search_radar._track_buffer[detection.track_id][1] == 0` filter with a public method.

**Files:**
- Modify: `controllers/search_radar_controller/search_radar.py` (add method after `get_detections`, ~line 279)
- Test: `controllers/search_radar_controller/tests/test_search_radar.py` (append; reuses the file's existing `_make_narrow_beam_radar` helper and `StubProjectile` import)

- [ ] **Step 1: Write the failing tests**

Append to `tests/test_search_radar.py`:

```python
# ---------------------------------------------------------------------------
# Fresh-detection accessor (Task 1) — age-0 tracks only
# ---------------------------------------------------------------------------

def test_get_fresh_detections_empty_before_update():
    """get_fresh_detections() returns an empty list before any update()."""
    proj = StubProjectile([[0.0, 10.0, 0.0]])
    radar = _make_radar([proj])
    assert radar.get_fresh_detections() == []


def test_get_fresh_detections_returns_age_zero_track():
    """A track detected on the most recent update() is fresh (age 0)."""
    proj = StubProjectile([[0.0, 100.0, 0.0]] * 5)
    radar = _make_narrow_beam_radar([proj], track_timeout=3)
    radar._beam_azimuth = 0.0      # beam on target (due north)
    radar.update()
    assert [d.track_id for d in radar.get_fresh_detections()] == [0]


def test_get_fresh_detections_excludes_buffered_but_aged_track():
    """A track held in the buffer but not detected this cycle is NOT fresh,
    even though get_detections() still returns it."""
    proj = StubProjectile([[0.0, 100.0, 0.0]] * 5)
    radar = _make_narrow_beam_radar([proj], track_timeout=3)
    radar._beam_azimuth = 0.0      # detect once → age 0
    radar.update()
    radar._beam_azimuth = math.radians(90)   # sweep away → next update ages it
    radar.update()
    assert radar.get_fresh_detections() == []                   # aged out of "fresh"
    assert [d.track_id for d in radar.get_detections()] == [0]   # still buffered
```

- [ ] **Step 2: Run tests to verify they fail**

Run: `cd controllers/search_radar_controller && python -m pytest tests/test_search_radar.py -k fresh_detections -v`
Expected: FAIL with `AttributeError: 'SearchRadar' object has no attribute 'get_fresh_detections'`

- [ ] **Step 3: Add the method**

In `search_radar.py`, immediately after the `get_detections` method (after the `return [det for det, _age in self._track_buffer.values()]` line, ~line 279):

```python
    def get_fresh_detections(self) -> list[Detection]:
        """Return Detection objects for tracks detected on the most recent update().

        A track is *fresh* when its buffer age is 0 — the beam was directly on it
        during the last ``update()``. Tracks held between beam passes (age > 0, kept
        alive by ``track_timeout``) are excluded. This is the membrane-safe
        replacement for the controller reaching into ``_track_buffer`` to test age.

        Returns a fresh list each call; ``Detection`` is immutable so elements are
        safe to share. Returns an empty list before the first ``update()``.
        """
        return [det for det, age in self._track_buffer.values() if age == 0]
```

- [ ] **Step 4: Run tests to verify they pass**

Run: `cd controllers/search_radar_controller && python -m pytest tests/test_search_radar.py -k fresh_detections -v`
Expected: PASS (3 passed)

- [ ] **Step 5: Commit**

```bash
git add controllers/search_radar_controller/search_radar.py controllers/search_radar_controller/tests/test_search_radar.py
git commit -m "feat(search radar): add get_fresh_detections() public accessor

Membrane-safe replacement for the controller reaching into _track_buffer
to filter to age-0 tracks (ADR-0006)."
```

---

## Task 2: `SearchRadar.set_beam_pose()`

Replaces the loop's direct writes to `search_radar._beam_azimuth` and `search_radar._radar_position`.

**Files:**
- Modify: `controllers/search_radar_controller/search_radar.py` (add method after `set_target`, end of class)
- Test: `controllers/search_radar_controller/tests/test_search_radar.py` (append)

- [ ] **Step 1: Write the failing tests**

Append to `tests/test_search_radar.py`:

```python
# ---------------------------------------------------------------------------
# Beam-pose setter (Task 2) — public override of azimuth (+ optional origin)
# ---------------------------------------------------------------------------

def test_set_beam_pose_sets_azimuth_only_when_origin_omitted():
    """Omitting origin overrides azimuth but leaves the phase centre untouched."""
    radar = _make_radar([StubProjectile([[0.0, 10.0, 0.0]])])
    original_origin = list(radar._radar_position)
    radar.set_beam_pose(math.radians(45))
    assert radar._beam_azimuth == math.radians(45)
    assert radar._radar_position == original_origin


def test_set_beam_pose_sets_origin_and_azimuth():
    """Passing origin overrides both the phase centre and the azimuth."""
    radar = _make_radar([StubProjectile([[0.0, 10.0, 0.0]])])
    radar.set_beam_pose(math.radians(30), origin=[1.0, 2.0, 3.0])
    assert radar._beam_azimuth == math.radians(30)
    assert radar._radar_position == [1.0, 2.0, 3.0]


def test_set_beam_pose_copies_origin_list():
    """The stored origin must be a copy, not the caller's list."""
    radar = _make_radar([StubProjectile([[0.0, 10.0, 0.0]])])
    caller_origin = [1.0, 2.0, 3.0]
    radar.set_beam_pose(0.0, origin=caller_origin)
    caller_origin[0] = 99.0
    assert radar._radar_position == [1.0, 2.0, 3.0]
```

- [ ] **Step 2: Run tests to verify they fail**

Run: `cd controllers/search_radar_controller && python -m pytest tests/test_search_radar.py -k set_beam_pose -v`
Expected: FAIL with `AttributeError: 'SearchRadar' object has no attribute 'set_beam_pose'`

- [ ] **Step 3: Add the method**

In `search_radar.py`, at the very end of the `SearchRadar` class (after `set_target`):

```python
    def set_beam_pose(self, azimuth: float, origin: list[float] | None = None) -> None:
        """Override the beam azimuth, and optionally the phase-centre origin.

        The controller calls this each step to align the software detection gate
        with the rendered FOV node, making the Webots proto visual the source of
        truth for beam direction (this avoids re-deriving sign/frame conventions
        from the spin-motor joint). It is the membrane-safe replacement for the
        controller writing ``_beam_azimuth`` / ``_radar_position`` directly.

        Args:
            azimuth: New beam azimuth in radians (SearchRadar convention:
                     x = sin(az), y = cos(az)).
            origin:  New world-frame phase-centre ``[x, y, z]``. ``None`` leaves
                     the constructed ``radar_position`` unchanged — used when only
                     a joint-angle sensor drives the azimuth and the phase centre
                     is fixed.
        """
        self._beam_azimuth = azimuth
        if origin is not None:
            self._radar_position = list(origin)
```

- [ ] **Step 4: Run tests to verify they pass**

Run: `cd controllers/search_radar_controller && python -m pytest tests/test_search_radar.py -k set_beam_pose -v`
Expected: PASS (3 passed)

- [ ] **Step 5: Commit**

```bash
git add controllers/search_radar_controller/search_radar.py controllers/search_radar_controller/tests/test_search_radar.py
git commit -m "feat(search radar): add set_beam_pose() public setter

Membrane-safe replacement for the controller writing _beam_azimuth and
_radar_position directly when aligning to the rendered FOV node (ADR-0006)."
```

---

## Task 3: `beam_alignment` pure geometry functions

Lift the two helper functions out of the controller into a tested module. Keep them **pure** — they take plain numbers, not Webots nodes.

**Files:**
- Create: `controllers/search_radar_controller/beam_alignment.py`
- Test: `controllers/search_radar_controller/tests/test_beam_alignment.py`

- [ ] **Step 1: Write the failing tests**

Create `tests/test_beam_alignment.py`:

```python
"""Tests for beam_alignment — Search Radar beam-pose geometry helpers."""
import math

from beam_alignment import (
    joint_angle_to_search_azimuth,
    visual_beam_pose_to_search_frame,
)


def test_joint_angle_zero_maps_to_zero():
    assert joint_angle_to_search_azimuth(0.0) == 0.0


def test_joint_angle_is_negated_and_wrapped():
    """SearchRadar azimuth is the negated +Z joint angle, wrapped to [0, 2pi)."""
    result = joint_angle_to_search_azimuth(math.radians(90))
    assert math.isclose(result, 2 * math.pi - math.radians(90))


def test_visual_pose_identity_orientation_points_north():
    """Identity orientation: local +Y = world +Y -> azimuth 0; origin sits half a
    beam-range behind the node centre along +Y."""
    orientation = [1, 0, 0, 0, 1, 0, 0, 0, 1]   # row-major identity
    center = [0.0, 5.0, 2.0]
    origin, azimuth, direction = visual_beam_pose_to_search_frame(
        orientation, center, beam_range=10.0
    )
    assert direction == [0.0, 1.0, 0.0]
    assert azimuth == 0.0
    assert origin == [0.0, 0.0, 2.0]            # 5.0 - 1.0 * (10/2)


def test_visual_pose_east_facing_orientation():
    """Local +Y rotated onto world +X -> azimuth = atan2(1, 0) = pi/2."""
    # Column 1 (indices 1,4,7) is the local +Y axis; set it to (1, 0, 0).
    orientation = [0, 1, 0, 0, 0, 0, 0, 0, 0]
    center = [10.0, 0.0, 0.0]
    origin, azimuth, direction = visual_beam_pose_to_search_frame(
        orientation, center, beam_range=20.0
    )
    assert direction == [1.0, 0.0, 0.0]
    assert math.isclose(azimuth, math.pi / 2)
    assert origin == [0.0, 0.0, 0.0]            # 10.0 - 1.0 * (20/2)
```

- [ ] **Step 2: Run tests to verify they fail**

Run: `cd controllers/search_radar_controller && python -m pytest tests/test_beam_alignment.py -v`
Expected: FAIL with `ModuleNotFoundError: No module named 'beam_alignment'`

- [ ] **Step 3: Create the module with the pure functions**

Create `beam_alignment.py`:

```python
"""Search Radar beam-pose geometry — pure helpers + a thin Webots-node wrapper.

The pure functions convert between the spin-motor joint angle / the rendered FOV
node pose and the SearchRadar azimuth convention (x = sin(az), y = cos(az)).
``BeamAlignment`` (added in a later task) wraps the Webots devices so the
controller loop calls a single ``apply(radar)``.
"""
import math


def joint_angle_to_search_azimuth(joint_angle: float) -> float:
    """Convert a Webots +Z joint angle to the SearchRadar azimuth convention.

    SearchRadar azimuth uses x = sin(az), y = cos(az): positive azimuth turns
    local +Y toward +X. Webots positive rotation about +Z turns local +Y toward
    -X, so the physical beam azimuth is the negated joint angle, wrapped to
    [0, 2pi).
    """
    return (-joint_angle) % (2 * math.pi)


def visual_beam_pose_to_search_frame(orientation, center, beam_range):
    """Derive ``(origin, azimuth, direction)`` from a rendered FOV node's pose.

    Webots returns a node orientation as a row-major 9-element rotation matrix.
    Column 1 (indices 1, 4, 7) is the node's local +Y axis in world coordinates —
    the direction the SR_FOV volume extends in SearchRadar.proto.

    Args:
        orientation: row-major 9-element rotation matrix (``node.getOrientation()``).
        center:      world position of the node centre (``node.getPosition()``).
        beam_range:  full beam length in metres; the phase-centre origin sits half
                     a range behind the centre along the beam direction.

    Returns:
        ``(origin, azimuth, direction)`` where ``origin`` is the phase-centre
        ``[x, y, z]``, ``azimuth = atan2(dir_x, dir_y)`` wrapped to [0, 2pi), and
        ``direction`` is the local +Y axis ``[x, y, z]``.
    """
    direction = [orientation[1], orientation[4], orientation[7]]
    origin = [center[i] - direction[i] * (beam_range / 2.0) for i in range(3)]
    azimuth = math.atan2(direction[0], direction[1]) % (2 * math.pi)
    return origin, azimuth, direction
```

- [ ] **Step 4: Run tests to verify they pass**

Run: `cd controllers/search_radar_controller && python -m pytest tests/test_beam_alignment.py -v`
Expected: PASS (4 passed)

- [ ] **Step 5: Commit**

```bash
git add controllers/search_radar_controller/beam_alignment.py controllers/search_radar_controller/tests/test_beam_alignment.py
git commit -m "feat(search radar): extract beam-pose geometry into beam_alignment

Pure, tested helpers lifted verbatim from the controller's inline functions."
```

---

## Task 4: `BeamAlignment` Webots wrapper + `BeamInfo`

Wrap the FOV node and angle sensor so the loop calls one `apply(radar)`. The only Webots-touching code in the alignment path; tested with stub nodes.

**Files:**
- Modify: `controllers/search_radar_controller/beam_alignment.py` (add `BeamInfo` dataclass + `BeamAlignment` class)
- Test: `controllers/search_radar_controller/tests/test_beam_alignment.py` (append)

- [ ] **Step 1: Write the failing tests**

Append to `tests/test_beam_alignment.py`:

```python
from beam_alignment import BeamAlignment, BeamInfo


class StubNode:
    """Minimal Webots node: fixed orientation matrix + world position."""

    def __init__(self, orientation, position):
        self._orientation = orientation
        self._position = position

    def getOrientation(self):
        return self._orientation

    def getPosition(self):
        return self._position


class StubAngleSensor:
    def __init__(self, value):
        self._value = value

    def getValue(self):
        return self._value


class RecordingRadar:
    """Captures the last set_beam_pose() call (azimuth + origin)."""

    def __init__(self):
        self.azimuth = None
        self.origin = "unset"   # sentinel distinct from a real None argument

    def set_beam_pose(self, azimuth, origin=None):
        self.azimuth = azimuth
        self.origin = origin


def test_apply_uses_visual_node_when_present():
    """With an FOV node, apply() derives origin + azimuth from its pose and
    returns a 'visual' BeamInfo. The angle sensor is ignored."""
    node = StubNode([0, 1, 0, 0, 0, 0, 0, 0, 0], [10.0, 0.0, 0.0])  # +Y -> world +X
    align = BeamAlignment(node, StubAngleSensor(1.23), max_range=20.0)
    radar = RecordingRadar()

    info = align.apply(radar)

    assert info.source == "visual"
    assert math.isclose(radar.azimuth, math.pi / 2)
    assert radar.origin == [0.0, 0.0, 0.0]
    assert info.direction == [1.0, 0.0, 0.0]


def test_apply_falls_back_to_joint_when_no_visual_node():
    """Without an FOV node, apply() derives azimuth only (origin left as None)
    from the angle sensor and returns a 'joint' BeamInfo."""
    align = BeamAlignment(None, StubAngleSensor(math.radians(90)), max_range=20.0)
    radar = RecordingRadar()

    info = align.apply(radar)

    assert info.source == "joint"
    assert radar.origin is None
    assert math.isclose(radar.azimuth, 2 * math.pi - math.radians(90))
    assert math.isclose(info.azimuth, 2 * math.pi - math.radians(90))


def test_apply_self_scan_when_no_devices():
    """With neither device, apply() leaves the radar untouched (it self-scans via
    its own scan_rate) and returns a 'self' BeamInfo."""
    align = BeamAlignment(None, None, max_range=20.0)
    radar = RecordingRadar()

    info = align.apply(radar)

    assert info.source == "self"
    assert radar.azimuth is None       # set_beam_pose was never called
    assert info.azimuth is None
```

- [ ] **Step 2: Run tests to verify they fail**

Run: `cd controllers/search_radar_controller && python -m pytest tests/test_beam_alignment.py -k "apply or BeamInfo" -v`
Expected: FAIL with `ImportError: cannot import name 'BeamAlignment' from 'beam_alignment'`

- [ ] **Step 3: Add the dataclass and class**

In `beam_alignment.py`, add `from dataclasses import dataclass` to the imports, then append:

```python
@dataclass(frozen=True)
class BeamInfo:
    """What the beam alignment did this step — consumed only by telemetry.

    ``source`` is "visual" (pose taken from the rendered FOV node), "joint"
    (azimuth taken from the spin-motor angle sensor), or "self" (no device — the
    radar advances its own beam via scan_rate). The optional fields are populated
    only for the sources that compute them.
    """

    source: str
    azimuth: float | None = None
    origin: list[float] | None = None
    direction: list[float] | None = None


class BeamAlignment:
    """Aligns a SearchRadar's beam pose to the rendered FOV node each step.

    Wraps the Webots FOV node and spin-motor angle sensor so the controller loop
    calls a single ``apply(radar)``. Preference order mirrors the original
    controller:

      1. rendered FOV beam node present -> derive origin + azimuth from its pose
         (the proto visual is the source of truth for beam direction);
      2. else angle sensor present -> derive azimuth only (phase centre fixed);
      3. else neither -> the radar self-scans via its own scan_rate (no override).
    """

    def __init__(self, fov_beam_node, angle_sensor, max_range: float):
        self._fov_beam_node = fov_beam_node
        self._angle_sensor = angle_sensor
        self._max_range = max_range

    def apply(self, radar) -> BeamInfo:
        """Compute the beam pose, push it onto ``radar``, and report what was done."""
        if self._fov_beam_node is not None:
            origin, azimuth, direction = visual_beam_pose_to_search_frame(
                self._fov_beam_node.getOrientation(),
                self._fov_beam_node.getPosition(),
                self._max_range,
            )
            radar.set_beam_pose(azimuth, origin=origin)
            return BeamInfo("visual", azimuth, origin, direction)

        if self._angle_sensor is not None:
            azimuth = joint_angle_to_search_azimuth(self._angle_sensor.getValue())
            radar.set_beam_pose(azimuth)
            return BeamInfo("joint", azimuth)

        return BeamInfo("self")
```

- [ ] **Step 4: Run tests to verify they pass**

Run: `cd controllers/search_radar_controller && python -m pytest tests/test_beam_alignment.py -v`
Expected: PASS (7 passed total in the file)

- [ ] **Step 5: Commit**

```bash
git add controllers/search_radar_controller/beam_alignment.py controllers/search_radar_controller/tests/test_beam_alignment.py
git commit -m "feat(search radar): add BeamAlignment wrapper + BeamInfo

Single apply(radar) call hides FOV-node/angle-sensor selection from the loop."
```

---

## Task 5: `cue_emitter` module (`CueResult` + `CueEmitter`)

Producer-side mirror of `SearchRadarLink`: select the fresh detection, convert turret-relative → world, pack, and send. Returns a `CueResult` describing the outcome for telemetry.

**Files:**
- Create: `controllers/search_radar_controller/cue_emitter.py`
- Test: `controllers/search_radar_controller/tests/test_cue_emitter.py`

- [ ] **Step 1: Write the failing tests**

Create `tests/test_cue_emitter.py`:

```python
"""Tests for cue_emitter — producer-side mirror of SearchRadarLink.

Selects the first fresh detection, converts it from turret-relative to world
frame, packs struct.pack("ddd", x, y, z), and sends on the wrapped emitter.
"""
import struct

from search_radar import Detection
from cue_emitter import CueEmitter, CueResult


class StubEmitter:
    """Captures struct-packed payloads passed to send()."""

    def __init__(self):
        self.sent = []

    def send(self, data):
        self.sent.append(data)


def test_emit_returns_none_status_when_no_fresh_detections():
    emitter = StubEmitter()
    result = CueEmitter(emitter).emit([], turret_position=[1.0, 2.0, 3.0])
    assert result.status == "none"
    assert emitter.sent == []


def test_emit_converts_to_world_frame_and_sends():
    emitter = StubEmitter()
    detection = Detection(track_id=0, position=[1.0, 2.0, 3.0])  # turret-relative
    result = CueEmitter(emitter).emit([detection], turret_position=[10.0, 20.0, 30.0])

    assert result.status == "sent"
    assert result.track_id == 0
    assert result.turret_relative == [1.0, 2.0, 3.0]
    assert result.world == [11.0, 22.0, 33.0]
    assert len(emitter.sent) == 1
    assert struct.unpack("ddd", emitter.sent[0]) == (11.0, 22.0, 33.0)


def test_emit_selects_first_fresh_detection():
    emitter = StubEmitter()
    d0 = Detection(track_id=0, position=[1.0, 0.0, 0.0])
    d1 = Detection(track_id=1, position=[2.0, 0.0, 0.0])
    result = CueEmitter(emitter).emit([d0, d1], turret_position=[0.0, 0.0, 0.0])
    assert result.track_id == 0


def test_emit_reports_dropped_when_emitter_missing():
    detection = Detection(track_id=2, position=[0.0, 0.0, 0.0])
    result = CueEmitter(None).emit([detection], turret_position=[5.0, 5.0, 5.0])
    assert result.status == "dropped"
    assert result.track_id == 2
    assert result.world == [5.0, 5.0, 5.0]
```

- [ ] **Step 2: Run tests to verify they fail**

Run: `cd controllers/search_radar_controller && python -m pytest tests/test_cue_emitter.py -v`
Expected: FAIL with `ModuleNotFoundError: No module named 'cue_emitter'`

- [ ] **Step 3: Create the module**

Create `cue_emitter.py`:

```python
"""CueEmitter — selects a fresh detection, converts it to world frame, emits it.

Producer-side mirror of ``SearchRadarLink`` (which consumes these cues in
atlas_controller). The payload is ``struct.pack("ddd", x, y, z)`` — three
IEEE-754 doubles, 24 bytes — sent on the wrapped Webots ``Emitter`` (channel is
configured on the device in the world file).

``Detection.position`` is turret-relative ``[dx, dy, dz]``; the world-frame cue is
``position + turret_position``. Per ADR-0006 only this bare coordinate crosses the
radio link to ATLAS — no node handle and no track_id.
"""
import struct
from dataclasses import dataclass


@dataclass(frozen=True)
class CueResult:
    """Outcome of an ``emit()`` call, consumed by CueTelemetry.

    ``status`` is one of:
      "sent"    — a fresh detection was selected and broadcast;
      "dropped" — a fresh detection was selected but no emitter device exists;
      "none"    — there were no fresh detections to emit this step.

    The position fields are populated for "sent" and "dropped" only.
    """

    status: str
    track_id: int | None = None
    turret_relative: list[float] | None = None
    world: list[float] | None = None


class CueEmitter:
    """Wraps a Webots ``Emitter`` to broadcast the chosen Search Radar cue.

    The controller is responsible for fetching the emitter device; this class
    accepts it (or ``None`` if the device is missing) so the wiring stays in the
    controller and the class stays testable with a stub.
    """

    def __init__(self, emitter):
        self._emitter = emitter

    def emit(self, fresh_detections, turret_position) -> CueResult:
        """Select the first fresh detection, convert to world frame, and send it.

        Args:
            fresh_detections: detections from ``SearchRadar.get_fresh_detections()``.
                              Empty list -> a "none" result.
            turret_position:  world-frame ``[x, y, z]`` of the turret origin; added
                              to the turret-relative detection position.

        Returns:
            A ``CueResult`` describing what happened (for telemetry).
        """
        if not fresh_detections:
            return CueResult(status="none")

        chosen = fresh_detections[0]
        world = [chosen.position[i] + turret_position[i] for i in range(3)]

        if self._emitter is None:
            return CueResult(
                "dropped", chosen.track_id, list(chosen.position), world
            )

        self._emitter.send(struct.pack("ddd", *world))
        return CueResult("sent", chosen.track_id, list(chosen.position), world)
```

- [ ] **Step 4: Run tests to verify they pass**

Run: `cd controllers/search_radar_controller && python -m pytest tests/test_cue_emitter.py -v`
Expected: PASS (4 passed)

- [ ] **Step 5: Commit**

```bash
git add controllers/search_radar_controller/cue_emitter.py controllers/search_radar_controller/tests/test_cue_emitter.py
git commit -m "feat(search radar): add CueEmitter (producer-side mirror of SearchRadarLink)

Selects the fresh detection, converts turret-relative -> world, packs and sends.
Returns a CueResult for telemetry. Closes the inline selection/conversion logic."
```

---

## Task 6: `cue_telemetry` module (`CueTelemetry`)

Mirror of `atlas_controller`'s `TrackTelemetry`. Owns every per-step log line; collapses the three duplicated logging blocks into one place.

**Files:**
- Create: `controllers/search_radar_controller/cue_telemetry.py`
- Test: `controllers/search_radar_controller/tests/test_cue_telemetry.py`

- [ ] **Step 1: Write the failing tests**

Create `tests/test_cue_telemetry.py`:

```python
"""Tests for cue_telemetry — per-step logging for the Search Radar controller."""
from beam_alignment import BeamInfo
from cue_emitter import CueResult
from cue_telemetry import CueTelemetry
from search_radar import Detection


class RecordingLog:
    """Captures (level, message_template) for each logging call."""

    def __init__(self):
        self.calls = []

    def info(self, msg, *args):
        self.calls.append(("info", msg))

    def warning(self, msg, *args):
        self.calls.append(("warning", msg))

    def debug(self, msg, *args):
        self.calls.append(("debug", msg))


def _report(log, *, beam, all_detections, fresh_detections, cue):
    CueTelemetry(log).report(
        step_count=1,
        sim_time=0.1,
        beam=beam,
        all_detections=all_detections,
        fresh_detections=fresh_detections,
        cue=cue,
    )


def test_sent_cue_logs_info():
    log = RecordingLog()
    det = Detection(0, [1.0, 2.0, 3.0])
    _report(
        log,
        beam=BeamInfo("visual", 0.5, [0.0, 0.0, 0.0], [0.0, 1.0, 0.0]),
        all_detections=[det],
        fresh_detections=[det],
        cue=CueResult("sent", 0, [1.0, 2.0, 3.0], [4.0, 5.0, 6.0]),
    )
    assert log.calls[0][0] == "info"
    assert "[SR CUE SENT]" in log.calls[0][1]


def test_dropped_cue_logs_warning():
    log = RecordingLog()
    det = Detection(0, [1.0, 2.0, 3.0])
    _report(
        log,
        beam=BeamInfo("self"),
        all_detections=[det],
        fresh_detections=[],
        cue=CueResult("dropped", 0, [1.0, 2.0, 3.0], [4.0, 5.0, 6.0]),
    )
    assert log.calls[0][0] == "warning"
    assert "[SR CUE DROPPED]" in log.calls[0][1]


def test_detections_but_no_fresh_logs_buffer_held():
    log = RecordingLog()
    det = Detection(0, [1.0, 2.0, 3.0])
    _report(
        log,
        beam=BeamInfo("joint", 0.2),
        all_detections=[det],     # buffered
        fresh_detections=[],      # but none fresh this cycle
        cue=CueResult("none"),
    )
    assert log.calls[0][0] == "debug"
    assert "[SR BUFFER HELD]" in log.calls[0][1]


def test_no_detections_logs_no_cue():
    log = RecordingLog()
    _report(
        log,
        beam=BeamInfo("self"),
        all_detections=[],
        fresh_detections=[],
        cue=CueResult("none"),
    )
    assert log.calls[0][0] == "debug"
    assert "[SR NO CUE]" in log.calls[0][1]
```

- [ ] **Step 2: Run tests to verify they fail**

Run: `cd controllers/search_radar_controller && python -m pytest tests/test_cue_telemetry.py -v`
Expected: FAIL with `ModuleNotFoundError: No module named 'cue_telemetry'`

- [ ] **Step 3: Create the module**

Create `cue_telemetry.py`:

```python
"""Per-step telemetry for the Search Radar controller — a development instrument.

Mirror of atlas_controller's TrackTelemetry: it owns every log line the
controller's loop used to emit inline (cue sent, cue dropped, buffered-but-stale,
no detection at all), so the loop stays pure orchestration. Verbosity is
controlled centrally via lib/atlas_logging.py — set SearchRadarController to INFO
to silence the per-step DEBUG trace.

Note: the buffered-track log reports track_ids (from the public get_detections())
but not per-track ages, which the old inline log read from SearchRadar's private
_track_buffer. Dropping ages keeps the membrane closed (ADR-0006); they were a
debug-only nicety.
"""


class CueTelemetry:
    """Logs one line per step describing what the Search Radar emitted (or didn't)."""

    def __init__(self, log):
        self._log = log

    @staticmethod
    def _beam_fields(beam):
        """Format the BeamInfo fields once for reuse across every log line."""
        azimuth = "%.3f" % beam.azimuth if beam.azimuth is not None else "n/a"
        origin = (
            "(%.3f, %.3f, %.3f)" % tuple(beam.origin)
            if beam.origin is not None
            else "n/a"
        )
        direction = (
            "(%.3f, %.3f, %.3f)" % tuple(beam.direction)
            if beam.direction is not None
            else "n/a"
        )
        return beam.source, azimuth, origin, direction

    def report(
        self,
        step_count,
        sim_time,
        beam,
        all_detections,
        fresh_detections,
        cue,
    ):
        """Emit the single log line appropriate to this step's cue outcome.

        Args:
            step_count:       simulation step counter.
            sim_time:         robot.getTime() in seconds.
            beam:             BeamInfo from BeamAlignment.apply().
            all_detections:   SearchRadar.get_detections() (live buffer).
            fresh_detections: SearchRadar.get_fresh_detections() (age-0 only).
            cue:              CueResult from CueEmitter.emit().
        """
        source, azimuth, origin, direction = self._beam_fields(beam)

        if cue.status == "sent":
            self._log.info(
                "[SR CUE SENT] step=%d t=%.2fs source=fresh_beam_hit track_id=%s "
                "sender=search_radar_controller device=SR_CUE_EMITTER channel=1 "
                "beam_source=%s beam_az=%s origin=%s dir=%s "
                "turret_relative=(%.3f, %.3f, %.3f) world=(%.3f, %.3f, %.3f)",
                step_count,
                sim_time,
                cue.track_id,
                source,
                azimuth,
                origin,
                direction,
                cue.turret_relative[0],
                cue.turret_relative[1],
                cue.turret_relative[2],
                cue.world[0],
                cue.world[1],
                cue.world[2],
            )
        elif cue.status == "dropped":
            self._log.warning(
                "[SR CUE DROPPED] step=%d t=%.2fs sender=search_radar_controller "
                "reason=missing_emitter track_id=%s world=(%.3f, %.3f, %.3f)",
                step_count,
                sim_time,
                cue.track_id,
                cue.world[0],
                cue.world[1],
                cue.world[2],
            )
        elif all_detections:
            self._log.debug(
                "[SR BUFFER HELD] step=%d t=%.2fs buffered_track_ids=%s "
                "beam_source=%s beam_az=%s origin=%s dir=%s",
                step_count,
                sim_time,
                [d.track_id for d in all_detections],
                source,
                azimuth,
                origin,
                direction,
            )
        else:
            self._log.debug(
                "[SR NO CUE] step=%d t=%.2fs detections=0 "
                "beam_source=%s beam_az=%s origin=%s dir=%s",
                step_count,
                sim_time,
                source,
                azimuth,
                origin,
                direction,
            )
```

- [ ] **Step 4: Run tests to verify they pass**

Run: `cd controllers/search_radar_controller && python -m pytest tests/test_cue_telemetry.py -v`
Expected: PASS (4 passed)

- [ ] **Step 5: Commit**

```bash
git add controllers/search_radar_controller/cue_telemetry.py controllers/search_radar_controller/tests/test_cue_telemetry.py
git commit -m "feat(search radar): add CueTelemetry (mirror of TrackTelemetry)

Collapses three duplicated inline logging blocks into one tested instrument."
```

---

## Task 7: Rewrite `search_radar_controller.py` into thin orchestration

Wire the new components together; reduce the loop to numbered one-call steps like `atlas_controller`. This task has no new unit test (the controller imports `from controller import Supervisor`, which only exists inside Webots); it is verified by the full suite still passing, a static check that no membrane leak remains, and a syntax compile.

**Files:**
- Modify (rewrite): `controllers/search_radar_controller/search_radar_controller.py`

- [ ] **Step 1: Replace the file contents**

Overwrite `search_radar_controller.py` with:

```python
"""Search Radar Webots controller — thin Supervisor orchestration.

Constructs the components, wires them, then runs a per-step loop where every
line delegates to a tested module (mirrors atlas_controller.py). No decision
logic lives here:
  - beam_alignment.BeamAlignment aligns the detection beam to the rendered FOV;
  - SearchRadar gates detections and buffers tracks;
  - cue_emitter.CueEmitter selects a fresh detection, converts it to world frame,
    and broadcasts struct.pack("ddd", x, y, z) on channel 1 via SR_CUE_EMITTER;
  - cue_telemetry.CueTelemetry owns all per-step logging.

ADR-0006 (sensor membrane): only a bare world coordinate crosses the radio link
to ATLAS. Node handles and track_ids stay inside this process; the controller
talks to SearchRadar only through its public API.

DEF names (confirmed from worlds/ATLA_v1.wbt):
  PROJECTILE   — the Projectile node
  TURRET_BASE  — the AtlasTurret node (turret origin, world frame)
  SEARCH_RADAR — this robot's own node (base pose; phase-centre offset is
                 defined by SearchRadar.proto)

Execution order each step:
  1. Align  — beam_alignment.apply(search_radar)
  2. Sense  — search_radar.update()
  3. Emit   — cue_emitter.emit(fresh detections)
  4. Report — telemetry.report(...)  (development instrument; no control effect)
"""

import math

from controller import Supervisor

from atlas_logging import configure
from scene import DEF_PROJECTILE, DEF_TURRET_BASE
from search_radar import SearchRadar
from beam_alignment import BeamAlignment
from cue_emitter import CueEmitter
from cue_telemetry import CueTelemetry

# ---------------------------------------------------------------------------
# Search Radar sensor and visualisation parameters. Single source for both the
# SearchRadar model and the visible debug beam.
# ---------------------------------------------------------------------------
SEARCH_RADAR_MAX_RANGE_M = 20.0
SEARCH_RADAR_VERTICAL_FOV_RAD = math.pi / 2
SEARCH_RADAR_BEAM_WIDTH_RAD = 0.35
SEARCH_RADAR_SCAN_RATE_RAD_PER_STEP = 0.32
SEARCH_RADAR_TRACK_TIMEOUT_STEPS = 20
SEARCH_RADAR_NOISE_STD_M = 0.2
SEARCH_RADAR_ROTATING_ENDPOINT_Z_M = 1.0
SEARCH_RADAR_BEAM_LOCAL_Z_M = 1.1
# SearchRadar.proto places the rotating endpoint Solid at z=1.0 and the FOV
# Pose / radar head at local z=1.1 inside that endpoint, so the sensor phase
# centre is 2.1 m above the Robot origin.
SEARCH_RADAR_PHASE_CENTER_OFFSET_M = [
    0.0,
    0.0,
    SEARCH_RADAR_ROTATING_ENDPOINT_Z_M + SEARCH_RADAR_BEAM_LOCAL_Z_M,
]

# ---------------------------------------------------------------------------
# Telemetry logging — console level set centrally in lib/atlas_logging.py
# (LOG_LEVELS["SearchRadarController"]); override per run with
# SEARCHRADARCONTROLLER_LOG_LEVEL or ATLAS_LOG_LEVEL.
# ---------------------------------------------------------------------------
log = configure("SearchRadarController", log_file="search_radar_telemetry.log")
log.info("SEARCH RADAR CONTROLLER STARTED")

# ---------------------------------------------------------------------------
# Bootstrap + devices
# ---------------------------------------------------------------------------
robot = Supervisor()
timestep = int(robot.getBasicTimeStep())

motor = robot.getDevice("SEARCH_RADAR_MOTOR")
if motor is None:
    log.error("SEARCH_RADAR_MOTOR not found")
    while robot.step(timestep) != -1:
        pass

motor.setPosition(float("inf"))
motor_velocity = SEARCH_RADAR_SCAN_RATE_RAD_PER_STEP / (timestep / 1000.0)
motor.setVelocity(motor_velocity)
log.info(
    "SEARCH_RADAR_MOTOR spinning at %.3f rad/s; SearchRadar gates on the "
    "measured joint angle (nominal scan_rate %.3f rad/step)",
    motor_velocity,
    SEARCH_RADAR_SCAN_RATE_RAD_PER_STEP,
)

angle_sensor = robot.getDevice("SEARCH_RADAR_ANGLE")
if angle_sensor is None:
    log.warning("SEARCH_RADAR_ANGLE not found - visual beam angle will not be logged")
else:
    angle_sensor.enable(timestep)

emitter = robot.getDevice("SR_CUE_EMITTER")
if emitter is None:
    log.error("SR_CUE_EMITTER not found - cues will not be emitted")
else:
    log.info("SR_CUE_EMITTER ready: channel=1 payload=struct.pack('ddd', x, y, z)")

# ---------------------------------------------------------------------------
# Scene nodes — confirmed DEF names from worlds/ATLA_v1.wbt
# ---------------------------------------------------------------------------
projectile_node = robot.getFromDef(DEF_PROJECTILE)
if projectile_node is None:
    raise RuntimeError(f"[SR] Could not find DEF {DEF_PROJECTILE} in the world file.")

turret_node = robot.getFromDef(DEF_TURRET_BASE)
if turret_node is None:
    raise RuntimeError(f"[SR] Could not find DEF {DEF_TURRET_BASE} in the world file.")

radar_node = robot.getSelf()

# Static snapshots — neither the turret nor the radar body translates.
turret_position = list(turret_node.getPosition())
radar_origin_position = list(radar_node.getPosition())
radar_position = [
    radar_origin_position[i] + SEARCH_RADAR_PHASE_CENTER_OFFSET_M[i] for i in range(3)
]

log.info("turret_position (world) = %s", [round(v, 3) for v in turret_position])
log.info("radar_origin_position (world) = %s", [round(v, 3) for v in radar_origin_position])
log.info("radar_phase_center    (world) = %s", [round(v, 3) for v in radar_position])

# ---------------------------------------------------------------------------
# Components
# ---------------------------------------------------------------------------
search_radar = SearchRadar(
    [projectile_node],  # list index = track_id (ADR-0006)
    turret_position,
    radar_position,
    noise_std=SEARCH_RADAR_NOISE_STD_M,  # high noise — SearchRadar is coarse
    timestep_ms=timestep,
    max_range=SEARCH_RADAR_MAX_RANGE_M,
    vertical_fov=SEARCH_RADAR_VERTICAL_FOV_RAD,
    beam_width=SEARCH_RADAR_BEAM_WIDTH_RAD,
    # When an FOV node or angle sensor drives the beam, SearchRadar must not also
    # advance its own azimuth; only self-scan when neither device is present.
    scan_rate=0.0 if angle_sensor is not None else SEARCH_RADAR_SCAN_RATE_RAD_PER_STEP,
    track_timeout=SEARCH_RADAR_TRACK_TIMEOUT_STEPS,
)

# Visible FOV beam — drive the exposed PROTO parameters from the SearchRadar
# visual spec so the rendered cone matches the software detection gate.
fov_beam_node = radar_node.getFromProtoDef("SR_FOV_BEAM")
fov_cone_node = radar_node.getFromProtoDef("SR_FOV_CONE")
if fov_beam_node is None or fov_cone_node is None:
    log.warning(
        "Search Radar FOV visual nodes not found via getFromProtoDef; "
        "continuing without beam visual"
    )
else:
    beam_spec = search_radar.get_beam_visual_spec()
    radar_node.getField("fovBeamTranslation").setSFVec3f(
        [
            beam_spec.cone_center_local[0],
            beam_spec.cone_center_local[1],
            SEARCH_RADAR_BEAM_LOCAL_Z_M + beam_spec.cone_center_local[2],
        ]
    )
    radar_node.getField("fovConeHeight").setSFFloat(beam_spec.cone_height)
    radar_node.getField("fovConeRadius").setSFFloat(beam_spec.cone_radius)
    log.info(
        "FOV visual cone configured from SearchRadar visual spec: "
        "origin_world=%s centre_azimuth=%.3frad range=%.2fm beam_width=%.3frad "
        "vertical_fov=%.3frad half_angle=%.3frad cone_height=%.3fm "
        "cone_radius=%.3fm local_center=%s phase_center_offset=%s",
        [round(v, 3) for v in beam_spec.origin_world],
        beam_spec.centre_azimuth,
        beam_spec.max_range,
        beam_spec.beam_width,
        beam_spec.vertical_fov,
        beam_spec.half_angle,
        beam_spec.cone_height,
        beam_spec.cone_radius,
        [round(v, 3) for v in beam_spec.cone_center_local],
        [round(v, 3) for v in SEARCH_RADAR_PHASE_CENTER_OFFSET_M],
    )

beam_alignment = BeamAlignment(fov_beam_node, angle_sensor, SEARCH_RADAR_MAX_RANGE_M)
cue_emitter = CueEmitter(emitter)
telemetry = CueTelemetry(log)

# ---------------------------------------------------------------------------
# Main loop
# ---------------------------------------------------------------------------
step_count = 0

while robot.step(timestep) != -1:
    step_count += 1

    # 1. Align the detection beam to the rendered FOV node (or joint angle).
    beam = beam_alignment.apply(search_radar)

    # 2. Sense — gate projectiles and refresh the track buffer.
    search_radar.update()

    # 3. Emit — select the first fresh detection, convert to world, broadcast.
    detections = search_radar.get_detections()
    fresh_detections = search_radar.get_fresh_detections()
    cue = cue_emitter.emit(fresh_detections, turret_position)

    # 4. Report — per-step development telemetry (no effect on the cue itself).
    telemetry.report(
        step_count, robot.getTime(), beam, detections, fresh_detections, cue
    )
```

- [ ] **Step 2: Compile-check the controller (no Webots needed for syntax)**

Run: `cd controllers/search_radar_controller && python -m py_compile search_radar_controller.py`
Expected: no output, exit code 0 (syntax valid). The Webots `controller` import is only resolved at runtime inside Webots, so do not attempt to execute the file directly.

- [ ] **Step 3: Static membrane-leak check**

Run: `git grep -n "_track_buffer\|_beam_azimuth\|_radar_position\|_max_range" -- controllers/search_radar_controller/search_radar_controller.py`
Expected: **no matches** (empty output). The controller must no longer touch any `SearchRadar` private field. (Matches inside `search_radar.py` and the test files are expected and fine.)

- [ ] **Step 4: Run the full controller test suite**

Run: `cd controllers/search_radar_controller && python -m pytest tests/ -q`
Expected: PASS — the original 36 tests plus the new ones (6 in test_search_radar additions, 7 in test_beam_alignment, 4 in test_cue_emitter, 4 in test_cue_telemetry).

- [ ] **Step 5: Commit**

```bash
git add controllers/search_radar_controller/search_radar_controller.py
git commit -m "refactor(search radar): reduce controller to thin orchestration

Loop now delegates to BeamAlignment, SearchRadar public API, CueEmitter, and
CueTelemetry (mirrors atlas_controller). Closes the ADR-0006 membrane leaks the
loop committed (direct _track_buffer / _beam_azimuth / _radar_position access)."
```

---

## Final verification (after all tasks)

- [ ] Run the whole repo's controller test suites to confirm nothing else broke:
  - `cd controllers/search_radar_controller && python -m pytest tests/ -q`
  - `cd controllers/atlas_controller && python -m pytest tests/ -q`
- [ ] Confirm the loop body in `search_radar_controller.py` is ~6 statements of pure delegation and contains no `if`/`for` decision logic beyond the `while robot.step(...)` driver.
- [ ] Confirm `git grep "_track_buffer\|_beam_azimuth\|_radar_position"` returns matches only in `search_radar.py` and `tests/`.

## Notes for the implementer

- **Import style:** modules import bare (`from search_radar import ...`, `from beam_alignment import ...`) because `tests/conftest.py` and the Webots runtime both put the controller directory on `sys.path`. Do not use package-relative imports.
- **TDD discipline:** for Tasks 1–6, write the test, watch it fail for the *expected* reason, then implement. Don't write implementation ahead of the failing test.
- **Do not** rewrite the existing `search_radar.py` tests that assign `radar._beam_azimuth` directly — they still pass against the field and are out of scope.
- **Do not** touch `sweep_control.py`.
```
