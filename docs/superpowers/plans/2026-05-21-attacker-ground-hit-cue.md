# Attacker Ground-Hit Cue → ATLAS Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Make the attacker emit a one-shot ground-hit pulse and have ATLAS treat that emission as its only source of ground-hit truth, removing the FSM's track-Z landing inference and the PREDICT below-ground guard.

**Architecture:** The attacker's existing `on_ground_hit` seam sends `struct.pack("i", count)` on the existing `ATTACKER_EMITTER` (channel 2). ATLAS wraps the existing `ATTACKER_GROUND_HIT_RECEIVER` in a new `AttackerGroundHitLink` (sibling of `SearchRadarLink`), and the FSM reads `hit_this_step()` in ENGAGING. `_target_in_range` collapses to a range-only `_target_within_range` shared by PREDICT and ENGAGING; `ground_threshold` survives only as a telemetry readout.

**Tech Stack:** Python, Webots Emitter/Receiver radio devices, `struct`, pytest. No proto changes (both devices already declared). No `Projectile` logic changes.

**Spec:** `docs/superpowers/specs/2026-05-21-attacker-ground-hit-cue-design.md`

---

## File Structure

- **Create:** `controllers/atlas_controller/attacker_ground_hit_link.py` — `AttackerGroundHitLink`, decodes the channel-2 int pulse, exposes `update()`, `hit_this_step()`, `count`.
- **Create:** `controllers/atlas_controller/tests/test_attacker_ground_hit_link.py` — unit tests with a local stub receiver.
- **Modify:** `controllers/atlas_controller/fsm.py` — add `ground_hit_link` to `SensorSuite`; replace `_target_in_range` with range-only `_target_within_range`; rewire PREDICT and ENGAGING; drop ground reasoning from FSM logic.
- **Modify:** `controllers/atlas_controller/tests/stubs.py` — add `StubGroundHitLink`.
- **Modify:** `controllers/atlas_controller/tests/test_fsm.py` — thread the stub into all `SensorSuite(...)` call sites and `_make_fsm_in_engaging`; rework/remove the affected tests; add the new "no cue → stays ENGAGING" test.
- **Modify:** `controllers/atlas_controller/atlas_controller.py` — get + enable the receiver, build the link, pass into `SensorSuite`, call `update()` in the sense phase, log hits.
- **Modify:** `controllers/attacker_controller/attacker_controller.py` — get `ATTACKER_EMITTER`, send the packed count from `on_ground_hit`.

Test commands assume CWD `controllers/atlas_controller` (conftest puts the controller dir + tests dir on `sys.path`). Run pytest from that directory.

---

## Task 1: `AttackerGroundHitLink` consumer

**Files:**
- Create: `controllers/atlas_controller/attacker_ground_hit_link.py`
- Test: `controllers/atlas_controller/tests/test_attacker_ground_hit_link.py`

- [ ] **Step 1: Write the failing tests**

Create `controllers/atlas_controller/tests/test_attacker_ground_hit_link.py`:

```python
"""Tests for AttackerGroundHitLink — wraps a Webots Receiver to consume the
attacker's ground-hit pulse.

The attacker emits exactly one packet on the step a ground hit registers:
``struct.pack("i", count)`` on channel 2. The link drains the queue each
update() and reports whether a pulse arrived this step plus the latest count.
"""
import struct
from attacker_ground_hit_link import AttackerGroundHitLink


class StubReceiver:
    """Test double for a Webots Receiver pre-loaded with packed int packets.

    Mimics the Webots Receiver queue API: getQueueLength(), getBytes(),
    nextPacket(). Each int in ``counts`` is packed as "i" exactly as the
    attacker emitter does.
    """

    def __init__(self, counts=None):
        self._queue = [struct.pack("i", c) for c in (counts or [])]

    def load(self, counts):
        """Append more packed-int packets to the queue (simulate a new step)."""
        self._queue.extend(struct.pack("i", c) for c in counts)

    def getQueueLength(self):
        return len(self._queue)

    def getBytes(self):
        return self._queue[0]

    def nextPacket(self):
        self._queue.pop(0)


def test_no_hit_before_any_update():
    """hit_this_step() is False and count is 0 before update() runs."""
    link = AttackerGroundHitLink(StubReceiver())
    assert link.hit_this_step() is False
    assert link.count == 0


def test_empty_queue_reports_no_hit():
    """update() on an empty queue → no hit, count unchanged."""
    link = AttackerGroundHitLink(StubReceiver())
    link.update()
    assert link.hit_this_step() is False
    assert link.count == 0


def test_one_packet_reports_hit_and_count():
    """A single packet → hit_this_step() True and the decoded count."""
    link = AttackerGroundHitLink(StubReceiver([1]))
    link.update()
    assert link.hit_this_step() is True
    assert link.count == 1


def test_multiple_packets_drained_keeps_latest_count():
    """Several packets in one update() → hit True, latest count retained."""
    link = AttackerGroundHitLink(StubReceiver([1, 2, 3]))
    link.update()
    assert link.hit_this_step() is True
    assert link.count == 3


def test_hit_flag_clears_on_next_silent_update():
    """The per-step hit flag resets to False on a later empty update(),
    while count persists."""
    receiver = StubReceiver([5])
    link = AttackerGroundHitLink(receiver)
    link.update()
    assert link.hit_this_step() is True

    link.update()  # queue now empty
    assert link.hit_this_step() is False
    assert link.count == 5  # latest count persists
```

- [ ] **Step 2: Run tests to verify they fail**

Run: `python -m pytest tests/test_attacker_ground_hit_link.py -v`
Expected: FAIL — `ModuleNotFoundError: No module named 'attacker_ground_hit_link'`.

- [ ] **Step 3: Write the implementation**

Create `controllers/atlas_controller/attacker_ground_hit_link.py`:

```python
"""AttackerGroundHitLink — consumes the attacker's ground-hit pulse from a
Webots Receiver.

The attacker emitter sends exactly one packet on the step a ground hit
registers: ``struct.pack("i", count)`` (one 4-byte signed int, the running
ground-hit count) on channel 2, and nothing on any other step. This class
wraps the matching Receiver and reports, per step, whether a pulse arrived.

This is the ATLAS process's sole source of ground-hit truth: the FSM no longer
infers landings from the track estimate (see
docs/superpowers/specs/2026-05-21-attacker-ground-hit-cue-design.md and
ADR-0009 for the radio-cue pattern).
"""
import struct


class AttackerGroundHitLink:
    """Wraps a Webots ``Receiver`` to consume the attacker's ground-hit pulse.

    The controller enables the receiver (``receiver.enable(timestep_ms)``)
    **before** constructing this object; ``__init__`` does not call ``enable()``
    so the wiring stays in the controller and the class stays testable with a
    stub. Stub surface: ``getQueueLength()``, ``getBytes()``, ``nextPacket()``.

    Args:
        receiver: an already-enabled Webots ``Receiver`` (or compatible stub).
    """

    def __init__(self, receiver):
        self._receiver = receiver
        self._hit_this_step = False
        self.count = 0  # latest received running ground-hit count

    def update(self) -> None:
        """Drain the receiver queue, recording whether a pulse arrived this step.

        Sets ``hit_this_step()`` True if at least one packet was read during
        this call (else False), and stores the latest decoded count. Each packet
        is 4 bytes — ``struct.pack("i", count)``.
        """
        self._hit_this_step = False
        while self._receiver.getQueueLength() > 0:
            (count,) = struct.unpack("i", self._receiver.getBytes())
            self.count = count
            self._hit_this_step = True
            self._receiver.nextPacket()

    def hit_this_step(self) -> bool:
        """Return True iff a ground-hit pulse arrived during the most recent update()."""
        return self._hit_this_step
```

- [ ] **Step 4: Run tests to verify they pass**

Run: `python -m pytest tests/test_attacker_ground_hit_link.py -v`
Expected: PASS (5 passed).

- [ ] **Step 5: Commit**

```bash
git add controllers/atlas_controller/attacker_ground_hit_link.py controllers/atlas_controller/tests/test_attacker_ground_hit_link.py
git commit -m "feat(atlas): AttackerGroundHitLink consumes ground-hit pulse"
```

---

## Task 2: FSM — emission is the only ground-hit signal

**Files:**
- Modify: `controllers/atlas_controller/tests/stubs.py`
- Modify: `controllers/atlas_controller/tests/test_fsm.py`
- Modify: `controllers/atlas_controller/fsm.py`

- [ ] **Step 1: Add the stub link**

In `controllers/atlas_controller/tests/stubs.py`, append after `StubCueLink`:

```python
class StubGroundHitLink:
    """Test double for AttackerGroundHitLink.

    ``hit_this_step()`` returns the configured flag (default False).
    ``update()`` is a no-op. Set ``.hit`` between steps to simulate a pulse.
    """

    def __init__(self, hit=False, count=0):
        self.hit = hit
        self.count = count

    def update(self):
        pass

    def hit_this_step(self):
        return self.hit
```

- [ ] **Step 2: Thread the stub through every SensorSuite construction**

In `controllers/atlas_controller/tests/test_fsm.py`:

1. Update the import on line 10 to include the stub:

```python
from stubs import StubFCR, StubCueLink, StubTrackFilter, StubMotor, StubGroundHitLink
```

2. Add `ground_hit_link=StubGroundHitLink(),` to **every** `SensorSuite(...)` construction. There are 6 call sites — at (approx) lines 59, 493, 599, 691, 801, 894. Each currently begins:

```python
    sensors = SensorSuite(
        cue_link=StubCueLink(...),
        fcr=StubFCR(...),
        track_filter=track_filter,
        ballistic_predictor=...,
    )
```

Add the field, e.g.:

```python
    sensors = SensorSuite(
        cue_link=StubCueLink(),
        fcr=StubFCR(position=[1.0, 1.0, 1.0]),
        track_filter=track_filter,
        ballistic_predictor=predictor,
        ground_hit_link=StubGroundHitLink(),
    )
```

3. Give `_make_fsm_in_engaging` a `ground_hit` parameter so engage tests can drive the cue. Change its signature and the `SensorSuite` it builds:

```python
def _make_fsm_in_engaging(
    intercept=None,
    track_position=None,
    max_range=10.0,
    ground_threshold=0.1,
    ground_hit=False,
):
```

and within it:

```python
    sensors = SensorSuite(
        cue_link=StubCueLink(),
        fcr=StubFCR(position=[1.0, 1.0, 1.0]),
        track_filter=track_filter,
        ballistic_predictor=None,
        ground_hit_link=StubGroundHitLink(hit=ground_hit),
    )
```

- [ ] **Step 3: Add the field to SensorSuite, then run tests (still green)**

In `controllers/atlas_controller/fsm.py`, add the field to `SensorSuite` (after `ballistic_predictor`):

```python
@dataclass
class SensorSuite:
    """All sensing and processing components (Sense + Think layers).

    Injected into AtlasFSM at construction. Swap out in tests by substituting
    stub implementations for each field.
    """

    cue_link: object  # SearchRadarLink — world-frame cues over the radio link
    fcr: object  # FireControlRadar
    track_filter: object  # TrackFilter
    ballistic_predictor: object  # BallisticTrajectoryPredictor
    ground_hit_link: object  # AttackerGroundHitLink — attacker ground-hit pulse
```

Run: `python -m pytest tests/test_fsm.py -q`
Expected: PASS — behaviour is unchanged so far; the new field is just present and constructed everywhere.

- [ ] **Step 4: Rework the affected tests (red)**

In `controllers/atlas_controller/tests/test_fsm.py`:

1. **Delete** `test_predict_below_ground_intercept_transitions_to_track` entirely (the guard it covers is being removed).

2. **Replace** `test_engage_transitions_to_reset_when_target_hits_ground` with a cue-driven version, and add a "no cue stays engaged" test:

```python
def test_engage_transitions_to_reset_on_ground_hit_cue():
    """ENGAGING must transition to RESET when the attacker's ground-hit cue fires,
    even though the target is still in range and above ground."""
    fsm, _, _ = _make_fsm_in_engaging(
        track_position=[1.0, 2.0, 1.0],  # in range, above ground
        ground_hit=True,
    )
    fsm.step()
    assert fsm.state == AtlasFSM.RESET


def test_engage_stays_engaged_for_low_target_without_cue():
    """ENGAGING must NOT exit on a low/descending target by itself: ground hits
    come only from the emitted cue, never from the track Z estimate."""
    fsm, _, _ = _make_fsm_in_engaging(
        track_position=[1.0, 1.0, 0.0],  # at the floor, but no cue
        ground_hit=False,
    )
    fsm.step()
    assert fsm.state == AtlasFSM.ENGAGING
```

Run: `python -m pytest tests/test_fsm.py -k "engage or predict" -v`
Expected: `test_engage_transitions_to_reset_on_ground_hit_cue` FAILS (stays ENGAGING under old code) and `test_engage_stays_engaged_for_low_target_without_cue` FAILS (old code RESETs on low Z). Other engage/predict tests pass.

- [ ] **Step 5: Implement the FSM changes**

In `controllers/atlas_controller/fsm.py`:

1. Replace `_target_in_range` with a range-only `_target_within_range`:

```python
    def _target_within_range(self, rel_position: list[float]) -> bool:
        """Return True if the target is within engagement range.

        A target/intercept is in range when its Euclidean distance from the
        turret is <= config.max_range (metres). Ground reasoning deliberately
        lives nowhere in the FSM: a ground hit is signalled only by the
        attacker's emitted cue (ground_hit_link), never inferred from Z. See
        the 2026-05-21 attacker-ground-hit-cue spec.

        Args:
            rel_position: Relative [dx, dy, dz] from turret origin (metres).
        """
        dx, dy, dz = rel_position
        distance = math.sqrt(dx * dx + dy * dy + dz * dz)
        return distance <= self.config.max_range
```

2. In `_do_predict`, change the validity check to the range-only predicate (and update the docstring's "above ground_threshold" wording):

```python
        intercept = self.sensors.ballistic_predictor.get_intercept(
            self.config.lookahead_steps
        )
        if self._target_within_range(intercept):
            self._intercept = intercept
            self._transition(self.AIMING)
        else:
            self._transition(self.TRACK)
```

3. In `_do_engage`, drive the exit from the cue OR the range envelope:

```python
        self.laser_active = True

        # Hold aim on the fixed intercept
        self._aim_at(self._intercept)

        # Exit on the authoritative ground-hit cue from the attacker, or when
        # the target leaves the range envelope. Ground hits are NEVER inferred
        # from the track Z here — the emitted cue is the only ground-hit signal.
        target_position = self.sensors.track_filter.get_position()
        if self.sensors.ground_hit_link.hit_this_step() or not self._target_within_range(
            target_position
        ):
            self._transition(self.RESET)
```

Also update the `_do_engage` docstring line about exiting "out of range or hits ground" to: "Transitions to RESET when the attacker's ground-hit cue fires or the target leaves max_range." Leave `FSMConfig.ground_threshold` in place (telemetry reads it); update its comment to note it no longer drives FSM logic, e.g. `ground_threshold: float = 0.1  # metres world-Z — telemetry readout only; not used by FSM logic`.

- [ ] **Step 6: Run the FSM tests**

Run: `python -m pytest tests/test_fsm.py -v`
Expected: PASS (including the two reworked engage tests; the deleted predict test is gone).

- [ ] **Step 7: Commit**

```bash
git add controllers/atlas_controller/fsm.py controllers/atlas_controller/tests/stubs.py controllers/atlas_controller/tests/test_fsm.py
git commit -m "refactor(atlas fsm): ground hits come only from the attacker cue"
```

---

## Task 3: Wire the receiver into the ATLAS controller

**Files:**
- Modify: `controllers/atlas_controller/atlas_controller.py`

(Glue layer over the Webots Supervisor API — covered by the unit-tested
`AttackerGroundHitLink`, consistent with how `SearchRadarLink` wiring is
unit-tested but its controller glue is not.)

- [ ] **Step 1: Import the link**

Near the existing `from search_radar_link import SearchRadarLink` (line ~40), add:

```python
from attacker_ground_hit_link import AttackerGroundHitLink
```

- [ ] **Step 2: Get and enable the receiver**

After the FCR-cue receiver block (lines ~89–92), add:

```python
# Receiver for the attacker's ground-hit pulse (channel 2). Enabled before the
# link is constructed, like the Search Radar receiver above.
ground_hit_receiver = robot.getDevice("ATTACKER_GROUND_HIT_RECEIVER")
ground_hit_receiver.enable(timestep)
```

- [ ] **Step 3: Build the link near `cue_link`**

After `cue_link = SearchRadarLink(receiver)` (line ~114), add:

```python
ground_hit_link = AttackerGroundHitLink(ground_hit_receiver)
```

- [ ] **Step 4: Pass it into the SensorSuite**

Add the field to the controller's `SensorSuite(...)` (line ~130):

```python
sensors = SensorSuite(
    cue_link=cue_link,
    fcr=fcr,
    track_filter=track_filter,
    ballistic_predictor=ballistic_predictor,
    ground_hit_link=ground_hit_link,
)
```

- [ ] **Step 5: Drain it in the sense phase and log hits**

In the main loop's sense phase, right after `cue_link.update()` (line ~177):

```python
    cue_link.update()
    ground_hit_link.update()
    if ground_hit_link.hit_this_step():
        log.info(
            "[ATLAS] ground-hit cue received: hit #%d at t=%.2fs",
            ground_hit_link.count,
            robot.getTime(),
        )
```

- [ ] **Step 6: Byte-compile to catch syntax errors**

Run: `python -m py_compile controllers/atlas_controller/atlas_controller.py`
Expected: no output (exit 0). (Webots is required to actually run the controller; py_compile validates syntax without it.)

- [ ] **Step 7: Commit**

```bash
git add controllers/atlas_controller/atlas_controller.py
git commit -m "feat(atlas): consume attacker ground-hit cue in the controller loop"
```

---

## Task 4: Emit the ground-hit pulse from the attacker

**Files:**
- Modify: `controllers/attacker_controller/attacker_controller.py`

(Glue over the Supervisor/Emitter API — mirrors the un-unit-tested
`SR_CUE_EMITTER` wiring in `search_radar_controller.py`. `Projectile` is
unchanged.)

- [ ] **Step 1: Import struct**

At the top of `controllers/attacker_controller/attacker_controller.py`, add a stdlib import (above the `from controller import Supervisor` line):

```python
import struct
```

- [ ] **Step 2: Get the emitter**

After `timestep = int(robot.getBasicTimeStep())` (line ~27), add:

```python
emitter = robot.getDevice("ATTACKER_EMITTER")
if emitter is None:
    raise RuntimeError("[ATTACKER] Could not find device ATTACKER_EMITTER.")
```

- [ ] **Step 3: Emit in the ground-hit seam**

In `on_ground_hit(count)`, after the existing `log.info(...)` call, add the send:

```python
    # Emit the authoritative ground-hit pulse (channel 2, one 4-byte int =
    # running ground-hit count). This is ATLAS's only source of ground-hit
    # truth — see docs/superpowers/specs/2026-05-21-attacker-ground-hit-cue-design.md.
    emitter.send(struct.pack("i", count))
```

- [ ] **Step 4: Byte-compile to catch syntax errors**

Run: `python -m py_compile controllers/attacker_controller/attacker_controller.py`
Expected: no output (exit 0).

- [ ] **Step 5: Commit**

```bash
git add controllers/attacker_controller/attacker_controller.py
git commit -m "feat(attacker): emit ground-hit pulse on floor contact"
```

---

## Final verification

- [ ] **Run the full atlas_controller test suite**

Run (from `controllers/atlas_controller`): `python -m pytest -q`
Expected: all pass, including the new link tests and reworked FSM tests.

- [ ] **Confirm no stray ground-Z inference remains in the FSM**

Run: `grep -n "ground_threshold" controllers/atlas_controller/fsm.py`
Expected: only the `FSMConfig` field definition/comment and the `lookahead_steps` explanatory comment — **no** use inside `_do_predict`, `_do_engage`, or any `_target_*` method.
```
