# Engage-fire Bullet — Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** On entering the FSM `ENGAGING` state, the turret fires a small, fast, *recycled* bullet at the predicted intercept; the bullet destroys the incoming projectile, and the engagement resets.

**Architecture:** The `Projectile` already senses mid-air contact (the single source of hit truth, #32). The FSM emits a one-shot **fire command** on `→ ENGAGING` (stays pure). A new `Bullet` lifecycle class recycles one pre-placed `DEF ATLAS_BULLET` Solid (park → fire → park), driven by the `atlas_controller` supervisor. The attacker emits a **bullet-hit pulse** on channel 3; a new `BulletHitLink` (clone of `AttackerGroundHitLink`) carries it to the FSM, which adds a "projectile destroyed → RESET" exit. No controller, sensor, or radio lives on the bullet — Webots forbids a non-supervisor node moving itself or sharing a TouchSensor cross-process.

**Tech Stack:** Python 3.13 (uv: `.python-version`, `pyproject.toml`), pytest, Webots R2025a. No new dependencies.

**Spec:** `docs/superpowers/specs/2026-05-22-engage-fire-bullet-design.md`.

---

## How to work this plan

- **Execution mode: subagent-driven.** An Opus orchestrator dispatches one fresh subagent per task, reviews (spec compliance, then code quality), then dispatches the next.
- Run atlas tests from `controllers/atlas_controller/`: `python -m pytest tests/ -v`. Run attacker tests from `controllers/attacker_controller/`. Each `conftest.py` puts the controller dir + `tests/` + `lib/` on `sys.path`.
- **Webots glue is not unit-tested** — `atlas_controller.py`, the `.proto`/`.wbt` files match the project's existing untested-glue pattern; they get explicit manual-verification steps. Proto/world *structure* is covered by `tests/test_world_proto_layout.py`.
- Commit after every task with a conventional-commit message ending:
  `Co-Authored-By: Claude Opus 4.7 <noreply@anthropic.com>`

### Model · Effort policy
- **haiku** — fully specified, mechanical. **sonnet** — logic within a clear spec / Webots syntax. **opus** — judgment, cross-cutting, tuning.
- **Webots parameter tuning (`MUZZLE_SPEED`, `MUZZLE_OFFSET_M`, bullet radius, lifetime, the lead/flight-time co-tune) is never delegated to a cheap model** — orchestrator (Opus) or human tunes it in Webots.

---

## File structure

| File | Status | Responsibility |
|---|---|---|
| `controllers/atlas_controller/fsm.py` | modify | Fire command (set on `→ENGAGING`, `consume_fire_command()`); bullet-hit RESET exit; `bullet_hit_link` in `SensorSuite`. |
| `controllers/atlas_controller/tests/test_fsm.py` | modify | Fire-command + bullet-hit-RESET tests. |
| `controllers/atlas_controller/tests/stubs.py` | modify | Add `StubBulletHitLink`. |
| `controllers/atlas_controller/bullet_hit_link.py` | create | `BulletHitLink` receiver wrapper (channel 3). |
| `controllers/atlas_controller/tests/test_bullet_hit_link.py` | create | Unit tests (receiver stub). |
| `controllers/atlas_controller/bullet.py` | create | `Bullet` recycle lifecycle class. |
| `controllers/atlas_controller/tests/test_bullet.py` | create | Unit tests (node stub). |
| `protos/AtlasBullet.proto` | create | The recycled `Solid` bullet (currently `# PLACEHOLDER`). |
| `protos/Attacker.proto` | modify | Add `ATTACKER_BULLET_EMITTER` (channel 3). |
| `protos/AtlasTurret.proto` | modify | Add `ATLAS_BULLET_HIT_RECEIVER` (channel 3). |
| `worlds/ATLA_v1.wbt` | modify | Pre-place `DEF ATLAS_BULLET AtlasBullet`; add `ContactProperties`. |
| `controllers/atlas_controller/tests/test_world_proto_layout.py` | modify | Regression assertions for the bullet proto + world wiring. |
| `controllers/attacker_controller/attacker_controller.py` | modify | Emit bullet-hit pulse on channel 3 from `on_bullet_hit`. |
| `controllers/atlas_controller/atlas_controller.py` | modify | Wire bullet + ch-3 receiver + `BulletHitLink`; fire + recycle in the loop. |
| `docs/adr/0013-engage-fire-recycled-bullet.md` | create | Records the decision. |
| `CONTEXT.md` | modify | Rewrite the `Bullet` glossary entry. |

---

## Task 1 — FSM fire command

**Model: sonnet · Effort: medium** — small, well-bounded, unit-tested state-machine change.

**Files:**
- Modify: `controllers/atlas_controller/fsm.py`
- Test: `controllers/atlas_controller/tests/test_fsm.py`

**Required:** The controller must know the exact moment the FSM commits to a shot and where to aim it, signalled *once* per engagement. The FSM must spawn nothing itself.

**Current state:** `fsm.py` — `_transition()` resets per-state bookkeeping for the entered state (has branches for IDLE/AIM/TRACK_PREDICT). `_do_track_predict()` sets `self._intercept` then calls `_transition(self.ENGAGING)`. `__init__` initialises TRACK_PREDICT bookkeeping including `self._intercept = None`. `_do_reset()` clears `_target`, `_intercept`, etc.

- [ ] **Step 1: Write the failing tests** — append to `controllers/atlas_controller/tests/test_fsm.py`:

```python
# ---------------------------------------------------------------------------
# Fire command — emitted once on entering ENGAGING
# ---------------------------------------------------------------------------

def _force_engage(fsm, intercept):
    """Set a validated intercept and transition straight into ENGAGING.

    Mirrors what _do_track_predict does at the convergence gate: _intercept is
    set, then _transition(ENGAGING) is taken. Calling _transition directly keeps
    the test focused on the fire-command behaviour, not the convergence logic.
    """
    fsm._intercept = list(intercept)
    fsm._transition(AtlasFSM.ENGAGING)


def test_no_fire_command_before_engaging():
    """fire command is None until the FSM enters ENGAGING."""
    fsm, _, _ = _build_fsm()
    assert fsm.consume_fire_command() is None


def test_entering_engaging_sets_fire_command_to_intercept():
    """The → ENGAGING transition records the intercept as the fire command."""
    fsm, _, _ = _build_fsm()
    _force_engage(fsm, [2.0, 3.0, 1.5])
    assert fsm.state == AtlasFSM.ENGAGING
    assert fsm.consume_fire_command() == [2.0, 3.0, 1.5]


def test_consume_fire_command_returns_then_clears():
    """consume_fire_command returns the command once, then None."""
    fsm, _, _ = _build_fsm()
    _force_engage(fsm, [2.0, 3.0, 1.5])
    assert fsm.consume_fire_command() == [2.0, 3.0, 1.5]
    assert fsm.consume_fire_command() is None


def test_fire_command_is_a_copy_not_the_intercept_alias():
    """The fire command must not alias the FSM's internal _intercept list."""
    fsm, _, _ = _build_fsm()
    _force_engage(fsm, [2.0, 3.0, 1.5])
    cmd = fsm.consume_fire_command()
    cmd[0] = 99.0
    assert fsm._intercept == [2.0, 3.0, 1.5]


def test_reset_clears_an_unconsumed_fire_command():
    """A fire command never survives a RESET."""
    fsm, _, _ = _build_fsm()
    _force_engage(fsm, [2.0, 3.0, 1.5])
    fsm.state = AtlasFSM.RESET
    fsm.step()
    assert fsm.consume_fire_command() is None


def test_re_entering_engaging_emits_a_fresh_fire_command():
    """A second engagement emits its own fire command."""
    fsm, _, _ = _build_fsm()
    _force_engage(fsm, [2.0, 3.0, 1.5])
    assert fsm.consume_fire_command() == [2.0, 3.0, 1.5]
    _force_engage(fsm, [-1.0, 4.0, 0.8])
    assert fsm.consume_fire_command() == [-1.0, 4.0, 0.8]
```

- [ ] **Step 2: Run the tests, confirm they fail**

Run: `cd controllers/atlas_controller && python -m pytest tests/test_fsm.py -k fire -v`
Expected: FAIL — `AttributeError: 'AtlasFSM' object has no attribute 'consume_fire_command'`.

- [ ] **Step 3: Implement** in `fsm.py`:

In `__init__`, in the TRACK_PREDICT bookkeeping block (near `self._intercept = None`), add:

```python
        # One-shot fire command: set to a copy of the intercept [dx, dy, dz]
        # (turret-relative metres) when the FSM enters ENGAGING; returned and
        # cleared by the controller via consume_fire_command(). See ADR-0013.
        self._fire_command = None
```

In `_transition()`, add a branch (after the existing `elif new_state == self.TRACK_PREDICT:` block):

```python
        elif new_state == self.ENGAGING:
            # Commit the shot: hand the validated intercept to the controller
            # exactly once. _intercept is guaranteed set — TRACK_PREDICT sets it
            # immediately before taking this transition.
            self._fire_command = list(self._intercept)
```

Add the method (place it near `commanded_aim`, in the public surface):

```python
    def consume_fire_command(self):
        """Return the pending fire command and clear it, or None if none pending.

        The fire command is the intercept point [dx, dy, dz] (turret-relative
        metres) the FSM committed to on entering ENGAGING. One-shot: the
        controller calls this once per step after fsm.step(); it is returned
        exactly once and is None on every subsequent call until the FSM
        re-enters ENGAGING. The FSM spawns nothing — node launch is the
        controller's job (keeps the FSM pure and unit-testable).
        """
        command = self._fire_command
        self._fire_command = None
        return command
```

In `_do_reset()`, add alongside the other explicit clears (e.g. after `self._intercept = None`):

```python
        self._fire_command = None
```

- [ ] **Step 4: Run the full suite, confirm green**

Run: `cd controllers/atlas_controller && python -m pytest tests/ -v`
Expected: PASS — new fire-command tests pass; every existing test stays green (change is additive).

- [ ] **Step 5: Commit**

```bash
git add controllers/atlas_controller/fsm.py controllers/atlas_controller/tests/test_fsm.py
git commit -m "feat(fsm): emit a one-shot fire command on entering ENGAGING"
```

---

## Task 2 — `BulletHitLink` receiver wrapper

**Model: sonnet · Effort: low** — a near-clone of `AttackerGroundHitLink`; structure fully dictated.

**Files:**
- Create: `controllers/atlas_controller/bullet_hit_link.py`
- Test: `controllers/atlas_controller/tests/test_bullet_hit_link.py`

**Required:** ATLAS must learn, per step, that the bullet destroyed the projectile. The attacker emits the bullet-hit pulse on channel 3 (`struct.pack("i", count)`), exactly like the ground-hit pulse on channel 2. This wraps the matching Receiver. Pattern reference: `controllers/atlas_controller/attacker_ground_hit_link.py`.

- [ ] **Step 1: Write the failing tests** — create `controllers/atlas_controller/tests/test_bullet_hit_link.py`:

```python
"""Tests for BulletHitLink — wraps a Webots Receiver to consume the attacker's
bullet-hit pulse (channel 3).

The attacker emits exactly one packet on the step a bullet hit registers:
struct.pack("i", count) on channel 3. The link drains the queue each update()
and reports whether a pulse arrived this step plus the latest count. Mirrors
AttackerGroundHitLink (channel 2).
"""
import struct
from bullet_hit_link import BulletHitLink


class StubReceiver:
    """Test double for a Webots Receiver pre-loaded with packed int packets."""

    def __init__(self, counts=None):
        self._queue = [struct.pack("i", c) for c in (counts or [])]

    def load(self, counts):
        self._queue.extend(struct.pack("i", c) for c in counts)

    def getQueueLength(self):
        return len(self._queue)

    def getBytes(self):
        return self._queue[0]

    def nextPacket(self):
        self._queue.pop(0)


def test_no_hit_before_any_update():
    link = BulletHitLink(StubReceiver())
    assert link.hit_this_step() is False
    assert link.count == 0


def test_empty_queue_reports_no_hit():
    link = BulletHitLink(StubReceiver())
    link.update()
    assert link.hit_this_step() is False
    assert link.count == 0


def test_one_packet_reports_hit_and_count():
    link = BulletHitLink(StubReceiver([1]))
    link.update()
    assert link.hit_this_step() is True
    assert link.count == 1


def test_multiple_packets_drained_keeps_latest_count():
    link = BulletHitLink(StubReceiver([1, 2, 3]))
    link.update()
    assert link.hit_this_step() is True
    assert link.count == 3


def test_hit_flag_clears_on_next_silent_update():
    receiver = StubReceiver([5])
    link = BulletHitLink(receiver)
    link.update()
    assert link.hit_this_step() is True

    link.update()  # queue now empty
    assert link.hit_this_step() is False
    assert link.count == 5  # latest count persists
```

- [ ] **Step 2: Run, confirm fail**

Run: `cd controllers/atlas_controller && python -m pytest tests/test_bullet_hit_link.py -v`
Expected: FAIL — `ModuleNotFoundError: No module named 'bullet_hit_link'`.

- [ ] **Step 3: Implement** — create `controllers/atlas_controller/bullet_hit_link.py`:

```python
"""BulletHitLink — consumes the attacker's bullet-hit pulse from a Webots Receiver.

The attacker emitter sends exactly one packet on the step a bullet hit registers
(the turret bullet struck the projectile mid-air): struct.pack("i", count) (one
4-byte signed int, the running bullet-hit count) on channel 3, and nothing on any
other step. This class wraps the matching Receiver and reports, per step, whether
a pulse arrived.

This is ATLAS's sole source of bullet-hit (projectile-destroyed) truth: the FSM
never infers a destroyed projectile from the track estimate. It mirrors
AttackerGroundHitLink (channel 2) exactly — see the engage-fire bullet design
(docs/superpowers/specs/2026-05-22-engage-fire-bullet-design.md) and ADR-0009 for
the radio-cue pattern.
"""
import struct


class BulletHitLink:
    """Wraps a Webots ``Receiver`` to consume the attacker's bullet-hit pulse.

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
        self.count = 0  # latest received running bullet-hit count

    def update(self) -> None:
        """Drain the receiver queue, recording whether a pulse arrived this step.

        Sets ``hit_this_step()`` True if at least one packet was read during this
        call (else False), and stores the latest decoded count. Each packet is 4
        bytes — ``struct.pack("i", count)``.
        """
        self._hit_this_step = False
        while self._receiver.getQueueLength() > 0:
            (count,) = struct.unpack("i", self._receiver.getBytes())
            self.count = count
            self._hit_this_step = True
            self._receiver.nextPacket()

    def hit_this_step(self) -> bool:
        """Return True iff a bullet-hit pulse arrived during the most recent update()."""
        return self._hit_this_step
```

- [ ] **Step 4: Run, confirm green**

Run: `cd controllers/atlas_controller && python -m pytest tests/test_bullet_hit_link.py -v`
Expected: PASS — all 5 tests pass.

- [ ] **Step 5: Commit**

```bash
git add controllers/atlas_controller/bullet_hit_link.py controllers/atlas_controller/tests/test_bullet_hit_link.py
git commit -m "feat(atlas): add BulletHitLink for the attacker bullet-hit pulse (ch 3)"
```

---

## Task 3 — FSM bullet-hit → RESET exit

**Model: sonnet · Effort: medium** — wires a new cue into the FSM; touches the shared `SensorSuite` and test helper, so the interface must stay consistent.

**Files:**
- Modify: `controllers/atlas_controller/fsm.py`
- Modify: `controllers/atlas_controller/tests/stubs.py`
- Test: `controllers/atlas_controller/tests/test_fsm.py`

**Required:** `_do_engage` has a TODO to exit on a projectile-destroyed cue. The bullet-hit cue (via `BulletHitLink`) is that signal. Like the ground-hit cue, it means "engagement over → RESET", and must be honoured in every state so a late pulse can't strand the FSM. The FSM never learns *which* body was struck — both cues share the RESET exit.

**Current state:** `SensorSuite` is a dataclass with fields `cue_link, fcr, track_filter, ballistic_predictor, ground_hit_link`. `_do_idle`, `_do_aim`, `_do_track_predict`, `_do_engage` each begin with `if self.sensors.ground_hit_link.hit_this_step(): self._transition(self.RESET); return` (engage combines it with the range check). `stubs.py` has `StubGroundHitLink`. `test_fsm.py::_build_fsm` builds `SensorSuite(...)` without a bullet-hit link.

- [ ] **Step 1: Add `StubBulletHitLink` to `controllers/atlas_controller/tests/stubs.py`** (append, mirroring `StubGroundHitLink`):

```python
class StubBulletHitLink:
    """Test double for BulletHitLink.

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

- [ ] **Step 2: Write the failing tests** — append to `controllers/atlas_controller/tests/test_fsm.py`:

```python
# ---------------------------------------------------------------------------
# Bullet-hit cue → RESET (projectile destroyed)
# ---------------------------------------------------------------------------

from stubs import StubBulletHitLink  # noqa: E402  (grouped with the new tests)


def test_bullet_hit_in_engaging_transitions_to_reset():
    """A bullet-hit pulse while ENGAGING ends the engagement → RESET."""
    bullet_hit_link = StubBulletHitLink(hit=True)
    fsm, _, _ = _build_fsm(bullet_hit_link=bullet_hit_link)
    fsm._intercept = [1.0, 1.0, 1.0]
    fsm._transition(AtlasFSM.ENGAGING)
    fsm.consume_fire_command()  # clear the fire command set on entry
    fsm.step()
    assert fsm.state == AtlasFSM.RESET


def test_bullet_hit_in_track_predict_transitions_to_reset():
    """A bullet-hit pulse takes precedence in TRACK_PREDICT → RESET."""
    bullet_hit_link = StubBulletHitLink(hit=True)
    fsm, _, _ = _build_fsm(
        bullet_hit_link=bullet_hit_link,
        ballistic_predictor=StubPredictor([1.0, 1.0, 1.0]),
    )
    fsm.state = AtlasFSM.TRACK_PREDICT
    fsm.step()
    assert fsm.state == AtlasFSM.RESET


def test_no_bullet_hit_keeps_engaging():
    """With no bullet hit (and in range), ENGAGING holds (regression guard)."""
    fsm, _, _ = _build_fsm(bullet_hit_link=StubBulletHitLink(hit=False))
    fsm._intercept = [1.0, 1.0, 1.0]
    fsm._transition(AtlasFSM.ENGAGING)
    fsm.consume_fire_command()
    fsm.step()
    assert fsm.state == AtlasFSM.ENGAGING
```

Also update the `_build_fsm` helper in `test_fsm.py` to accept and wire a bullet-hit link. Add the parameter and default, and pass it into `SensorSuite`:

```python
    bullet_hit_link=None,
```
(add to the `_build_fsm` signature, alongside `ground_hit_link=None`), then after the `ground_hit_link` default block add:

```python
    if bullet_hit_link is None:
        bullet_hit_link = StubBulletHitLink()
```

and add `bullet_hit_link=bullet_hit_link,` to the `SensorSuite(...)` call. Add `StubBulletHitLink` to the existing `from stubs import ...` line at the top of the file (so the helper can default it) — remove the local re-import in the test block if it duplicates.

- [ ] **Step 3: Run, confirm fail**

Run: `cd controllers/atlas_controller && python -m pytest tests/test_fsm.py -k "bullet_hit" -v`
Expected: FAIL — `TypeError: __init__() missing ... 'bullet_hit_link'` (SensorSuite has no such field) / the new exits not taken.

- [ ] **Step 4: Implement** in `fsm.py`:

Add the field to `SensorSuite` (after `ground_hit_link`):

```python
    bullet_hit_link: object  # BulletHitLink — attacker bullet-hit (projectile-destroyed) pulse
```

In each of `_do_idle`, `_do_aim`, `_do_track_predict`, replace the ground-hit precedence guard:

```python
        if self.sensors.ground_hit_link.hit_this_step():
            self._transition(self.RESET)
            return
```
with:

```python
        if (
            self.sensors.ground_hit_link.hit_this_step()
            or self.sensors.bullet_hit_link.hit_this_step()
        ):
            self._transition(self.RESET)
            return
```

In `_do_engage`, extend the exit condition to include the bullet-hit cue:

```python
        target_position = self.sensors.track_filter.get_position()
        if (
            self.sensors.ground_hit_link.hit_this_step()
            or self.sensors.bullet_hit_link.hit_this_step()
            or not self._target_within_range(target_position)
        ):
            self._transition(self.RESET)
```

Update the `_do_engage` docstring TODO line (`TODO(projectile-destroyed-cue): ...`) — replace it with a sentence noting the bullet-hit cue now provides that exit.

- [ ] **Step 5: Run the full suite, confirm green**

Run: `cd controllers/atlas_controller && python -m pytest tests/ -v`
Expected: PASS — new bullet-hit tests pass; existing tests stay green (the `_build_fsm` helper now injects a default `StubBulletHitLink`).

- [ ] **Step 6: Commit**

```bash
git add controllers/atlas_controller/fsm.py controllers/atlas_controller/tests/stubs.py controllers/atlas_controller/tests/test_fsm.py
git commit -m "feat(fsm): RESET on the bullet-hit (projectile-destroyed) cue"
```

---

## Task 4 — `Bullet` recycle lifecycle class

**Model: sonnet · Effort: medium** — pure logic over a node interface; mirrors `Projectile`.

**Files:**
- Create: `controllers/atlas_controller/bullet.py`
- Test: `controllers/atlas_controller/tests/test_bullet.py`

**Required:** A single recycled bullet, driven by the supervisor. `fire(intercept_rel)` teleports it to the muzzle and launches it toward the intercept; `recycle()` parks it. The same `setVelocity`/`resetPhysics` discipline as `Projectile` (never reset physics in the same step as launch). Node interface mirrors `StubProjectileNode`: `getField("translation").setSFVec3f`, `setVelocity`, `resetPhysics`.

**Reference:** `controllers/attacker_controller/projectile.py` and `controllers/attacker_controller/tests/attacker_stubs.py`.

- [ ] **Step 1: Write the failing tests** — create `controllers/atlas_controller/tests/test_bullet.py`:

```python
"""Unit tests for the turret Bullet recycle lifecycle.

The Bullet is one pre-placed Solid the atlas_controller supervisor recycles:
fire() teleports it to the muzzle and launches it toward the intercept;
recycle() zeroes motion and parks it. Tests use a node stub mirroring the
attacker's StubProjectileNode (getField('translation'), setVelocity,
resetPhysics).
"""
import math
from bullet import Bullet, BulletConfig


class StubField:
    def __init__(self):
        self.value = None

    def setSFVec3f(self, value):
        self.value = list(value)


class StubBulletNode:
    def __init__(self):
        self.translation_field = StubField()
        self.set_velocity_calls = []
        self.reset_physics_calls = 0

    def getField(self, name):
        assert name == "translation"
        return self.translation_field

    def setVelocity(self, velocity):
        self.set_velocity_calls.append(list(velocity))

    def resetPhysics(self):
        self.reset_physics_calls += 1


TURRET = [0.0, 0.0, 0.0]


def _make(config=None, turret_position=None):
    node = StubBulletNode()
    bullet = Bullet(
        node,
        turret_position=turret_position or TURRET,
        config=config or BulletConfig(),
    )
    return node, bullet


def test_starts_parked():
    _, bullet = _make()
    assert bullet.is_parked is True
    assert bullet.age_steps == 0


def test_fire_teleports_to_muzzle_along_aim_direction():
    # Intercept straight along +Y at 4 m; muzzle offset 0.6 m → spawn at y=0.6.
    config = BulletConfig(muzzle_speed=10.0, muzzle_offset_m=0.6)
    node, bullet = _make(config)

    bullet.fire([0.0, 4.0, 0.0])

    assert node.translation_field.value == [0.0, 0.6, 0.0]
    # Velocity = unit(intercept) * muzzle_speed, zero angular.
    assert node.set_velocity_calls[-1] == [0.0, 10.0, 0.0, 0.0, 0.0, 0.0]
    assert bullet.is_parked is False


def test_fire_offsets_from_turret_world_position():
    # Turret not at origin: spawn = turret + unit(intercept) * offset.
    config = BulletConfig(muzzle_speed=10.0, muzzle_offset_m=1.0)
    node, bullet = _make(config, turret_position=[5.0, 0.0, 2.0])

    bullet.fire([0.0, 0.0, 3.0])  # straight up

    assert node.translation_field.value == [5.0, 0.0, 3.0]


def test_fire_does_not_reset_physics_in_same_step_as_launch():
    # Regression: resetPhysics() with setVelocity() in one step zeroes the launch.
    node, bullet = _make()
    bullet.fire([1.0, 1.0, 1.0])
    assert node.reset_physics_calls == 0


def test_recycle_zeroes_motion_parks_and_resets_physics():
    config = BulletConfig(park_position=[0.0, 0.0, -100.0])
    node, bullet = _make(config)
    bullet.fire([0.0, 4.0, 0.0])

    bullet.recycle()

    assert node.set_velocity_calls[-1] == [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    assert node.translation_field.value == [0.0, 0.0, -100.0]
    assert node.reset_physics_calls == 1
    assert bullet.is_parked is True
    assert bullet.age_steps == 0


def test_step_ages_only_in_flight():
    _, bullet = _make()
    bullet.step()
    assert bullet.age_steps == 0  # parked → no aging

    bullet.fire([0.0, 4.0, 0.0])
    bullet.step()
    bullet.step()
    assert bullet.age_steps == 2

    bullet.recycle()
    bullet.step()
    assert bullet.age_steps == 0  # parked again → reset and not aging


def test_fire_normalises_arbitrary_intercept():
    config = BulletConfig(muzzle_speed=12.0, muzzle_offset_m=0.0)
    node, bullet = _make(config)

    bullet.fire([3.0, 4.0, 0.0])  # |v| = 5

    vx, vy, vz = node.set_velocity_calls[-1][:3]
    speed = math.sqrt(vx * vx + vy * vy + vz * vz)
    assert abs(speed - 12.0) < 1e-9
    assert abs(vx - 12.0 * 3 / 5) < 1e-9
    assert abs(vy - 12.0 * 4 / 5) < 1e-9
```

- [ ] **Step 2: Run, confirm fail**

Run: `cd controllers/atlas_controller && python -m pytest tests/test_bullet.py -v`
Expected: FAIL — `ModuleNotFoundError: No module named 'bullet'`.

- [ ] **Step 3: Implement** — create `controllers/atlas_controller/bullet.py`:

```python
"""Turret Bullet — a single recycled projectile the turret fires on ENGAGING.

The bullet is one pre-placed Solid (DEF ATLAS_BULLET); the atlas_controller
supervisor recycles it rather than spawning/deleting a node each shot, mirroring
the incoming Projectile (ADR-0011): a stable node handle, no per-shot controller
process, and a unit-testable lifecycle. See
docs/superpowers/specs/2026-05-22-engage-fire-bullet-design.md and ADR-0013.

Two states:
    PARKED    — out of range, waiting to fire
    IN_FLIGHT — launched, travelling toward the intercept

The Bullet owns no hit decision: it is recycled by the controller when the
bullet-hit or ground-hit cue arrives, or a flight-time safety timeout fires.

setVelocity / resetPhysics discipline (same as Projectile): resetPhysics() zeroes
velocity in Webots, so fire() must NOT call it (it would kill the launch);
recycle() zeroes velocity first, then resetPhysics() on the parked body.

Node-agnostic: takes anything exposing setVelocity / getField / resetPhysics, so
it is unit-tested against a stub without Webots.
"""

import math
from dataclasses import dataclass, field
from enum import Enum, auto


@dataclass
class BulletConfig:
    """Tunable parameters for the bullet. Webots-tuned values; see the spec.

    muzzle_speed: launch speed (m/s). Capped below the tunnelling limit
                  v_max < (r_ball + r_bullet) / timestep (~20 m/s at 32 ms).
    muzzle_offset_m: spawn this far along the aim direction so the bullet starts
                  clear of the turret's bounding box.
    park_position: where the bullet rests while PARKED — far out of sensor range
                  (below the floor), like the despawned Projectile.
    """

    muzzle_speed: float = 18.0
    muzzle_offset_m: float = 0.6
    park_position: list = field(default_factory=lambda: [0.0, 0.0, -100.0])


class _State(Enum):
    PARKED = auto()
    IN_FLIGHT = auto()


class Bullet:
    """Lifecycle of the single recycled turret bullet."""

    def __init__(self, node, turret_position, config: BulletConfig | None = None):
        """
        Args:
            node:            Webots node handle for the DEF ATLAS_BULLET Solid.
            turret_position: world-frame [x, y, z] of the turret origin (the
                             muzzle reference; the bullet launches from an offset
                             along the aim direction).
            config:          Tunable parameters. Defaults to BulletConfig().
        """
        self.config = config if config is not None else BulletConfig()
        self._node = node
        self._translation = node.getField("translation")
        self._turret = list(turret_position)
        self._state = _State.PARKED
        self.age_steps = 0  # timesteps since launch (IN_FLIGHT only)

    @property
    def is_parked(self) -> bool:
        return self._state is _State.PARKED

    def fire(self, intercept_rel) -> None:
        """Teleport to the muzzle and launch toward the intercept (enter IN_FLIGHT).

        Args:
            intercept_rel: turret-relative [dx, dy, dz] aim point (metres), the
                           FSM fire command. The launch direction is its unit
                           vector; speed is config.muzzle_speed.

        Does NOT call resetPhysics() — that would zero the launch velocity in the
        same step (see module docstring). The body was reset on the previous
        recycle().
        """
        mag = math.sqrt(sum(c * c for c in intercept_rel)) or 1.0
        direction = [c / mag for c in intercept_rel]

        spawn = [
            self._turret[i] + direction[i] * self.config.muzzle_offset_m
            for i in range(3)
        ]
        self._translation.setSFVec3f(spawn)
        self._node.setVelocity(
            [direction[i] * self.config.muzzle_speed for i in range(3)]
            + [0.0, 0.0, 0.0]
        )
        self.age_steps = 0
        self._state = _State.IN_FLIGHT

    def recycle(self) -> None:
        """Stop the bullet, park it out of range, reset physics (enter PARKED).

        Order matters: zero the velocity, teleport to the park position, THEN
        resetPhysics() — the body is already at zero so reset is a clean no-op on
        velocity and clears residual ODE inertia before the next flight.
        """
        self._node.setVelocity([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        self._translation.setSFVec3f(self.config.park_position)
        self._node.resetPhysics()
        self.age_steps = 0
        self._state = _State.PARKED

    def step(self) -> None:
        """Advance one tick: age the bullet while IN_FLIGHT (for the timeout)."""
        if self._state is _State.IN_FLIGHT:
            self.age_steps += 1
```

- [ ] **Step 4: Run, confirm green**

Run: `cd controllers/atlas_controller && python -m pytest tests/test_bullet.py -v`
Expected: PASS — all tests pass.

- [ ] **Step 5: Commit**

```bash
git add controllers/atlas_controller/bullet.py controllers/atlas_controller/tests/test_bullet.py
git commit -m "feat(atlas): add the recycled Bullet lifecycle class"
```

---

## Task 5 — `AtlasBullet.proto` + world wiring + layout regression test

**Model: sonnet · Effort: medium** — Webots PROTO/`.wbt` syntax. Inspect `protos/Projectile.proto` (Solid sphere body) and `worlds/ATLA_v1.wbt` first.

**Files:**
- Modify (replace placeholder): `protos/AtlasBullet.proto`
- Modify: `worlds/ATLA_v1.wbt`
- Test: `controllers/atlas_controller/tests/test_world_proto_layout.py`

**Required:** The bullet needs a node template. It is a passive `Solid` (no controller/sensor/radio), pre-placed once in the world with `DEF ATLAS_BULLET` so the supervisor resolves it via `getFromDef`. `ContactProperties` makes the bullet/ball contact non-bouncy so the ball does not get knocked away before despawn.

**Current state:** `protos/AtlasBullet.proto` is `# PLACEHOLDER`. `worlds/ATLA_v1.wbt` has an `EXTERNPROTO` block (lines 9-12), an empty `WorldInfo {}` (line 14), and DEF instances for PROJECTILE/TURRET_BASE/SEARCH_RADAR/ATTACKER (lines 30-38).

- [ ] **Step 1: Write `protos/AtlasBullet.proto`** (replace the placeholder):

```
#VRML_SIM R2025a utf8
# The turret's fired projectile: a small, fast, recycled sphere.
# A passive Solid (NOT a Robot) — it carries no controller, sensor, or radio.
# The atlas_controller supervisor recycles ONE pre-placed instance
# (DEF ATLAS_BULLET): teleport + setVelocity to fire, teleport to park.
# Hit detection is owned by the incoming Projectile (single source of truth, #32),
# so the bullet needs no sensor of its own. See ADR-0013.

PROTO AtlasBullet [
  field SFVec3f translation 0 0 -100
  field SFString name "ATLAS_BULLET"
  field SFVec3f linearVelocity 0 0 0
] {
  Solid {
    translation IS translation
    children [
      DEF BULLET_SHAPE Shape {
        appearance PBRAppearance {
          baseColor 0 0.4 1
          emissiveColor 0 0.2 0.6
          roughness 0.3
          metalness 0
        }
        geometry Sphere {
          radius 0.15
          subdivision 3
        }
      }
    ]
    name IS name
    model "atlas_bullet"
    contactMaterial "atlas_bullet"
    boundingObject USE BULLET_SHAPE
    physics Physics {
      density -1
      mass 0.05
    }
    linearVelocity IS linearVelocity
  }
}
```

- [ ] **Step 2: Edit `worlds/ATLA_v1.wbt`:**

Add to the `EXTERNPROTO` block (after the `Attacker.proto` line):

```
EXTERNPROTO "../protos/AtlasBullet.proto"
```

Replace the empty `WorldInfo {}` with one carrying the non-bouncy contact pair (the projectile's default `contactMaterial` is `"default"`):

```
WorldInfo {
  contactProperties [
    ContactProperties {
      material1 "atlas_bullet"
      material2 "default"
      bounce 0
      bounceVelocity 0
    }
  ]
}
```

Pre-place one parked instance among the other DEF instances (after `DEF ATTACKER Attacker {}`):

```
DEF ATLAS_BULLET AtlasBullet {
}
```

- [ ] **Step 3: Add the regression test** — append to `controllers/atlas_controller/tests/test_world_proto_layout.py` (match the file's existing `REPO_ROOT` / `WORLD_FILE` constants and style; if a `BULLET_PROTO` path constant is needed, define it near the others):

```python
def test_atlas_bullet_proto_is_a_prebplaced_solid():
    """The bullet PROTO is a passive Solid (no controller/sensor), pre-placed
    once as DEF ATLAS_BULLET, and is the non-bouncy contact material."""
    world = WORLD_FILE.read_text(encoding="utf-8")
    bullet_proto = (REPO_ROOT / "protos" / "AtlasBullet.proto").read_text(encoding="utf-8")

    # Pre-placed instance the supervisor recycles (not dynamically spawned).
    assert "DEF ATLAS_BULLET AtlasBullet" in world
    assert 'EXTERNPROTO "../protos/AtlasBullet.proto"' in world

    # Non-bouncy bullet/ball contact so the ball is not knocked away pre-despawn.
    assert 'material1 "atlas_bullet"' in world
    assert "bounce 0" in world

    # A passive Solid: no controller, no sensor, no Robot wrapper.
    assert "PROTO AtlasBullet" in bullet_proto
    assert "Solid {" in bullet_proto
    assert "Robot {" not in bullet_proto
    assert "controller" not in bullet_proto
    assert "TouchSensor" not in bullet_proto
    assert 'contactMaterial "atlas_bullet"' in bullet_proto
```

- [ ] **Step 4: Run the layout tests, confirm green**

Run: `cd controllers/atlas_controller && python -m pytest tests/test_world_proto_layout.py -v`
Expected: PASS — new test passes; existing layout tests stay green.

- [ ] **Step 5: Manual check (Webots):** Open `worlds/ATLA_v1.wbt` — it loads with no PROTO parse errors; no blue bullet is visible in the arena (it is parked at z=-100). Commit:

```bash
git add protos/AtlasBullet.proto worlds/ATLA_v1.wbt controllers/atlas_controller/tests/test_world_proto_layout.py
git commit -m "feat(world): pre-place a recycled AtlasBullet Solid + non-bouncy contact"
```

---

## Task 6 — Attacker emits the bullet-hit pulse (channel 3)

**Model: sonnet · Effort: low** — a one-line emit mirroring the existing ground-hit emit, plus a proto emitter device.

**Files:**
- Modify: `protos/Attacker.proto`
- Modify: `controllers/attacker_controller/attacker_controller.py`

**Required:** The `Projectile` already fires `on_bullet_hit(count)` on a mid-air contact. Wire that to a channel-3 emitter so ATLAS's `BulletHitLink` receives it — mirroring how `on_ground_hit` emits on channel 2.

**Current state:** `Attacker.proto` has one `Emitter` named `ATTACKER_EMITTER`, `type "radio"`, `channel 2`, `range -1`. `attacker_controller.py` gets `ATTACKER_EMITTER`, and `on_ground_hit` does `emitter.send(struct.pack("i", count))`; `on_bullet_hit` currently only logs.

- [ ] **Step 1: Edit `protos/Attacker.proto`** — add a second emitter inside the `children` list (after the existing `ATTACKER_EMITTER` Emitter):

```
      Emitter {
        name "ATTACKER_BULLET_EMITTER"
        type "radio"
        channel 3
        range -1
      }
```

- [ ] **Step 2: Edit `controllers/attacker_controller/attacker_controller.py`:**

After fetching `ATTACKER_EMITTER`, fetch the bullet emitter:

```python
bullet_emitter = robot.getDevice("ATTACKER_BULLET_EMITTER")
if bullet_emitter is None:
    raise RuntimeError("[ATTACKER] Could not find device ATTACKER_BULLET_EMITTER.")
```

In `on_bullet_hit(count)`, after the existing log call, add the emit (mirroring `on_ground_hit`):

```python
    # Emit the authoritative bullet-hit pulse (channel 3, one 4-byte int =
    # running bullet-hit count). This is ATLAS's only source of
    # projectile-destroyed truth — see the engage-fire bullet design and #32.
    bullet_emitter.send(struct.pack("i", count))
```

- [ ] **Step 3: Run the attacker tests, confirm green** (the `Projectile` logic is unchanged; this verifies nothing regressed):

Run: `cd controllers/attacker_controller && python -m pytest tests/ -v`
Expected: PASS — `Projectile` tests unchanged and green (the emit is in the untested thin controller).

- [ ] **Step 4: Commit**

```bash
git add protos/Attacker.proto controllers/attacker_controller/attacker_controller.py
git commit -m "feat(attacker): emit the bullet-hit pulse on channel 3"
```

---

## Task 7 — Wire & launch the bullet in `atlas_controller` (+ ch-3 receiver)

**Model: sonnet · Effort: medium** for the controller edit. **Tuning (`MUZZLE_SPEED`, `MUZZLE_OFFSET_M`, lifetime, lead/flight-time co-tune) is orchestrator/human in Webots — not delegated.**

**Files:**
- Modify: `protos/AtlasTurret.proto`
- Modify: `controllers/atlas_controller/atlas_controller.py`

**Required:** The supervisor must (a) receive the channel-3 bullet-hit pulse, (b) build the `Bullet` and `BulletHitLink`, add the link to `SensorSuite`, (c) fire on the FSM fire command, and (d) recycle the bullet when the engagement ends (bullet hit, ground hit, or flight timeout).

**Current state:** `atlas_controller.py` builds `fcr`, `cue_link`, `ground_hit_link`, `track_filter`, `ballistic_predictor`, then `SensorSuite(cue_link=..., fcr=..., track_filter=..., ballistic_predictor=..., ground_hit_link=...)` and `AtlasFSM(sensors, hardware)`. The loop drains `cue_link.update()` / `ground_hit_link.update()`, updates the FCR, predicts, fuses, then `fsm.step()`, then telemetry. `turret_position` is the turret world origin. `AtlasTurret.proto` has Receivers `FCR_CUE_RECEIVER` (ch 1) and `ATTACKER_GROUND_HIT_RECEIVER` (ch 2).

- [ ] **Step 1: Edit `protos/AtlasTurret.proto`** — add a third Receiver inside `children` (after `ATTACKER_GROUND_HIT_RECEIVER`):

```
      Receiver {
        name "ATLAS_BULLET_HIT_RECEIVER"
        channel 3
      }
```

- [ ] **Step 2: Edit `controllers/atlas_controller/atlas_controller.py`:**

Add imports (with the other local imports):

```python
from bullet_hit_link import BulletHitLink
from bullet import Bullet, BulletConfig
from scene import DEF_PROJECTILE, DEF_ATLAS_BULLET
```

(If `scene.py` does not yet export `DEF_ATLAS_BULLET`, add `DEF_ATLAS_BULLET = "ATLAS_BULLET"` to `lib/scene.py` with a one-line comment, mirroring the existing constants — do this in this task.)

Add turret-weapon constants near the existing tuning block:

```python
# --- Turret weapon (bullet) ---
# v_max < (r_ball + r_bullet)/timestep ≈ 20 m/s at 32 ms (ODE is discrete — no
# CCD — so a too-fast bullet tunnels through the ball). Tune in Webots.
MUZZLE_SPEED_MPS = 18.0
MUZZLE_OFFSET_M = 0.6           # spawn this far along the aim, clear of the turret
BULLET_PARK_POSITION = [0.0, 0.0, -100.0]
BULLET_MAX_LIFETIME_STEPS = 400  # safety: recycle a bullet that never resolves
```

Enable the channel-3 receiver (with the other receiver setup):

```python
bullet_hit_receiver = robot.getDevice("ATLAS_BULLET_HIT_RECEIVER")
bullet_hit_receiver.enable(timestep)
```

Build the link and the bullet node + lifecycle (near the other sensor construction):

```python
bullet_hit_link = BulletHitLink(bullet_hit_receiver)

bullet_node = robot.getFromDef(DEF_ATLAS_BULLET)
if bullet_node is None:
    raise RuntimeError(f"Could not find DEF {DEF_ATLAS_BULLET} in the world file.")
bullet = Bullet(
    bullet_node,
    turret_position=turret_position,
    config=BulletConfig(
        muzzle_speed=MUZZLE_SPEED_MPS,
        muzzle_offset_m=MUZZLE_OFFSET_M,
        park_position=BULLET_PARK_POSITION,
    ),
)
```

Add `bullet_hit_link=bullet_hit_link,` to the `SensorSuite(...)` call.

In the main loop's Sense block, drain the new link next to `ground_hit_link.update()`:

```python
    bullet_hit_link.update()
```

After `fsm.step()` (step 4), add the fire + recycle block:

```python
    # 4b. Fire — launch the recycled bullet when the FSM commits to a shot.
    #     One bullet at a time: a fire command while in flight is ignored.
    fire_command = fsm.consume_fire_command()
    if fire_command is not None:
        if bullet.is_parked:
            bullet.fire(fire_command)
            log.info("FIRE — bullet launched toward intercept %s", fire_command)
        else:
            log.info("FIRE ignored — a bullet is still in flight")

    # 4c. Recycle — park the bullet when the engagement ends: it struck the
    #     ball (bullet-hit cue), the ball landed (ground-hit cue, shot missed),
    #     or the flight-time safety timeout fired.
    bullet.step()
    if not bullet.is_parked and (
        bullet_hit_link.hit_this_step()
        or ground_hit_link.hit_this_step()
        or bullet.age_steps >= BULLET_MAX_LIFETIME_STEPS
    ):
        log.info("RECYCLE — parking bullet (age=%d steps)", bullet.age_steps)
        bullet.recycle()
```

- [ ] **Step 3: Run the atlas tests, confirm green** (controller is untested glue, but ensure nothing imports-breaks and the suite passes):

Run: `cd controllers/atlas_controller && python -m pytest tests/ -v`
Expected: PASS — full suite green.

- [ ] **Step 4: Manual verification (Webots):**
  - Controller starts with no exceptions (Webots console + `atlas_telemetry.log`).
  - When the FSM reaches `ENGAGING`, a blue bullet flies from the turret toward the intercept; console logs `FIRE — bullet launched ...`.
  - When the bullet strikes the ball, the attacker logs `bullet hit #N`, ATLAS logs the bullet-hit cue + `RECYCLE`, the ball recycles, and the bullet parks (disappears).
  - When the shot misses and the ball lands, the ground-hit cue recycles the bullet too. At most one bullet is ever visible.
  - **Tuning (orchestrator/human):** co-tune `MUZZLE_SPEED_MPS` with `FSMConfig.lookahead_steps` so the bullet reaches the intercept roughly when the ball does (the lead/flight-time risk in the spec). Raise `MUZZLE_OFFSET_M` if the bullet clips the turret on launch; raise `BULLET_MAX_LIFETIME_STEPS` only if a healthy bullet is being culled mid-flight.

- [ ] **Step 5: Commit**

```bash
git add protos/AtlasTurret.proto controllers/atlas_controller/atlas_controller.py lib/scene.py
git commit -m "feat(atlas): fire and recycle the bullet on the FSM fire command"
```

---

## Task 8 — ADR + CONTEXT.md

**Model: haiku · Effort: low** — content drafted below; mechanical authoring.

**Files:**
- Create: `docs/adr/0013-engage-fire-recycled-bullet.md`
- Modify: `CONTEXT.md`

- [ ] **Step 1: Create `docs/adr/0013-engage-fire-recycled-bullet.md`:**

```markdown
# ADR-0013: Engage-fire — a recycled, supervisor-driven bullet

**Status:** Accepted

## Decision

On entering the FSM `ENGAGING` state the turret fires a small, fast **Bullet** at
the `BallisticTrajectoryPredictor`'s intercept point; the bullet destroys the
incoming `Projectile` on contact.

- The Bullet is a passive `Solid` (`AtlasBullet.proto`, `DEF ATLAS_BULLET`),
  **pre-placed once** and **recycled** by the `atlas_controller` supervisor
  (park → fire → park), mirroring the incoming `Projectile` (ADR-0011). It
  carries no controller, sensor, or radio.
- The FSM emits a one-shot **fire command** (the intercept) on the `→ ENGAGING`
  edge and spawns nothing itself — staying pure and unit-testable.
- Hit detection stays where it already is: the `Projectile` classifies a mid-air
  contact as a bullet hit (`getContactPoints`, by world-Z). The attacker emits a
  **bullet-hit pulse** on channel 3; a new `BulletHitLink` carries it to the FSM,
  which adds a "projectile destroyed → RESET" exit.

## Context

The system already has a `BallisticTrajectoryPredictor` and a `TRACK_PREDICT`
state computing an intercept ahead of the target. That pipeline only earns its
place if the weapon has travel time — an instant laser would aim at the ball's
current position and make the predictor dead code.

## Reasoning

**Recycle, not spawn/delete:** `node.remove()` can leave stale `getFromDef`
handles (cyberbotics/webots#2123), and dynamic import spawns a controller process
per shot and cannot expose new devices cleanly (#6378). Recycling one node keeps
a stable handle and matches the established `Projectile` pattern (ADR-0011).

**Passive Solid, not a self-reporting Robot:** A non-supervisor Robot cannot
`setVelocity`/teleport itself (supervisor-only), so it could not recycle itself.
A `bumper` TouchSensor is boolean-only, readable only by the owning controller,
and needs a radio relay to share — adding latency and a second hit source. So the
bullet stays passive and the supervisor drives it.

**Ball is the single source of hit truth:** The `Projectile` already
disambiguates ground vs. bullet hits by world-Z, radius-independently. Reusing it
(emitting on channel 3, mirroring the ground-hit cue on channel 2, #32) avoids
the double-counting a bullet-side detector would reintroduce.

**FSM emits, controller executes:** Node motion is Webots-specific and
untestable; keeping it in the controller and having the FSM emit a plain
`[dx, dy, dz]` keeps the FSM pure.

## Consequences

- New: `protos/AtlasBullet.proto`, `controllers/atlas_controller/bullet.py`,
  `controllers/atlas_controller/bullet_hit_link.py`. `Attacker.proto` /
  `AtlasTurret.proto` gain a channel-3 emitter/receiver; the world pre-places
  `DEF ATLAS_BULLET` and a non-bouncy `ContactProperties` pair.
- The FSM gains `consume_fire_command()` and a bullet-hit RESET exit (resolving
  the prior `_do_engage` TODO); `SensorSuite` gains `bullet_hit_link`.
- This supersedes the 2026-05-18 turret-weapon spec/plan (self-reporting Robot +
  `customData` polling + dynamic spawn), which predated the FSM restructure
  (ADR-0012) and the ground-hit single-source work (#32).
- Known limitations: **lead/flight-time mismatch** — the fixed-lookahead
  intercept and the bullet's longer flight time must be co-tuned via
  `MUZZLE_SPEED`/`lookahead_steps`; a time-of-flight-aware intercept is future
  work. **Tunnelling** is mitigated (capped speed, generous radius), not solved.
  **One bullet at a time**; a pool is future work.
```

- [ ] **Step 2: Rewrite the `## Bullet` section of `CONTEXT.md`** — replace it with:

```markdown
## Bullet

The turret's fired projectile, defined by `AtlasBullet.proto` (DEF `ATLAS_BULLET`).
A passive `Solid` — no controller, sensor, or radio — that the `atlas_controller`
supervisor *recycles*, mirroring the incoming `Projectile`: one pre-placed node is
parked out of range, teleported to the muzzle and launched (`setVelocity`) toward
the intercept when the FSM enters `ENGAGING`, then parked again when the
engagement resolves (the bullet struck the ball, the ball landed, or a flight-time
timeout). It carries no hit sensor: the incoming `Projectile` is the single source
of hit truth (it classifies a mid-air contact as a bullet hit by world-Z), and the
attacker relays that as a bullet-hit pulse on channel 3. Distinct from the
incoming ball (`Projectile`), which the turret aims *at*. See ADR-0013.
```

(If a `## Fire Command` entry exists and still describes a `fire_command`
attribute consumed by the controller, leave it — it remains accurate. Verify its
wording matches `consume_fire_command()`; adjust only if it conflicts.)

- [ ] **Step 3: Commit**

```bash
git add docs/adr/0013-engage-fire-recycled-bullet.md CONTEXT.md
git commit -m "docs: record the engage-fire recycled bullet (ADR-0013)"
```

---

## Definition of done

- [ ] `cd controllers/atlas_controller && python -m pytest tests/ -v` — full suite green (fire command, `BulletHitLink`, `Bullet`, bullet-hit RESET, world layout).
- [ ] `cd controllers/attacker_controller && python -m pytest tests/ -v` — green.
- [ ] In Webots: entering `ENGAGING` launches a blue bullet toward the intercept; striking the ball recycles both ball and bullet; a missed shot recycles the bullet on the ground-hit cue; at most one bullet exists at a time; the safety timeout never fires in normal play.
- [ ] No change to the incoming `Projectile`'s own behaviour; `AtlasLaser` remains cosmetic.
- [ ] ADR-0013 + CONTEXT.md updated; the 2026-05-18 turret-weapon spec/plan are superseded.
```
