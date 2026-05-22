# Adaptive Launch-Point Estimation — Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** ATLAS learns the attacker's roughly-static launch origin online from noisy Search Radar cues and pre-aims the idle turret there, cutting acquisition latency and adapting to drift.

**Architecture:** The attacker launches from a single jittered/drifting point (never told to ATLAS). ATLAS observes each launch radar-only (first acquisition after a resolution cue) and feeds the cued position to a `LaunchPointEstimator` (exponentially-weighted recursive mean per axis — classical noise reduction, one `alpha` knob, numpy only). The estimator exposes a ready-aim `(pan, tilt)` that the pure FSM `IDLE` state consumes. No dataset, no new device, no scikit-learn.

**Tech Stack:** Python 3.13, numpy, pytest. Webots controllers under `controllers/{attacker_controller,atlas_controller}`, shared `lib/` on `PYTHONPATH`. Tests under `controllers/*/tests/` with per-package `conftest.py`.

**Spec:** `docs/superpowers/specs/2026-05-22-launch-point-estimation-design.md`

**Conventions:** TDD, frequent commits. Run a package's tests from its tests dir, e.g. `cd controllers/atlas_controller && python -m pytest tests/ -v`. This worktree shares the main repo's interpreter — if `python` is not on PATH, use the repo venv: `C:/Users/madel/source/repos/programming_robotics_project/.venv/Scripts/python -m pytest ...` (it has numpy+pytest; conftest sets sys.path relative to the test file, so cwd is the worktree). Every commit message ends with the trailer (per `CLAUDE.md`):
```
Co-Authored-By: Claude Opus 4.7 <noreply@anthropic.com>
```
Controllers (`*_controller.py`) are thin, untested glue (Tasks 1 & 4); their logic lives in the tested modules. Verify those via a smoke import + a Webots run.

---

### Task 1: Multi-point launching — Projectile `select_launch` hook + jittered attacker wiring

Add an optional callback invoked at the top of `Projectile.spawn()` that supplies the next launch's `(spawn_position, launch_velocity)`. The attacker uses it to jitter (and optionally drift) a single nominal launch point. Backward compatible: default `None` keeps today's fixed behaviour.

**Files:**
- Modify: `controllers/attacker_controller/projectile.py` (`__init__` + `spawn`)
- Modify: `controllers/attacker_controller/attacker_controller.py` (jittered point)
- Test: `controllers/attacker_controller/tests/test_projectile.py` (add cases)

- [ ] **Step 1: Write the failing test**

Add to `controllers/attacker_controller/tests/test_projectile.py` (keep existing tests; reuse the existing stub node class — inspect the file for its name, referred to here as `StubProjectileNode`; extend it with a `last_velocity` attribute set in `setVelocity` and a `getField("translation")` returning an object whose `setSFVec3f` records `last_translation`, only if not already present):

```python
def test_select_launch_sets_pose_before_spawn():
    """spawn() calls select_launch and uses its (position, velocity)."""
    node = StubProjectileNode()
    calls = []

    def select_launch():
        calls.append(True)
        return ([1.0, 2.0, 0.5], [3.0, 4.0, 5.0, 0, 0, 0])

    proj = Projectile(node, ProjectileConfig(), select_launch=select_launch)
    proj.spawn()

    assert calls == [True]
    assert proj.config.spawn_position == [1.0, 2.0, 0.5]
    assert node.last_velocity == [3.0, 4.0, 5.0, 0, 0, 0]


def test_no_select_launch_keeps_fixed_config():
    """Without select_launch, spawn() uses the config as-is (back-compat)."""
    node = StubProjectileNode()
    cfg = ProjectileConfig(spawn_position=[0, -10, 0.5], launch_velocity=[0, 2, 10, 0, 0, 0])
    proj = Projectile(node, cfg)
    proj.spawn()
    assert proj.config.spawn_position == [0, -10, 0.5]
    assert node.last_velocity == [0, 2, 10, 0, 0, 0]
```

- [ ] **Step 2: Run test to verify it fails**

Run: `cd controllers/attacker_controller && python -m pytest tests/test_projectile.py -k select_launch -v`
Expected: FAIL — `Projectile.__init__() got an unexpected keyword argument 'select_launch'`.

- [ ] **Step 3: Implement the hook in Projectile**

In `controllers/attacker_controller/projectile.py`, add `select_launch=None` to `__init__`'s signature and store it:
```python
        # Optional callable () -> (spawn_position, launch_velocity), invoked at
        # the top of spawn() to choose where the NEXT ball launches from. None →
        # the fixed config is reused every spawn (original behaviour). The
        # attacker uses this to jitter/drift the launch point.
        self._select_launch = select_launch
```
Update `spawn()` to consult the hook first (keep the existing resetPhysics() docstring note):
```python
    def spawn(self):
        if self._select_launch is not None:
            spawn_position, launch_velocity = self._select_launch()
            self.config.spawn_position = spawn_position
            self.config.launch_velocity = launch_velocity
        self._translation.setSFVec3f(self.config.spawn_position)
        self._node.setVelocity(self.config.launch_velocity)
        self._airborne = False
        self._state = _State.ACTIVE
```

- [ ] **Step 4: Run test to verify it passes**

Run: `cd controllers/attacker_controller && python -m pytest tests/test_projectile.py -v`
Expected: all pass (existing + 2 new).

- [ ] **Step 5: Wire a jittered/drifting launch point into the attacker controller**

In `controllers/attacker_controller/attacker_controller.py`, add `import numpy as np` near the imports. After `config = ProjectileConfig(...)`, add:
```python
# --- Launch point: roughly static, with small per-launch jitter (and a slow
#     drift). The nominal point and jitter are the attacker's ground truth and
#     are NEVER sent to ATLAS — it estimates the launch origin from noisy radar.
NOMINAL_LAUNCH_POINT = [0.0, -10.0, 0.5]
LAUNCH_JITTER_M = 0.3            # zero-mean per-axis jitter (x, y; z kept fixed)
LAUNCH_DRIFT_PER_SHOT = [0.02, 0.0, 0.0]  # slow nominal drift per launch (SHOULD)
FIXED_LAUNCH_VELOCITY = [0, 2, 10, 0, 0, 0]

_launch_rng = np.random.default_rng(20260522)
_nominal = list(NOMINAL_LAUNCH_POINT)


def _next_launch():
    """select_launch hook: jitter (and drift) the nominal launch point."""
    global _nominal
    point = [
        _nominal[0] + _launch_rng.normal(0.0, LAUNCH_JITTER_M),
        _nominal[1] + _launch_rng.normal(0.0, LAUNCH_JITTER_M),
        _nominal[2],
    ]
    _nominal = [_nominal[i] + LAUNCH_DRIFT_PER_SHOT[i] for i in range(3)]
    log.info("[ATTACKER] launching from ~%s at t=%.2fs", [round(p, 2) for p in point], robot.getTime())
    return point, list(FIXED_LAUNCH_VELOCITY)
```
Pass the hook to the `Projectile(...)` construction: add `select_launch=_next_launch,`.

- [ ] **Step 6: Smoke-check imports + commit**

Run: `cd controllers/attacker_controller && ../../.venv/Scripts/python -c "import projectile; print('ok')"`
Expected: prints `ok`.
```bash
git add controllers/attacker_controller/
git commit -m "feat(attacker): jittered/drifting launch point via select_launch hook"
```

---

### Task 2: `LaunchPointEstimator` — recursive launch-origin estimate

**Files:**
- Create: `controllers/atlas_controller/launch_point_estimator.py`
- Test: `controllers/atlas_controller/tests/test_launch_point_estimator.py`

- [ ] **Step 1: Write the failing test**

```python
# controllers/atlas_controller/tests/test_launch_point_estimator.py
"""Tests for LaunchPointEstimator — recursive estimate of the launch origin."""
import math
import numpy as np
from launch_point_estimator import LaunchPointEstimator


def test_no_estimate_before_any_observation():
    est = LaunchPointEstimator(alpha=0.2)
    assert est.get_estimate() is None
    assert est.get_ready_aim([0.0, 0.0, 0.0]) is None
    assert est.samples == 0


def test_first_observation_sets_estimate():
    est = LaunchPointEstimator(alpha=0.2)
    est.observe([1.0, 2.0, 3.0])
    assert est.get_estimate() == [1.0, 2.0, 3.0]
    assert est.samples == 1


def test_converges_to_true_mean_under_noise():
    """Noisy samples around a true point → estimate close to truth (≪ noise)."""
    true = np.array([2.0, -9.0, 0.5])
    rng = np.random.default_rng(0)
    est = LaunchPointEstimator(alpha=0.15)
    for _ in range(300):
        est.observe((true + rng.normal(0, 0.3, size=3)).tolist())
    err = np.linalg.norm(np.array(est.get_estimate()) - true)
    assert err < 0.15  # well below the 0.3 per-sample noise


def test_adapts_after_drift():
    """After the true point shifts, the estimate moves toward the new point."""
    rng = np.random.default_rng(1)
    est = LaunchPointEstimator(alpha=0.2)
    for _ in range(200):
        est.observe((np.array([0.0, -10.0, 0.5]) + rng.normal(0, 0.2, 3)).tolist())
    before = np.array(est.get_estimate())
    for _ in range(200):
        est.observe((np.array([5.0, -6.0, 0.5]) + rng.normal(0, 0.2, 3)).tolist())
    after = np.array(est.get_estimate())
    assert np.linalg.norm(after - np.array([5.0, -6.0, 0.5])) < np.linalg.norm(before - np.array([5.0, -6.0, 0.5]))
    assert after[0] > before[0]  # moved toward the new point


def test_ready_aim_points_at_estimate_relative_to_turret():
    est = LaunchPointEstimator(alpha=0.2)
    est.observe([3.0, 4.0, 1.0])      # estimate = [3,4,1]
    turret = [0.0, 0.0, 0.0]
    pan, tilt = est.get_ready_aim(turret)
    assert abs(pan - math.atan2(3.0, 4.0)) < 1e-9
    assert abs(tilt - math.atan2(1.0, math.hypot(3.0, 4.0))) < 1e-9
```

- [ ] **Step 2: Run test to verify it fails**

Run: `cd controllers/atlas_controller && python -m pytest tests/test_launch_point_estimator.py -v`
Expected: FAIL — `ModuleNotFoundError: No module named 'launch_point_estimator'`.

- [ ] **Step 3: Write minimal implementation**

```python
# controllers/atlas_controller/launch_point_estimator.py
"""LaunchPointEstimator — online recursive estimate of the attacker's launch origin.

The Think layer of the adaptive launch-point feature. Each launch is observed as
a noisy world-frame position (the Search Radar cue at first acquisition). The
estimator fuses observations into a running per-axis estimate with exponential
forgetting:

    first obs:  mu = obs
    later:      mu += alpha * (obs - mu)

This is classical noise reduction (a moving-average / 1-state-per-axis recursive
filter): variance falls as cues accumulate, while the forgetting factor `alpha`
lets the estimate track a slowly drifting launch point. `alpha` is the single
tuning knob — the dial between noise-robustness (small alpha) and adaptation
speed (large alpha). It exposes a ready-aim (pan, tilt) for the FSM IDLE state to
pre-slew toward. No dataset, no serialized model. See the launch-point spec.
"""
import math


class LaunchPointEstimator:
    """Recursive estimate of the launch origin from noisy launch observations.

    Args:
        alpha: forgetting/adaptation rate in (0, 1]. Smaller = smoother/less
            reactive; larger = tracks drift faster.
    """

    def __init__(self, alpha=0.15):
        self._alpha = alpha
        self._mu: list[float] | None = None
        self.samples = 0

    def observe(self, position) -> None:
        """Fuse one noisy launch-origin observation (world-frame [x, y, z])."""
        if self._mu is None:
            self._mu = [float(position[0]), float(position[1]), float(position[2])]
        else:
            for i in range(3):
                self._mu[i] += self._alpha * (float(position[i]) - self._mu[i])
        self.samples += 1

    def get_estimate(self):
        """Return the current launch-origin estimate [x, y, z], or None (cold start)."""
        return None if self._mu is None else list(self._mu)

    def get_ready_aim(self, turret_position):
        """Return (pan, tilt) aiming at the estimate relative to the turret, or None.

        Uses the FSM's convention: pan = atan2(dx, dy),
        tilt = atan2(dz, sqrt(dx² + dy²)). None before any observation.
        """
        if self._mu is None:
            return None
        dx = self._mu[0] - turret_position[0]
        dy = self._mu[1] - turret_position[1]
        dz = self._mu[2] - turret_position[2]
        pan = math.atan2(dx, dy)
        tilt = math.atan2(dz, math.sqrt(dx * dx + dy * dy))
        return (pan, tilt)
```

- [ ] **Step 4: Run test to verify it passes**

Run: `cd controllers/atlas_controller && python -m pytest tests/test_launch_point_estimator.py -v`
Expected: 5 passed.

- [ ] **Step 5: Commit**

```bash
git add controllers/atlas_controller/launch_point_estimator.py controllers/atlas_controller/tests/test_launch_point_estimator.py
git commit -m "feat(atlas): LaunchPointEstimator recursive launch-origin estimate"
```

---

### Task 3: FSM `IDLE` consumes the ready-aim (pure)

Add `idle_aim_provider` to `SensorSuite` (default `None`) and make `_do_idle` pre-aim using it when available, else hold the fixed beam. FSM stays pure.

**Files:**
- Modify: `controllers/atlas_controller/fsm.py` (`SensorSuite`, `_do_idle`)
- Test: `controllers/atlas_controller/tests/test_fsm.py` (add cases)

- [ ] **Step 1: Write the failing test**

Reuse the existing `test_fsm.py` FSM-construction pattern (inspect the file). Add:
```python
class _StubAimProvider:
    def __init__(self, ready_aim):
        self._ready_aim = ready_aim
    def get_ready_aim(self, turret_position):
        return self._ready_aim


def test_idle_preaims_at_estimate(make_idle_fsm):
    fsm = make_idle_fsm(idle_aim_provider=_StubAimProvider((1.234, 0.3)))
    fsm.step()
    pan, tilt = fsm.commanded_aim
    assert abs(pan - 1.234) < 1e-9
    assert abs(tilt - 0.3) < 1e-9


def test_idle_falls_back_when_no_estimate(make_idle_fsm):
    fsm = make_idle_fsm(idle_aim_provider=_StubAimProvider(None))
    fsm.step()
    pan, tilt = fsm.commanded_aim
    assert abs(tilt - fsm.config.idle_tilt) < 1e-9


def test_idle_fixed_beam_when_provider_absent(make_idle_fsm):
    fsm = make_idle_fsm()  # default None
    fsm.step()
    pan, tilt = fsm.commanded_aim
    assert abs(tilt - fsm.config.idle_tilt) < 1e-9
```
If `test_fsm.py` builds the FSM inline, write a small local `make_idle_fsm(**kw)` helper mirroring the existing tests' SensorSuite/TurretHardware stub construction, accepting an optional `idle_aim_provider`, and returning an `AtlasFSM` whose recording motor stubs make `commanded_aim` reflect issued angles.

- [ ] **Step 2: Run test to verify it fails**

Run: `cd controllers/atlas_controller && python -m pytest tests/test_fsm.py -k "preaims or fall or provider_absent" -v`
Expected: FAIL — `SensorSuite` rejects `idle_aim_provider`, or IDLE still commands the fixed beam.

- [ ] **Step 3: Add the SensorSuite field**

In `fsm.py`, add to the `SensorSuite` dataclass (after the last existing field):
```python
    # Optional Think-layer provider of a pre-aim suggestion for IDLE.
    # get_ready_aim(turret_position) -> (pan, tilt) | None. Default None keeps
    # the original fixed-beam IDLE behaviour (and existing tests) working.
    idle_aim_provider: object = None
```

- [ ] **Step 4: Use the ready-aim in _do_idle**

Replace the fixed-aim line in `_do_idle`:
```python
        self._command_angles(self.config.idle_pan, self.config.idle_tilt)
```
with:
```python
        # Pre-aim at the estimated launch origin (Layer-3 suggestion); fall back
        # to the fixed idle beam before any estimate exists or when no provider
        # is wired. The FSM stays pure: it consumes a (pan, tilt) value.
        ready_aim = None
        if self.sensors.idle_aim_provider is not None:
            ready_aim = self.sensors.idle_aim_provider.get_ready_aim(
                self.hardware.turret_position
            )
        if ready_aim is not None:
            self._command_angles(ready_aim[0], ready_aim[1])
        else:
            self._command_angles(self.config.idle_pan, self.config.idle_tilt)
```
Update the `_do_idle` docstring's first paragraph to note the pre-aim/fallback.

- [ ] **Step 5: Run tests to verify they pass**

Run: `cd controllers/atlas_controller && python -m pytest tests/test_fsm.py -v`
Expected: all pass (existing stay green — the new field defaults to `None`).

- [ ] **Step 6: Commit**

```bash
git add controllers/atlas_controller/fsm.py controllers/atlas_controller/tests/test_fsm.py
git commit -m "feat(atlas): IDLE pre-aims at the estimated launch origin"
```

---

### Task 4: Wire the estimator into the ATLAS controller + docs

**Files:**
- Modify: `controllers/atlas_controller/atlas_controller.py`
- Modify: `CONTEXT.md` (add a `LaunchPointEstimator` glossary entry)
- Create: `docs/adr/0014-adaptive-launch-point-estimation.md`

- [ ] **Step 1: Add imports + constant + construct the estimator**

In `atlas_controller.py`, add `from launch_point_estimator import LaunchPointEstimator`. Add a constant in the tuning block:
```python
# --- Adaptive launch-point estimator ---
LAUNCH_POINT_ALPHA = 0.15   # forgetting/adaptation rate (noise-robustness vs drift)
```
After `bullet_hit_link = BulletHitLink(bullet_hit_receiver)`, add:
```python
launch_point_estimator = LaunchPointEstimator(alpha=LAUNCH_POINT_ALPHA)
```
Add `idle_aim_provider=launch_point_estimator` to the `SensorSuite(...)` construction.

- [ ] **Step 2: Initialise the observation latch + feed the estimator in the loop**

Before `while robot.step(timestep) != -1:` add:
```python
# A launch is the FIRST radar acquisition after the previous projectile
# resolved. _estimator_armed re-arms on each resolution cue so each engagement
# yields exactly one launch observation (debounces mid-flight dropouts).
estimator_armed = True
```
In the loop, after the `bullet_hit_link.update()` / logging block and before `fcr.update(...)`:
```python
    # Launch-origin observation: re-arm on a resolution cue, then feed the next
    # fresh Search Radar cue (the launch is still near its origin) to the
    # estimator. Done before fsm.step() so IDLE pre-aims with the freshest estimate.
    if ground_hit_link.hit_this_step() or bullet_hit_link.hit_this_step():
        estimator_armed = True
    cue = cue_link.get_cue()
    if estimator_armed and cue is not None:
        launch_point_estimator.observe(cue)
        estimator_armed = False
        log.info(
            "[ATLAS] launch observed at %s; estimate now %s (n=%d)",
            [round(c, 2) for c in cue],
            [round(v, 2) for v in launch_point_estimator.get_estimate()],
            launch_point_estimator.samples,
        )
```

- [ ] **Step 3: (SHOULD) Seed the tracker at the estimate on acquisition**

Optional, only if `TrackFilter` exposes a seed/initial-state setter (inspect `track_filter.py`). If it does, on the IDLE→AIM transition seed it with the estimate; if it does not, skip this step and note it in the PR. Do not invent a method that does not exist.

- [ ] **Step 4: Smoke-check imports + run full ATLAS suite**

Run: `cd controllers/atlas_controller && ../../.venv/Scripts/python -c "import launch_point_estimator; print('ok')"`
Expected: prints `ok`.
Run: `cd controllers/atlas_controller && python -m pytest tests/ -v`
Expected: all pass.

- [ ] **Step 5: Manual verification in Webots**

Run the simulation. Confirm in the log/console:
- `[ATTACKER] launching from ~[...]` jitters around the nominal point (and slowly drifts).
- `[ATLAS] launch observed ...; estimate now [...] (n=K)` appears once per engagement; the estimate **tightens** toward the nominal as `n` grows.
- The idle turret **pre-orients** toward the estimated launch point and the FCR locks sooner than from the straight-up rest pose; after drift the aim re-converges.
Note any tuning of `LAUNCH_POINT_ALPHA`, `LAUNCH_JITTER_M`, or the nominal point needed for the behaviour to read on screen.

- [ ] **Step 6: Docs — glossary + ADR**

Add to `CONTEXT.md`:
```markdown
## LaunchPointEstimator

Online learning subsystem on the ATLAS side. Observes each launch radar-only
(the Search Radar cue at first acquisition after a resolution cue) and fuses it
into a recursive per-axis estimate of the attacker's launch origin (exponentially
-weighted mean — classical noise reduction; variance falls as cues accumulate, and
the forgetting factor tracks slow drift). Exposes a ready-aim (pan, tilt) the FSM
IDLE state uses to pre-slew toward the launch origin, cutting acquisition latency.
Distinct from the BallisticTrajectoryPredictor (per-projectile). No dataset and no
serialized model — it learns live.
```
Create `docs/adr/0014-adaptive-launch-point-estimation.md`:
```markdown
# 0014 — Adaptive launch-point estimation (online, radar-only)

## Status
Accepted

## Context
The Advanced / Contextual Component (10%) requires an "AI or Adaptive Behaviour"
element. This is the lower-risk alternative to the launch-direction learner: a
classical online estimator that improves acquisition latency.

## Decision
ATLAS estimates the attacker's roughly-static launch origin online from the noisy
Search Radar cues it already receives (first acquisition after a resolution cue),
using an exponentially-weighted recursive mean per axis (one forgetting/adaptation
knob). It pre-aims the pure FSM IDLE state at the estimate (and may seed the
tracker), cutting acquisition latency; the estimate improves with experience and
adapts to slow drift. No new sensor/radio device, no dataset, no scikit-learn. The
five-state FSM structure (ADR-0012) is unchanged (only IDLE gains an input).

## Consequences
- Improves acquisition latency / pre-lock estimate, NOT terminal aim (the FCR is
  already precise once locked) — the demonstrable claim is framed accordingly.
- "Value of learning" is measurable: the estimate-vs-launch-number convergence
  curve and reduced time-to-lock.
- Cold start (no estimate yet) falls back to the fixed idle beam — a
  safety/robustness property.
```

- [ ] **Step 7: Commit**

```bash
git add controllers/atlas_controller/atlas_controller.py CONTEXT.md docs/adr/0014-adaptive-launch-point-estimation.md
git commit -m "feat(atlas): wire LaunchPointEstimator + IDLE pre-aim; glossary + ADR-0014"
```

---

## Self-Review

**Spec coverage:** jittered/drifting attacker point → Task 1; radar-only observation segmented by resolution cues → Task 4; `LaunchPointEstimator` recursive estimate w/ forgetting → Task 2; IDLE pre-aim + cold-start fallback → Task 3; tracker seeding (SHOULD) → Task 4 step 3 (guarded on the method existing); glossary + ADR → Task 4. ✓

**Type/name consistency:** `LaunchPointEstimator(alpha=)`, `.observe(position)`, `.get_estimate()`, `.get_ready_aim(turret_position)`, `.samples` (Task 2) match the FSM stub signature (Task 3, `get_ready_aim(turret_position)`) and the controller usage (Task 4). `SensorSuite.idle_aim_provider` (Task 3) is set in the controller (Task 4). `select_launch` hook (Task 1) matches its caller. Consistent.

**Note:** the FSM uses `idle_aim_provider.get_ready_aim(turret_position)` (provider needs the turret position), whereas the direction-learner PR used a no-arg `get_ready_aim()`. That difference is intentional and contained to this branch.
```
