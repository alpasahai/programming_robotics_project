# Adaptive Launch-Direction Learning — Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Make ATLAS learn the attacker's launch-direction pattern online and pre-aim the idle turret at the sector it predicts next, re-adapting when the pattern drifts.

**Architecture:** The attacker launches from several physical sectors following a structured-but-noisy, drifting pattern (`launch_sequence`). ATLAS observes each launch *radar-only* (first acquisition after a resolution cue), discovers the sectors by online angular clustering (`SectorMap`), and learns `P(next sector | context)` with an online logistic regression (`SGDClassifier.partial_fit`) inside `AttackPredictor`. The predictor exposes a ready-aim bearing that the pure FSM `IDLE` state consumes. No new radio channel, no dataset, no committed model.

**Tech Stack:** Python 3.13, numpy, scikit-learn (new), pytest. Webots controllers under `controllers/{attacker_controller,atlas_controller}`, shared `lib/` on `PYTHONPATH`. Tests under `controllers/*/tests/` with per-package `conftest.py` adding the controller dir + `lib/` to `sys.path`.

**Spec:** `docs/superpowers/specs/2026-05-22-attack-plan-and-adaptive-ml-design.md`

**Conventions:** This repo uses TDD and frequent commits. Run tests for a package from its tests dir, e.g. `cd controllers/atlas_controller && python -m pytest tests/ -v` (uses `conftest.py`). Every commit message ends with the trailer (per `CLAUDE.md`):
```
Co-Authored-By: Claude Opus 4.7 <noreply@anthropic.com>
```
Controllers (`*_controller.py`) are thin, untested glue (Tasks 3 & 8); their logic lives in the tested modules. Verify those two via a smoke import + a Webots run.

---

### Task 1: Add the scikit-learn dependency

**Files:**
- Modify: `pyproject.toml:7-12` (dependencies array)

- [ ] **Step 1: Add scikit-learn via uv**

Run (from repo root):
```bash
uv add scikit-learn
```
Expected: `pyproject.toml` `dependencies` now contains a `scikit-learn>=...` entry and the package is installed into `.venv`. If `uv` is unavailable, instead edit `pyproject.toml` to add `"scikit-learn>=1.5"` to the `dependencies` list and run `.venv/Scripts/python -m pip install scikit-learn`.

- [ ] **Step 2: Verify the import works**

Run:
```bash
.venv/Scripts/python -c "from sklearn.linear_model import SGDClassifier; print('sklearn ok')"
```
Expected: prints `sklearn ok`.

- [ ] **Step 3: Commit**

```bash
git add pyproject.toml uv.lock
git commit -m "build: add scikit-learn for the adaptive launch-direction learner"
```
(If `uv.lock` does not exist in the repo, omit it from the `git add`.)

---

### Task 2: Attacker launch pattern — `launch_sequence` (structured + noisy + drifting)

A pure, seeded generator of sector ids. Structure = a cyclic *tour* of sectors with characteristic burst lengths; noise = stochastic burst lengths + occasional deviations; drift = a list of phases that switch the generating rule.

**Files:**
- Create: `controllers/attacker_controller/attack_pattern.py`
- Test: `controllers/attacker_controller/tests/test_attack_pattern.py`

- [ ] **Step 1: Write the failing test**

```python
# controllers/attacker_controller/tests/test_attack_pattern.py
"""Tests for launch_sequence — the attacker's structured/noisy/drifting pattern."""
import itertools
import numpy as np
from attack_pattern import PatternRule, launch_sequence


def _take(seq, n):
    return list(itertools.islice(seq, n))


def test_reproducible_under_seed():
    """Same seed → identical sequence (pure given the rng)."""
    rule = PatternRule(tour=[0, 1, 2], mean_burst=[5, 2, 4], deviation_prob=0.1)
    a = _take(launch_sequence([(rule, None)], np.random.default_rng(7)), 100)
    b = _take(launch_sequence([(rule, None)], np.random.default_rng(7)), 100)
    assert a == b


def test_all_sectors_in_tour_when_no_deviation():
    """deviation_prob=0 → every emitted sector is in the tour."""
    rule = PatternRule(tour=[0, 1, 2], mean_burst=[5, 2, 4], deviation_prob=0.0)
    out = _take(launch_sequence([(rule, None)], np.random.default_rng(1)), 200)
    assert set(out) <= {0, 1, 2}


def test_structure_produces_runs():
    """deviation_prob=0 → consecutive repeats (bursts) exist, not alternation."""
    rule = PatternRule(tour=[0, 1, 2], mean_burst=[5, 2, 4], deviation_prob=0.0)
    out = _take(launch_sequence([(rule, None)], np.random.default_rng(2)), 300)
    longest_run = max(
        len(list(g)) for _, g in itertools.groupby(out)
    )
    assert longest_run >= 3  # bursts of several launches occur


def test_drift_switches_distribution():
    """After phase A's count, sectors come from phase B's tour."""
    rule_a = PatternRule(tour=[0, 1], mean_burst=[4, 4], deviation_prob=0.0)
    rule_b = PatternRule(tour=[2, 3], mean_burst=[4, 4], deviation_prob=0.0)
    out = _take(launch_sequence([(rule_a, 40), (rule_b, None)], np.random.default_rng(3)), 120)
    assert set(out[:40]) <= {0, 1}
    assert set(out[60:]) <= {2, 3}  # well past the phase boundary
```

- [ ] **Step 2: Run test to verify it fails**

Run: `cd controllers/attacker_controller && python -m pytest tests/test_attack_pattern.py -v`
Expected: FAIL with `ModuleNotFoundError: No module named 'attack_pattern'`.

- [ ] **Step 3: Write minimal implementation**

```python
# controllers/attacker_controller/attack_pattern.py
"""The attacker's launch-direction pattern: structured, noisy, and drifting.

A pure, seeded generator of sector ids (ints). It is the attacker's *behaviour*,
NOT a training set — ATLAS never sees it; it only observes the resulting
launches via radar. See
docs/superpowers/specs/2026-05-22-attack-plan-and-adaptive-ml-design.md.

Model
-----
* Structure: a cyclic ``tour`` of sectors, each fired in a burst of a
  characteristic mean length (run-length structure).
* Noise: burst lengths are sampled stochastically around their mean, and with
  probability ``deviation_prob`` a launch deviates to a random other sector.
* Drift: ``launch_sequence`` takes a list of (rule, count) phases; when one
  phase's count is exhausted the generating rule switches, so the pattern the
  defender must learn changes mid-run.
"""
from dataclasses import dataclass


@dataclass
class PatternRule:
    """One stationary launch rule (one phase of the run)."""

    tour: list[int]            # cyclic sector order, e.g. [0, 1, 2]
    mean_burst: list[float]    # mean burst length per tour position, e.g. [5, 2, 4]
    deviation_prob: float = 0.1  # chance a launch jumps to a random other sector


def _sample_burst(rng, mean):
    """A stochastic burst length ≥ 1, centred on ``mean`` (Gaussian, ~30% spread)."""
    return max(1, int(round(rng.normal(mean, 0.3 * mean))))


def _random_other(rng, tour, current):
    """A sector from ``tour`` other than ``current`` (the deviation target)."""
    others = [s for s in tour if s != current]
    return current if not others else others[rng.integers(len(others))]


def launch_sequence(phases, rng):
    """Yield sector ids forever, walking through ``phases`` then holding the last.

    Args:
        phases: list of ``(PatternRule, count)``. ``count`` launches are emitted
            under that rule, then the next phase begins. The final phase should
            use ``count=None`` to emit indefinitely.
        rng:    a ``numpy.random.Generator`` (seed it for reproducibility).

    Yields:
        int sector ids.
    """
    for rule, count in phases:
        emitted = 0
        tour_idx = 0
        burst_left = _sample_burst(rng, rule.mean_burst[0])
        while count is None or emitted < count:
            if rng.random() < rule.deviation_prob:
                sector = _random_other(rng, rule.tour, rule.tour[tour_idx])
            else:
                sector = rule.tour[tour_idx]
                burst_left -= 1
                if burst_left <= 0:
                    tour_idx = (tour_idx + 1) % len(rule.tour)
                    burst_left = _sample_burst(rng, rule.mean_burst[tour_idx])
            emitted += 1
            yield sector
```

- [ ] **Step 4: Run test to verify it passes**

Run: `cd controllers/attacker_controller && python -m pytest tests/test_attack_pattern.py -v`
Expected: 4 passed.

- [ ] **Step 5: Commit**

```bash
git add controllers/attacker_controller/attack_pattern.py controllers/attacker_controller/tests/test_attack_pattern.py
git commit -m "feat(attacker): structured/noisy/drifting launch pattern generator"
```

---

### Task 3: Multi-sector launching — Projectile `select_launch` hook + attacker wiring

The `Projectile` currently launches from a fixed `config.spawn_position`/`launch_velocity`. Add an optional callback, invoked at the top of `spawn()`, that supplies the next launch's `(spawn_position, launch_velocity)`. The attacker controller wires it to `launch_sequence` + a sector→launch-geometry map. Backward compatible: default `None` keeps today's fixed behaviour.

**Files:**
- Modify: `controllers/attacker_controller/projectile.py:66-106` (`__init__` + `spawn`)
- Create: `controllers/attacker_controller/sectors.py` (sector geometry helper)
- Modify: `controllers/attacker_controller/attacker_controller.py` (wire the pattern)
- Test: `controllers/attacker_controller/tests/test_projectile.py` (add cases)
- Test: `controllers/attacker_controller/tests/test_sectors.py` (new)

- [ ] **Step 1: Write the failing test for the Projectile hook**

Add to `controllers/attacker_controller/tests/test_projectile.py` (keep existing tests; reuse its stub — inspect the file for the stub node class name, referred to here as `StubProjectileNode`):

```python
def test_select_launch_sets_pose_before_spawn():
    """spawn() calls select_launch and uses its (position, velocity)."""
    node = StubProjectileNode()  # the stub already used in this test module
    calls = []

    def select_launch():
        calls.append(True)
        return ([1.0, 2.0, 0.5], [3.0, 4.0, 5.0, 0, 0, 0])

    proj = Projectile(node, ProjectileConfig(), select_launch=select_launch)
    proj.spawn()

    assert calls == [True]
    assert proj.config.spawn_position == [1.0, 2.0, 0.5]
    assert proj.config.launch_velocity == [3.0, 4.0, 5.0, 0, 0, 0]
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

If the existing stub does not record the applied velocity, add a `last_velocity` attribute to it set inside its `setVelocity`, and a `getField("translation")` returning an object whose `setSFVec3f` records `last_translation`. (Inspect `tests/attacker_stubs.py` / `test_projectile.py` first and reuse the existing stub; only extend it if needed.)

- [ ] **Step 2: Run test to verify it fails**

Run: `cd controllers/attacker_controller && python -m pytest tests/test_projectile.py -k select_launch -v`
Expected: FAIL — `Projectile.__init__() got an unexpected keyword argument 'select_launch'`.

- [ ] **Step 3: Implement the hook in Projectile**

In `controllers/attacker_controller/projectile.py`, extend `__init__` (add the parameter + store it):

```python
    def __init__(
        self,
        node,
        config: ProjectileConfig | None = None,
        on_ground_hit=None,
        on_bullet_hit=None,
        select_launch=None,
    ):
```
Add after `self._on_bullet_hit = on_bullet_hit`:
```python
        # Optional callable () -> (spawn_position, launch_velocity), invoked at
        # the top of spawn() to choose where the NEXT ball launches from. None →
        # the fixed config is reused every spawn (original single-corridor
        # behaviour). The attacker uses this to launch from multiple sectors.
        self._select_launch = select_launch
```
Replace the body of `spawn()` so it consults the hook first:
```python
    def spawn(self):
        """Place the ball at the spawn point and launch it (enter ACTIVE).

        If a select_launch callback was supplied, it chooses this launch's
        spawn_position/launch_velocity first (multi-sector attack). See the
        resetPhysics() note below.

        Deliberately does NOT call resetPhysics() here: in Webots, a
        resetPhysics() in the same timestep as setVelocity() zeroes the velocity
        we are trying to apply, so the ball never launches. Physics is reset in
        _despawn() instead (a different timestep), leaving the parked body clean
        before we relaunch it.
        """
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

- [ ] **Step 5: Write the failing test for sector geometry**

```python
# controllers/attacker_controller/tests/test_sectors.py
"""Tests for sector launch geometry."""
import math
from sectors import sector_launch


def test_due_south_matches_legacy_corridor():
    """Azimuth π (due south, ENU pan = atan2(dx,dy)) reproduces the old corridor."""
    pos, vel = sector_launch(math.pi, ground_range=10.0, height=0.5,
                             h_speed=2.0, v_speed=10.0)
    assert pos[0] == 0.0 or abs(pos[0]) < 1e-9   # dx ≈ 0
    assert abs(pos[1] - (-10.0)) < 1e-9          # dy ≈ -10 (south)
    assert abs(pos[2] - 0.5) < 1e-9
    assert abs(vel[0]) < 1e-9                    # vx ≈ 0
    assert abs(vel[1] - 2.0) < 1e-9              # vy toward turret (+north)
    assert abs(vel[2] - 10.0) < 1e-9


def test_distinct_azimuths_give_distinct_positions():
    p1, _ = sector_launch(0.0, 10.0, 0.5, 2.0, 10.0)
    p2, _ = sector_launch(math.pi / 2, 10.0, 0.5, 2.0, 10.0)
    assert p1[:2] != p2[:2]


def test_velocity_points_back_at_origin():
    """Horizontal velocity is anti-parallel to the spawn's horizontal offset."""
    az = 1.0
    pos, vel = sector_launch(az, 10.0, 0.5, 2.0, 10.0)
    # pos horizontal points away from origin at bearing az; vel horizontal opposes it
    assert vel[0] * pos[0] <= 0 and vel[1] * pos[1] <= 0
```

- [ ] **Step 6: Run test to verify it fails**

Run: `cd controllers/attacker_controller && python -m pytest tests/test_sectors.py -v`
Expected: FAIL — `ModuleNotFoundError: No module named 'sectors'`.

- [ ] **Step 7: Implement sector geometry**

```python
# controllers/attacker_controller/sectors.py
"""Launch geometry for the attacker's sectors.

A sector is a bearing around the turret (assumed near the world origin). ENU,
Z-up; the turret's aim convention is pan = atan2(dx, dy), so a target at bearing
``az`` and ground range R sits at horizontal offset (R·sin az, R·cos az). The
ball launches from there with horizontal velocity aimed back at the origin plus
an upward component, matching the original single-corridor launch.
"""
import math


def sector_launch(azimuth_rad, ground_range, height, h_speed, v_speed):
    """Return ``(spawn_position, launch_velocity)`` for a launch sector.

    Args:
        azimuth_rad:  bearing of the sector (pan convention, atan2(dx, dy)).
        ground_range: horizontal distance from the turret/origin (m).
        height:       spawn height (m).
        h_speed:      horizontal speed toward the turret (m/s).
        v_speed:      upward speed (m/s).

    Returns:
        (spawn_position[x,y,z], launch_velocity[vx,vy,vz,wx,wy,wz]).
    """
    sx, cx = math.sin(azimuth_rad), math.cos(azimuth_rad)
    spawn_position = [ground_range * sx, ground_range * cx, height]
    launch_velocity = [-h_speed * sx, -h_speed * cx, v_speed, 0.0, 0.0, 0.0]
    return spawn_position, launch_velocity
```

- [ ] **Step 8: Run test to verify it passes**

Run: `cd controllers/attacker_controller && python -m pytest tests/test_sectors.py -v`
Expected: 3 passed.

- [ ] **Step 9: Wire the pattern into the attacker controller**

In `controllers/attacker_controller/attacker_controller.py`, add imports near the others:
```python
import math
import numpy as np
from attack_pattern import PatternRule, launch_sequence
from sectors import sector_launch
```
Define the sectors + drifting pattern after `config = ProjectileConfig(...)` (replace the fixed spawn/velocity intent with sector-driven launches):
```python
# --- Attack plan: launch from several sectors on a structured/noisy/drifting
#     pattern. The sectors and pattern are the attacker's ground truth and are
#     NEVER sent to ATLAS — it perceives launches from radar and discovers the
#     sectors itself. See the attack-plan spec.
SECTOR_AZIMUTHS_RAD = [math.pi, math.pi * 0.6, math.pi * 1.4]  # 3 well-separated bearings
SECTOR_GROUND_RANGE_M = 10.0
SECTOR_HEIGHT_M = 0.5
SECTOR_H_SPEED = 2.0
SECTOR_V_SPEED = 10.0

_sector_launches = [
    sector_launch(az, SECTOR_GROUND_RANGE_M, SECTOR_HEIGHT_M, SECTOR_H_SPEED, SECTOR_V_SPEED)
    for az in SECTOR_AZIMUTHS_RAD
]

# Phase A then a drift to phase B (different tour + burst lengths) so ATLAS must
# re-learn mid-run. Seeded for reproducible demos.
_pattern_rng = np.random.default_rng(20260522)
_phase_a = PatternRule(tour=[0, 1, 2], mean_burst=[5, 2, 4], deviation_prob=0.1)
_phase_b = PatternRule(tour=[2, 0, 1], mean_burst=[3, 5, 2], deviation_prob=0.1)
_pattern = launch_sequence([(_phase_a, 40), (_phase_b, None)], _pattern_rng)


def _next_launch():
    """select_launch hook: advance the pattern and return that sector's geometry."""
    sector = next(_pattern)
    log.info("[ATTACKER] launching from sector %d at t=%.2fs", sector, robot.getTime())
    return _sector_launches[sector]
```
Change the `Projectile(...)` construction to pass the hook:
```python
projectile = Projectile(
    projectile_node,
    config,
    on_ground_hit=on_ground_hit,
    on_bullet_hit=on_bullet_hit,
    select_launch=_next_launch,
)
```

- [ ] **Step 10: Smoke-check the controller imports**

Run:
```bash
cd controllers/attacker_controller && ../../.venv/Scripts/python -c "import attack_pattern, sectors; print('attacker modules import ok')"
```
Expected: prints `attacker modules import ok`. (The controller file itself imports `controller` (Webots) and can only fully run inside Webots — verified in Task 8's run step.)

- [ ] **Step 11: Commit**

```bash
git add controllers/attacker_controller/
git commit -m "feat(attacker): launch from multiple sectors via the drifting pattern"
```

---

### Task 4: `SectorMap` — online angular clustering (sector discovery)

ATLAS is not told how many sectors exist. `SectorMap` discovers them by online 1-D angular leader-clustering of observed bearings, assigning stable ids. No `k`. The tolerance is a perception threshold (Task 8 derives it from radar noise).

**Files:**
- Create: `controllers/atlas_controller/sector_map.py`
- Test: `controllers/atlas_controller/tests/test_sector_map.py`

- [ ] **Step 1: Write the failing test**

```python
# controllers/atlas_controller/tests/test_sector_map.py
"""Tests for SectorMap — online angular clustering that discovers launch sectors."""
import math
from sector_map import SectorMap


def test_first_bearing_opens_cluster_zero():
    sm = SectorMap(tol_rad=0.1)
    assert sm.observe(1.0) == 0
    assert len(sm) == 1


def test_near_bearing_joins_same_cluster():
    sm = SectorMap(tol_rad=0.1)
    a = sm.observe(1.00)
    b = sm.observe(1.05)  # within tol
    assert a == b
    assert len(sm) == 1


def test_far_bearing_opens_new_cluster():
    sm = SectorMap(tol_rad=0.1)
    sm.observe(1.0)
    assert sm.observe(2.0) == 1  # beyond tol → new id
    assert len(sm) == 2


def test_count_emerges_from_data_no_k():
    """Three well-separated bearings → exactly three discovered sectors."""
    sm = SectorMap(tol_rad=0.2)
    seq = [0.0, 0.02, 2.0, 1.99, -2.0, -2.01, 0.01]
    ids = [sm.observe(b) for b in seq]
    assert len(sm) == 3
    assert ids[0] == ids[1] == ids[6]   # all near 0.0
    assert ids[2] == ids[3]             # near 2.0
    assert ids[4] == ids[5]             # near -2.0


def test_wraparound_pi_is_one_cluster():
    """Bearings near +π and −π are the same direction → one cluster."""
    sm = SectorMap(tol_rad=0.2)
    a = sm.observe(math.pi - 0.05)
    b = sm.observe(-math.pi + 0.05)
    assert a == b
    assert len(sm) == 1


def test_bearing_returns_cluster_centre():
    sm = SectorMap(tol_rad=0.2)
    sm.observe(1.0)
    sm.observe(1.1)
    assert abs(sm.bearing(0) - 1.0) < 0.2
```

- [ ] **Step 2: Run test to verify it fails**

Run: `cd controllers/atlas_controller && python -m pytest tests/test_sector_map.py -v`
Expected: FAIL — `ModuleNotFoundError: No module named 'sector_map'`.

- [ ] **Step 3: Write minimal implementation**

```python
# controllers/atlas_controller/sector_map.py
"""SectorMap — online discovery of launch directions by angular clustering.

ATLAS is not told how many launch sectors exist or where they are. SectorMap
clusters the stream of observed launch bearings online (1-D angular leader
clustering): each bearing joins the nearest existing cluster centre within an
angular tolerance, or opens a new cluster with a fresh stable id. The number of
sectors emerges from the data — no k. Centres are nudged toward their members
(exponential moving average) so they track slow sensor bias. Handles the ±π
wrap-around. See the attack-plan spec.
"""
import math

_TWO_PI = 2.0 * math.pi


def _ang_diff(a, b):
    """Signed smallest angle a−b, in (−π, π]."""
    return (a - b + math.pi) % _TWO_PI - math.pi


class SectorMap:
    """Online angular clustering of launch bearings into stable sector ids.

    Args:
        tol_rad: a bearing within this angular distance of an existing centre
            joins that cluster; otherwise a new cluster opens. A *perception*
            threshold (set from radar bearing noise), not a learning knob.
        nudge:   EMA factor for moving a centre toward new members (0 = frozen
            centres, 1 = centre snaps to the latest bearing).
    """

    def __init__(self, tol_rad, nudge=0.2):
        self._tol = tol_rad
        self._nudge = nudge
        self._centers: list[float] = []

    def observe(self, bearing) -> int:
        """Assign ``bearing`` to a sector id, discovering a new one if needed."""
        best_id, best_dist = None, None
        for i, c in enumerate(self._centers):
            d = abs(_ang_diff(bearing, c))
            if best_dist is None or d < best_dist:
                best_id, best_dist = i, d
        if best_id is not None and best_dist <= self._tol:
            # Nudge the centre toward the new bearing (wrap-safe).
            self._centers[best_id] = self._centers[best_id] + self._nudge * _ang_diff(
                bearing, self._centers[best_id]
            )
            return best_id
        self._centers.append(bearing)
        return len(self._centers) - 1

    def bearing(self, sector_id) -> float:
        """Return the current centre bearing of a discovered sector."""
        return self._centers[sector_id]

    def __len__(self) -> int:
        return len(self._centers)
```

- [ ] **Step 4: Run test to verify it passes**

Run: `cd controllers/atlas_controller && python -m pytest tests/test_sector_map.py -v`
Expected: 6 passed.

- [ ] **Step 5: Commit**

```bash
git add controllers/atlas_controller/sector_map.py controllers/atlas_controller/tests/test_sector_map.py
git commit -m "feat(atlas): SectorMap online angular clustering for sector discovery"
```

---

### Task 5: `extract_features` — pure feature vector

Fixed-width feature vector for predicting the next sector: one-hot(current sector) ⊕ [run_length]. Width is fixed to `max_sectors` (a capacity bound) because `SGDClassifier.partial_fit` needs a constant feature dimension and a fixed class set.

**Files:**
- Create: `controllers/atlas_controller/attack_features.py`
- Test: `controllers/atlas_controller/tests/test_attack_features.py`

- [ ] **Step 1: Write the failing test**

```python
# controllers/atlas_controller/tests/test_attack_features.py
"""Tests for extract_features — context features for next-sector prediction."""
import numpy as np
from attack_features import extract_features


def test_width_is_max_sectors_plus_one():
    f = extract_features([0], max_sectors=4)
    assert f.shape == (5,)  # 4 one-hot + 1 run_length


def test_one_hot_marks_current_sector():
    f = extract_features([2, 0, 1], max_sectors=4)
    assert f[1] == 1.0                 # current sector is 1
    assert f[0] == 0.0 and f[2] == 0.0


def test_run_length_counts_trailing_repeats():
    assert extract_features([0], max_sectors=4)[-1] == 1.0
    assert extract_features([0, 0, 0], max_sectors=4)[-1] == 3.0
    assert extract_features([1, 1, 0, 0], max_sectors=4)[-1] == 2.0


def test_run_length_resets_on_switch():
    assert extract_features([2, 2, 2, 1], max_sectors=4)[-1] == 1.0
```

- [ ] **Step 2: Run test to verify it fails**

Run: `cd controllers/atlas_controller && python -m pytest tests/test_attack_features.py -v`
Expected: FAIL — `ModuleNotFoundError: No module named 'attack_features'`.

- [ ] **Step 3: Write minimal implementation**

```python
# controllers/atlas_controller/attack_features.py
"""Feature extraction for next-sector prediction (shared by training & inference).

The feature vector for "given the launch history, which sector launches next?" is
one-hot(current sector) followed by the current run length (consecutive launches
from the current sector). The run-length feature lets a linear model learn that a
longer current burst makes a sector switch more likely. Width is fixed to
``max_sectors`` + 1 so SGDClassifier.partial_fit sees a constant feature
dimension and a fixed class set. See the attack-plan spec.
"""
import numpy as np


def extract_features(history, max_sectors):
    """Return the feature vector for predicting the sector AFTER ``history``.

    Args:
        history:     non-empty list of discovered sector ids (ints), oldest first.
        max_sectors: capacity bound; one-hot width.

    Returns:
        np.ndarray of shape (max_sectors + 1,): one-hot(current) ⊕ [run_length].
    """
    current = history[-1]
    run_length = 1
    for s in reversed(history[:-1]):
        if s == current:
            run_length += 1
        else:
            break
    features = np.zeros(max_sectors + 1, dtype=float)
    features[current] = 1.0
    features[-1] = float(run_length)
    return features
```

- [ ] **Step 4: Run test to verify it passes**

Run: `cd controllers/atlas_controller && python -m pytest tests/test_attack_features.py -v`
Expected: 4 passed.

- [ ] **Step 5: Commit**

```bash
git add controllers/atlas_controller/attack_features.py controllers/atlas_controller/tests/test_attack_features.py
git commit -m "feat(atlas): pure extract_features for next-sector prediction"
```

---

### Task 6: `AttackPredictor` — online learner + ready-aim

Owns the `SectorMap` and the injected classifier. On each observed launch it updates the model online (`partial_fit`) and predicts the next sector. Exposes `get_ready_aim()` for the FSM. Reports no prediction (cold start) until ≥2 sectors have been seen.

**Files:**
- Create: `controllers/atlas_controller/attack_predictor.py`
- Test: `controllers/atlas_controller/tests/test_attack_predictor.py`

- [ ] **Step 1: Write the failing test**

```python
# controllers/atlas_controller/tests/test_attack_predictor.py
"""Tests for AttackPredictor — online next-sector learner feeding the FSM."""
import itertools
import math
import numpy as np
from sklearn.linear_model import SGDClassifier

from attack_predictor import AttackPredictor
from sector_map import SectorMap
from attack_pattern import PatternRule, launch_sequence  # reused from attacker pkg path

MAX_SECTORS = 8
PRE_AIM_TILT = 0.3


class StubModel:
    """Records partial_fit calls; predict returns a fixed sector."""

    def __init__(self, prediction=0):
        self.fits = []
        self._prediction = prediction

    def partial_fit(self, X, y, classes=None):
        self.fits.append((X.tolist(), list(y), None if classes is None else list(classes)))

    def predict(self, X):
        return [self._prediction]


def _bearings_for(sectors):
    """Map sector ids to well-separated bearings for SectorMap to rediscover."""
    table = {0: 0.0, 1: 1.5, 2: 3.0, 3: -1.5}
    return [table[s] for s in sectors]


def test_cold_start_reports_no_prediction():
    """Before two sectors are seen, get_ready_aim() is None."""
    p = AttackPredictor(StubModel(), SectorMap(tol_rad=0.3), max_sectors=MAX_SECTORS,
                        pre_aim_tilt=PRE_AIM_TILT)
    p.observe(bearing=0.0, time=0.0)              # first sector only
    assert p.get_ready_aim() is None


def test_first_partial_fit_passes_classes():
    """The first online update declares the full class set (SGD requirement)."""
    model = StubModel()
    p = AttackPredictor(model, SectorMap(tol_rad=0.3), max_sectors=MAX_SECTORS,
                        pre_aim_tilt=PRE_AIM_TILT)
    p.observe(0.0, 0.0)   # no fit yet (no prior context)
    p.observe(1.5, 1.0)   # now one transition → first partial_fit
    assert len(model.fits) == 1
    assert model.fits[0][2] == list(range(MAX_SECTORS))  # classes passed once


def test_ready_aim_uses_predicted_sector_bearing():
    """get_ready_aim() returns (predicted sector's centre bearing, pre_aim_tilt)."""
    model = StubModel(prediction=1)
    sm = SectorMap(tol_rad=0.3)
    p = AttackPredictor(model, sm, max_sectors=MAX_SECTORS, pre_aim_tilt=PRE_AIM_TILT)
    p.observe(0.0, 0.0)
    p.observe(1.5, 1.0)   # sectors 0 and 1 now seen; model predicts sector 1
    aim = p.get_ready_aim()
    assert aim is not None
    pan, tilt = aim
    assert abs(pan - sm.bearing(1)) < 1e-9
    assert tilt == PRE_AIM_TILT


def test_beats_mode_baseline_on_structured_pattern():
    """A real SGD learner predicts next sector better than always-predict-mode."""
    rule = PatternRule(tour=[0, 1, 2], mean_burst=[5, 2, 4], deviation_prob=0.1)
    sectors = list(itertools.islice(
        launch_sequence([(rule, None)], np.random.default_rng(0)), 400))
    bearings = _bearings_for(sectors)

    model = SGDClassifier(loss="log_loss", random_state=0)
    p = AttackPredictor(model, SectorMap(tol_rad=0.3), max_sectors=MAX_SECTORS,
                        pre_aim_tilt=PRE_AIM_TILT)

    correct = total = 0
    for i, b in enumerate(bearings):
        pred = p.predicted_sector  # prediction made BEFORE seeing this launch
        if pred is not None and i > 50:        # skip warm-up
            total += 1
            correct += int(pred == sectors[i])
        p.observe(b, float(i))

    model_acc = correct / total
    mode = max(set(sectors), key=sectors.count)
    mode_acc = sum(s == mode for s in sectors[51:]) / len(sectors[51:])
    assert model_acc > mode_acc  # learning adds value over the naive baseline


def test_adapts_after_drift():
    """Accuracy recovers after the pattern drifts to a new rule."""
    rule_a = PatternRule(tour=[0, 1], mean_burst=[5, 5], deviation_prob=0.05)
    rule_b = PatternRule(tour=[2, 3], mean_burst=[5, 5], deviation_prob=0.05)
    sectors = list(itertools.islice(
        launch_sequence([(rule_a, 150), (rule_b, None)], np.random.default_rng(1)), 320))
    bearings = _bearings_for(sectors)

    model = SGDClassifier(loss="log_loss", random_state=0)
    p = AttackPredictor(model, SectorMap(tol_rad=0.3), max_sectors=MAX_SECTORS,
                        pre_aim_tilt=PRE_AIM_TILT)

    hits = []  # (index, correct?) after the drift at 150
    for i, b in enumerate(bearings):
        pred = p.predicted_sector
        if pred is not None:
            hits.append((i, int(pred == sectors[i])))
        p.observe(b, float(i))

    just_after = [c for i, c in hits if 150 <= i < 175]
    well_after = [c for i, c in hits if i >= 295]
    assert sum(well_after) / len(well_after) > sum(just_after) / len(just_after)
```

Note: `test_attack_predictor.py` imports `attack_pattern` from the *attacker* package. Add the attacker controller dir to `sys.path` for this test by inserting at the top of the test file, before the `attack_pattern` import:
```python
import os, sys
sys.path.insert(0, os.path.join(
    os.path.dirname(__file__), "..", "..", "attacker_controller"))
```

- [ ] **Step 2: Run test to verify it fails**

Run: `cd controllers/atlas_controller && python -m pytest tests/test_attack_predictor.py -v`
Expected: FAIL — `ModuleNotFoundError: No module named 'attack_predictor'`.

- [ ] **Step 3: Write minimal implementation**

```python
# controllers/atlas_controller/attack_predictor.py
"""AttackPredictor — online learner of the attacker's launch-direction pattern.

The Think layer of the adaptive feature. Fed one launch observation at a time
(bearing + time), it:
  1. discovers the launch sector via an online SectorMap (no preset count),
  2. updates an online classifier (P(next sector | context)) with partial_fit,
  3. predicts the next sector and exposes a ready-aim bearing for the FSM IDLE
     state to pre-slew toward.

It reasons about the *threat stream*, distinct from BallisticTrajectoryPredictor
(per-projectile). There is no dataset and no serialized model — it learns live.
Cold start: until two sectors have been observed (the classifier needs ≥2
classes to be useful), it reports no prediction and the FSM falls back to its
fixed idle aim. See the attack-plan spec.
"""
import numpy as np

from attack_features import extract_features


class AttackPredictor:
    """Online next-sector predictor feeding a ready-aim bearing to the FSM.

    Args:
        model:        an online classifier exposing ``partial_fit(X, y, classes=)``
            and ``predict(X)`` (e.g. sklearn ``SGDClassifier(loss="log_loss")``).
            Injected for testability.
        sector_map:   a ``SectorMap`` for online sector discovery.
        max_sectors:  capacity bound = one-hot width and class set size.
        pre_aim_tilt: tilt (rad) to hold while pre-aiming at the predicted sector.
        min_sectors_seen: cold-start gate — no prediction until this many distinct
            sectors have been observed.
    """

    def __init__(self, model, sector_map, max_sectors, pre_aim_tilt,
                 min_sectors_seen=2):
        self._model = model
        self._sector_map = sector_map
        self._max_sectors = max_sectors
        self._pre_aim_tilt = pre_aim_tilt
        self._min_sectors_seen = min_sectors_seen
        self._history: list[int] = []
        self._seen: set[int] = set()
        self._fitted = False
        self._predicted_sector: int | None = None

    @property
    def predicted_sector(self):
        """The currently predicted next sector id, or None during cold start."""
        return self._predicted_sector

    def observe(self, bearing, time) -> None:
        """Ingest one launch: discover its sector, learn online, re-predict."""
        sector = self._sector_map.observe(bearing)

        # Online update: the just-arrived sector is the label for the context
        # that preceded it. Needs a prior context (history non-empty).
        if self._history:
            X = extract_features(self._history, self._max_sectors).reshape(1, -1)
            y = [sector]
            if not self._fitted:
                self._model.partial_fit(X, y, classes=np.arange(self._max_sectors))
                self._fitted = True
            else:
                self._model.partial_fit(X, y)

        self._history.append(sector)
        self._seen.add(sector)

        # Predict the next sector from the updated context.
        if self._fitted and len(self._seen) >= self._min_sectors_seen:
            X_next = extract_features(self._history, self._max_sectors).reshape(1, -1)
            self._predicted_sector = int(self._model.predict(X_next)[0])
        else:
            self._predicted_sector = None

    def get_ready_aim(self):
        """Return (pan, tilt) toward the predicted sector, or None (cold start).

        ``pan`` is the predicted sector's discovered centre bearing (the turret's
        pan convention is pan = atan2(dx, dy), the same value SectorMap stores);
        ``tilt`` is the configured pre-aim elevation.
        """
        if self._predicted_sector is None:
            return None
        return (self._sector_map.bearing(self._predicted_sector), self._pre_aim_tilt)
```

- [ ] **Step 4: Run test to verify it passes**

Run: `cd controllers/atlas_controller && python -m pytest tests/test_attack_predictor.py -v`
Expected: 5 passed. If `test_beats_mode_baseline_on_structured_pattern` or `test_adapts_after_drift` is flaky, widen the warm-up / window or lower the margin — they assert a *direction* (ML > mode; later > earlier), not a fixed accuracy. Do not weaken them to trivially true.

- [ ] **Step 5: Commit**

```bash
git add controllers/atlas_controller/attack_predictor.py controllers/atlas_controller/tests/test_attack_predictor.py
git commit -m "feat(atlas): AttackPredictor online next-sector learner with ready-aim"
```

---

### Task 7: FSM `IDLE` consumes the ready-aim (pure)

Add `attack_predictor` to `SensorSuite` (default `None` for back-compat) and make `_do_idle` pre-aim at the predicted sector when one is available, else hold the fixed idle beam. The FSM stays pure — it calls a method returning a value, holding no model.

**Files:**
- Modify: `controllers/atlas_controller/fsm.py:16-30` (`SensorSuite`), `fsm.py:221-253` (`_do_idle`)
- Test: `controllers/atlas_controller/tests/test_fsm.py` (add cases)

- [ ] **Step 1: Write the failing test**

Inspect `tests/test_fsm.py` for its existing SensorSuite/stub construction helper (commonly a fixture or a `_make_fsm(...)`). Reusing that, add:

```python
class _StubPredictor:
    def __init__(self, ready_aim):
        self._ready_aim = ready_aim
    def get_ready_aim(self):
        return self._ready_aim


def test_idle_preaims_at_predicted_sector_bearing(make_idle_fsm):
    """When the predictor returns a ready-aim, IDLE commands that pan/tilt."""
    fsm = make_idle_fsm(attack_predictor=_StubPredictor((1.234, 0.3)))
    fsm.step()  # IDLE
    pan, tilt = fsm.commanded_aim
    assert abs(pan - 1.234) < 1e-9
    assert abs(tilt - 0.3) < 1e-9


def test_idle_falls_back_to_fixed_beam_when_no_prediction(make_idle_fsm):
    """No prediction → IDLE holds the configured idle_pan/idle_tilt."""
    fsm = make_idle_fsm(attack_predictor=_StubPredictor(None))
    fsm.step()
    pan, tilt = fsm.commanded_aim
    assert abs(pan - fsm.config.idle_pan) < 1e-9
    assert abs(tilt - fsm.config.idle_tilt) < 1e-9


def test_idle_fixed_beam_when_predictor_absent(make_idle_fsm):
    """Default SensorSuite has no predictor → original fixed-beam behaviour."""
    fsm = make_idle_fsm()  # attack_predictor defaults to None
    fsm.step()
    pan, tilt = fsm.commanded_aim
    assert abs(tilt - fsm.config.idle_tilt) < 1e-9
```

If `test_fsm.py` builds the FSM inline rather than via a `make_idle_fsm` helper, write a small local helper in the test that constructs a `SensorSuite` with stub fields (mirroring the existing tests) and accepts an optional `attack_predictor`, plus a `TurretHardware` with recording motor stubs, and returns an `AtlasFSM`. Use the stub-motor pattern already in the file so `commanded_aim` reflects the issued angles.

- [ ] **Step 2: Run test to verify it fails**

Run: `cd controllers/atlas_controller && python -m pytest tests/test_fsm.py -k "idle_preaims or fallback or predictor_absent" -v`
Expected: FAIL — `SensorSuite.__init__()` rejects `attack_predictor` (unexpected keyword) OR the pre-aim assertion fails (IDLE still commands the fixed beam).

- [ ] **Step 3: Add the SensorSuite field**

In `controllers/atlas_controller/fsm.py`, add to the `SensorSuite` dataclass (after `bullet_hit_link`):
```python
    # Optional Think-layer predictor providing a pre-aim suggestion for IDLE.
    # get_ready_aim() -> (pan, tilt) | None. Default None keeps the original
    # fixed-beam IDLE behaviour (and existing tests) working.
    attack_predictor: object = None
```

- [ ] **Step 4: Use the ready-aim in _do_idle**

In `_do_idle`, replace the fixed-aim line:
```python
        # Hold the fixed idle direction.
        self._command_angles(self.config.idle_pan, self.config.idle_tilt)
```
with:
```python
        # Pre-aim at the sector the AttackPredictor expects next (Layer-3
        # strategic suggestion); fall back to the fixed idle beam when there is
        # no prediction (cold start) or no predictor wired. The FSM stays pure:
        # it consumes a (pan, tilt) value, not a model.
        ready_aim = None
        if self.sensors.attack_predictor is not None:
            ready_aim = self.sensors.attack_predictor.get_ready_aim()
        if ready_aim is not None:
            self._command_angles(ready_aim[0], ready_aim[1])
        else:
            self._command_angles(self.config.idle_pan, self.config.idle_tilt)
```
Also update the `_do_idle` docstring's first paragraph to note it pre-aims at the predicted sector when available, else holds the fixed beam.

- [ ] **Step 5: Run tests to verify they pass**

Run: `cd controllers/atlas_controller && python -m pytest tests/test_fsm.py -v`
Expected: all pass (existing + 3 new). The existing tests must stay green — `attack_predictor` defaulting to `None` preserves prior behaviour.

- [ ] **Step 6: Commit**

```bash
git add controllers/atlas_controller/fsm.py controllers/atlas_controller/tests/test_fsm.py
git commit -m "feat(atlas): IDLE pre-aims at the predicted launch sector"
```

---

### Task 8: Wire `AttackPredictor` into the ATLAS controller

Construct the predictor, derive the clustering tolerance from radar bearing noise, detect the launch observation edge (first acquisition after a resolution cue), feed the predictor before `fsm.step()`, and pass it into `SensorSuite`.

**Files:**
- Modify: `controllers/atlas_controller/atlas_controller.py`

- [ ] **Step 1: Add imports and constants**

In `atlas_controller.py`, add to the imports:
```python
from sklearn.linear_model import SGDClassifier
from sector_map import SectorMap
from attack_predictor import AttackPredictor
```
Add to the tuning-constants block:
```python
# --- Adaptive launch-direction learner ---
# The Search Radar position noise (≈0.2 m, see search_radar_controller) at the
# ~10 m launch range subtends a small bearing noise; the sector-clustering
# tolerance is a few × that, comfortably below the spacing between real sectors.
# Derived, not hand-tuned.
SEARCH_RADAR_NOISE_STD_M = 0.2
NOMINAL_LAUNCH_RANGE_M = 10.0
SECTOR_TOL_RAD = 4.0 * SEARCH_RADAR_NOISE_STD_M / NOMINAL_LAUNCH_RANGE_M  # ≈0.08 rad
MAX_SECTORS = 8                 # capacity bound (fixed feature width / class set)
PRE_AIM_TILT_RAD = 0.3          # elevation held while pre-aiming at a sector
```

- [ ] **Step 2: Construct the predictor and add it to the SensorSuite**

After `bullet_hit_link = BulletHitLink(bullet_hit_receiver)`, add:
```python
# Adaptive launch-direction learner (Think layer). Online logistic regression
# over self-discovered sectors; no dataset, no serialized model.
attack_predictor = AttackPredictor(
    model=SGDClassifier(loss="log_loss", random_state=0),
    sector_map=SectorMap(tol_rad=SECTOR_TOL_RAD),
    max_sectors=MAX_SECTORS,
    pre_aim_tilt=PRE_AIM_TILT_RAD,
)
```
Add `attack_predictor=attack_predictor` to the `SensorSuite(...)` construction.

- [ ] **Step 3: Initialise the observation-edge state before the loop**

Just before `while robot.step(timestep) != -1:` add:
```python
# Launch observation gating: a launch is the FIRST radar acquisition after the
# previous projectile resolved. _predictor_armed re-arms on each resolution cue
# (ground/bullet hit) so each engagement yields exactly one launch observation,
# debouncing mid-flight detection dropouts. Armed at start for the first launch.
predictor_armed = True
```

- [ ] **Step 4: Feed the predictor inside the loop, before fsm.step()**

In the main loop, after the `bullet_hit_link.update()` / logging block and **before** `fcr.update(...)` (it only needs the cue + resolution flags), add:
```python
    # Adaptive launcher observation: re-arm on a resolution cue, then record the
    # next fresh Search Radar acquisition as one launch (bearing = where it was
    # first seen ≈ its launch sector). Fed before fsm.step() so IDLE pre-aims
    # using the freshest prediction.
    if ground_hit_link.hit_this_step() or bullet_hit_link.hit_this_step():
        predictor_armed = True
    cue = cue_link.get_cue()
    if predictor_armed and cue is not None:
        rel_x = cue[0] - turret_position[0]
        rel_y = cue[1] - turret_position[1]
        launch_bearing = math.atan2(rel_x, rel_y)  # pan convention
        attack_predictor.observe(launch_bearing, robot.getTime())
        predictor_armed = False
        log.info(
            "[ATLAS] launch observed: bearing=%.3f rad → predicted next sector %s",
            launch_bearing,
            attack_predictor.predicted_sector,
        )
```

- [ ] **Step 5: Smoke-check the controller imports**

Run:
```bash
cd controllers/atlas_controller && ../../.venv/Scripts/python -c "import sector_map, attack_features, attack_predictor; from sklearn.linear_model import SGDClassifier; print('atlas modules import ok')"
```
Expected: prints `atlas modules import ok`.

- [ ] **Step 6: Run the full ATLAS test suite (nothing regressed)**

Run: `cd controllers/atlas_controller && python -m pytest tests/ -v`
Expected: all pass.

- [ ] **Step 7: Manual verification in Webots**

Open the world in Webots and run the simulation. Confirm in `atlas_telemetry.log` / console:
- `[ATTACKER] launching from sector N` lines cycle with burst structure and switch tour after ~40 launches.
- `[ATLAS] launch observed: bearing=… → predicted next sector …` appears once per engagement (not multiple times mid-flight).
- After a warm-up the turret, while idle between engagements, **pre-rotates** toward the next sector before the projectile is acquired; and after the mid-run drift it re-orients to the new pattern within a few bursts.

This is the integration check the unit tests cannot make; note any tuning of `SECTOR_AZIMUTHS_RAD`, `SECTOR_TOL_RAD`, `PRE_AIM_TILT_RAD`, or the SGD learning rate needed for the behaviour to read clearly on screen.

- [ ] **Step 8: Commit**

```bash
git add controllers/atlas_controller/atlas_controller.py
git commit -m "feat(atlas): wire AttackPredictor — radar-only launch obs + IDLE pre-aim"
```

---

### Task 9: Documentation — glossary + ADR

**Files:**
- Modify: `CONTEXT.md` (the `## AttackPredictor` entry)
- Create: `docs/adr/0014-adaptive-launch-direction-learning.md`

- [ ] **Step 1: Sharpen the AttackPredictor glossary entry**

Replace the `## AttackPredictor` body in `CONTEXT.md` with:
```markdown
## AttackPredictor

Online learning subsystem on the ATLAS side. Observes each launch radar-only
(first Search Radar acquisition after a resolution cue), discovers the launch
sectors by online angular clustering (`SectorMap`), and learns
`P(next launch sector | recent context)` with an online logistic regression
(`SGDClassifier.partial_fit`). Predicts the **next launch sector** and exposes a
ready-aim bearing the FSM `IDLE` state uses to pre-slew the turret. Re-adapts when
the attacker's pattern drifts. Reasons about the *threat stream*, distinct from
the `BallisticTrajectoryPredictor` (individual projectile trajectories). No
dataset and no serialized model — it learns live.
```

- [ ] **Step 2: Write the ADR**

```markdown
# 0014 — Adaptive launch-direction learning (online, radar-only)

## Status
Accepted

## Context
The Advanced / Contextual Component (10%) requires a genuine "AI or Adaptive
Behaviour" element. Earlier drafts predicted launch *cadence* offline and/or with
synthetic data; that read as offline analysis, not adaptive behaviour, and risked
being a lookup rather than learning.

## Decision
ATLAS learns the attacker's launch-*direction* pattern online during the run:
- The attacker launches from multiple sectors on a structured-but-noisy,
  drifting pattern, designed so a statistical learner is required and beats naive
  baselines.
- ATLAS observes launches radar-only (first acquisition after a resolution cue) —
  no launch-announcement radio channel — and discovers the sectors itself by
  online angular clustering (no preset count/boundaries; tolerance derived from
  radar noise).
- An online logistic regression (`SGDClassifier`, log-loss, `partial_fit`)
  estimates `P(next sector | context)`; its learning rate provides forgetting so
  it tracks drift. One meaningful hyperparameter (the adaptation rate).
- The prediction feeds the pure FSM `IDLE` state as a ready-aim bearing
  (Layer-3 strategic suggestion → Layer-2 FSM); the five-state structure
  (ADR-0012) is unchanged.
- No synthetic data, no collected dataset, no serialized model — learned live.

## Consequences
- Adds a `scikit-learn` dependency.
- The "value of ML" is measurable: prediction accuracy beats predict-last /
  predict-mode baselines, and recovers after a mid-run drift.
- Cold start (no prediction until ≥2 sectors seen) falls back to the fixed idle
  beam — a safety/robustness property.
- Launch timing is approximated by first-acquisition time (small constant lag),
  acceptable because direction, not timing, is the headline.
```

- [ ] **Step 3: Commit**

```bash
git add CONTEXT.md docs/adr/0014-adaptive-launch-direction-learning.md
git commit -m "docs: AttackPredictor glossary + ADR-0014 adaptive launch-direction learning"
```

---

## Self-Review

**Spec coverage:**
- Predict direction / pre-aim → Tasks 6, 7, 8. ✓
- Structured + noisy + drifting attacker pattern → Task 2 (+ wiring Task 3). ✓
- No new radio channel / radar-only observation segmented by resolution cues → Task 8 (edge gating). ✓
- Self-discovered sectors, tol from radar noise → Tasks 4, 8. ✓
- Online logistic regression with forgetting, no dataset/artifact → Tasks 1, 6. ✓
- Run-length feature → Task 5. ✓
- Cold-start UNKNOWN fallback → Tasks 6, 7. ✓
- "ML beats baseline" + post-drift recovery evidence → Task 6 tests. ✓
- FSM purity / ADR-0012 unchanged → Task 7 (value not model; only IDLE input). ✓
- Glossary + report linkage → Task 9. ✓

**Known deviation from spec wording:** the spec says the one-hot width "tracks the number of clusters discovered so far." `SGDClassifier.partial_fit` requires a fixed feature dimension and a fixed class set, so the plan fixes the width to a capacity bound `MAX_SECTORS` (Task 5/6) instead. Behaviourally equivalent (unused sectors never fire); documented in the code comments and ADR.

**Type/name consistency:** `extract_features(history, max_sectors)` (Task 5) is called with `max_sectors=MAX_SECTORS` in `AttackPredictor` (Task 6); `SectorMap.observe/bearing/__len__` (Task 4) used as such in Task 6; `AttackPredictor.observe(bearing, time)`, `.predicted_sector`, `.get_ready_aim()` (Task 6) match the controller (Task 8) and FSM/test usage (Task 7); `launch_sequence(phases, rng)` + `PatternRule(tour, mean_burst, deviation_prob)` (Task 2) match attacker wiring (Task 3) and predictor tests (Task 6); `sector_launch(...)` signature (Task 3) matches its caller. Consistent.
