# FCR Engagement Fixes — Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development
> (recommended) or superpowers:executing-plans to implement this plan task-by-task.
> Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Make the ATLAS FSM reach `ENGAGING` by fixing the two confirmed bugs
that stop the Fire-Control Radar locking a cued target.

**Architecture:** Two independent fixes. Fix 1 corrects a coordinate-frame
mismatch in the FSM's ground gate (unit-testable, deterministic). Fix 2 raises
the Search Radar's scan revisit rate so the cue feeding the FCR is refreshed
often enough for the FCR's narrow cone to lock (parameter change, Webots-tuned).

**Tech stack:** Python 3.13, pytest, uv. Webots controllers under
`controllers/`. Tests live beside each controller in `tests/` with a `conftest.py`
that wires `sys.path` — run pytest pointed at the controller's `tests/` directory.

**Source spec:** `docs/superpowers/specs/2026-05-19-fcr-engage-fix-design.md`
(read it first — it carries the root-cause analysis and design decisions D1–D3).

---

## Task overview, models, and parallelism

| Task | Scope | Files | Model · effort | Depends on |
|------|-------|-------|----------------|------------|
| 1 | Fix 1a — world-frame ground check | `fsm.py`, `tests/test_fsm.py` | Sonnet · medium | — |
| 2 | Fix 1b — telemetry + constant docs | `atlas_controller.py` | Sonnet · low | — |
| 3 | Fix 2 — Search Radar revisit rate | `search_radar_controller.py`, `tests/test_search_radar.py` | Sonnet · medium | — |
| 4 | Full-suite verification + commit | (none) | Sonnet · low | 1, 2, 3 |

**Parallelism:** Tasks 1, 2, and 3 touch disjoint files and share no state —
**dispatch all three in parallel.** Task 4 is the join point: run it only after
1–3 are all complete and individually green. Suggested agent type for Tasks 1–3:
`atlas-controls-engineer`.

**Shared convention (Tasks 1 and 2 must agree).** "Ground" is world Z = 0.
Turret-relative height is converted to world frame by adding the turret's world
Z: `world_z = relative_z + turret_position[2]`. The comparison threshold
(`FSMConfig.ground_threshold`, default 0.1) stays world-framed. Both tasks
implement this same formula independently — do not change the formula in one
without the other.

---

## Task 1: Fix 1a — frame-consistent ground check

**What and why.** `AtlasFSM._target_in_range` (`fsm.py:535-551`) currently tests
the turret-*relative* height `dz` against the world-framed `ground_threshold`.
Because the turret base sits at world Z = 0.05 m, this lifts the effective
ground plane to 0.15 m and rejects valid low intercepts — `PREDICT` then bounces
back to `TRACK` and `AIMING`/`ENGAGING` become unreachable (issue #19). The fix
converts the height to world frame before the comparison.

**Files:**
- Modify: `controllers/atlas_controller/fsm.py` — `_target_in_range` (the
  `dz > ground_threshold` test) and the `FSMConfig.ground_threshold` docstring
  (`fsm.py:51`).
- Modify: `controllers/atlas_controller/tests/test_fsm.py` — the `_target_in_range`
  test group (around lines 442–466).

- [ ] **Step 1: Add the failing regression test for the issue #19 case.**
  In `test_fsm.py`, add a test asserting that `_target_in_range` **accepts** an
  intercept whose turret-relative `dz` is *below* `ground_threshold` but whose
  *world* height (`dz + turret_position[2]`) is *above* it. Concretely: with the
  turret at world Z = 0.05 and `ground_threshold = 0.1`, an intercept with
  `dz = 0.07` has world height 0.12 (valid) — current code rejects it, so the
  test fails today. Use the existing `_make_fsm()` helper / `TURRET_POS` constant;
  if `TURRET_POS` does not already have a non-zero Z, construct a `TurretHardware`
  with `turret_position` Z = 0.05 so the bug is exercised. Also add (or confirm)
  a companion assertion that a genuinely sub-ground intercept — world height
  below `ground_threshold` — is still **rejected**.

- [ ] **Step 2: Run the new test and confirm it fails.**
  `uv run pytest controllers/atlas_controller/tests/test_fsm.py -k target_in_range -v`
  Expected: the new "world height above threshold" test FAILS (returns False
  where True is expected); the existing `target_in_range` tests still pass.

- [ ] **Step 3: Implement the world-frame conversion.**
  In `_target_in_range`, convert the height operand to world frame
  (`dz + self.hardware.turret_position[2]`) and compare *that* against
  `self.config.ground_threshold`. The `max_range` distance check is unchanged.

- [ ] **Step 4: Correct the `FSMConfig.ground_threshold` docstring.**
  State that the threshold is world-framed and that `_target_in_range` converts
  the (turret-relative) operand to world frame at the comparison site before
  testing it. Also update the `_target_in_range` docstring, which currently
  claims "Its world-Z component (rel_position[2])" — `rel_position[2]` is
  turret-relative, not world-Z; correct that wording.

- [ ] **Step 5: Run the full FSM test file and confirm green.**
  `uv run pytest controllers/atlas_controller/tests/test_fsm.py -v`
  Expected: all tests PASS, including the new regression test. Check that the
  existing `_do_predict` tests (`test_predict_below_ground_intercept_*`,
  `test_predict_valid_intercept_*`) still pass — those exercise `_target_in_range`
  through PREDICT and their `_make_fsm_in_predict` helper uses `TURRET_POS`. If
  any now fail, the helper's turret Z and intercept values need reconciling with
  the new world-frame semantics; adjust the *test inputs*, not the production
  logic.

- [ ] **Step 6: Commit.**
  Commit `fsm.py` and `test_fsm.py` together with a message describing the
  frame-mismatch fix and citing issue #19.

**Acceptance:** `_target_in_range` gates on world height; the issue #19 case
(valid low intercept) is accepted; genuine sub-ground intercepts still rejected;
full `test_fsm.py` green.

---

## Task 2: Fix 1b — telemetry alignment and constant documentation

**What and why.** The controller's per-step telemetry computes `above_ground`
(`atlas_controller.py:247`) the same wrong way the FSM gate used to —
`icept[2] > fsm.config.ground_threshold` on a turret-relative height. Once Task 1
lands, the log would disagree with the gate. This task realigns the telemetry so
the `ABOVE-GROUND`/`BELOW-GROUND` line matches the FSM's actual decision, and
documents why two ground-related constants coexist.

**Files:**
- Modify: `controllers/atlas_controller/atlas_controller.py` — the `above_ground`
  telemetry computation (line 247) and the `GROUND_HIT_THRESHOLD_M` definition
  (line 35).

- [ ] **Step 1: Apply the world-frame conversion to the telemetry.**
  Change the `above_ground` computation so the intercept height is converted to
  world frame before the `ground_threshold` comparison — the same formula as
  Task 1 (`world_z = icept[2] + turret_position[2]`). The controller already
  holds `turret_position` (used elsewhere in the telemetry block). The logged
  value and the FSM gate must now agree.

- [ ] **Step 2: Document the two-threshold distinction.**
  `GROUND_HIT_THRESHOLD_M` (0.05, world) and `FSMConfig.ground_threshold` (0.1)
  are *not* duplicates and must not be unified. Add a comment at the
  `GROUND_HIT_THRESHOLD_M` definition explaining the split:
  `GROUND_HIT_THRESHOLD_M` answers "has the projectile physically landed?" — it
  drives the relaunch logic (`atlas_controller.py:285-289`).
  `ground_threshold` answers "is a predicted intercept high enough to be worth
  aiming at?" — it drives the FSM gate. Different questions, different values,
  intentionally separate.

- [ ] **Step 3: Sanity-check the controller still parses.**
  `uv run python -c "import ast; ast.parse(open('controllers/atlas_controller/atlas_controller.py').read())"`
  Expected: no output, exit 0. (`atlas_controller.py` is a Webots entry-point
  module — it has no unit tests and cannot run outside Webots, so an AST parse is
  the available static check.)

- [ ] **Step 4: Commit.**
  Commit `atlas_controller.py` with a message noting the telemetry now matches
  the Task 1 gate and the constant distinction is documented.

**Acceptance:** telemetry `above_ground` uses the world-frame height; the
`GROUND_HIT_THRESHOLD_M` vs `ground_threshold` distinction is documented; module
parses.

---

## Task 3: Fix 2 — raise the Search Radar revisit rate

**What and why.** The Search Radar scans a narrow beam (`beam_width = 0.35 rad`)
at `scan_rate = 0.15 rad/step` — a full revolution every ~42 steps (~1.3 s),
longer than the ~1.2 s projectile flight. The beam crosses the ball at most
once or twice per engagement, so the cue feeding the FCR is fresh only briefly
and then frozen (held by `SearchRadarLink`) while the ball flies on — the FCR's
narrow cone never catches it and the FSM stalls in `ACQUIRE`. The fix raises the
revisit rate so the beam crosses the ball several times per flight.

**The dwell coupling (read before changing numbers).** Faster scanning shrinks
the beam *dwell* — the steps the ball spends inside the beam ≈
`beam_width / scan_rate`. Below ~1 step of dwell the beam can step *past* the
ball between timesteps and emit no fresh cue at all. So `scan_rate` and
`beam_width` must be raised **together** to hold dwell at ≈ 2+ steps. The
invariant is `beam_width ≥ 2 × scan_rate`.

**Files:**
- Modify: `controllers/search_radar_controller/search_radar_controller.py` —
  `SEARCH_RADAR_SCAN_RATE_RAD_PER_STEP` (line 42), `SEARCH_RADAR_BEAM_WIDTH_RAD`
  (line 41), and add a dwell-safety guard near those constants.
- Modify: `controllers/search_radar_controller/tests/test_search_radar.py` — add
  a test in the rotating-scan-beam group (around lines 477–527).
- Do **not** modify `search_radar.py` or `search_radar_link.py` — the buffered-
  `Detection` freeze and the hold-last-cue policy are both correct (see spec).

- [ ] **Step 1: Add a failing test that the new scan rate does not skip an
  in-beam target.** In `test_search_radar.py`, add a test that builds a radar
  with the *new* `scan_rate` (0.30) and `beam_width` (0.70) and a stationary
  target placed on the beam's path, advances the beam across the target over a
  sweep, and asserts the target is detected on at least ~2 consecutive
  `update()` steps (dwell ≥ 2). Model it on the existing
  `test_beam_sweep_discovers_target_then_loses_it` / `test_target_within_beam_is_detected`
  patterns and the `_make_radar` helper. Written against the new constants this
  test passes once Step 2 lands; written first it documents the dwell intent.
  (If preferred, express it as a parametrised check that the shipped
  `beam_width ≥ 2 × scan_rate` invariant holds — either form is acceptable;
  the point is a regression guard on dwell.)

- [ ] **Step 2: Raise the two constants together.**
  Set `SEARCH_RADAR_SCAN_RATE_RAD_PER_STEP = 0.30` and
  `SEARCH_RADAR_BEAM_WIDTH_RAD = 0.70`. Update the inline comments to record
  that the two are coupled by the dwell invariant and that the values are a
  Webots-tuned starting point (revisit ~21 steps, dwell ~2.3 steps) — see the
  spec's Fix 2 and Risk sections.

- [ ] **Step 3: Add the dwell-safety guard.**
  At module scope, after the two constants are defined, check
  `SEARCH_RADAR_BEAM_WIDTH_RAD >= 2 * SEARCH_RADAR_SCAN_RATE_RAD_PER_STEP`. If it
  fails, emit a `log.warning` naming both values and the dwell risk. Use the
  controller's existing `log` logger. A warning (not a hard failure) is right
  here: the controller must still start so a human can tune in Webots.

- [ ] **Step 4: Run the Search Radar tests and confirm green.**
  `uv run pytest controllers/search_radar_controller/tests/test_search_radar.py -v`
  Expected: all tests PASS, including the new dwell test. Pay attention to
  `test_beam_advances_by_scan_rate_each_update` and `test_beam_sweep_discovers_target_then_loses_it`
  — they pass `scan_rate` explicitly to `_make_radar` so they are unaffected by
  the constant change, but confirm they still pass.

- [ ] **Step 5: Sanity-check the controller still parses.**
  `uv run python -c "import ast; ast.parse(open('controllers/search_radar_controller/search_radar_controller.py').read())"`
  Expected: no output, exit 0.

- [ ] **Step 6: Commit.**
  Commit `search_radar_controller.py` and `test_search_radar.py` together with a
  message describing the revisit-rate fix and citing issues #21 and #17.

**Acceptance:** revisit rate roughly doubled with dwell held at ≈ 2+ steps; the
dwell invariant is guarded; `test_search_radar.py` green; `search_radar.py` and
`search_radar_link.py` untouched.

---

## Task 4: Full-suite verification and integration

**What and why.** Tasks 1–3 commit independently; this task confirms nothing
regressed across the whole project and records the verification evidence.

- [ ] **Step 1: Run the full atlas_controller test suite.**
  `uv run pytest controllers/atlas_controller/tests/ -v`
  Expected: all tests PASS.

- [ ] **Step 2: Run the full search_radar_controller test suite.**
  `uv run pytest controllers/search_radar_controller/tests/ -v`
  Expected: all tests PASS.

- [ ] **Step 3: Record the results.**
  Capture the pass counts from Steps 1–2. If anything fails, do not paper over
  it — diagnose and fix the cause, or report it back. Per
  superpowers:verification-before-completion, no success claim without the
  command output to back it.

- [ ] **Step 4: Hand off the manual check.**
  Unit tests cannot confirm an end-to-end lock. Note for the human reviewer the
  Webots checks from the spec's *Verification* section: the FSM reaches
  `ENGAGING`, the FCR locks the cued ball, the issue #19 below-ground misreport
  is gone, and the Fix 2 `scan_rate`/`beam_width` pair is tuned if the starting
  values do not yield a lock.

**Acceptance:** both suites green with output captured; manual Webots checklist
handed to the reviewer.

---

## Self-review notes

- **Spec coverage:** Fix 1 → Tasks 1 (gate) + 2 (telemetry, constant docs);
  Fix 2 → Task 3. The spec's *Out of scope* items (FCR acquisition scan, radar
  refactor, vertical-FOV gate, docstring cleanups) are deliberately absent —
  the docstring cleanups are already committed (2793e37).
- **No production change is driven by a test failure caused by stale test
  inputs** — Task 1 Step 5 and Task 3 Step 4 explicitly call out which existing
  tests to re-check and that test *inputs* (not production logic) get adjusted
  if helpers carry frame-dependent fixtures.
- **Frame convention** is stated once up front and referenced by Tasks 1 and 2
  so the parallel agents cannot diverge.
