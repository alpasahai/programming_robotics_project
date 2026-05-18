# Physical Search Radar — Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Make the Search Radar a physically grounded Webots node with a field-of-view gate and a rotating scan beam, so it is visually clear where the radar is and what it can see.

**Architecture:** The `SearchRadar` model gains a world *pose* (read from a new Webots node) and detects a projectile only when it is within range, within the vertical field of view, and within the rotating scan beam's current azimuth sector. A short-lived track buffer bridges the gaps between beam sweeps. Output stays on the existing `Detection` interface (turret-relative position), so the FSM and `TrackFilter` are untouched. The model runs in-process inside the ATLAS controller — no separate controller or radio link in this increment.

**Tech stack:** Python 3.13 (see `.python-version` / `pyproject.toml`), pytest, numpy/filterpy (already present), Webots R2025a. No new dependencies.

**Spec:** `docs/superpowers/specs/2026-05-17-radar-perception-design.md` — Increment 1.

---

## Prerequisites

- **Issue #9 must be complete.** The `.wbt` scene structures are expected to live as `protos/*.proto` files instantiated by `worlds/ATLA_v1.wbt`. This plan adds the new radar as `protos/SearchRadar.proto`. If #9 produced different conventions (file naming, directory, how the `.wbt` references protos), adjust Task 5 to match what #9 actually created — inspect an existing proto first and copy its structure.

## How to work this plan

- **Execution mode: subagent-driven.** An Opus orchestrator dispatches one fresh subagent per task, reviews the result, then dispatches the next. Use the `superpowers:subagent-driven-development` sub-skill.
- Each task is one reviewable unit. Within a task, follow red-green-refactor: write the failing test, confirm it fails for the expected reason, implement the minimum, confirm green, commit.
- Run tests from `controllers/atlas_controller/`: `python -m pytest tests/ -v`. `conftest.py` puts both `tests/` and the controller dir on `sys.path`.
- Webots controller glue and the world/proto files are not unit-tested (matches the project's existing pattern — the controller is documented as untested glue). They get explicit manual-verification steps in Webots instead.
- Commit after every task with a conventional-commit message.

### Model and effort policy

Each task carries a **Model · Effort** tag for the subagent that executes it. Goal: cheapest model and lowest reasoning effort that will not degrade quality.

**Model** — capability tier:
- **haiku** (Codex: `gpt-5.4-mini`) — fully specified, mechanical, self-contained. Tests/interfaces are pinned by the plan; little judgment needed.
- **sonnet** (Codex: `gpt-5.4`) — moderate reasoning: rewriting a class under an interface-preservation constraint, non-trivial logic, Webots/proto syntax.
- **opus** (Codex: `gpt-5.5`) — judgment calls, cross-cutting changes, ambiguity. Also: **the orchestrator is always Opus**, and **every between-task review is done by Opus** regardless of the task's execution model.
- **Codex orchestration:** If you're Codex and you're orchestrating this plan, use the Codex models instead: `gpt-5.5` for the orchestrator/reviews, `gpt-5.4` where the plan says `sonnet`, and `gpt-5.4-mini` where the plan says `haiku`.

**Effort** — how much reasoning the subagent spends, set independently of model:
- **low** — transcription-level: the plan dictates the code/content; follow it.
- **medium** — the subagent must design small pieces of logic within a clear spec.
- **high** — the subagent must reason through edge cases or cross-file effects. Rare in this plan.

**Rules:**
- **Tuning is never delegated to haiku.** Webots parameter tuning (FOV/scan/slew values until the FSM behaves) is iterative judgment — the orchestrator (Opus) or the human does it, even within a sonnet-tagged task.
- If a subagent on a cheaper model or lower effort hits ambiguity or the task turns out under-specified, it should stop and escalate to the orchestrator rather than guess.

---

## File structure

| File | Status | Responsibility |
|---|---|---|
| `controllers/atlas_controller/geometry.py` | create | Pure world-frame angle helpers (azimuth/elevation/range, angle differencing). No Webots import. |
| `controllers/atlas_controller/tests/test_geometry.py` | create | Tests for `geometry.py`. |
| `controllers/atlas_controller/search_radar.py` | rewrite | `SearchRadar` with pose, FOV gate, rotating scan, track buffer. `Detection` NamedTuple kept as-is. |
| `controllers/atlas_controller/tests/test_search_radar.py` | rewrite | Tests for the new `SearchRadar` (existing tests assume the old constructor). |
| `protos/SearchRadar.proto` | create | Webots PROTO for the radar body (mast + antenna). |
| `worlds/ATLA_v1.wbt` | modify | Instantiate `DEF SEARCH_RADAR SearchRadar { … }`. |
| `controllers/atlas_controller/atlas_controller.py` | modify | Read the `SEARCH_RADAR` node pose; construct `SearchRadar` with FOV/scan parameters. |
| `docs/adr/0007-physical-search-radar.md` | create | Records this increment. |
| `CONTEXT.md` | modify | Rewrite the `## Search Radar` glossary entry. |

---

## Task 1 — Geometry helpers

**Model: haiku (Codex: `gpt-5.4-mini`) · Effort: low** — pure functions, tests fully specified in the plan, no judgment.

**Required:** Both radars need to reason about a target's direction relative to a sensor pose. There is no shared place for that today (the FSM inlines its own `atan2` math). Extract it once so both this plan and the FCR plan reuse it.

**What to build:** A `geometry.py` module with two pure functions:

- `azimuth_elevation_range(observer, target) -> (azimuth, elevation, range)` — vector from `observer` to `target` in world frame; Z-up ENU convention matching `fsm.py:_compute_aim_angles` (`azimuth = atan2(dx, dy)`, `elevation = atan2(dz, sqrt(dx²+dy²))`).
- `angle_diff(a, b) -> float` — shortest signed difference `a - b` wrapped to `[-π, π]` (use `atan2(sin(d), cos(d))`).

**Approach (TDD):**
- [ ] Write `tests/test_geometry.py` covering: a target due +Y → azimuth 0; due +X → azimuth +π/2; directly overhead → elevation +π/2; a non-zero observer offset is subtracted before the angle is taken; `angle_diff` returns the short way around and never exceeds π in magnitude.
- [ ] Run the tests, confirm they fail with `ModuleNotFoundError: geometry`.
- [ ] Implement `geometry.py` with the two functions and clear docstrings.
- [ ] Run the tests, confirm green.
- [ ] Commit: `feat(geometry): add world-frame azimuth/elevation/range helpers`.

---

## Task 2 — SearchRadar gains a pose and FOV gate

**Model: sonnet (Codex: `gpt-5.4`) · Effort: medium** — class rewrite under a hard interface-preservation constraint (`Detection` output unchanged); needs care, not just transcription.

**Required:** The radar must know its own world position and detect a projectile only when it is in range and within the vertical field of view — not omnisciently as today.

**Current state:** `SearchRadar.__init__(projectiles, turret_position, noise_std, timestep_ms, rng)`. `update()` records every projectile unconditionally. `Detection(track_id, position)` carries a turret-relative position.

**What to change:**
- New constructor parameters: `radar_position` (world `[x,y,z]`), `max_range`, `vertical_fov` (full angle; gate as `|elevation| ≤ vertical_fov/2`). The scan parameters (`beam_width`, `scan_rate`, `track_timeout`) are added in Tasks 3–4 — introduce them now as constructor arguments so the signature is stable, but leave the scan logic for Task 3.
- `update()` gates each projectile through `geometry.azimuth_elevation_range(radar_position, world_pos)`: keep it only if `range ≤ max_range` and `|elevation| ≤ vertical_fov/2`.
- **Keep the output interface identical:** `Detection.position` stays turret-relative (`world − turret_position`) with the existing Gaussian noise. This is what keeps the FSM and `TrackFilter` untouched — verify by *not* editing any of their files.
- Keep `get_detections()`, `get_target_position()`, `set_target(track_id)` working — the FSM and controller call them.

**Approach (TDD):**
- [ ] Rewrite `tests/test_search_radar.py` (the old tests use the removed constructor signature). A small helper that builds a radar with permissive defaults keeps each test focused on one gate. Cover: nothing before `update()`; a target in range+FOV is detected with `track_id` 0; the position is `world − turret_position`; a target beyond `max_range` is rejected; a target above a narrow `vertical_fov` is rejected; one near the horizon inside it is accepted; non-zero `noise_std` perturbs the position; `get_detections()` returns a fresh list.
- [ ] Run the tests, confirm they fail (constructor rejects the new keyword arguments).
- [ ] Rewrite `search_radar.py`: new constructor, `geometry`-based gating in `update()`, `Detection` unchanged. Defer scan/buffer.
- [ ] Run `pytest tests/` — the new search-radar tests pass; the FSM/filter/predictor tests remain green (proves the interface held).
- [ ] Commit: `feat(search-radar): give the radar a world pose and FOV gating`.

---

## Task 3 — Rotating scan beam

**Model: haiku (Codex: `gpt-5.4-mini`) · Effort: low** — small, well-specified addition (advance an angle, gate on it).

**Required:** Detection should depend on a sweeping beam, not cover all azimuths at once.

**What to change:** `update()` advances an internal `_beam_azimuth` by `scan_rate` each call (wrapping at 2π). A projectile is gated in only when its azimuth (from `geometry`) lies within `beam_width/2` of `_beam_azimuth` (use `geometry.angle_diff`).

**Consequence to carry forward:** detections now flicker as the beam sweeps past — Task 4 absorbs this.

**Approach (TDD):**
- [ ] Add tests: the beam advances by `scan_rate` per `update()` and wraps past 2π; a narrow beam pointed away from a target yields no detection.
- [ ] Run, confirm failure (beam never advances).
- [ ] Implement the beam advance and azimuth gate.
- [ ] Run `pytest tests/`, confirm green.
- [ ] Commit: `feat(search-radar): advance the rotating scan beam each update`.

---

## Task 4 — Track buffer persistence

**Model: sonnet (Codex: `gpt-5.4`) · Effort: medium** — ageing/refresh/expiry logic with an off-by-one risk; moderate reasoning.

**Required:** A rotating beam covers a target only intermittently. The FSM (`fsm.py:_do_search`) needs `acquire_frames` *consecutive* detections to leave `SEARCH`; flickering detections would reset that counter and the FSM would never transition. Real search radars hold a track between beam passes — model that.

**What to change:** Replace the direct detection list with a track buffer keyed by `track_id`. Each `update()`: age every existing track by one; a detected target refreshes its entry to age 0; drop any track whose age exceeds `track_timeout`. `get_detections()` returns the current buffer; `get_target_position()` reads the locked track from it.

**Approach (TDD):**
- [ ] Add tests: a track stays visible for `track_timeout` updates after the beam passes; it is dropped once the timeout is exceeded with no re-detection; continuous re-detection keeps a track alive indefinitely.
- [ ] Run, confirm failure (tracks never expire / never persist).
- [ ] Implement track ageing, refresh, and expiry in `update()`.
- [ ] Run the full suite `pytest tests/ -v`, confirm green.
- [ ] Commit: `feat(search-radar): hold tracks across beam sweeps with a timeout`.

---

## Task 5 — SearchRadar PROTO and world instantiation

**Model: sonnet (Codex: `gpt-5.4`) · Effort: medium** — Webots PROTO/`.wbt` syntax; must match conventions #9 produced (inspect an existing proto first).

**Required:** A physically visible radar in the scene. Per issue #9 the scene structures are PROTO files.

**Prerequisite check:** Open an existing proto produced by #9 (e.g. the turret proto) and copy its conventions — header line, `PROTO` block, field declarations, directory location. The instructions below assume `protos/` at repo root; correct the path if #9 differs.

**What to build:**
- `protos/SearchRadar.proto` — a `Robot`-based PROTO (a `Robot`, not a `Solid`, so a controller can be attached in a later increment without changing the node type) with `controller "<none>"`, exposing at least a `translation` field. Geometry: a simple, unmistakable body — a cylinder mast with a distinctly coloured box "antenna" on top. No devices.
- In `worlds/ATLA_v1.wbt`: add `EXTERNPROTO`/`IMPORTABLE EXTERNPROTO` for `protos/SearchRadar.proto` (match how #9 wired the other protos), and instantiate `DEF SEARCH_RADAR SearchRadar { translation 3 0 0 }`. Place it a few metres from the turret with a clear line to the projectile's arc; the exact spot is tuned in Task 6.

**Verification (manual, in Webots):**
- [ ] The world loads with no parse errors in the Webots console.
- [ ] The radar body is visible at roughly `[3, 0, 0]`.
- [ ] The simulation still runs (the turret controller starts as before).
- [ ] Commit: `feat(world): add a physical SEARCH_RADAR proto and instance`.

---

## Task 6 — Wire the SearchRadar model to the node

**Model: sonnet (Codex: `gpt-5.4`) · Effort: low** for the controller edit (small, mechanical). **The verification/tuning step is orchestrator (Opus; Codex: `gpt-5.5`, effort: medium) or human** — converging FOV/scan/`track_timeout`/placement until the FSM transitions is iterative judgment, not a haiku task.

**Required:** The controller must give the `SearchRadar` model the real node pose and the FOV/scan parameters.

**What to change in `controllers/atlas_controller/atlas_controller.py`:**
- After the existing `getFromDef("PROJECTILE")` lookup, add `robot.getFromDef("SEARCH_RADAR").getPosition()` to obtain the radar's world position.
- Replace the current `SearchRadar(...)` construction with the new signature, passing `radar_position`, `turret_position`, and tuned FOV/scan values. Suggested starting values, to be confirmed by the verification below: `max_range ≈ 20`, `vertical_fov ≈ π/2`, `beam_width ≈ 0.35` rad, `scan_rate ≈ 0.15` rad/step, `track_timeout ≈ 20` steps, `noise_std = 0.2` (keep the existing coarse value).

**Verification (manual, in Webots):**
- [ ] The controller starts with no exceptions (check the Webots console and `atlas_telemetry.log`).
- [ ] The FSM still progresses `SEARCH → ACQUIRE → TRACK → …` — the track buffer must bridge the rotating beam so the consecutive-detection counter reaches `acquire_frames`.
- [ ] **If the FSM never leaves `SEARCH`:** the projectile is being missed. Widen `vertical_fov`, raise `max_range`, increase `track_timeout`, or reposition the `SEARCH_RADAR` instance so the projectile arc passes through coverage. Re-test until the FSM transitions.
- [ ] Commit: `feat(controller): wire SearchRadar to the SEARCH_RADAR node pose`.

---

## Task 7 — Documentation

**Model: haiku (Codex: `gpt-5.4-mini`) · Effort: low** — ADR/CONTEXT.md content is already drafted in this task; mechanical authoring.

**Required:** Record the decision and fix the glossary contradiction issue #8 flagged.

- [ ] Create `docs/adr/0007-physical-search-radar.md`: decision (Search Radar is a dedicated Webots node with a world pose, FOV gate, rotating scan, and a track buffer; model runs in-process; own-controller/radio link deferred); context (radars were co-located at the turret origin with no FOV); reasoning (a node grounds the pose; FOV/scan make detection geometric; keeping `Detection` turret-relative keeps the FSM/filter untouched); consequences (`SearchRadar` needs a pose + FOV/scan params; detections are intermittent at sensor level, bridged by the buffer; new `geometry.py`; world-frame output and radio link are future increments).
- [ ] Update the `## Search Radar` section of `CONTEXT.md` to describe the dedicated node, the simulated in-process model, FOV + rotating scan, the track buffer, and the still-turret-relative `Detection` output. Reference ADR-0007.
- [ ] Commit: `docs: record the physical search radar increment (ADR-0007)`.

---

## Definition of done

- [ ] `cd controllers/atlas_controller && python -m pytest tests/ -v` — full suite green.
- [ ] The Webots simulation runs and the FSM progresses past `SEARCH`.
- [ ] The `SEARCH_RADAR` proto instance is visible in the scene.
- [ ] No `Detection` consumer (FSM, `TrackFilter`, controller decision logic) needed a change — the interface held.
