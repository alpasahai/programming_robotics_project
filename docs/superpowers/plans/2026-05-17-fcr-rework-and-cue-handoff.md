# FCR Rework and Cue Handoff — Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Turn the Fire-Control Radar into a physically grounded, FOV-gated slewing tracker that is cued onto the target by the Search Radar, so the FSM's `ACQUIRE` and `TRACK` states reflect real sensor geometry.

**Architecture:** The FCR gains a world *pose* and a modelled *boresight* (azimuth/elevation it is pointing). It detects the projectile only when the projectile lies inside a narrow FOV cone around the boresight and within range. The boresight slews at a bounded rate toward a cued direction; once the target is inside the cone the FCR locks and closed-loop tracks it. The Search Radar's detection supplies the cue. The FCR's measurement output stays turret-relative, so `TrackFilter` is untouched; the world-frame migration remains a later increment.

**Tech stack:** Python 3.13 (see `.python-version` / `pyproject.toml`), pytest, numpy/filterpy (already present), Webots R2025a. No new dependencies.

**Spec:** `docs/superpowers/specs/2026-05-17-radar-perception-design.md` — Increment 2.

---

## Prerequisites

- **Issue #9 complete** — scene structures are `protos/*.proto` files instantiated by `worlds/ATLA_v1.wbt`. This plan adds `protos/FireControlRadar.proto`.
- **Increment 1 complete** (`2026-05-17-search-radar-physical-node.md`) — `geometry.py` exists with `azimuth_elevation_range` and `angle_diff`, and `SearchRadar` produces FOV-gated `Detection`s.

## How to work this plan

- **Execution mode: subagent-driven.** An Opus orchestrator dispatches one fresh subagent per task, reviews, then dispatches the next. Use the `superpowers:subagent-driven-development` sub-skill.
- Same rhythm as Increment 1: per task, red-green-refactor, then commit. Run tests from `controllers/atlas_controller/` with `python -m pytest tests/ -v`.
- The world/proto files and the controller glue are verified manually in Webots, not unit-tested.

### Model and effort policy

Each task carries a **Model · Effort** tag. Same policy as Increment 1's plan — cheapest model and lowest reasoning effort that won't degrade quality.

- **Model:** `haiku` = mechanical/fully-specified; `sonnet` = moderate reasoning (class rewrites, non-trivial logic, proto syntax); `opus` = judgment calls and cross-cutting changes.
- **Effort:** `low` = transcription-level; `medium` = design small logic within a clear spec; `high` = reason through edge cases / cross-file effects.
- **The orchestrator is always Opus, and every between-task review is done by Opus**, regardless of a task's execution tag.
- **Tuning is never delegated to haiku** — Webots parameter tuning is orchestrator (Opus) or human.
- A subagent that hits ambiguity or an under-specified step stops and escalates to the orchestrator rather than guessing.

This plan skews higher than Increment 1: it touches the FSM and reconciles ADRs, so it has no haiku-only tasks except parts of the documentation task.

---

## File structure

| File | Status | Responsibility |
|---|---|---|
| `controllers/atlas_controller/fire_control_radar.py` | rewrite | `FireControlRadar` with pose, FOV cone, slewing boresight, cue interface, closed-loop tracking. |
| `controllers/atlas_controller/tests/test_fcr.py` | rewrite | Tests for the new FCR (existing tests assume the old constructor). |
| `controllers/atlas_controller/fsm.py` | modify | `_do_acquire` cues the FCR from the locked Search Radar detection. |
| `controllers/atlas_controller/tests/test_fsm.py` | modify | Cover the new cueing behaviour in `ACQUIRE`. |
| `controllers/atlas_controller/tests/stubs.py` | modify | `StubFCR` gains `cue()` / `is_locked()`; update as needed. |
| `protos/FireControlRadar.proto` | create | Webots PROTO for the FCR body. |
| `worlds/ATLA_v1.wbt` | modify | Instantiate `DEF FCR FireControlRadar { … }`. |
| `controllers/atlas_controller/atlas_controller.py` | modify | Read the `FCR` node pose; construct the new FCR. |
| `docs/adr/0008-fcr-grounded-slewing-tracker.md` | create | Supersedes ADR-0004. |
| `docs/adr/0006-sensor-membrane-and-track-id-identity.md` | modify | Revise: the FSM now cues the FCR. |
| `docs/adr/` ADR-0003 duplicate | modify | Resolve the two files numbered 0003 (land on handoff). |
| `CONTEXT.md` | modify | Rewrite the `## Fire-Control Radar (FCR)` entry. |

---

## Task 1 — FCR gains a pose and FOV cone

**Model: sonnet · Effort: medium** — class rewrite plus a new `geometry.angular_separation` helper; interface care.

**Required:** The FCR must know its own world position and detect the projectile only when it is inside a narrow FOV cone and within range — not unconditionally as today.

**Current state:** `FireControlRadar.__init__(projectile, turret_position, noise_std, rng)`. `update()` always stores a turret-relative noisy reading. `get_target_position()` returns it; `set_target(node)` re-targets.

**What to change (this task: the cone gate only — slewing is Task 2):**
- New constructor parameters: `fcr_position` (world `[x,y,z]`), `fov_half_angle` (radians — narrow), `max_range`, plus `slew_rate` and `timestep_ms` (declare now for a stable signature; `slew_rate` is used in Task 2).
- Introduce a modelled boresight `_boresight_az` / `_boresight_el`. For this task, leave the boresight fixed (set it at construction or default it pointing at the cued direction once available); the slew loop arrives in Task 2.
- `update()`: compute the target's azimuth/elevation/range from `fcr_position` via `geometry.azimuth_elevation_range`. The target is detected only when the angular separation between the boresight direction and the target direction is `≤ fov_half_angle` **and** `range ≤ max_range`.
- Angular separation between two `(az, el)` directions: convert each to a unit vector and take `acos(clamp(dot, -1, 1))`. Put this in `geometry.py` as `angular_separation(az1, el1, az2, el2) -> float` (with its own tests) so it is reusable and tested.
- On detection: store a turret-relative noisy position (`world − turret_position` + Gaussian noise), as today. When not detected: store `None`.
- `get_target_position()` returns the stored position or `None`. Add `is_locked() -> bool`.

**Approach (TDD):**
- [ ] Add `angular_separation` tests to `tests/test_geometry.py` (same direction → 0; perpendicular → π/2; antipodal → π), then implement it.
- [ ] Rewrite `tests/test_fcr.py`: a target inside the cone and in range is detected and `is_locked()` is true; a target outside the cone is not; a target beyond `max_range` is not; the reported position is `world − turret_position`; noise perturbs it; `get_target_position()` is `None` before any detection.
- [ ] Run, confirm failure (constructor signature / missing methods).
- [ ] Rewrite `fire_control_radar.py` for this task's scope.
- [ ] Run `pytest tests/`, confirm the new FCR tests pass and `TrackFilter`/predictor tests are still green.
- [ ] Commit: `feat(fcr): add a world pose and FOV-cone gating`.

---

## Task 2 — Slewing boresight

**Model: sonnet · Effort: medium** — bounded-rate angle stepping with clamping; small but easy to get wrong.

**Required:** The FCR boresight must move toward where it is told to look, at a bounded rate — it cannot snap instantly onto a target.

**What to change:** Add a slew target (`_cued_az` / `_cued_el`) and, in `update()`, step the boresight toward it before the cone test. Each update, move `_boresight_az` and `_boresight_el` toward the slew target by at most `slew_rate` radians (use `geometry.angle_diff` to get the signed step, clamp its magnitude to `slew_rate`). The slew target is set by the cue interface (Task 3); for this task a test can set it directly.

**Approach (TDD):**
- [ ] Add tests: given a slew target offset by more than `slew_rate`, the boresight moves exactly `slew_rate` toward it per `update()`; once within `slew_rate` it snaps onto the target and stops; it converges over several updates.
- [ ] Run, confirm failure.
- [ ] Implement the slew step in `update()`.
- [ ] Run `pytest tests/`, confirm green.
- [ ] Commit: `feat(fcr): slew the boresight toward the cued direction`.

---

## Task 3 — Cue interface and closed-loop tracking

**Model: sonnet · Effort: medium** — the closed-loop slew-target swap (cue vs. measured direction) is the trickiest pure-Python logic in this plan; warrants a careful Opus review afterward.

**Required:** Something must point the FCR at the target, and once locked the FCR must keep itself centred on a *moving* target rather than drifting back to a stale cue.

**What to change:**
- Add `cue(world_position)`: convert the cued world position to `(az, el)` from `fcr_position` and store it as the slew target. This replaces `set_target(node)` as the FSM-facing entry point — remove `set_target` (it took a node, which the membrane discipline of ADR-0006 wants to avoid; the cue is a plain position).
- Closed-loop tracking: when the FCR is locked (target inside the cone), the slew target each update becomes the *measured* target direction, not the last external cue — so the boresight follows the moving projectile. When not locked, the slew target stays the last external cue. Implement this as: at the end of `update()`, if locked, overwrite `_cued_az/_cued_el` with the target's measured direction.

**Approach (TDD):**
- [ ] Add tests: `cue(world_position)` sets a slew target such that the boresight slews toward that bearing; an initially-missed target is acquired after enough updates once cued onto it; once locked onto a moving target (feed a `StubProjectile` with a moving position sequence) the FCR stays locked across updates instead of losing it.
- [ ] Run, confirm failure.
- [ ] Implement `cue()` and the closed-loop slew-target update; remove `set_target`.
- [ ] Update `tests/stubs.py`: `StubFCR` gains `cue()` and `is_locked()`, drops `set_target`.
- [ ] Run `pytest tests/`, confirm green.
- [ ] Commit: `feat(fcr): add cue interface and closed-loop target tracking`.

---

## Task 4 — Cue handoff in the FSM

**Model: opus · Effort: medium** — modifies the central FSM and supersedes an ADR-0006 clause; cross-cutting, sensitive, not for a cheaper model.

**Required:** The FSM must hand the Search Radar's detection of the chosen target to the FCR so the FCR knows where to slew. Today `_do_acquire` deliberately does not touch the FCR (ADR-0006); that clause is now superseded.

**What to change in `fsm.py:_do_acquire`:**
- Each step in `ACQUIRE`, look up the locked target's current Search Radar detection (`search_radar.get_detections()`, find the entry whose `track_id == self._target`), convert its turret-relative position to world (`+ hardware.turret_position`), and call `fcr.cue(world_position)`. Continuous cueing while the FCR is still slewing keeps it aimed at a moving projectile.
- The existing `ACQUIRE → TRACK` condition (`track_filter.is_initialised()`) needs no change: the controller only calls `track_filter.update_fcr(...)` when `fcr.get_target_position()` is non-`None`, which now happens only once the FCR has slewed in and locked. So `ACQUIRE` naturally waits for the FCR to acquire — the state gains real substance for free.
- Handle the case where the locked track is briefly absent from `get_detections()` (the Search Radar's beam is mid-sweep / the track buffer dropped it): skip cueing that step rather than erroring.

**Approach (TDD):**
- [ ] Update `tests/test_fsm.py`: in `ACQUIRE`, the FSM calls `fcr.cue(...)` with the world-frame position of the locked detection; it transitions to `TRACK` once the (stub) filter reports initialised; it does not error when the locked track is missing from the detection list.
- [ ] Run, confirm failure.
- [ ] Implement the cueing in `_do_acquire`.
- [ ] Run `pytest tests/`, confirm green.
- [ ] Commit: `feat(fsm): cue the FCR from the Search Radar detection at ACQUIRE`.

---

## Task 5 — FireControlRadar PROTO and world instantiation

**Model: sonnet · Effort: medium** — Webots PROTO/`.wbt` syntax; match #9 conventions and the SearchRadar proto.

**Required:** A physically visible FCR in the scene.

**Prerequisite check:** Copy conventions from an existing #9 proto and from `protos/SearchRadar.proto` (created in Increment 1).

**What to build:**
- `protos/FireControlRadar.proto` — a visible body (e.g. a small dish/box on a short post), distinct in colour from the Search Radar. The boresight slew is modelled in Python only, so the proto needs no motors in this increment; a physically rotating head is a noted future increment.
- In `worlds/ATLA_v1.wbt`: add the `EXTERNPROTO` reference and instantiate `DEF FCR FireControlRadar { translation … }` near the turret (the FCR is part of ATLAS — place it adjacent to the `TURRET_BASE`).

**Verification (manual, in Webots):**
- [ ] World loads with no parse errors; the FCR body is visible near the turret.
- [ ] Simulation still runs.
- [ ] Commit: `feat(world): add a physical FCR proto and instance`.

---

## Task 6 — Wire the FCR model to the node

**Model: sonnet · Effort: low** for the controller edit. **The verification/tuning step is orchestrator (Opus, effort: medium) or human** — converging `fov_half_angle`/`slew_rate` so `ACQUIRE` has real but bounded duration is iterative judgment.

**Required:** The controller must construct the new FCR with the real node pose and FOV/slew parameters, and the cue path must be live.

**What to change in `controllers/atlas_controller/atlas_controller.py`:**
- Add `robot.getFromDef("FCR").getPosition()` for the FCR world position.
- Replace the `FireControlRadar(...)` construction with the new signature: `fcr_position`, `turret_position`, a narrow `fov_half_angle` (e.g. ~0.1 rad), `max_range` (e.g. ~20), a `slew_rate` (e.g. ~0.05 rad/step), the existing low `noise_std = 0.02`, and `timestep_ms`.
- The per-step loop is otherwise unchanged: `fcr.update()` still runs each step; `get_target_position()` now returns `None` until the FCR locks, so `track_filter.update_fcr(...)` is naturally skipped until then. The FSM (Task 4) issues the cues.

**Verification (manual, in Webots):**
- [ ] Controller starts without exceptions.
- [ ] Observe in `atlas_telemetry.log`: the FSM dwells in `ACQUIRE` while the FCR slews, then advances to `TRACK` once the FCR locks — `ACQUIRE` is no longer instantaneous.
- [ ] **Tuning:** if the FCR never locks, the cue is not bringing the target into the cone — widen `fov_half_angle`, raise `slew_rate`, or check the cue conversion (turret-relative → world). If it locks instantly, `ACQUIRE` has no substance — narrow `fov_half_angle` or lower `slew_rate`.
- [ ] Commit: `feat(controller): wire the FCR to the FCR node pose and cue path`.

---

## Task 7 — Documentation

**Model: split.** ADR-0008 authoring and the CONTEXT.md rewrite: **haiku · Effort: low** (content drafted below). Resolving the duplicate ADR-0003 and revising ADR-0006: **opus · Effort: medium** — these are judgment calls about which design stands and how to renumber, not transcription.

**Required:** Record the new FCR design and reconcile the affected ADRs.

- [ ] Create `docs/adr/0008-fcr-grounded-slewing-tracker.md`: decision (FCR is a dedicated Webots node with a world pose, a narrow FOV cone, a modelled slewing boresight, and closed-loop tracking; cued by the Search Radar; permanently simulated); mark it **Supersedes ADR-0004** (the upgrade path to a real Webots `Radar` device is rejected — the device has no elevation output; see the perception spec); consequences (FCR needs a pose + FOV/slew params; `set_target(node)` is replaced by `cue(world_position)`; `ACQUIRE` now has real duration).
- [ ] Edit `docs/adr/0006-sensor-membrane-and-track-id-identity.md`: revise the clause "the FSM does not call `fcr.set_target`". The FSM now cues the FCR with a plain world position (not a node), so the membrane still holds — no node-typed value enters FSM scope. Note the revision and the reason.
- [ ] Resolve the duplicate ADR-0003: there are two files numbered 0003 (`continuous-kalman-fusion-both-sensors` and `search-radar-handoff-not-continuous-fusion`). Land on the handoff design, renumber/retire so only one 0003 remains, and make its status consistent with this plan (Search Radar cues; FCR tracks).
- [ ] Rewrite the `## Fire-Control Radar (FCR)` section of `CONTEXT.md`: dedicated node, simulated, world pose, narrow FOV cone, slewing boresight cued by the Search Radar, closed-loop tracking, turret-relative output. Reference ADR-0008.
- [ ] Commit: `docs: record the grounded slewing FCR (ADR-0008) and reconcile ADRs`.

---

## Definition of done

- [ ] `cd controllers/atlas_controller && python -m pytest tests/ -v` — full suite green.
- [ ] In Webots, `ACQUIRE` visibly takes time while the FCR slews, then `TRACK` begins once it locks.
- [ ] Both radar nodes are visible and the engagement cycle completes (`SEARCH → … → ENGAGING → RESET`).
- [ ] `TrackFilter` was not modified — the turret-relative measurement interface held.
