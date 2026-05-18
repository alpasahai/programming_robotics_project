# FCR Rework and Cue Handoff (Revised) — Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

> **Supersedes** `docs/superpowers/plans/2026-05-17-fcr-rework-and-cue-handoff.md`. That plan predates PR #18 (turret reframed as the FCR body) and the arrival of `search_radar_controller` as a separate process. Do not execute the old plan.

**Goal:** Make the Fire-Control Radar an FOV-gated sensor whose boresight is the turret's own aim, cued by a separate Search Radar process over an Emitter/Receiver radio link, so the FSM's `ACQUIRE`/`TRACK` states reflect real sensor geometry and real motor travel time.

**Architecture:** The turret *is* the FCR (`AtlasTurret.proto`). The `FireControlRadar` class keeps no boresight of its own — it is told the turret's aim each `update()` and detects the projectile only inside an FOV cone around that aim. The Search Radar runs in its own controller process: it owns detection and target selection, and emits one chosen world-frame cue per step. `atlas_controller` receives the cue and the FSM slews the turret toward it; the cone sweeps onto the target and the FCR locks. Shared geometry code moves to a repo-root `lib/`.

**Tech stack:** Python 3.13 (see `.python-version` / `pyproject.toml`), pytest, numpy/filterpy (already present), Webots R2025a. No new dependencies.

**Spec:** `docs/superpowers/specs/2026-05-18-fcr-rework-cue-handoff-revised-design.md`.

---

## Prerequisites

- **Increment 1 complete** — `geometry.py` exists with `azimuth_elevation_range` and `angle_diff`; `SearchRadar` produces FOV-gated `Detection`s with a `track_id`-keyed track buffer.
- **`search_radar_controller/` exists** as a Webots controller directory with its own `runtime.ini` (`SearchRadar.proto` sets `controller "search_radar_controller"`).
- **PR #18 merged** — `AtlasTurret.proto` is the FCR body; `AtlasLaser.proto` is the cosmetic aim line.

## How to work this plan

- **Execution mode: subagent-driven.** An Opus orchestrator dispatches one fresh subagent per task, reviews, then dispatches the next. Use `superpowers:subagent-driven-development`.
- Per task: red-green-refactor, then commit. Run tests from `controllers/atlas_controller/` with `python -m pytest tests/ -v`; the Search Radar process gets its own `controllers/search_radar_controller/tests/`.
- World/proto files and controller glue are verified manually in Webots, not unit-tested.

### Model and effort policy

Use the `atlas-controls-engineer` agent for every subagent and the orchestrator. Each task carries a **Model · Effort** tag.

- **Model:** `haiku` (Codex: `gpt-5.4-mini`) = mechanical/fully-specified; `sonnet` (Codex: `gpt-5.4`) = moderate reasoning; `opus` (Codex: `gpt-5.5`) = judgment calls and cross-cutting changes.
- **Effort:** `low` = transcription-level; `medium` = design small logic within a clear spec; `high` = reason through edge cases / cross-file effects.
- **The orchestrator is always Opus, and every between-task review is done by Opus.**
- **Codex orchestration:** If you're Codex and you're orchestrating this plan, use the Codex models instead: `gpt-5.5` for the orchestrator/reviews, `gpt-5.4` where the plan says `sonnet`, and `gpt-5.4-mini` where the plan says `haiku`.
- **Tuning is never delegated to haiku** — Webots parameter tuning is orchestrator (Opus) or human.
- **Announce the `model · effort` tag in visible text before every `Agent` dispatch.**
- A subagent that hits ambiguity stops and escalates to the orchestrator rather than guessing.

### Parallelism

Tasks 3 (FCR) and 4–5 (radio link) touch disjoint files once Task 1 lands and may run in parallel. WARNING from the Plan 1 retro: parallel subagents each `git add`/`git commit` — give each a tight file scope, tell it to `git add` only its own files explicitly, and re-check commit boundaries afterward.

---

## File structure

| File | Status | Responsibility |
|---|---|---|
| `lib/geometry.py` | move + extend | Shared world-frame helpers; gains `angular_separation`. Moved from `controllers/atlas_controller/geometry.py`. |
| `lib/__init__.py` | n/a | Not created — `lib/` is a path on `PYTHONPATH`, modules imported flat (`import geometry`). |
| `controllers/atlas_controller/runtime.ini` + `.example` | modify | Add `PYTHONPATH` extension to `../../lib`. |
| `controllers/search_radar_controller/runtime.ini` + `.example` | modify | Add `PYTHONPATH` extension to `../../lib`. |
| `controllers/atlas_controller/tests/conftest.py` | create or modify | Put `../../../lib` on `sys.path` for tests. |
| `controllers/search_radar_controller/search_radar.py` | move | `SearchRadar` class — moved from `atlas_controller/`. |
| `controllers/search_radar_controller/tests/test_search_radar.py` | move | Moved with the class. |
| `controllers/search_radar_controller/tests/conftest.py` | create | `sys.path` for `lib/` + the controller dir. |
| `controllers/atlas_controller/fire_control_radar.py` | rewrite | `FireControlRadar` — fixed pose, FOV cone gated by the turret aim passed into `update()`. |
| `controllers/atlas_controller/tests/test_fcr.py` | rewrite | Tests for the new FCR. |
| `controllers/search_radar_controller/search_radar_controller.py` | rewrite | Supervisor; builds `SearchRadar`; selects one target; `Emitter.send()`s its world position. |
| `controllers/atlas_controller/search_radar_link.py` | create | Wraps the `Receiver`; `update()` drains the queue, `get_cue()` returns the last cue. |
| `controllers/atlas_controller/tests/test_search_radar_link.py` | create | Tests with a stub receiver. |
| `controllers/atlas_controller/fsm.py` | modify | `SensorSuite.search_radar` → `cue_link`; `_do_search`/`_do_acquire` rework. |
| `controllers/atlas_controller/tests/test_fsm.py` | modify | Cover cue-driven `SEARCH`/`ACQUIRE`. |
| `controllers/atlas_controller/tests/stubs.py` | modify | `StubFCR` gains `is_locked()`; add `StubCueLink`; drop `set_target`. |
| `protos/SearchRadar.proto` | modify | Add an `Emitter` device node. |
| `protos/AtlasTurret.proto` | modify | Add a `Receiver` device node. |
| `controllers/atlas_controller/atlas_controller.py` | modify | Construct `SearchRadarLink` + new FCR; feed turret aim to `fcr.update()`; drop in-process `SearchRadar` and `update_search()`. |
| `docs/adr/0008-fcr-turret-aim-sensor.md` | create | Supersedes ADR-0004. |
| `docs/adr/0009-two-process-radio-cue-link.md` | create | The Emitter/Receiver cue link. |
| `docs/adr/0006-sensor-membrane-and-track-id-identity.md` | modify | Membrane now spans a process boundary. |
| `docs/adr/` ADR-0003 duplicate | modify | Resolve the two files numbered 0003. |
| `CONTEXT.md` | modify | Rewrite the Search Radar and FCR entries. |

---

## Task 1 — Shared `lib/` directory

**Model: sonnet (Codex: `gpt-5.4`) · Effort: low** — file move plus `runtime.ini`/`conftest` path wiring; mechanical but easy to get path levels wrong.

**Required:** `geometry.py` is needed by both the FCR (in `atlas_controller`) and `SearchRadar` (moving to `search_radar_controller`). It must live in one place importable by both processes.

**What to change:**
- Move `controllers/atlas_controller/geometry.py` → `lib/geometry.py` (repo root). `controllers/atlas_controller/tests/test_geometry.py` stays in the atlas test suite for now — it continues to `import geometry`, which resolves from `lib/` via the `conftest.py` path below.
- In **both** `controllers/atlas_controller/runtime.ini` and `controllers/search_radar_controller/runtime.ini`, add an `[environment variables with paths]` section: `PYTHONPATH = $(PYTHONPATH):../../lib`. The path is relative to the controller directory; `../../lib` reaches the repo-root `lib/`.
- Mirror the identical change into **both** `runtime.ini.example` files.
- Add (or extend) `controllers/atlas_controller/tests/conftest.py` so the test process puts the repo-root `lib/` on `sys.path` — `conftest.py` is at `controllers/atlas_controller/tests/`, so `lib/` is three levels up. Pytest does not read `runtime.ini`, so the path must be set independently for tests.

**Verification:**
- [ ] `cd controllers/atlas_controller && python -m pytest tests/ -v` — full suite still green (every module that did `import geometry` now resolves it from `lib/`).
- [ ] Confirm no remaining `geometry.py` under `controllers/`.
- [ ] Commit: `refactor: move geometry.py to a shared repo-root lib/`.

---

## Task 2 — Move `SearchRadar` into its controller

**Model: sonnet (Codex: `gpt-5.4`) · Effort: low** — file move plus a test `conftest`; the class body does not change.

**Required:** With the Search Radar running as its own process, `SearchRadar` must live in `search_radar_controller/`, not `atlas_controller/`. `atlas_controller` no longer imports it.

**What to change:**
- Move `controllers/atlas_controller/search_radar.py` → `controllers/search_radar_controller/search_radar.py`. The class body is unchanged (it already imports `geometry`, now resolved via `lib/`).
- Move `controllers/atlas_controller/tests/test_search_radar.py` → `controllers/search_radar_controller/tests/test_search_radar.py`.
- Create `controllers/search_radar_controller/tests/conftest.py` putting both `lib/` and the controller directory on `sys.path` (mirror the atlas `conftest.py` pattern).
- Move any `SearchRadar`-specific stub (e.g. `StubProjectile`) needed by the moved test, or copy the minimal stub into the new test dir. Do **not** leave the atlas suite importing `search_radar`.
- Remove the `from search_radar import ...` line in `atlas_controller.py` only if Task 7 has not yet run — otherwise leave it for Task 7. (If executing in order, this task leaves `atlas_controller.py` temporarily importing a missing module; that is acceptable because the controller is not unit-tested. Note it for Task 7.)

**Verification:**
- [ ] `cd controllers/search_radar_controller && python -m pytest tests/ -v` — the moved `SearchRadar` tests pass.
- [ ] `cd controllers/atlas_controller && python -m pytest tests/ -v` — the atlas suite passes with no `search_radar` import (the FSM still references `search_radar` via `SensorSuite`; that is reworked in Task 6 — if the atlas suite breaks here purely on the FSM's `search_radar` field, that is expected and resolved in Task 6; flag it, do not patch the FSM in this task).
- [ ] Commit: `refactor: move SearchRadar into search_radar_controller`.

---

## Task 3 — FCR rework: FOV cone gated by the turret aim

**Model: sonnet (Codex: `gpt-5.4`) · Effort: medium** — class rewrite plus a `geometry.angular_separation` helper; interface care.

**Required:** The FCR must detect the projectile only when it lies inside a narrow FOV cone around the *turret's current aim* and within range — not unconditionally as today. It has no boresight of its own; the turret aim is passed in.

**Current state:** `FireControlRadar.__init__(projectile, turret_position, noise_std, rng)`. `update()` always stores a turret-relative noisy reading. `set_target(node)` re-targets.

**What to change:**
- New constructor signature: `FireControlRadar(projectile, fcr_position, turret_position, fov_half_angle, max_range, noise_std=0.0, rng=None)`. `fcr_position` is the FCR's world position (the turret's position — co-located); kept as a distinct parameter so the gating frame is explicit.
- Add `geometry.angular_separation(az1, el1, az2, el2) -> float` to `lib/geometry.py`: convert each `(az, el)` to a unit vector, return `acos(clamp(dot, -1, 1))`. Give it its own tests (same direction → 0; perpendicular → π/2; antipodal → π).
- `update(boresight_az, boresight_el)` — takes the turret's current aim direction. Compute the projectile's azimuth/elevation/range from `fcr_position` via `geometry.azimuth_elevation_range`. The target is detected only when `angular_separation(boresight_az, boresight_el, target_az, target_el) ≤ fov_half_angle` **and** `range ≤ max_range`.
- On detection: store a turret-relative noisy position (`world − turret_position` + Gaussian noise). When not detected: store `None`.
- `get_target_position()` returns the stored position or `None`. Add `is_locked() -> bool` (true iff the last `update()` detected the target).
- Remove `set_target`.

**Approach (TDD):**
- [ ] Add `angular_separation` tests to `tests/test_geometry.py`, then implement it in `lib/geometry.py`.
- [ ] Rewrite `tests/test_fcr.py`: a target inside the cone and in range is detected and `is_locked()` is true; a target outside the cone is not (and `is_locked()` false); a target beyond `max_range` is not; the reported position is `world − turret_position`; noise perturbs it; `get_target_position()` is `None` before any `update()`; passing a boresight that points away from the target loses the lock.
- [ ] Run, confirm failure (constructor signature / missing methods).
- [ ] Rewrite `fire_control_radar.py`.
- [ ] Run `pytest tests/`, confirm the new FCR tests pass and `TrackFilter`/predictor tests are still green.
- [ ] Commit: `feat(fcr): gate detection by an FOV cone around the turret aim`.

---

## Task 4 — Search Radar process emits cues

**Model: sonnet (Codex: `gpt-5.4`) · Effort: medium** — Webots `Emitter` PROTO syntax plus a controller rewrite; `struct` serialisation.

**Required:** The Search Radar process must detect the projectile, select one target, and broadcast that target's world position each step over a radio link.

**Prerequisite check:** Copy `Emitter` conventions from the Webots reference; copy PROTO device-node placement from the existing motor/sensor declarations in `SearchRadar.proto`.

**What to change:**
- `protos/SearchRadar.proto`: add an `Emitter` device node to the `Robot { children [ … ] }` list — `name "SR_CUE_EMITTER"`, `channel 1`, `range -1`, `type "radio"`.
- Rewrite `controllers/search_radar_controller/search_radar_controller.py`:
  - Construct a `Supervisor` (not a plain `Robot`) — `SearchRadar` needs projectile ground truth.
  - Read the projectile node (`getFromDef("PROJECTILE")`) and the radar's own pose; keep the existing motor spin-up.
  - Construct `SearchRadar` with the same FOV/scan parameters `atlas_controller` used in Increment 1 (`max_range`, `vertical_fov`, `beam_width`, `scan_rate`, `track_timeout`, `noise_std`).
  - Each step: `search_radar.update()`, then select one live track — the first of `search_radar.get_detections()` (matching the old FSM's `detections[0]`). Convert that `Detection`'s turret-relative position back to world frame (`+ turret_position`) — or, simpler, have the selection read the chosen track's world position directly. Decide and document which; the cue payload must be **world-frame**.
  - If a track is selected, `emitter.send(struct.pack("ddd", x, y, z))`. If none, send nothing.
- Note for the orchestrator: the SR process needs `turret_position` to produce a world-frame cue. It can read `getFromDef("TURRET_BASE").getPosition()` (it is a Supervisor). Confirm the DEF name against `worlds/ATLA_v1.wbt`.

**Verification (manual, in Webots — flag to the user; the controller is not unit-tested):**
- [ ] World loads; the SR process starts without exceptions; the radar bar still spins.
- [ ] Add a temporary `print` of the sent cue; confirm a plausible world position is emitted when the projectile is in the beam.
- [ ] Commit: `feat(search-radar): emit a chosen world-frame cue over an Emitter`.

---

## Task 5 — Turret receives cues

**Model: sonnet (Codex: `gpt-5.4`) · Effort: medium** — `Receiver` PROTO syntax plus a small new tested class.

**Required:** `atlas_controller` must receive the Search Radar's cue each step and expose the latest one through a narrow, testable interface.

**What to change:**
- `protos/AtlasTurret.proto`: add a `Receiver` device node to the `Robot { children [ … ] }` list — `name "FCR_CUE_RECEIVER"`, `channel 1`, `allowedChannels [1]`.
- Create `controllers/atlas_controller/search_radar_link.py` with a `SearchRadarLink` class:
  - `__init__(receiver)` — stores the Webots `Receiver` (or a stub); calls `receiver.enable(timestep)` is the *controller's* job, not the class's — accept an already-enabled receiver. (Document this in the docstring.)
  - `update()` — drain the queue: while `receiver.getQueueLength() > 0`, `unpack` the packet and keep the last, then `receiver.nextPacket()`. Store the last cue as `[x, y, z]`.
  - `get_cue() -> list[float] | None` — the most recent cue, or `None` if none has ever arrived.
  - Decide cue staleness policy: simplest is "last cue persists until a new one arrives". Document it; if the orchestrator wants a freshness timeout, that is a noted extension, not in this task.
- Create `tests/test_search_radar_link.py` with a `StubReceiver` (configurable queue of `struct`-packed packets): `get_cue()` is `None` before any `update()`; after `update()` with one packet it returns that position; with several queued packets it returns the last; with an empty queue it retains the previous cue.

**Approach (TDD):**
- [ ] Write `tests/test_search_radar_link.py` and the `StubReceiver`.
- [ ] Run, confirm failure.
- [ ] Implement `search_radar_link.py`.
- [ ] Run `pytest tests/`, confirm green.
- [ ] Add the `Receiver` to `AtlasTurret.proto`.
- [ ] Commit: `feat(atlas): add SearchRadarLink to receive radio cues`.

---

## Task 6 — FSM rework: cue-driven SEARCH and ACQUIRE

**Model: opus (Codex: `gpt-5.5`) · Effort: medium** — modifies the central FSM and the `SensorSuite` contract; cross-cutting and sensitive.

**Required:** The FSM must consume cues from the `SearchRadarLink` instead of an in-process `SearchRadar`. `SEARCH` waits for a cue; `ACQUIRE` slews the turret toward the cue and waits for the FCR to lock.

**Current state:** `SensorSuite.search_radar` holds a `SearchRadar`. `_do_search` counts `search_radar.get_detections()` and picks `detections[0].track_id`. `_do_acquire` calls `search_radar.set_target(track_id)`.

**What to change in `fsm.py`:**
- `SensorSuite`: replace the `search_radar` field with `cue_link` (a `SearchRadarLink`). The FCR still needs the turret aim each step — see below.
- `_do_search`: keep the pan sweep if desired, but the transition trigger changes. Each step read `cue_link.get_cue()`. Count consecutive steps with a non-`None` cue (the `acquire_frames` debounce stays in the FSM). On reaching `acquire_frames`, store the cue as `self._target` (a world position now, not a `track_id`) and transition to `ACQUIRE`.
- `_do_acquire`: each step, read the latest cue from `cue_link.get_cue()`, convert it to a turret-relative bearing, and command the turret motors toward it (use the existing `_aim_at` / `_compute_aim_angles` helpers — they already convert a relative position to pan/tilt and command both motors). The real motors slew at `maxVelocity`. Transition `ACQUIRE → TRACK` when `fcr.is_locked()` **and** `track_filter.is_initialised()`. Remove the `search_radar.set_target` call and the `_acquire_entry_done`-guarded entry block (or repurpose it — `track_filter.reset()` on entry should stay).
- Feeding the turret aim to the FCR: the FCR's `update(boresight_az, boresight_el)` needs the turret's current aim. The FSM tracks `_commanded_pan` / `_commanded_tilt`. Add public read access (a property or accessor) so the controller can pass them to `fcr.update()` — OR have the FSM own the `fcr.update()` call. Decide and document: the cleaner option is a `commanded_aim` property on the FSM that the controller reads; the controller then calls `fcr.update(*fsm.commanded_aim)` in its sense phase. (One-step lag is acceptable — the motor is mid-slew anyway.)
- The `ACQUIRE → TRACK` docstring/ADR clause about `set_target` is superseded — note it for Task 8.
- Update `tests/stubs.py`: add `StubCueLink` (configurable `get_cue()` return); `StubFCR` gains `is_locked()` and an `update(az, el)` signature; drop `set_target` from `StubFCR`/`StubSearchRadar`.

**Approach (TDD):**
- [ ] Update `tests/stubs.py` with `StubCueLink` and the revised `StubFCR`.
- [ ] Update `tests/test_fsm.py`: `SEARCH` stays until `acquire_frames` consecutive cues, then enters `ACQUIRE`; `ACQUIRE` commands the turret motors toward the cue bearing each step; `ACQUIRE → TRACK` fires only when the stub FCR reports `is_locked()` and the stub filter reports initialised; `ACQUIRE` does not error when `get_cue()` briefly returns `None`.
- [ ] Run, confirm failure.
- [ ] Implement the `fsm.py` changes.
- [ ] Run `pytest tests/`, confirm green.
- [ ] Commit: `feat(fsm): drive SEARCH/ACQUIRE from the Search Radar cue`.

---

## Task 7 — Wire `atlas_controller.py`

**Model: sonnet (Codex: `gpt-5.4`) · Effort: low** for the controller edit. **Webots verification/tuning is orchestrator (Opus; Codex: `gpt-5.5`, effort: medium) or human.**

**Required:** The controller must construct the `SearchRadarLink` and the new FCR, feed the turret aim to the FCR, and drop the removed in-process `SearchRadar` and `update_search()` calls.

**What to change in `controllers/atlas_controller/atlas_controller.py`:**
- Remove the `from search_radar import SearchRadar` import and the `SearchRadar(...)` construction.
- Get the `Receiver` device (`robot.getDevice("FCR_CUE_RECEIVER")`), call `receiver.enable(timestep)`, and construct `SearchRadarLink(receiver)`.
- Replace the `FireControlRadar(...)` construction with the new signature: `projectile`, `fcr_position` (= `turret_position`), `turret_position`, a narrow `fov_half_angle` (e.g. ~0.1 rad), `max_range` (e.g. ~20), the existing low `noise_std = 0.02`.
- Per-step loop: call `cue_link.update()` in the sense phase; call `fcr.update(*fsm.commanded_aim)` (the turret aim accessor from Task 6) instead of the old `fcr.update()`.
- Remove the `search_radar.update()` call and the `track_filter.update_search(...)` block. Only `track_filter.update_fcr(...)` remains, still gated on `fcr.get_target_position() is not None`.
- Pass the `cue_link` into `SensorSuite` in place of `search_radar`.

**Verification (manual, in Webots — flag to the user; not unit-tested):**
- [ ] Controller starts without exceptions; no missing-device or import errors.
- [ ] In `atlas_telemetry.log`: the FSM dwells in `SEARCH` until a cue arrives, then `ACQUIRE` while the turret slews, then `TRACK` once the FCR locks — `ACQUIRE` is no longer instantaneous.
- [ ] **Tuning:** if the FCR never locks, the turret is not bringing the target into the cone — widen `fov_half_angle`, raise the motor `maxVelocity` in `AtlasTurret.proto`, or check the cue bearing conversion. If it locks instantly, narrow `fov_half_angle`.
- [ ] Commit: `feat(controller): wire the cue link and turret-aim FCR`.

---

## Task 8 — Documentation

**Model: split.** ADR-0008/0009 authoring and the `CONTEXT.md` rewrite: **haiku (Codex: `gpt-5.4-mini`) · Effort: low** (content outlined below). Revising ADR-0006 and resolving the duplicate ADR-0003: **opus (Codex: `gpt-5.5`) · Effort: medium** — judgment calls, not transcription.

**Required:** Record the new FCR design and the radio cue link, and reconcile the affected ADRs.

- [ ] Create `docs/adr/0008-fcr-turret-aim-sensor.md`: decision — the FCR is the ATLAS turret itself; a narrow-beam sensor with no boresight of its own, FOV-cone gated around the turret's commanded aim, permanently simulated. Mark it **Supersedes ADR-0004** (the real-Webots-`Radar` upgrade path is rejected — the device has no elevation output). Consequences: `FireControlRadar` takes the turret aim in `update()`; `set_target` is removed; `ACQUIRE` now has real duration (physical motor travel time).
- [ ] Create `docs/adr/0009-two-process-radio-cue-link.md`: decision — the Search Radar runs as its own controller process and hands ATLAS one chosen world-frame cue per step over an `Emitter`/`Receiver` radio link (channel 1, `type "radio"`, `range -1`). Target selection lives in the Search Radar process. Consequences: a one-step delivery lag; `atlas_controller` no longer fuses Search Radar measurements (`track_filter.update_search()` removed); `TrackFilter` is now FCR-only.
- [ ] Edit `docs/adr/0006-sensor-membrane-and-track-id-identity.md`: the membrane now spans a process boundary. The `Detection`/`track_id` discipline lives wholly inside the Search Radar process; the cue crossing the radio link is a bare world position — no node handle, no track identity, reaches `atlas_controller`. Note the revision and the reason.
- [ ] Resolve the duplicate ADR-0003: two files are numbered 0003 (`continuous-kalman-fusion-both-sensors` and `search-radar-handoff-not-continuous-fusion`). Land on the handoff design (Search Radar cues; FCR tracks; no continuous search fusion), renumber/retire so only one 0003 remains, and make its status consistent with this plan.
- [ ] Rewrite the `## Fire-Control Radar (FCR)` and `## Search Radar` sections of `CONTEXT.md`: the FCR is the turret, FOV-cone gated by the turret aim; the Search Radar is a separate process emitting a radio cue; turret-relative measurement output retained. Reference ADR-0008 and ADR-0009.
- [ ] Commit: `docs: record the turret-aim FCR and radio cue link; reconcile ADRs`.

---

## Definition of done

- [ ] `cd controllers/atlas_controller && python -m pytest tests/ -v` — full suite green.
- [ ] `cd controllers/search_radar_controller && python -m pytest tests/ -v` — full suite green.
- [ ] In Webots, the FSM waits in `SEARCH` for a cue, dwells in `ACQUIRE` while the turret slews, then enters `TRACK` once the FCR locks.
- [ ] Both controller processes run; the engagement cycle completes.
- [ ] `TrackFilter` change is deletion-only — `update_search()` removed, no behavioural rewrite.
- [ ] No `geometry.py` or `search_radar.py` remains under `controllers/atlas_controller/`.
