# FCR engagement fixes — design spec

**Status:** Approved design — ready for implementation planning.
**Date:** 2026-05-19
**PR:** #23
**Related issues:** #19, #21, #17

## Purpose

The Fire-Control Radar (FCR) cannot reliably lock a cued target, so the FSM
never reaches `ENGAGING`. Diagnosis (three parallel investigations) found two
compounding, confirmed bugs plus one dependency. This spec defines the fixes
for both bugs and the Search Radar cue-cadence change they rely on. It does
**not** cover the radar base-class / shared-proto refactor — that is deferred
(see *Out of scope*).

## Background — confirmed root causes

**Bug A — coordinate-frame mismatch (issue #19).** The turret base sits at
world Z = 0.05 m (`AtlasTurret.proto`). The FCR emits turret-*relative*
coordinates (`world − turret_position`), so an intercept's height is
`z_world − 0.05`. `FSMConfig.ground_threshold` (0.1, documented "world-Z") is
compared against that turret-relative height inside `_target_in_range`
(`fsm.py:535-551`). The effective ground plane is lifted to world Z = 0.15 m.
Consequence: valid low intercepts are rejected → `PREDICT` bounces back to
`TRACK` → `AIMING`/`ENGAGING` are unreachable. The "below ground" telemetry is
the visible symptom; the gating failure is the real damage.

**Bug B — acquisition cannot converge.** Two reinforcing causes:
1. The FCR FOV is a single `fov_half_angle = 0.1 rad` (5.7°) — a *tracking*-
   grade beam used for *acquisition* (`fire_control_radar.py:106`).
2. Since the cone-rework PR the Search Radar emits a cue only on the ~2–3 steps
   per revolution its rotating cone overlaps the ball (`fresh_detections` only,
   `search_radar_controller.py:286`). The cue ATLAS acts on is stale and
   carries 0.2 m noise; a stale + noisy coarse cue routinely places the true
   ball outside the 0.1 rad cone → the FCR never locks → the FSM is stuck in
   `ACQUIRE` (the lock gate `fsm.py:281`).

**Unconfirmed — out of scope.** The Search Radar narrow-cone gate may lack a
separate vertical-FOV term and could strand the FSM in `SEARCH` for a
high-elevation ball. This cannot be confirmed without a live simulation — see
*Out of scope* and *Risk*.

## Design decisions

**D1 — ground checks compare in the world frame.** The FCR → TrackFilter →
predictor pipeline is entirely turret-relative and stays that way. Rather than
redefine "ground" relative to a mount that could move, convert the intercept
height to world frame at the comparison site (`+ turret_position[2]`) and
compare against a single world-frame threshold. *Rationale:* "ground" is
physically world Z = 0; a world-framed threshold stays meaningful and survives
a turret remount.

**D2 — two-tier FCR FOV.** The FCR gets a wide *acquisition* cone and a narrow
*tracking* cone, selected by lock state. *Rationale:* a single value cannot
serve both — acquisition must tolerate a coarse cue, tracking wants precision;
widening uniformly would degrade track quality.

**D3 — Search Radar emits a cue every step from its buffered track
(issue #21).** While a track is buffered (within `track_timeout`), the
controller emits a cue every step, not only on fresh beam hits. *Rationale:*
ACQUIRE needs a continuously-updating aim point; the fresh-hit-only policy
introduced by the cone-rework PR is the regression. The cue payload carries a
`source` / `track_age` field (already present in the controller logs) so ATLAS
and tests can distinguish fresh from buffered.

## Fixes

### Fix 1 — frame-consistent ground check (Bug A, issue #19)

- In `fsm.py`, `_target_in_range` compares the intercept's *world* height. The
  FSM already holds `turret_position` via `TurretHardware`; add
  `intercept[2] + self.hardware.turret_position[2]` before the
  `> ground_threshold` test.
- `FSMConfig.ground_threshold` stays world-framed; correct its docstring to
  state the operand is converted to world frame at the comparison.
- `atlas_controller.py` telemetry `above_ground` (line 247) applies the same
  conversion so the log matches the gate.
- Reconcile `GROUND_HIT_THRESHOLD_M` (0.05, world) and `ground_threshold`
  (0.1): document why they differ (projectile-landed vs intercept-validity) or
  unify them. Implementer to pick; record the choice.

### Fix 2 — two-tier FCR FOV (Bug B, issue #17)

- `FireControlRadar.__init__` takes `acquire_fov_half_angle` and
  `track_fov_half_angle` in place of the single `fov_half_angle`.
- `update()` gates against the acquisition angle while unlocked and the
  tracking angle while locked, so a lock, once achieved, holds against the
  tighter cone without re-acquisition jitter. The implementer pins the exact
  unlocked→locked semantics and the corresponding test (see *Verification*).
- `atlas_controller.py` passes both values. Starting points: acquire ≈ 0.35
  rad, track ≈ 0.1 rad — **tuned in Webots by the human**, not the implementing
  agent (no live sim here).
- This satisfies issue #17's "set one for fire-control".

### Fix 3 — continuous buffered cue (Bug B, issue #21)

- `search_radar_controller.py` emits a cue every step for every buffered track
  still within `track_timeout`, not only `fresh_detections`.
- The cue message includes `source` (`fresh_beam_hit` | `buffered`) and
  `track_age`, matching the existing log fields.
- `SearchRadarLink` already persists the last cue — no behavioural change
  needed there; optionally surface the new fields for future use.

## Out of scope

- **Radar base-class / shared cone-geometry / shared beam-proto refactor**
  (issues #16, #17 cleanup). The investigation confirmed the FCR's gating math
  is geometrically correct — the refactor will *not* fix lock-on. Spec it
  separately; the honest shared surface is cone geometry only, not a full
  `Radar` hierarchy (the radars differ in scan vs boresight, multi-target
  buffering, and process model — all ADR-mandated).
- **Search Radar vertical-FOV gate** — unconfirmed; needs a live-sim telemetry
  check (FSM stuck in `SEARCH` vs `ACQUIRE`) before any change.
- **Code-docstring cleanups** surfaced during investigation: `track_filter.py`
  still claims dual-sensor fusion; `fire_control_radar.py` cites superseded
  ADR-0004; `SearchRadar.set_target`/`get_target_position` are dead. Fold into
  this PR's housekeeping if cheap; otherwise a follow-up.

## Verification

No live simulation is available to the implementing agent — unit tests are the
primary gate:

- **`test_fsm`** — `_target_in_range` accepts an intercept whose *world* height
  is above `ground_threshold` but whose turret-relative height is below it (the
  exact issue #19 case); still rejects a genuinely sub-ground intercept.
- **`test_fcr`** — a target inside the acquisition cone but outside the
  tracking cone is detected while unlocked; the chosen unlocked→locked
  semantics are pinned by an explicit test.
- **Search Radar controller** — a buffered track (age ≥ 1, within timeout)
  emits a cue every step with `source="buffered"`.

**Manual Webots verification (human):** the FSM reaches `ENGAGING` on a real
engagement, and the issue #19 below-ground misreport is gone.

## Risk

All three fixes are unit-testable, but end-to-end confirmation that the FCR
engages requires the human to run Webots. If the FSM still hangs after these
fixes, the prime remaining suspect is the unconfirmed Search Radar
vertical-FOV gate.
