# FCR engagement fixes — design spec

**Status:** Approved design — ready for implementation planning.
**Date:** 2026-05-19
**PR:** #23
**Related issues:** #19, #21, #17

## Purpose

The Fire-Control Radar (FCR) cannot reliably lock a cued target, so the FSM
never reaches `ENGAGING`. Diagnosis (three parallel investigations) found two
compounding, confirmed bugs. This spec defines the fix for each. It does
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

The ground check is *retained*: `_target_in_range` runs it on the **predicted
intercept** in `PREDICT` (`fsm.py:316`) and on the current target in `TRACK`
(`fsm.py:393`). The intercept is a future point that the predictor computes —
no sensor (search radar or otherwise) measures it, so sensor coverage cannot
replace this gate. The bug is the *frame* of the comparison, not the check
itself.

**Bug B — the cue is too stale for the FCR to lock.** The FCR has a single
narrow boresight cone (`fov_half_angle`, `fire_control_radar.py:76`) — this is
correct by design: the FCR is the precision *tracking* sensor and the Search
Radar is the exclusive radar that cues it (ADR-0008). The lock failure is in
the cue, not the cone:

1. The Search Radar emits a cue only on a fresh beam hit (`fresh_detections`
   only — buffered-only steps hit `continue`,
   `search_radar_controller.py:286-323`).
2. A buffered `Detection` "holds the position measured at the last detection —
   it is NOT updated while the beam is away" (`search_radar.py:237-238`). So
   between hits the cue is either absent or a frozen, increasingly stale point.

This is not about how briefly the ball is in the beam — beam dwell is tunable.
The defect is what the cue *is* between fresh hits: ATLAS aims at a frozen,
stale point while the ball moves on, so the ball is outside the FCR's narrow
cone when `update()` runs → the FCR never locks → the FSM is stuck in
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

**D2 — the FCR keeps its single tracking cone.** The FCR is not widened and
gains no second "acquisition" cone. It is the precision tracking sensor; the
Search Radar is the exclusive radar that cues it. *Rationale:* a two-tier FOV
would let the FCR self-acquire and blur the radar split — acquisition is the
Search Radar's job. The fix belongs in the cue (D3), not the FCR cone.

**D3 — the cue must stay a good-enough estimate for FCR lock.** The Search
Radar's job is to tell the FCR where to point; the cue it emits must be a
close-enough estimate that the ball falls inside the FCR's narrow cone when
`update()` runs. Two changes serve this:
- Emit a cue *every step* while a track is buffered (within `track_timeout`),
  not only on fresh beam hits — ATLAS always has a current aim point.
- The buffered track must not be a frozen position. Between beam hits the
  Search Radar advances the buffered `Detection` (a velocity estimate from
  successive fresh hits) so the emitted cue follows the ball. The exact
  extrapolation model is an implementation choice (see *Fixes*); the
  acceptance bar is "good enough for the FCR to lock", verified in Webots.
The cue payload carries a `source` (`fresh_beam_hit` | `buffered`) and
`track_age` field so ATLAS and tests can distinguish fresh from extrapolated.

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

### Fix 2 — continuous, non-stale cue (Bug B, issues #21, #17)

- `search_radar_controller.py` emits a cue every step for every buffered track
  still within `track_timeout`, not only `fresh_detections`.
- `search_radar.py` advances the buffered `Detection` between fresh hits using
  a velocity estimate derived from successive fresh detections, so the emitted
  cue is a current estimate rather than the frozen last-hit position. Keep the
  model simple (constant-velocity extrapolation is the starting point); the
  acceptance bar is a Webots-confirmed FCR lock, not estimator sophistication.
- The cue message includes `source` (`fresh_beam_hit` | `buffered`) and
  `track_age`, matching the existing log fields.
- `SearchRadarLink` already persists the last cue — no behavioural change
  needed there; optionally surface the new fields for future use.
- This resolves issue #17: the FCR keeps its single fire-control cone — no FOV
  change — and acquisition is fixed in the cue, where the regression lives.

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
- **Search Radar controller** — a buffered track (age ≥ 1, within timeout)
  emits a cue every step with `source="buffered"`.
- **`test_search_radar`** — a buffered track's position advances between fresh
  hits (no longer frozen at the last-hit value).

**Manual Webots verification (human):** the FSM reaches `ENGAGING` on a real
engagement, the FCR locks the cued ball, and the issue #19 below-ground
misreport is gone.

## Risk

The fixes are unit-testable, but end-to-end confirmation that the FCR engages
requires the human to run Webots — in particular, whether the extrapolated cue
is "good enough" for the narrow FCR cone to lock can only be confirmed live and
may need tuning. If the FSM still hangs after these fixes, the prime remaining
suspect is the unconfirmed Search Radar vertical-FOV gate.
