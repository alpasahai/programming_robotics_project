# FCR engagement fixes — design spec

**Status:** Approved design — ready for implementation planning.
**Date:** 2026-05-19 (revised after live-trace diagnosis of Bug B)
**PR:** #23
**Related issues:** #19, #21, #17

## Purpose

The Fire-Control Radar (FCR) cannot reliably lock a cued target, so the FSM
never reaches `ENGAGING`. Diagnosis found two compounding, confirmed bugs.
This spec defines the fix for each. It does **not** cover the radar
base-class / shared-proto refactor, nor the FCR acquisition scan — both are
deferred (see *Out of scope*).

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
no sensor measures it, so sensor coverage cannot replace this gate. The bug is
the *frame* of the comparison, not the check itself.

**Bug B — the Search Radar revisit rate is too slow for the FCR to lock.**
The FCR has a single narrow boresight cone (`fov_half_angle`,
`fire_control_radar.py:76`) — correct by design: the FCR is the precision
*tracking* sensor and the Search Radar is the exclusive radar that cues it
(ADR-0008). The lock failure is in how often the cue is refreshed:

1. The Search Radar scans a narrow beam (`SEARCH_RADAR_BEAM_WIDTH_RAD = 0.35`)
   at `SEARCH_RADAR_SCAN_RATE_RAD_PER_STEP = 0.15` — a full revolution every
   ~42 steps (~1.3 s at the 32 ms timestep).
2. The ATLAS projectile's flight is only ~1.2 s (~37 steps). So the SR beam
   crosses the ball at most once or twice per engagement.
3. A live trace confirms it: the SR emits fresh `fresh_beam_hit` cues only on
   the handful of consecutive steps the beam is actually on the ball (e.g.
   steps 27–28), then enters `[SR BUFFER HELD]` and **correctly emits nothing**
   — the ball is genuinely outside the beam, and a radar must report only what
   it measures.
4. `SearchRadarLink` holds the last cue (its documented no-expiry staleness
   policy, `search_radar_link.py:7-9, 23-26`). So between beam passes ATLAS
   aims the turret — and therefore the FCR boresight — at a frozen,
   increasingly stale point. The same trace shows `ACQUIRE` aiming at the
   *identical* step-28 cue on steps 30 and 31.
5. The ball flies on, never enters the FCR's narrow cone, the FCR never locks,
   and the FSM is stuck in `ACQUIRE` (the lock gate `fsm.py:281`).

The defect is the *revisit rate*, not the SR's emission logic. `BUFFER HELD`
emitting nothing is correct radar behaviour.

> **Correction note.** An earlier draft of this spec attributed Bug B to the
> SR emitting/holding stale cues and proposed making the SR emit a cue every
> step and *advance* the buffered `Detection` between hits. That was wrong: it
> would have the SR fabricate measurements it never took. The buffered
> `Detection` staying frozen while the beam is away is correct, and is
> enshrined by `test_buffered_position_is_frozen_while_beam_is_away`. The
> revised Fix 2 below leaves `search_radar.py` and `SearchRadarLink` untouched.

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
Search Radar's job.

**D3 — the cue stays fresh by raising the Search Radar revisit rate, not by
extrapolation.** The SR scans faster so its beam crosses the ball several
times per flight; each crossing emits a fresh cue, shrinking the stale window
between fresh cues below what the FCR's narrow cone can tolerate.

Spinning faster has one coupling that must be respected: faster scanning
shrinks the beam *dwell* — the number of steps the ball spends inside the beam
≈ `beam_width / scan_rate`. If dwell falls below ~1 step the beam can skip the
ball entirely between timesteps and emit no fresh cue at all. Therefore
`scan_rate` and `beam_width` are raised **together**, keeping dwell ≈ 2+ steps.

*Rationale:* real acquisition radars are characterised by revisit rate — a
search radar that sees a fast target only once per flight cannot support
fire-control acquisition. Predictive/extrapolated cues were considered and
rejected: the cue stays an honest measurement.

*Doctrinal note.* In real systems the FCR does not point-slew to a cue and
expect the target there — it rasters a small *acquisition scan* over the
uncertainty volume around the cue, then locks. That is the robust fix and is
**recommended as a follow-up** (see *Out of scope*). Raising the revisit rate
is the minimal change that makes the current point-slew design lock.

## Fixes

### Fix 1 — frame-consistent ground check (Bug A, issue #19)

- In `fsm.py`, `_target_in_range` compares the intercept's *world* height: add
  `intercept[2] + self.hardware.turret_position[2]` before the
  `> ground_threshold` test. The FSM already holds `turret_position` via
  `TurretHardware`.
- `FSMConfig.ground_threshold` stays world-framed; correct its docstring to
  state the operand is converted to world frame at the comparison site.
- `atlas_controller.py` telemetry `above_ground` (line 247) applies the same
  conversion so the log matches the gate.
- `GROUND_HIT_THRESHOLD_M` (0.05, world, `atlas_controller.py:35`) and
  `ground_threshold` (0.1) serve different purposes — projectile-has-landed
  (relaunch logic) vs intercept-worth-aiming-at. Document the distinction in
  both definitions; do **not** unify them.

### Fix 2 — raise the Search Radar revisit rate (Bug B, issues #21, #17)

- In `search_radar_controller.py`, raise `SEARCH_RADAR_SCAN_RATE_RAD_PER_STEP`
  and `SEARCH_RADAR_BEAM_WIDTH_RAD` **together**. Starting point: scan_rate
  0.15 → 0.30, beam_width 0.35 → 0.70 (revisit ~42 → ~21 steps; dwell held at
  ~2.3 steps). These are Webots-tuned values — the acceptance bar is a
  confirmed FCR lock, so the human may iterate further.
- Add a dwell-safety guard at module scope: if
  `SEARCH_RADAR_BEAM_WIDTH_RAD < 2 × SEARCH_RADAR_SCAN_RATE_RAD_PER_STEP`,
  log a warning. This stops a future tuner from silently creating a
  beam-skip.
- No change to `search_radar.py` — the buffered-`Detection` freeze is correct.
- No change to `SearchRadarLink` — its hold-last-cue policy is acceptable once
  cues are refreshed often enough.
- The emitted cue packet stays `struct.pack("ddd", x, y, z)`; `source` and
  `track_age` remain log-only fields and need no packet change.
- This resolves issue #17 without an FCR FOV change: acquisition is fixed by
  the cueing radar's revisit rate, where the regression lives.

## Out of scope

- **FCR acquisition scan (recommended follow-up).** Per real-world doctrine
  the FCR should raster a small uncertainty volume around the cue rather than
  point-slewing to it, making acquisition robust to a coarse or slightly stale
  cue instead of relying on revisit rate alone. This is a substantial design
  change (covariance-sized scan volume, raster/spiral pattern) and gets its
  own spec/plan cycle.
- **Radar base-class / shared cone-geometry / shared beam-proto refactor**
  (issue #16). The FCR's gating math is geometrically correct — the refactor
  will not fix lock-on. Spec it separately.
- **Search Radar vertical-FOV gate** — unconfirmed; needs a live-sim telemetry
  check (FSM stuck in `SEARCH` vs `ACQUIRE`) before any change.
- **Code-docstring cleanups** — already done on this branch (commit 2793e37:
  `track_filter.py`, `fire_control_radar.py`, `CONTEXT.md`).

## Verification

No live simulation is available to the implementing agent — unit tests are the
primary gate:

- **`test_fsm`** — `_target_in_range` accepts an intercept whose *world* height
  is above `ground_threshold` but whose turret-relative height is below it (the
  exact issue #19 case); still rejects a genuinely sub-ground intercept.
- **`test_search_radar` / controller** — the shipped Fix 2 constants satisfy
  the dwell invariant `beam_width ≥ 2 × scan_rate`; a stationary in-beam target
  is still detected on a sweep at the new (faster) scan rate, i.e. the faster
  scan does not skip an in-beam target.

**Manual Webots verification (human):** the FSM reaches `ENGAGING` on a real
engagement, the FCR locks the cued ball, and the issue #19 below-ground
misreport is gone. The Fix 2 constants are tuned here if the starting values
do not yield a lock.

## Risk

Fix 1 is fully unit-testable. Fix 2's revisit-rate values are empirical: the
starting `scan_rate`/`beam_width` pair may not be fast enough, and the human
may need to iterate in Webots. If revisit-rate tuning alone cannot make the
narrow FCR cone lock reliably, the fallback is the FCR acquisition scan
(deferred above). If the FSM still hangs after Fix 1 + Fix 2, the prime
remaining suspect is the unconfirmed Search Radar vertical-FOV gate.
