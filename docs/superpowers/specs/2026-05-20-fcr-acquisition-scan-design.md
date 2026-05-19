# FCR acquisition scan — design spec

**Status:** Draft — ready for review.
**Date:** 2026-05-20
**PR:** (none yet — proposed follow-up to PR #23)
**Related issues:** #17 (and as the deferred follow-up named in `2026-05-19-fcr-engage-fix-design.md` *Out of scope*).
**Supersedes:** nothing. Refines D2 of `2026-05-19-fcr-engage-fix-design.md` by changing *how* ACQUIRE uses the FCR, not the FCR's FOV.

## Purpose

ACQUIRE point-slews the turret (and so the FCR boresight) to a single Search
Radar cue and waits for the FCR to lock. A live trace after Fix 1 + Fix 2 of
`2026-05-19-fcr-engage-fix-design.md` shows the FSM now *reaches* ACQUIRE but
the FCR never locks: the cue freezes the moment the SR beam scans off the
ball, the ball moves on, and the narrow FCR cone (`fov_half_angle = 0.1 rad`)
is empty by the time the turret arrives. This spec defines an *acquisition
scan*: ACQUIRE rasters the FCR boresight over the uncertainty volume around
the cue instead of point-slewing to it, so a moving target is somewhere in
the scanned region even if the exact cue point is stale.

## Background — confirmed root cause

After the Fix 2 revisit-rate change the SR delivers a ~3-step burst of fresh
cues per beam pass, enough for SEARCH to satisfy `acquire_frames` and
transition to ACQUIRE. The new failure mode is in ACQUIRE itself:

1. ACQUIRE stores the cue at SEARCH-exit and `_do_acquire` re-aims at it each
   step (`fsm.py:279-281`). `SearchRadarLink.get_cue()` returns the last
   received cue forever (its documented no-expiry staleness policy,
   `search_radar_link.py:23-26`), so once the SR beam scans off the ball and
   the controller logs `[SR BUFFER HELD]`, `_do_acquire` keeps aiming at the
   *same* frozen point. The live trace shows `target_rel` literally unchanged
   over steps 27–30.
2. At a representative close engagement (range ≈ 1.9 m, ball ≈ 0.3 m/step)
   the ball's angular velocity at the turret is ~0.16 rad/step — *larger
   than the FCR cone half-angle (0.1 rad)*. The ball crosses the entire FCR
   cone in less than one timestep, so even a perfectly fresh cue is already
   too stale by the time the turret acts on it. Cue → motor → `fcr.update()`
   has a floor latency of one timestep, and the cone-crossing time is below
   that floor.
3. The FCR therefore never returns a measurement, `update_fcr()` is never
   called, `track_filter.is_initialised()` stays False, and ACQUIRE's exit
   gate (`fsm.py:283`, `fcr.is_locked() AND track_filter.is_initialised()`)
   can never be satisfied. The FSM is stuck.

The defect is the *acquisition pattern*, not any sensor or filter. Raising
the SR revisit rate (Fix 2) shortened the stale-cue window but cannot push
handoff latency below one step, so it cannot resolve a sub-timestep
cone-crossing. This is the spec's *Risk* section coming true and the
doctrinal follow-up it called out:

> In real systems the FCR does not point-slew to a cue and expect the target
> there — it rasters a small acquisition scan over the uncertainty volume
> around the cue, then locks. That is the robust fix and is recommended as a
> follow-up.

A diagnostic-only `[FCR REJECT]` telemetry line (separate, small commit on
top of PR #23) verifies the cone-crossing geometry on a live run before this
spec is implemented.

## Design decisions

D1–D3 from `2026-05-19-fcr-engage-fix-design.md` carry forward unchanged.

**D4 — ACQUIRE rasters, it does not point-slew.** `_do_acquire` sweeps the
FCR boresight through a small scan pattern over the angular box around the
cue instead of commanding a single boresight derived from the cue's bearing.
The exit condition is unchanged: lock as soon as `fcr.is_locked() AND
track_filter.is_initialised()` becomes true. *Rationale:* the cone-crossing
time is shorter than handoff latency; a single boresight cannot keep the
ball inside the cone, but a swept region can.

**D5 — Scan volume is sized by cue uncertainty, not the full sky.** The
angular extent of the scan is a fixed configurable half-width
(`acquire_scan_half_width_rad`) starting at ~0.4 rad, chosen so the ball is
expected to lie inside the box for at least several timesteps of motion at
typical angular velocity. The box is *not* derived from a Kalman covariance
— the track filter is uninitialised in ACQUIRE — it is a static empirical
bound, tuned in Webots. *Rationale:* the right shape of the uncertainty is
unknown at ACQUIRE entry (no filter yet); a generous fixed box is robust
and trivial to reason about. A covariance-shaped scan is a future option
once the filter feeds it.

**D6 — Scan pattern: outward spiral from the cue.** Most likely position
is the cue itself, so begin there. Each scan step advances by one cone
*diameter* (`2 × fov_half_angle`) to the next raster cell, in an outward
spiral (or expanding-square boustrophedon — pattern shape is
implementation choice, the invariant is *centre-first* and *exhaustive
within the box*). *Rationale:* fastest expected-time-to-lock when the cue
is approximately right; degrades gracefully when the cue is several
cone-widths off.

**D7 — Scan time is bounded; on exhaustion ACQUIRE falls back to SEARCH.**
After `acquire_scan_max_steps` raster cells without a lock, `_do_acquire`
transitions to SEARCH so the next SR beam pass can re-cue. *Rationale:*
ACQUIRE must not loop forever on a stale cue. Returning to SEARCH is the
existing recovery path and re-uses its working cue-counting logic.

**D8 — Fresh cue during scan recentres the raster.** If a new SR cue
arrives mid-scan (`cue_link.get_cue()` differs from the cue stored at
ACQUIRE entry), the raster restarts centred on the new cue and the
exhaustion counter resets. *Rationale:* fresh information should immediately
improve the scan centre; ignoring it would waste cells.

**D9 — Sensor and filter contracts are unchanged.** This is an
ACQUIRE-only behaviour change inside `fsm.py`. The FCR's gating math, the
TrackFilter, the SearchRadar, `SearchRadarLink`, the cue packet, and the
ballistic predictor are not touched. ACQUIRE's exit gate
(`is_locked() AND is_initialised()`) is unchanged. *Rationale:* keep the
blast radius small; the regression is in ACQUIRE, fix it there.

**D10 — Motor-velocity awareness: raster step is bounded by what the
turret can actually slew per timestep.** The scanner's per-step angular
displacement is `min(2 × fov_half_angle, max_motor_step_rad)`. If the cone
diameter is larger than the motors can slew in one step, the boresight is
moving slower than the gate cell — overlap is fine. If the motors can slew
faster than a cone diameter, we step by a full cone diameter so the gate
keeps up. *Rationale:* a raster that commands the boresight past where the
motors can physically reach in one step just wastes scan cells.

## Implementation sketch

The work is a new module + targeted edits to `fsm.py`. No changes outside
`controllers/atlas_controller/`.

- **New: `controllers/atlas_controller/acquisition_scanner.py`** — pure,
  filterpy-free, deterministic. Constructed with `cue_world: list[float]`,
  `cone_half_angle: float`, `scan_half_width: float`, `max_steps: int`, and
  `max_motor_step_rad: float`. Exposes `next_boresight() -> (pan_az,
  tilt_el)`, `recenter(new_cue_world)`, and `exhausted() -> bool`. Unit
  tests cover: centre-first ordering, exhaustive coverage of the box,
  bounded step count, recenter resets the counter, motor-step cap honoured.
- **Modified: `fsm.py` `_do_acquire`** — on entry, construct an
  `AcquisitionScanner` from the stored cue and `FSMConfig.acquire_scan_*`
  params; each step, command motors to `scanner.next_boresight()`; on a
  fresh cue mid-scan, call `scanner.recenter()`; on `scanner.exhausted()`,
  transition to SEARCH (D7); existing exit gate unchanged (D9).
- **Modified: `fsm.py` `FSMConfig`** — add `acquire_scan_half_width_rad`
  (default 0.4), `acquire_scan_max_steps` (default ~25 — tune in Webots),
  `acquire_max_motor_step_rad` (derived from motor `maxVelocity` ×
  timestep, default ~0.1).
- **Modified: `fsm.py` `_transition(ACQUIRE)`** — resets the scanner-entry
  bookkeeping flag alongside the existing `_acquire_entry_done` reset.

Out-of-scope refactors (D9) — `FireControlRadar`, `TrackFilter`,
`SearchRadar*`, `SearchRadarLink`, `BallisticTrajectoryPredictor`, the cue
packet format — are not touched.

## Out of scope

- **Covariance-shaped scan volume.** D5 explicitly uses a static box. A
  covariance-driven scan needs a measurement-fed filter at ACQUIRE entry,
  which contradicts the current hard-handoff design.
- **FCR FOV change.** D2 (single tracking cone) stands. No widening.
- **Search Radar changes.** Fix 2's revisit-rate change does its part; the
  SR is correct.
- **Continuous SR fusion into TrackFilter.** Still rejected per ADR-0010.
- **Motor-controller upgrades.** The motor model is what it is; D10
  adapts to it rather than changing it.
- **A second cue source.** One cueing radar (the SR) remains the contract.

## Verification

**Unit tests (primary gate, no live sim required for the agent):**

- `test_acquisition_scanner` — centre-first ordering: `next_boresight()`
  first call returns the cue bearing exactly; subsequent calls walk
  outward; the boresight sequence is deterministic for a given seed; the
  full sequence covers the box within `max_steps`; `exhausted()` becomes
  True after `max_steps`; `recenter()` resets the counter and recentres
  the spiral on the new cue.
- `test_fsm` — `_do_acquire` calls `scanner.next_boresight()` each step
  and aims the motors there; a fresh cue mid-scan triggers
  `scanner.recenter()`; FCR lock + filter-initialised exits to TRACK
  unchanged; scanner exhaustion exits to SEARCH (new transition).
- `test_fsm` — `_target_in_range` and the rest of the PREDICT path are
  unchanged (regression guard on existing behaviour).

**Manual Webots verification (human):** with the diagnostic
`[FCR REJECT]` line from the PR #23 follow-up, confirm that during the
scan the boresight moves through the angular box, that the
boresight-to-target separation crosses below `fov_half_angle` for at
least one timestep, that the FCR latches, and that the FSM reaches
ENGAGING. Tune `acquire_scan_half_width_rad`, `acquire_scan_max_steps`,
and `acquire_max_motor_step_rad` if the first lock is unreliable.

## Risk

- **Cue positional error exceeds the scan box.** If the SR cue is more
  than `acquire_scan_half_width_rad` off the true bearing, even the full
  scan misses. Mitigation: widen the box (cheap); allow D8 mid-scan
  recentering on the next SR cue cycle (already in the design).
- **Motor slew dominates scan-step time.** If the motors cannot slew a
  full cone diameter per timestep, the scan moves slower than expected
  and `max_steps` may be too small to cover the box. Mitigation: D10
  caps the raster step at the motor-feasible angle; `max_steps` is
  tuned to the slower of the two rates.
- **Target leaves the box before the scan finds it.** A very fast target
  may move out of the scan box during the scan. Mitigation: tune
  `max_steps` so the scan completes in well under a flight time; D7
  fallback to SEARCH means a re-cue cycles within ~21 SR steps anyway.
- **Spiral pattern is hand-rolled — off-by-one risk.** Mitigated by
  the scanner being a small, pure module with deterministic unit-test
  coverage.

## Verification of root cause (precondition)

This spec assumes the live `[FCR REJECT]` trace confirms a separation that
climbs above `fov_half_angle` within one timestep of ACQUIRE entry while
the boresight points at the cue. If the trace instead shows a different
gate failing (range, geometry mismatch, motor-position lag much larger
than expected), this spec should be revisited before implementation —
the wrong fix is worse than no fix.
