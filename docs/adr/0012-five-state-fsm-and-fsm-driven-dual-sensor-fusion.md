# ADR-0012: Five-State FSM and FSM-Driven Dual-Sensor Fusion

**Status:** Accepted
**Supersedes:** the state list in [ADR-0003](0003-search-radar-handoff-not-continuous-fusion.md); revisits the fusion stance of [ADR-0003](0003-search-radar-handoff-not-continuous-fusion.md) and [ADR-0010](0010-rejected-continuous-kalman-fusion-both-sensors.md)

## Decision

The ATLAS fire-control FSM is restructured from seven states
(`SEARCH → ACQUIRE → TRACK → PREDICT → AIMING → ENGAGING → RESET`) to **five**:

| State | Replaces | Role |
|-------|----------|------|
| `IDLE` | `SEARCH` | Hold a fixed beam direction (`idle_pan`, `idle_tilt`); no pan sweep. Wait for Search Radar cues. |
| `AIM` | `ACQUIRE` | Slew toward the cue, aiming off the raw cue only; wait for the FCR to lock. |
| `TRACK_PREDICT` | `TRACK` + `PREDICT` + `AIMING` | Fuse both sensors, follow the filtered estimate, predict the intercept, and fire once the predicted-vs-observed error converges. |
| `ENGAGING` | `ENGAGING` | Hold aim at the fired intercept. Exit on the attacker's ground-hit cue or out-of-range. |
| `RESET` | `RESET` | Wipe the Kalman filter and bookkeeping; return to `IDLE`. |

Two further decisions accompany the restructure:

1. **Dual-sensor fusion is reintroduced, FSM-driven.** The Track Filter regains
   `update_search()`. During `TRACK_PREDICT` the FSM fuses the Search Radar cue
   into the filter (high `R_search`, weak correction) alongside the FCR
   measurement (low `R_fcr`, fused continuously by the main loop). Fusion is
   confined to `TRACK_PREDICT` — `IDLE` and `AIM` do not fuse the Search Radar.
2. **No `laser_active` flag.** Entering `ENGAGING` *is* the "ready to fire"
   signal; consumers check `fsm.state == AtlasFSM.ENGAGING`. The dead boolean is
   removed.

## Context

The FSM was redesigned in `report/fsm.typ`. The seven-state machine carried
three states that were really one behaviour split by implementation convenience:
`TRACK` (follow), `PREDICT` (compute intercept), and `AIMING` (slew + converge).
Merging them into `TRACK_PREDICT` removes the artificial `PREDICT`-as-a-state
round trip and the `AIMING` convergence stage whose error metric (commanded-angle
delta) was acknowledged as a simplification.

`SEARCH` swept the pan motor; the redesign specifies an idle turret pointing in a
fixed direction, so the sweep is dropped.

## Reasoning

- **Single tracking-and-firing state** matches how the system actually behaves:
  once locked, the turret continuously tracks, predicts, and decides whether the
  prediction is trustworthy enough to fire. The firing gate is now an explicit,
  testable *prediction-stability* check — the intercept predicted `lookahead_steps`
  ago FOR the current step must land within `track_error_threshold` of the current
  estimate for `converge_frames` consecutive steps — rather than a motor-angle
  convergence proxy. This reuses the rolling-history pattern already proven in
  `telemetry.record_intercept_and_error`.

- **Reintroducing Search fusion** follows the redesign's explicit call for "sensor
  fusion of the Search Radar and the FCR data points" in `TRACK_PREDICT`.
  Confining it to that state preserves the spirit of the handoff (ADR-0003): the
  Search Radar still cues acquisition and does not drive `IDLE`/`AIM` tracking,
  but its world-frame cue now contributes a weak correction once tracking is
  underway. This differs from the *continuous, loop-level, all-states* fusion
  rejected in ADR-0010 — fusion here is state-scoped and FSM-owned, keeping the
  loop a pure cadence layer.

## Consequences

- `TrackFilter.update_search()` returns; `R_search` (already a constructor
  parameter) is now used.
- The main loop is unchanged: it still fuses the FCR continuously. Search fusion
  is invoked from the `TRACK_PREDICT` handler.
- ADR-0003's state names (`SEARCH`, `ACQUIRE`, `TRACK`) are superseded by this
  ADR's five-state vocabulary. ADR-0003's acquisition-handoff principle stands.
- The "projectile destroyed" exit named in the redesign is **not** implemented;
  `ENGAGING` keeps the ground-hit and out-of-range exits, with a `TODO` seam for
  the future bullet-hit cue (see `docs/superpowers/specs/2026-05-18-turret-weapon-design.md`).
