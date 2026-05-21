# Attacker Ground-Hit Cue → ATLAS (design)

**Date:** 2026-05-21
**Status:** Approved (brainstorm)

## Problem

ATLAS currently decides "the incoming ball has landed" by watching its own
TrackFilter estimate: in ENGAGING it exits to RESET when the live target's
world-Z drops at/below `FSMConfig.ground_threshold`. That is an *inference* from
a noisy estimate, not a real contact, and it duplicates ground-truth that the
attacker already owns — the attacker detects landings from real physical
contact points (`Supervisor.getContactPoints`, see `projectile.py`).

We want a single, authoritative source of ground-hit truth: **the attacker
emits a ground-hit signal, and that emission is the only thing in the ATLAS
process that represents a ground hit.**

## What already exists

- `protos/Attacker.proto` already declares `ATTACKER_EMITTER` — `type "radio"`,
  `channel 2`, `range -1`. Currently unused by `attacker_controller.py`.
- `protos/AtlasTurret.proto` already declares the receiver
  `ATTACKER_GROUND_HIT_RECEIVER` on `channel 2`. Currently unused by
  `atlas_controller.py`.
- `attacker_controller.py` has an `on_ground_hit(count)` seam already fired by
  `Projectile` on a real floor contact (count = running ground-hit total).
- The radio-cue pattern is established by SearchRadar → `SearchRadarLink`
  (channel 1, `struct.pack("ddd", …)`); see ADR-0009. This work mirrors it.

So no proto changes and no projectile-logic changes are needed — only
controller wiring plus an FSM cleanup.

## Payload

`struct.pack("i", count)` — a single 4-byte signed int carrying the running
ground-hit count.

- **One-shot pulse:** the attacker sends exactly one packet on the step a ground
  hit registers, and nothing on any other step (mirrors SearchRadar's "send
  nothing when there is no cue").
- The count lets ATLAS tell successive hits apart and notice dropped packets;
  the consumer treats *any* packet this step as "a ground hit happened this
  step."
- Channel 2 + `"i"` is distinct from SearchRadar's channel 1 + `"ddd"`, so the
  two receivers never cross-talk.

## Components

### Attacker side — `controllers/attacker_controller/attacker_controller.py`

- `emitter = robot.getDevice("ATTACKER_EMITTER")`.
- In `on_ground_hit(count)` (already the landing seam), add
  `emitter.send(struct.pack("i", count))` alongside the existing log line.
- No change to `Projectile`; the seam already fires on physical floor contact.

### ATLAS side — new `controllers/atlas_controller/attacker_ground_hit_link.py`

`AttackerGroundHitLink`, a sibling of `SearchRadarLink`:

- Constructed around an **already-enabled** receiver (the controller enables it;
  the class does not — keeps wiring in the controller, keeps the class testable
  with a stub). Stub surface: `getQueueLength()`, `getBytes()`, `nextPacket()`.
- `update() -> None`: drains the queue. Sets a per-step "hit this step" flag
  True if at least one packet arrived during this call (else False), and stores
  the latest decoded count. Computing the flag once per `update()` matches the
  FSM's once-per-step cadence.
- `hit_this_step() -> bool`: True iff a ground-hit pulse arrived during the most
  recent `update()`.
- `count -> int`: most recent received count (0 before any hit).

### ATLAS controller wiring — `controllers/atlas_controller/atlas_controller.py`

- `gh_receiver = robot.getDevice("ATTACKER_GROUND_HIT_RECEIVER")`,
  `gh_receiver.enable(timestep)`.
- Build `AttackerGroundHitLink(gh_receiver)`; pass it into the FSM's
  `SensorSuite` (see below).
- In the sense phase, call `ground_hit_link.update()` (alongside
  `cue_link.update()`), and **log** when `hit_this_step()` is true (count + sim
  time).

### FSM cleanup — `controllers/atlas_controller/fsm.py`

- Add `ground_hit_link` to `SensorSuite`.
- **ENGAGING → RESET:** exit to RESET when the ground-hit cue fired this step
  **OR** the live target is beyond `max_range`. Remove the
  `dz <= ground_threshold` "landed" clause from the ENGAGING exit. Ground-hit
  truth now comes solely from the emission.
- **PREDICT:** validate the intercept against `max_range` **only**. The
  below-`ground_threshold` rejection is removed (decision: "get rid of the
  predict guard for now").
- `_target_in_range` collapses to a **range-only** predicate, shared by PREDICT
  and ENGAGING. `ground_threshold` no longer influences any FSM decision.
- `ground_threshold` stays as an `FSMConfig` field because `telemetry.py` reads
  it for its `ground_check` readout (a pure development instrument, no
  behaviour). FSM logic no longer references it.

#### Tradeoff (accepted)

With the PREDICT guard gone, a descending ball near landing can yield a
predicted intercept below the floor, so AIMING/ENGAGING may briefly slew the
turret to a sub-floor point until the ground-hit cue fires and triggers RESET.
This is a few cosmetic steps in simulation and is accepted for now. The upside:
the FSM makes no ground decisions at all — the emitted cue is the only
ground-hit signal in the ATLAS process.

## Data flow

```
attacker: Projectile floor contact → on_ground_hit(count)
          → ATTACKER_EMITTER.send(pack("i", count))   [channel 2, pulse]
                              │  (one-step radio delivery lag, per ADR-0009)
                              ▼
atlas:    ATTACKER_GROUND_HIT_RECEIVER → AttackerGroundHitLink.update()
          → hit_this_step() / count
          → controller logs;  FSM ENGAGING reads hit_this_step() → RESET
```

## Testing

New — `tests/test_attacker_ground_hit_link.py`:
- empty queue → `hit_this_step()` False, `count` unchanged;
- one packet → `hit_this_step()` True, `count` decoded;
- multiple packets drained in one `update()` → True, latest count kept;
- flag clears to False on a subsequent `update()` with an empty queue.

FSM — `tests/test_fsm.py`:
- rework `test_engage_transitions_to_reset_when_target_hits_ground` to drive a
  stub `ground_hit_link.hit_this_step()` → True (instead of a sub-threshold Z);
- add: ENGAGING with an in-range descending target and **no** cue stays in
  ENGAGING (proves the Z-landing inference is gone);
- ENGAGING exits to RESET when target goes beyond `max_range` with no cue
  (range envelope still works);
- **remove** `test_predict_below_ground_intercept_transitions_to_track` (the
  guard it covers no longer exists);
- update `_make_fsm_in_*` / `SensorSuite` construction helpers to supply a stub
  ground-hit link.

Existing `test_default_lookahead_keeps_intercept_above_ground` (in
`test_ballistic_predictor.py`) is unaffected — it asserts the *predictor's*
output, not the FSM guard.

## Out of scope

- Scoring / referee logic on either side (the attacker's `on_ground_hit` log
  remains the scoring seam).
- Freshness/expiry timers on the link (matches `SearchRadarLink`: latest packet
  wins, no timeout).
- Any change to `Projectile` contact classification.
