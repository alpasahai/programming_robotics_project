# ADR-0006: Sensor Membrane and Track-ID Target Identity

**Status:** Accepted
**Supersedes (in part):** ADR-0003-continuous-kalman-fusion-both-sensors.md — the
clause "the FSM only controls set_target() ... at the ACQUIRE transition" is
revised here (see Decision).

## Decision

**The sensor membrane.** Webots `Solid` node handles are treated as a
*capability to read ground truth* — anything holding a node can call
`getPosition()` and obtain the true, un-noised position. Such handles must not
propagate outside the radar classes. `FireControlRadar` and `SearchRadar` are
the membrane: inside them, code touches ground truth and adds simulated noise;
everything downstream — `TrackFilter`, `BallisticTrajectoryPredictor`, the FSM —
handles only noised measurements and abstract integer `track_id`s.

**Track-ID target identity.** `SearchRadar.get_detections()` returns
`Detection` values — an immutable `(track_id: int, position: list[float])`
pair. The FSM selects a target by `track_id` and passes that integer to
`SearchRadar.set_target(track_id)`. The SearchRadar owns the `track_id → node`
mapping and resolves it internally.

**FCR is single-target, cued at construction.** The Fire-Control Radar tracks
exactly one target. It receives its projectile node at construction and is not
re-targeted by the FSM. `FireControlRadar.set_target(node)` remains in the
interface for future controller-driven re-cueing, but the FSM does not call it.
Consequently the FSM, at ACQUIRE, locks only the SearchRadar (and resets the
TrackFilter) — it does **not** call `fcr.set_target`.

## Context

At the ACQUIRE transition the FSM must lock a sensor onto a chosen target. The
FSM's only view of the scene is `SearchRadar.get_detections()`. Originally that
returned bare positions, so the FSM had no way to name the target a `set_target`
call needs.

The deeper issue: the project *simulates* sensors — the radars read the
projectile's true Webots position and add Gaussian noise to fake an imprecise
measurement. A Webots node handle is live access to that true position. If a
node escaped the radar classes into the FSM or the estimator, downstream code
could bypass the noise simulation and read ground truth — biasing the system.
This boundary existed in design intent (ADR-0004) but was never codified.

## Options considered

1. **Detection carries a Webots node.** The FSM holds a node in `self._target`.
   Rejected: a live `getPosition()` capability sits in FSM state for five
   states — a latent membrane leak, clean only by discipline.
2. **Detection carries an opaque handle.** Rejected: leak-free only if the
   handle is genuinely inert; opacity by convention is fragile.
3. **Detection carries an integer `track_id` (chosen).** An integer carries no
   capability — the FSM cannot read truth from it. The membrane holds by
   construction, not discipline.

A variant of (3) had FCR also resolve `track_id`s, which required giving FCR the
full projectile list. Rejected: FCR tracks one target by definition and never
needs a node universe. Only the wide-beam SearchRadar does.

## Reasoning

Track-IDs make the membrane a structural property: no node-typed value exists in
the FSM's scope, so no leak is possible regardless of future edits. Keeping the
`track_id → node` map solely in SearchRadar matches the domain — the wide-beam
acquisition radar is the component that sees every target. Leaving FCR
single-target keeps it untouched and honest to its narrow-beam nature.

## Consequences

- `Detection` is a small shared value type (`search_radar.py`) used by the
  SearchRadar, the FSM, and the test stubs.
- `track_id` is the projectile's index in the list passed to `SearchRadar`. The
  list is built once and never reordered, so IDs are stable.
- Re-cueing FCR onto a different target (a multi-projectile future) is a
  controller responsibility — the controller holds nodes legitimately — not the
  FSM's. This is the upgrade path; it does not change the membrane rule.
