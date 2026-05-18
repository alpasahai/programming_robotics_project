# ADR-0006: Sensor Membrane and Track-ID Target Identity

**Status:** Accepted, revised by ADR-0008 and ADR-0009
**Supersedes (in part):** ADR-0010 — the continuous-fusion design and the
clause "the FSM only controls set_target() ... at the ACQUIRE transition" are
rejected by the current cue-handoff design.

## Decision

**The sensor membrane.** Webots `Solid` node handles are treated as a
*capability to read ground truth* — anything holding a node can call
`getPosition()` and obtain the true, un-noised position. Such handles must not
propagate outside the radar processes and sensor classes.

The membrane now spans a process boundary. The Search Radar process owns
projectile nodes, `Detection` values, and `track_id` discipline internally. The
radio cue crossing into `atlas_controller` is only a bare world position:
`[x, y, z]`. No Webots node handle and no `track_id` reaches the ATLAS FSM.

**FCR is a single-target turret-aim sensor.** The Fire-Control Radar tracks the
one projectile supplied by the controller and detects it only when the turret's
commanded aim places it inside the FOV cone. The FSM does not retarget the FCR,
and `FireControlRadar.set_target()` no longer exists.

## Context

At the ACQUIRE transition the FSM must point the turret toward a target without
receiving a ground-truth capability. In the earlier in-process design the FSM's
only view of the scene was `SearchRadar.get_detections()`, and `track_id`
provided an inert target identity. In the revised two-process design, even that
identity stays inside the Search Radar process.

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
3. **Detection carries an integer `track_id` (previous in-process design).** An
   integer carries no capability — the FSM cannot read truth from it. This was
   acceptable while Search Radar lived inside `atlas_controller`.
4. **Radio cue carries only a world position (chosen current design).** The
   Search Radar process keeps `track_id`s and node handles private. ATLAS gets
   enough information to slew the turret, but not enough to bypass the sensor
   model.

A variant had FCR also resolve `track_id`s, which required giving FCR the full
projectile list. Rejected: FCR tracks one target by definition and never needs a
node universe. Only the wide-beam Search Radar process does.

## Reasoning

The process boundary makes the membrane a structural property: no node-typed
value exists in the FSM's scope, so no leak is possible regardless of future
edits. Keeping `track_id → node` state solely in the Search Radar process
matches the domain — the wide-beam acquisition radar is the component that sees
and selects targets. Leaving FCR single-target keeps it honest to its
narrow-beam nature.

## Consequences

- `Detection` and `track_id` are internal to `search_radar_controller`.
- `SearchRadarLink` exposes only `get_cue() -> list[float] | None`.
- `TrackFilter` receives FCR measurements only; Search Radar cues are not
  filter measurements.
- Multi-projectile target selection remains a Search Radar process
  responsibility. The ATLAS FSM consumes only the chosen cue.
