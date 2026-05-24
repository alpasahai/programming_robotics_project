= Technical Requirements Compliance
// TODO: In this section you must explicitly explain how your system satisfies all technical
// requirements listed in the assignment sheet.
// For each requirement, briefly describe how your system implements it.
// This section helps reviewers verify that your system meets the required specifications

ATLAS is a complex system with more components and interfaces than are enumerated in
this section. To keep the report concise, only the principal features that demonstrate
alignment with each requirement are presented here.

== General Requirements
=== Inputs
- World-frame target cue $(x, y, z)$ from the external Search Radar process, delivered
  over a Webots radio link. Fed into the FSM as the `IDLE` → `AIM` trigger (debounced
  detection count) and the slew target for `AIM`, and into the `AttackPredictor` as the
  per-launch observation that drives online sector learning.
- Turret-relative projectile position from ATLAS's own on-board Fire Control Radar,
  gated by range and FOV and produced only when locked. Fed into the Kalman
  `TrackFilter` as the low-noise measurement update, which the
  `BallisticTrajectoryPredictor` then propagates to an intercept; the lock flag also
  gates the FSM `AIM` → `TRACK_PREDICT` transition.

=== Outputs
- Pan and tilt motor position commands to the turret's azimuth and elevation actuators,
  slewing the boresight onto the target.
- Bullet teleport-to-muzzle and velocity write toward the predicted intercept — the
  physical fire action that engages the threat.

=== FSM and Behavioural States
#figure(
  image("fsm.pdf", width: 40%),
  caption: [ATLAS finite state machine (FSM) diagram],
) <FSM>
@FSM shows ATLAS' FSM contains 5 behavioural states, demonstrating alignment with the
FSM and minimum behavioural states requirement.

=== Multi-Condition Decision Logic
The transition labels in @FSM are summaries; the actual guards are multi-condition.

`TRACK_PREDICT` → `ENGAGING`: prediction error below threshold AND intercept in range
AND sustained for N consecutive steps.

`IDLE` → `AIM`: cue present for N consecutive steps (debounced; single-frame flickers
are rejected).

`ENGAGING` → `RESET`: ground-hit cue OR bullet-hit cue OR target out of range.

=== Safety or fail-safe mechanism
// TODO: update thsi with the anti-friendly fire mechanism
ATLAS implements two principal fail-safe mechanisms.

Convergence gate before firing: the turret will not fire on a single-frame "good
prediction". `TRACK_PREDICT` only transitions to `ENGAGING` once the prediction error
stays below threshold AND the intercept stays within range for `converge_frames`
consecutive steps, preventing the system from committing a shot on a spurious low-error
blip.

`RESET` recovery state: every state has a path to `RESET`, which wipes the Kalman
filter, clears the stale Search Radar cue, and returns the FSM to `IDLE`. A corrupted
track, a missed projectile, or any unexpected condition cannot persist across
engagements. This ensures the system always recovers to a clean starting state before
the next launch.

== Embedded Intelligence Requirements
=== Sensor Fusion
A Kalman filter fuses two heterogeneous radar sources: the wide-area Search Radar (high
noise, `R_SEARCH = 0.1`) and the narrow-beam FCR (low noise, `R_FCR = 0.001`). The
filter weights precise FCR measurements far more heavily than coarse Search Radar cues.
The predict step runs every tick regardless of measurement availability, so a sensor
dropout does not collapse the estimate.

=== Context-aware Behaviour
The turret adapts to engagement context instead of following hard-coded triggers. In
`IDLE`, the `AttackPredictor` (online logistic regression over discovered launch
sectors) supplies a predicted next-sector bearing and the turret pre-slews toward it.
The convergence gate also waits longer when the track is volatile and fires sooner when
it is stable, rather than firing on a fixed timer.

=== Real-time Logic
ATLAS runs a synchronous Sense → Predict → Fuse → Decide loop on the Webots simulation
clock (32 ms per tick). Sensor sampling, Kalman prediction, and FSM evaluation run every
tick unconditionally, so the world model is always fresh when a decision is made.
