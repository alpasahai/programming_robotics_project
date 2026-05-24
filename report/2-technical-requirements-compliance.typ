= Technical Requirements Compliance
// TODO: In this section you must explicitly explain how your system satisfies all technical
// requirements listed in the assignment sheet.
// For each requirement, briefly describe how your system implements it.
// This section helps reviewers verify that your system meets the required specifications

ATLAS is a complex system with more components and interfaces than are enumerated in
this section. To keep the report concise, only the principal features that demonstrate
alignment with each requirement are presented here.
== Inputs
- World-frame target cue $(x, y, z)$ from the external Search Radar process, delivered
  over a Webots radio link. Fed into the FSM as the `IDLE` → `AIM` trigger (debounced
  detection count) and the slew target for `AIM`, and into the `AttackPredictor` as the
  per-launch observation that drives online sector learning.
- Turret-relative projectile position from ATLAS's own on-board Fire Control Radar,
  gated by range and FOV and produced only when locked. Fed into the Kalman
  `TrackFilter` as the low-noise measurement update, which the
  `BallisticTrajectoryPredictor` then propagates to an intercept; the lock flag also
  gates the FSM `AIM` → `TRACK_PREDICT` transition.

== Outputs
- Pan and tilt motor position commands to the turret's azimuth and elevation actuators,
  slewing the boresight onto the target.
- Bullet teleport-to-muzzle and velocity write toward the predicted intercept — the
  physical fire action that engages the threat.

== FSM and Behavioural States
#figure(
  image("fsm.pdf", width: 30%),
  caption: [ATLAS finite state machine (FSM) diagram],
) <FSM>
@FSM shows ATLAS' FSM contains 5 behavioural states, demonstrating alignment with the
FSM and minimum behavioural states requirement.

== Multi-Condition Decision Logic
The transition labels in @FSM are summaries; the actual guards are multi-condition.

`TRACK_PREDICT` → `ENGAGING`: prediction error below threshold AND intercept in range
AND sustained for N consecutive steps AND turret not aiming at Search Radar.

`ENGAGING` → `RESET`: ground-hit cue OR bullet-hit cue OR target out of range.

== Safety or fail-safe mechanisms
Convergence gate: `TRACK_PREDICT` → `ENGAGING` requires prediction error below threshold
AND intercept in range for `converge_frames` consecutive steps, so the turret cannot
commit a shot on a single-frame low-error blip.

`RESET` recovery: every state has a path to `RESET`, which wipes the Kalman filter,
clears the stale cue, and returns to `IDLE` — no corrupted track or unresolved
projectile can persist across engagements.

Anti-friendly-fire exclusion zone: every fire command is gated against the measured pan
angle; if the boresight is within $plus.minus 0.4$ rad ($approx 23 degree$) of the
Search Radar's bearing, the shot is suppressed, preventing rounds into the friendly
radar.

== Sensor Fusion
A Kalman filter fuses two heterogeneous radar sources: the wide-area Search Radar (high
noise, `R_SEARCH = 0.1`) and the narrow-beam FCR (low noise, `R_FCR = 0.001`). The
filter weights precise FCR measurements far more heavily than coarse Search Radar cues.
The predict step runs every tick regardless of measurement availability, so a sensor
dropout does not collapse the estimate.

== Context-aware Behaviour
The turret adapts to engagement context instead of following hard-coded triggers. In
`IDLE`, the `AttackPredictor` (online logistic regression over discovered launch
sectors) supplies a predicted next-sector bearing and the turret pre-slews toward it.
The convergence gate also waits longer when the track is volatile and fires sooner when
it is stable, rather than firing on a fixed timer.

== Real-time Logic
ATLAS runs a synchronous Sense → Predict → Fuse → Decide loop on the Webots simulation
clock. Sensor sampling, Kalman prediction, and FSM evaluation run every tick
unconditionally, so the world model is always fresh when a decision is made.

== Advanced / Contextual Component - AI or Adaptive Behaviour
The `AttackPredictor` is an online logistic regression (`SGDClassifier`, `partial_fit`
per launch) over sectors discovered online by leader-clustering bearings, with
EMA-tracked centres that follow attacker drift. Features are a one-hot of the current
sector plus a run-length count, modelling burst-switching. A `LaunchLatch` yields one
observation per engagement, and prediction is gated until $gt.eq 2$ sectors are seen.
The predicted next-sector bearing pre-slews the turret in `IDLE`, shortening
time-to-acquire on the next launch.
