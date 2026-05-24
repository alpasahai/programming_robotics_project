= Perception / Sensor Processing <sec:perception>
// TODO: Explain how sensor data is used, for example:
// ● Sensor fusion
// ● Multi-condition logic
// ● Camera detection
// ● Colour tracking
// ● Distance measurement
// Explain:
// ● What data is read
// ● How it affects decisions

Perception is built from radar, inter-process radio, and joint-angle sensors. Each tick
the controller drains five streams: a coarse world-frame cue from the Search Radar
(noise std $approx 0.2$ m), a turret-relative position from the on-board Fire-Control
Radar (FCR) when locked (noise std $approx 0.02$ m), one-shot ground-hit and bullet-hit
pulses, and the pan/tilt joint angles giving the measured boresight.

At the core of the pipeline is a Kalman filter — a running estimate of the projectile's
position $(x, y, z)$ and velocity $(v_x, v_y, v_z)$. The *predict* step advances the
estimate each tick using projectile physics (position by velocity, gravity on velocity),
so a transient dropout cannot collapse the track. The *update* step blends incoming
measurements with the prediction; radars report position only, so velocity is inferred
from its evolution over time. Measurement weight is governed by the noise variance $R$,
and the two radars are separated by two orders of magnitude — $R_"FCR" = 0.001$ vs
$R_"SEARCH" = 0.1$ — so the FCR dominates corrections. The FCR is fused on every locked
tick; the Search Radar is fused only during `TRACK_PREDICT`, adding a weak long-range
correction without polluting the precise track. The resulting estimate feeds the
`BallisticTrajectoryPredictor`, which extrapolates to a closed-form intercept.

These products drive the FSM through compound guards. The FCR lock flag promotes `AIM`
$arrow$ `TRACK_PREDICT`; the fire decision is stricter, requiring the
predicted-vs-observed error to stay below threshold AND the intercept to stay in range,
both for several consecutive frames, before `TRACK_PREDICT` $arrow$ `ENGAGING`. A final
gate suppresses the shot if the measured pan angle lies inside the friendly-radar
exclusion cone.

// === REFERENCE: original prose version (kept for comparison; not included in build) ===
/*
The system implements a dual-radar perception architecture made up of the Search Radar
and a Fire-Control Radar (FCR). Since the native Webots Radar device only supports 2D
radar measurements without elevation data, both radars were implemented as custom
simulated 3D radar systems in Python. The radars are represented by a physical Webots
node with a real position and orientation in the environment, while the detection logic
itself is simulated through Supervisor access to projectile ground-truth positions.


The Search Radar is a wide-area acquisition radar that continuously scans the
environment using a broader field-of-view (FOV). It periodically emits coarse
world-frame projectile position cues with intentionally high noise added to the
measurements. This creates long-range radar uncertainty while still allowing reliable
target discovery. The Search Radar communicates these cues to the FCR through a
cue-handoff process.


The FCR is implemented as a narrow-beam precision tracking radar mounted directly onto
the turret. The FCR only monitors targets that fall within its restricted FOV cone and
maximum engagement range. Once the turret slews toward the incoming Search Radar cue and
the projectile enters the FCR cone, the system transitions into closed-loop tracking.
The FCR produces lower-noise measurements, allowing for accurate predictions when
following the projectile position.


Sensor measurements are processed using a Kalman-based TrackFilter which continuously
predicts projectile motion and corrects the prediction using incoming FCR observations;
this reduces sensor jitter and smooths noisy radar measurements. This allows the system
to estimate projectile position and velocity more reliably for future intercept
prediction. The filtered projectile estimate is then used by the TRACK_PREDICT state to
compute a ballistic intercept point ahead of the projectile trajectory. ATLAS also
includes an adaptive attack pattern prediction feature. The attacker launches
projectiles according to predefined frequency-based attack patterns with slight random
variation. By observing projectile launch timing and trajectory behaviour over time, the
system can pre-aim the turret toward regions where future launches are more likely to
occur. This reduces acquisition delay and improves interception readiness during
repeated attack sequences.
*/
