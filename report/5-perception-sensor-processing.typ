= Perception / Sensor Processing
// TODO: Explain how sensor data is used, for example:
// ● Sensor fusion
// ● Multi-condition logic
// ● Camera detection
// ● Colour tracking
// ● Distance measurement Explain:
// ● What data is read
// ● How it affects decisions

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
