= Code Structure
// TODO: Briefly describe the structure of your program and explain:
// ● Main modules/functions
// ● Control loop
// ● Key libraries used

`atlas_controller.py` is the hub from which all of ATLAS's system logic is delegated. It
is the only module inside the turret package that talks to Webots directly — every other
module operates on plain Python objects passed in by the controller. At startup it
instantiates every module, wires their dependencies, and then loops on
`robot.step(timestep)` until the simulation ends. The loop body is deliberately thin —
no control logic lives in the controller itself; it only delegates, in a fixed order, to
the modules below and applies whatever commands they emit. The other Webots process,
`search_radar_controller/`, is comparatively simple: it sweeps a beam and emits a noisy
world-frame cue over radio, which ATLAS consumes as one of its inputs.

ATLAS's modules each carry a single responsibility. `FireControlRadar` is the on-board
radar; `SearchRadarLink`, `AttackerGroundHitLink`, and `BulletHitLink` are the radio
receivers for the Search Radar cue and the attacker's resolution pulses. `TrackFilter`
is the Kalman estimator (built on `filterpy`), `BallisticTrajectoryPredictor` computes
the intercept, and `AttackPredictor` (with its supporting `SectorMap` and `LaunchLatch`)
is the online learner. `AtlasFSM` is the state machine. `Bullet` owns the
recycled-projectile lifecycle. `telemetry.py` and `scoreboard.py` are observers and have
no effect on control.

Third-party libraries: `numpy` (linear algebra), `filterpy` (Kalman), `scikit-learn`
(`SGDClassifier` for online learning), and the Webots `controller` API. The modules are
written against stub interfaces, so the 26 `pytest` files under each controller's
`tests/` directory run without Webots.

// === REFERENCE: original prose version (kept for comparison; not included in build) ===
/*
The ATLAS software system is designed using modular architecture to separate all the
independent components. The central layer that dictates the main process is implemented
in atlas_controller.py. This is primarily responsible for constructing the system
modules, wiring them together, and executing the main simulation loop.


Core perception and tracking functionalities are distributed across the FireControlRadar
module (handling the narrow-field precision target tracking using field-of-view gating
and range filtering) and the SearchRadarLink (receives target cues from external Search
Radar controller). Projectile state estimation and prediction are handled by the
TrackFilter (implements Kalman filtering) and the BallisticTrajectoryPredictor (computes
future intercept locations for engagement). AtlasFSM controls the system’s behaviour
(managing IDLE, AIM, TRACK_PREDICT, ENGAGE, and RESET states). The adaptive behaviour is
implemented through the AttackPredictor (analyses the attack-launch patterns and
pre-slews the turret toward the future launch regions). Additional helper modules
include: LaunchLatch, Bullet, Scoreboard, TrackTelemetry, AttackerGroundHItLink and
BulletHitLink. These support projectile management, scoring, telemetry, and engagement
landing.


The main control loop operates using Webot’s fixed timestep execution model
(robot.step(timestep) != -1) and during each timestep, the system follows Sense ->
Process -> Decide -> Act architecture. Sense focuses on updating radar cues,
projectile-hit signals, and FCR measurements. Process discusses the states of: Predict
which focuses on the Kalman filter state estimate and Fuse which focuses on integrating
the new radar observations into the filter. Decide advances onto the FSM and determine
the tracking and engagement actions. Act is moving the turret, firing the bullet toward
the intercept point and updating the engagement state.


Several external Python libraries were used throughout this project. The Webots Python
API (controller.Supervisor) provides access to robot devices, simulation state, and
Supervisor functionality. numpy was used for numerical calculations and geometry
handling, while filterpy provides the Kalman filtering framework used in projectile
tracking. scikit-learn is used within the adaptive attack-pattern learner for
lightweight machine-learning classification, and standard Python libraries such as math
and os are used for geometry and configuration handling. The repository also includes
pytest for automated unit testing and validation.
*/
