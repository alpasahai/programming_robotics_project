"""ATLAS turret Webots supervisor controller.

Thin glue layer: constructs all components, wires them together, then runs
the per-timestep loop. No logic lives here — all decision-making is inside
the tested modules (FireControlRadar, SearchRadarLink, TrackFilter,
BallisticTrajectoryPredictor, AtlasFSM). The incoming projectile's
launch/relaunch lifecycle is owned by the separate attacker_controller; this
controller only reads the projectile's position as ground-truth telemetry.

Two clocks, one loop
--------------------
This loop sequences two different notions of "state", and keeping them
distinct is the key to reading it:

  * Sensor sampling state is driven by the *simulation clock*. Every sensor
    is sampled once per physics tick, unconditionally, so the world model
    stays current no matter what the turret is doing. That cadence lives
    here, in the loop (steps 1-3 below). These calls carry no decisions.
  * FSM state is driven by *events* (locks, cue counts, ranges). It only
    advances when conditions are met, and all of that — every if/then — lives
    inside AtlasFSM (step 4). The loop never inspects sensor readings to make
    a control decision; it just keeps the senses ticking and lets the FSM
    read an always-fresh world model.

So the brain is the FSM. This loop is pure cadence: it guarantees the senses
fire on the clock, in a fixed order, exactly once per tick.

Execution order each step (per ADR-0003 continuous fusion and the FSM plan):
  1. Sense   — cue_link.update(), fcr.update(turret_aim)   [sim-clock cadence]
  2. Predict — track_filter.predict()                      [sim-clock cadence]
  3. Fuse    — track_filter.update_fcr() when FCR is locked [sim-clock cadence]
  4. Decide  — fsm.step()                                  [event-driven state]
  4a. Launch — bullet.launch() for a bullet armed last step [event-driven state]
  4b. Fire   — fsm.consume_fire_command(), bullet.fire()    [event-driven state]
  4c. Recycle — bullet.step(), bullet.recycle() on engagement end [event-driven]
  5. Report  — telemetry.report() (development instrument; no effect on control)
"""

import math

from controller import Supervisor

from atlas_logging import configure
from fire_control_radar import FireControlRadar
from search_radar_link import SearchRadarLink
from attacker_ground_hit_link import AttackerGroundHitLink
from track_filter import TrackFilter
from ballistic_trajectory_predictor import BallisticTrajectoryPredictor
from fsm import AtlasFSM, SensorSuite, TurretHardware
from telemetry import TrackTelemetry
from bullet_hit_link import BulletHitLink
from bullet import Bullet, BulletConfig
from scene import DEF_PROJECTILE, DEF_ATLAS_BULLET
from launch_point_estimator import LaunchPointEstimator
from launch_observation_gate import LaunchObservationGate

# ---------------------------------------------------------------------------
# Telemetry logging
#
# A StreamHandler lands in the Webots console; a FileHandler writes a reviewable
# trace to controllers/atlas_controller/. The console level is set centrally in
# lib/atlas_logging.py (LOG_LEVELS["AtlasController"]) so verbosity for every
# controller lives in one place; override per run with ATLASCONTROLLER_LOG_LEVEL
# or ATLAS_LOG_LEVEL.
# ---------------------------------------------------------------------------

log = configure("AtlasController", log_file="atlas_telemetry.log")

# ---------------------------------------------------------------------------
# Tuning constants
#
# Sensor geometry/noise and Kalman filter noise, kept here as a single labelled
# place (mirrors the SEARCH_RADAR_* block in search_radar_controller). Note the
# FCR detection range (20 m) is wider than the FSM engagement range
# (FSMConfig.max_range, 10 m): the turret detects farther than it shoots.
# ---------------------------------------------------------------------------

# Fire-Control Radar — narrow-beam, precise.
FCR_FOV_HALF_ANGLE_RAD = 0.1  # ~5.7° half-angle
FCR_MAX_RANGE_M = 20.0        # detection range (cf. FSM engagement range)
FCR_NOISE_STD_M = 0.02        # low noise — FCR is precise (metres std)

# Track filter (Kalman) noise.
TRACK_FILTER_R_FCR = 0.001    # FCR measurement noise variance (m²) — low, trusted
TRACK_FILTER_R_SEARCH = 0.1   # retained for constructor; update_search() removed
TRACK_FILTER_Q = 0.01         # process noise scale — small for near-ballistic motion

# --- Adaptive launch-point estimator ---
LAUNCH_POINT_ALPHA = 0.15   # forgetting/adaptation rate (noise-robustness vs drift)

# --- Turret weapon (bullet) ---
# v_max < (r_ball + r_bullet)/timestep ≈ 20 m/s at 32 ms (ODE is discrete — no
# CCD — so a too-fast bullet tunnels through the ball). Tune in Webots.
MUZZLE_SPEED_MPS = 18.0
MUZZLE_OFFSET_M = 0.6           # spawn this far along the aim, clear of the turret
BULLET_PARK_POSITION = [0.0, 0.0, -100.0]
BULLET_MAX_LIFETIME_STEPS = 400  # safety: recycle a bullet that never resolves

# ---------------------------------------------------------------------------
# Setup
# ---------------------------------------------------------------------------

robot = Supervisor()
timestep = int(robot.getBasicTimeStep())

# --- Devices ---
pan = robot.getDevice("PAN_MOTOR")
tilt = robot.getDevice("TILT_MOTOR")

# Position sensors on the pan/tilt joints. These report the turret's REAL aim
# (the measured joint angle), which the FCR gates its lock on — so the FCR only
# locks once the barrel has physically slewed onto the target, not the instant
# the FSM commands the angle. Must be enabled before the loop reads them.
pan_sensor = robot.getDevice("PAN_SENSOR")
tilt_sensor = robot.getDevice("TILT_SENSOR")
pan_sensor.enable(timestep)
tilt_sensor.enable(timestep)

# Receiver for Search Radar cues delivered over the inter-process radio link.
# Must be enabled before constructing SearchRadarLink.
receiver = robot.getDevice("FCR_CUE_RECEIVER")
receiver.enable(timestep)

# Receiver for the attacker's ground-hit pulse (channel 2). Enabled before the
# link is constructed, like the Search Radar receiver above.
ground_hit_receiver = robot.getDevice("ATTACKER_GROUND_HIT_RECEIVER")
ground_hit_receiver.enable(timestep)

# Receiver for the bullet-hit pulse (channel 3). Enabled before the link is
# constructed, matching the pattern above.
bullet_hit_receiver = robot.getDevice("ATLAS_BULLET_HIT_RECEIVER")
bullet_hit_receiver.enable(timestep)

# --- Scene nodes ---
projectile = robot.getFromDef(DEF_PROJECTILE)
if projectile is None:
    raise RuntimeError(f"Could not find DEF {DEF_PROJECTILE} in the world file.")

turret_position = (
    robot.getSelf().getPosition()
)  # static snapshot — turret base never moves

# --- Sensors ---
# Tuning parameters: FCR is narrow-beam / precise; cue_link delivers coarse
# world-frame cues from the separate Search Radar process.
fcr = FireControlRadar(
    projectile,
    fcr_position=turret_position,   # FCR is co-located with the turret
    turret_position=turret_position,
    fov_half_angle=FCR_FOV_HALF_ANGLE_RAD,
    max_range=FCR_MAX_RANGE_M,
    noise_std=FCR_NOISE_STD_M,
)
cue_link = SearchRadarLink(receiver)
ground_hit_link = AttackerGroundHitLink(ground_hit_receiver)
bullet_hit_link = BulletHitLink(bullet_hit_receiver)
launch_point_estimator = LaunchPointEstimator(alpha=LAUNCH_POINT_ALPHA)
launch_observation_gate = LaunchObservationGate()

bullet_node = robot.getFromDef(DEF_ATLAS_BULLET)
if bullet_node is None:
    raise RuntimeError(f"Could not find DEF {DEF_ATLAS_BULLET} in the world file.")
bullet = Bullet(
    bullet_node,
    turret_position=turret_position,
    config=BulletConfig(
        muzzle_speed=MUZZLE_SPEED_MPS,
        muzzle_offset_m=MUZZLE_OFFSET_M,
        park_position=BULLET_PARK_POSITION,
    ),
)

# --- State estimator ---
# R_search is retained as a constructor parameter (TrackFilter still accepts it)
# but update_search() is gone — only update_fcr() is called.
track_filter = TrackFilter(
    timestep_ms=timestep,
    R_fcr=TRACK_FILTER_R_FCR,
    R_search=TRACK_FILTER_R_SEARCH,
    Q=TRACK_FILTER_Q,
)

# --- Ballistic predictor ---
ballistic_predictor = BallisticTrajectoryPredictor(track_filter, timestep_ms=timestep)

# --- FSM wiring ---
sensors = SensorSuite(
    cue_link=cue_link,
    fcr=fcr,
    track_filter=track_filter,
    ballistic_predictor=ballistic_predictor,
    ground_hit_link=ground_hit_link,
    bullet_hit_link=bullet_hit_link,
    idle_aim_provider=launch_point_estimator,
)
hardware = TurretHardware(
    pan_motor=pan,
    tilt_motor=tilt,
    turret_position=turret_position,
    timestep_ms=timestep,
)
fsm = AtlasFSM(sensors, hardware)  # uses default FSMConfig

# The incoming projectile's launch/relaunch lifecycle is owned by the separate
# attacker_controller. atlas_controller only reads its position (ground truth
# for telemetry) via the DEF PROJECTILE handle obtained above.

# --- Telemetry (development instrument; compares against ground truth) ---
telemetry = TrackTelemetry(
    log,
    lookahead_steps=fsm.config.lookahead_steps,
    max_range=fsm.config.max_range,
    ground_threshold=fsm.config.ground_threshold,
    turret_position=turret_position,
    timestep_ms=timestep,
)
telemetry.log_legend()

step_count = 0  # simulation steps elapsed — shown in telemetry

# ---------------------------------------------------------------------------
# Main loop
#
# Steps 1-3 run on the simulation clock: they sample the world once per physics
# tick, unconditionally, keeping the world model current regardless of FSM
# state. They contain no control decisions. Step 4 is event-driven — the FSM
# reads that fresh world model and advances its own state only when conditions
# are met. See the module docstring ("Two clocks, one loop").
# ---------------------------------------------------------------------------

while robot.step(timestep) != -1:
    step_count += 1

    # 1. Sense [sim-clock cadence] — drain the cue link and update the FCR with
    #    the turret's REAL aim, read from the pan/tilt position sensors. The FCR
    #    gates detection against this measured boresight, so it locks only once
    #    the barrel has physically slewed onto the target — not when the FSM
    #    merely commands the angle. getValue() can be NaN before the first step
    #    settles; fall back to 0.0 in that case.
    cue_link.update()
    ground_hit_link.update()
    bullet_hit_link.update()
    if ground_hit_link.hit_this_step():
        log.info(
            "[ATLAS] ground-hit cue received: hit #%d at t=%.2fs",
            ground_hit_link.count,
            robot.getTime(),
        )
    if bullet_hit_link.hit_this_step():
        log.info(
            "[ATLAS] bullet-hit cue received: hit #%d at t=%.2fs",
            bullet_hit_link.count,
            robot.getTime(),
        )

    # Launch-origin observation: on the resolution step (hit received) the gate
    # arms — cue_link still holds the stale impact/mid-flight position and
    # fsm.step() → RESET → cue_link.clear() has not run yet. On the next
    # (non-resolution) step the cue_link delivers the first FRESH position after
    # relaunch, which is close to the true launch origin. Cold-start (first
    # engagement) contributes no observation; the turret falls back to the fixed
    # idle beam (ADR-0014). Done before fsm.step() so IDLE pre-aims with the
    # freshest estimate.
    obs = launch_observation_gate.observation(
        resolved=ground_hit_link.hit_this_step() or bullet_hit_link.hit_this_step(),
        cue=cue_link.get_cue(),
    )
    if obs is not None:
        launch_point_estimator.observe(obs)
        log.info(
            "[ATLAS] launch observed at %s; estimate now %s (n=%d)",
            [round(c, 2) for c in obs],
            [round(v, 2) for v in launch_point_estimator.get_estimate()],
            launch_point_estimator.samples,
        )

    pan_actual = pan_sensor.getValue()
    tilt_actual = tilt_sensor.getValue()
    if math.isnan(pan_actual):
        pan_actual = 0.0
    if math.isnan(tilt_actual):
        tilt_actual = 0.0
    fcr.update(pan_actual, tilt_actual)

    # 2. Predict [sim-clock cadence] — propagate Kalman state forward one
    #    timestep. Runs every tick so the estimate stays warm even in states
    #    that don't read it (ADR-0003 continuous fusion).
    track_filter.predict()

    # 3. Fuse [sim-clock cadence] — update filter with FCR measurement when
    #    locked. The Search Radar is a separate process; it cues the FSM via the
    #    radio link but no longer feeds the TrackFilter directly.
    #    NOTE: the `is not None` guard is the one control decision left in the
    #    Sense layer ("when to fuse") — the FSM already knows lock state via
    #    fcr.is_locked(). A candidate to push into the FSM / TrackFilter later.
    fcr_pos = fcr.get_target_position()
    if fcr_pos is not None:
        track_filter.update_fcr(fcr_pos)

    # 4. Decide [event-driven state] — advance FSM one step
    fsm.step()

    # 4a. Launch — apply the launch velocity to a bullet armed last step. The
    #     muzzle teleport issued by fire() has now been applied by robot.step(),
    #     so setVelocity here is not wiped by a pending translation write
    #     (two-phase launch — see Bullet.fire/launch and ADR-0013).
    if bullet.is_launching:
        bullet.launch()

    # 4b. Fire — arm the recycled bullet when the FSM commits to a shot. One
    #     bullet at a time: a fire command while armed/in flight is ignored.
    fire_command = fsm.consume_fire_command()
    if fire_command is not None:
        if bullet.is_parked:
            bullet.fire(fire_command)
            log.info(
                "FIRE — bullet armed toward intercept [%.3f, %.3f, %.3f]",
                fire_command[0],
                fire_command[1],
                fire_command[2],
            )
        else:
            log.warning("FIRE ignored — a bullet is still in flight")

    # 4c. Recycle — park the bullet when the engagement ends: it struck the
    #     ball (bullet-hit cue), the ball landed (ground-hit cue, shot missed),
    #     or the flight-time safety timeout fired.
    bullet.step()
    if not bullet.is_parked and (
        bullet_hit_link.hit_this_step()
        or ground_hit_link.hit_this_step()
        or bullet.age_steps >= BULLET_MAX_LIFETIME_STEPS
    ):
        log.info("RECYCLE — parking bullet (age=%d steps)", bullet.age_steps)
        bullet.recycle()

    # 5. Report — per-step development telemetry (no effect on control)
    telemetry.report(
        step_count,
        robot.getTime(),
        fsm.state,
        track_filter,
        ballistic_predictor,
        projectile,
    )

