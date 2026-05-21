"""ATLAS turret Webots supervisor controller.

Thin glue layer: constructs all components, wires them together, then runs
the per-timestep loop. No logic lives here — all decision-making is inside
the tested modules (FireControlRadar, SearchRadarLink, TrackFilter,
BallisticTrajectoryPredictor, AtlasFSM). The incoming projectile's
launch/relaunch lifecycle is owned by the separate attacker_controller; this
controller only reads the projectile's position as ground-truth telemetry.

Execution order each step (per ADR-0003 continuous fusion and the FSM plan):
  1. Sense   — cue_link.update(), fcr.update(turret_aim)
  2. Predict — track_filter.predict()
  3. Fuse    — track_filter.update_fcr() when FCR is locked
  4. Decide  — fsm.step()
  5. Report  — telemetry.report() (development instrument; no effect on control)
"""

from controller import Supervisor

from atlas_logging import configure
from fire_control_radar import FireControlRadar
from search_radar_link import SearchRadarLink
from track_filter import TrackFilter
from ballistic_trajectory_predictor import BallisticTrajectoryPredictor
from fsm import AtlasFSM, SensorSuite, TurretHardware
from telemetry import TrackTelemetry
from scene import DEF_PROJECTILE

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

# ---------------------------------------------------------------------------
# Setup
# ---------------------------------------------------------------------------

robot = Supervisor()
timestep = int(robot.getBasicTimeStep())

# --- Devices ---
pan = robot.getDevice("PAN_MOTOR")
tilt = robot.getDevice("TILT_MOTOR")

# Receiver for Search Radar cues delivered over the inter-process radio link.
# Must be enabled before constructing SearchRadarLink.
receiver = robot.getDevice("FCR_CUE_RECEIVER")
receiver.enable(timestep)

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
# ---------------------------------------------------------------------------

while robot.step(timestep) != -1:
    step_count += 1

    # 1. Sense — drain the cue link and update the FCR with the turret's
    #    current aim. The FCR gates detection against the boresight, so it
    #    must be fed the aim angles the FSM last commanded.
    cue_link.update()
    fcr.update(*fsm.commanded_aim)

    # 2. Predict — propagate Kalman state forward one timestep
    track_filter.predict()

    # 3. Fuse — update filter with FCR measurement when locked.
    #    The Search Radar is a separate process; it cues the FSM via the
    #    radio link but no longer feeds the TrackFilter directly.
    fcr_pos = fcr.get_target_position()
    if fcr_pos is not None:
        track_filter.update_fcr(fcr_pos)

    # 4. Decide — advance FSM one step
    fsm.step()

    # 5. Report — per-step development telemetry (no effect on control)
    telemetry.report(
        step_count,
        robot.getTime(),
        fsm.state,
        track_filter,
        ballistic_predictor,
        projectile,
    )

