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
"""

import math
from collections import deque

from controller import Supervisor

from atlas_logging import configure
from fire_control_radar import FireControlRadar
from search_radar_link import SearchRadarLink
from track_filter import TrackFilter
from ballistic_trajectory_predictor import BallisticTrajectoryPredictor
from fsm import AtlasFSM, SensorSuite, TurretHardware

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


def _distance(a, b):
    """Euclidean distance between two 3-vectors."""
    return ((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2 + (a[2] - b[2]) ** 2) ** 0.5


_ORIGIN = [0.0, 0.0, 0.0]

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
projectile = robot.getFromDef("PROJECTILE")
if projectile is None:
    raise RuntimeError("Could not find DEF PROJECTILE in the world file.")

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
    fov_half_angle=0.1,             # ~5.7° half-angle — narrow FCR beam
    max_range=20.0,                 # metres — tune in Webots if needed
    noise_std=0.02,                 # low noise — FCR is precise (metres std)
)
cue_link = SearchRadarLink(receiver)

# --- State estimator ---
# R_fcr=0.001 (trusts FCR heavily); R_search retained as a constructor
# parameter (TrackFilter still accepts it), but update_search() is gone —
# only update_fcr() is called. Q=0.01 (small process noise, near-ballistic).
track_filter = TrackFilter(
    timestep_ms=timestep,
    R_fcr=0.001,  # FCR measurement noise variance (m²) — low, trusted
    R_search=0.1,  # retained for TrackFilter constructor; update_search removed
    Q=0.01,  # process noise scale — small for near-ballistic motion
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

log.info("=" * 78)
log.info("ATLAS telemetry legend")
log.info("  All positions below are TURRET-RELATIVE metres [dx, dy, dz] — the")
log.info("  offset from the turret, which sits at [0,0,0] in this frame.")
log.info("  filtered_pos  : Kalman filter's estimate of where the projectile is NOW")
log.info("  filtered_vel  : Kalman filter's estimate of projectile velocity (m/s)")
log.info("  intercept     : predicted aim point — where the projectile is expected")
log.info(
    "                  to be %d timesteps ahead (the point PREDICT validates)",
    fsm.config.lookahead_steps,
)
log.info("  true_pos      : actual projectile position, un-noised ground truth")
log.info(
    "                  (shown beside filtered_pos for comparison; FSM never sees it)"
)
log.info("  filter_error  : distance between filtered_pos and true_pos — how")
log.info("                  accurate the Kalman estimate is RIGHT NOW (lower = better)")
log.info("  pred_error    : distance between true_pos NOW and the intercept that was")
log.info(
    "                  predicted %d steps ago FOR now — how accurate the",
    fsm.config.lookahead_steps,
)
log.info("                  prediction was (spikes at projectile relaunch — expected)")
log.info("  range_check   : intercept's distance from turret vs config.max_range")
log.info("  ground_check  : intercept's height (z) vs config.ground_threshold")
log.info("-" * 78)
log.info(
    "turret_position (WORLD frame, fixed) = %s", [round(v, 3) for v in turret_position]
)
log.info(
    "timestep = %d ms   lookahead_steps = %d   max_range = %.1f m   "
    "ground_threshold = %.2f m",
    timestep,
    fsm.config.lookahead_steps,
    fsm.config.max_range,
    fsm.config.ground_threshold,
)
log.info("=" * 78)

# Past intercepts, kept so each step can compare the prediction made
# lookahead_steps ago against where the projectile actually ended up.
# When full, pred_history[0] is the prediction that targeted the current step.
pred_history = deque(maxlen=fsm.config.lookahead_steps + 1)

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

    # --- Telemetry — per-step view of what the FSM is working from ---
    if not track_filter.is_initialised():
        # The track filter has no estimate yet — happens before the first fuse
        # and for one step after ACQUIRE calls track_filter.reset().
        log.debug(
            "FSM=%s   step=%d   t=%.2fs\n"
            "    (track filter not initialised — no estimate this step)",
            fsm.state,
            step_count,
            robot.getTime(),
        )
    else:
        fpos = track_filter.get_position()
        fvel = track_filter.get_velocity()
        icept = ballistic_predictor.get_intercept(fsm.config.lookahead_steps)
        true_rel = [projectile.getPosition()[i] - turret_position[i] for i in range(3)]

        # Filter accuracy now: filtered estimate vs ground truth.
        filter_error = _distance(fpos, true_rel)

        # Prediction accuracy: the intercept predicted lookahead_steps ago,
        # which targeted the current step, vs where the projectile actually is.
        pred_history.append(icept)
        if len(pred_history) == pred_history.maxlen:
            pred_error = "%.3f m" % _distance(true_rel, pred_history[0])
        else:
            pred_error = "n/a (warming up)"

        dist = _distance(icept, _ORIGIN)
        in_range = dist <= fsm.config.max_range
        above_ground = icept[2] > fsm.config.ground_threshold

        fp = [round(v, 2) for v in fpos]
        tp = [round(v, 2) for v in true_rel]
        fv = [round(v, 2) for v in fvel]
        ic = [round(v, 2) for v in icept]
        log.debug(
            "FSM=%s   step=%d   t=%.2fs\n"
            "    filtered_pos = %-22s true_pos = %-22s filter_error = %.3f m\n"
            "    filtered_vel = %s\n"
            "    intercept    = %-22s pred_error = %s\n"
            "    range_check  : %.2f m vs max %.1f m   -> %s\n"
            "    ground_check : intercept height %.2f m vs min %.2f m   -> %s",
            fsm.state,
            step_count,
            robot.getTime(),
            str(fp),
            str(tp),
            filter_error,
            str(fv),
            str(ic),
            pred_error,
            dist,
            fsm.config.max_range,
            "IN-RANGE" if in_range else "OUT-OF-RANGE",
            icept[2],
            fsm.config.ground_threshold,
            "ABOVE-GROUND" if above_ground else "BELOW-GROUND",
        )

