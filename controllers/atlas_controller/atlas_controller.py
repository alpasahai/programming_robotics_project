"""ATLAS turret Webots supervisor controller.

Thin glue layer: constructs all components, wires them together, then runs
the per-timestep loop. No logic lives here — all decision-making is inside
the tested modules (FireControlRadar, SearchRadar, TrackFilter,
BallisticTrajectoryPredictor, AtlasFSM). Projectile launch/reset is the only
controller-level stateful behaviour (it is Webots-specific and untestable
outside the runtime).

Execution order each step (per ADR-0003 continuous fusion and the FSM plan):
  1. Sense   — search_radar.update(), fcr.update()
  2. Predict — track_filter.predict()
  3. Fuse    — track_filter.update_fcr() and/or update_search() when available
  4. Decide  — fsm.step()
  5. Manage  — projectile reset/relaunch independent of FSM
"""

import logging
from collections import deque

from controller import Supervisor

from fire_control_radar import FireControlRadar
from search_radar import SearchRadar
from track_filter import TrackFilter
from ballistic_trajectory_predictor import BallisticTrajectoryPredictor
from projectile_system import ProjectileSystem
from fsm import AtlasFSM, SensorSuite, TurretHardware

# ---------------------------------------------------------------------------
# Constants
# ---------------------------------------------------------------------------

GROUND_HIT_THRESHOLD_M = 0.05  # metres — projectile below this height counts as landed

# ---------------------------------------------------------------------------
# Telemetry logging
#
# logging works in Webots: a StreamHandler lands in the Webots console, a
# FileHandler writes a reviewable trace to controllers/atlas_controller/.
# Set TELEMETRY_LEVEL to logging.INFO for transitions-only, logging.DEBUG for
# full per-step telemetry.
# ---------------------------------------------------------------------------

TELEMETRY_LEVEL = logging.DEBUG

logging.basicConfig(
    level=TELEMETRY_LEVEL,
    format="[%(levelname)s - %(name)s] %(message)s",
    handlers=[
        logging.StreamHandler(),                                # Webots console
        logging.FileHandler("atlas_telemetry.log", mode="w", encoding="utf-8"),
    ],
)
log = logging.getLogger("Controller")


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

# --- Scene nodes ---
projectile = robot.getFromDef("PROJECTILE")
turret_position = robot.getSelf().getPosition()  # static snapshot — turret base never moves

# --- Sensors ---
# Tuning parameters: FCR is narrow-beam / precise; SearchRadar is wide-beam / coarse.
fcr = FireControlRadar(
    projectile,
    turret_position,
    noise_std=0.02,          # low noise — FCR is precise (metres std)
)
search_radar = SearchRadar(
    [projectile],            # list of all projectile nodes in the scene
    turret_position,
    noise_std=0.2,           # high noise — SearchRadar is coarse (metres std)
    timestep_ms=timestep,
)

# --- State estimator ---
# Tuning parameters chosen to match sensor noise characteristics.
# R_fcr=0.001 (trusts FCR heavily), R_search=0.1 (down-weights wide-beam),
# Q=0.01 (small process noise for near-ballistic motion).
track_filter = TrackFilter(
    timestep_ms=timestep,
    R_fcr=0.001,             # FCR measurement noise variance (m²) — low, trusted
    R_search=0.1,            # SearchRadar noise variance (m²) — high, coarse
    Q=0.01,                  # process noise scale — small for near-ballistic motion
)

# --- Ballistic predictor ---
ballistic_predictor = BallisticTrajectoryPredictor(track_filter, timestep_ms=timestep)

# --- FSM wiring ---
sensors = SensorSuite(
    search_radar=search_radar,
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

# --- Projectile system ---
projectile_system = ProjectileSystem(projectile)

# Projectile reset state (independent of FSM)
projectile_launched = False
waiting_for_launch = False
reset_time = 10  # ms
launch_delay = 2000  # ms — 2 second gap between shots

projectile_system.launch_projectile()
projectile_launched = True

log.info("=" * 78)
log.info("ATLAS telemetry legend")
log.info("  All positions below are TURRET-RELATIVE metres [dx, dy, dz] — the")
log.info("  offset from the turret, which sits at [0,0,0] in this frame.")
log.info("  filtered_pos  : Kalman filter's estimate of where the projectile is NOW")
log.info("  filtered_vel  : Kalman filter's estimate of projectile velocity (m/s)")
log.info("  intercept     : predicted aim point — where the projectile is expected")
log.info("                  to be %d timesteps ahead (the point PREDICT validates)",
         fsm.config.lookahead_steps)
log.info("  true_pos      : actual projectile position, un-noised ground truth")
log.info("                  (shown beside filtered_pos for comparison; FSM never sees it)")
log.info("  filter_error  : distance between filtered_pos and true_pos — how")
log.info("                  accurate the Kalman estimate is RIGHT NOW (lower = better)")
log.info("  pred_error    : distance between true_pos NOW and the intercept that was")
log.info("                  predicted %d steps ago FOR now — how accurate the",
         fsm.config.lookahead_steps)
log.info("                  prediction was (spikes at projectile relaunch — expected)")
log.info("  range_check   : intercept's distance from turret vs config.max_range")
log.info("  ground_check  : intercept's height (z) vs config.ground_threshold")
log.info("-" * 78)
log.info("turret_position (WORLD frame, fixed) = %s",
         [round(v, 3) for v in turret_position])
log.info("timestep = %d ms   lookahead_steps = %d   max_range = %.1f m   "
         "ground_threshold = %.2f m",
         timestep, fsm.config.lookahead_steps,
         fsm.config.max_range, fsm.config.ground_threshold)
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

    # 1. Sense — read both sensors
    search_radar.update()
    fcr.update()

    # 2. Predict — propagate Kalman state forward one timestep
    track_filter.predict()

    # 3. Fuse — update filter with whichever measurements are available.
    #    Both are None before their respective sensors have locked a target.
    fcr_pos = fcr.get_target_position()
    if fcr_pos is not None:
        track_filter.update_fcr(fcr_pos)

    search_pos = search_radar.get_target_position()
    if search_pos is not None:
        track_filter.update_search(search_pos)

    # 4. Decide — advance FSM one step
    fsm.step()

    # --- Telemetry — per-step view of what the FSM is working from ---
    if not track_filter.is_initialised():
        # The track filter has no estimate yet — happens before the first fuse
        # and for one step after ACQUIRE calls track_filter.reset().
        log.debug(
            "FSM=%s   step=%d   t=%.2fs\n"
            "    (track filter not initialised — no estimate this step)",
            fsm.state, step_count, robot.getTime(),
        )
    else:
        fpos = track_filter.get_position()
        fvel = track_filter.get_velocity()
        icept = ballistic_predictor.get_intercept(fsm.config.lookahead_steps)
        true_rel = [
            projectile.getPosition()[i] - turret_position[i] for i in range(3)
        ]

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
            fsm.state, step_count, robot.getTime(),
            str(fp), str(tp), filter_error,
            str(fv),
            str(ic), pred_error,
            dist, fsm.config.max_range,
            "IN-RANGE" if in_range else "OUT-OF-RANGE",
            icept[2], fsm.config.ground_threshold,
            "ABOVE-GROUND" if above_ground else "BELOW-GROUND",
        )

    # 5. Manage projectile — detect ground hit and relaunch after delay.
    #    This is independent of FSM state: the turret cycle and the projectile
    #    trajectory are decoupled so the FSM can run through RESET→SEARCH while
    #    waiting for the next shot.
    projectile_position = projectile.getPosition()
    projectile_velocity = projectile.getVelocity()
    current_time = robot.getTime() * 1000  # convert to ms

    if projectile_position[2] < GROUND_HIT_THRESHOLD_M and projectile_velocity[2] < 0 and projectile_launched:
        projectile_launched = False
        waiting_for_launch = True
        reset_time = current_time
        projectile_system.reset_projectile()

    if waiting_for_launch and (current_time - reset_time > launch_delay):
        projectile_system.launch_projectile()
        projectile_launched = True
        waiting_for_launch = False
