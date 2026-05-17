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
reset_time = 0  # ms
launch_delay = 2000  # ms — 2 second gap between shots

projectile_system.launch_projectile()
projectile_launched = True

# ---------------------------------------------------------------------------
# Main loop
# ---------------------------------------------------------------------------

while robot.step(timestep) != -1:

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
