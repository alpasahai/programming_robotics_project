"""Search Radar Webots controller — thin Supervisor orchestration.

Constructs the components, wires them, then runs a per-step loop where the
decision-making delegates to tested modules (mirrors atlas_controller.py):
  - beam_alignment.BeamAlignment aligns the detection beam to the rendered FOV
    node (or the joint-angle sensor) each step;
  - SearchRadar gates detections and buffers tracks;
  - cue_emitter.CueEmitter selects a fresh detection, converts it to world frame,
    and broadcasts struct.pack("ddd", x, y, z) on channel 1 via SR_CUE_EMITTER;
  - cue_telemetry.CueTelemetry owns all per-step logging.

The only hardware logic that remains inline is the bounded-sweep motor drive:
when the joint-angle sensor is present the motor ping-pongs between
SEARCH_RADAR_MIN_ANGLE_RAD and SEARCH_RADAR_MAX_ANGLE_RAD via
sweep_control.next_bounded_sweep_target; otherwise it spins continuously.

ADR-0006 (sensor membrane): only a bare world coordinate crosses the radio link
to ATLAS. Node handles and track_ids stay inside this process; the controller
talks to SearchRadar only through its public API (get_detections,
get_fresh_detections, set_beam_pose — applied via BeamAlignment).

The radar's mounting geometry lives solely in SearchRadar.proto (its mastHeight
field). The controller derives the phase centre by reading the rendered
SR_FOV_BEAM node rather than duplicating z offsets here.

DEF names (confirmed from worlds/ATLA_v1.wbt):
  PROJECTILE   — the Projectile node
  TURRET_BASE  — the AtlasTurret node (turret origin, world frame)
  SEARCH_RADAR — this robot's own node (base pose; phase-centre offset is
                 defined by SearchRadar.proto)

Execution order each step:
  0. Sweep  — advance the bounded-sweep motor target (only with angle feedback)
  1. Align  — beam_alignment.apply(search_radar)
  2. Sense  — search_radar.update()
  3. Emit   — cue_emitter.emit(fresh detections)
  4. Report — telemetry.report(...)  (development instrument; no control effect)
"""

import math

from controller import Supervisor

from atlas_logging import configure
from scene import DEF_PROJECTILE, DEF_TURRET_BASE
from search_radar import SearchRadar
from sweep_control import next_bounded_sweep_target
from beam_alignment import BeamAlignment, visual_beam_pose_to_search_frame
from cue_emitter import CueEmitter
from cue_telemetry import CueTelemetry

# Search Radar sensor and visualisation parameters. Keep these constants as the
# single source for both the SearchRadar model and the visible debug beam.
SEARCH_RADAR_MAX_RANGE_M = 20.0
SEARCH_RADAR_VERTICAL_FOV_RAD = math.pi / 2
SEARCH_RADAR_BEAM_WIDTH_RAD = 0.35
SEARCH_RADAR_SCAN_RATE_RAD_PER_STEP = 0.15
SEARCH_RADAR_TRACK_TIMEOUT_STEPS = 20
SEARCH_RADAR_NOISE_STD_M = 0.2
SEARCH_RADAR_MIN_ANGLE_RAD = 0.0
SEARCH_RADAR_MAX_ANGLE_RAD = math.pi
SEARCH_RADAR_SWEEP_TARGET_TOLERANCE_RAD = 0.02

# ---------------------------------------------------------------------------
# Telemetry logging — console level set centrally in lib/atlas_logging.py
# (LOG_LEVELS["SearchRadarController"]); override per run with
# SEARCHRADARCONTROLLER_LOG_LEVEL or ATLAS_LOG_LEVEL.
# ---------------------------------------------------------------------------
log = configure("SearchRadarController", log_file="search_radar_telemetry.log")
log.info("SEARCH RADAR CONTROLLER STARTED")

# ---------------------------------------------------------------------------
# Bootstrap
# ---------------------------------------------------------------------------
robot = Supervisor()
timestep = int(robot.getBasicTimeStep())

# ---------------------------------------------------------------------------
# Devices
# ---------------------------------------------------------------------------
motor = robot.getDevice("SEARCH_RADAR_MOTOR")
if motor is None:
    log.error("SEARCH_RADAR_MOTOR not found")
    while robot.step(timestep) != -1:
        pass

angle_sensor = robot.getDevice("SEARCH_RADAR_ANGLE")
if angle_sensor is None:
    log.warning("SEARCH_RADAR_ANGLE not found - visual beam angle will not be logged")
else:
    angle_sensor.enable(timestep)

motor_velocity = SEARCH_RADAR_SCAN_RATE_RAD_PER_STEP / (timestep / 1000.0)
motor.setVelocity(motor_velocity)

if angle_sensor is not None:
    scan_target = SEARCH_RADAR_MAX_ANGLE_RAD
    motor.setPosition(scan_target)
    log.info(
        "SEARCH_RADAR_MOTOR sweeping between %.3f and %.3f rad at %.3f rad/s; "
        "SearchRadar gates on the measured joint angle (nominal scan_rate %.3f rad/step)",
        SEARCH_RADAR_MIN_ANGLE_RAD,
        SEARCH_RADAR_MAX_ANGLE_RAD,
        motor_velocity,
        SEARCH_RADAR_SCAN_RATE_RAD_PER_STEP,
    )
else:
    scan_target = None
    motor.setPosition(float("inf"))
    log.info(
        "SEARCH_RADAR_MOTOR spinning at %.3f rad/s without angle feedback "
        "(nominal scan_rate %.3f rad/step)",
        motor_velocity,
        SEARCH_RADAR_SCAN_RATE_RAD_PER_STEP,
    )

emitter = robot.getDevice("SR_CUE_EMITTER")
if emitter is None:
    log.error("SR_CUE_EMITTER not found - cues will not be emitted")
else:
    log.info("SR_CUE_EMITTER ready: channel=1 payload=struct.pack('ddd', x, y, z)")

# ---------------------------------------------------------------------------
# Scene nodes — confirmed DEF names from worlds/ATLA_v1.wbt
# ---------------------------------------------------------------------------
projectile_node = robot.getFromDef(DEF_PROJECTILE)
if projectile_node is None:
    raise RuntimeError(f"[SR] Could not find DEF {DEF_PROJECTILE} in the world file.")

turret_node = robot.getFromDef(DEF_TURRET_BASE)
if turret_node is None:
    raise RuntimeError(f"[SR] Could not find DEF {DEF_TURRET_BASE} in the world file.")

radar_node = robot.getSelf()

# The rendered FOV beam node is the single source of truth for the radar's beam
# pose. Fetched once here and reused for seeding the phase centre, syncing the
# visible cone, and the per-step alignment in the main loop.
fov_beam_node = radar_node.getFromProtoDef("SR_FOV_BEAM")
fov_cone_node = radar_node.getFromProtoDef("SR_FOV_CONE")

# Static snapshots — neither the turret nor the radar body translates.
turret_position = list(turret_node.getPosition())
radar_origin_position = list(radar_node.getPosition())

log.info("turret_position (world) = %s", [round(v, 3) for v in turret_position])
log.info(
    "radar_origin_position (world) = %s", [round(v, 3) for v in radar_origin_position]
)

# Seed the phase centre (cone apex) by reading the rendered beam node — the same
# derivation BeamAlignment applies every step. This initial value is overwritten
# on the first loop iteration before any detection runs, so it only covers the
# pre-first-step window and the log below; the proto geometry, not a constant,
# determines it.
if fov_beam_node is not None:
    radar_position, _, _ = visual_beam_pose_to_search_frame(
        fov_beam_node.getOrientation(),
        fov_beam_node.getPosition(),
        SEARCH_RADAR_MAX_RANGE_M,
    )
else:
    radar_position = list(radar_origin_position)  # degraded: no beam visual present
log.info("radar_phase_center    (world) = %s", [round(v, 3) for v in radar_position])

# ---------------------------------------------------------------------------
# Components
# ---------------------------------------------------------------------------
search_radar = SearchRadar(
    [projectile_node],  # list index = track_id (ADR-0006)
    turret_position,
    radar_position,
    noise_std=SEARCH_RADAR_NOISE_STD_M,  # high noise — SearchRadar is coarse
    timestep_ms=timestep,
    max_range=SEARCH_RADAR_MAX_RANGE_M,
    vertical_fov=SEARCH_RADAR_VERTICAL_FOV_RAD,
    beam_width=SEARCH_RADAR_BEAM_WIDTH_RAD,
    scan_rate=0.0 if angle_sensor is not None else SEARCH_RADAR_SCAN_RATE_RAD_PER_STEP,
    track_timeout=SEARCH_RADAR_TRACK_TIMEOUT_STEPS,
)

# Visible FOV beam — drive the exposed PROTO parameters from the SearchRadar
# visual spec so the rendered cone matches the software detection gate. The
# beam's mounting height lives in the proto (mastHeight drives the mount Pose),
# so the cone offset's z stays at the spec value and we do not touch height here.
if fov_beam_node is None or fov_cone_node is None:
    log.warning(
        "Search Radar FOV visual nodes not found via getFromProtoDef; "
        "continuing without beam visual"
    )
else:
    beam_spec = search_radar.get_beam_visual_spec()
    radar_node.getField("fovConeOffset").setSFVec3f(
        [
            beam_spec.cone_center_local[0],
            beam_spec.cone_center_local[1],
            beam_spec.cone_center_local[2],
        ]
    )
    radar_node.getField("fovConeHeight").setSFFloat(beam_spec.cone_height)
    radar_node.getField("fovConeRadius").setSFFloat(beam_spec.cone_radius)
    log.info(
        "FOV visual cone configured from SearchRadar visual spec: "
        "origin_world=%s centre_azimuth=%.3frad range=%.2fm "
        "beam_width=%.3frad vertical_fov=%.3frad half_angle=%.3frad "
        "cone_height=%.3fm cone_radius=%.3fm local_center=%s",
        [round(v, 3) for v in beam_spec.origin_world],
        beam_spec.centre_azimuth,
        beam_spec.max_range,
        beam_spec.beam_width,
        beam_spec.vertical_fov,
        beam_spec.half_angle,
        beam_spec.cone_height,
        beam_spec.cone_radius,
        [round(v, 3) for v in beam_spec.cone_center_local],
    )

beam_alignment = BeamAlignment(fov_beam_node, angle_sensor, SEARCH_RADAR_MAX_RANGE_M)
cue_emitter = CueEmitter(emitter)
telemetry = CueTelemetry(log)

# ---------------------------------------------------------------------------
# Main loop
# ---------------------------------------------------------------------------
step_count = 0

while robot.step(timestep) != -1:
    step_count += 1

    # 0. Bounded sweep — flip the motor target at the sweep endpoints (only when
    #    angle feedback is present; otherwise the motor spins continuously).
    if angle_sensor is not None and scan_target is not None:
        next_scan_target = next_bounded_sweep_target(
            joint_angle=angle_sensor.getValue(),
            current_target=scan_target,
            min_angle=SEARCH_RADAR_MIN_ANGLE_RAD,
            max_angle=SEARCH_RADAR_MAX_ANGLE_RAD,
            tolerance=SEARCH_RADAR_SWEEP_TARGET_TOLERANCE_RAD,
        )
        if next_scan_target != scan_target:
            scan_target = next_scan_target
            motor.setPosition(scan_target)

    # 1. Align the detection beam to the rendered FOV node (or joint angle),
    #    making the proto visual the source of truth for beam direction and
    #    phase-centre position.
    beam = beam_alignment.apply(search_radar)

    # 2. Sense — gate projectiles and refresh the track buffer.
    search_radar.update()

    # 3. Emit — select the first fresh detection, convert to world, broadcast.
    detections = search_radar.get_detections()
    fresh_detections = search_radar.get_fresh_detections()
    cue = cue_emitter.emit(fresh_detections, turret_position)

    # 4. Report — per-step development telemetry (no effect on the cue itself).
    telemetry.report(
        step_count, robot.getTime(), beam, detections, fresh_detections, cue
    )
