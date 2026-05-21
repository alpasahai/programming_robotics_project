"""Search Radar Webots controller.

Thin Supervisor loop: reads the projectile ground-truth position, drives the
radar's spin motor, runs SearchRadar.update() each step, selects the first
live track, converts the turret-relative Detection position back to world frame,
and broadcasts a 24-byte struct.pack("ddd", x, y, z) payload on channel 1 via
the SR_CUE_EMITTER radio device.

World-frame conversion choice
------------------------------
Detection.position is turret-relative [dx, dy, dz].  The simplest correct path
to a world-frame cue is:

    world_pos[i] = detection.position[i] + turret_position[i]

where turret_position is a one-time Supervisor snapshot of DEF TURRET_BASE.
This avoids any dependency on projectile-node handles inside the main loop and
keeps the SearchRadar membrane intact (ADR-0006): only the cue coordinate
crosses the boundary, never a node handle.

DEF names (confirmed from worlds/ATLA_v1.wbt)
----------------------------------------------
  PROJECTILE   — the Projectile node
  TURRET_BASE  — the AtlasTurret node (turret origin, world frame)
  SEARCH_RADAR — this robot's own node (base pose; phase-centre offset is
                 defined by SearchRadar.proto)
"""

import math
import struct

from controller import Supervisor

from atlas_logging import configure
from scene import DEF_PROJECTILE, DEF_TURRET_BASE
from search_radar import SearchRadar
from sweep_control import next_bounded_sweep_target

# Search Radar sensor and visualisation parameters. Keep these constants as the
# single source for both the SearchRadar model and the visible debug beam.
SEARCH_RADAR_MAX_RANGE_M = 20.0
SEARCH_RADAR_VERTICAL_FOV_RAD = math.pi / 2
SEARCH_RADAR_BEAM_WIDTH_RAD = 0.35
SEARCH_RADAR_SCAN_RATE_RAD_PER_STEP = 0.32
SEARCH_RADAR_TRACK_TIMEOUT_STEPS = 20
SEARCH_RADAR_NOISE_STD_M = 0.2
SEARCH_RADAR_MIN_ANGLE_RAD = 0.0
SEARCH_RADAR_MAX_ANGLE_RAD = math.pi
SEARCH_RADAR_SWEEP_TARGET_TOLERANCE_RAD = 0.02
# The radar's mounting geometry (pole / endpoint / beam height) lives solely in
# SearchRadar.proto. The controller derives the phase centre by reading the
# rendered SR_FOV_BEAM node instead of duplicating those z offsets here — change
# the beam height in the proto (fovBeamTranslation z) and the controller follows.

# ---------------------------------------------------------------------------
# Telemetry logging
#
# Webots console via StreamHandler and a controller-local file trace for review
# after a run. The console level is set centrally in lib/atlas_logging.py
# (LOG_LEVELS["SearchRadarController"]); override per run with
# SEARCHRADARCONTROLLER_LOG_LEVEL or ATLAS_LOG_LEVEL.
# ---------------------------------------------------------------------------

log = configure("SearchRadarController", log_file="search_radar_telemetry.log")

# ---------------------------------------------------------------------------
# Bootstrap
# ---------------------------------------------------------------------------

robot = Supervisor()
timestep = int(robot.getBasicTimeStep())

log.info("SEARCH RADAR CONTROLLER STARTED")

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


def _joint_angle_to_search_azimuth(joint_angle: float) -> float:
    """Convert Webots +Z joint angle to SearchRadar azimuth convention.

    SearchRadar azimuth uses x = sin(az), y = cos(az), so positive azimuth
    turns local +Y toward +X. Webots positive rotation about +Z turns local +Y
    toward -X. The physical beam azimuth is therefore the negative joint angle.
    """
    return (-joint_angle) % (2 * math.pi)


def _visual_beam_pose_to_search_frame(beam_node, beam_range: float):
    """Derive SearchRadar origin and azimuth from the rendered FOV node.

    Webots returns a node orientation matrix in row-major order. The second
    column is the node's local +Y axis expressed in world coordinates, which
    is the direction the SR_FOV_BOX extends in SearchRadar.proto.
    """
    orientation = beam_node.getOrientation()
    direction = [orientation[1], orientation[4], orientation[7]]
    center = beam_node.getPosition()
    origin = [center[i] - direction[i] * (beam_range / 2.0) for i in range(3)]
    azimuth = math.atan2(direction[0], direction[1]) % (2 * math.pi)
    return origin, azimuth, direction


# Seed the phase centre (cone apex) by reading the rendered beam node — the same
# derivation the main loop applies every step. This initial value is overwritten
# on the first loop iteration before any detection runs, so it only covers the
# pre-first-step window and the log below; the proto geometry, not a constant,
# determines it.
if fov_beam_node is not None:
    radar_position, _, _ = _visual_beam_pose_to_search_frame(
        fov_beam_node, SEARCH_RADAR_MAX_RANGE_M
    )
else:
    radar_position = list(radar_origin_position)  # degraded: no beam visual present
log.info("radar_phase_center    (world) = %s", [round(v, 3) for v in radar_position])


# ---------------------------------------------------------------------------
# SearchRadar — parameter values copied verbatim from atlas_controller.py
# (Increment 1 constructor call, lines 100-111).
# ---------------------------------------------------------------------------

search_radar = SearchRadar(
    [projectile_node],  # list of all projectile nodes in the scene
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

# ---------------------------------------------------------------------------
# Visible FOV beam
# ---------------------------------------------------------------------------


if fov_beam_node is None or fov_cone_node is None:
    log.warning(
        "Search Radar FOV visual nodes not found via getFromProtoDef; "
        "continuing without beam visual"
    )
else:
    beam_spec = search_radar.get_beam_visual_spec()
    # Set only the spec-derived cone centre (x/y) and dimensions on the exposed
    # PROTO params. Preserve the proto's mount height (z) — the beam height lives
    # in SearchRadar.proto's fovBeamTranslation, not here, so changing it there
    # is honoured rather than overwritten.
    mount_z = radar_node.getField("fovBeamTranslation").getSFVec3f()[2]
    radar_node.getField("fovBeamTranslation").setSFVec3f(
        [
            beam_spec.cone_center_local[0],
            beam_spec.cone_center_local[1],
            mount_z + beam_spec.cone_center_local[2],
        ]
    )
    radar_node.getField("fovConeHeight").setSFFloat(beam_spec.cone_height)
    radar_node.getField("fovConeRadius").setSFFloat(beam_spec.cone_radius)

    log.info(
        "FOV visual cone configured from SearchRadar visual spec: "
        "origin_world=%s centre_azimuth=%.3frad range=%.2fm "
        "beam_width=%.3frad vertical_fov=%.3frad half_angle=%.3frad "
        "cone_height=%.3fm cone_radius=%.3fm "
        "local_center=%s mount_z=%.3fm",
        [round(v, 3) for v in beam_spec.origin_world],
        beam_spec.centre_azimuth,
        beam_spec.max_range,
        beam_spec.beam_width,
        beam_spec.vertical_fov,
        beam_spec.half_angle,
        beam_spec.cone_height,
        beam_spec.cone_radius,
        [round(v, 3) for v in beam_spec.cone_center_local],
        mount_z,
    )

# ---------------------------------------------------------------------------
# Main loop
# ---------------------------------------------------------------------------

step_count = 0

while robot.step(timestep) != -1:
    step_count += 1

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

    # 1. Align the radar model to the actual rendered FOV node, then update.
    #    This makes the proto visual the source of truth for beam direction and
    #    phase-centre position, avoiding joint sign/frame assumptions.
    visual_origin = None
    visual_direction = None
    if fov_beam_node is not None:
        visual_origin, visual_azimuth, visual_direction = (
            _visual_beam_pose_to_search_frame(fov_beam_node, search_radar._max_range)
        )
        search_radar._radar_position = visual_origin
        search_radar._beam_azimuth = visual_azimuth
    elif angle_sensor is not None:
        search_radar._beam_azimuth = _joint_angle_to_search_azimuth(
            angle_sensor.getValue()
        )
    search_radar.update()

    # 2. Select a target: first of get_detections(), matching old FSM logic.
    detections = search_radar.get_detections()
    if not detections:
        log.debug(
            "[SR NO CUE] step=%d t=%.2fs detections=0 emitter_present=%s "
            "model_beam_az=%.3f visual_beam_az=%s "
            "visual_origin=%s visual_dir=%s",
            step_count,
            robot.getTime(),
            emitter is not None,
            search_radar._beam_azimuth,
            (
                "%.3f" % search_radar._beam_azimuth
                if fov_beam_node is not None or angle_sensor is not None
                else "n/a"
            ),
            (
                "(%.3f, %.3f, %.3f)" % tuple(visual_origin)
                if visual_origin is not None
                else "n/a"
            ),
            (
                "(%.3f, %.3f, %.3f)" % tuple(visual_direction)
                if visual_direction is not None
                else "n/a"
            ),
        )
        continue  # nothing in view yet — no cue to emit

    fresh_detections = [
        detection
        for detection in detections
        if search_radar._track_buffer[detection.track_id][1] == 0
    ]
    if not fresh_detections:
        buffered_track_ids = [detection.track_id for detection in detections]
        buffered_track_ages = [
            search_radar._track_buffer[detection.track_id][1]
            for detection in detections
        ]
        log.debug(
            "[SR BUFFER HELD] step=%d t=%.2fs buffered_track_ids=%s "
            "track_ages=%s emitter_present=%s model_beam_az=%.3f "
            "visual_beam_az=%s visual_origin=%s visual_dir=%s",
            step_count,
            robot.getTime(),
            buffered_track_ids,
            buffered_track_ages,
            emitter is not None,
            search_radar._beam_azimuth,
            (
                "%.3f" % search_radar._beam_azimuth
                if fov_beam_node is not None or angle_sensor is not None
                else "n/a"
            ),
            (
                "(%.3f, %.3f, %.3f)" % tuple(visual_origin)
                if visual_origin is not None
                else "n/a"
            ),
            (
                "(%.3f, %.3f, %.3f)" % tuple(visual_direction)
                if visual_direction is not None
                else "n/a"
            ),
        )
        continue

    chosen = fresh_detections[0]
    _buffered_detection, track_age = search_radar._track_buffer[chosen.track_id]
    cue_source = "fresh_beam_hit"

    # 3. Convert turret-relative Detection.position → world frame.
    #    Detection.position = [dx, dy, dz] relative to turret_position.
    #    world_pos[i] = dx[i] + turret_position[i]
    #    (See module docstring for rationale.)
    wx = chosen.position[0] + turret_position[0]
    wy = chosen.position[1] + turret_position[1]
    wz = chosen.position[2] + turret_position[2]

    # 4. Emit the world-frame cue over the radio link.
    if emitter is not None:
        emitter.send(struct.pack("ddd", wx, wy, wz))
        log.info(
            "[SR CUE SENT] step=%d t=%.2fs source=%s track_age=%d "
            "sender=search_radar_controller device=SR_CUE_EMITTER channel=1 "
            "track_id=%s model_beam_az=%.3f visual_beam_az=%s "
            "visual_origin=%s visual_dir=%s "
            "turret_relative=(%.3f, %.3f, %.3f) world=(%.3f, %.3f, %.3f)",
            step_count,
            robot.getTime(),
            cue_source,
            track_age,
            chosen.track_id,
            search_radar._beam_azimuth,
            (
                "%.3f" % search_radar._beam_azimuth
                if fov_beam_node is not None or angle_sensor is not None
                else "n/a"
            ),
            (
                "(%.3f, %.3f, %.3f)" % tuple(visual_origin)
                if visual_origin is not None
                else "n/a"
            ),
            (
                "(%.3f, %.3f, %.3f)" % tuple(visual_direction)
                if visual_direction is not None
                else "n/a"
            ),
            chosen.position[0],
            chosen.position[1],
            chosen.position[2],
            wx,
            wy,
            wz,
        )
    else:
        log.warning(
            "[SR CUE DROPPED] step=%d t=%.2fs sender=search_radar_controller "
            "reason=missing_emitter source=%s track_age=%d track_id=%s "
            "world=(%.3f, %.3f, %.3f)",
            step_count,
            robot.getTime(),
            cue_source,
            track_age,
            chosen.track_id,
            wx,
            wy,
            wz,
        )
