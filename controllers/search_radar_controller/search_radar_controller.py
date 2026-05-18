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
  PROJECTILE   — the SimulatedProjectile node
  TURRET_BASE  — the AtlasTurret node (turret origin, world frame)
  SEARCH_RADAR — this robot's own node (base pose; phase-centre offset is
                 defined by SearchRadar.proto)
"""

import logging
import math
import struct

from controller import Supervisor

from search_radar import SearchRadar

# Search Radar sensor and visualisation parameters. Keep these constants as the
# single source for both the SearchRadar model and the visible debug beam.
SEARCH_RADAR_MAX_RANGE_M = 20.0
SEARCH_RADAR_VERTICAL_FOV_RAD = math.pi / 2
SEARCH_RADAR_BEAM_WIDTH_RAD = 0.35
SEARCH_RADAR_SCAN_RATE_RAD_PER_STEP = 0.15
SEARCH_RADAR_TRACK_TIMEOUT_STEPS = 20
SEARCH_RADAR_NOISE_STD_M = 0.2
SEARCH_RADAR_BEAM_CENTER_Z_M = 1.1
# SearchRadar.proto places the rotating endpoint Solid at z=1.0 and the radar
# head / FOV apex at local z=1.1 inside that endpoint, so the sensor phase
# centre is 2.1m above the Robot origin.
SEARCH_RADAR_PHASE_CENTER_OFFSET_M = [0.0, 0.0, 2.1]

# ---------------------------------------------------------------------------
# Telemetry logging
#
# Same structure as atlas_controller.py: Webots console via StreamHandler and a
# controller-local file trace for review after a run.
# ---------------------------------------------------------------------------

TELEMETRY_LEVEL = logging.DEBUG

logging.basicConfig(
    level=TELEMETRY_LEVEL,
    format="[%(levelname)s - %(name)s] %(message)s",
    handlers=[
        logging.StreamHandler(),
        logging.FileHandler("search_radar_telemetry.log", mode="w", encoding="utf-8"),
    ],
)
log = logging.getLogger("SearchRadarController")

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

motor.setPosition(float("inf"))
motor_velocity = SEARCH_RADAR_SCAN_RATE_RAD_PER_STEP / (timestep / 1000.0)
motor.setVelocity(motor_velocity)
log.info(
    "SEARCH_RADAR_MOTOR spinning at %.3f rad/s; SearchRadar gates on the "
    "measured joint angle (nominal scan_rate %.3f rad/step)",
    motor_velocity,
    SEARCH_RADAR_SCAN_RATE_RAD_PER_STEP,
)

angle_sensor = robot.getDevice("SEARCH_RADAR_ANGLE")
if angle_sensor is None:
    log.warning("SEARCH_RADAR_ANGLE not found - visual beam angle will not be logged")
else:
    angle_sensor.enable(timestep)

emitter = robot.getDevice("SR_CUE_EMITTER")
if emitter is None:
    log.error("SR_CUE_EMITTER not found - cues will not be emitted")
else:
    log.info("SR_CUE_EMITTER ready: channel=1 payload=struct.pack('ddd', x, y, z)")

# ---------------------------------------------------------------------------
# Scene nodes — confirmed DEF names from worlds/ATLA_v1.wbt
# ---------------------------------------------------------------------------

projectile_node = robot.getFromDef("PROJECTILE")
if projectile_node is None:
    raise RuntimeError("[SR] Could not find DEF PROJECTILE in the world file.")

turret_node = robot.getFromDef("TURRET_BASE")
if turret_node is None:
    raise RuntimeError("[SR] Could not find DEF TURRET_BASE in the world file.")

radar_node = robot.getSelf()

# Static snapshots — neither the turret nor the radar body translates.
turret_position = list(turret_node.getPosition())
radar_origin_position = list(radar_node.getPosition())
radar_position = [
    radar_origin_position[i] + SEARCH_RADAR_PHASE_CENTER_OFFSET_M[i]
    for i in range(3)
]

log.info("turret_position (world) = %s", [round(v, 3) for v in turret_position])
log.info("radar_origin_position (world) = %s", [round(v, 3) for v in radar_origin_position])
log.info("radar_phase_center    (world) = %s", [round(v, 3) for v in radar_position])

# ---------------------------------------------------------------------------
# Visible FOV beam
# ---------------------------------------------------------------------------

fov_coords_node = robot.getFromDef("SR_FOV_COORDS")
if fov_coords_node is None:
    log.warning("Search Radar FOV visual nodes not found; continuing without beam visual")
else:
    half_az = SEARCH_RADAR_BEAM_WIDTH_RAD / 2.0
    half_el = SEARCH_RADAR_VERTICAL_FOV_RAD / 2.0

    def _beam_corner(az: float, el: float) -> list[float]:
        return [
            math.sin(az) * math.cos(el) * SEARCH_RADAR_MAX_RANGE_M,
            math.cos(az) * math.cos(el) * SEARCH_RADAR_MAX_RANGE_M,
            SEARCH_RADAR_BEAM_CENTER_Z_M
            + math.sin(el) * SEARCH_RADAR_MAX_RANGE_M,
        ]

    fov_points = [
        [0.0, 0.0, SEARCH_RADAR_BEAM_CENTER_Z_M],
        _beam_corner(-half_az, half_el),
        _beam_corner(half_az, half_el),
        _beam_corner(half_az, -half_el),
        _beam_corner(-half_az, -half_el),
    ]
    point_field = fov_coords_node.getField("point")
    for index, point in enumerate(fov_points):
        point_field.setMFVec3f(index, point)

    log.info(
        "FOV visual configured from SearchRadar gates: max_range=%.2fm "
        "beam_width=%.3frad vertical_fov=%.3frad far_corners=%s",
        SEARCH_RADAR_MAX_RANGE_M,
        SEARCH_RADAR_BEAM_WIDTH_RAD,
        SEARCH_RADAR_VERTICAL_FOV_RAD,
        [[round(v, 3) for v in point] for point in fov_points[1:]],
    )

# ---------------------------------------------------------------------------
# SearchRadar — parameter values copied verbatim from atlas_controller.py
# (Increment 1 constructor call, lines 100-111).
# ---------------------------------------------------------------------------

search_radar = SearchRadar(
    [projectile_node],          # list of all projectile nodes in the scene
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
# Main loop
# ---------------------------------------------------------------------------

step_count = 0

while robot.step(timestep) != -1:
    step_count += 1

    # 1. Align the radar model to the visible Webots joint, then update.
    #    With an angle sensor present, scan_rate is zero so update() gates on
    #    this measured beam angle instead of an independent software phase.
    if angle_sensor is not None:
        search_radar._beam_azimuth = angle_sensor.getValue() % (2 * math.pi)
    search_radar.update()

    # 2. Select a target: first of get_detections(), matching old FSM logic.
    detections = search_radar.get_detections()
    if not detections:
        log.debug(
            "[SR NO CUE] step=%d t=%.2fs detections=0 emitter_present=%s "
            "model_beam_az=%.3f visual_beam_az=%s",
            step_count,
            robot.getTime(),
            emitter is not None,
            search_radar._beam_azimuth,
            "%.3f" % angle_sensor.getValue() if angle_sensor is not None else "n/a",
        )
        continue  # nothing in view yet — no cue to emit

    chosen = detections[0]
    _buffered_detection, track_age = search_radar._track_buffer[chosen.track_id]
    cue_source = "fresh_beam_hit" if track_age == 0 else "track_buffer"

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
            "turret_relative=(%.3f, %.3f, %.3f) world=(%.3f, %.3f, %.3f)",
            step_count,
            robot.getTime(),
            cue_source,
            track_age,
            chosen.track_id,
            search_radar._beam_azimuth,
            "%.3f" % angle_sensor.getValue() if angle_sensor is not None else "n/a",
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
