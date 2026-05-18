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
  SEARCH_RADAR — this robot's own node (used only for its position)
"""

import math
import struct

from controller import Supervisor

from search_radar import SearchRadar

# ---------------------------------------------------------------------------
# Bootstrap
# ---------------------------------------------------------------------------

robot = Supervisor()
timestep = int(robot.getBasicTimeStep())

print("[SR] SEARCH RADAR CONTROLLER STARTED")

# ---------------------------------------------------------------------------
# Devices
# ---------------------------------------------------------------------------

motor = robot.getDevice("SEARCH_RADAR_MOTOR")
if motor is None:
    print("[SR] ERROR: SEARCH_RADAR_MOTOR not found")
    while robot.step(timestep) != -1:
        pass

motor.setPosition(float("inf"))
motor.setVelocity(2.0)
print("[SR] SEARCH_RADAR_MOTOR spinning")

emitter = robot.getDevice("SR_CUE_EMITTER")
if emitter is None:
    print("[SR] ERROR: SR_CUE_EMITTER not found — cues will not be emitted")

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
radar_position = list(radar_node.getPosition())

print(f"[SR] turret_position (world) = {[round(v, 3) for v in turret_position]}")
print(f"[SR] radar_position  (world) = {[round(v, 3) for v in radar_position]}")

# ---------------------------------------------------------------------------
# SearchRadar — parameter values copied verbatim from atlas_controller.py
# (Increment 1 constructor call, lines 100-111).
# ---------------------------------------------------------------------------

search_radar = SearchRadar(
    [projectile_node],          # list of all projectile nodes in the scene
    turret_position,
    radar_position,
    noise_std=0.2,              # high noise — SearchRadar is coarse (metres std)
    timestep_ms=timestep,
    max_range=20.0,             # metres — starting value, tuned in Webots
    vertical_fov=math.pi / 2,  # 90 deg full vertical FOV — starting value
    beam_width=0.35,            # rad — rotating beam width, starting value
    scan_rate=0.15,             # rad per step — beam sweep speed, starting value
    track_timeout=20,           # steps a track persists across beam sweeps
)

# ---------------------------------------------------------------------------
# Main loop
# ---------------------------------------------------------------------------

while robot.step(timestep) != -1:
    # 1. Update the radar — advances beam, ages/refreshes track buffer.
    search_radar.update()

    # 2. Select a target: first of get_detections(), matching old FSM logic.
    detections = search_radar.get_detections()
    if not detections:
        continue  # nothing in view yet — no cue to emit

    chosen = detections[0]

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

    # DEBUG: print the cue so a human can verify plausible world coordinates.
    # Remove or guard with a verbosity flag once Webots integration is confirmed.
    print(
        f"[SR DEBUG] cue sent: track_id={chosen.track_id}"
        f"  world=({wx:.3f}, {wy:.3f}, {wz:.3f})"
    )
