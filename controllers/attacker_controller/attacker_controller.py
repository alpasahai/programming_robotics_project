"""Attacker Webots supervisor controller.

Offense side of the turret-defense game. Owns the incoming projectile: spawns
it, registers ground hits, despawns the ball on each hit, and spawns a fresh one
after a short delay. Today this is a single recycled ball on a fixed cadence;
the attack-pattern schedule and ML launch log will be added here later (see the
attack-pattern design doc).

Ground-hit registration: the Projectile fires the on_ground_hit callback below
each time the ball lands. Right now that just logs and counts; it is the seam
where scoring / the referee will later hook in.

Thin glue layer — the lifecycle logic lives in the tested Projectile class.
The projectile itself is a passive Solid (DEF PROJECTILE); the attacker drives
it via the supervisor API and getFromDef, never via a node handle on itself.
"""

import math
import struct

import numpy as np
from controller import Supervisor

from atlas_logging import configure
from attack_pattern import PatternRule, launch_sequence
from projectile import Projectile, ProjectileConfig
from scene import DEF_PROJECTILE
from sectors import sector_launch

log = configure("AttackerController", log_file="attacker_telemetry.log")

robot = Supervisor()
timestep = int(robot.getBasicTimeStep())

emitter = robot.getDevice("ATTACKER_EMITTER")
if emitter is None:
    raise RuntimeError("[ATTACKER] Could not find device ATTACKER_EMITTER.")

bullet_emitter = robot.getDevice("ATTACKER_BULLET_EMITTER")
if bullet_emitter is None:
    raise RuntimeError("[ATTACKER] Could not find device ATTACKER_BULLET_EMITTER.")

projectile_node = robot.getFromDef(DEF_PROJECTILE)
if projectile_node is None:
    raise RuntimeError(
        f"[ATTACKER] Could not find DEF {DEF_PROJECTILE} in the world file."
    )

# Spawn parameters surfaced here at the controller level.
config = ProjectileConfig(
    spawn_position=[0, -10, 0.5],
    launch_velocity=[0, 2, 10, 0, 0, 0],
    floor_contact_z_m=0.1,
    respawn_delay_ms=2000,
)

# --- Attack plan: launch from several sectors on a structured/noisy/drifting
#     pattern. The sectors and pattern are the attacker's ground truth and are
#     NEVER sent to ATLAS — it perceives launches from radar and discovers the
#     sectors itself. See the attack-plan spec.
SECTOR_AZIMUTHS_RAD = [math.pi, math.pi * 0.6, math.pi * 1.4]  # 3 well-separated bearings
SECTOR_GROUND_RANGE_M = 10.0
SECTOR_HEIGHT_M = 0.5
SECTOR_H_SPEED = 2.0
SECTOR_V_SPEED = 10.0

_sector_launches = [
    sector_launch(az, SECTOR_GROUND_RANGE_M, SECTOR_HEIGHT_M, SECTOR_H_SPEED, SECTOR_V_SPEED)
    for az in SECTOR_AZIMUTHS_RAD
]

# Phase A then a drift to phase B (different tour + burst lengths) so ATLAS must
# re-learn mid-run. Seeded for reproducible demos.
_pattern_rng = np.random.default_rng(20260522)
_phase_a = PatternRule(tour=[0, 1, 2], mean_burst=[5, 2, 4], deviation_prob=0.1)
_phase_b = PatternRule(tour=[2, 0, 1], mean_burst=[3, 5, 2], deviation_prob=0.1)
_pattern = launch_sequence([(_phase_a, 40), (_phase_b, None)], _pattern_rng)


def _next_launch():
    """select_launch hook: advance the pattern and return that sector's geometry."""
    sector = next(_pattern)
    log.info("[ATTACKER] launching from sector %d at t=%.2fs", sector, robot.getTime())
    return _sector_launches[sector]


def on_ground_hit(count):
    """Register a ground hit. For now just log/count; future: scoring hook."""
    log.info(
        "[ATTACKER] ground hit #%d registered at t=%.2fs — despawning ball; "
        "spawning a new one in %.0fms",
        count,
        robot.getTime(),
        config.respawn_delay_ms,
    )
    # Emit the authoritative ground-hit pulse (channel 2, one 4-byte int =
    # running ground-hit count). This is ATLAS's only source of ground-hit
    # truth — see docs/superpowers/specs/2026-05-21-attacker-ground-hit-cue-design.md.
    emitter.send(struct.pack("i", count))


def on_bullet_hit(count):
    """Register a turret-bullet hit (emits on channel 3)."""
    log.info(
        "[ATTACKER] bullet hit #%d registered at t=%.2fs — despawning ball; "
        "spawning a new one in %.0fms",
        count,
        robot.getTime(),
        config.respawn_delay_ms,
    )
    # Emit the authoritative bullet-hit pulse (channel 3, one 4-byte int =
    # running bullet-hit count). This is ATLAS's only source of
    # projectile-destroyed truth — see docs/superpowers/specs/2026-05-22-engage-fire-bullet-design.md and #32.
    bullet_emitter.send(struct.pack("i", count))


projectile = Projectile(
    projectile_node,
    config,
    on_ground_hit=on_ground_hit,
    on_bullet_hit=on_bullet_hit,
    select_launch=_next_launch,
)

log.info("ATTACKER CONTROLLER STARTED — spawning first projectile")
projectile.spawn()

while robot.step(timestep) != -1:
    projectile.step(robot.getTime() * 1000)  # sim clock in ms
