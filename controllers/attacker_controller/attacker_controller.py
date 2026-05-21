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

from controller import Supervisor

from atlas_logging import configure
from projectile import Projectile, ProjectileConfig

log = configure("AttackerController", log_file="attacker_telemetry.log")

robot = Supervisor()
timestep = int(robot.getBasicTimeStep())

projectile_node = robot.getFromDef("PROJECTILE")
if projectile_node is None:
    raise RuntimeError("[ATTACKER] Could not find DEF PROJECTILE in the world file.")

# Spawn parameters surfaced here at the controller level.
config = ProjectileConfig(
    spawn_position=[0, -3, 0.5],
    launch_velocity=[0, 4, 6, 0, 0, 0],
    floor_contact_z_m=0.1,
    respawn_delay_ms=2000,
)


def on_ground_hit(count):
    """Register a ground hit. For now just log/count; future: scoring hook."""
    log.info(
        "[ATTACKER] ground hit #%d registered at t=%.2fs — despawning ball; "
        "spawning a new one in %.0fms",
        count,
        robot.getTime(),
        config.respawn_delay_ms,
    )


def on_bullet_hit(count):
    """Register a turret-bullet hit (dormant until the bullet exists)."""
    log.info(
        "[ATTACKER] bullet hit #%d registered at t=%.2fs — despawning ball; "
        "spawning a new one in %.0fms",
        count,
        robot.getTime(),
        config.respawn_delay_ms,
    )


projectile = Projectile(
    projectile_node,
    config,
    on_ground_hit=on_ground_hit,
    on_bullet_hit=on_bullet_hit,
)

log.info("ATTACKER CONTROLLER STARTED — spawning first projectile")
projectile.spawn()

while robot.step(timestep) != -1:
    projectile.step(robot.getTime() * 1000)  # sim clock in ms
