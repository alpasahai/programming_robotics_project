"""Attacker Webots supervisor controller.

Offense side of the turret-defense game. Owns the incoming projectile: launches
it, then recycles it on each ground hit. Today this is a single recycled ball on
a fixed cadence; the attack-pattern schedule and ML launch log will be added
here later (see the attack-pattern design doc).

Thin glue layer — the lifecycle logic lives in the tested Projectile class.
The projectile itself is a passive Solid (DEF PROJECTILE); the attacker drives
it via the supervisor API and getFromDef, never via a node handle on itself.
"""

import logging

from controller import Supervisor

from projectile import Projectile, ProjectileConfig

logging.basicConfig(
    level=logging.INFO,
    format="[%(levelname)s - %(name)s] %(message)s",
)
log = logging.getLogger("AttackerController")

robot = Supervisor()
timestep = int(robot.getBasicTimeStep())

projectile_node = robot.getFromDef("PROJECTILE")
if projectile_node is None:
    raise RuntimeError("[ATTACKER] Could not find DEF PROJECTILE in the world file.")

# Launch parameters surfaced here at the controller level.
config = ProjectileConfig(
    spawn_position=[2.59591e-05, -2.53, 0.04984303999999999],
    launch_velocity=[0, 4, 6, 0, 0, 0],
    ground_hit_threshold_m=0.05,
    launch_delay_ms=2000,
)
projectile = Projectile(projectile_node, config)

log.info("ATTACKER CONTROLLER STARTED — launching first projectile")
projectile.launch()

while robot.step(timestep) != -1:

    projectile.step(robot.getTime() * 1000)  # sim clock in ms
