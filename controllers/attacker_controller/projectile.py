"""Attacker-owned model of an incoming projectile.

The attacker supervisor drives a passive Projectile Solid via the Webots
supervisor API. This module wraps that node and owns its spawn/despawn
lifecycle as a small two-state machine:

    ACTIVE     — in flight
    DESPAWNED  — parked out of sensor range after a hit, waiting to respawn

A single node is *recycled* rather than deleted: "despawn" teleports the ball
far out of every sensor's range and zeroes its motion; "spawn" teleports it
back to the spawn point and launches it. This keeps the DEF PROJECTILE handle
that search_radar_controller and atlas_controller hold valid, which true node
deletion would not (see ADR-0011, "recycle, not delete").

Hit detection — physical contacts, not a height heuristic
----------------------------------------------------------
Each step we ask Webots for the ball's real contact points (Supervisor
getContactPoints, world-frame). A contact is classified by its world-Z:

  * near the floor (z <= floor_contact_z_m)  → GROUND hit (attacker scores)
  * up in the air  (z >  floor_contact_z_m)  → BULLET hit (defender scores)

This is radius-independent and is how ground vs. turret-bullet hits are told
apart (Webots' contact node_id identifies the ball itself, not the other body,
so height is the discriminator — matching the referee design). The turret
bullet does not exist yet, so the bullet branch is dormant until on_bullet_hit
is wired up.

A ground contact only counts once the ball has *cleared the floor* (the
``_airborne`` latch), so the floor contact present at launch is not mistaken for
a landing.

Node-agnostic: ``Projectile`` takes anything exposing ``setVelocity`` /
``getContactPoints`` / ``getField`` (the Webots node interface), so it can be
unit tested against a stub without Webots.
"""

from dataclasses import dataclass, field
from enum import Enum, auto


@dataclass
class ProjectileConfig:
    """Tunable parameters for the incoming projectile's spawn cycle."""

    spawn_position: list = field(default_factory=lambda: [0, -5, 0.5])
    # Where the ball is parked while despawned: far below the floor, out of
    # every sensor's range/FOV so it is effectively gone until respawned.
    despawn_position: list = field(default_factory=lambda: [0.0, 0.0, -100.0])
    launch_velocity: list = field(default_factory=lambda: [0, 4, 6, 0, 0, 0])
    # A contact point at/below this world-Z is a floor contact (ground hit);
    # above it is a mid-air contact (bullet hit). Small positive margin above 0.
    floor_contact_z_m: float = 0.1
    respawn_delay_ms: float = 2000  # gap between a hit and the next spawn


class _State(Enum):
    ACTIVE = auto()
    DESPAWNED = auto()


class Projectile:
    """Lifecycle of a single recycled incoming projectile."""

    def __init__(
        self,
        node,
        config: ProjectileConfig | None = None,
        on_ground_hit=None,
        on_bullet_hit=None,
        select_launch=None,
    ):
        """
        Args:
            node:          Webots node handle for the projectile Solid.
            config:        Tunable parameters. Defaults to ProjectileConfig().
            on_ground_hit: Optional callable(count:int) fired when the ball lands
                           on the floor. Used by the attacker to log/score.
            on_bullet_hit: Optional callable(count:int) fired when the ball is
                           struck mid-air (by the turret bullet). Dormant until
                           the bullet exists.
            select_launch: Optional callable() -> (spawn_position, launch_velocity),
                           invoked at the top of spawn() to choose where the NEXT
                           ball launches from. None → the fixed config is reused
                           every spawn (original single-corridor behaviour). The
                           attacker uses this to launch from multiple sectors.
        """
        self.config = config if config is not None else ProjectileConfig()
        self._node = node
        self._translation = node.getField("translation")
        self._on_ground_hit = on_ground_hit
        self._on_bullet_hit = on_bullet_hit
        self._select_launch = select_launch
        self._state = _State.DESPAWNED
        self._despawn_time_ms = 0
        self._airborne = False  # has the ball cleared the floor since spawn?
        self.ground_hits = 0  # running count of registered ground hits
        self.bullet_hits = 0  # running count of registered bullet hits

    def spawn(self):
        """Place the ball at the spawn point and launch it (enter ACTIVE).

        If a select_launch callback was supplied, it chooses this launch's
        spawn_position/launch_velocity first (multi-sector attack). See the
        resetPhysics() note below.

        Deliberately does NOT call resetPhysics() here: in Webots, a
        resetPhysics() in the same timestep as setVelocity() zeroes the velocity
        we are trying to apply, so the ball never launches. Physics is reset in
        _despawn() instead (a different timestep), leaving the parked body clean
        before we relaunch it.
        """
        if self._select_launch is not None:
            spawn_position, launch_velocity = self._select_launch()
            self.config.spawn_position = list(spawn_position)
            self.config.launch_velocity = list(launch_velocity)
        self._translation.setSFVec3f(self.config.spawn_position)
        self._node.setVelocity(self.config.launch_velocity)
        self._airborne = False
        self._state = _State.ACTIVE

    def _despawn(self, now_ms):
        """Stop the ball, park it out of sensor range, enter DESPAWNED."""
        self._node.setVelocity([0, 0, 0, 0, 0, 0])
        self._translation.setSFVec3f(self.config.despawn_position)
        self._node.resetPhysics()
        self._state = _State.DESPAWNED
        self._despawn_time_ms = now_ms

    def _register_ground_hit(self):
        self.ground_hits += 1
        if self._on_ground_hit is not None:
            self._on_ground_hit(self.ground_hits)

    def _register_bullet_hit(self):
        self.bullet_hits += 1
        if self._on_bullet_hit is not None:
            self._on_bullet_hit(self.bullet_hits)

    def step(self, now_ms):
        """Advance one tick. Call once per timestep with the sim clock in ms."""
        if self._state is _State.ACTIVE:
            contacts = self._node.getContactPoints()
            floor_z = self.config.floor_contact_z_m
            floor_contact = any(c.point[2] <= floor_z for c in contacts)
            air_contact = any(c.point[2] > floor_z for c in contacts)

            if not floor_contact:
                # No floor contact this step: the ball has cleared the ground.
                self._airborne = True

            if air_contact:
                # Struck mid-air — the turret bullet, not the ground.
                self._register_bullet_hit()
                self._despawn(now_ms)
            elif floor_contact and self._airborne:
                # Came back down and touched the floor → landed.
                self._register_ground_hit()
                self._despawn(now_ms)
        elif self._state is _State.DESPAWNED:
            if now_ms - self._despawn_time_ms > self.config.respawn_delay_ms:
                self.spawn()
