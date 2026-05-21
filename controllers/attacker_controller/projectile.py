"""Attacker-owned model of an incoming projectile.

The attacker supervisor drives a passive Projectile Solid via the Webots
supervisor API. This module wraps that node and owns its spawn/despawn
lifecycle as a small two-state machine:

    ACTIVE     — in flight
    DESPAWNED  — parked out of sensor range after a ground hit, waiting to
                 respawn

A single node is *recycled* rather than deleted: "despawn" teleports the ball
far out of every sensor's range and zeroes its motion; "spawn" teleports it
back to the spawn point and launches it. This keeps the DEF PROJECTILE handle
that search_radar_controller and atlas_controller hold valid, which true node
deletion would not (see the attack-pattern design doc, "recycle, not delete").

Ground hits are *registered*: each one increments ``ground_hits`` and fires the
optional ``on_ground_hit`` callback, which is how the attacker logs/scores them.

Node-agnostic: ``Projectile`` takes anything exposing ``setVelocity`` /
``getPosition`` / ``getVelocity`` / ``resetPhysics`` / ``getField`` (the Webots
node interface), so it can be unit tested against a stub without Webots.
"""

from dataclasses import dataclass, field
from enum import Enum, auto


@dataclass
class ProjectileConfig:
    """Tunable parameters for the incoming projectile's spawn cycle."""

    spawn_position: list = field(
        default_factory=lambda: [2.59591e-05, -2.53, 0.04984303999999999]
    )
    # Where the ball is parked while despawned: far below the floor, out of
    # every sensor's range/FOV so it is effectively gone until respawned.
    despawn_position: list = field(default_factory=lambda: [0.0, 0.0, -100.0])
    launch_velocity: list = field(default_factory=lambda: [0, 4, 6, 0, 0, 0])
    ground_hit_threshold_m: float = 0.05  # world-Z below this → landed
    respawn_delay_ms: float = 2000  # gap between a ground hit and the next spawn


class _State(Enum):
    ACTIVE = auto()
    DESPAWNED = auto()


class Projectile:
    """Lifecycle of a single recycled incoming projectile."""

    def __init__(self, node, config: ProjectileConfig | None = None, on_ground_hit=None):
        """
        Args:
            node:          Webots node handle for the projectile Solid.
            config:        Tunable parameters. Defaults to ProjectileConfig().
            on_ground_hit: Optional callable(count:int) fired each time a ground
                           hit is registered. Used by the attacker to log/score.
        """
        self.config = config if config is not None else ProjectileConfig()
        self._node = node
        self._translation = node.getField("translation")
        self._on_ground_hit = on_ground_hit
        self._state = _State.DESPAWNED
        self._despawn_time_ms = 0
        self.ground_hits = 0  # running count of registered ground hits

    def spawn(self):
        """Place the ball at the spawn point and launch it (enter ACTIVE)."""
        self._translation.setSFVec3f(self.config.spawn_position)
        self._node.resetPhysics()
        self._node.setVelocity(self.config.launch_velocity)
        self._state = _State.ACTIVE

    def _despawn(self):
        """Stop the ball and park it out of sensor range (enter DESPAWNED)."""
        self._node.setVelocity([0, 0, 0, 0, 0, 0])
        self._translation.setSFVec3f(self.config.despawn_position)
        self._node.resetPhysics()
        self._state = _State.DESPAWNED

    def _register_ground_hit(self):
        """Record a ground hit and notify the attacker."""
        self.ground_hits += 1
        if self._on_ground_hit is not None:
            self._on_ground_hit(self.ground_hits)

    def step(self, now_ms):
        """Advance one tick. Call once per timestep with the sim clock in ms."""
        position = self._node.getPosition()
        velocity = self._node.getVelocity()

        if (
            self._state is _State.ACTIVE
            and position[2] < self.config.ground_hit_threshold_m
            and velocity[2] < 0
        ):
            # Ground hit: register it, then despawn the ball.
            self._register_ground_hit()
            self._despawn()
            self._despawn_time_ms = now_ms
        elif (
            self._state is _State.DESPAWNED
            and now_ms - self._despawn_time_ms > self.config.respawn_delay_ms
        ):
            # Respawn delay elapsed: spawn a fresh ball.
            self.spawn()
