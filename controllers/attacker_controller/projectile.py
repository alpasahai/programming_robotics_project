"""Attacker-owned model of an incoming projectile.

The attacker supervisor drives a passive Projectile Solid via the Webots
supervisor API. This module wraps that node and owns its launch/relaunch
lifecycle as a small two-state machine:

    LAUNCHED  — in flight
    WAITING   — landed, counting down to the next launch

Node-agnostic: ``Projectile`` takes anything exposing ``setVelocity`` /
``getPosition`` / ``getVelocity`` / ``resetPhysics`` / ``getField`` (the Webots
node interface), so it can be unit tested against a stub without Webots.

This is the seed of the attack-pattern system: ``ProjectileConfig`` will grow to
carry per-launch timing/locations once patterns are implemented. See
docs/superpowers/specs/2026-05-18-attack-pattern-and-ml-detection-design.md
"""

from dataclasses import dataclass, field
from enum import Enum, auto


@dataclass
class ProjectileConfig:
    """Tunable parameters for the incoming projectile's launch cycle."""

    spawn_position: list = field(
        default_factory=lambda: [2.59591e-05, -2.53, 0.04984303999999999]
    )
    launch_velocity: list = field(default_factory=lambda: [0, 4, 6, 0, 0, 0])
    ground_hit_threshold_m: float = 0.05  # world-Z below this → landed
    launch_delay_ms: float = 2000  # gap between shots


class _State(Enum):
    LAUNCHED = auto()
    WAITING = auto()


class Projectile:
    """Lifecycle of a single recycled incoming projectile."""

    def __init__(self, node, config: ProjectileConfig | None = None):
        self.config = config if config is not None else ProjectileConfig()
        self._node = node
        self._translation = node.getField("translation")
        self._state = _State.WAITING
        self._reset_time_ms = 0

    def launch(self):
        """Fire the projectile and enter the LAUNCHED state."""
        self._node.setVelocity(self.config.launch_velocity)
        self._state = _State.LAUNCHED

    def _reset(self):
        """Zero motion and teleport back to the spawn position."""
        self._node.setVelocity([0, 0, 0, 0, 0, 0])
        self._translation.setSFVec3f(self.config.spawn_position)
        self._node.resetPhysics()

    def step(self, now_ms):
        """Advance one tick. Call once per timestep with the sim clock in ms."""
        position = self._node.getPosition()
        velocity = self._node.getVelocity()

        if (
            self._state is _State.LAUNCHED
            and position[2] < self.config.ground_hit_threshold_m
            and velocity[2] < 0
        ):
            self._state = _State.WAITING
            self._reset_time_ms = now_ms
            self._reset()
        elif (
            self._state is _State.WAITING
            and now_ms - self._reset_time_ms > self.config.launch_delay_ms
        ):
            self.launch()
