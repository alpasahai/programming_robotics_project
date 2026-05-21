"""Turret Bullet — a single recycled projectile the turret fires on ENGAGING.

The bullet is one pre-placed Solid (DEF ATLAS_BULLET); the atlas_controller
supervisor recycles it rather than spawning/deleting a node each shot, mirroring
the incoming Projectile (ADR-0011): a stable node handle, no per-shot controller
process, and a unit-testable lifecycle. See
docs/superpowers/specs/2026-05-22-engage-fire-bullet-design.md and ADR-0013.

Two states:
    PARKED    — out of range, waiting to fire
    IN_FLIGHT — launched, travelling toward the intercept

The Bullet owns no hit decision: it is recycled by the controller when the
bullet-hit or ground-hit cue arrives, or a flight-time safety timeout fires.

setVelocity / resetPhysics discipline (same as Projectile): resetPhysics() zeroes
velocity in Webots, so fire() must NOT call it (it would kill the launch);
recycle() zeroes velocity first, then resetPhysics() on the parked body.

Node-agnostic: takes anything exposing setVelocity / getField / resetPhysics, so
it is unit-tested against a stub without Webots.
"""

import math
from dataclasses import dataclass, field
from enum import Enum, auto


@dataclass
class BulletConfig:
    """Tunable parameters for the bullet. Webots-tuned values; see the spec.

    muzzle_speed: launch speed (m/s). Capped below the tunnelling limit
                  v_max < (r_ball + r_bullet) / timestep (~20 m/s at 32 ms).
    muzzle_offset_m: spawn this far along the aim direction so the bullet starts
                  clear of the turret's bounding box.
    park_position: where the bullet rests while PARKED — far out of sensor range
                  (below the floor), like the despawned Projectile.
    """

    muzzle_speed: float = 18.0
    muzzle_offset_m: float = 0.6
    park_position: list = field(default_factory=lambda: [0.0, 0.0, -100.0])


class _State(Enum):
    PARKED = auto()
    IN_FLIGHT = auto()


class Bullet:
    """Lifecycle of the single recycled turret bullet."""

    def __init__(self, node, turret_position, config: BulletConfig | None = None):
        """
        Args:
            node:            Webots node handle for the DEF ATLAS_BULLET Solid.
            turret_position: world-frame [x, y, z] of the turret origin (the
                             muzzle reference; the bullet launches from an offset
                             along the aim direction).
            config:          Tunable parameters. Defaults to BulletConfig().
        """
        self.config = config if config is not None else BulletConfig()
        self._node = node
        self._translation = node.getField("translation")
        self._turret = list(turret_position)
        self._state = _State.PARKED
        self.age_steps = 0  # timesteps since launch (IN_FLIGHT only)

    @property
    def is_parked(self) -> bool:
        return self._state is _State.PARKED

    def fire(self, intercept_rel) -> None:
        """Teleport to the muzzle and launch toward the intercept (enter IN_FLIGHT).

        Args:
            intercept_rel: turret-relative [dx, dy, dz] aim point (metres), the
                           FSM fire command. The launch direction is its unit
                           vector; speed is config.muzzle_speed.

        Does NOT call resetPhysics() — that would zero the launch velocity in the
        same step (see module docstring). The body was reset on the previous
        recycle().
        """
        # Defensive guard: a zero intercept would teleport the bullet onto the
        # turret with zero velocity. Callers must never pass one — the FSM
        # intercept is always non-zero (see the predictor/range checks).
        mag = math.sqrt(sum(c * c for c in intercept_rel)) or 1.0
        direction = [c / mag for c in intercept_rel]

        spawn = [
            self._turret[i] + direction[i] * self.config.muzzle_offset_m
            for i in range(3)
        ]
        self._translation.setSFVec3f(spawn)
        self._node.setVelocity(
            [direction[i] * self.config.muzzle_speed for i in range(3)]
            + [0.0, 0.0, 0.0]
        )
        self.age_steps = 0
        self._state = _State.IN_FLIGHT

    def recycle(self) -> None:
        """Stop the bullet, park it out of range, reset physics (enter PARKED).

        Order matters: zero the velocity, teleport to the park position, THEN
        resetPhysics() — the body is already at zero so reset is a clean no-op on
        velocity and clears residual ODE inertia before the next flight.
        """
        self._node.setVelocity([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        self._translation.setSFVec3f(self.config.park_position)
        self._node.resetPhysics()
        self.age_steps = 0
        self._state = _State.PARKED

    def step(self) -> None:
        """Advance one tick: age the bullet while IN_FLIGHT (for the timeout)."""
        if self._state is _State.IN_FLIGHT:
            self.age_steps += 1
