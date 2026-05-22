"""Turret Bullet — a single recycled projectile the turret fires on ENGAGING.

The bullet is one pre-placed Solid (DEF ATLAS_BULLET); the atlas_controller
supervisor recycles it rather than spawning/deleting a node each shot, mirroring
the incoming Projectile (ADR-0011): a stable node handle, no per-shot controller
process, and a unit-testable lifecycle. See
docs/superpowers/specs/2026-05-22-engage-fire-bullet-design.md and ADR-0013.

Three states:
    PARKED    — out of range, waiting to fire
    LAUNCHING — teleported to the muzzle this step; velocity applied next step
    IN_FLIGHT — launched, travelling toward the intercept

The Bullet owns no hit decision: it is recycled by the controller when the
bullet-hit or ground-hit cue arrives, or a flight-time safety timeout fires.

Two-phase launch — why fire() and launch() are separate steps:
    Teleporting a Solid by writing its ``translation`` field is applied by Webots
    on the *next* ``robot.step()``, and that teleport resets the body's velocity.
    So a ``setVelocity()`` issued in the same step as the teleport is wiped before
    it can move the body — the bullet just drops at the muzzle (observed in
    Webots; unit tests over a stub cannot see this). fire() therefore only
    teleports + resetPhysics() and stashes the launch velocity; launch() applies
    that velocity one step later, once the teleport has landed and no pending
    translation write remains to clobber it. The controller drives the two phases
    (launch() before the next fire()). The incoming Projectile does not hit this
    because its relaunch is preceded by a despawn in an earlier step; the bullet's
    fire/launch are adjacent, so the ordering must be made explicit. See ADR-0013.

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
    LAUNCHING = auto()  # teleported to the muzzle this step; velocity applied next step
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
        self._launch_velocity = None  # pending 6-vector, applied by launch()

    @property
    def is_parked(self) -> bool:
        return self._state is _State.PARKED

    @property
    def is_launching(self) -> bool:
        """True after fire() teleported to the muzzle, before launch() runs.

        The controller calls launch() on the step *after* fire() so the
        translation-field teleport has been applied by robot.step() first;
        otherwise the deferred teleport wipes the launch velocity (Webots resets
        a Solid's velocity when its translation field write lands). See ADR-0013.
        """
        return self._state is _State.LAUNCHING

    def fire(self, intercept_rel) -> None:
        """Teleport to the muzzle and arm the launch (enter LAUNCHING).

        This is phase one of a two-step launch. It teleports the bullet to the
        muzzle and resets its physics, but does NOT apply the launch velocity
        yet: in Webots a translation-field write is applied on the *next*
        robot.step(), and that teleport resets the Solid's velocity — so a
        setVelocity() issued in the same step as the teleport is wiped before it
        ever moves the body. Instead we stash the launch velocity and apply it in
        launch() one step later, once the teleport has landed and there is no
        pending translation write left to clobber it. See ADR-0013.

        Args:
            intercept_rel: turret-relative [dx, dy, dz] aim point (metres), the
                           FSM fire command. The launch direction is its unit
                           vector; speed is config.muzzle_speed.
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
        self._node.resetPhysics()
        self._launch_velocity = [
            direction[i] * self.config.muzzle_speed for i in range(3)
        ] + [0.0, 0.0, 0.0]
        self.age_steps = 0
        self._state = _State.LAUNCHING

    def launch(self) -> None:
        """Apply the stashed launch velocity (enter IN_FLIGHT) — phase two.

        Must be called on the step *after* fire(), so the muzzle teleport has
        already been applied by robot.step() and no pending translation-field
        write remains to reset the velocity (see fire()). No-op unless LAUNCHING.
        """
        if self._state is not _State.LAUNCHING:
            return
        self._node.setVelocity(self._launch_velocity)
        self._launch_velocity = None
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
        self._launch_velocity = None
        self._state = _State.PARKED

    def step(self) -> None:
        """Advance one tick: age the bullet while IN_FLIGHT (for the timeout)."""
        if self._state is _State.IN_FLIGHT:
            self.age_steps += 1
