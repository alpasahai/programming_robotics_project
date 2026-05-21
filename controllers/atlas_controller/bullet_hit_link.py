"""BulletHitLink — consumes the attacker's bullet-hit pulse from a Webots Receiver.

The attacker emitter sends exactly one packet on the step a bullet hit registers
(the turret bullet struck the projectile mid-air): struct.pack("i", count) (one
4-byte signed int, the running bullet-hit count) on channel 3, and nothing on any
other step. This class wraps the matching Receiver and reports, per step, whether
a pulse arrived.

This is ATLAS's sole source of bullet-hit (projectile-destroyed) truth: the FSM
never infers a destroyed projectile from the track estimate. It mirrors
AttackerGroundHitLink (channel 2) exactly — see the engage-fire bullet design
(docs/superpowers/specs/2026-05-22-engage-fire-bullet-design.md) and ADR-0009 for
the radio-cue pattern.
"""
import struct


class BulletHitLink:
    """Wraps a Webots ``Receiver`` to consume the attacker's bullet-hit pulse.

    The controller enables the receiver (``receiver.enable(timestep_ms)``)
    **before** constructing this object; ``__init__`` does not call ``enable()``
    so the wiring stays in the controller and the class stays testable with a
    stub. Stub surface: ``getQueueLength()``, ``getBytes()``, ``nextPacket()``.

    Args:
        receiver: an already-enabled Webots ``Receiver`` (or compatible stub).
    """

    def __init__(self, receiver):
        self._receiver = receiver
        self._hit_this_step = False
        self.count = 0  # latest received running bullet-hit count

    def update(self) -> None:
        """Drain the receiver queue, recording whether a pulse arrived this step.

        Sets ``hit_this_step()`` True if at least one packet was read during this
        call (else False), and stores the latest decoded count. Each packet is 4
        bytes — ``struct.pack("i", count)``.
        """
        self._hit_this_step = False
        while self._receiver.getQueueLength() > 0:
            (count,) = struct.unpack("i", self._receiver.getBytes())
            self.count = count
            self._hit_this_step = True
            self._receiver.nextPacket()

    def hit_this_step(self) -> bool:
        """Return True iff a bullet-hit pulse arrived during the most recent update()."""
        return self._hit_this_step
