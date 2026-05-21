"""CueEmitter — selects a fresh detection, converts it to world frame, emits it.

Producer-side mirror of ``SearchRadarLink`` (which consumes these cues in
atlas_controller). The payload is ``struct.pack("ddd", x, y, z)`` — three
IEEE-754 doubles, 24 bytes — sent on the wrapped Webots ``Emitter`` (channel is
configured on the device in the world file).

``Detection.position`` is turret-relative ``[dx, dy, dz]``; the world-frame cue is
``position + turret_position``. Per ADR-0006 only this bare coordinate crosses the
radio link to ATLAS — no node handle and no track_id.
"""
import struct
from dataclasses import dataclass


@dataclass(frozen=True)
class CueResult:
    """Outcome of an ``emit()`` call, consumed by CueTelemetry.

    ``status`` is one of:
      "sent"    — a fresh detection was selected and broadcast;
      "dropped" — a fresh detection was selected but no emitter device exists;
      "none"    — there were no fresh detections to emit this step.

    The position fields are populated for "sent" and "dropped" only.
    """

    status: str
    track_id: int | None = None
    turret_relative: list[float] | None = None
    world: list[float] | None = None


class CueEmitter:
    """Wraps a Webots ``Emitter`` to broadcast the chosen Search Radar cue.

    The controller is responsible for fetching the emitter device; this class
    accepts it (or ``None`` if the device is missing) so the wiring stays in the
    controller and the class stays testable with a stub.
    """

    def __init__(self, emitter):
        self._emitter = emitter

    def emit(self, fresh_detections, turret_position) -> CueResult:
        """Select the first fresh detection, convert to world frame, and send it.

        Args:
            fresh_detections: detections from ``SearchRadar.get_fresh_detections()``.
                              Empty list -> a "none" result.
            turret_position:  world-frame ``[x, y, z]`` of the turret origin; added
                              to the turret-relative detection position.

        Returns:
            A ``CueResult`` describing what happened (for telemetry).
        """
        if not fresh_detections:
            return CueResult(status="none")

        chosen = fresh_detections[0]
        world = [chosen.position[i] + turret_position[i] for i in range(3)]

        if self._emitter is None:
            return CueResult(
                "dropped", chosen.track_id, list(chosen.position), world
            )

        self._emitter.send(struct.pack("ddd", *world))
        return CueResult("sent", chosen.track_id, list(chosen.position), world)
