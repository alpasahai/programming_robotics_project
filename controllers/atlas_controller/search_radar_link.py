"""SearchRadarLink — consumes Search Radar position cues from a Webots Receiver.

The Search Radar emitter packs each cue as ``struct.pack("ddd", x, y, z)``
(three IEEE-754 doubles, 24 bytes) and transmits on channel 1.  This class
wraps the matching Receiver device and provides a clean, testable interface.

Staleness policy: the last received cue persists until a newer one arrives.
A freshness timeout (dropping stale cues after N steps of silence) is a noted
future extension and is intentionally NOT implemented here.
"""
import struct


class SearchRadarLink:
    """Wraps a Webots ``Receiver`` to consume Search Radar world-frame position cues.

    The controller is responsible for enabling the receiver (calling
    ``receiver.enable(timestep_ms)``) **before** constructing this object.
    ``SearchRadarLink.__init__`` does not call ``enable()`` — it accepts an
    already-enabled receiver so that the wiring stays in the controller and the
    class remains testable with a stub.

    Staleness policy:
        The most recently received cue is stored and returned by
        ``get_cue()`` until a new cue overwrites it.  If no packet has ever
        arrived, ``get_cue()`` returns ``None``.  There is no expiry timer.

    Args:
        receiver: A Webots ``Receiver`` device (or a compatible stub) that has
                  already been enabled on the desired timestep.  Must implement
                  ``getQueueLength() -> int``, ``getData() -> bytes``, and
                  ``nextPacket() -> None``.
    """

    def __init__(self, receiver):
        self._receiver = receiver
        self._last_cue: list[float] | None = None

    def update(self) -> None:
        """Drain the receiver queue, keeping the last packet as the current cue.

        Reads every packet currently in the queue.  If the queue is empty,
        the previously stored cue is left unchanged (staleness policy).

        Each packet is expected to be 24 bytes — three IEEE-754 doubles packed
        with ``struct.pack("ddd", x, y, z)`` — representing a world-frame
        target position in metres (ENU, Z-up).
        """
        while self._receiver.getQueueLength() > 0:
            data = self._receiver.getData()
            x, y, z = struct.unpack("ddd", data)
            self._last_cue = [x, y, z]
            self._receiver.nextPacket()

    def get_cue(self) -> list[float] | None:
        """Return the most recently received world-frame target position.

        Returns:
            A ``[x, y, z]`` list in metres (world frame, ENU Z-up), or
            ``None`` if no cue has been received yet.  The returned list is
            a direct reference to the stored cue; callers that mutate it
            should copy first.
        """
        return self._last_cue
