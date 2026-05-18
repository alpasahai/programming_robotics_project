"""Tests for SearchRadarLink — wraps a Webots Receiver to consume Search Radar cues.

The SearchRadarLink drains the Receiver queue on each update() and retains the
last received cue until a newer one arrives (staleness policy: last-cue-persists).
"""
import struct
import pytest
from search_radar_link import SearchRadarLink


# ---------------------------------------------------------------------------
# Stub Webots Receiver
# ---------------------------------------------------------------------------

class StubReceiver:
    """Test double for a Webots Receiver pre-loaded with struct-packed packets.

    Mimics the Webots Receiver queue API:
        getQueueLength() -> int
        getData()        -> bytes  (current front packet)
        nextPacket()     -> None   (pop front of queue)

    The receiver is constructed with an optional list of (x, y, z) tuples.
    Each tuple is packed as "ddd" (three doubles) exactly as the Search Radar
    emitter does. The queue starts populated; call drain() is not exposed —
    SearchRadarLink.update() drives the queue via nextPacket().
    """

    def __init__(self, positions=None):
        """
        Args:
            positions: list of (x, y, z) tuples to pre-load as packets.
                       Defaults to an empty queue.
        """
        self._queue = [struct.pack("ddd", x, y, z) for x, y, z in (positions or [])]

    def getQueueLength(self):
        """Return the number of unread packets currently in the queue."""
        return len(self._queue)

    def getData(self):
        """Return the raw bytes of the front packet without popping it."""
        return self._queue[0]

    def nextPacket(self):
        """Pop the front packet, advancing the queue."""
        self._queue.pop(0)


# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------

def test_get_cue_returns_none_before_any_update():
    """get_cue() must return None before update() has ever been called."""
    receiver = StubReceiver()
    link = SearchRadarLink(receiver)
    assert link.get_cue() is None


def test_update_with_one_packet_returns_that_position():
    """After update() with one queued packet, get_cue() returns its position."""
    pos = (1.5, 2.5, 3.5)
    receiver = StubReceiver([pos])
    link = SearchRadarLink(receiver)
    link.update()
    assert link.get_cue() == pytest.approx(list(pos))


def test_update_with_several_packets_returns_last_one():
    """With multiple queued packets, get_cue() returns only the LAST one."""
    positions = [(1.0, 2.0, 3.0), (4.0, 5.0, 6.0), (7.0, 8.0, 9.0)]
    receiver = StubReceiver(positions)
    link = SearchRadarLink(receiver)
    link.update()
    assert link.get_cue() == pytest.approx(list(positions[-1]))


def test_update_with_empty_queue_retains_previous_cue():
    """If update() finds an empty queue, the previous cue is preserved (staleness policy)."""
    pos = (10.0, 20.0, 30.0)
    receiver = StubReceiver([pos])
    link = SearchRadarLink(receiver)

    # First update: consume the one packet
    link.update()
    assert link.get_cue() == pytest.approx(list(pos))

    # Second update: queue is now empty — cue must persist
    link.update()
    assert link.get_cue() == pytest.approx(list(pos))


def test_get_cue_returns_list_not_tuple():
    """get_cue() must return a list, not a tuple (struct.unpack returns tuple)."""
    receiver = StubReceiver([(1.0, 2.0, 3.0)])
    link = SearchRadarLink(receiver)
    link.update()
    result = link.get_cue()
    assert isinstance(result, list)
