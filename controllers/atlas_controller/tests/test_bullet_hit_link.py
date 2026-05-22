"""Tests for BulletHitLink — wraps a Webots Receiver to consume the attacker's
bullet-hit pulse (channel 3).

The attacker emits exactly one packet on the step a bullet hit registers:
struct.pack("i", count) on channel 3. The link drains the queue each update()
and reports whether a pulse arrived this step plus the latest count. Mirrors
AttackerGroundHitLink (channel 2).
"""
import struct
from bullet_hit_link import BulletHitLink


class StubReceiver:
    """Test double for a Webots Receiver pre-loaded with packed int packets.

    Mimics the Webots Receiver queue API: getQueueLength(), getBytes(),
    nextPacket(). Each int in ``counts`` is packed as "i" exactly as the
    attacker emitter does.
    """

    def __init__(self, counts=None):
        self._queue = [struct.pack("i", c) for c in (counts or [])]

    def load(self, counts):
        """Append more packed-int packets to the queue (simulate a new step)."""
        self._queue.extend(struct.pack("i", c) for c in counts)

    def getQueueLength(self):
        return len(self._queue)

    def getBytes(self):
        return self._queue[0]

    def nextPacket(self):
        self._queue.pop(0)


def test_no_hit_before_any_update():
    """hit_this_step() is False and count is 0 before update() runs."""
    link = BulletHitLink(StubReceiver())
    assert link.hit_this_step() is False
    assert link.count == 0


def test_empty_queue_reports_no_hit():
    """update() on an empty queue → no hit, count unchanged."""
    link = BulletHitLink(StubReceiver())
    link.update()
    assert link.hit_this_step() is False
    assert link.count == 0


def test_one_packet_reports_hit_and_count():
    """A single packet → hit_this_step() True and the decoded count."""
    link = BulletHitLink(StubReceiver([1]))
    link.update()
    assert link.hit_this_step() is True
    assert link.count == 1


def test_multiple_packets_drained_keeps_latest_count():
    """Several packets in one update() → hit True, latest count retained."""
    link = BulletHitLink(StubReceiver([1, 2, 3]))
    link.update()
    assert link.hit_this_step() is True
    assert link.count == 3


def test_hit_flag_clears_on_next_silent_update():
    """The per-step hit flag resets to False on a later empty update(),
    while count persists."""
    receiver = StubReceiver([5])
    link = BulletHitLink(receiver)
    link.update()
    assert link.hit_this_step() is True

    link.update()  # queue now empty
    assert link.hit_this_step() is False
    assert link.count == 5  # latest count persists
