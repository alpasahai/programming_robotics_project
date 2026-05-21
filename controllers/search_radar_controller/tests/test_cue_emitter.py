"""Tests for cue_emitter — producer-side mirror of SearchRadarLink.

Selects the first fresh detection, converts it from turret-relative to world
frame, packs struct.pack("ddd", x, y, z), and sends on the wrapped emitter.
"""
import struct

from search_radar import Detection
from cue_emitter import CueEmitter, CueResult


class StubEmitter:
    """Captures struct-packed payloads passed to send()."""

    def __init__(self):
        self.sent = []

    def send(self, data):
        self.sent.append(data)


def test_emit_returns_none_status_when_no_fresh_detections():
    emitter = StubEmitter()
    result = CueEmitter(emitter).emit([], turret_position=[1.0, 2.0, 3.0])
    assert result.status == "none"
    assert emitter.sent == []


def test_emit_converts_to_world_frame_and_sends():
    emitter = StubEmitter()
    detection = Detection(track_id=0, position=[1.0, 2.0, 3.0])  # turret-relative
    result = CueEmitter(emitter).emit([detection], turret_position=[10.0, 20.0, 30.0])

    assert result.status == "sent"
    assert result.track_id == 0
    assert result.turret_relative == [1.0, 2.0, 3.0]
    assert result.world == [11.0, 22.0, 33.0]
    assert len(emitter.sent) == 1
    assert struct.unpack("ddd", emitter.sent[0]) == (11.0, 22.0, 33.0)


def test_emit_selects_first_fresh_detection():
    emitter = StubEmitter()
    d0 = Detection(track_id=0, position=[1.0, 0.0, 0.0])
    d1 = Detection(track_id=1, position=[2.0, 0.0, 0.0])
    result = CueEmitter(emitter).emit([d0, d1], turret_position=[0.0, 0.0, 0.0])
    assert result.track_id == 0


def test_emit_reports_dropped_when_emitter_missing():
    detection = Detection(track_id=2, position=[0.0, 0.0, 0.0])
    result = CueEmitter(None).emit([detection], turret_position=[5.0, 5.0, 5.0])
    assert result.status == "dropped"
    assert result.track_id == 2
    assert result.world == [5.0, 5.0, 5.0]
