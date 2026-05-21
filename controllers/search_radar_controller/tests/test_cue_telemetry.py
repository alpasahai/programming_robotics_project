"""Tests for cue_telemetry — per-step logging for the Search Radar controller."""
from beam_alignment import BeamInfo
from cue_emitter import CueResult
from cue_telemetry import CueTelemetry
from search_radar import Detection


class RecordingLog:
    """Captures (level, message_template) for each logging call."""

    def __init__(self):
        self.calls = []

    def info(self, msg, *args):
        self.calls.append(("info", msg))

    def warning(self, msg, *args):
        self.calls.append(("warning", msg))

    def debug(self, msg, *args):
        self.calls.append(("debug", msg))


def _report(log, *, beam, all_detections, fresh_detections, cue):
    CueTelemetry(log).report(
        step_count=1,
        sim_time=0.1,
        beam=beam,
        all_detections=all_detections,
        fresh_detections=fresh_detections,
        cue=cue,
    )


def test_sent_cue_logs_info():
    log = RecordingLog()
    det = Detection(0, [1.0, 2.0, 3.0])
    _report(
        log,
        beam=BeamInfo("visual", 0.5, [0.0, 0.0, 0.0], [0.0, 1.0, 0.0]),
        all_detections=[det],
        fresh_detections=[det],
        cue=CueResult("sent", 0, [1.0, 2.0, 3.0], [4.0, 5.0, 6.0]),
    )
    assert log.calls[0][0] == "info"
    assert "[SR CUE SENT]" in log.calls[0][1]


def test_dropped_cue_logs_warning():
    log = RecordingLog()
    det = Detection(0, [1.0, 2.0, 3.0])
    _report(
        log,
        beam=BeamInfo("self"),
        all_detections=[det],
        fresh_detections=[],
        cue=CueResult("dropped", 0, [1.0, 2.0, 3.0], [4.0, 5.0, 6.0]),
    )
    assert log.calls[0][0] == "warning"
    assert "[SR CUE DROPPED]" in log.calls[0][1]


def test_detections_but_no_fresh_logs_buffer_held():
    log = RecordingLog()
    det = Detection(0, [1.0, 2.0, 3.0])
    _report(
        log,
        beam=BeamInfo("joint", 0.2),
        all_detections=[det],     # buffered
        fresh_detections=[],      # but none fresh this cycle
        cue=CueResult("none"),
    )
    assert log.calls[0][0] == "debug"
    assert "[SR BUFFER HELD]" in log.calls[0][1]


def test_no_detections_logs_no_cue():
    log = RecordingLog()
    _report(
        log,
        beam=BeamInfo("self"),
        all_detections=[],
        fresh_detections=[],
        cue=CueResult("none"),
    )
    assert log.calls[0][0] == "debug"
    assert "[SR NO CUE]" in log.calls[0][1]
