"""Per-step telemetry for the Search Radar controller — a development instrument.

Mirror of atlas_controller's TrackTelemetry: it owns every log line the
controller's loop used to emit inline (cue sent, cue dropped, buffered-but-stale,
no detection at all), so the loop stays pure orchestration. Verbosity is
controlled centrally via lib/atlas_logging.py — set SearchRadarController to INFO
to silence the per-step DEBUG trace.

Note: the buffered-track log reports track_ids (from the public get_detections())
but not per-track ages, which the old inline log read from SearchRadar's private
_track_buffer. Dropping ages keeps the membrane closed (ADR-0006); they were a
debug-only nicety.
"""


class CueTelemetry:
    """Logs one line per step describing what the Search Radar emitted (or didn't)."""

    def __init__(self, log):
        self._log = log

    @staticmethod
    def _beam_fields(beam):
        """Format the BeamInfo fields once for reuse across every log line."""
        azimuth = "%.3f" % beam.azimuth if beam.azimuth is not None else "n/a"
        origin = (
            "(%.3f, %.3f, %.3f)" % tuple(beam.origin)
            if beam.origin is not None
            else "n/a"
        )
        direction = (
            "(%.3f, %.3f, %.3f)" % tuple(beam.direction)
            if beam.direction is not None
            else "n/a"
        )
        return beam.source, azimuth, origin, direction

    def report(
        self,
        step_count,
        sim_time,
        beam,
        all_detections,
        fresh_detections,
        cue,
    ):
        """Emit the single log line appropriate to this step's cue outcome.

        Args:
            step_count:       simulation step counter.
            sim_time:         robot.getTime() in seconds.
            beam:             BeamInfo from BeamAlignment.apply().
            all_detections:   SearchRadar.get_detections() (live buffer).
            fresh_detections: SearchRadar.get_fresh_detections() (age-0 only).
            cue:              CueResult from CueEmitter.emit().
        """
        source, azimuth, origin, direction = self._beam_fields(beam)

        if cue.status == "sent":
            self._log.info(
                "[SR CUE SENT] step=%d t=%.2fs source=fresh_beam_hit track_id=%s "
                "sender=search_radar_controller device=SR_CUE_EMITTER channel=1 "
                "beam_source=%s beam_az=%s origin=%s dir=%s "
                "turret_relative=(%.3f, %.3f, %.3f) world=(%.3f, %.3f, %.3f)",
                step_count,
                sim_time,
                cue.track_id,
                source,
                azimuth,
                origin,
                direction,
                cue.turret_relative[0],
                cue.turret_relative[1],
                cue.turret_relative[2],
                cue.world[0],
                cue.world[1],
                cue.world[2],
            )
        elif cue.status == "dropped":
            self._log.warning(
                "[SR CUE DROPPED] step=%d t=%.2fs sender=search_radar_controller "
                "reason=missing_emitter track_id=%s world=(%.3f, %.3f, %.3f)",
                step_count,
                sim_time,
                cue.track_id,
                cue.world[0],
                cue.world[1],
                cue.world[2],
            )
        elif all_detections:
            self._log.debug(
                "[SR BUFFER HELD] step=%d t=%.2fs buffered_track_ids=%s "
                "beam_source=%s beam_az=%s origin=%s dir=%s",
                step_count,
                sim_time,
                [d.track_id for d in all_detections],
                source,
                azimuth,
                origin,
                direction,
            )
        else:
            self._log.debug(
                "[SR NO CUE] step=%d t=%.2fs detections=0 "
                "beam_source=%s beam_az=%s origin=%s dir=%s",
                step_count,
                sim_time,
                source,
                azimuth,
                origin,
                direction,
            )
