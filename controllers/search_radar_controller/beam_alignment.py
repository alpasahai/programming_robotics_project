"""Search Radar beam-pose geometry — pure helpers + a thin Webots-node wrapper.

The pure functions convert between the spin-motor joint angle / the rendered FOV
node pose and the SearchRadar azimuth convention (x = sin(az), y = cos(az)).
``BeamAlignment`` (added in a later task) wraps the Webots devices so the
controller loop calls a single ``apply(radar)``.
"""
import math
from dataclasses import dataclass


def joint_angle_to_search_azimuth(joint_angle: float) -> float:
    """Convert a Webots +Z joint angle to the SearchRadar azimuth convention.

    SearchRadar azimuth uses x = sin(az), y = cos(az): positive azimuth turns
    local +Y toward +X. Webots positive rotation about +Z turns local +Y toward
    -X, so the physical beam azimuth is the negated joint angle, wrapped to
    [0, 2pi).

    Args:
        joint_angle: current +Z motor position in radians (from
                     ``PositionSensor.getValue()``).

    Returns:
        Azimuth in radians, wrapped to [0, 2pi).
    """
    return (-joint_angle) % (2 * math.pi)


def visual_beam_pose_to_search_frame(orientation, center, beam_range):
    """Derive ``(origin, azimuth, direction)`` from a rendered FOV node's pose.

    Webots returns a node orientation as a row-major 9-element rotation matrix.
    Column 1 (indices 1, 4, 7) is the node's local +Y axis in world coordinates —
    the direction the SR_FOV volume extends in SearchRadar.proto.

    Args:
        orientation: row-major 9-element rotation matrix (``node.getOrientation()``).
        center:      world position of the node centre (``node.getPosition()``).
        beam_range:  full beam length in metres; the phase-centre origin sits half
                     a range behind the centre along the beam direction.

    Returns:
        ``(origin, azimuth, direction)`` where ``origin`` is the phase-centre
        ``[x, y, z]``, ``azimuth = atan2(dir_x, dir_y)`` wrapped to [0, 2pi), and
        ``direction`` is the local +Y axis ``[x, y, z]``.
    """
    direction = [orientation[1], orientation[4], orientation[7]]
    origin = [center[i] - direction[i] * (beam_range / 2.0) for i in range(3)]
    azimuth = math.atan2(direction[0], direction[1]) % (2 * math.pi)
    return origin, azimuth, direction


@dataclass(frozen=True)
class BeamInfo:
    """What the beam alignment did this step — consumed only by telemetry.

    ``source`` is "visual" (pose taken from the rendered FOV node), "joint"
    (azimuth taken from the spin-motor angle sensor), or "self" (no device — the
    radar advances its own beam via scan_rate). The optional fields are populated
    only for the sources that compute them.
    """

    source: str
    azimuth: float | None = None
    origin: list[float] | None = None
    direction: list[float] | None = None


class BeamAlignment:
    """Aligns a SearchRadar's beam pose to the rendered FOV node each step.

    Wraps the Webots FOV node and spin-motor angle sensor so the controller loop
    calls a single ``apply(radar)``. Preference order mirrors the original
    controller:

      1. rendered FOV beam node present -> derive origin + azimuth from its pose
         (the proto visual is the source of truth for beam direction);
      2. else angle sensor present -> derive azimuth only (phase centre fixed);
      3. else neither -> the radar self-scans via its own scan_rate (no override).
    """

    def __init__(self, fov_beam_node, angle_sensor, max_range: float):
        self._fov_beam_node = fov_beam_node
        self._angle_sensor = angle_sensor
        self._max_range = max_range

    def apply(self, radar) -> BeamInfo:
        """Compute the beam pose, push it onto ``radar``, and report what was done."""
        if self._fov_beam_node is not None:
            origin, azimuth, direction = visual_beam_pose_to_search_frame(
                self._fov_beam_node.getOrientation(),
                self._fov_beam_node.getPosition(),
                self._max_range,
            )
            radar.set_beam_pose(azimuth, origin=origin)
            return BeamInfo("visual", azimuth, origin, direction)

        if self._angle_sensor is not None:
            azimuth = joint_angle_to_search_azimuth(self._angle_sensor.getValue())
            radar.set_beam_pose(azimuth)
            return BeamInfo("joint", azimuth)

        return BeamInfo("self")
