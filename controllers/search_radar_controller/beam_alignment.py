"""Search Radar beam-pose geometry — pure helpers + a thin Webots-node wrapper.

The pure functions convert between the spin-motor joint angle / the rendered FOV
node pose and the SearchRadar azimuth convention (x = sin(az), y = cos(az)).
``BeamAlignment`` (added in a later task) wraps the Webots devices so the
controller loop calls a single ``apply(radar)``.
"""
import math


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
