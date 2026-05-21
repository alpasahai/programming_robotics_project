"""Tests for beam_alignment — Search Radar beam-pose geometry helpers."""
import math

from beam_alignment import (
    joint_angle_to_search_azimuth,
    visual_beam_pose_to_search_frame,
)


def test_joint_angle_zero_maps_to_zero():
    assert joint_angle_to_search_azimuth(0.0) == 0.0


def test_joint_angle_is_negated_and_wrapped():
    """SearchRadar azimuth is the negated +Z joint angle, wrapped to [0, 2pi)."""
    result = joint_angle_to_search_azimuth(math.radians(90))
    assert math.isclose(result, 2 * math.pi - math.radians(90))


def test_visual_pose_identity_orientation_points_north():
    """Identity orientation: local +Y = world +Y -> azimuth 0; origin sits half a
    beam-range behind the node centre along +Y."""
    orientation = [1, 0, 0, 0, 1, 0, 0, 0, 1]   # row-major identity
    center = [0.0, 5.0, 2.0]
    origin, azimuth, direction = visual_beam_pose_to_search_frame(
        orientation, center, beam_range=10.0
    )
    assert direction == [0.0, 1.0, 0.0]
    assert azimuth == 0.0
    assert origin == [0.0, 0.0, 2.0]            # 5.0 - 1.0 * (10/2)


def test_visual_pose_east_facing_orientation():
    """Local +Y rotated onto world +X -> azimuth = atan2(1, 0) = pi/2."""
    # Column 1 (indices 1,4,7) is the local +Y axis; set it to (1, 0, 0).
    orientation = [0, 1, 0, 0, 0, 0, 0, 0, 0]
    center = [10.0, 0.0, 0.0]
    origin, azimuth, direction = visual_beam_pose_to_search_frame(
        orientation, center, beam_range=20.0
    )
    assert direction == [1.0, 0.0, 0.0]
    assert math.isclose(azimuth, math.pi / 2)
    assert origin == [0.0, 0.0, 0.0]            # 10.0 - 1.0 * (20/2)


from beam_alignment import BeamAlignment, BeamInfo


class StubNode:
    """Minimal Webots node: fixed orientation matrix + world position."""

    def __init__(self, orientation, position):
        self._orientation = orientation
        self._position = position

    def getOrientation(self):
        return self._orientation

    def getPosition(self):
        return self._position


class StubAngleSensor:
    def __init__(self, value):
        self._value = value

    def getValue(self):
        return self._value


class RecordingRadar:
    """Captures the last set_beam_pose() call (azimuth + origin)."""

    def __init__(self):
        self.azimuth = None
        self.origin = "unset"   # sentinel distinct from a real None argument

    def set_beam_pose(self, azimuth, origin=None):
        self.azimuth = azimuth
        self.origin = origin


def test_apply_uses_visual_node_when_present():
    """With an FOV node, apply() derives origin + azimuth from its pose and
    returns a 'visual' BeamInfo. The angle sensor is ignored."""
    node = StubNode([0, 1, 0, 0, 0, 0, 0, 0, 0], [10.0, 0.0, 0.0])  # +Y -> world +X
    align = BeamAlignment(node, StubAngleSensor(1.23), max_range=20.0)
    radar = RecordingRadar()

    info = align.apply(radar)

    assert info.source == "visual"
    assert math.isclose(radar.azimuth, math.pi / 2)
    assert radar.origin == [0.0, 0.0, 0.0]
    assert info.direction == [1.0, 0.0, 0.0]


def test_apply_falls_back_to_joint_when_no_visual_node():
    """Without an FOV node, apply() derives azimuth only (origin left as None)
    from the angle sensor and returns a 'joint' BeamInfo."""
    align = BeamAlignment(None, StubAngleSensor(math.radians(90)), max_range=20.0)
    radar = RecordingRadar()

    info = align.apply(radar)

    assert info.source == "joint"
    assert radar.origin is None
    assert math.isclose(radar.azimuth, 2 * math.pi - math.radians(90))
    assert math.isclose(info.azimuth, 2 * math.pi - math.radians(90))


def test_apply_self_scan_when_no_devices():
    """With neither device, apply() leaves the radar untouched (it self-scans via
    its own scan_rate) and returns a 'self' BeamInfo."""
    align = BeamAlignment(None, None, max_range=20.0)
    radar = RecordingRadar()

    info = align.apply(radar)

    assert info.source == "self"
    assert radar.azimuth is None       # set_beam_pose was never called
    assert info.azimuth is None
