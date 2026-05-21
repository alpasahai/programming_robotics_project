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
