import math

from sweep_control import next_bounded_sweep_target


def test_bounded_sweep_holds_target_until_upper_limit_is_reached():
    target = math.pi

    assert next_bounded_sweep_target(
        joint_angle=math.pi / 2,
        current_target=target,
        min_angle=0.0,
        max_angle=math.pi,
        tolerance=0.02,
    ) == target


def test_bounded_sweep_flips_at_upper_and_lower_limits():
    assert next_bounded_sweep_target(
        joint_angle=math.pi - 0.01,
        current_target=math.pi,
        min_angle=0.0,
        max_angle=math.pi,
        tolerance=0.02,
    ) == 0.0

    assert next_bounded_sweep_target(
        joint_angle=0.01,
        current_target=0.0,
        min_angle=0.0,
        max_angle=math.pi,
        tolerance=0.02,
    ) == math.pi
