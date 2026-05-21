def next_bounded_sweep_target(
    *,
    joint_angle: float,
    current_target: float,
    min_angle: float,
    max_angle: float,
    tolerance: float,
) -> float:
    if current_target == max_angle and joint_angle >= max_angle - tolerance:
        return min_angle
    if current_target == min_angle and joint_angle <= min_angle + tolerance:
        return max_angle
    return current_target
