"""Launch geometry for the attacker's sectors.

A sector is a bearing around the turret (assumed near the world origin). ENU,
Z-up; the turret's aim convention is pan = atan2(dx, dy), so a target at bearing
``az`` and ground range R sits at horizontal offset (R·sin az, R·cos az). The
ball launches from there with horizontal velocity aimed back at the origin plus
an upward component, matching the original single-corridor launch.
"""
import math


def sector_launch(azimuth_rad, ground_range, height, h_speed, v_speed):
    """Return ``(spawn_position, launch_velocity)`` for a launch sector.

    Args:
        azimuth_rad:  bearing of the sector (pan convention, atan2(dx, dy)).
        ground_range: horizontal distance from the turret/origin (m).
        height:       spawn height (m).
        h_speed:      horizontal speed toward the turret (m/s).
        v_speed:      upward speed (m/s).

    Returns:
        (spawn_position[x,y,z], launch_velocity[vx,vy,vz,wx,wy,wz]).
    """
    sx, cx = math.sin(azimuth_rad), math.cos(azimuth_rad)
    spawn_position = [ground_range * sx, ground_range * cx, height]
    launch_velocity = [-h_speed * sx, -h_speed * cx, v_speed, 0.0, 0.0, 0.0]
    return spawn_position, launch_velocity
