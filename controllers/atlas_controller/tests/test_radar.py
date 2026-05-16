import sys, os

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from stubs import StubProjectile
from radar_system import RadarSystem

TIMESTEP_MS = 32


def test_get_target_position_returns_current_position():
    proj = StubProjectile([[1.0, 2.0, 3.0]])
    radar = RadarSystem(proj, TIMESTEP_MS)
    radar.update()
    assert radar.get_target_position() == [1.0, 2.0, 3.0]


def test_get_target_velocity_returns_zero_on_first_update():
    proj = StubProjectile([[1.0, 2.0, 3.0]])
    radar = RadarSystem(proj, TIMESTEP_MS)
    radar.update()
    assert radar.get_target_velocity() == [0.0, 0.0, 0.0]


def test_get_target_velocity_computes_from_history():
    dt = TIMESTEP_MS / 1000.0
    proj = StubProjectile([[0.0, 0.0, 0.0], [1.0, 2.0, 3.0]])
    radar = RadarSystem(proj, TIMESTEP_MS)
    radar.update()
    radar.update()
    vel = radar.get_target_velocity()
    assert abs(vel[0] - 1.0 / dt) < 0.001
    assert abs(vel[1] - 2.0 / dt) < 0.001
    assert abs(vel[2] - 3.0 / dt) < 0.001


def test_history_capped_at_max_size():
    positions = [[float(i), 0.0, 0.0] for i in range(20)]
    proj = StubProjectile(positions)
    radar = RadarSystem(proj, TIMESTEP_MS)
    for _ in range(20):
        radar.update()
    assert len(radar._history) == RadarSystem.HISTORY_SIZE
