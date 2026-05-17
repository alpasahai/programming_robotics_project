"""Regression tests for the Webots world/PROTO layout."""

from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[3]
WORLD_FILE = REPO_ROOT / "worlds" / "ATLA_v1.wbt"
PROJECTILE_PROTO = REPO_ROOT / "protos" / "SimulatedProjectile.proto"
TURRET_PROTO = REPO_ROOT / "protos" / "AtlasTurret.proto"


def test_world_uses_project_local_protos_for_projectile_and_turret():
    """The world should instantiate local PROTOs instead of inline structures."""
    world = WORLD_FILE.read_text(encoding="utf-8")

    assert 'EXTERNPROTO "../protos/SimulatedProjectile.proto"' in world
    assert 'EXTERNPROTO "../protos/AtlasTurret.proto"' in world
    assert "DEF PROJECTILE SimulatedProjectile {" in world
    assert "DEF TURRET_BASE AtlasTurret {" in world
    assert "DEF PROJECTILE Solid {" not in world
    assert "Robot {" not in world


def test_projectile_and_turret_protos_preserve_controller_contracts():
    """PROTO files should keep the names the Python controller depends on."""
    projectile_proto = PROJECTILE_PROTO.read_text(encoding="utf-8")
    turret_proto = TURRET_PROTO.read_text(encoding="utf-8")

    assert "PROTO SimulatedProjectile" in projectile_proto
    assert "PROTO AtlasTurret" in turret_proto
    assert 'field SFString name "PROJECTILE"' in projectile_proto
    assert 'field SFString name "TURRET_BASE"' in turret_proto
    assert 'field SFString controller "atlas_controller"' in turret_proto
    assert "field SFBool supervisor TRUE" in turret_proto
    assert 'name "PAN_MOTOR"' in turret_proto
    assert 'name "TILT_MOTOR"' in turret_proto
    assert 'name "CAMERA"' in turret_proto
