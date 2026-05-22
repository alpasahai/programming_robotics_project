"""Regression tests for the Webots world/PROTO layout."""

from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[3]
WORLD_FILE = REPO_ROOT / "worlds" / "ATLA_v1.wbt"
PROJECTILE_PROTO = REPO_ROOT / "protos" / "Projectile.proto"
TURRET_PROTO = REPO_ROOT / "protos" / "AtlasTurret.proto"
SEARCH_RADAR_PROTO = REPO_ROOT / "protos" / "SearchRadar.proto"
BULLET_PROTO = REPO_ROOT / "protos" / "AtlasBullet.proto"


def test_world_uses_project_local_protos_for_projectile_and_turret():
    """The world should instantiate local PROTOs instead of inline structures."""
    world = WORLD_FILE.read_text(encoding="utf-8")

    assert 'EXTERNPROTO "../protos/Projectile.proto"' in world
    assert 'EXTERNPROTO "../protos/AtlasTurret.proto"' in world
    assert "DEF PROJECTILE Projectile {" in world
    assert "DEF TURRET_BASE AtlasTurret {" in world
    assert "DEF PROJECTILE Solid {" not in world
    assert "Robot {" not in world


def test_projectile_and_turret_protos_preserve_controller_contracts():
    """PROTO files should keep the names the Python controller depends on."""
    projectile_proto = PROJECTILE_PROTO.read_text(encoding="utf-8")
    turret_proto = TURRET_PROTO.read_text(encoding="utf-8")

    assert "PROTO Projectile" in projectile_proto
    assert "PROTO AtlasTurret" in turret_proto
    assert 'field SFString name "PROJECTILE"' in projectile_proto
    assert 'field SFString name "TURRET_BASE"' in turret_proto
    assert 'field SFString controller "atlas_controller"' in turret_proto
    assert "field SFBool supervisor TRUE" in turret_proto
    assert 'name "PAN_MOTOR"' in turret_proto
    assert 'name "TILT_MOTOR"' in turret_proto
    # Position sensors the controller reads for the turret's real aim (fed to the FCR).
    assert 'name "PAN_SENSOR"' in turret_proto
    assert 'name "TILT_SENSOR"' in turret_proto
    # Joint axes must realise the FSM/geometry az/el convention (lib/geometry.py):
    # pan about -Z (so +pan increases azimuth toward +X), tilt about +X (so tilt
    # elevates the +Y boresight instead of spinning it about its own axis).
    assert "axis 0 0 -1" in turret_proto  # PAN
    assert "axis 1 0 0" in turret_proto   # TILT


def test_search_radar_proto_preserves_runtime_fov_contracts():
    """The SearchRadar controller depends on these DEF/device names."""
    search_radar_proto = SEARCH_RADAR_PROTO.read_text(encoding="utf-8")

    assert "DEF SR_FOV_BEAM Pose" in search_radar_proto
    assert "geometry DEF SR_FOV_CONE Cone" in search_radar_proto
    # Radar height is a single template knob; the FOV cone offset/dimensions are
    # exposed PROTO fields the controller writes at runtime, bound via IS.
    assert "field SFFloat mastHeight 2" in search_radar_proto
    assert "field SFVec3f fovConeOffset 0 10 0" in search_radar_proto
    assert "field SFFloat fovConeHeight 20" in search_radar_proto
    assert "field SFFloat fovConeRadius 3.5" in search_radar_proto
    assert "translation IS fovConeOffset" in search_radar_proto
    assert "height IS fovConeHeight" in search_radar_proto
    assert "bottomRadius IS fovConeRadius" in search_radar_proto
    assert 'name "SEARCH_RADAR_ANGLE"' in search_radar_proto


def test_atlas_bullet_proto_is_a_preplaced_solid():
    """The bullet PROTO is a passive Solid (no controller/sensor), pre-placed
    once as DEF ATLAS_BULLET, and is the non-bouncy contact material."""
    world = WORLD_FILE.read_text(encoding="utf-8")
    bullet_proto = BULLET_PROTO.read_text(encoding="utf-8")

    # Pre-placed instance the supervisor recycles (not dynamically spawned).
    assert "DEF ATLAS_BULLET AtlasBullet" in world
    assert 'EXTERNPROTO "../protos/AtlasBullet.proto"' in world

    # Non-bouncy bullet/ball contact so the ball is not knocked away pre-despawn.
    assert 'material1 "atlas_bullet"' in world
    assert "bounce 0" in world
    assert "bounceVelocity 0" in world

    # A passive Solid: no controller, no sensor, no Robot wrapper.
    assert "PROTO AtlasBullet" in bullet_proto
    assert "Solid {" in bullet_proto
    assert "Robot {" not in bullet_proto
    assert "controller" not in bullet_proto
    assert "TouchSensor" not in bullet_proto
    assert 'contactMaterial "atlas_bullet"' in bullet_proto
