"""Shared Webots scene DEF names.

These DEF names are referenced from more than one controller process via
``getFromDef``. Defining them once here (lib/ is on every controller's
PYTHONPATH) keeps the controllers in sync and turns a silent typo — getFromDef
returns None for an unknown DEF — into an import error instead.

The world file (worlds/ATLA_v1.wbt) is the source of truth for the actual DEF
strings; these constants must mirror it. Device names and PROTO-internal DEFs
are intentionally not here: each is used by a single controller, so a local
literal is clearer than a shared constant.
"""

DEF_PROJECTILE = "PROJECTILE"  # the incoming ball (Projectile.proto)
DEF_TURRET_BASE = "TURRET_BASE"  # the turret origin (AtlasTurret.proto)
