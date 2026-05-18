"""pytest configuration for atlas_controller tests.

Inserts both the tests/ directory and the atlas_controller/ directory onto
sys.path so that test modules can import stubs (``from stubs import ...``)
and production modules (``from fire_control_radar import ...``) without
maintaining their own path boilerplate.
"""
import sys
import os

_tests_dir = os.path.dirname(__file__)
_atlas_controller_dir = os.path.dirname(_tests_dir)
_repo_root = os.path.dirname(os.path.dirname(_atlas_controller_dir))
_lib_dir = os.path.join(_repo_root, "lib")

# Make stubs.py importable: ``from stubs import StubProjectile``
sys.path.insert(0, _tests_dir)
# Make production modules importable: ``from fire_control_radar import ...``
sys.path.insert(0, _atlas_controller_dir)
# Make shared lib/ importable: ``import geometry``
sys.path.insert(0, _lib_dir)
