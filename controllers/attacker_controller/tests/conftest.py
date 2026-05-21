"""pytest configuration for attacker_controller tests.

Inserts both the tests/ directory and the attacker_controller/ directory onto
sys.path so test modules can import stubs (``from stubs import ...``) and
production modules (``from projectile import ...``) without per-file path
boilerplate. Mirrors controllers/atlas_controller/tests/conftest.py.
"""
import sys
import os

_tests_dir = os.path.dirname(__file__)
_attacker_controller_dir = os.path.dirname(_tests_dir)
_repo_root = os.path.dirname(os.path.dirname(_attacker_controller_dir))
_lib_dir = os.path.join(_repo_root, "lib")

sys.path.insert(0, _tests_dir)
sys.path.insert(0, _attacker_controller_dir)
sys.path.insert(0, _lib_dir)
