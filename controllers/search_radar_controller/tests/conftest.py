"""pytest configuration for search_radar_controller tests.

Inserts the tests/ directory, the search_radar_controller/ directory, and the
shared lib/ directory onto sys.path so that test modules can import stubs
(``from stubs import ...``), production modules
(``from search_radar import ...``), and the shared geometry library
(``import geometry``) without path boilerplate.
"""
import sys
import os

_tests_dir = os.path.dirname(__file__)
_search_radar_controller_dir = os.path.dirname(_tests_dir)
_repo_root = os.path.dirname(os.path.dirname(_search_radar_controller_dir))
_lib_dir = os.path.join(_repo_root, "lib")

# Make stubs.py importable: ``from stubs import StubProjectile``
sys.path.insert(0, _tests_dir)
# Make production modules importable: ``from search_radar import ...``
sys.path.insert(0, _search_radar_controller_dir)
# Make shared lib/ importable: ``import geometry``
sys.path.insert(0, _lib_dir)
