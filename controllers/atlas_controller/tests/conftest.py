"""pytest configuration for atlas_controller tests.

Inserts the tests/ directory onto sys.path so that bare
``from stubs import ...`` imports resolve regardless of pytest's rootdir.
"""
import sys
import os

# Make the tests/ directory importable as a plain namespace
# (stubs.py lives here alongside the test modules).
sys.path.insert(0, os.path.dirname(__file__))
