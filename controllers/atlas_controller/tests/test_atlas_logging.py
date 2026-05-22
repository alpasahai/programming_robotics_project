"""Tests for the shared logging config — focus on central scoreboard control."""
import logging

import atlas_logging


def test_atlas_score_defaults_to_info():
    """The scoreboard stream is registered and defaults to INFO (on by default)."""
    assert atlas_logging.LOG_LEVELS.get("AtlasScore") == "INFO"


def test_controllers_default_to_warning():
    """Controllers are quiet by default so the scoreboard stays in focus."""
    for ctrl in ("AttackerController", "AtlasController", "SearchRadarController"):
        assert atlas_logging.LOG_LEVELS[ctrl] == "WARNING"
    assert atlas_logging.DEFAULT_LEVEL == "WARNING"


def test_scoreboard_logger_is_isolated_with_handlers():
    """configure_scoreboard returns a non-propagating logger with handlers."""
    log = atlas_logging.configure_scoreboard(name="AtlasScore", log_file=None)
    assert log.propagate is False
    assert len(log.handlers) >= 1
    assert log.level == logging.INFO  # AtlasScore is INFO in the central table


def test_scoreboard_level_controlled_centrally_by_env(monkeypatch):
    """Setting the level to CRITICAL (the 'off' switch) suppresses INFO scores."""
    monkeypatch.setenv("TESTSCOREOFF_LOG_LEVEL", "CRITICAL")
    log = atlas_logging.configure_scoreboard(name="TestScoreOff", log_file=None)
    assert log.level == logging.CRITICAL
    assert log.isEnabledFor(logging.INFO) is False  # score lines (INFO) silenced


def test_global_env_overrides_scoreboard_level(monkeypatch):
    """ATLAS_LOG_LEVEL beats the per-logger setting for the scoreboard too."""
    monkeypatch.setenv("ATLAS_LOG_LEVEL", "CRITICAL")
    log = atlas_logging.configure_scoreboard(name="TestScoreGlobal", log_file=None)
    assert log.level == logging.CRITICAL
