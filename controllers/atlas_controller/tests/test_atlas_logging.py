"""Tests for the shared logging config — focus on central scoreboard control."""
import logging

import atlas_logging


def test_log_levels_table_has_atlas_score():
    """The scoreboard stream is registered in the central level table."""
    assert "AtlasScore" in atlas_logging.LOG_LEVELS


def test_scoreboard_logger_is_isolated_with_handlers():
    """configure_scoreboard returns a non-propagating logger with handlers."""
    log = atlas_logging.configure_scoreboard(name="TestScoreIsolated", log_file=None)
    assert log.propagate is False
    assert len(log.handlers) >= 1
    assert log.level == logging.INFO  # default from the table / DEFAULT_LEVEL


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
