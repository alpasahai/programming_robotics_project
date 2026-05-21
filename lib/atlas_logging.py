"""Shared logging setup for all ATLAS controllers.

Each Webots controller runs as its own process, so there is no shared runtime to
configure logging in. Instead every controller calls ``configure()`` from this
module (lib/ is on each controller's PYTHONPATH via its runtime.ini), giving one
place to set console verbosity for the whole project.

Adjust levels here
------------------
Edit ``LOG_LEVELS`` below — that single table controls every controller's
console level. To stream just one controller, set the others to "CRITICAL".

Per-run override
-----------------
An environment variable ``<NAME>_LOG_LEVEL`` (e.g. ATTACKERCONTROLLER_LOG_LEVEL)
overrides the table for that controller without editing code. A global
``ATLAS_LOG_LEVEL`` overrides *all* controllers and beats the per-controller var.
"""

import logging
import os

# ---------------------------------------------------------------------------
# The one place to set per-controller console log levels.
# Keys are the logger names passed to configure(); values are level names.
# ---------------------------------------------------------------------------
LOG_LEVELS = {
    "AttackerController": "DEBUG",
    "AtlasController": "INFO",
    "SearchRadarController": "INFO",
}
DEFAULT_LEVEL = "INFO"

_FORMAT = "[%(levelname)s - %(name)s] %(message)s"


def _resolve_level(name: str) -> int:
    """Pick a level: global env > per-controller env > LOG_LEVELS table > default."""
    level_name = (
        os.environ.get("ATLAS_LOG_LEVEL")
        or os.environ.get(f"{name.upper()}_LOG_LEVEL")
        or LOG_LEVELS.get(name, DEFAULT_LEVEL)
    )
    return getattr(logging, level_name.upper(), logging.INFO)


def configure(name: str, log_file: str | None = None) -> logging.Logger:
    """Configure root logging for a controller and return its logger.

    Args:
        name:     Logger name, also the LOG_LEVELS / env-var key (e.g.
                  "AttackerController").
        log_file: Optional path for a per-controller file trace. The console
                  StreamHandler is always added; the file handler only if given.

    Returns:
        The named logger.
    """
    handlers: list[logging.Handler] = [logging.StreamHandler()]  # Webots console
    if log_file is not None:
        handlers.append(logging.FileHandler(log_file, mode="w", encoding="utf-8"))

    logging.basicConfig(level=_resolve_level(name), format=_FORMAT, handlers=handlers)
    return logging.getLogger(name)
