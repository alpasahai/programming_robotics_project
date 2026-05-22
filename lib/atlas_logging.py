"""Shared logging setup for all ATLAS controllers.

Each Webots controller runs as its own process, so there is no shared runtime to
configure logging in. Instead every controller calls ``configure()`` from this
module (lib/ is on each controller's PYTHONPATH via its runtime.ini), giving one
place to set console verbosity for the whole project.

Adjust levels here
------------------
Edit ``LOG_LEVELS`` below — that single table controls every controller's
console level. To stream just one controller, set the others to "CRITICAL".

Scoreboard stream
-----------------
The operational scoreboard (shot-down vs ground-hit tally) has its own isolated
logger, "AtlasScore" (see ``configure_scoreboard``). It is controlled from the
same ``LOG_LEVELS`` table / env overrides as everything else: set "AtlasScore"
to "CRITICAL" here (or ATLASSCORE_LOG_LEVEL / ATLAS_LOG_LEVEL at run time) to
turn the scoreboard logging OFF without touching controller code.

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
    # Controllers default to WARNING so the console stays focused on the
    # scoreboard; raise an individual one to "INFO"/"DEBUG" (here or via its
    # env var) when debugging that controller.
    "AttackerController": "WARNING",
    "AtlasController": "WARNING",
    "SearchRadarController": "WARNING",
    "AtlasScore": "INFO",  # the operational scoreboard; set "CRITICAL" to silence
}
DEFAULT_LEVEL = "WARNING"

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


_SCORE_FORMAT = "[SCORE] %(message)s"


def configure_scoreboard(
    name: str = "AtlasScore", log_file: str | None = "atlas_score.log"
) -> logging.Logger:
    """Configure an ISOLATED logger for the operational scoreboard.

    The shot-down vs ground-hit tally goes to its own stream so it can be
    followed on its own: a ``[SCORE]``-tagged console line plus ``log_file``
    (default ``atlas_score.log``). The logger does NOT propagate to root, so the
    scores stay out of the busy main controller log.

    Its level comes from the same ``LOG_LEVELS`` table / env overrides as every
    controller (the "AtlasScore" key): set that to "CRITICAL" — or export
    ATLASSCORE_LOG_LEVEL / ATLAS_LOG_LEVEL — to turn scoreboard logging OFF
    without editing the controller. Safe to call more than once: the level is
    refreshed each call and handlers are added only once.

    Args:
        name:     Logger name, also the LOG_LEVELS / env-var key.
        log_file: Optional path for the dedicated score file; console-only if None.

    Returns:
        The isolated scoreboard logger.
    """
    logger = logging.getLogger(name)
    logger.setLevel(_resolve_level(name))
    logger.propagate = False
    if not logger.handlers:
        handlers: list[logging.Handler] = [logging.StreamHandler()]  # Webots console
        if log_file is not None:
            handlers.append(logging.FileHandler(log_file, mode="w", encoding="utf-8"))
        for handler in handlers:
            handler.setFormatter(logging.Formatter(_SCORE_FORMAT))
            logger.addHandler(handler)
    return logger
