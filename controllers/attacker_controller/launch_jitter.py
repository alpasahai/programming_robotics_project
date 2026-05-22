"""Jitter/drift policy for the attacker's launch-point selector.

Pure helper — no global state, no Webots, no logging. The attacker controller
owns the mutable nominal and calls this once per launch.
"""

from __future__ import annotations


def next_launch_point(
    nominal: list[float],
    jitter_std: float,
    drift: list[float],
    rng,
) -> tuple[list[float], list[float]]:
    """Apply per-launch jitter and drift to produce the next launch origin.

    The x and y axes receive independent zero-mean Gaussian noise; the z axis
    is kept fixed. The nominal is advanced by ``drift`` each call so the
    launch origin wanders slowly over many engagements.

    Args:
        nominal:    Current nominal launch point ``[x, y, z]`` (metres, world
                    frame). Not mutated.
        jitter_std: Standard deviation of the per-axis Gaussian jitter applied
                    to x and y (metres).
        drift:      Per-call additive shift applied to the nominal before
                    returning it: ``new_nominal[i] = nominal[i] + drift[i]``.
                    Typically ``[dx, dy, dz]`` with small values.
        rng:        A ``numpy.random.Generator`` (e.g.
                    ``numpy.random.default_rng(seed)``). Caller owns it; this
                    function advances it by drawing two normal samples.

    Returns:
        ``(point, new_nominal)`` where

        - ``point``       — the jittered launch position for this launch
          ``[x + εx, y + εy, z]``;
        - ``new_nominal`` — the drifted nominal for the *next* call
          ``[x + dx, y + dy, z + dz]``.

    Both lists are freshly allocated; ``nominal`` is not mutated.
    """
    jx = rng.normal(0.0, jitter_std)
    jy = rng.normal(0.0, jitter_std)
    point = [nominal[0] + jx, nominal[1] + jy, nominal[2]]
    new_nominal = [nominal[i] + drift[i] for i in range(3)]
    return point, new_nominal
