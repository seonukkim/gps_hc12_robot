"""Invariant guards for the consolidated path-planning package.

The load-bearing invariant: every summary the package emits must carry
``ready_for_full_path_following = False``. Autonomous full-path following is not
sanctioned yet (calibration is incomplete and the firmware motor-output gate is
the real safety), so preview/controller/cli all route their summary through
``assert_not_ready_for_full_path_following`` before printing or writing it.
"""
from __future__ import annotations

from typing import Mapping, TypeVar

READY_KEY = "ready_for_full_path_following"

_SummaryT = TypeVar("_SummaryT", bound=Mapping[str, object])


class FullPathFollowingNotAllowed(AssertionError):
    """Raised when a summary claims (or omits) the full-path-following readiness flag."""


def assert_not_ready_for_full_path_following(summary: _SummaryT) -> _SummaryT:
    """Hard-assert ``summary[READY_KEY] is False`` and return ``summary`` unchanged.

    Returns the summary so callers can guard inline, e.g.::

        return checks.assert_not_ready_for_full_path_following(summary)

    Raises ``FullPathFollowingNotAllowed`` if the key is missing or the value is
    anything other than the literal ``False`` (a truthy value, ``None``, or even
    ``0`` -- only the exact bool ``False`` affirmatively declares "not ready").
    """
    if READY_KEY not in summary:
        raise FullPathFollowingNotAllowed(
            f"summary is missing required key {READY_KEY!r}; "
            "it must explicitly declare ready_for_full_path_following=False"
        )
    value = summary[READY_KEY]
    if value is not False:
        raise FullPathFollowingNotAllowed(
            f"{READY_KEY} must be the literal False, got {value!r}"
        )
    return summary
