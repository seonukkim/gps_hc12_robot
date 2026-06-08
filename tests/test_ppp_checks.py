"""Tests for the ready_for_full_path_following invariant guard."""
from __future__ import annotations

import pytest

from tools.physical_path_planning import checks
from tools.physical_path_planning.checks import FullPathFollowingNotAllowed


def test_passes_and_returns_summary_when_flag_is_false() -> None:
    summary = {"ready_for_full_path_following": False, "stage": "preview"}
    assert checks.assert_not_ready_for_full_path_following(summary) is summary


def test_rejects_true_flag() -> None:
    with pytest.raises(FullPathFollowingNotAllowed, match="literal False"):
        checks.assert_not_ready_for_full_path_following(
            {"ready_for_full_path_following": True}
        )


def test_rejects_missing_flag() -> None:
    with pytest.raises(FullPathFollowingNotAllowed, match="missing required key"):
        checks.assert_not_ready_for_full_path_following({"stage": "preview"})


def test_rejects_truthy_non_bool() -> None:
    # Only the literal False counts; a truthy or even falsy non-bool is rejected.
    with pytest.raises(FullPathFollowingNotAllowed, match="literal False"):
        checks.assert_not_ready_for_full_path_following(
            {"ready_for_full_path_following": 0}
        )


def test_guard_is_assertion_error_subclass() -> None:
    # Callers that broadly catch AssertionError still see the guard fire.
    assert issubclass(FullPathFollowingNotAllowed, AssertionError)
