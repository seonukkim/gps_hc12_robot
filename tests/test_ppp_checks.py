"""``ready_for_full_path_following`` 불변식 가드의 계약을 고정하는 테스트.

목적/역할 (KO):
    ``checks.assert_not_ready_for_full_path_following`` 가드가 패키지 전역
    불변식 -- "밖으로 나가는 모든 요약(summary)은 ``ready_for_full_path_following``
    이 *정확히* 리터럴 ``False`` 여야 한다" -- 를 엄격하게 지키는지 못 박는다.
    이 가드는 preview/controller/cli 가 요약을 출력·저장하기 직전에 반드시
    통과시키는 마지막 안전 관문이므로, 그 통과/거부 규칙이 회귀하지 않도록 잠근다.

계약 요약 (KO):
    통과 조건은 오직 리터럴 ``False`` 뿐이다. ``True`` 는 물론이고 키 누락,
    truthy 값, falsy 이지만 bool 이 아닌 ``0`` 까지 모두 ``FullPathFollowingNotAllowed``
    로 거부된다. 통과 시에는 입력 dict 을 *그대로* 반환한다(동일 객체, ``is`` 비교).
    또한 이 예외는 ``AssertionError`` 의 하위 클래스이므로, ``AssertionError`` 를
    넓게 잡는 호출자에게도 가드 발동이 보인다.

Purpose (EN):
    Locks the contract of the ``assert_not_ready_for_full_path_following`` guard:
    a summary passes only when the flag is the literal ``False``; ``True``, a
    missing key, and any non-bool (even a falsy ``0``) are all rejected with
    ``FullPathFollowingNotAllowed``. On success the exact input dict is returned
    (identity), and the exception subclasses ``AssertionError`` so broad
    ``except AssertionError`` handlers still catch it.
"""
from __future__ import annotations

import pytest

from tools.physical_path_planning import checks
from tools.physical_path_planning.checks import FullPathFollowingNotAllowed


def test_passes_and_returns_summary_when_flag_is_false() -> None:
    """리터럴 False 면 통과하고 입력 dict 을 동일 객체로 반환한다 / passes and
    returns the same dict when the flag is the literal ``False``."""
    summary = {"ready_for_full_path_following": False, "stage": "preview"}
    assert checks.assert_not_ready_for_full_path_following(summary) is summary


def test_rejects_true_flag() -> None:
    """플래그가 True 면 거부한다 / rejects a summary whose flag is ``True``."""
    with pytest.raises(FullPathFollowingNotAllowed, match="literal False"):
        checks.assert_not_ready_for_full_path_following(
            {"ready_for_full_path_following": True}
        )


def test_rejects_missing_flag() -> None:
    """필수 키가 아예 없으면 거부한다 / rejects when the required key is missing."""
    with pytest.raises(FullPathFollowingNotAllowed, match="missing required key"):
        checks.assert_not_ready_for_full_path_following({"stage": "preview"})


def test_rejects_truthy_non_bool() -> None:
    """리터럴 False 만 인정: bool 이 아닌 값(falsy 0 포함)은 거부 / only the literal
    ``False`` counts, so a non-bool -- even a falsy ``0`` -- is rejected."""
    # Only the literal False counts; a truthy or even falsy non-bool is rejected.
    with pytest.raises(FullPathFollowingNotAllowed, match="literal False"):
        checks.assert_not_ready_for_full_path_following(
            {"ready_for_full_path_following": 0}
        )


def test_guard_is_assertion_error_subclass() -> None:
    """가드 예외는 AssertionError 하위 클래스이다 / the guard exception subclasses
    ``AssertionError`` so broad handlers still see it fire."""
    # Callers that broadly catch AssertionError still see the guard fire.
    assert issubclass(FullPathFollowingNotAllowed, AssertionError)
