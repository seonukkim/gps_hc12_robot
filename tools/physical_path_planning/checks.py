"""Invariant guards for the consolidated path-planning package.

The load-bearing invariant: every summary the package emits must carry
``ready_for_full_path_following = False``. Autonomous full-path following is not
sanctioned yet (calibration is incomplete and the firmware motor-output gate is
the real safety), so preview/controller/cli all route their summary through
``assert_not_ready_for_full_path_following`` before printing or writing it.

목적/역할 (KO):
    패키지 전역의 "전체 경로 자율 추종 금지" 불변식을 강제하는 아주 작은 잎
    모듈이다. 이 패키지가 밖으로 내보내는 모든 요약(summary) dict 은 반드시
    ``ready_for_full_path_following = False`` 키를 실어야 한다. 자율 전체 경로
    추종은 아직 승인되지 않았고(보정 미완, 실제 안전 게이트는 펌웨어의 모터 출력
    차단), preview/controller/cli 는 요약을 출력·저장하기 전에 이 게이트를 통과시킨다.

시스템 내 위치 (KO):
    임포트 그래프의 잎(leaf)이다. 아무것도 임포트하지 않으며(표준 typing 제외),
    executor 를 거쳐 controller/cli 가 이 모듈을 사용한다. 방향은 언제나
    ...-> executor -> controller -> cli 로 단방향이다.

핵심 개념·함정 (KO):
    이 가드는 "정확히 bool False" 만 통과시킨다. truthy 값, ``None``, 심지어
    ``0`` 도 거부한다 -- 오직 리터럴 ``False`` 만이 "준비 안 됨"을 *명시적으로*
    선언한 것으로 본다. 이렇게 엄격히 잡는 이유는, 키가 누락되거나 실수로 참이
    새어 나가는 것을 조용히 통과시키지 않기 위함이다.

Purpose (EN):
    Tiny leaf module enforcing the package-wide "no autonomous full-path
    following" invariant. Every summary the package emits must carry
    ``ready_for_full_path_following = False``; preview/controller/cli route their
    summary through the guard before printing/writing. Only the literal ``False``
    passes -- a truthy value, ``None``, or ``0`` all raise -- so a missing key or
    an accidentally-truthy flag can never leak out silently. Imports nothing from
    the package; used downstream via ...-> executor -> controller -> cli.
"""
from __future__ import annotations

from typing import Mapping, TypeVar

READY_KEY = "ready_for_full_path_following"

_SummaryT = TypeVar("_SummaryT", bound=Mapping[str, object])


class FullPathFollowingNotAllowed(AssertionError):
    """Raised when a summary claims (or omits) the full-path-following readiness flag.

    요약이 ``ready_for_full_path_following`` 플래그를 리터럴 False 로 명시하지
    못했을 때(누락했거나 다른 값일 때) 발생한다. ``AssertionError`` 를 상속하므로
    기존 assert 기반 검증과 동일하게 취급된다.
    """


def assert_not_ready_for_full_path_following(summary: _SummaryT) -> _SummaryT:
    """Hard-assert ``summary[READY_KEY] is False`` and return ``summary`` unchanged.

    Returns the summary so callers can guard inline, e.g.::

        return checks.assert_not_ready_for_full_path_following(summary)

    Raises ``FullPathFollowingNotAllowed`` if the key is missing or the value is
    anything other than the literal ``False`` (a truthy value, ``None``, or even
    ``0`` -- only the exact bool ``False`` affirmatively declares "not ready").

    KO: ``summary[READY_KEY] is False`` 를 강하게 단언하고 요약을 그대로 반환한다
    (부수효과 없음, 인라인 가드용). 키가 없거나 값이 리터럴 False 가 아니면
    ``FullPathFollowingNotAllowed`` 를 던진다.
    """
    if READY_KEY not in summary:
        # 키 누락 자체가 계약 위반 / a missing key is itself a contract violation:
        # 요약은 준비 상태를 *명시적으로* False 로 선언해야 한다.
        raise FullPathFollowingNotAllowed(
            f"summary is missing required key {READY_KEY!r}; "
            "it must explicitly declare ready_for_full_path_following=False"
        )
    value = summary[READY_KEY]
    # ``is not False`` 로 정체성 비교(identity) 사용 / identity check, not truthiness:
    # 0, None, "" 같은 falsy 값도 거부해 우회로를 막는다.
    if value is not False:
        raise FullPathFollowingNotAllowed(
            f"{READY_KEY} must be the literal False, got {value!r}"
        )
    return summary
