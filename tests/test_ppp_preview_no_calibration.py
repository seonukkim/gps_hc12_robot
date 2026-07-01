"""Preview must work with MISSING turn-angle calibration.

The guarantee under test: a field operator can preview an A->B serpentine plan
before any turn-angle calibration exists. The resolver falls back to repeated
pulses (``fallback_to_repeated_pulses=True``) and preview still emits a full,
guarded (never-ready) plan -- it must not raise or block on missing calibration.

목적/역할 (KO):
    "선회각(turn-angle) 보정이 아직 없는" 흔한 현장 상태에서도 미리보기
    (preview) 가 동작함을 못 박는다. 현장 작업자는 어떤 보정도 만들기 전에 A->B
    서펜타인 계획을 미리 볼 수 있어야 한다. 리졸버는 반복 펄스(repeated pulses)
    로 폴백하고, preview 는 여전히 완전하고 가드된(절대 ready 아님) 계획을
    내놓아야 하며, 보정 부재로 예외를 던지거나 막혀서는 안 된다.

핵심 계약·불변식 (KO):
    - 보정 인자를 전혀 주지 않아도 preview 는 ``fallback_to_repeated_pulses=True``,
      ``connector_mode_effective="repeated_pulses"`` 로 계획을 완성한다.
    - 어떤 목표 모드(absolute/bearing_distance/relative_enu)에서도, 그리고 명시적
      으로 보정을 ``None`` 으로 resolve 한 경우에도 동일하게 동작한다.
    - ``direct_line`` 형상은 폭/보정 없이도 성립(workspace 는 ``None``).
    - 대각(diagonal) 형상은 폭이 필수 -> 폭 없으면 ``ValueError``.
    - 모든 요약은 ``ready_for_full_path_following=False`` 이며 readiness 가드를
      통과한다. PNG 렌더링은 matplotlib 유무에 따라 Path 또는 None(우아한 저하).

Purpose (EN):
    Locks that preview works with MISSING turn-angle calibration: it falls back
    to repeated pulses and still emits a full, guarded (never-ready) plan across
    goal modes and with calibration explicitly resolved to ``None``. Direct-line
    needs no width/calibration; diagonal requires width; every summary passes the
    readiness guard; PNG rendering degrades gracefully to ``None`` without mpl.
"""
from __future__ import annotations

import pytest

from tools.physical_path_planning import preview
from tools.physical_path_planning.checks import FullPathFollowingNotAllowed


def test_preview_builds_with_no_calibration_at_all() -> None:
    """보정 인자를 전혀 주지 않아도 반복-펄스 폴백으로 완전한 가드 계획을 낸다 / with
    no calibration args at all, preview completes a full, guarded plan via the
    repeated-pulses fallback."""
    summary = preview.build_preview(
        start_lat=35.0,
        start_lon=129.0,
        goal_mode="absolute",
        goal_lat=35.00008,
        goal_lon=129.00008,
        workspace_width_m=2.0,
    )
    assert summary["fallback_to_repeated_pulses"] is True
    assert summary["connector_mode_effective"] == "repeated_pulses"
    assert summary["segment_count"] > 1
    assert summary["lane_count"] >= 1
    assert summary["ready_for_full_path_following"] is False


def test_preview_with_explicitly_missing_angle_calibration() -> None:
    """선회각 소스를 명시적으로 None 으로 resolve 해도 preview 는 반복 펄스로 계획을
    낸다 / resolving calibration with the turn-angle source explicitly absent
    still yields a repeated-pulses plan."""
    # Resolve calibration with the turn-angle source absent (the common field
    # state) and confirm preview still produces a plan via repeated pulses.
    cal = preview.resolve_preview_calibration(turn_angle_calibration_json=None)
    assert cal["fallback_to_repeated_pulses"] is True
    summary = preview.build_preview(
        start_lat=35.0,
        start_lon=129.0,
        goal_mode="bearing_distance",
        goal_bearing_deg=45.0,
        goal_distance_m=10.0,
        workspace_width_m=3.0,
        calibration=cal,
    )
    assert summary["segment_count"] > 1
    assert summary["fallback_to_repeated_pulses"] is True
    assert summary["ready_for_full_path_following"] is False


def test_preview_direct_line_needs_no_width_or_calibration() -> None:
    """direct_line 형상은 폭·보정 없이 성립하고 workspace 는 None / the direct-line
    shape needs no width or calibration and reports ``workspace is None``."""
    summary = preview.build_preview(
        start_lat=35.0,
        start_lon=129.0,
        goal_mode="absolute",
        goal_lat=35.00008,
        goal_lon=129.00008,
        path_shape="direct_line",
    )
    assert summary["workspace"] is None
    assert summary["segment_count"] >= 1
    assert summary["ready_for_full_path_following"] is False


def test_preview_diagonal_requires_width() -> None:
    """대각 형상(기본값)은 폭이 없으면 ValueError / the diagonal shape (the default)
    raises ``ValueError`` when no workspace width is given."""
    with pytest.raises(ValueError, match="workspace width is required"):
        preview.build_preview(
            start_lat=35.0,
            start_lon=129.0,
            goal_mode="absolute",
            goal_lat=35.00008,
            goal_lon=129.00008,
        )


def test_preview_summary_is_guarded_not_ready() -> None:
    """요약은 readiness 가드를 통과하고, 플래그를 true 로 뒤집으면 가드가 발동한다 /
    the summary passes the readiness guard, and flipping the flag true makes the
    guard fire."""
    # The summary always passes the readiness guard (and the guard would fire if
    # the flag were ever flipped true).
    summary = preview.build_preview(
        start_lat=35.0,
        start_lon=129.0,
        goal_mode="relative_enu",
        goal_east_m=5.0,
        goal_north_m=8.0,
        workspace_width_m=2.0,
    )
    from tools.physical_path_planning import checks

    assert checks.assert_not_ready_for_full_path_following(summary) is summary
    summary["ready_for_full_path_following"] = True
    with pytest.raises(FullPathFollowingNotAllowed):
        checks.assert_not_ready_for_full_path_following(summary)


def test_write_preview_png_does_not_raise_on_uncalibrated_plan(tmp_path) -> None:
    """보정 없는 계획의 PNG 렌더링은 예외 없이 Path 또는 None(우아한 저하)을 낸다 /
    rendering the no-calibration plan never raises; it returns a Path (mpl
    present) or ``None`` (mpl absent)."""
    # Rendering the no-calibration plan must not raise; it returns a Path when
    # matplotlib is present or None when it is not (graceful degradation).
    summary = preview.build_preview(
        start_lat=35.0,
        start_lon=129.0,
        goal_mode="absolute",
        goal_lat=35.00008,
        goal_lon=129.00008,
        workspace_width_m=2.0,
    )
    out = preview.write_preview_png(
        tmp_path / "preview.png",
        summary["segments"],
        35.0,
        129.0,
        float(summary["goal_lat"]),
        float(summary["goal_lon"]),
        summary["workspace"],
    )
    assert out is None or out.exists()
