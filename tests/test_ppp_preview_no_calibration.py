"""Preview must work with MISSING turn-angle calibration.

The guarantee under test: a field operator can preview an A->B serpentine plan
before any turn-angle calibration exists. The resolver falls back to repeated
pulses (``fallback_to_repeated_pulses=True``) and preview still emits a full,
guarded (never-ready) plan -- it must not raise or block on missing calibration.
"""
from __future__ import annotations

import pytest

from tools.physical_path_planning import preview
from tools.physical_path_planning.checks import FullPathFollowingNotAllowed


def test_preview_builds_with_no_calibration_at_all() -> None:
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
    with pytest.raises(ValueError, match="workspace width is required"):
        preview.build_preview(
            start_lat=35.0,
            start_lon=129.0,
            goal_mode="absolute",
            goal_lat=35.00008,
            goal_lon=129.00008,
        )


def test_preview_summary_is_guarded_not_ready() -> None:
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
