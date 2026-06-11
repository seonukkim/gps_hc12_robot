"""No-motion path preview: build a coverage/direct plan and render it.

``build_preview`` composes the geometry (axis-aligned ㄹ/lawnmower coverage,
explicit A->B-diagonal rectangle coverage, or a direct line) with a resolved
calibration and returns a pure summary dict -- no serial, no firmware, no motion.
It is guaranteed to succeed when the turn-angle calibration is MISSING: the
resolver falls back to repeated-pulse connectors
(``fallback_to_repeated_pulses=True``) and preview still produces a full plan.
Every summary is routed through the readiness guard so it can never claim
``ready_for_full_path_following``.

``write_preview_png`` renders the segments to a PNG; it returns ``None`` when
matplotlib is unavailable. CLI preview modes treat that as a clear failure
because field operation requires visual inspection.
"""
from __future__ import annotations

from pathlib import Path
from typing import Sequence

from tools.physical_path_planning import calibration as calibration_resolver
from tools.physical_path_planning import checks, geometry


def resolve_preview_calibration(
    *,
    motion_calibration_json: Path | None = None,
    fine_calibration_json: Path | None = None,
    turn_calibration_json: Path | None = None,
    turn_angle_calibration_json: Path | None = None,
    smooth_turn_calibration_json: Path | None = None,
    calibration_mode: str = "auto",
) -> dict[str, object]:
    """Resolve calibration for preview, defaulting every source to absent.

    Unlike the resolver's own defaults (which point at on-disk calibration
    files), this defaults all sources to ``None`` so preview never depends on
    calibration existing -- a missing turn-angle file simply yields the
    repeated-pulses fallback instead of an error.
    """
    return calibration_resolver.resolve_physical_calibration(
        motion_calibration_json=motion_calibration_json,
        fine_calibration_json=fine_calibration_json,
        turn_calibration_json=turn_calibration_json,
        turn_angle_calibration_json=turn_angle_calibration_json,
        smooth_turn_calibration_json=smooth_turn_calibration_json,
        calibration_mode=calibration_mode,
    )


def build_preview(
    *,
    start_lat: float,
    start_lon: float,
    goal_mode: str,
    goal_lat: float | None = None,
    goal_lon: float | None = None,
    goal_east_m: float | None = None,
    goal_north_m: float | None = None,
    goal_dlat: float | None = None,
    goal_dlon: float | None = None,
    goal_bearing_deg: float | None = None,
    goal_distance_m: float | None = None,
    path_shape: str = geometry.COVERAGE_LAWNMOWER,
    workspace_width_m: float | None = None,
    step_spacing_m: float = 0.5,
    diagonal_orientation: str = "A_top_left_to_B_bottom_right",
    max_segment_pulses: int = 8,
    nominal_forward_pulse_m: float = 0.30,
    calibration: dict[str, object] | None = None,
    connector_style: str = geometry.DEFAULT_CONNECTOR_STYLE,
) -> dict[str, object]:
    """Build a no-motion preview summary (segments + plan metadata).

    ``calibration`` may be a resolved calibration dict; when omitted it defaults
    to the missing-everything fallback so preview works with no calibration at
    all. ``path_shape='direct_line'`` builds the single-line escape hatch.
    ``coverage_lawnmower`` is the default ㄹ/lawnmower sweep in local ENU.
    ``diagonal_rectangle_serpentine`` is the explicit A->B-diagonal frame.
    """
    resolved = calibration if calibration is not None else geometry.FALLBACK_RESOLVED_CALIBRATION
    path_shape = geometry.canonical_path_shape(path_shape)

    goal_point, goal_context = geometry.resolve_goal_point(
        start_lat=start_lat,
        start_lon=start_lon,
        goal_mode=goal_mode,
        goal_lat=goal_lat,
        goal_lon=goal_lon,
        goal_east_m=goal_east_m,
        goal_north_m=goal_north_m,
        goal_dlat=goal_dlat,
        goal_dlon=goal_dlon,
        goal_bearing_deg=goal_bearing_deg,
        goal_distance_m=goal_distance_m,
    )
    goal_lat = goal_point.lat
    goal_lon = goal_point.lon

    workspace: dict[str, object] | None = None
    if path_shape == geometry.DIRECT_LINE:
        segments, primitives, distance = geometry.build_direct_segments(
            start_lat=start_lat,
            start_lon=start_lon,
            goal_lat=goal_lat,
            goal_lon=goal_lon,
            max_segment_pulses=max_segment_pulses,
            nominal_forward_pulse_m=nominal_forward_pulse_m,
            calibration=resolved,
        )
        path_points = geometry.planned_path_points(
            segments, geometry.GeoPoint(start_lat, start_lon)
        )
    elif path_shape == geometry.COVERAGE_LAWNMOWER:
        if workspace_width_m is None:
            raise ValueError("workspace width is required because A-B defines the coverage field")
        workspace = geometry.build_axis_aligned_lawnmower_workspace(
            start_lat=start_lat,
            start_lon=start_lon,
            goal_lat=goal_lat,
            goal_lon=goal_lon,
            width_m=workspace_width_m,
            step_spacing_m=step_spacing_m,
        )
        segments, primitives, path_points = geometry.build_serpentine_segments(
            workspace,
            max_segment_pulses=max_segment_pulses,
            nominal_forward_pulse_m=nominal_forward_pulse_m,
            calibration=resolved,
            connector_style=connector_style,
        )
        distance = float(workspace["diagonal_length_m"])
    elif path_shape == geometry.DIAGONAL_RECTANGLE_SERPENTINE:
        if workspace_width_m is None:
            raise ValueError(
                "workspace width is required because A-B is a diagonal, not a direct path"
            )
        workspace = geometry.build_rectangle_from_diagonal_and_width(
            start_lat=start_lat,
            start_lon=start_lon,
            goal_lat=goal_lat,
            goal_lon=goal_lon,
            width_m=workspace_width_m,
            step_spacing_m=step_spacing_m,
            diagonal_orientation=diagonal_orientation,
        )
        segments, primitives, path_points = geometry.build_serpentine_segments(
            workspace,
            max_segment_pulses=max_segment_pulses,
            nominal_forward_pulse_m=nominal_forward_pulse_m,
            calibration=resolved,
            connector_style=connector_style,
        )
        distance = float(workspace["diagonal_length_m"])
    else:
        raise ValueError(f"unsupported path_shape: {path_shape}")

    # lane_count counts FULL coverage lanes; step-over lanes and pivot turns are
    # reported separately. connector_count keeps its historical meaning of
    # lane-to-lane transitions (one ㄹ corner), whatever style implements them.
    lane_count = len(
        [seg for seg in segments if str(seg["segment_type"]) in geometry.FULL_LANE_SEGMENT_TYPES]
    )
    step_lane_count = len(
        [seg for seg in segments if str(seg["segment_type"]) == "step_lane"]
    )
    connector_turn_count = len(
        [seg for seg in segments if str(seg["segment_type"]) in geometry.CONNECTOR_SEGMENT_TYPES]
    )
    connector_count = max(0, lane_count - 1) if path_shape != geometry.DIRECT_LINE else 0
    summary: dict[str, object] = {
        "planner_mode": "preview",
        "preview_only": True,
        "path_shape": path_shape,
        "goal_mode": goal_mode,
        "goal_input": goal_context,
        "start_lat": start_lat,
        "start_lon": start_lon,
        "goal_lat": goal_lat,
        "goal_lon": goal_lon,
        "goal_distance_m": distance,
        "workspace_width_m": workspace.get("workspace_width_m") if workspace else None,
        "workspace_length_m": workspace.get("workspace_length_m") if workspace else None,
        "diagonal_length_m": workspace.get("diagonal_length_m") if workspace else distance,
        "step_spacing_m": workspace.get("step_spacing_m") if workspace else step_spacing_m,
        "segment_count": len(segments),
        "primitive_count": len(primitives),
        "lane_count": lane_count,
        "connector_count": connector_count,
        "connector_turn_count": connector_turn_count,
        "step_lane_count": step_lane_count,
        "connector_style": (
            connector_style if path_shape != geometry.DIRECT_LINE else "none"
        ),
        "coverage_area_estimate_m2": workspace.get("coverage_area_estimate_m2") if workspace else None,
        "expected_sweep_style": workspace.get("expected_sweep_style") if workspace else "direct_line",
        "segments": segments,
        "primitives": primitives,
        "path_points": path_points,
        "workspace": workspace,
        "connector_mode_effective": resolved["connector_mode_effective"],
        "connector_mode_recommended": resolved["connector_mode_recommended"],
        "forward_source": resolved["forward"]["source"],  # type: ignore[index]
        "backward_source": resolved["backward"]["source"],  # type: ignore[index]
        "turn_left_source": resolved["turn_left"]["source"],  # type: ignore[index]
        "turn_right_source": resolved["turn_right"]["source"],  # type: ignore[index]
        "angle_based_connector_available": resolved["angle_based_connector_available"],
        "smooth_connector_available": resolved["smooth_connector_available"],
        "fallback_to_repeated_pulses": resolved["fallback_to_repeated_pulses"],
        "ready_for_full_path_following": False,
    }
    return checks.assert_not_ready_for_full_path_following(summary)


def write_preview_png(
    path: Path,
    segments: Sequence[dict[str, object]],
    start_lat: float,
    start_lon: float,
    goal_lat: float,
    goal_lon: float,
    workspace: dict[str, object] | None = None,
    *,
    path_shape: str = geometry.COVERAGE_LAWNMOWER,
) -> Path | None:
    try:
        import matplotlib

        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError:
        return None
    fig, ax = plt.subplots(figsize=(7, 5))
    if workspace is not None:
        a = workspace["A_corner"]  # type: ignore[index]
        b = workspace["B_corner"]  # type: ignore[index]
        c = workspace["C_corner"]  # type: ignore[index]
        d = workspace["D_corner"]  # type: ignore[index]
        boundary_x = [float(a["x_m"]), float(c["x_m"]), float(b["x_m"]), float(d["x_m"]), float(a["x_m"])]  # type: ignore[index]
        boundary_y = [float(a["y_m"]), float(c["y_m"]), float(b["y_m"]), float(d["y_m"]), float(a["y_m"])]  # type: ignore[index]
        ax.plot(boundary_x, boundary_y, color="black", linewidth=1.5, label="coverage rectangle")
        ax.plot([float(a["x_m"]), float(b["x_m"])], [float(a["y_m"]), float(b["y_m"])], color="tab:orange", linestyle="--", label="A-B diagonal")  # type: ignore[index]
        ax.annotate("A/start", (float(a["x_m"]), float(a["y_m"])), textcoords="offset points", xytext=(8, 8), color="green")  # type: ignore[index]
        ax.annotate("B/goal", (float(b["x_m"]), float(b["y_m"])), textcoords="offset points", xytext=(8, -14), color="red")  # type: ignore[index]
        ax.annotate(
            f"width={float(workspace['workspace_width_m']):.2f}m",  # type: ignore[index]
            ((float(a["x_m"]) + float(d["x_m"])) / 2.0, (float(a["y_m"]) + float(d["y_m"])) / 2.0),  # type: ignore[index]
            color="tab:purple",
        )
        ax.annotate(
            f"step={float(workspace['step_spacing_m']):.2f}m",  # type: ignore[index]
            ((float(a["x_m"]) + float(c["x_m"])) / 2.0, (float(a["y_m"]) + float(c["y_m"])) / 2.0),  # type: ignore[index]
            color="tab:brown",
        )
    for segment in segments:
        xs = [float(segment["start_x_m"]), float(segment["end_x_m"])]
        ys = [float(segment["start_y_m"]), float(segment["end_y_m"])]
        is_connector = str(segment["segment_type"]) in {"connector_turn", "path_connector"}
        style = "--" if is_connector else "-"
        color = "tab:gray" if is_connector else "tab:blue"
        label = "connector turns" if is_connector else "lane lines"
        ax.plot(xs, ys, linestyle=style, linewidth=2.0, marker="o", color=color, label=label)
        mid_x = sum(xs) / 2.0
        mid_y = sum(ys) / 2.0
        ax.annotate(f"seg {segment['segment_index']}", (mid_x, mid_y))
        dx = xs[1] - xs[0]
        dy = ys[1] - ys[0]
        if abs(dx) > 1e-9 or abs(dy) > 1e-9:
            ax.annotate(
                "",
                xy=(xs[0] + dx * 0.62, ys[0] + dy * 0.62),
                xytext=(xs[0] + dx * 0.38, ys[0] + dy * 0.38),
                arrowprops={"arrowstyle": "->", "color": color, "lw": 1.5},
            )
    ax.scatter([0], [0], c="green", s=80, label="current/start")
    goal_x, goal_y = geometry.goal_to_local(start_lat, start_lon, goal_lat, goal_lon)
    ax.scatter([goal_x], [goal_y], c="red", s=80, label="goal")
    ax.annotate("A/start", (0, 0), textcoords="offset points", xytext=(8, 8), color="green")
    ax.annotate("B/goal", (goal_x, goal_y), textcoords="offset points", xytext=(8, -14), color="red")
    ax.arrow(0, 0, 1.0, 0, color="tab:blue", head_width=0.08, length_includes_head=True)
    ax.arrow(0, 0, 0, 1.0, color="tab:cyan", head_width=0.08, length_includes_head=True)
    ax.text(1.05, 0, "East +X", color="tab:blue")
    ax.text(0, 1.05, "North +Y", color="tab:cyan")
    ax.text(
        0.01,
        0.95,
        f"path_shape={path_shape}\nlane lines: solid\nconnector turns: dashed",
        transform=ax.transAxes,
        fontsize=8,
        va="top",
    )
    ax.set_title("Physical Path Planning Coverage Preview")
    ax.set_xlabel("East from start (m)")
    ax.set_ylabel("North from start (m)")
    ax.grid(True, linestyle="--", alpha=0.4)
    ax.axis("equal")
    handles, labels = ax.get_legend_handles_labels()
    deduped = dict(zip(labels, handles))
    ax.legend(deduped.values(), deduped.keys())
    ax.text(0.01, 0.01, f"start={start_lat:.7f},{start_lon:.7f}\ngoal={goal_lat:.7f},{goal_lon:.7f}", transform=ax.transAxes, fontsize=8, va="bottom")
    fig.tight_layout()
    fig.savefig(path, dpi=160)
    plt.close(fig)
    return path
