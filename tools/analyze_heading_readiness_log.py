from __future__ import annotations

import argparse
import math
from collections import Counter
from pathlib import Path
from typing import Sequence

try:
    from tools import _bootstrap  # type: ignore  # noqa: F401
except ImportError:
    import _bootstrap  # type: ignore  # noqa: F401

from gps_coverage_core.geo import GeoPoint, latlon_to_local

ALLOWED_MOTOR_BLOCK_REASONS = {"COMPILE_GATE_OFF", "MOTOR_OUTPUT_DISABLED"}
HEADING_MOVEMENT_BBOX_MIN_M = 10.0
HEADING_MOVEMENT_PATH_MIN_M = 20.0


def parse_usbdbg_rows(text: str) -> list[dict[str, str]]:
    rows: list[dict[str, str]] = []
    for line in text.splitlines():
        if not line.startswith("USBDBG "):
            continue
        row: dict[str, str] = {}
        for token in line.split():
            if "=" in token:
                key, value = token.split("=", 1)
                row[key] = value
        rows.append(row)
    return rows


def _float_values(rows: Sequence[dict[str, str]], field: str) -> list[float]:
    values: list[float] = []
    for row in rows:
        value = row.get(field)
        if not value or value == "NA":
            continue
        try:
            values.append(float(value))
        except ValueError:
            continue
    return values


def _counter(rows: Sequence[dict[str, str]], field: str) -> Counter[str]:
    return Counter(row.get(field, "MISSING") for row in rows)


def _true_count(rows: Sequence[dict[str, str]], field: str) -> int:
    return sum(1 for row in rows if row.get(field) == "true")


def _any_true(rows: Sequence[dict[str, str]], field: str) -> bool:
    return _true_count(rows, field) > 0


def _range_text(values: Sequence[float]) -> str:
    if not values:
        return "NA"
    return f"{min(values):.3f}..{max(values):.3f}"


def _movement_summary(rows: Sequence[dict[str, str]]) -> dict[str, float | int | str]:
    points: list[GeoPoint] = []
    for row in rows:
        try:
            points.append(GeoPoint(lat=float(row["current_lat"]), lon=float(row["current_lon"])))
        except (KeyError, ValueError):
            continue
    if not points:
        return {
            "point_count": 0,
            "bbox_x_span_m": 0.0,
            "bbox_y_span_m": 0.0,
            "bbox_diag_m": 0.0,
            "cumulative_path_m": 0.0,
            "max_step_m": 0.0,
            "moved_enough": "false",
        }

    origin = points[0]
    local_points = [latlon_to_local(origin, point) for point in points]
    xs = [point.x_m for point in local_points]
    ys = [point.y_m for point in local_points]
    bbox_x_span_m = max(xs) - min(xs)
    bbox_y_span_m = max(ys) - min(ys)
    bbox_diag_m = math.hypot(bbox_x_span_m, bbox_y_span_m)
    steps = [
        math.hypot(curr.x_m - prev.x_m, curr.y_m - prev.y_m)
        for prev, curr in zip(local_points, local_points[1:])
    ]
    cumulative_path_m = sum(steps)
    max_step_m = max(steps) if steps else 0.0
    return {
        "point_count": len(points),
        "bbox_x_span_m": bbox_x_span_m,
        "bbox_y_span_m": bbox_y_span_m,
        "bbox_diag_m": bbox_diag_m,
        "cumulative_path_m": cumulative_path_m,
        "max_step_m": max_step_m,
        "moved_enough": (
            "true"
            if bbox_diag_m >= HEADING_MOVEMENT_BBOX_MIN_M
            or cumulative_path_m >= HEADING_MOVEMENT_PATH_MIN_M
            else "false"
        ),
    }


def _reset_like_events(course_displacements: Sequence[float]) -> list[tuple[int, float, float]]:
    events: list[tuple[int, float, float]] = []
    for index, (previous, current) in enumerate(
        zip(course_displacements, course_displacements[1:]), start=1
    ):
        if previous >= 1.5 and current <= 0.25:
            events.append((index, previous, current))
    return events


def _numeric_nonzero(values: Sequence[float], epsilon: float = 1e-6) -> bool:
    return any(abs(value) > epsilon for value in values)


def _motor_safety_ok(rows: Sequence[dict[str, str]]) -> bool:
    if not rows:
        return False
    if any(row.get("physical_output_active") == "true" for row in rows):
        return False

    left_values = _float_values(rows, "final_left_cmd")
    right_values = _float_values(rows, "final_right_cmd")
    if _numeric_nonzero(left_values) or _numeric_nonzero(right_values):
        return False

    block_reasons = {
        row["physical_block_reason"] for row in rows if "physical_block_reason" in row
    }
    return bool(block_reasons) and block_reasons.issubset(ALLOWED_MOTOR_BLOCK_REASONS)


def _gps_quality_ok(rows: Sequence[dict[str, str]]) -> bool:
    if not rows:
        return False
    gps_reasons = {row.get("gps_block_reason") for row in rows if "gps_block_reason" in row}
    if not gps_reasons or any(reason != "OK" for reason in gps_reasons):
        return False

    sats_values = _float_values(rows, "gps_sats")
    hdop_values = _float_values(rows, "gps_hdop")
    if sats_values and max(sats_values) < 4:
        return False
    if hdop_values and min(hdop_values) > 6.0:
        return False
    return True


def _bmi160_valid(row: dict[str, str]) -> bool:
    if row.get("imu_type") != "BMI160":
        return False
    if row.get("imu_present") == "false":
        return False
    chip_id = row.get("imu_chip_id")
    if chip_id not in {None, "0xD1"}:
        return False
    if row.get("imu_pmu_normal") == "false":
        return False
    if row.get("imu_data_plausible") == "false":
        return False
    return True


def _imu_ok(rows: Sequence[dict[str, str]]) -> bool:
    if not rows:
        return False
    return any(_bmi160_valid(row) for row in rows)


def _rc_manual_ok(rows: Sequence[dict[str, str]]) -> bool:
    return any(
        row.get("rc_ok") == "true"
        and row.get("mode") == "MANUAL"
        and row.get("control_source") == "RC_MANUAL"
        for row in rows
    )


def _heading_ready_count(rows: Sequence[dict[str, str]]) -> int:
    return _true_count(rows, "heading_ready")


def _accepted_course_count(rows: Sequence[dict[str, str]]) -> int:
    accepted_by_reset = sum(
        1 for row in rows if row.get("gps_course_anchor_reset_reason") == "COURSE_ACCEPTED_RESEED"
    )
    accepted_by_latch = _true_count(rows, "gps_course_last_accepted_valid")
    return max(accepted_by_reset, accepted_by_latch)


def _heading_diag_partial_count(rows: Sequence[dict[str, str]]) -> int:
    return sum(
        1
        for row in rows
        if row.get("gps_course_last_accepted_valid") == "true"
        and row.get("heading_agreement_error_deg") not in {None, "NA"}
    )


def _course_latch_available_count(rows: Sequence[dict[str, str]]) -> int:
    return max(
        _true_count(rows, "gps_course_output_valid"),
        len(_float_values(rows, "gps_course_output_deg")),
    )


def _sensor_baseline_fail_reason(rows: Sequence[dict[str, str]]) -> str:
    if not _gps_quality_ok(rows):
        return "GPS_QUALITY_FAIL"
    if not _imu_ok(rows):
        return "IMU_FAIL"
    if not _rc_manual_ok(rows):
        return "RC_MANUAL_FAIL"
    return "OK"


def _verdict(rows: Sequence[dict[str, str]], movement: dict[str, float | int | str]) -> str:
    if not _motor_safety_ok(rows):
        return "MOTOR_SAFETY_FAIL"
    if _sensor_baseline_fail_reason(rows) != "OK":
        return "SENSOR_BASELINE_FAIL"
    if _heading_ready_count(rows) > 0:
        return "HEADING_READY_PASS"
    if _course_latch_available_count(rows) > 0:
        return "HEADING_COURSE_LATCH_AVAILABLE"
    if _accepted_course_count(rows) > 0:
        return "HEADING_COURSE_ACCEPTED_BUT_NOT_OUTPUT"
    if movement["moved_enough"] == "true":
        return "HEADING_NOT_READY_BUT_MOVEMENT_SUFFICIENT"
    return "SENSOR_BASELINE_PASS"


def _next_action(verdict: str) -> str:
    if verdict == "MOTOR_SAFETY_FAIL":
        return "ABORT_SAFETY_INSPECT_NO_MOTION_GATES"
    if verdict == "SENSOR_BASELINE_FAIL":
        return "FIX_SENSOR_OR_RC_BASELINE_BEFORE_HEADING"
    if verdict == "HEADING_READY_PASS":
        return "PROCEED_TO_OFFLINE_PATH_PLANNING_PREVIEW_ONLY"
    if verdict == "HEADING_COURSE_LATCH_AVAILABLE":
        return "REVIEW_HEADING_AGREEMENT_AND_PROCEED_TO_OFFLINE_PREVIEW_ONLY"
    if verdict == "HEADING_COURSE_ACCEPTED_BUT_NOT_OUTPUT":
        return "REVIEW_COURSE_OUTPUT_BLOCK_REASON"
    if verdict == "HEADING_NOT_READY_BUT_MOVEMENT_SUFFICIENT":
        return "USE_NEW_DIAGNOSTIC_FIRMWARE_AND_REPEAT_HAND_CARRY"
    return "REPEAT_HAND_CARRY_WITH_LONGER_STRAIGHT_SEGMENT"


def summarize(rows: Sequence[dict[str, str]], total_line_count: int | None = None) -> list[str]:
    lines: list[str] = []
    if total_line_count is not None:
        lines.append(f"total_line_count={total_line_count}")
    lines.append(f"usbdbg_rows={len(rows)}")
    for field in (
        "gps_block_reason",
        "position_source",
        "heading_agreement_diag",
        "path_following_block_reason",
        "gps_course_anchor_reset_reason",
        "gps_course_block_reason",
        "gps_course_output_block_reason",
        "imu_type",
        "imu_present",
        "imu_i2c_addr",
        "imu_chip_id",
        "imu_pmu_normal",
        "imu_data_plausible",
        "physical_block_reason",
        "physical_output_active",
        "mode",
        "control_source",
        "rc_ok",
        "neutral_ok",
    ):
        lines.append(f"{field}_counts={dict(_counter(rows, field))}")

    for field in (
        "gps_sats",
        "gps_hdop",
        "course_displacement_m",
        "estimated_course_deg",
        "gps_course_candidate_deg",
        "gps_course_last_accepted_deg",
        "gps_course_last_accepted_age_ms",
        "gps_course_last_accepted_displacement_m",
        "gps_course_deg",
        "gps_course_output_deg",
        "gps_course_output_age_ms",
        "heading_agreement_error_deg",
        "imu_accel_mag_g",
        "imu_gyro_mag_dps",
        "final_left_cmd",
        "final_right_cmd",
    ):
        values = _float_values(rows, field)
        lines.append(f"{field}_range={_range_text(values)} count={len(values)}")

    course_values = _float_values(rows, "course_displacement_m")
    estimated_values = _float_values(rows, "estimated_course_deg")
    unique_estimated = sorted({round(value, 1) for value in estimated_values})
    lines.append(f"max_course_displacement_m={max(course_values):.3f}" if course_values else "max_course_displacement_m=NA")
    lines.append(f"estimated_course_deg_unique_sample_count={len(unique_estimated)}")
    if unique_estimated:
        preview = ",".join(f"{value:.1f}" for value in unique_estimated[:16])
        lines.append(f"estimated_course_deg_unique_preview={preview}")

    movement = _movement_summary(rows)
    for key, value in movement.items():
        if isinstance(value, float):
            lines.append(f"{key}={value:.3f}")
        else:
            lines.append(f"{key}={value}")

    course_displacements = _float_values(rows, "course_displacement_m")
    reset_events = _reset_like_events(course_displacements)
    lines.append(f"reset_like_event_count={len(reset_events)}")
    if reset_events:
        preview = ", ".join(f"idx{idx}:{prev:.2f}->{cur:.2f}" for idx, prev, cur in reset_events[:12])
        lines.append(f"reset_like_events_preview={preview}")

    gps_course_count = len(_float_values(rows, "gps_course_deg"))
    gps_course_output_count = len(_float_values(rows, "gps_course_output_deg"))
    course_latch_available_count = _course_latch_available_count(rows)
    accepted_course_count = _accepted_course_count(rows)
    heading_ready_true_count = _true_count(rows, "heading_ready")
    lines.append(f"gps_course_deg_non_na_count={gps_course_count}")
    lines.append(f"gps_course_output_deg_non_na_count={gps_course_output_count}")
    lines.append(f"gps_course_output_valid_true_count={_true_count(rows, 'gps_course_output_valid')}")
    lines.append(f"course_latch_available_count={course_latch_available_count}")
    lines.append(f"accepted_course_count={accepted_course_count}")
    lines.append(
        "accepted_course_without_output="
        f"{str(accepted_course_count > 0 and course_latch_available_count == 0).lower()}"
    )
    lines.append(f"heading_ready_true_count={heading_ready_true_count}")
    lines.append(f"baseline_fail_reason={_sensor_baseline_fail_reason(rows)}")
    lines.append(f"bmi160_valid={str(_imu_ok(rows)).lower()}")
    lines.append(f"rc_manual_ok={str(_rc_manual_ok(rows)).lower()}")
    lines.append(f"motor_safety_ok={str(_motor_safety_ok(rows)).lower()}")
    lines.append(f"gps_quality_ok={str(_gps_quality_ok(rows)).lower()}")
    verdict = _verdict(rows, movement)
    lines.append(f"verdict={verdict}")
    lines.append(f"next_action={_next_action(verdict)}")
    if gps_course_count == 0 and course_displacements:
        lines.append(
            "likely_block_reason=course heading is computed only briefly when the segment crosses "
            "the displacement threshold; the anchor then reseeds and the debug sample sees NO_HEADING"
        )
    return lines


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Analyze integrated no-motion heading readiness logs without touching hardware."
    )
    parser.add_argument("log", type=Path)
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    text = args.log.read_text(encoding="utf-8", errors="ignore")
    rows = parse_usbdbg_rows(text)
    for line in summarize(rows, total_line_count=len(text.splitlines())):
        print(line)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
