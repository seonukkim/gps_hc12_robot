from __future__ import annotations

import argparse
import csv
import json
import math
import time
from pathlib import Path
from typing import Sequence

try:
    from tools import _bootstrap  # type: ignore  # noqa: F401
except ImportError:
    import _bootstrap  # type: ignore  # noqa: F401

from legacy.stage_tools import stage26_calibrated_primitive_sequence as stage26
from tools import station_path_package_tracker


SENSOR_MOTION_FIELDS = (
    "primitive_index",
    "primitive_name",
    "a_cmd",
    "b_cmd",
    "pulse_ms",
    "gps_block_reason_before",
    "gps_block_reason_after",
    "position_source_before",
    "position_source_after",
    "current_lat_before",
    "current_lon_before",
    "current_lat_after",
    "current_lon_after",
    "imu_type_before",
    "imu_type_after",
    "imu_calibrated_before",
    "imu_calibrated_after",
    "imu_heading_block_reason_before",
    "imu_heading_block_reason_after",
    "imu_yaw_before_deg",
    "imu_yaw_after_deg",
    "imu_yaw_delta_deg",
    "arm_seen",
    "ack_seen",
    "stop_seen",
    "reject_seen",
    "reject_reason",
    "final_left_cmd",
    "final_right_cmd",
    "final_left_cmd_zero",
    "final_right_cmd_zero",
    "physical_output_active_after_stop",
    "user_motion_report",
    "user_turn_angle_report",
    "valid_primitive",
    "invalid_reason",
    "ready_for_full_path_following",
)


def _parse_bool(value: object, default: bool = False) -> bool:
    if value is None:
        return default
    return str(value).strip().lower() in {"1", "true", "yes", "y", "ok", "active"}


def _optional_float(value: object) -> float | None:
    if value is None:
        return None
    text = str(value).strip()
    if text.upper() in {"", "NA", "NAN", "NONE", "NULL"}:
        return None
    try:
        parsed = float(text)
    except ValueError:
        return None
    return parsed if math.isfinite(parsed) else None


def unwrap_delta(final_yaw: float | None, before_yaw: float | None) -> float | str:
    if final_yaw is None or before_yaw is None:
        return ""
    delta = final_yaw - before_yaw
    while delta > 180.0:
        delta -= 360.0
    while delta < -180.0:
        delta += 360.0
    return delta


def _latest(rows: Sequence[dict[str, str]], key: str, default: str = "NA") -> str:
    for row in reversed(rows):
        if key in row:
            return row[key]
    return default


def _serial_rows(raw_lines: Sequence[str], start_index: int = 0) -> list[dict[str, str]]:
    return station_path_package_tracker.parse_usbdbg_rows("\n".join(raw_lines[start_index:]))


def _wait_for_event(handle: object, raw_lines: list[str], wanted: set[str], timeout_s: float) -> list[dict[str, str]]:
    deadline = time.monotonic() + timeout_s
    start_index = len(raw_lines)
    while time.monotonic() < deadline:
        raw = handle.readline()
        if raw:
            line = raw.decode("utf-8", errors="replace").strip()
            print(line)
            raw_lines.append(line)
            rows = _serial_rows(raw_lines, start_index)
            if any(row.get("event", "").upper() in wanted for row in rows):
                return station_path_package_tracker.parse_usbdbg_rows("\n".join(raw_lines))
    return station_path_package_tracker.parse_usbdbg_rows("\n".join(raw_lines))


def _wait_for_precondition(handle: object, raw_lines: list[str], timeout_s: float) -> list[dict[str, str]]:
    return stage26._wait_for_precondition(handle, raw_lines, timeout_s)  # type: ignore[attr-defined]


def sensor_row_from_primitive(
    *,
    planned: dict[str, object],
    rows: Sequence[dict[str, str]],
    user_motion_report: str,
    user_turn_angle_report: str,
) -> dict[str, object]:
    base = stage26.row_from_primitive(
        planned=planned,
        rows=rows,
        user_motion_report=user_motion_report,
        user_turn_angle_report=user_turn_angle_report,
    )
    before = rows[0] if rows else {}
    after = rows[-1] if rows else {}
    before_yaw = _optional_float(before.get("imu_relative_yaw_deg"))
    after_yaw = _optional_float(_latest(rows, "imu_relative_yaw_deg", "NA"))
    return {
        "primitive_index": planned["primitive_index"],
        "primitive_name": planned["primitive_name"],
        "a_cmd": planned["a_cmd"],
        "b_cmd": planned["b_cmd"],
        "pulse_ms": planned["pulse_ms"],
        "gps_block_reason_before": before.get("gps_block_reason", "NA"),
        "gps_block_reason_after": _latest(rows, "gps_block_reason", "NA"),
        "position_source_before": before.get("position_source", "NA"),
        "position_source_after": _latest(rows, "position_source", "NA"),
        "current_lat_before": before.get("current_lat", "NA"),
        "current_lon_before": before.get("current_lon", "NA"),
        "current_lat_after": _latest(rows, "current_lat", "NA"),
        "current_lon_after": _latest(rows, "current_lon", "NA"),
        "imu_type_before": before.get("imu_type", "NA"),
        "imu_type_after": _latest(rows, "imu_type", "NA"),
        "imu_calibrated_before": before.get("imu_calibrated", "NA"),
        "imu_calibrated_after": _latest(rows, "imu_calibrated", "NA"),
        "imu_heading_block_reason_before": before.get("imu_heading_block_reason", "NA"),
        "imu_heading_block_reason_after": _latest(rows, "imu_heading_block_reason", "NA"),
        "imu_yaw_before_deg": "" if before_yaw is None else before_yaw,
        "imu_yaw_after_deg": "" if after_yaw is None else after_yaw,
        "imu_yaw_delta_deg": unwrap_delta(after_yaw, before_yaw),
        "arm_seen": base["arm_seen"],
        "ack_seen": base["ack_seen"],
        "stop_seen": base["stop_seen"],
        "reject_seen": base["reject_seen"],
        "reject_reason": base["reject_reason"],
        "final_left_cmd": base["final_left_cmd"],
        "final_right_cmd": base["final_right_cmd"],
        "final_left_cmd_zero": base["final_left_cmd_zero"],
        "final_right_cmd_zero": base["final_right_cmd_zero"],
        "physical_output_active_after_stop": base["physical_output_active_after_stop"],
        "user_motion_report": user_motion_report,
        "user_turn_angle_report": user_turn_angle_report,
        "valid_primitive": base["valid_primitive"],
        "invalid_reason": base["invalid_reason"],
        "ready_for_full_path_following": False,
    }


def write_csv(path: Path, rows: Sequence[dict[str, object]]) -> None:
    with path.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=SENSOR_MOTION_FIELDS)
        writer.writeheader()
        for row in rows:
            writer.writerow({field: row.get(field, "") for field in SENSOR_MOTION_FIELDS})


def build_summary(rows: Sequence[dict[str, object]], *, sequence: str, primitive_count: int, dry_run: bool) -> dict[str, object]:
    valid_count = sum(1 for row in rows if row.get("valid_primitive") is True)
    reject_count = sum(1 for row in rows if row.get("reject_seen") is True)
    final = rows[-1] if rows else {}
    return {
        "sequence": sequence,
        "dry_run": dry_run,
        "primitive_count": primitive_count,
        "valid_primitive_count": valid_count,
        "reject_count": reject_count,
        "final_stop_confirmed": bool(rows) and final.get("stop_seen") is True and final.get("final_left_cmd_zero") is True and final.get("final_right_cmd_zero") is True,
        "used_calibrated_thresholds": True,
        "ready_for_stage28d_closed_loop": (not dry_run and valid_count == primitive_count and reject_count == 0),
        "ready_for_full_path_following": False,
    }


def write_summary(path: Path, summary: dict[str, object]) -> None:
    lines = ["# Stage 28C Calibrated Motion With Sensors", "", "Calibrated Stage20 pulses with GPS/IMU logging. Not closed-loop.", ""]
    lines.extend(f"- {key}: `{value}`" for key, value in summary.items())
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def _prompt_choice(prompt: str, choices: set[str]) -> str:
    while True:
        answer = input(prompt).strip().lower()
        if answer in choices:
            return answer
        print("Use one of: " + ", ".join(sorted(choices)))


def run(args: argparse.Namespace) -> int:
    fine = json.loads(Path(args.fine_calibration_json).read_text(encoding="utf-8"))
    turn = json.loads(Path(args.turn_calibration_json).read_text(encoding="utf-8"))
    primitive_map = stage26.build_primitive_map(fine, turn)
    plan = stage26.build_plan(args.sequence, primitive_map, max_abs_a=args.max_abs_a, max_abs_b=args.max_abs_b, max_ms=args.max_ms)
    out_dir = Path(args.out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    stage26.write_primitive_sequence(out_dir / "primitive_sequence_used.json", plan)
    dry_run = _parse_bool(args.dry_run, default=False)
    if dry_run:
        write_summary(out_dir / "summary.md", build_summary([], sequence=args.sequence, primitive_count=len(plan), dry_run=True))
        print("dry_run=true")
        print("ready_for_full_path_following=false")
        return 0
    try:
        import serial  # type: ignore[import-not-found]
    except ImportError as exc:
        raise RuntimeError("pyserial is not available; cannot run Stage28C") from exc
    rows: list[dict[str, object]] = []
    raw_lines: list[str] = []
    require_enter = _parse_bool(args.require_enter, default=True)
    interactive = _parse_bool(args.interactive_visible_motion, default=True)
    with serial.Serial(args.port, baudrate=115200, timeout=0.5) as handle:
        for planned in plan:
            start_index = len(raw_lines)
            precondition_rows = _wait_for_precondition(handle, raw_lines, args.heartbeat_timeout_s)
            if not stage26.stage20_precondition_ready(precondition_rows):
                raise RuntimeError("Stage28C precondition failed before primitive")
            if require_enter:
                input(f"Press Enter for Stage28C {planned['primitive_name']} #{planned['primitive_index']}: {planned['stage20_command_text']} ")
            handle.write((str(planned["arm_command_text"]) + "\n").encode("ascii"))
            handle.flush()
            arm_rows = _wait_for_event(handle, raw_lines, {"ARM", "REJECT"}, args.event_timeout_s)
            if not any(row.get("event", "").upper() == "ARM" for row in arm_rows):
                rows.append(sensor_row_from_primitive(planned=planned, rows=_serial_rows(raw_lines, start_index), user_motion_report="unknown", user_turn_angle_report="unknown"))
                break
            handle.write((str(planned["stage20_command_text"]) + "\n").encode("ascii"))
            handle.flush()
            _wait_for_event(handle, raw_lines, {"ACK", "REJECT"}, args.event_timeout_s)
            pulse_ms = int(planned["pulse_ms"])
            _wait_for_event(handle, raw_lines, {"STOP", "PULSE_COMPLETE", "PULSE_DONE"}, max(args.event_timeout_s, pulse_ms / 1000.0 + 1.0))
            handle.write((str(planned["stop_command_text"]) + "\n").encode("ascii"))
            handle.flush()
            _wait_for_event(handle, raw_lines, {"STOP"}, args.event_timeout_s)
            primitive_rows = _serial_rows(raw_lines, start_index)
            if interactive:
                motion = _prompt_choice("Observed motion? [forward/backward/left/right/twitch/none/unknown] ", stage26.MOTION_REPORTS)
                turn_report = _prompt_choice("Observed turn angle? [none/twitch/small_under_15deg/medium_15_45deg/big_over_45deg/unknown] ", stage26.TURN_REPORTS) if planned["primitive_name"] in {"turn_left", "turn_right"} else "none"
            else:
                motion = "unknown"
                turn_report = "unknown"
            row = sensor_row_from_primitive(planned=planned, rows=primitive_rows, user_motion_report=motion, user_turn_angle_report=turn_report)
            rows.append(row)
            if row["valid_primitive"] is not True:
                break
    write_csv(out_dir / "stage28_calibrated_motion_with_sensors.csv", rows)
    (out_dir / "raw_usbdbg.log").write_text("\n".join(raw_lines) + ("\n" if raw_lines else ""), encoding="utf-8")
    write_summary(out_dir / "summary.md", build_summary(rows, sequence=args.sequence, primitive_count=len(plan), dry_run=False))
    print("Stage28C calibrated motion with sensors complete.")
    print("ready_for_full_path_following=false")
    return 0


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Stage28C calibrated primitive motion with GPS/IMU logging.")
    parser.add_argument("--port", default="/dev/ttyACM0")
    parser.add_argument("--fine-calibration-json", required=True)
    parser.add_argument("--turn-calibration-json", required=True)
    parser.add_argument("--sequence", required=True)
    parser.add_argument("--require-enter", default="true")
    parser.add_argument("--interactive-visible-motion", default="true")
    parser.add_argument("--dry-run", default="false")
    parser.add_argument("--max-abs-a", type=float, default=stage26.DEFAULT_MAX_ABS_A)
    parser.add_argument("--max-abs-b", type=float, default=stage26.DEFAULT_MAX_ABS_B)
    parser.add_argument("--max-ms", type=int, default=stage26.DEFAULT_MAX_MS)
    parser.add_argument("--heartbeat-timeout-s", type=float, default=10.0)
    parser.add_argument("--event-timeout-s", type=float, default=5.0)
    parser.add_argument("--out-dir", required=True)
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    try:
        return run(build_parser().parse_args(argv))
    except RuntimeError as exc:
        print(f"reason={exc}")
        print("ready_for_full_path_following=false")
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
