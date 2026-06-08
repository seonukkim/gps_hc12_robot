from __future__ import annotations

import argparse
import csv
import json
import math
import statistics
import time
from pathlib import Path
from typing import Sequence

try:
    from tools import _bootstrap  # type: ignore  # noqa: F401
except ImportError:
    import _bootstrap  # type: ignore  # noqa: F401

from legacy.stage_tools import stage26_calibrated_primitive_sequence as stage26
from tools import station_path_package_tracker


DEFAULT_MAX_ABS_A = 0.35
DEFAULT_MAX_ABS_B = 0.35
DEFAULT_MAX_MS = 1000
MOTION_REPORTS = {"forward", "backward", "left", "right", "twitch", "none", "unknown"}
TURN_REPORTS = {"none", "twitch", "small_under_15deg", "medium_15_45deg", "big_over_45deg", "unknown"}
INVALID_REJECT_REASONS = {"RC_INVALID", "LATCHED_STOP"}

STAGE27_FIELDS = (
    "trial_index",
    "mode",
    "a_cmd",
    "b_cmd",
    "pulse_ms",
    "imu_type",
    "imu_present",
    "imu_data_plausible",
    "imu_pmu_normal",
    "imu_yaw_baseline_deg",
    "imu_yaw_final_deg",
    "imu_yaw_delta_deg",
    "imu_yaw_delta_abs_deg",
    "user_motion_report",
    "user_turn_angle_report",
    "arm_seen",
    "ack_seen",
    "stop_seen",
    "reject_seen",
    "reject_reason",
    "rc_invalid_seen",
    "final_left_cmd",
    "final_right_cmd",
    "final_left_cmd_zero",
    "final_right_cmd_zero",
    "valid_trial",
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


def _latest(rows: Sequence[dict[str, str]], key: str, default: str = "NA") -> str:
    for row in reversed(rows):
        if key in row:
            return row[key]
    return default


def _latest_reject_reason(rows: Sequence[dict[str, str]]) -> str:
    reason = _latest(rows, "stage20_reject_reason", "")
    if reason:
        return reason
    return _latest(rows, "stage16_reject_reason", "NONE")


def unwrap_yaw_delta_deg(final_yaw: float, baseline_yaw: float) -> float:
    delta = final_yaw - baseline_yaw
    while delta > 180.0:
        delta -= 360.0
    while delta < -180.0:
        delta += 360.0
    return delta


def mean_median_std(values: Sequence[float]) -> tuple[float | None, float | None, float | None]:
    if not values:
        return None, None, None
    mean = statistics.fmean(values)
    median = statistics.median(values)
    std = statistics.pstdev(values) if len(values) > 1 else 0.0
    return mean, median, std


def estimate_repeat_count(target_deg: float, mean_yaw_delta_deg: float | None) -> int | None:
    if mean_yaw_delta_deg is None or abs(mean_yaw_delta_deg) < 1e-9:
        return None
    return max(1, round(target_deg / abs(mean_yaw_delta_deg)))


def parse_sequence_for_mode(mode: str, repeat_count: int, sequence: str | None = None) -> list[str]:
    if mode in {"turn_left", "turn_right"}:
        return [mode] * repeat_count
    if mode != "sequence":
        raise ValueError(f"unsupported Stage27 mode: {mode}")
    raw_sequence = sequence or "turn_left,turn_right"
    expanded = stage26.parse_sequence(raw_sequence)
    names = [str(item["primitive_name"]) for item in expanded]
    if any(name not in {"turn_left", "turn_right"} for name in names):
        raise ValueError("Stage27 sequence mode supports only turn_left/turn_right primitives")
    return names * repeat_count


def _serial_rows(raw_lines: Sequence[str], start_index: int = 0) -> list[dict[str, str]]:
    return station_path_package_tracker.parse_usbdbg_rows("\n".join(raw_lines[start_index:]))


def _imu_ready_row(row: dict[str, str]) -> bool:
    return (
        row.get("event", "").upper() == "HEARTBEAT"
        and _parse_bool(row.get("stage20_physical_ab_guarded_crawl")) is True
        and _parse_bool(row.get("stage20_firmware_ready")) is True
        and _parse_bool(row.get("rc_ok")) is True
        and _parse_bool(row.get("neutral_ok")) is True
        and _parse_bool(row.get("physical_output_active")) is False
        and row.get("imu_type") == "BMI160"
        and _parse_bool(row.get("imu_present")) is True
        and _parse_bool(row.get("imu_pmu_normal")) is True
        and _parse_bool(row.get("imu_data_plausible")) is True
    )


def imu_precondition_ready(rows: Sequence[dict[str, str]]) -> bool:
    return any(_imu_ready_row(row) for row in rows)


def valid_imu_yaws(rows: Sequence[dict[str, str]]) -> list[float]:
    values: list[float] = []
    for row in rows:
        yaw = _optional_float(row.get("imu_relative_yaw_deg"))
        if yaw is not None:
            values.append(yaw)
    return values


def representative_yaw(rows: Sequence[dict[str, str]]) -> float | None:
    yaws = valid_imu_yaws(rows)
    return statistics.median(yaws) if yaws else None


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


def _collect_rows(handle: object, raw_lines: list[str], duration_s: float) -> list[dict[str, str]]:
    deadline = time.monotonic() + duration_s
    start_index = len(raw_lines)
    while time.monotonic() < deadline:
        raw = handle.readline()
        if raw:
            line = raw.decode("utf-8", errors="replace").strip()
            print(line)
            raw_lines.append(line)
    return _serial_rows(raw_lines, start_index)


def _wait_for_imu_precondition(handle: object, raw_lines: list[str], timeout_s: float) -> list[dict[str, str]]:
    deadline = time.monotonic() + timeout_s
    start_index = len(raw_lines)
    while time.monotonic() < deadline:
        rows = _serial_rows(raw_lines, start_index)
        if imu_precondition_ready(rows):
            return station_path_package_tracker.parse_usbdbg_rows("\n".join(raw_lines))
        raw = handle.readline()
        if raw:
            line = raw.decode("utf-8", errors="replace").strip()
            print(line)
            raw_lines.append(line)
    return station_path_package_tracker.parse_usbdbg_rows("\n".join(raw_lines))


def _prompt_choice(prompt: str, choices: set[str]) -> str:
    while True:
        answer = input(prompt).strip().lower()
        if answer in choices:
            return answer
        print("Use one of: " + ", ".join(sorted(choices)))


def planned_turn(
    *,
    trial_index: int,
    mode: str,
    primitive_map: dict[str, dict[str, object]],
    max_abs_a: float = DEFAULT_MAX_ABS_A,
    max_abs_b: float = DEFAULT_MAX_ABS_B,
    max_ms: int = DEFAULT_MAX_MS,
) -> dict[str, object]:
    if mode not in {"turn_left", "turn_right"}:
        raise ValueError("Stage27 planned turn supports turn_left/turn_right only")
    return stage26.planned_primitive(
        primitive_index=trial_index,
        primitive_name=mode,
        repeat_index=1,
        primitive_map=primitive_map,
        max_abs_a=max_abs_a,
        max_abs_b=max_abs_b,
        max_ms=max_ms,
    )


def row_from_trial(
    *,
    trial_index: int,
    mode: str,
    planned: dict[str, object],
    baseline_rows: Sequence[dict[str, str]],
    final_rows: Sequence[dict[str, str]],
    pulse_rows: Sequence[dict[str, str]],
    user_motion_report: str,
    user_turn_angle_report: str,
) -> dict[str, object]:
    all_rows = [*baseline_rows, *pulse_rows, *final_rows]
    baseline_yaw = representative_yaw(baseline_rows)
    final_yaw = representative_yaw(final_rows)
    yaw_delta = unwrap_yaw_delta_deg(final_yaw, baseline_yaw) if baseline_yaw is not None and final_yaw is not None else None
    reject_reason = _latest_reject_reason(pulse_rows)
    reject_seen = any(row.get("event", "").upper() == "REJECT" for row in pulse_rows)
    final_left = _optional_float(_latest(pulse_rows, "final_left_cmd", "0")) or 0.0
    final_right = _optional_float(_latest(pulse_rows, "final_right_cmd", "0")) or 0.0
    final_left_zero = abs(final_left) <= 1e-9
    final_right_zero = abs(final_right) <= 1e-9
    output_after_stop = any(
        row.get("event", "").upper() == "STOP" and _parse_bool(row.get("physical_output_active"))
        for row in pulse_rows
    )
    invalid_reason = "NONE"
    if _latest(all_rows, "imu_type", "NA") != "BMI160" or _parse_bool(_latest(all_rows, "imu_present", "false")) is not True:
        invalid_reason = "IMU_BMI160_ABSENT"
    elif _parse_bool(_latest(all_rows, "imu_data_plausible", "false")) is not True:
        invalid_reason = "IMU_DATA_NOT_PLAUSIBLE"
    elif _parse_bool(_latest(all_rows, "imu_pmu_normal", "false")) is not True:
        invalid_reason = "IMU_PMU_NOT_NORMAL"
    elif yaw_delta is None:
        invalid_reason = "IMU_YAW_DELTA_UNAVAILABLE"
    elif reject_seen:
        invalid_reason = reject_reason if reject_reason != "NONE" else "REJECT"
    elif reject_reason in INVALID_REJECT_REASONS:
        invalid_reason = reject_reason
    elif not any(row.get("event", "").upper() == "ACK" for row in pulse_rows):
        invalid_reason = "ACK_MISSING"
    elif not any(row.get("event", "").upper() in {"STOP", "PULSE_COMPLETE", "PULSE_DONE"} for row in pulse_rows):
        invalid_reason = "STOP_MISSING"
    elif output_after_stop:
        invalid_reason = "OUTPUT_ACTIVE_AFTER_STOP"
    elif not final_left_zero or not final_right_zero:
        invalid_reason = "FINAL_COMMANDS_NONZERO"

    return {
        "trial_index": trial_index,
        "mode": mode,
        "a_cmd": planned["a_cmd"],
        "b_cmd": planned["b_cmd"],
        "pulse_ms": planned["pulse_ms"],
        "imu_type": _latest(all_rows, "imu_type", "NA"),
        "imu_present": _latest(all_rows, "imu_present", "false"),
        "imu_data_plausible": _latest(all_rows, "imu_data_plausible", "false"),
        "imu_pmu_normal": _latest(all_rows, "imu_pmu_normal", "false"),
        "imu_yaw_baseline_deg": "NA" if baseline_yaw is None else f"{baseline_yaw:.3f}",
        "imu_yaw_final_deg": "NA" if final_yaw is None else f"{final_yaw:.3f}",
        "imu_yaw_delta_deg": "NA" if yaw_delta is None else f"{yaw_delta:.3f}",
        "imu_yaw_delta_abs_deg": "NA" if yaw_delta is None else f"{abs(yaw_delta):.3f}",
        "user_motion_report": user_motion_report,
        "user_turn_angle_report": user_turn_angle_report,
        "arm_seen": any(row.get("event", "").upper() == "ARM" for row in pulse_rows),
        "ack_seen": any(row.get("event", "").upper() == "ACK" for row in pulse_rows),
        "stop_seen": any(row.get("event", "").upper() in {"STOP", "PULSE_COMPLETE", "PULSE_DONE"} for row in pulse_rows),
        "reject_seen": reject_seen,
        "reject_reason": reject_reason,
        "rc_invalid_seen": reject_reason == "RC_INVALID",
        "final_left_cmd": f"{final_left:.3f}",
        "final_right_cmd": f"{final_right:.3f}",
        "final_left_cmd_zero": final_left_zero,
        "final_right_cmd_zero": final_right_zero,
        "valid_trial": invalid_reason == "NONE",
        "invalid_reason": invalid_reason,
        "ready_for_full_path_following": False,
    }


def build_summary(rows: Sequence[dict[str, object]], *, mode: str, repeat_count: int) -> dict[str, object]:
    valid_rows = [row for row in rows if row.get("valid_trial") is True]
    deltas = [
        float(str(row["imu_yaw_delta_deg"]))
        for row in valid_rows
        if str(row.get("imu_yaw_delta_deg", "NA")).upper() != "NA"
    ]
    abs_deltas = [abs(value) for value in deltas]
    mean, median, std = mean_median_std(deltas)
    abs_mean, _abs_median, _abs_std = mean_median_std(abs_deltas)
    repeat_45 = estimate_repeat_count(45.0, mean)
    repeat_90 = estimate_repeat_count(90.0, mean)
    turn_too_small = abs_mean is None or abs_mean < 5.0
    turn_unstable = std is not None and std > max(5.0, (abs_mean or 0.0) * 0.5)
    return {
        "mode": mode,
        "repeat_count": repeat_count,
        "valid_trial_count": len(valid_rows),
        "imu_yaw_delta_mean_deg": "NA" if mean is None else f"{mean:.3f}",
        "imu_yaw_delta_median_deg": "NA" if median is None else f"{median:.3f}",
        "imu_yaw_delta_std_deg": "NA" if std is None else f"{std:.3f}",
        "imu_yaw_delta_abs_mean_deg": "NA" if abs_mean is None else f"{abs_mean:.3f}",
        "recommended_repeat_for_45deg": "NA" if repeat_45 is None else repeat_45,
        "recommended_repeat_for_90deg": "NA" if repeat_90 is None else repeat_90,
        "turn_too_small": turn_too_small,
        "turn_unstable": turn_unstable,
        "user_angle_consistency": "operator_review",
        "ready_for_stage27_connector_repeat": bool(valid_rows) and not turn_too_small and not turn_unstable,
        "ready_for_full_path_following": False,
    }


def _write_csv(path: Path, rows: Sequence[dict[str, object]]) -> None:
    with path.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=STAGE27_FIELDS)
        writer.writeheader()
        for row in rows:
            writer.writerow({field: row.get(field, "") for field in STAGE27_FIELDS})


def _write_summary(path: Path, summary: dict[str, object]) -> None:
    lines = [
        "# Stage 27 IMU Turn Calibration",
        "",
        "BMI160 relative-yaw validation for bounded Stage20 turn primitives. Not full path following.",
        "",
    ]
    lines.extend(f"- {key}: `{value}`" for key, value in summary.items())
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def _write_calibration(path: Path, summary: dict[str, object], rows: Sequence[dict[str, object]]) -> None:
    payload = {
        "stage": "stage27_imu_turn_calibration",
        "ready_for_full_path_following": False,
        "summary": summary,
        "trials": list(rows),
    }
    path.write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n", encoding="utf-8")


def run_stage27(args: argparse.Namespace) -> int:
    fine = json.loads(Path(args.fine_calibration_json).read_text(encoding="utf-8"))
    turn = json.loads(Path(args.turn_calibration_json).read_text(encoding="utf-8"))
    primitive_map = stage26.build_primitive_map(fine, turn)
    modes = parse_sequence_for_mode(args.mode, args.repeat_count, args.sequence)

    try:
        import serial  # type: ignore[import-not-found]
    except ImportError as exc:
        raise RuntimeError("pyserial is not available; cannot run Stage27 IMU turn calibration") from exc

    out_dir = Path(args.out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    rows: list[dict[str, object]] = []
    raw_lines: list[str] = []
    require_enter = _parse_bool(args.require_enter, default=True)
    interactive = _parse_bool(args.interactive_visible_motion, default=True)

    with serial.Serial(args.port, baudrate=115200, timeout=0.5) as handle:
        for trial_index, mode in enumerate(modes, start=1):
            trial_start_index = len(raw_lines)
            precondition_rows = _wait_for_imu_precondition(handle, raw_lines, args.heartbeat_timeout_s)
            if not imu_precondition_ready(precondition_rows):
                raise RuntimeError("Stage27 IMU precondition failed before pulse")
            baseline_rows = _collect_rows(handle, raw_lines, args.baseline_sample_s)
            planned = planned_turn(
                trial_index=trial_index,
                mode=mode,
                primitive_map=primitive_map,
                max_abs_a=args.max_abs_a,
                max_abs_b=args.max_abs_b,
                max_ms=args.max_ms,
            )
            if require_enter:
                input(f"Press Enter for Stage27 {mode} trial {trial_index}: {planned['stage20_command_text']} ")
            handle.write((str(planned["arm_command_text"]) + "\n").encode("ascii"))
            handle.flush()
            arm_rows = _wait_for_event(handle, raw_lines, {"ARM", "REJECT"}, args.event_timeout_s)
            if any(row.get("event", "").upper() == "ARM" for row in arm_rows):
                handle.write((str(planned["stage20_command_text"]) + "\n").encode("ascii"))
                handle.flush()
                _wait_for_event(handle, raw_lines, {"ACK", "REJECT"}, args.event_timeout_s)
                _wait_for_event(handle, raw_lines, {"STOP", "PULSE_COMPLETE", "PULSE_DONE"}, max(args.event_timeout_s, int(planned["pulse_ms"]) / 1000.0 + 1.0))
            handle.write((str(planned["stop_command_text"]) + "\n").encode("ascii"))
            handle.flush()
            _wait_for_event(handle, raw_lines, {"STOP"}, args.event_timeout_s)
            final_rows = _collect_rows(handle, raw_lines, args.final_sample_s)
            pulse_rows = _serial_rows(raw_lines, trial_start_index)
            if interactive:
                user_motion = _prompt_choice("Observed motion? [forward/backward/left/right/twitch/none/unknown] ", MOTION_REPORTS)
                user_turn = _prompt_choice(
                    "Observed turn angle? [none/twitch/small_under_15deg/medium_15_45deg/big_over_45deg/unknown] ",
                    TURN_REPORTS,
                )
            else:
                user_motion = "unknown"
                user_turn = "unknown"
            row = row_from_trial(
                trial_index=trial_index,
                mode=mode,
                planned=planned,
                baseline_rows=baseline_rows,
                final_rows=final_rows,
                pulse_rows=pulse_rows,
                user_motion_report=user_motion,
                user_turn_angle_report=user_turn,
            )
            rows.append(row)
            if row["reject_seen"] is True or row["rc_invalid_seen"] is True or row["valid_trial"] is not True:
                break

    summary = build_summary(rows, mode=args.mode, repeat_count=args.repeat_count)
    _write_csv(out_dir / "stage27_imu_turn_calibration.csv", rows)
    (out_dir / "raw_usbdbg.log").write_text("\n".join(raw_lines) + ("\n" if raw_lines else ""), encoding="utf-8")
    _write_summary(out_dir / "summary.md", summary)
    _write_calibration(out_dir / "imu_turn_calibration.json", summary, rows)
    print("Stage 27 IMU turn calibration complete.")
    print(f"stage27_csv={out_dir / 'stage27_imu_turn_calibration.csv'}")
    print(f"raw_usbdbg_log={out_dir / 'raw_usbdbg.log'}")
    print(f"summary_md={out_dir / 'summary.md'}")
    print(f"imu_turn_calibration_json={out_dir / 'imu_turn_calibration.json'}")
    print("ready_for_full_path_following=false")
    return 0


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Stage27 BMI160 relative-yaw turn pulse calibration.")
    parser.add_argument("--port", required=True)
    parser.add_argument("--fine-calibration-json", required=True)
    parser.add_argument("--turn-calibration-json", required=True)
    parser.add_argument("--mode", choices=("turn_left", "turn_right", "sequence"), required=True)
    parser.add_argument("--sequence")
    parser.add_argument("--repeat-count", type=int, default=3)
    parser.add_argument("--require-enter", default="true")
    parser.add_argument("--interactive-visible-motion", default="true")
    parser.add_argument("--baseline-sample-s", type=float, default=0.8)
    parser.add_argument("--final-sample-s", type=float, default=0.8)
    parser.add_argument("--heartbeat-timeout-s", type=float, default=10.0)
    parser.add_argument("--event-timeout-s", type=float, default=5.0)
    parser.add_argument("--max-abs-a", type=float, default=DEFAULT_MAX_ABS_A)
    parser.add_argument("--max-abs-b", type=float, default=DEFAULT_MAX_ABS_B)
    parser.add_argument("--max-ms", type=int, default=DEFAULT_MAX_MS)
    parser.add_argument("--out-dir", required=True)
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    if args.repeat_count <= 0:
        raise SystemExit("--repeat-count must be > 0")
    if args.max_abs_a <= 0 or args.max_abs_a > DEFAULT_MAX_ABS_A:
        raise SystemExit(f"--max-abs-a must be > 0 and <= {DEFAULT_MAX_ABS_A}")
    if args.max_abs_b <= 0 or args.max_abs_b > DEFAULT_MAX_ABS_B:
        raise SystemExit(f"--max-abs-b must be > 0 and <= {DEFAULT_MAX_ABS_B}")
    if args.max_ms <= 0 or args.max_ms > DEFAULT_MAX_MS:
        raise SystemExit(f"--max-ms must be > 0 and <= {DEFAULT_MAX_MS}")
    try:
        return run_stage27(args)
    except RuntimeError as exc:
        print(f"reason={exc}")
        print("ready_for_full_path_following=false")
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
