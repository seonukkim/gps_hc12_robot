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

from legacy.stage_tools import stage26_calibrated_primitive_sequence
from tools import station_path_package_tracker


CALIBRATION: dict[str, dict[str, float | int]] = {
    "forward": {"a": 0.30, "b": 0.0, "ms": 800},
    "backward": {"a": -0.08, "b": 0.0, "ms": 300},
    "turn_left": {"a": 0.0, "b": 0.26, "ms": 700},
    "turn_right": {"a": 0.0, "b": -0.08, "ms": 250},
}
SEQUENCE = "forward,backward,turn_left,turn_right"

STAGE29_FIELDS = (
    "baseline_name",
    "status",
    "reason",
    "detail",
    "ready_for_full_path_following",
)


def _parse_bool(value: object) -> bool | None:
    if value is None:
        return None
    text = str(value).strip().lower()
    if text in {"1", "true", "yes", "ok", "active"}:
        return True
    if text in {"0", "false", "no", "off", "inactive"}:
        return False
    return None


def _parse_float(value: object) -> float | None:
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


def parse_usb_rows(text: str) -> list[dict[str, str]]:
    return station_path_package_tracker.parse_usbdbg_rows(text)


def _read_rows(path: str | None) -> list[dict[str, str]]:
    if not path:
        return []
    return parse_usb_rows(Path(path).read_text(encoding="utf-8"))


def _status(pass_ready: bool) -> str:
    return "PASS" if pass_ready else "FAIL"


def _result(name: str, pass_ready: bool, reasons: Sequence[str], detail: dict[str, object]) -> dict[str, object]:
    return {
        "baseline_name": name,
        "status": _status(pass_ready),
        "reason": "OK" if pass_ready else ",".join(reasons),
        "detail": detail,
        "ready_for_full_path_following": False,
    }


def evaluate_manual_rc_baseline(rows: Sequence[dict[str, str]]) -> dict[str, object]:
    rc_ok = any(_parse_bool(row.get("rc_ok")) is True for row in rows)
    manual_source_rows = [
        row for row in rows
        if row.get("control_source") == "RC_MANUAL"
        or row.get("active_target_source") == "manual_rc"
        or row.get("mode") == "MANUAL"
    ]
    nonzero_rows = [
        row for row in rows
        if abs(_parse_float(row.get("final_left_cmd")) or 0.0) > 1e-3
        or abs(_parse_float(row.get("final_right_cmd")) or 0.0) > 1e-3
    ]
    zero_return = any(
        _parse_bool(row.get("neutral_ok")) is True
        and abs(_parse_float(row.get("final_left_cmd")) or 0.0) <= 1e-3
        and abs(_parse_float(row.get("final_right_cmd")) or 0.0) <= 1e-3
        for row in rows
    )
    output_active = any(_parse_bool(row.get("physical_output_active")) is True for row in rows)
    autonomous_sources = [
        row.get("active_target_source", "NA")
        for row in rows
        if row.get("active_target_source") not in {"", None, "NA", "NONE", "manual_rc"}
    ]
    reasons: list[str] = []
    if not rc_ok:
        reasons.append("RC_NOT_OK")
    if not manual_source_rows:
        reasons.append("RC_MANUAL_SOURCE_NOT_SEEN")
    if not nonzero_rows:
        reasons.append("NO_NONZERO_FINAL_COMMANDS")
    if not zero_return:
        reasons.append("NO_NEUTRAL_ZERO_RETURN")
    if not output_active:
        reasons.append("PHYSICAL_OUTPUT_ACTIVE_NOT_SEEN")
    if autonomous_sources:
        reasons.append("AUTONOMOUS_TARGET_SOURCE_SEEN")
    return _result(
        "MANUAL_RC_BASELINE",
        not reasons,
        reasons,
        {
            "row_count": len(rows),
            "rc_ok": rc_ok,
            "manual_source_rows": len(manual_source_rows),
            "nonzero_final_command_rows": len(nonzero_rows),
            "zero_return": zero_return,
            "physical_output_active_seen": output_active,
            "autonomous_target_sources": sorted(set(str(item) for item in autonomous_sources)),
        },
    )


def _lat_lon_present(row: dict[str, str]) -> bool:
    lat = _parse_float(row.get("current_lat", row.get("gps_lat")))
    lon = _parse_float(row.get("current_lon", row.get("gps_lon")))
    return lat is not None and lon is not None


def evaluate_gps_baseline(rows: Sequence[dict[str, str]], *, min_sats: int = 5, max_hdop: float = 2.5) -> dict[str, object]:
    pass_rows: list[dict[str, str]] = []
    for row in rows:
        sats = _parse_float(row.get("gps_sats"))
        hdop = _parse_float(row.get("gps_hdop"))
        gps_ok = (
            row.get("gps_block_reason") == "OK"
            or _parse_bool(row.get("gps_ready")) is True
            or _parse_bool(row.get("gps_solution_valid")) is True
        )
        location_ok = _parse_bool(row.get("gps_location_valid")) is True or _lat_lon_present(row)
        if gps_ok and location_ok and sats is not None and hdop is not None and sats >= min_sats and hdop <= max_hdop:
            pass_rows.append(row)
    reasons: list[str] = []
    if not pass_rows:
        reasons.append("GPS_READY_ROW_NOT_SEEN")
    return _result(
        "GPS_BASELINE",
        bool(pass_rows),
        reasons,
        {
            "row_count": len(rows),
            "pass_rows": len(pass_rows),
            "min_sats": min_sats,
            "max_hdop": max_hdop,
            "latest_gps_block_reason": _latest(rows, "gps_block_reason"),
            "latest_gps_sats": _latest(rows, "gps_sats"),
            "latest_gps_hdop": _latest(rows, "gps_hdop"),
        },
    )


def evaluate_imu_baseline(rows: Sequence[dict[str, str]]) -> dict[str, object]:
    pass_rows: list[dict[str, str]] = []
    for row in rows:
        heading_ready = (
            _parse_bool(row.get("imu_heading_ready")) is True
            or row.get("imu_heading_block_reason") == "RELATIVE_YAW_DIAG_ONLY"
        )
        if (
            row.get("imu_type") == "BMI160"
            and _parse_bool(row.get("imu_present")) is True
            and _parse_bool(row.get("imu_data_plausible")) is True
            and _parse_bool(row.get("imu_calibrated")) is True
            and heading_ready
        ):
            pass_rows.append(row)
    reasons: list[str] = []
    if not pass_rows:
        reasons.append("BMI160_READY_ROW_NOT_SEEN")
    return _result(
        "IMU_BASELINE",
        bool(pass_rows),
        reasons,
        {
            "row_count": len(rows),
            "pass_rows": len(pass_rows),
            "latest_imu_type": _latest(rows, "imu_type"),
            "latest_imu_heading_block_reason": _latest(rows, "imu_heading_block_reason"),
        },
    )


def _load_csv_rows(path: str | None) -> list[dict[str, str]]:
    if not path:
        return []
    csv_path = Path(path)
    if csv_path.is_dir():
        csv_path = csv_path / "stage26_calibrated_primitive_sequence.csv"
    if not csv_path.exists():
        return []
    with csv_path.open("r", encoding="utf-8", newline="") as handle:
        return [{key: value for key, value in row.items() if key is not None} for row in csv.DictReader(handle)]


def _station_pulse_from_csv(rows: Sequence[dict[str, str]]) -> tuple[bool, list[str], dict[str, object]]:
    reasons: list[str] = []
    by_name = {row.get("primitive_name", ""): row for row in rows}
    for name, expected in CALIBRATION.items():
        row = by_name.get(name)
        if row is None:
            reasons.append(f"{name.upper()}_ROW_MISSING")
            continue
        if _parse_bool(row.get("ack_seen")) is not True:
            reasons.append(f"{name.upper()}_ACK_MISSING")
        if _parse_bool(row.get("stop_seen")) is not True:
            reasons.append(f"{name.upper()}_STOP_MISSING")
        if _parse_bool(row.get("reject_seen")) is True:
            reasons.append(f"{name.upper()}_REJECT_SEEN")
        if str(row.get("reject_reason", "NONE")).upper() == "RC_INVALID":
            reasons.append(f"{name.upper()}_RC_INVALID")
        if _parse_bool(row.get("final_left_cmd_zero")) is not True or _parse_bool(row.get("final_right_cmd_zero")) is not True:
            reasons.append(f"{name.upper()}_FINAL_COMMAND_NONZERO")
        if _parse_float(row.get("a_cmd")) != float(expected["a"]):
            reasons.append(f"{name.upper()}_A_CHANGED")
        if _parse_float(row.get("b_cmd")) != float(expected["b"]):
            reasons.append(f"{name.upper()}_B_CHANGED")
        if int(_parse_float(row.get("pulse_ms")) or -1) != int(expected["ms"]):
            reasons.append(f"{name.upper()}_MS_CHANGED")
    return not reasons, reasons, {"row_count": len(rows), "primitive_rows": sorted(by_name)}


def _station_pulse_from_usb(rows: Sequence[dict[str, str]]) -> tuple[bool, list[str], dict[str, object]]:
    ack_rows = [row for row in rows if row.get("event", "").upper() == "ACK"]
    stop_rows = [row for row in rows if row.get("event", "").upper() in {"STOP", "PULSE_COMPLETE", "PULSE_DONE"}]
    reject_rows = [row for row in rows if row.get("event", "").upper() == "REJECT"]
    final_left = _parse_float(_latest(rows, "final_left_cmd", "0.0")) or 0.0
    final_right = _parse_float(_latest(rows, "final_right_cmd", "0.0")) or 0.0
    reasons: list[str] = []
    if len(ack_rows) < 4:
        reasons.append("ACK_COUNT_LT_4")
    if len(stop_rows) < 4:
        reasons.append("STOP_COUNT_LT_4")
    if reject_rows:
        reasons.append("REJECT_SEEN")
    if any(str(row.get("stage20_reject_reason", "")).upper() == "RC_INVALID" for row in rows):
        reasons.append("RC_INVALID")
    if abs(final_left) > 1e-9 or abs(final_right) > 1e-9:
        reasons.append("FINAL_COMMAND_NONZERO")
    if any(row.get("active_target_source") == "compile_time" for row in rows):
        reasons.append("COMPILE_TIME_TARGET_ACTIVE")
    return not reasons, reasons, {
        "row_count": len(rows),
        "ack_count": len(ack_rows),
        "stop_count": len(stop_rows),
        "reject_count": len(reject_rows),
        "final_left_cmd": final_left,
        "final_right_cmd": final_right,
    }


def evaluate_station_pulse_baseline(
    *,
    csv_rows: Sequence[dict[str, str]] | None = None,
    usb_rows: Sequence[dict[str, str]] | None = None,
) -> dict[str, object]:
    if csv_rows:
        passed, reasons, detail = _station_pulse_from_csv(csv_rows)
    else:
        passed, reasons, detail = _station_pulse_from_usb(usb_rows or [])
    return _result("STATION_PULSE_BASELINE", passed, reasons, detail)


def build_baseline_lock(
    *,
    manual_rows: Sequence[dict[str, str]],
    sensor_rows: Sequence[dict[str, str]],
    station_pulse_csv_rows: Sequence[dict[str, str]],
    station_pulse_usb_rows: Sequence[dict[str, str]],
    min_sats: int,
    max_hdop: float,
) -> tuple[dict[str, object], list[dict[str, object]]]:
    manual = evaluate_manual_rc_baseline(manual_rows)
    gps = evaluate_gps_baseline(sensor_rows, min_sats=min_sats, max_hdop=max_hdop)
    imu = evaluate_imu_baseline(sensor_rows)
    pulse = evaluate_station_pulse_baseline(csv_rows=station_pulse_csv_rows, usb_rows=station_pulse_usb_rows)
    rows = [manual, gps, imu, pulse]
    ready_stage30 = all(row["status"] == "PASS" for row in rows)
    payload = {
        "manual_rc_baseline": manual["status"],
        "gps_baseline": gps["status"],
        "imu_baseline": imu["status"],
        "station_pulse_baseline": pulse["status"],
        "calibration": CALIBRATION,
        "ready_for_stage30_physical_path_planning": ready_stage30,
        "ready_for_full_path_following": False,
        "details": {str(row["baseline_name"]): row for row in rows},
    }
    return payload, rows


def _write_outputs(out_dir: Path, payload: dict[str, object], rows: Sequence[dict[str, object]], raw_lines: Sequence[str]) -> None:
    out_dir.mkdir(parents=True, exist_ok=True)
    (out_dir / "baseline_lock.json").write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    (out_dir / "raw_usbdbg.log").write_text("\n".join(raw_lines) + ("\n" if raw_lines else ""), encoding="utf-8")
    with (out_dir / "stage29_baseline_lock.csv").open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=STAGE29_FIELDS)
        writer.writeheader()
        for row in rows:
            writer.writerow({
                "baseline_name": row["baseline_name"],
                "status": row["status"],
                "reason": row["reason"],
                "detail": json.dumps(row["detail"], sort_keys=True),
                "ready_for_full_path_following": False,
            })
    lines = ["# Stage29 Baseline Lock", "", "Clean baseline consolidation. Not full path following.", ""]
    for key in ("manual_rc_baseline", "gps_baseline", "imu_baseline", "station_pulse_baseline", "ready_for_stage30_physical_path_planning", "ready_for_full_path_following"):
        lines.append(f"- {key}: `{payload[key]}`")
    lines.append("")
    lines.append("## Calibration")
    for name, values in CALIBRATION.items():
        lines.append(f"- {name}: `A={values['a']} B={values['b']} ms={values['ms']}`")
    lines.append("")
    lines.append("## Details")
    for row in rows:
        lines.append(f"- {row['baseline_name']}: `{row['status']}` reason=`{row['reason']}`")
    (out_dir / "summary.md").write_text("\n".join(lines) + "\n", encoding="utf-8")


def _fixed_primitive_map() -> dict[str, dict[str, object]]:
    return {
        name: {"primitive_name": name, "a_cmd": values["a"], "b_cmd": values["b"], "pulse_ms": values["ms"]}
        for name, values in CALIBRATION.items()
    } | {"stop": {"primitive_name": "stop", "a_cmd": 0.0, "b_cmd": 0.0, "pulse_ms": 0}}


def _prompt_choice(prompt: str, choices: set[str]) -> str:
    while True:
        answer = input(prompt).strip().lower()
        if answer in choices:
            return answer
        print("Use one of: " + ", ".join(sorted(choices)))


def _run_station_pulses(handle: object, raw_lines: list[str], args: argparse.Namespace) -> list[dict[str, object]]:
    plan = stage26_calibrated_primitive_sequence.build_plan(
        SEQUENCE,
        _fixed_primitive_map(),
        max_abs_a=0.35,
        max_abs_b=0.35,
        max_ms=1000,
    )
    rows: list[dict[str, object]] = []
    for planned in plan:
        primitive_start = len(raw_lines)
        precondition_rows = stage26_calibrated_primitive_sequence._wait_for_precondition(handle, raw_lines, args.heartbeat_timeout_s)
        if not stage26_calibrated_primitive_sequence.stage20_precondition_ready(precondition_rows):
            raise RuntimeError("Stage29 Stage20 precondition failed before station pulse")
        if _parse_bool(args.require_enter) is True:
            input(f"Press Enter for Stage29 {planned['primitive_name']}: {planned['stage20_command_text']} ")
        handle.write((str(planned["arm_command_text"]) + "\n").encode("ascii"))
        handle.flush()
        stage26_calibrated_primitive_sequence._wait_for_event(handle, raw_lines, {"ARM", "REJECT"}, args.event_timeout_s)
        handle.write((str(planned["stage20_command_text"]) + "\n").encode("ascii"))
        handle.flush()
        stage26_calibrated_primitive_sequence._wait_for_event(handle, raw_lines, {"ACK", "REJECT"}, args.event_timeout_s)
        pulse_ms = int(planned["pulse_ms"])
        stage26_calibrated_primitive_sequence._wait_for_event(handle, raw_lines, {"STOP", "PULSE_COMPLETE", "PULSE_DONE"}, max(args.event_timeout_s, pulse_ms / 1000.0 + 1.0))
        handle.write((str(planned["stop_command_text"]) + "\n").encode("ascii"))
        handle.flush()
        stage26_calibrated_primitive_sequence._wait_for_event(handle, raw_lines, {"STOP"}, args.event_timeout_s)
        pulse_rows = stage26_calibrated_primitive_sequence._serial_rows(raw_lines, primitive_start)
        motion = _prompt_choice("Observed motion? [forward/backward/left/right/twitch/none/unknown] ", stage26_calibrated_primitive_sequence.MOTION_REPORTS)
        turn = "none"
        if planned["primitive_name"] in {"turn_left", "turn_right"}:
            turn = _prompt_choice("Observed turn angle? [none/twitch/small_under_15deg/medium_15_45deg/big_over_45deg/unknown] ", stage26_calibrated_primitive_sequence.TURN_REPORTS)
        row = stage26_calibrated_primitive_sequence.row_from_primitive(
            planned=planned,
            rows=pulse_rows,
            user_motion_report=motion,
            user_turn_angle_report=turn,
        )
        rows.append(row)
        if row["valid_primitive"] is not True:
            break
    return rows


def run_live(args: argparse.Namespace) -> tuple[list[dict[str, str]], list[dict[str, str]], list[dict[str, object]], list[str]]:
    try:
        import serial  # type: ignore[import-not-found]
    except ImportError as exc:
        raise RuntimeError("pyserial is not available; cannot run Stage29 live baseline") from exc
    raw_lines: list[str] = []
    with serial.Serial(args.port, baudrate=115200, timeout=0.5) as handle:
        if _parse_bool(args.require_enter) is True:
            input("Press Enter to start MANUAL_RC_BASELINE. Keep AUTO OFF and move sticks manually. ")
        manual_start = len(raw_lines)
        deadline = time.monotonic() + args.manual_duration_s
        while time.monotonic() < deadline:
            raw = handle.readline()
            if raw:
                line = raw.decode("utf-8", errors="replace").strip()
                print(line)
                raw_lines.append(line)
        manual_rows = parse_usb_rows("\n".join(raw_lines[manual_start:]))

        if _parse_bool(args.require_enter) is True:
            input("Return sticks neutral. Press Enter to start SENSOR_BASELINE no-motion watch. ")
        sensor_start = len(raw_lines)
        deadline = time.monotonic() + args.sensor_duration_s
        while time.monotonic() < deadline:
            raw = handle.readline()
            if raw:
                line = raw.decode("utf-8", errors="replace").strip()
                print(line)
                raw_lines.append(line)
        sensor_rows = parse_usb_rows("\n".join(raw_lines[sensor_start:]))
        pulse_rows = _run_station_pulses(handle, raw_lines, args)
    return manual_rows, sensor_rows, pulse_rows, raw_lines


def run(args: argparse.Namespace) -> int:
    out_dir = Path(args.out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    raw_lines: list[str] = []
    if args.mode == "live":
        manual_rows, sensor_rows, pulse_csv_rows_obj, raw_lines = run_live(args)
        pulse_csv_rows = [{key: str(value) for key, value in row.items()} for row in pulse_csv_rows_obj]
        pulse_usb_rows: list[dict[str, str]] = []
    else:
        manual_rows = _read_rows(args.manual_log or args.combined_log)
        sensor_rows = _read_rows(args.sensor_log or args.combined_log)
        pulse_usb_rows = _read_rows(args.station_pulse_log)
        pulse_csv_rows = _load_csv_rows(args.station_pulse_csv)
        for path in (args.manual_log, args.sensor_log, args.station_pulse_log, args.combined_log):
            if path:
                raw_lines.extend(Path(path).read_text(encoding="utf-8").splitlines())

    payload, rows = build_baseline_lock(
        manual_rows=manual_rows,
        sensor_rows=sensor_rows,
        station_pulse_csv_rows=pulse_csv_rows,
        station_pulse_usb_rows=pulse_usb_rows,
        min_sats=args.min_sats,
        max_hdop=args.max_hdop,
    )
    _write_outputs(out_dir, payload, rows, raw_lines)
    print(f"manual_rc_baseline={payload['manual_rc_baseline']}")
    print(f"gps_baseline={payload['gps_baseline']}")
    print(f"imu_baseline={payload['imu_baseline']}")
    print(f"station_pulse_baseline={payload['station_pulse_baseline']}")
    print(f"ready_for_stage30_physical_path_planning={str(payload['ready_for_stage30_physical_path_planning']).lower()}")
    print("ready_for_full_path_following=false")
    return 0


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Stage29 baseline lock validator.")
    parser.add_argument("--mode", choices=("evaluate_logs", "live"), default="evaluate_logs")
    parser.add_argument("--port", default="/dev/ttyACM0")
    parser.add_argument("--manual-log")
    parser.add_argument("--sensor-log")
    parser.add_argument("--station-pulse-log")
    parser.add_argument("--station-pulse-csv")
    parser.add_argument("--combined-log")
    parser.add_argument("--manual-duration-s", type=float, default=60.0)
    parser.add_argument("--sensor-duration-s", type=float, default=30.0)
    parser.add_argument("--min-sats", type=int, default=5)
    parser.add_argument("--max-hdop", type=float, default=2.5)
    parser.add_argument("--require-enter", default="true")
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
