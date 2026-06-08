from __future__ import annotations

import argparse
import csv
import json
import math
import re
import time
from pathlib import Path
from typing import Sequence

try:
    from tools import _bootstrap  # type: ignore  # noqa: F401
except ImportError:
    import _bootstrap  # type: ignore  # noqa: F401

from tools import station_path_package_tracker


DEFAULT_FORWARD_A_CMD = 0.30
DEFAULT_FORWARD_MS = 800
DEFAULT_BACKWARD_A_CMD = -0.08
DEFAULT_BACKWARD_MS = 300
DEFAULT_TURN_LEFT_B_CMD = 0.26
DEFAULT_TURN_LEFT_MS = 700
DEFAULT_TURN_RIGHT_B_CMD = -0.08
DEFAULT_TURN_RIGHT_MS = 250
DEFAULT_MAX_ABS_A = 0.35
DEFAULT_MAX_ABS_B = 0.35
DEFAULT_MAX_MS = 1000
INVALID_REJECT_REASONS = {"RC_INVALID", "LATCHED_STOP"}
PRIMITIVE_NAMES = {"forward", "backward", "turn_left", "turn_right", "stop"}
MOTION_REPORTS = {"forward", "backward", "left", "right", "twitch", "none", "unknown"}
TURN_REPORTS = {"none", "twitch", "small_under_15deg", "medium_15_45deg", "big_over_45deg", "unknown"}

STAGE26_FIELDS = (
    "primitive_index",
    "primitive_name",
    "repeat_index",
    "a_cmd",
    "b_cmd",
    "pulse_ms",
    "arm_seen",
    "ack_seen",
    "stop_seen",
    "reject_seen",
    "reject_reason",
    "rc_ok_before_cmd",
    "neutral_ok_before_cmd",
    "physical_output_active_seen",
    "physical_output_active_after_stop",
    "final_left_cmd",
    "final_right_cmd",
    "final_left_cmd_zero",
    "final_right_cmd_zero",
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


def _optional_int(value: object) -> int | None:
    parsed = _optional_float(value)
    return int(parsed) if parsed is not None else None


def _latest(rows: Sequence[dict[str, str]], key: str, default: str = "NA") -> str:
    for row in reversed(rows):
        if key in row:
            return row[key]
    return default


def _latest_reject_reason(rows: Sequence[dict[str, str]]) -> str:
    stage20_reason = _latest(rows, "stage20_reject_reason", "")
    if stage20_reason:
        return stage20_reason
    return _latest(rows, "stage16_reject_reason", "NONE")


def _first_float(calibration: dict[str, object], keys: Sequence[str], default: float) -> float:
    for key in keys:
        parsed = _optional_float(calibration.get(key))
        if parsed is not None and parsed != 0.0:
            return parsed
    return default


def _first_int(calibration: dict[str, object], keys: Sequence[str], default: int) -> int:
    for key in keys:
        parsed = _optional_int(calibration.get(key))
        if parsed is not None and parsed > 0:
            return parsed
    return default


def _load_json(path: Path) -> dict[str, object]:
    return json.loads(path.read_text(encoding="utf-8"))


def build_primitive_map(
    fine_calibration: dict[str, object] | None,
    turn_calibration: dict[str, object] | None,
) -> dict[str, dict[str, object]]:
    fine = fine_calibration or {}
    turn = turn_calibration or {}

    forward_a = abs(_first_float(
        fine,
        ("stage26_forward_a_cmd", "stage22_forward_a_cmd", "stage21_forward_a_cmd", "forward_recommended_crawl_a_cmd", "forward_visible_a_cmd"),
        DEFAULT_FORWARD_A_CMD,
    ))
    backward_a = -abs(_first_float(
        fine,
        ("stage26_backward_a_cmd", "stage21_backward_a_cmd", "backward_recommended_crawl_a_cmd", "backward_visible_a_cmd", "backward_min_a_cmd"),
        DEFAULT_BACKWARD_A_CMD,
    ))
    turn_left_b = abs(_first_float(
        turn,
        ("stage26_turn_left_b_cmd", "turn_left_usable_b_cmd", "turn_left_min_b_cmd", "safe_turn_left_cmd"),
        DEFAULT_TURN_LEFT_B_CMD,
    ))
    turn_right_b = -abs(_first_float(
        turn,
        ("stage26_turn_right_b_cmd", "turn_right_small_b_cmd", "turn_right_min_b_cmd", "safe_turn_right_cmd"),
        DEFAULT_TURN_RIGHT_B_CMD,
    ))

    return {
        "forward": {
            "primitive_name": "forward",
            "a_cmd": forward_a,
            "b_cmd": 0.0,
            "pulse_ms": _first_int(
                fine,
                ("stage26_forward_pulse_ms", "stage22_forward_pulse_ms", "stage21_forward_pulse_ms", "forward_recommended_crawl_pulse_ms", "forward_visible_pulse_ms"),
                DEFAULT_FORWARD_MS,
            ),
        },
        "backward": {
            "primitive_name": "backward",
            "a_cmd": backward_a,
            "b_cmd": 0.0,
            "pulse_ms": _first_int(
                fine,
                ("stage26_backward_pulse_ms", "stage21_backward_pulse_ms", "backward_recommended_crawl_pulse_ms", "backward_visible_pulse_ms", "backward_min_pulse_ms"),
                DEFAULT_BACKWARD_MS,
            ),
        },
        "turn_left": {
            "primitive_name": "turn_left",
            "a_cmd": 0.0,
            "b_cmd": turn_left_b,
            "pulse_ms": _first_int(
                turn,
                ("stage26_turn_left_pulse_ms", "turn_left_usable_pulse_ms", "turn_left_min_pulse_ms", "safe_turn_left_pulse_ms"),
                DEFAULT_TURN_LEFT_MS,
            ),
        },
        "turn_right": {
            "primitive_name": "turn_right",
            "a_cmd": 0.0,
            "b_cmd": turn_right_b,
            "pulse_ms": _first_int(
                turn,
                ("stage26_turn_right_pulse_ms", "turn_right_small_pulse_ms", "turn_right_min_pulse_ms", "safe_turn_right_pulse_ms"),
                DEFAULT_TURN_RIGHT_MS,
            ),
        },
        "stop": {
            "primitive_name": "stop",
            "a_cmd": 0.0,
            "b_cmd": 0.0,
            "pulse_ms": 0,
        },
    }


def parse_sequence(sequence: str) -> list[dict[str, object]]:
    expanded: list[dict[str, object]] = []
    primitive_index = 0
    pattern = re.compile(r"^(forward|backward|turn_left|turn_right|stop)(?:x([1-9][0-9]*))?$")
    for token in sequence.split(","):
        text = token.strip()
        if not text:
            continue
        match = pattern.match(text)
        if not match:
            raise ValueError(f"unsupported Stage26 primitive token: {text}")
        name = match.group(1)
        count = int(match.group(2) or "1")
        for repeat_index in range(1, count + 1):
            primitive_index += 1
            expanded.append({
                "primitive_index": primitive_index,
                "primitive_name": name,
                "repeat_index": repeat_index,
            })
    if not expanded:
        raise ValueError("empty Stage26 sequence")
    return expanded


def planned_primitive(
    *,
    primitive_index: int,
    primitive_name: str,
    repeat_index: int,
    primitive_map: dict[str, dict[str, object]],
    max_abs_a: float = DEFAULT_MAX_ABS_A,
    max_abs_b: float = DEFAULT_MAX_ABS_B,
    max_ms: int = DEFAULT_MAX_MS,
) -> dict[str, object]:
    if primitive_name not in primitive_map:
        raise ValueError(f"unknown Stage26 primitive: {primitive_name}")
    definition = primitive_map[primitive_name]
    a_cmd = float(definition["a_cmd"])
    b_cmd = float(definition["b_cmd"])
    pulse_ms = int(definition["pulse_ms"])
    if abs(a_cmd) > max_abs_a + 1e-12:
        raise ValueError(f"{primitive_name} A command exceeds max_abs_a")
    if abs(b_cmd) > max_abs_b + 1e-12:
        raise ValueError(f"{primitive_name} B command exceeds max_abs_b")
    if pulse_ms > max_ms:
        raise ValueError(f"{primitive_name} pulse_ms exceeds max_ms")
    return {
        "primitive_index": primitive_index,
        "primitive_name": primitive_name,
        "repeat_index": repeat_index,
        "a_cmd": a_cmd,
        "b_cmd": b_cmd,
        "pulse_ms": pulse_ms,
        "arm_command_text": "" if primitive_name == "stop" else f"STAGE20_ARM seq={primitive_index}",
        "stage20_command_text": "" if primitive_name == "stop" else f"STAGE20_CMD seq={primitive_index} a={a_cmd:.3f} b={b_cmd:.3f} ms={pulse_ms}",
        "stop_command_text": f"STAGE20_STOP seq={primitive_index}",
        "ready_for_full_path_following": False,
    }


def build_plan(
    sequence: str,
    primitive_map: dict[str, dict[str, object]],
    *,
    max_abs_a: float = DEFAULT_MAX_ABS_A,
    max_abs_b: float = DEFAULT_MAX_ABS_B,
    max_ms: int = DEFAULT_MAX_MS,
) -> list[dict[str, object]]:
    return [
        planned_primitive(
            primitive_index=int(item["primitive_index"]),
            primitive_name=str(item["primitive_name"]),
            repeat_index=int(item["repeat_index"]),
            primitive_map=primitive_map,
            max_abs_a=max_abs_a,
            max_abs_b=max_abs_b,
            max_ms=max_ms,
        )
        for item in parse_sequence(sequence)
    ]


def _serial_rows(raw_lines: Sequence[str], start_index: int = 0) -> list[dict[str, str]]:
    return station_path_package_tracker.parse_usbdbg_rows("\n".join(raw_lines[start_index:]))


def _stage20_precondition_row(row: dict[str, str]) -> bool:
    return (
        row.get("event", "").upper() == "HEARTBEAT"
        and _parse_bool(row.get("stage20_physical_ab_guarded_crawl")) is True
        and _parse_bool(row.get("stage20_firmware_ready")) is True
        and _parse_bool(row.get("physical_path_following_enable")) is False
        and _parse_bool(row.get("allow_motor_output")) is False
        and _parse_bool(row.get("rc_ok")) is True
        and _parse_bool(row.get("neutral_ok")) is True
        and _parse_bool(row.get("physical_output_active")) is False
    )


def stage20_precondition_ready(rows: Sequence[dict[str, str]]) -> bool:
    return any(_stage20_precondition_row(row) for row in rows)


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
    deadline = time.monotonic() + timeout_s
    start_index = len(raw_lines)
    while time.monotonic() < deadline:
        rows = _serial_rows(raw_lines, start_index)
        if stage20_precondition_ready(rows):
            return station_path_package_tracker.parse_usbdbg_rows("\n".join(raw_lines))
        raw = handle.readline()
        if raw:
            line = raw.decode("utf-8", errors="replace").strip()
            print(line)
            raw_lines.append(line)
    return station_path_package_tracker.parse_usbdbg_rows("\n".join(raw_lines))


def _active_rows(rows: Sequence[dict[str, str]]) -> list[dict[str, str]]:
    return [
        row
        for row in rows
        if row.get("event", "").upper() == "ACK"
        or row.get("stage20_cmd_state", "").upper() == "ACTIVE"
        or _parse_bool(row.get("physical_output_active"))
    ]


def row_from_primitive(
    *,
    planned: dict[str, object],
    rows: Sequence[dict[str, str]],
    user_motion_report: str,
    user_turn_angle_report: str,
) -> dict[str, object]:
    active = _active_rows(rows)
    final_left = _optional_float(_latest(rows, "final_left_cmd", "0.0")) or 0.0
    final_right = _optional_float(_latest(rows, "final_right_cmd", "0.0")) or 0.0
    reject_reason = _latest_reject_reason(rows)
    reject_seen = any(row.get("event", "").upper() == "REJECT" for row in rows)
    stop_seen = any(row.get("event", "").upper() in {"STOP", "PULSE_COMPLETE", "PULSE_DONE"} for row in rows)
    ack_seen = any(row.get("event", "").upper() == "ACK" for row in rows)
    is_stop = planned["primitive_name"] == "stop"
    output_after_stop = any(
        row.get("event", "").upper() == "STOP" and _parse_bool(row.get("physical_output_active"))
        for row in rows
    )
    final_left_zero = abs(final_left) <= 1e-9
    final_right_zero = abs(final_right) <= 1e-9
    invalid_reason = "NONE"
    if reject_seen:
        invalid_reason = reject_reason if reject_reason != "NONE" else "REJECT"
    elif reject_reason in INVALID_REJECT_REASONS:
        invalid_reason = reject_reason
    elif not is_stop and not ack_seen:
        invalid_reason = "ACK_MISSING"
    elif not stop_seen:
        invalid_reason = "STOP_MISSING"
    elif output_after_stop:
        invalid_reason = "OUTPUT_ACTIVE_AFTER_STOP"
    elif not final_left_zero or not final_right_zero:
        invalid_reason = "FINAL_COMMANDS_NONZERO"
    valid = invalid_reason == "NONE"
    return {
        "primitive_index": planned["primitive_index"],
        "primitive_name": planned["primitive_name"],
        "repeat_index": planned["repeat_index"],
        "a_cmd": planned["a_cmd"],
        "b_cmd": planned["b_cmd"],
        "pulse_ms": planned["pulse_ms"],
        "arm_seen": any(row.get("event", "").upper() == "ARM" for row in rows),
        "ack_seen": ack_seen,
        "stop_seen": stop_seen,
        "reject_seen": reject_seen,
        "reject_reason": reject_reason,
        "rc_ok_before_cmd": _latest(rows, "rc_ok", "NA"),
        "neutral_ok_before_cmd": _latest(rows, "neutral_ok", "NA"),
        "physical_output_active_seen": any(_parse_bool(row.get("physical_output_active")) for row in rows) or bool(active),
        "physical_output_active_after_stop": output_after_stop,
        "final_left_cmd": f"{final_left:.3f}",
        "final_right_cmd": f"{final_right:.3f}",
        "final_left_cmd_zero": final_left_zero,
        "final_right_cmd_zero": final_right_zero,
        "user_motion_report": user_motion_report,
        "user_turn_angle_report": user_turn_angle_report,
        "valid_primitive": valid,
        "invalid_reason": invalid_reason,
        "ready_for_full_path_following": False,
    }


def _prompt_choice(prompt: str, choices: set[str]) -> str:
    while True:
        answer = input(prompt).strip().lower()
        if answer in choices:
            return answer
        print("Use one of: " + ", ".join(sorted(choices)))


def _write_csv(path: Path, rows: Sequence[dict[str, object]]) -> None:
    with path.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=STAGE26_FIELDS)
        writer.writeheader()
        for row in rows:
            writer.writerow({field: row.get(field, "") for field in STAGE26_FIELDS})


def write_primitive_sequence(path: Path, planned: Sequence[dict[str, object]]) -> None:
    payload = {
        "stage": "stage26_calibrated_primitive_sequence",
        "ready_for_full_path_following": False,
        "primitives": [
            {
                "primitive_index": row["primitive_index"],
                "primitive_name": row["primitive_name"],
                "repeat_index": row["repeat_index"],
                "a_cmd": row["a_cmd"],
                "b_cmd": row["b_cmd"],
                "pulse_ms": row["pulse_ms"],
                "arm_command_text": row["arm_command_text"],
                "stage20_command_text": row["stage20_command_text"],
                "stop_command_text": row["stop_command_text"],
            }
            for row in planned
        ],
    }
    path.write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n", encoding="utf-8")


def build_summary(
    rows: Sequence[dict[str, object]],
    *,
    sequence: str,
    primitive_count: int,
    dry_run: bool,
) -> dict[str, object]:
    reject_count = sum(1 for row in rows if row.get("reject_seen") is True)
    rc_invalid_count = sum(1 for row in rows if str(row.get("invalid_reason", "")).upper() == "RC_INVALID")
    invalid_count = sum(1 for row in rows if row.get("valid_primitive") is not True)
    successful_count = sum(1 for row in rows if row.get("valid_primitive") is True)
    final = rows[-1] if rows else {}
    return {
        "sequence": sequence,
        "dry_run": dry_run,
        "primitive_count": primitive_count,
        "successful_primitive_count": successful_count,
        "invalid_primitive_count": invalid_count,
        "reject_count": reject_count,
        "rc_invalid_count": rc_invalid_count,
        "final_stop_confirmed": bool(rows) and final.get("stop_seen") is True and final.get("final_left_cmd_zero") is True and final.get("final_right_cmd_zero") is True,
        "forward_count": sum(1 for row in rows if row.get("primitive_name") == "forward"),
        "backward_count": sum(1 for row in rows if row.get("primitive_name") == "backward"),
        "turn_left_count": sum(1 for row in rows if row.get("primitive_name") == "turn_left"),
        "turn_right_count": sum(1 for row in rows if row.get("primitive_name") == "turn_right"),
        "ready_for_stage27_connector_repeat": (not dry_run and primitive_count > 0 and successful_count == primitive_count and reject_count == 0),
        "ready_for_full_path_following": False,
    }


def _write_summary(path: Path, summary: dict[str, object]) -> None:
    lines = [
        "# Stage 26 Calibrated Primitive Sequence",
        "",
        "Guarded Stage20 physical A/B primitive sequence only. Not full path following.",
        "",
    ]
    lines.extend(f"- {key}: `{value}`" for key, value in summary.items())
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def run_stage26(args: argparse.Namespace) -> int:
    fine = _load_json(Path(args.fine_calibration_json))
    turn = _load_json(Path(args.turn_calibration_json))
    primitive_map = build_primitive_map(fine, turn)
    plan = build_plan(args.sequence, primitive_map, max_abs_a=args.max_abs_a, max_abs_b=args.max_abs_b, max_ms=args.max_ms)

    out_dir = Path(args.out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    write_primitive_sequence(out_dir / "primitive_sequence_used.json", plan)

    dry_run = _parse_bool(args.dry_run, default=False)
    if dry_run:
        _write_summary(
            out_dir / "summary.md",
            build_summary([], sequence=args.sequence, primitive_count=len(plan), dry_run=True),
        )
        print(f"primitive_sequence_used_json={out_dir / 'primitive_sequence_used.json'}")
        print(f"summary_md={out_dir / 'summary.md'}")
        print("dry_run=true")
        print("ready_for_full_path_following=false")
        return 0

    try:
        import serial  # type: ignore[import-not-found]
    except ImportError as exc:
        raise RuntimeError("pyserial is not available; cannot run Stage 26 primitive sequence") from exc

    require_enter = _parse_bool(args.require_enter, default=True)
    interactive = _parse_bool(args.interactive_visible_motion, default=True)
    rows: list[dict[str, object]] = []
    raw_lines: list[str] = []

    with serial.Serial(args.port, baudrate=115200, timeout=0.5) as handle:
        for planned in plan:
            primitive_start_index = len(raw_lines)
            precondition_rows = _wait_for_precondition(handle, raw_lines, args.heartbeat_timeout_s)
            if not stage20_precondition_ready(precondition_rows):
                raise RuntimeError("Stage26 precondition failed before primitive")
            if require_enter:
                command = planned["stage20_command_text"] or planned["stop_command_text"]
                input(f"Press Enter for Stage26 {planned['primitive_name']} #{planned['primitive_index']}: {command} ")
            if planned["primitive_name"] == "stop":
                handle.write((str(planned["stop_command_text"]) + "\n").encode("ascii"))
                handle.flush()
                _wait_for_event(handle, raw_lines, {"STOP"}, args.event_timeout_s)
                primitive_rows = _serial_rows(raw_lines, primitive_start_index)
                row = row_from_primitive(
                    planned=planned,
                    rows=primitive_rows,
                    user_motion_report="none",
                    user_turn_angle_report="none",
                )
                rows.append(row)
                continue

            handle.write((str(planned["arm_command_text"]) + "\n").encode("ascii"))
            handle.flush()
            arm_rows = _wait_for_event(handle, raw_lines, {"ARM", "REJECT"}, args.event_timeout_s)
            if not any(row.get("event", "").upper() == "ARM" for row in arm_rows):
                primitive_rows = _serial_rows(raw_lines, primitive_start_index)
                row = row_from_primitive(
                    planned=planned,
                    rows=primitive_rows,
                    user_motion_report="unknown",
                    user_turn_angle_report="unknown",
                )
                rows.append(row)
                break

            handle.write((str(planned["stage20_command_text"]) + "\n").encode("ascii"))
            handle.flush()
            ack_rows = _wait_for_event(handle, raw_lines, {"ACK", "REJECT"}, args.event_timeout_s)
            if any(row.get("event", "").upper() == "REJECT" for row in ack_rows):
                handle.write((str(planned["stop_command_text"]) + "\n").encode("ascii"))
                handle.flush()
                _wait_for_event(handle, raw_lines, {"STOP"}, args.event_timeout_s)
                primitive_rows = _serial_rows(raw_lines, primitive_start_index)
                rows.append(row_from_primitive(
                    planned=planned,
                    rows=primitive_rows,
                    user_motion_report="unknown",
                    user_turn_angle_report="unknown",
                ))
                break

            pulse_ms = int(planned["pulse_ms"])
            _wait_for_event(handle, raw_lines, {"STOP", "PULSE_COMPLETE", "PULSE_DONE"}, max(args.event_timeout_s, pulse_ms / 1000.0 + 1.0))
            handle.write((str(planned["stop_command_text"]) + "\n").encode("ascii"))
            handle.flush()
            _wait_for_event(handle, raw_lines, {"STOP"}, args.event_timeout_s)
            primitive_rows = _serial_rows(raw_lines, primitive_start_index)

            if interactive:
                user_motion = _prompt_choice(
                    "Observed motion? [forward/backward/left/right/twitch/none/unknown] ",
                    MOTION_REPORTS,
                )
                if planned["primitive_name"] in {"turn_left", "turn_right"}:
                    user_turn = _prompt_choice(
                        "Observed turn angle? [none/twitch/small_under_15deg/medium_15_45deg/big_over_45deg/unknown] ",
                        TURN_REPORTS,
                    )
                else:
                    user_turn = "none"
            else:
                user_motion = "unknown"
                user_turn = "unknown"

            row = row_from_primitive(
                planned=planned,
                rows=primitive_rows,
                user_motion_report=user_motion,
                user_turn_angle_report=user_turn,
            )
            rows.append(row)
            if row["reject_seen"] is True or row["invalid_reason"] in INVALID_REJECT_REASONS:
                break
            if row["valid_primitive"] is not True:
                break

    _write_csv(out_dir / "stage26_calibrated_primitive_sequence.csv", rows)
    (out_dir / "raw_usbdbg.log").write_text("\n".join(raw_lines) + ("\n" if raw_lines else ""), encoding="utf-8")
    _write_summary(
        out_dir / "summary.md",
        build_summary(rows, sequence=args.sequence, primitive_count=len(plan), dry_run=False),
    )
    print("Stage 26 calibrated primitive sequence complete.")
    print(f"stage26_csv={out_dir / 'stage26_calibrated_primitive_sequence.csv'}")
    print(f"raw_usbdbg_log={out_dir / 'raw_usbdbg.log'}")
    print(f"summary_md={out_dir / 'summary.md'}")
    print(f"primitive_sequence_used_json={out_dir / 'primitive_sequence_used.json'}")
    print("ready_for_full_path_following=false")
    return 0


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Stage 26 calibrated Stage20 physical A/B primitive sequence runner.")
    parser.add_argument("--port", default="/dev/ttyACM0")
    parser.add_argument("--fine-calibration-json", required=True)
    parser.add_argument("--turn-calibration-json", required=True)
    parser.add_argument("--sequence", required=True)
    parser.add_argument("--require-enter", default="true")
    parser.add_argument("--interactive-visible-motion", default="true")
    parser.add_argument("--dry-run", default="false")
    parser.add_argument("--max-abs-a", type=float, default=DEFAULT_MAX_ABS_A)
    parser.add_argument("--max-abs-b", type=float, default=DEFAULT_MAX_ABS_B)
    parser.add_argument("--max-ms", type=int, default=DEFAULT_MAX_MS)
    parser.add_argument("--heartbeat-timeout-s", type=float, default=10.0)
    parser.add_argument("--event-timeout-s", type=float, default=5.0)
    parser.add_argument("--out-dir", required=True)
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    if args.max_abs_a <= 0 or args.max_abs_a > DEFAULT_MAX_ABS_A:
        raise SystemExit(f"--max-abs-a must be > 0 and <= {DEFAULT_MAX_ABS_A}")
    if args.max_abs_b <= 0 or args.max_abs_b > DEFAULT_MAX_ABS_B:
        raise SystemExit(f"--max-abs-b must be > 0 and <= {DEFAULT_MAX_ABS_B}")
    if args.max_ms <= 0 or args.max_ms > DEFAULT_MAX_MS:
        raise SystemExit(f"--max-ms must be > 0 and <= {DEFAULT_MAX_MS}")
    try:
        return run_stage26(args)
    except RuntimeError as exc:
        print(f"reason={exc}")
        print("ready_for_full_path_following=false")
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
