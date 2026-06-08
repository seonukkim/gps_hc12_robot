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

from tools import station_path_package_tracker
from tools.path_no_motion_validation import PathPackageResolutionError, resolve_path_package


DEFAULT_A_CMD = 0.30
DEFAULT_B_CMD = 0.0
DEFAULT_PULSE_MS = 800
DEFAULT_MAX_ABS_A = 0.35
DEFAULT_MAX_ABS_B = 0.25
DEFAULT_MAX_MS = 1000
VISIBLE_FORWARD_REPORTS = {"forward", "twitch"}
BODY_REPORTS = {"forward", "twitch", "none", "backward", "unknown"}

STAGE22_FIELDS = (
    "pulse_index",
    "seq",
    "path_package_loaded",
    "fine_calibration_json",
    "straight_only",
    "turn_disabled",
    "selected_a_cmd",
    "selected_b_cmd",
    "selected_pulse_ms",
    "max_abs_a",
    "max_abs_b",
    "max_ms",
    "command_within_firmware_limits",
    "arm_command_text",
    "stage20_command_text",
    "stop_command_text",
    "arm_ack_seen",
    "ack_seen",
    "stop_seen",
    "reject_seen",
    "reject_reason",
    "physical_output_active_seen",
    "physical_output_active_after_stop",
    "final_left_cmd",
    "final_right_cmd",
    "final_left_cmd_zero",
    "final_right_cmd_zero",
    "body_motion_user_report",
    "visible_forward_or_twitch",
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


def parse_stage20_ready(rows: Sequence[dict[str, str]]) -> tuple[bool, bool]:
    heartbeat_seen = False
    for row in rows:
        if row.get("event", "").upper() != "HEARTBEAT":
            continue
        if _parse_bool(row.get("stage20_physical_ab_guarded_crawl")) is not True:
            continue
        heartbeat_seen = True
        if (
            _parse_bool(row.get("stage20_firmware_ready")) is True
            and _parse_bool(row.get("physical_path_following_enable")) is False
            and _parse_bool(row.get("allow_motor_output")) is False
        ):
            return True, heartbeat_seen
    return False, heartbeat_seen


def _load_fine_calibration(path: Path | None) -> dict[str, object]:
    if path is None:
        return {}
    return json.loads(path.read_text(encoding="utf-8"))


def select_straight_command(
    fine_calibration: dict[str, object] | None,
    *,
    a_cmd: float | None = None,
    pulse_ms: int | None = None,
) -> tuple[float, float, int]:
    calibration = fine_calibration or {}
    selected_a = a_cmd
    if selected_a is None:
        for key in ("stage22_forward_a_cmd", "stage21_forward_a_cmd", "forward_recommended_crawl_a_cmd"):
            parsed = _optional_float(calibration.get(key))
            if parsed is not None and parsed > 0:
                selected_a = parsed
                break
    if selected_a is None:
        selected_a = DEFAULT_A_CMD

    selected_ms = pulse_ms
    if selected_ms is None:
        for key in ("stage22_forward_pulse_ms", "stage21_forward_pulse_ms", "forward_recommended_crawl_pulse_ms"):
            parsed = _optional_float(calibration.get(key))
            if parsed is not None and parsed > 0:
                selected_ms = int(parsed)
                break
    if selected_ms is None:
        selected_ms = DEFAULT_PULSE_MS
    return selected_a, DEFAULT_B_CMD, selected_ms


def planned_pulse(
    *,
    pulse_index: int,
    seq: int,
    a_cmd: float,
    pulse_ms: int,
    fine_calibration_json: str = "",
    max_abs_a: float = DEFAULT_MAX_ABS_A,
    max_abs_b: float = DEFAULT_MAX_ABS_B,
    max_ms: int = DEFAULT_MAX_MS,
) -> dict[str, object]:
    b_cmd = DEFAULT_B_CMD
    within_limits = abs(a_cmd) <= max_abs_a + 1e-12 and abs(b_cmd) <= max_abs_b + 1e-12 and pulse_ms <= max_ms
    return {
        "pulse_index": pulse_index,
        "seq": seq,
        "path_package_loaded": True,
        "fine_calibration_json": fine_calibration_json,
        "straight_only": True,
        "turn_disabled": True,
        "selected_a_cmd": a_cmd,
        "selected_b_cmd": b_cmd,
        "selected_pulse_ms": pulse_ms,
        "max_abs_a": max_abs_a,
        "max_abs_b": max_abs_b,
        "max_ms": max_ms,
        "command_within_firmware_limits": within_limits,
        "arm_command_text": f"STAGE20_ARM seq={seq}",
        "stage20_command_text": f"STAGE20_CMD seq={seq} a={a_cmd:.3f} b={b_cmd:.3f} ms={pulse_ms}",
        "stop_command_text": f"STAGE20_STOP seq={seq}",
        "ready_for_full_path_following": False,
    }


def _active_rows(rows: Sequence[dict[str, str]]) -> list[dict[str, str]]:
    return [
        row
        for row in rows
        if row.get("event", "").upper() == "ACK"
        or row.get("stage20_cmd_state", "").upper() == "ACTIVE"
        or _parse_bool(row.get("physical_output_active"))
    ]


def row_from_pulse(
    *,
    planned: dict[str, object],
    rows: Sequence[dict[str, str]],
    body_motion_user_report: str,
) -> dict[str, object]:
    active = _active_rows(rows)
    final_left = _optional_float(_latest(rows, "final_left_cmd", "0.0")) or 0.0
    final_right = _optional_float(_latest(rows, "final_right_cmd", "0.0")) or 0.0
    output_after_stop = any(
        row.get("event", "").upper() == "STOP" and _parse_bool(row.get("physical_output_active"))
        for row in rows
    )
    physical_seen = any(_parse_bool(row.get("physical_output_active")) for row in rows)
    row = {
        **planned,
        "arm_ack_seen": any(row.get("event", "").upper() == "ARM" for row in rows),
        "ack_seen": any(row.get("event", "").upper() == "ACK" for row in rows),
        "stop_seen": any(row.get("event", "").upper() == "STOP" for row in rows),
        "reject_seen": any(row.get("event", "").upper() == "REJECT" for row in rows),
        "reject_reason": _latest(rows, "stage16_reject_reason", "NONE"),
        "physical_output_active_seen": physical_seen or bool(active),
        "physical_output_active_after_stop": output_after_stop,
        "final_left_cmd": f"{final_left:.3f}",
        "final_right_cmd": f"{final_right:.3f}",
        "final_left_cmd_zero": abs(final_left) <= 1e-9,
        "final_right_cmd_zero": abs(final_right) <= 1e-9,
        "body_motion_user_report": body_motion_user_report,
        "visible_forward_or_twitch": body_motion_user_report in VISIBLE_FORWARD_REPORTS,
        "ready_for_full_path_following": False,
    }
    return row


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


def _prompt_body_motion() -> str:
    while True:
        answer = input("Visible motion after pulse? [forward/twitch/none/backward/unknown] ").strip().lower()
        if answer in BODY_REPORTS:
            return answer
        print("Use one of: " + ", ".join(sorted(BODY_REPORTS)))


def _write_csv(path: Path, rows: Sequence[dict[str, object]]) -> None:
    with path.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=STAGE22_FIELDS)
        writer.writeheader()
        for row in rows:
            writer.writerow({field: row.get(field, "") for field in STAGE22_FIELDS})


def build_summary(rows: Sequence[dict[str, object]], *, repeat_count: int, selected_a: float, selected_b: float, pulse_ms: int) -> dict[str, object]:
    successful = [row for row in rows if row.get("ack_seen") is True and row.get("stop_seen") is True and row.get("reject_seen") is not True]
    visible_forward = [row for row in rows if row.get("body_motion_user_report") == "forward"]
    visible_twitch = [row for row in rows if row.get("body_motion_user_report") == "twitch"]
    final = rows[-1] if rows else {}
    reject_count = sum(1 for row in rows if row.get("reject_seen") is True)
    required_visible_count = min(2, repeat_count)
    ready_stage23 = (
        len(visible_forward) + len(visible_twitch) >= required_visible_count
        and len(successful) == repeat_count
        and reject_count == 0
    )
    return {
        "repeat_count": repeat_count,
        "successful_pulse_count": len(successful),
        "visible_forward_count": len(visible_forward),
        "visible_twitch_count": len(visible_twitch),
        "reject_count": reject_count,
        "final_stop_confirmed": final.get("stop_seen") is True and final.get("final_left_cmd_zero") is True and final.get("final_right_cmd_zero") is True,
        "selected_a_cmd": selected_a,
        "selected_b_cmd": selected_b,
        "selected_pulse_ms": pulse_ms,
        "ready_for_stage23_turn_calibration": ready_stage23,
        "ready_for_full_path_following": False,
    }


def _write_summary(path: Path, summary: dict[str, object]) -> None:
    lines = [
        "# Stage 22 Straight Segment Repeat",
        "",
        "Repeated straight-only Stage20 physical A/B pulses. Not turning and not full path following.",
        "",
    ]
    lines.extend(f"- {key}: `{value}`" for key, value in summary.items())
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def run_stage22(args: argparse.Namespace) -> int:
    try:
        import serial  # type: ignore[import-not-found]
    except ImportError as exc:
        raise RuntimeError("pyserial is not available; cannot run Stage 22 straight segment") from exc

    selected = resolve_path_package(args.path_package)
    json.loads(selected.read_text(encoding="utf-8"))
    fine_path = Path(args.fine_calibration_json) if args.fine_calibration_json else None
    fine = _load_fine_calibration(fine_path)
    selected_a, selected_b, selected_ms = select_straight_command(fine, a_cmd=args.a_cmd, pulse_ms=args.pulse_ms)
    if abs(selected_a) > args.max_abs_a + 1e-12:
        raise SystemExit("selected A command exceeds --max-abs-a")
    if abs(selected_b) > args.max_abs_b + 1e-12:
        raise SystemExit("selected B command exceeds --max-abs-b")
    if selected_ms > args.max_ms:
        raise SystemExit("selected pulse_ms exceeds --max-ms")

    out_dir = Path(args.out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    require_enter = _parse_bool(args.require_enter, default=True)
    rows: list[dict[str, object]] = []
    raw_lines: list[str] = []
    fine_json = "" if fine_path is None else str(fine_path)

    with serial.Serial(args.port, baudrate=115200, timeout=0.5) as handle:
        ready_rows = _wait_for_event(handle, raw_lines, {"HEARTBEAT", "STOP"}, args.heartbeat_timeout_s)
        ready, _heartbeat = parse_stage20_ready(ready_rows)
        if _parse_bool(args.require_stage20_ready, default=True) and not ready:
            raise RuntimeError("FIRMWARE_NOT_READY_NEEDS_UPLOAD")
        for pulse_index in range(1, args.repeat_count + 1):
            pulse_start_index = len(raw_lines)
            planned = planned_pulse(
                pulse_index=pulse_index,
                seq=pulse_index,
                a_cmd=selected_a,
                pulse_ms=selected_ms,
                fine_calibration_json=fine_json,
                max_abs_a=args.max_abs_a,
                max_abs_b=args.max_abs_b,
                max_ms=args.max_ms,
            )
            if planned["command_within_firmware_limits"] is not True:
                rows.append(row_from_pulse(planned=planned, rows=[], body_motion_user_report="unknown"))
                break
            if require_enter:
                input(f"Press Enter for Stage22 straight pulse {pulse_index}/{args.repeat_count}: {planned['stage20_command_text']} ")
            handle.write((str(planned["arm_command_text"]) + "\n").encode("ascii"))
            handle.flush()
            arm_rows = _wait_for_event(handle, raw_lines, {"ARM", "REJECT"}, args.event_timeout_s)
            if not any(row.get("event", "").upper() == "ARM" for row in arm_rows):
                pulse_rows = _serial_rows(raw_lines, pulse_start_index)
                row = row_from_pulse(planned=planned, rows=pulse_rows, body_motion_user_report="unknown")
                rows.append(row)
                break
            handle.write((str(planned["stage20_command_text"]) + "\n").encode("ascii"))
            handle.flush()
            ack_rows = _wait_for_event(handle, raw_lines, {"ACK", "REJECT"}, args.event_timeout_s)
            if any(row.get("event", "").upper() == "REJECT" for row in ack_rows):
                handle.write((str(planned["stop_command_text"]) + "\n").encode("ascii"))
                handle.flush()
                _wait_for_event(handle, raw_lines, {"STOP"}, args.event_timeout_s)
                pulse_rows = _serial_rows(raw_lines, pulse_start_index)
                rows.append(row_from_pulse(planned=planned, rows=pulse_rows, body_motion_user_report="unknown"))
                break
            _wait_for_event(handle, raw_lines, {"STOP", "PULSE_COMPLETE", "PULSE_DONE"}, max(args.event_timeout_s, selected_ms / 1000.0 + 1.0))
            handle.write((str(planned["stop_command_text"]) + "\n").encode("ascii"))
            handle.flush()
            _wait_for_event(handle, raw_lines, {"STOP"}, args.event_timeout_s)
            pulse_rows = _serial_rows(raw_lines, pulse_start_index)
            motion = _prompt_body_motion() if _parse_bool(args.interactive_visible_motion, default=True) else "unknown"
            row = row_from_pulse(planned=planned, rows=pulse_rows, body_motion_user_report=motion)
            rows.append(row)
            if row["reject_seen"] is True:
                break
            if row["physical_output_active_after_stop"] is True:
                raise RuntimeError("Stage22 output remained active after STOP; aborting")
            if row["final_left_cmd_zero"] is not True or row["final_right_cmd_zero"] is not True:
                raise RuntimeError("Stage22 final wheel commands were nonzero; aborting")
            if row["ack_seen"] is True and row["stop_seen"] is not True:
                raise RuntimeError("Stage22 ACK was seen without STOP; aborting")

    _write_csv(out_dir / "stage22_straight_segment.csv", rows)
    (out_dir / "raw_usbdbg.log").write_text("\n".join(raw_lines) + ("\n" if raw_lines else ""), encoding="utf-8")
    _write_summary(
        out_dir / "summary.md",
        build_summary(rows, repeat_count=args.repeat_count, selected_a=selected_a, selected_b=selected_b, pulse_ms=selected_ms),
    )
    print("Stage 22 straight segment complete.")
    print(f"selected_path_package={selected}")
    print(f"stage22_csv={out_dir / 'stage22_straight_segment.csv'}")
    print(f"raw_usbdbg_log={out_dir / 'raw_usbdbg.log'}")
    print(f"summary_md={out_dir / 'summary.md'}")
    print("ready_for_full_path_following=false")
    return 0


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Stage 22 repeated straight-only Stage20 physical A/B pulses.")
    parser.add_argument("--path-package", default="latest")
    parser.add_argument("--port", required=True)
    parser.add_argument("--fine-calibration-json", required=True)
    parser.add_argument("--repeat-count", type=int, default=3)
    parser.add_argument("--a-cmd", type=float)
    parser.add_argument("--pulse-ms", type=int)
    parser.add_argument("--max-abs-a", type=float, default=DEFAULT_MAX_ABS_A)
    parser.add_argument("--max-abs-b", type=float, default=DEFAULT_MAX_ABS_B)
    parser.add_argument("--max-ms", type=int, default=DEFAULT_MAX_MS)
    parser.add_argument("--require-enter", default="true")
    parser.add_argument("--require-stage20-ready", default="true")
    parser.add_argument("--interactive-visible-motion", default="true")
    parser.add_argument("--heartbeat-timeout-s", type=float, default=10.0)
    parser.add_argument("--event-timeout-s", type=float, default=5.0)
    parser.add_argument("--out-dir", required=True)
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    if args.repeat_count <= 0:
        raise SystemExit("--repeat-count must be > 0")
    if args.max_abs_a <= 0 or args.max_abs_a > 0.35:
        raise SystemExit("--max-abs-a must be > 0 and <= 0.35")
    if args.max_abs_b <= 0 or args.max_abs_b > 0.25:
        raise SystemExit("--max-abs-b must be > 0 and <= 0.25")
    if args.max_ms <= 0 or args.max_ms > 1000:
        raise SystemExit("--max-ms must be > 0 and <= 1000")
    try:
        return run_stage22(args)
    except RuntimeError as exc:
        print(f"reason={exc}")
        print("ready_for_full_path_following=false")
        return 2
    except PathPackageResolutionError as exc:
        print(f"provided_path_package={exc.provided}")
        print("file_exists=false")
        print("nearest_candidates:")
        for candidate in exc.candidates:
            print(f"- {candidate}")
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
