from __future__ import annotations

import argparse
import csv
import time
from pathlib import Path
from typing import Sequence

try:
    from tools import _bootstrap  # type: ignore  # noqa: F401
except ImportError:
    import _bootstrap  # type: ignore  # noqa: F401

from tools import station_path_package_tracker


PROBE_FIELDS = (
    "trial_index",
    "mode",
    "cmd",
    "pulse_ms",
    "seq",
    "arm_command_text",
    "stage18_command_text",
    "stop_command_text",
    "requested_left_cmd",
    "requested_right_cmd",
    "logical_left_cmd",
    "logical_right_cmd",
    "physical_a_cmd",
    "physical_b_cmd",
    "final_left_cmd",
    "final_right_cmd",
    "motor_write_called",
    "motor_backend",
    "motor_enable_state",
    "pwm_or_dynamixel_write_status",
    "firmware_active_target_source",
    "physical_path_following_enable",
    "allow_motor_output",
    "physical_output_active_seen",
    "physical_output_active_after_stop",
    "arm_ack_seen",
    "ack_seen",
    "stop_seen",
    "reject_seen",
    "reject_reason",
    "left_wheel_user_report",
    "right_wheel_user_report",
    "body_motion_user_report",
    "visible_motion_confirmed",
    "motor_power_or_mapping_suspect",
    "next_action",
    "ready_for_full_path_following",
)

WHEEL_RESPONSES = {"yes", "no", "twitch", "unknown"}
BODY_RESPONSES = {"none", "forward", "backward", "left", "right", "twitch", "unknown"}
VISIBLE_WHEEL = {"yes", "twitch"}
VISIBLE_BODY = {"forward", "backward", "left", "right", "twitch"}


def _parse_bool(value: object, default: bool = False) -> bool:
    if value is None:
        return default
    return str(value).strip().lower() in {"1", "true", "yes", "y", "ok", "active"}


def _parse_float(value: object, default: float = 0.0) -> float:
    try:
        return float(str(value).strip())
    except (TypeError, ValueError):
        return default


def _latest(rows: Sequence[dict[str, str]], key: str, default: str = "NA") -> str:
    for row in reversed(rows):
        if key in row:
            return row[key]
    return default


def parse_float_list(text: str, *, max_value: float) -> list[float]:
    values: list[float] = []
    for item in text.split(","):
        stripped = item.strip()
        if not stripped:
            continue
        value = float(stripped)
        if value <= 0 or value > max_value:
            raise ValueError(f"value {value} exceeds safe limit {max_value}")
        values.append(value)
    if not values:
        raise ValueError("empty command list")
    return values


def parse_int_list(text: str, *, max_value: int) -> list[int]:
    values: list[int] = []
    for item in text.split(","):
        stripped = item.strip()
        if not stripped:
            continue
        value = int(float(stripped))
        if value <= 0 or value > max_value:
            raise ValueError(f"value {value} exceeds safe limit {max_value}")
        values.append(value)
    if not values:
        raise ValueError("empty pulse list")
    return values


def command_for_probe_mode(mode: str, cmd: float) -> tuple[float, float]:
    if mode == "forward":
        return cmd, cmd
    if mode == "backward":
        return -cmd, -cmd
    if mode == "rotate_left":
        return -cmd, cmd
    if mode == "rotate_right":
        return cmd, -cmd
    if mode == "left_wheel_only":
        return cmd, 0.0
    if mode == "right_wheel_only":
        return 0.0, cmd
    raise ValueError(f"unsupported Stage18 probe mode: {mode}")


def modes_for_selection(mode: str) -> list[str]:
    if mode == "all":
        return ["forward", "backward", "rotate_left", "rotate_right", "left_wheel_only", "right_wheel_only"]
    return [mode]


def planned_probe_commands(*, seq: int, mode: str, cmd: float, pulse_ms: int) -> dict[str, object]:
    left, right = command_for_probe_mode(mode, cmd)
    return {
        "seq": seq,
        "mode": mode,
        "cmd": cmd,
        "pulse_ms": pulse_ms,
        "requested_left_cmd": left,
        "requested_right_cmd": right,
        "arm_command_text": f"STAGE18_ARM seq={seq}",
        "stage18_command_text": f"STAGE18_CMD seq={seq} left={left:.3f} right={right:.3f} ms={pulse_ms}",
        "stop_command_text": f"STAGE18_STOP seq={seq}",
    }


def visible_motion(left: str, right: str, body: str) -> bool:
    return left in VISIBLE_WHEEL or right in VISIBLE_WHEEL or body in VISIBLE_BODY


def _serial_rows(text: str) -> list[dict[str, str]]:
    return station_path_package_tracker.parse_usbdbg_rows(text)


def _wait_for_event(handle: object, raw_lines: list[str], wanted: set[str], timeout_s: float) -> list[dict[str, str]]:
    deadline = time.monotonic() + timeout_s
    start_index = len(raw_lines)
    while time.monotonic() < deadline:
        raw = handle.readline()
        if raw:
            line = raw.decode("utf-8", errors="replace").strip()
            print(line)
            raw_lines.append(line)
            rows = _serial_rows("\n".join(raw_lines[start_index:]))
            if any(row.get("event", "").upper() in wanted for row in rows):
                return _serial_rows("\n".join(raw_lines))
    return _serial_rows("\n".join(raw_lines))


def _prompt_choice(prompt: str, choices: set[str]) -> str:
    while True:
        response = input(prompt).strip().lower()
        if response in choices:
            return response
        print("Use one of: " + ", ".join(sorted(choices)))


def _active_rows(rows: Sequence[dict[str, str]]) -> list[dict[str, str]]:
    return [
        row
        for row in rows
        if row.get("event", "").upper() == "ACK"
        or row.get("stage16_cmd_state", "").upper() == "ACTIVE"
        or _parse_bool(row.get("physical_output_active"))
    ]


def row_from_trial(
    *,
    trial_index: int,
    planned: dict[str, object],
    rows: Sequence[dict[str, str]],
    left_report: str,
    right_report: str,
    body_report: str,
    upper_limit_trial: bool,
) -> dict[str, object]:
    active = _active_rows(rows)
    source = active[-1] if active else (rows[-1] if rows else {})
    final_left = _parse_float(_latest(rows, "final_left_cmd", "0"), default=0.0)
    final_right = _parse_float(_latest(rows, "final_right_cmd", "0"), default=0.0)
    active_source = source.get("active_target_source", _latest(rows, "active_target_source", "NONE"))
    output_after_stop = any(
        row.get("event", "").upper() == "STOP" and _parse_bool(row.get("physical_output_active"))
        for row in rows
    )
    motion = visible_motion(left_report, right_report, body_report)
    suspect = upper_limit_trial and not motion
    return {
        "trial_index": trial_index,
        "mode": planned["mode"],
        "cmd": planned["cmd"],
        "pulse_ms": planned["pulse_ms"],
        "seq": planned["seq"],
        "arm_command_text": planned["arm_command_text"],
        "stage18_command_text": planned["stage18_command_text"],
        "stop_command_text": planned["stop_command_text"],
        "requested_left_cmd": planned["requested_left_cmd"],
        "requested_right_cmd": planned["requested_right_cmd"],
        "logical_left_cmd": source.get("logical_left_cmd", "NA"),
        "logical_right_cmd": source.get("logical_right_cmd", "NA"),
        "physical_a_cmd": source.get("physical_a_cmd", "NA"),
        "physical_b_cmd": source.get("physical_b_cmd", "NA"),
        "final_left_cmd": f"{final_left:.3f}",
        "final_right_cmd": f"{final_right:.3f}",
        "motor_write_called": any(_parse_bool(row.get("motor_write_called")) for row in active),
        "motor_backend": source.get("motor_backend", "NA"),
        "motor_enable_state": source.get("motor_enable_state", "NA"),
        "pwm_or_dynamixel_write_status": source.get("pwm_or_dynamixel_write_status", "NA"),
        "firmware_active_target_source": active_source,
        "physical_path_following_enable": _latest(rows, "physical_path_following_enable", "false"),
        "allow_motor_output": _latest(rows, "allow_motor_output", "false"),
        "physical_output_active_seen": any(_parse_bool(row.get("physical_output_active")) for row in rows),
        "physical_output_active_after_stop": output_after_stop,
        "arm_ack_seen": any(row.get("event", "").upper() == "ARM" for row in rows),
        "ack_seen": any(row.get("event", "").upper() == "ACK" for row in rows),
        "stop_seen": any(row.get("event", "").upper() == "STOP" for row in rows),
        "reject_seen": any(row.get("event", "").upper() == "REJECT" for row in rows),
        "reject_reason": _latest(rows, "stage16_reject_reason", "NONE"),
        "left_wheel_user_report": left_report,
        "right_wheel_user_report": right_report,
        "body_motion_user_report": body_report,
        "visible_motion_confirmed": motion,
        "motor_power_or_mapping_suspect": suspect,
        "next_action": "check battery, motor driver enable, wiring, ground, and motor backend mapping" if suspect else "OK",
        "ready_for_full_path_following": False,
    }


def _write_csv(path: Path, rows: Sequence[dict[str, object]]) -> None:
    with path.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=PROBE_FIELDS)
        writer.writeheader()
        for row in rows:
            writer.writerow({field: row.get(field, "") for field in PROBE_FIELDS})


def build_summary(rows: Sequence[dict[str, object]]) -> dict[str, object]:
    visible_rows = [row for row in rows if row.get("visible_motion_confirmed") is True]
    first_visible = visible_rows[0] if visible_rows else {}
    suspect = bool(rows) and not visible_rows and rows[-1].get("motor_power_or_mapping_suspect") is True
    return {
        "stage18_heartbeat_seen": bool(rows),
        "arm_success_count": sum(1 for row in rows if row.get("arm_ack_seen") is True),
        "ack_count": sum(1 for row in rows if row.get("ack_seen") is True),
        "stop_count": sum(1 for row in rows if row.get("stop_seen") is True),
        "first_visible_motion_cmd": first_visible.get("cmd", "NA"),
        "first_visible_motion_ms": first_visible.get("pulse_ms", "NA"),
        "first_visible_motion_mode": first_visible.get("mode", "NA"),
        "motor_power_or_mapping_suspect": suspect,
        "next_action": "check battery, motor driver enable, wiring, ground, and motor backend mapping" if suspect else "OK",
        "final_stop_confirmed": bool(rows) and rows[-1].get("stop_seen") is True,
        "ready_for_full_path_following": False,
    }


def _write_summary(path: Path, summary: dict[str, object]) -> None:
    lines = [
        "# Stage 18 Motor Mapping Probe",
        "",
        "Bounded motor mapping and power diagnostic only. Not full path following.",
        "",
    ]
    for key, value in summary.items():
        lines.append(f"- {key}: `{value}`")
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def run_probe(args: argparse.Namespace) -> int:
    try:
        import serial  # type: ignore[import-not-found]
    except ImportError as exc:
        raise RuntimeError("pyserial is not available; cannot run Stage 18 motor mapping probe") from exc

    cmd_values = parse_float_list(args.cmd_list, max_value=0.08)
    pulse_values = parse_int_list(args.pulse_ms_list, max_value=800)
    selected_modes = modes_for_selection(args.mode)
    require_enter = _parse_bool(args.require_enter, default=True)
    continue_after_visible = _parse_bool(args.continue_after_visible, default=False)
    interactive = _parse_bool(args.interactive_visible_motion, default=True)
    max_cmd = max(cmd_values)
    max_ms = max(pulse_values)
    rows: list[dict[str, object]] = []
    raw_lines: list[str] = []
    trial_index = 0
    visible_found = False
    out_dir = Path(args.out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)

    with serial.Serial(args.port, baudrate=115200, timeout=0.5) as handle:
        _wait_for_event(handle, raw_lines, {"HEARTBEAT", "STOP"}, args.heartbeat_timeout_s)
        for mode in selected_modes:
            for cmd in cmd_values:
                for pulse_ms in pulse_values:
                    trial_index += 1
                    planned = planned_probe_commands(seq=trial_index, mode=mode, cmd=cmd, pulse_ms=pulse_ms)
                    if require_enter:
                        input(f"Press Enter for Stage18 {mode} cmd={cmd:.3f} pulse_ms={pulse_ms}. ")
                    handle.write((str(planned["arm_command_text"]) + "\n").encode("ascii"))
                    handle.flush()
                    arm_rows = _wait_for_event(handle, raw_lines, {"ARM", "REJECT"}, args.event_timeout_s)
                    if not any(row.get("event", "").upper() == "ARM" for row in arm_rows):
                        trial_rows = row_from_trial(
                            trial_index=trial_index,
                            planned=planned,
                            rows=arm_rows,
                            left_report="unknown",
                            right_report="unknown",
                            body_report="unknown",
                            upper_limit_trial=cmd >= max_cmd and pulse_ms >= max_ms,
                        )
                        rows.append(trial_rows)
                        continue
                    handle.write((str(planned["stage18_command_text"]) + "\n").encode("ascii"))
                    handle.flush()
                    _wait_for_event(handle, raw_lines, {"ACK", "REJECT"}, args.event_timeout_s)
                    _wait_for_event(handle, raw_lines, {"STOP"}, max(args.event_timeout_s, pulse_ms / 1000.0 + 1.0))
                    handle.write((str(planned["stop_command_text"]) + "\n").encode("ascii"))
                    handle.flush()
                    final_rows = _wait_for_event(handle, raw_lines, {"STOP"}, args.event_timeout_s)
                    if interactive:
                        left_report = _prompt_choice("Did left wheel move? [yes/no/twitch/unknown] ", WHEEL_RESPONSES)
                        right_report = _prompt_choice("Did right wheel move? [yes/no/twitch/unknown] ", WHEEL_RESPONSES)
                        body_report = _prompt_choice(
                            "Did rover body move? [none/forward/backward/left/right/twitch/unknown] ",
                            BODY_RESPONSES,
                        )
                    else:
                        left_report = "unknown"
                        right_report = "unknown"
                        body_report = "unknown"
                    row = row_from_trial(
                        trial_index=trial_index,
                        planned=planned,
                        rows=final_rows,
                        left_report=left_report,
                        right_report=right_report,
                        body_report=body_report,
                        upper_limit_trial=cmd >= max_cmd and pulse_ms >= max_ms,
                    )
                    rows.append(row)
                    if row["physical_output_active_after_stop"] is True:
                        raise RuntimeError("Stage18 output remained active after STOP; aborting")
                    if row["final_left_cmd"] != "0.000" or row["final_right_cmd"] != "0.000":
                        raise RuntimeError("Stage18 final wheel commands were nonzero; aborting")
                    visible_found = visible_found or row["visible_motion_confirmed"] is True
                    if visible_found and not continue_after_visible:
                        break
                if visible_found and not continue_after_visible:
                    break
            if visible_found and not continue_after_visible:
                break

    _write_csv(out_dir / "stage18_motor_mapping_probe.csv", rows)
    (out_dir / "raw_usbdbg.log").write_text("\n".join(raw_lines) + ("\n" if raw_lines else ""), encoding="utf-8")
    _write_summary(out_dir / "summary.md", build_summary(rows))
    print("Stage 18 motor mapping probe complete.")
    print(f"stage18_motor_mapping_probe_csv={out_dir / 'stage18_motor_mapping_probe.csv'}")
    print(f"raw_usbdbg_log={out_dir / 'raw_usbdbg.log'}")
    print(f"summary_md={out_dir / 'summary.md'}")
    print("ready_for_full_path_following=false")
    return 0


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Stage 18 bounded motor mapping and power probe.")
    parser.add_argument("--port", required=True)
    parser.add_argument("--mode", choices=("forward", "backward", "rotate_left", "rotate_right", "left_wheel_only", "right_wheel_only", "all"), default="all")
    parser.add_argument("--cmd-list", default="0.04,0.06,0.08")
    parser.add_argument("--pulse-ms-list", default="300,500,800")
    parser.add_argument("--require-enter", default="true")
    parser.add_argument("--interactive-visible-motion", default="true")
    parser.add_argument("--continue-after-visible", default="false")
    parser.add_argument("--heartbeat-timeout-s", type=float, default=10.0)
    parser.add_argument("--event-timeout-s", type=float, default=5.0)
    parser.add_argument("--out-dir", required=True)
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    return run_probe(build_parser().parse_args(argv))


if __name__ == "__main__":
    raise SystemExit(main())
