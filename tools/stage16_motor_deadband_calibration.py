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
from tools.station_guarded_path_crawl import format_stage16_command


CALIBRATION_FIELDS = (
    "trial_index",
    "mode",
    "cmd",
    "pulse_ms",
    "seq",
    "arm_command_text",
    "stage16_command_text",
    "stop_command_text",
    "arm_sent",
    "arm_ack_seen",
    "ack_seen",
    "stop_seen",
    "reject_seen",
    "reject_reason",
    "physical_output_active_seen",
    "physical_output_active_after_stop",
    "final_left_cmd",
    "final_right_cmd",
    "firmware_active_target_source",
    "physical_path_following_enable",
    "allow_motor_output",
    "visible_motion_response",
    "visible_motion_confirmed",
    "no_visible_motion_at_limit",
    "ready_for_full_path_following",
)

VISIBLE_RESPONSES = {"forward", "backward", "left", "right", "twitch"}
RESPONSE_CHOICES = VISIBLE_RESPONSES | {"none", "unknown"}


def _parse_bool(value: object, default: bool = False) -> bool:
    if value is None:
        return default
    return str(value).strip().lower() in {"1", "true", "yes", "y", "ok", "active"}


def _parse_float(value: object, default: float = 0.0) -> float:
    try:
        parsed = float(str(value).strip())
    except (TypeError, ValueError):
        return default
    return parsed


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


def command_for_mode(mode: str, cmd: float) -> tuple[float, float]:
    if mode == "forward":
        return cmd, cmd
    if mode == "backward":
        return -cmd, -cmd
    if mode == "rotate_left":
        return -cmd, cmd
    if mode == "rotate_right":
        return cmd, -cmd
    raise ValueError(f"unsupported calibration mode: {mode}")


def modes_for_selection(mode: str) -> list[str]:
    if mode == "all":
        return ["forward", "backward", "rotate_left", "rotate_right"]
    return [mode]


def planned_trial_commands(*, seq: int, mode: str, cmd: float, pulse_ms: int) -> dict[str, object]:
    left, right = command_for_mode(mode, cmd)
    return {
        "seq": seq,
        "mode": mode,
        "cmd": cmd,
        "pulse_ms": pulse_ms,
        "left": left,
        "right": right,
        "arm_command_text": f"STAGE16_ARM seq={seq}",
        "stage16_command_text": format_stage16_command(seq=seq, left=left, right=right, pulse_ms=pulse_ms),
        "stop_command_text": f"STAGE16_STOP seq={seq}",
    }


def visible_motion_confirmed(response: str) -> bool:
    return response.strip().lower() in VISIBLE_RESPONSES


def _write_csv(path: Path, rows: Sequence[dict[str, object]]) -> None:
    with path.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=CALIBRATION_FIELDS)
        writer.writeheader()
        for row in rows:
            writer.writerow({field: row.get(field, "") for field in CALIBRATION_FIELDS})


def build_summary(rows: Sequence[dict[str, object]]) -> dict[str, object]:
    visible_rows = [row for row in rows if row.get("visible_motion_confirmed") is True]
    first_visible = visible_rows[0] if visible_rows else {}
    no_visible_at_limit = bool(rows) and not visible_rows and rows[-1].get("no_visible_motion_at_limit") is True
    return {
        "stage16_heartbeat_seen": any(row.get("stage16_heartbeat_seen") is True for row in rows),
        "arm_success_count": sum(1 for row in rows if row.get("arm_ack_seen") is True),
        "ack_count": sum(1 for row in rows if row.get("ack_seen") is True),
        "stop_count": sum(1 for row in rows if row.get("stop_seen") is True),
        "first_visible_motion_cmd": first_visible.get("cmd", "NA"),
        "first_visible_motion_ms": first_visible.get("pulse_ms", "NA"),
        "first_visible_motion_mode": first_visible.get("mode", "NA"),
        "no_visible_motion_at_limit": no_visible_at_limit,
        "motor_power_or_mapping_suspect": no_visible_at_limit,
        "final_stop_confirmed": bool(rows) and rows[-1].get("stop_seen") is True,
        "ready_for_full_path_following": False,
    }


def _write_summary(path: Path, summary: dict[str, object]) -> None:
    lines = [
        "# Stage 16 Motor Deadband Calibration",
        "",
        "Bounded guarded pulses only. Not full path following.",
        "",
    ]
    for key, value in summary.items():
        lines.append(f"- {key}: `{value}`")
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


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


def _prompt_visible_motion() -> str:
    while True:
        response = input("Did you see motion? [none/forward/backward/left/right/twitch/unknown] ").strip().lower()
        if response in RESPONSE_CHOICES:
            return response
        print("Use one of: none, forward, backward, left, right, twitch, unknown")


def run_calibration(args: argparse.Namespace) -> int:
    try:
        import serial  # type: ignore[import-not-found]
    except ImportError as exc:
        raise RuntimeError("pyserial is not available; cannot run Stage 16 deadband calibration") from exc

    cmd_values = parse_float_list(args.cmd_list, max_value=0.040)
    pulse_values = parse_int_list(args.pulse_ms_list, max_value=150)
    selected_modes = modes_for_selection(args.mode)
    require_enter = _parse_bool(args.require_enter, default=True)
    interactive_visible = _parse_bool(args.interactive_visible_motion, default=True)
    continue_after_visible = _parse_bool(args.continue_after_visible, default=False)

    out_dir = Path(args.out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    rows: list[dict[str, object]] = []
    raw_lines: list[str] = []
    trial_index = 0
    visible_found = False

    with serial.Serial(args.port, baudrate=115200, timeout=0.5) as handle:
        heartbeat_rows = _wait_for_event(handle, raw_lines, {"HEARTBEAT", "STOP"}, args.heartbeat_timeout_s)
        heartbeat_seen = any(
            _parse_bool(row.get("stage16_guarded_crawl")) and row.get("event", "").upper() in {"HEARTBEAT", "STOP"}
            for row in heartbeat_rows
        )
        for mode in selected_modes:
            for cmd in cmd_values:
                for pulse_ms in pulse_values:
                    trial_index += 1
                    planned = planned_trial_commands(seq=trial_index, mode=mode, cmd=cmd, pulse_ms=pulse_ms)
                    if require_enter:
                        input(
                            "Press Enter for bounded Stage16 deadband pulse "
                            f"{mode} cmd={cmd:.3f} pulse_ms={pulse_ms}. "
                        )

                    handle.write((str(planned["arm_command_text"]) + "\n").encode("ascii"))
                    handle.flush()
                    arm_rows = _wait_for_event(handle, raw_lines, {"ARM", "REJECT"}, args.event_timeout_s)
                    arm_ack_seen = any(row.get("event", "").upper() == "ARM" for row in arm_rows)
                    reject_seen = any(row.get("event", "").upper() == "REJECT" for row in arm_rows)

                    ack_seen = False
                    stop_seen = False
                    physical_seen = False
                    if arm_ack_seen:
                        handle.write((str(planned["stage16_command_text"]) + "\n").encode("ascii"))
                        handle.flush()
                        ack_rows = _wait_for_event(handle, raw_lines, {"ACK", "REJECT"}, args.event_timeout_s)
                        ack_seen = any(row.get("event", "").upper() == "ACK" for row in ack_rows)
                        reject_seen = reject_seen or any(row.get("event", "").upper() == "REJECT" for row in ack_rows)
                        physical_seen = any(_parse_bool(row.get("physical_output_active")) for row in ack_rows)
                        stop_rows = _wait_for_event(handle, raw_lines, {"STOP"}, max(args.event_timeout_s, pulse_ms / 1000.0 + 1.0))
                        stop_seen = any(row.get("event", "").upper() == "STOP" for row in stop_rows)
                    handle.write((str(planned["stop_command_text"]) + "\n").encode("ascii"))
                    handle.flush()
                    final_rows = _wait_for_event(handle, raw_lines, {"STOP"}, args.event_timeout_s)
                    stop_seen = stop_seen or any(row.get("event", "").upper() == "STOP" for row in final_rows)
                    final_left = _parse_float(_latest(final_rows, "final_left_cmd", "0"), default=0.0)
                    final_right = _parse_float(_latest(final_rows, "final_right_cmd", "0"), default=0.0)
                    output_after_stop = any(
                        _parse_bool(row.get("physical_output_active")) and row.get("event", "").upper() == "STOP"
                        for row in final_rows
                    )
                    if output_after_stop:
                        raise RuntimeError("Stage16 output remained active after STOP; aborting calibration")

                    response = _prompt_visible_motion() if interactive_visible and ack_seen else "unknown"
                    visible_confirmed = visible_motion_confirmed(response)
                    visible_found = visible_found or visible_confirmed
                    row = {
                        "trial_index": trial_index,
                        "mode": mode,
                        "cmd": cmd,
                        "pulse_ms": pulse_ms,
                        "seq": planned["seq"],
                        "arm_command_text": planned["arm_command_text"],
                        "stage16_command_text": planned["stage16_command_text"],
                        "stop_command_text": planned["stop_command_text"],
                        "arm_sent": True,
                        "arm_ack_seen": arm_ack_seen,
                        "ack_seen": ack_seen,
                        "stop_seen": stop_seen,
                        "reject_seen": reject_seen,
                        "reject_reason": _latest(final_rows, "stage16_reject_reason", "NONE"),
                        "physical_output_active_seen": physical_seen,
                        "physical_output_active_after_stop": output_after_stop,
                        "final_left_cmd": f"{final_left:.3f}",
                        "final_right_cmd": f"{final_right:.3f}",
                        "firmware_active_target_source": _latest(final_rows, "active_target_source", "NONE"),
                        "physical_path_following_enable": _latest(final_rows, "physical_path_following_enable", "false"),
                        "allow_motor_output": _latest(final_rows, "allow_motor_output", "false"),
                        "visible_motion_response": response,
                        "visible_motion_confirmed": visible_confirmed,
                        "no_visible_motion_at_limit": False,
                        "ready_for_full_path_following": False,
                        "stage16_heartbeat_seen": heartbeat_seen,
                    }
                    rows.append(row)
                    if visible_confirmed and not continue_after_visible:
                        break
                if visible_found and not continue_after_visible:
                    break
            if visible_found and not continue_after_visible:
                break

    if rows and not visible_found:
        rows[-1]["no_visible_motion_at_limit"] = True
    _write_csv(out_dir / "deadband_calibration.csv", rows)
    (out_dir / "raw_usbdbg.log").write_text("\n".join(raw_lines) + ("\n" if raw_lines else ""), encoding="utf-8")
    _write_summary(out_dir / "summary.md", build_summary(rows))
    print("Stage 16 motor deadband calibration complete.")
    print(f"deadband_calibration_csv={out_dir / 'deadband_calibration.csv'}")
    print(f"raw_usbdbg_log={out_dir / 'raw_usbdbg.log'}")
    print(f"summary_md={out_dir / 'summary.md'}")
    print("ready_for_full_path_following=false")
    return 0


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Stage 16 guarded motor deadband calibration.")
    parser.add_argument("--port", required=True)
    parser.add_argument("--mode", choices=("forward", "backward", "rotate_left", "rotate_right", "all"), default="forward")
    parser.add_argument("--cmd-list", default="0.015,0.020,0.025,0.030,0.035,0.040")
    parser.add_argument("--pulse-ms-list", default="80,100,120,150")
    parser.add_argument("--require-enter", default="true")
    parser.add_argument("--interactive-visible-motion", default="true")
    parser.add_argument("--continue-after-visible", default="false")
    parser.add_argument("--heartbeat-timeout-s", type=float, default=10.0)
    parser.add_argument("--event-timeout-s", type=float, default=5.0)
    parser.add_argument("--out-dir", required=True)
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    return run_calibration(args)


if __name__ == "__main__":
    raise SystemExit(main())
