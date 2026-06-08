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

RC_INPUT_ABSENT_ACTION = (
    "Check RC receiver power; check receiver signal wire to OpenRB RC input; "
    "check whether receiver output mode is PPM/SBUS/PWM and firmware input mode matches; "
    "check mode channel index / CH mapping; check transmitter-receiver binding; "
    "if using individual PWM channels instead of PPM, firmware must read the correct pins; "
    "run manual-rc --diagnose-only true after changing wiring/binding."
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


def parse_rows(text: str) -> list[dict[str, str]]:
    return station_path_package_tracker.parse_usbdbg_rows(text)


def _row_input_zero(row: dict[str, str]) -> bool:
    keys = [f"raw_ch{i}_us" for i in range(1, 9)] + ["steer_us", "throttle_us", "mode_us"]
    present = [key for key in keys if key in row]
    if not present:
        return False
    return all(abs(_parse_float(row.get(key)) or 0.0) <= 1e-3 for key in present)


def _row_input_nonzero(row: dict[str, str]) -> bool:
    keys = [f"raw_ch{i}_us" for i in range(1, 9)] + ["steer_us", "throttle_us", "mode_us"]
    return any(abs(_parse_float(row.get(key)) or 0.0) > 1e-3 for key in keys)


def evaluate_rows(rows: Sequence[dict[str, str]]) -> dict[str, object]:
    rc_ok = any(_parse_bool(row.get("rc_ok")) is True for row in rows)
    input_zero_rows = [row for row in rows if _row_input_zero(row)]
    input_nonzero_rows = [row for row in rows if _row_input_nonzero(row)]
    rows_with_input_fields = [
        row for row in rows
        if any(key in row for key in [f"raw_ch{i}_us" for i in range(1, 9)] + ["steer_us", "throttle_us", "mode_us"])
    ]
    raw_channel_nonzero_seen = any(
        abs(_parse_float(row.get(f"raw_ch{i}_us")) or 0.0) > 1e-3
        for row in rows
        for i in range(1, 9)
    )
    rc_input_detected = bool(input_nonzero_rows)
    input_absent = bool(rows) and bool(rows_with_input_fields) and len(input_zero_rows) / max(1, len(rows_with_input_fields)) >= 0.8 and not rc_input_detected
    manual_rows = [
        row for row in rows
        if row.get("mode") == "MANUAL"
        or row.get("control_source") == "RC_MANUAL"
        or row.get("active_target_source") == "manual_rc"
    ]
    nonzero_final_rows = [
        row for row in rows
        if abs(_parse_float(row.get("final_left_cmd")) or 0.0) > 1e-3
        or abs(_parse_float(row.get("final_right_cmd")) or 0.0) > 1e-3
    ]
    nonzero_manual_rows = [
        row for row in rows
        if abs(_parse_float(row.get("manual_forward_cmd")) or 0.0) > 1e-3
        or abs(_parse_float(row.get("manual_turn_cmd")) or 0.0) > 1e-3
    ]
    physical_a_nonzero_rows = [
        row for row in rows
        if abs(_parse_float(row.get("physical_a_cmd", row.get("output_left_pin_cmd"))) or 0.0) > 1e-3
    ]
    physical_b_nonzero_rows = [
        row for row in rows
        if abs(_parse_float(row.get("physical_b_cmd", row.get("output_right_pin_cmd"))) or 0.0) > 1e-3
    ]
    motor_write_called = any(_parse_bool(row.get("motor_write_called")) is True for row in rows)
    physical_active = any(_parse_bool(row.get("physical_output_active")) is True for row in rows)
    autonomous_target_rows = [
        row for row in rows
        if row.get("active_target_source") not in {"", None, "NA", "NONE", "manual_rc"}
    ]
    neutral_zero = any(
        _parse_bool(row.get("neutral_ok")) is True
        and abs(_parse_float(row.get("final_left_cmd")) or 0.0) <= 1e-3
        and abs(_parse_float(row.get("final_right_cmd")) or 0.0) <= 1e-3
        for row in rows
    )
    stage20_seen = any("stage20_physical_ab_guarded_crawl" in row for row in rows)
    pass_ready = (
        rc_ok
        and bool(manual_rows)
        and bool(nonzero_final_rows)
        and (physical_active or motor_write_called)
        and neutral_zero
        and not autonomous_target_rows
    )
    if pass_ready:
        reason = "MANUAL_RC_PASS"
    elif not rows:
        reason = "SERIAL_ERROR"
    elif input_absent:
        reason = "RC_INPUT_ABSENT"
    elif not rc_ok:
        reason = "RC_INVALID_OR_UNSTABLE"
    elif not manual_rows:
        reason = "RC_CHANNELS_PRESENT_BUT_MODE_NOT_MANUAL"
    elif nonzero_manual_rows and not nonzero_final_rows:
        reason = "MOTOR_OUTPUT_BLOCKED"
    elif not nonzero_manual_rows or not physical_active:
        reason = "RC_CHANNELS_PRESENT_BUT_NO_MOTOR_OUTPUT"
    else:
        reason = "MOTOR_OUTPUT_BLOCKED"
    next_action = {
        "MANUAL_RC_PASS": "Manual RC passthrough is verified.",
        "RC_INPUT_ABSENT": RC_INPUT_ABSENT_ACTION,
        "RC_INVALID_OR_UNSTABLE": "Check receiver signal quality, transmitter binding, channel mapping, and input mode.",
        "RC_CHANNELS_PRESENT_BUT_MODE_NOT_MANUAL": "Set the transmitter mode switch to MANUAL / AUTO OFF and verify mode channel mapping.",
        "RC_CHANNELS_PRESENT_BUT_NO_MOTOR_OUTPUT": "Move sticks during the validation window and inspect manual command telemetry.",
        "MOTOR_OUTPUT_BLOCKED": "Inspect firmware manual RC priority, STOP latch handling, and motor output gating.",
        "SERIAL_ERROR": "Check USB serial connection and rerun manual-rc diagnostics.",
    }.get(reason, "Inspect manual RC telemetry and wiring before rerunning.")
    reasons: list[str] = []
    if not rc_ok:
        reasons.append("RC_NOT_OK")
    if not manual_rows:
        reasons.append("MANUAL_RC_SOURCE_NOT_SEEN")
    if not nonzero_final_rows:
        reasons.append("NO_NONZERO_MANUAL_FINAL_COMMANDS")
    if not physical_active:
        reasons.append("PHYSICAL_OUTPUT_NOT_SEEN_DURING_MANUAL")
    if not neutral_zero:
        reasons.append("NO_NEUTRAL_ZERO_RETURN")
    if autonomous_target_rows:
        reasons.append("AUTONOMOUS_TARGET_SOURCE_SEEN")
    return {
        "row_count": len(rows),
        "rc_input_detected": rc_input_detected,
        "rc_ok": rc_ok,
        "rc_ok_seen": rc_ok,
        "manual_mode_seen": bool(manual_rows),
        "control_source_rc_manual_seen": any(row.get("control_source") == "RC_MANUAL" for row in rows),
        "raw_channel_nonzero_seen": raw_channel_nonzero_seen,
        "nonzero_manual_command_seen": bool(nonzero_manual_rows),
        "physical_a_nonzero_seen": bool(physical_a_nonzero_rows),
        "physical_b_nonzero_seen": bool(physical_b_nonzero_rows),
        "nonzero_final_motor_command_seen": bool(nonzero_final_rows),
        "manual_rc_rows": len(manual_rows),
        "manual_nonzero_command_rows": len(nonzero_manual_rows),
        "nonzero_final_command_rows": len(nonzero_final_rows),
        "physical_output_active_seen": physical_active,
        "motor_write_called_seen": motor_write_called,
        "neutral_zero_return_seen": neutral_zero,
        "autonomous_target_source_rows": len(autonomous_target_rows),
        "guarded_pulse_mode_seen": stage20_seen,
        "manual_rc_passthrough_pass": pass_ready,
        "manual_rc_passthrough_ok": pass_ready,
        "validation_success": pass_ready,
        "success": pass_ready,
        "reason": reason,
        "reasons": reasons or ["OK"],
        "next_recommended_action": next_action,
        "station_command_used": False,
        "ready_for_full_path_following": False,
    }


def write_summary(out_dir: Path, summary: dict[str, object]) -> None:
    (out_dir / "manual_rc_passthrough_summary.json").write_text(json.dumps(summary, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    (out_dir / "summary.json").write_text(json.dumps(summary, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    lines = ["# Manual RC Passthrough Validation", "", "Telemetry-only validation. No station motor commands are sent.", ""]
    lines.extend(f"- {key}: `{value}`" for key, value in summary.items())
    (out_dir / "summary.md").write_text("\n".join(lines) + "\n", encoding="utf-8")


def write_rows_csv(path: Path, rows: Sequence[dict[str, str]]) -> None:
    if not rows:
        path.write_text("", encoding="utf-8")
        return
    fieldnames: list[str] = []
    for row in rows:
        for key in row.keys():
            if key not in fieldnames:
                fieldnames.append(key)
    with path.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        for row in rows:
            writer.writerow(row)


def run(args: argparse.Namespace) -> int:
    out_dir = Path(args.out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    raw_lines: list[str] = []
    if args.log:
        raw_lines = Path(args.log).read_text(encoding="utf-8").splitlines()
    else:
        try:
            import serial  # type: ignore[import-not-found]
        except ImportError as exc:
            raise RuntimeError("pyserial is not available; cannot validate manual RC passthrough") from exc
        print("Move RC sticks manually, then return to neutral. No station command will be sent.")
        deadline = time.monotonic() + args.duration_s
        with serial.Serial(args.port, baudrate=115200, timeout=0.5) as handle:
            while time.monotonic() < deadline:
                raw = handle.readline()
                if raw:
                    line = raw.decode("utf-8", errors="replace").strip()
                    print(line)
                    raw_lines.append(line)
    rows = parse_rows("\n".join(raw_lines))
    (out_dir / "raw_usbdbg.log").write_text("\n".join(raw_lines) + ("\n" if raw_lines else ""), encoding="utf-8")
    write_rows_csv(out_dir / "manual_rc_validation.csv", rows)
    summary = evaluate_rows(rows)
    write_summary(out_dir, summary)
    print(f"manual_rc_passthrough_pass={str(summary['manual_rc_passthrough_pass']).lower()}")
    print(f"reason={summary['reason']}")
    print("station_command_used=false")
    print("ready_for_full_path_following=false")
    return 0 if summary["manual_rc_passthrough_ok"] is True else 2


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Validate manual RC passthrough from USB telemetry.")
    parser.add_argument("--port", default="/dev/ttyACM0")
    parser.add_argument("--duration-s", type=float, default=30.0)
    parser.add_argument("--log")
    parser.add_argument("--diagnose-only", default="false")
    parser.add_argument("--out-dir", required=True)
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    try:
        return run(args)
    except RuntimeError as exc:
        out_dir = Path(args.out_dir)
        out_dir.mkdir(parents=True, exist_ok=True)
        summary = {
            "mode": "manual-rc",
            "success": False,
            "validation_success": False,
            "manual_rc_passthrough_ok": False,
            "reason": "SERIAL_ERROR",
            "next_recommended_action": "Check pyserial installation and USB telemetry access.",
            "station_command_used": False,
            "ready_for_full_path_following": False,
        }
        write_summary(out_dir, summary)
        print(f"reason={exc}")
        print("station_command_used=false")
        print("ready_for_full_path_following=false")
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
