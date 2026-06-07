from __future__ import annotations

import argparse
import math
import re
from pathlib import Path
from typing import Iterable, Sequence

try:
    from tools import _bootstrap  # type: ignore  # noqa: F401
except ImportError:
    import _bootstrap  # type: ignore  # noqa: F401


TRUE_VALUES = {"1", "true", "yes", "ok", "active"}
FALSE_VALUES = {"0", "false", "no", "off", "inactive"}


def _parse_bool(value: object) -> bool | None:
    if value is None:
        return None
    text = str(value).strip().lower()
    if text in TRUE_VALUES:
        return True
    if text in FALSE_VALUES:
        return False
    return None


def _parse_float(value: object) -> float | None:
    if value is None:
        return None
    text = str(value).strip()
    if text.upper() in {"", "NA", "NAN", "NONE"}:
        return None
    try:
        parsed = float(text)
    except ValueError:
        return None
    return parsed if math.isfinite(parsed) else None


def parse_key_value_lines(text: str) -> list[dict[str, str]]:
    rows: list[dict[str, str]] = []
    for line in text.splitlines():
        values = {
            key.lower(): value
            for key, value in re.findall(r"([A-Za-z0-9_]+)=([^,\s]+)", line)
        }
        if values:
            rows.append(values)
    return rows


def _is_active(values: dict[str, str], *keys: str) -> bool:
    for key in keys:
        parsed = _parse_bool(values.get(key))
        if parsed is True:
            return True
    return False


def evaluate_rows(
    rows: Sequence[dict[str, str]],
    *,
    max_cmd: float = 0.08,
    max_duration_ms: int = 300,
) -> dict[str, object]:
    reasons: list[str] = []
    pulse_waiting_for_stop = False
    pulse_count = 0
    stop_count = 0
    final_left_cmd = 0.0
    final_right_cmd = 0.0

    if not rows:
        reasons.append("NO_STAGE15_LOG_ROWS")

    for row in rows:
        if _is_active(
            row,
            "path_following_enable",
            "physical_path_following_enable",
            "path_package_used",
            "virtual_controller_used",
            "autonomous_mode",
            "allow_motor_output",
            "path_following_active",
            "auto_motion_armed",
        ):
            reasons.append("PATH_OR_AUTONOMOUS_FIELD_ACTIVE")

        is_stage15_row = _parse_bool(row.get("stage15_guarded_crawl")) is True
        pulse_type = row.get("pulse_type", "") if is_stage15_row else ""
        physical_output_active = _parse_bool(row.get("physical_output_active"))
        stop_confirmed = _parse_bool(row.get("stop_confirmed"))

        if pulse_type and pulse_type != "test_stop":
            pulse_count += 1
            if pulse_waiting_for_stop:
                reasons.append("MISSING_STOP_BETWEEN_PULSES")
            pulse_waiting_for_stop = True
            left = _parse_float(row.get("pulse_cmd_left"))
            right = _parse_float(row.get("pulse_cmd_right"))
            duration = _parse_float(row.get("pulse_ms"))
            if left is None or right is None:
                reasons.append("PULSE_COMMAND_NA")
            else:
                if abs(left) > max_cmd + 1e-9 or abs(right) > max_cmd + 1e-9:
                    reasons.append("PULSE_COMMAND_EXCEEDS_LIMIT")
            if duration is None:
                reasons.append("PULSE_DURATION_NA")
            elif duration > max_duration_ms:
                reasons.append("PULSE_DURATION_EXCEEDS_LIMIT")
            if physical_output_active is False:
                reasons.append("PULSE_WITHOUT_ACTIVE_OUTPUT")

        if pulse_type == "test_stop" or stop_confirmed is True:
            stop_count += 1
            if physical_output_active is True:
                reasons.append("STOP_ROW_OUTPUT_STILL_ACTIVE")
            pulse_waiting_for_stop = False
        elif is_stage15_row and not pulse_type and physical_output_active is True:
            reasons.append("UNEXPECTED_CONTINUOUS_OUTPUT")

        left_final = _parse_float(row.get("final_left_cmd"))
        right_final = _parse_float(row.get("final_right_cmd"))
        if left_final is not None:
            final_left_cmd = left_final
        if right_final is not None:
            final_right_cmd = right_final

    if pulse_waiting_for_stop:
        reasons.append("NO_STOP_CONFIRMATION_AFTER_LAST_PULSE")
    if pulse_count == 0:
        reasons.append("NO_PULSES_FOUND")
    if stop_count < pulse_count:
        reasons.append("STOP_COUNT_LESS_THAN_PULSE_COUNT")
    if abs(final_left_cmd) > 1e-9 or abs(final_right_cmd) > 1e-9:
        reasons.append("FINAL_COMMANDS_NONZERO")

    verdict = "PASS" if not reasons else "FAIL"
    return {
        "verdict": verdict,
        "pulse_count": pulse_count,
        "stop_count": stop_count,
        "final_left_cmd": final_left_cmd,
        "final_right_cmd": final_right_cmd,
        "reasons": sorted(set(reasons)),
        "motor_command_generated": False,
        "physical_output_active": False if verdict == "PASS" else None,
    }


def evaluate_log_text(
    text: str,
    *,
    max_cmd: float = 0.08,
    max_duration_ms: int = 300,
) -> dict[str, object]:
    return evaluate_rows(parse_key_value_lines(text), max_cmd=max_cmd, max_duration_ms=max_duration_ms)


def _format_result(result: dict[str, object]) -> str:
    reasons = result.get("reasons", [])
    reason_text = ",".join(str(reason) for reason in reasons) if reasons else "OK"
    return (
        f"{result['verdict']} pulse_count={result['pulse_count']} "
        f"stop_count={result['stop_count']} reason={reason_text} "
        "motor_command_generated=false"
    )


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Check a Stage 15 guarded crawl log.")
    parser.add_argument("log_path", type=Path)
    parser.add_argument("--max-cmd", type=float, default=0.08)
    parser.add_argument("--max-duration-ms", type=int, default=300)
    args = parser.parse_args(argv)

    text = args.log_path.read_text(encoding="utf-8", errors="replace")
    result = evaluate_log_text(text, max_cmd=args.max_cmd, max_duration_ms=args.max_duration_ms)
    print(_format_result(result))
    return 0 if result["verdict"] == "PASS" else 1


if __name__ == "__main__":
    raise SystemExit(main())
