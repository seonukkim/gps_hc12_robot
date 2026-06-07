from __future__ import annotations

import argparse
import csv
import math
import re
from pathlib import Path
from typing import Sequence

try:
    from tools import _bootstrap  # type: ignore  # noqa: F401
except ImportError:
    import _bootstrap  # type: ignore  # noqa: F401


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


def parse_log_text(text: str) -> list[dict[str, str]]:
    rows: list[dict[str, str]] = []
    for line in text.splitlines():
        values = {key.lower(): value for key, value in re.findall(r"([A-Za-z0-9_]+)=([^,\s]+)", line)}
        if values:
            rows.append(values)
    if rows:
        return rows
    try:
        return [
            {key.lower(): value for key, value in row.items() if value is not None}
            for row in csv.DictReader(text.splitlines())
        ]
    except csv.Error:
        return []


def evaluate_rows(
    rows: Sequence[dict[str, str]],
    *,
    max_cmd: float = 0.04,
    max_ms: int = 150,
    max_total_distance_m: float = 0.30,
) -> dict[str, object]:
    reasons: list[str] = []
    saw_stage16 = False
    armed_since_stop = False
    arm_count = 0
    active_waiting_for_stop = False
    active_count = 0
    stop_count = 0
    final_left = 0.0
    final_right = 0.0
    final_stop_confirmed = False
    max_estimated_distance = 0.0

    if not rows:
        reasons.append("NO_LOG_ROWS")

    for row in rows:
        if _parse_bool(row.get("stage16_guarded_crawl")) is True:
            saw_stage16 = True

        if _parse_bool(row.get("physical_path_following_enable")) is True or _parse_bool(row.get("path_following_enable")) is True:
            reasons.append("NORMAL_PATH_FOLLOWING_ENABLED")
        if _parse_bool(row.get("allow_motor_output")) is True or _parse_bool(row.get("path_package_used")) is True:
            reasons.append("UNEXPECTED_PATH_OR_MOTOR_GATE_ACTIVE")

        state = str(row.get("stage16_cmd_state", "")).upper()
        event = str(row.get("event", "")).upper()
        if event == "ARM" or state == "ARMED":
            armed_since_stop = True
            arm_count += 1
        physical_active = _parse_bool(row.get("physical_output_active", row.get("stage16_physical_output_active")))
        active = state == "ACTIVE" or event == "ACK" or physical_active is True
        stopped = state == "STOPPED" or event == "STOP" or (
            _parse_bool(row.get("final_stop_confirmed")) is True
        )
        rejected = state == "REJECTED" or event == "REJECT"

        if active:
            active_count += 1
            if not armed_since_stop:
                reasons.append("ACTIVE_WITHOUT_ARM")
            if active_waiting_for_stop:
                reasons.append("MISSING_STOP_BETWEEN_PULSES")
            active_waiting_for_stop = True
            left = _parse_float(row.get("stage16_left_cmd", row.get("left")))
            right = _parse_float(row.get("stage16_right_cmd", row.get("right")))
            pulse_ms = _parse_float(row.get("stage16_ms", row.get("ms")))
            if left is None or right is None:
                reasons.append("COMMAND_NA")
            elif abs(left) > max_cmd + 1e-9 or abs(right) > max_cmd + 1e-9:
                reasons.append("COMMAND_EXCEEDS_MAX_CMD")
            if pulse_ms is None:
                reasons.append("PULSE_MS_NA")
            elif pulse_ms > max_ms:
                reasons.append("PULSE_EXCEEDS_MAX_MS")
            source = row.get("active_target_source", row.get("firmware_active_target_source", ""))
            if source == "compile_time":
                reasons.append("COMPILE_TIME_TARGET_ACTIVE_DURING_STAGE16")

        if stopped:
            stop_count += 1
            final_stop_confirmed = True
            if physical_active is True:
                reasons.append("STOPPED_ROW_OUTPUT_ACTIVE")
            active_waiting_for_stop = False
            armed_since_stop = False

        if rejected and not row.get("stage16_reject_reason"):
            reasons.append("REJECT_WITHOUT_REASON")

        left_final = _parse_float(row.get("final_left_cmd"))
        right_final = _parse_float(row.get("final_right_cmd"))
        if left_final is not None:
            final_left = left_final
        if right_final is not None:
            final_right = right_final
        estimated = _parse_float(row.get("estimated_distance_m"))
        if estimated is not None:
            max_estimated_distance = max(max_estimated_distance, estimated)

    if not saw_stage16:
        reasons.append("STAGE16_NOT_REPORTED")
    if active_waiting_for_stop:
        reasons.append("SERIAL_DISCONNECT_OR_LOG_END_WHILE_ACTIVE")
    if active_count > 0 and stop_count < active_count:
        reasons.append("NO_STOP_AFTER_EACH_PULSE")
    if abs(final_left) > 1e-9 or abs(final_right) > 1e-9:
        reasons.append("FINAL_COMMANDS_NONZERO")
    if active_count > 0 and not final_stop_confirmed:
        reasons.append("FINAL_STOP_NOT_CONFIRMED")
    if max_estimated_distance > max_total_distance_m + 1e-9:
        reasons.append("TOTAL_DISTANCE_EXCEEDS_LIMIT")

    verdict = "PASS" if not reasons else "FAIL"
    return {
        "verdict": verdict,
        "arm_count": arm_count,
        "active_count": active_count,
        "stop_count": stop_count,
        "final_stop_confirmed": final_stop_confirmed,
        "estimated_distance_m": max_estimated_distance,
        "reasons": sorted(set(reasons)),
        "ready_for_full_path_following": False,
    }


def evaluate_log_text(
    text: str,
    *,
    max_cmd: float = 0.04,
    max_ms: int = 150,
    max_total_distance_m: float = 0.30,
) -> dict[str, object]:
    return evaluate_rows(
        parse_log_text(text),
        max_cmd=max_cmd,
        max_ms=max_ms,
        max_total_distance_m=max_total_distance_m,
    )


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Check Stage 16 guarded crawl logs.")
    parser.add_argument("log_path", type=Path)
    parser.add_argument("--max-cmd", type=float, default=0.04)
    parser.add_argument("--max-ms", type=int, default=150)
    parser.add_argument("--max-total-distance-m", type=float, default=0.30)
    args = parser.parse_args(argv)
    result = evaluate_log_text(
        args.log_path.read_text(encoding="utf-8", errors="replace"),
        max_cmd=args.max_cmd,
        max_ms=args.max_ms,
        max_total_distance_m=args.max_total_distance_m,
    )
    reasons = ",".join(result["reasons"]) if result["reasons"] else "OK"
    print(
        f"{result['verdict']} arm_count={result['arm_count']} active_count={result['active_count']} "
        f"stop_count={result['stop_count']} reason={reasons} "
        "ready_for_full_path_following=false"
    )
    return 0 if result["verdict"] == "PASS" else 1


if __name__ == "__main__":
    raise SystemExit(main())
