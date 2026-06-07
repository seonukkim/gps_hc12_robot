from __future__ import annotations

import argparse
import csv
import math
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


def _load_rows(path: Path) -> list[dict[str, str]]:
    if path.is_dir():
        path = path / "stage18_motor_mapping_probe.csv"
    if not path.exists():
        raise FileNotFoundError(f"stage18_motor_mapping_probe.csv not found at {path}")
    with path.open("r", encoding="utf-8", newline="") as handle:
        return [{key: value for key, value in row.items() if key is not None} for row in csv.DictReader(handle)]


def evaluate_rows(rows: Sequence[dict[str, str]], *, max_cmd: float = 0.08, max_ms: int = 800) -> dict[str, object]:
    reasons: list[str] = []
    pulse_count = 0
    visible_count = 0
    suspect = False
    final_left = 0.0
    final_right = 0.0

    if not rows:
        reasons.append("NO_STAGE18_ROWS")

    for row in rows:
        requested_left = _parse_float(row.get("requested_left_cmd"))
        requested_right = _parse_float(row.get("requested_right_cmd"))
        pulse_ms = _parse_float(row.get("pulse_ms"))
        if requested_left is None or requested_right is None:
            reasons.append("REQUESTED_COMMAND_NA")
        elif abs(requested_left) > max_cmd + 1e-9 or abs(requested_right) > max_cmd + 1e-9:
            reasons.append("COMMAND_EXCEEDS_MAX_CMD")
        if pulse_ms is None:
            reasons.append("PULSE_MS_NA")
        elif pulse_ms > max_ms:
            reasons.append("PULSE_EXCEEDS_MAX_MS")

        ack = _parse_bool(row.get("ack_seen")) is True
        stop = _parse_bool(row.get("stop_seen")) is True
        if ack:
            pulse_count += 1
            if _parse_bool(row.get("arm_ack_seen")) is not True:
                reasons.append("ACK_WITHOUT_ARM")
            if not stop:
                reasons.append("STOP_MISSING_AFTER_PULSE")
            if _parse_bool(row.get("motor_write_called")) is not True:
                reasons.append("MOTOR_WRITE_CALLED_FIELD_FALSE_OR_MISSING")
            if _parse_bool(row.get("physical_output_active_after_stop")) is True:
                reasons.append("OUTPUT_ACTIVE_AFTER_STOP")
            if row.get("firmware_active_target_source") == "compile_time":
                reasons.append("COMPILE_TIME_TARGET_ACTIVE_DURING_STAGE18")
            if _parse_bool(row.get("physical_path_following_enable")) is True:
                reasons.append("NORMAL_PATH_FOLLOWING_ENABLED")
            if _parse_bool(row.get("allow_motor_output")) is True:
                reasons.append("ALLOW_MOTOR_OUTPUT_ENABLED")
        if _parse_bool(row.get("visible_motion_confirmed")) is True:
            visible_count += 1
        if _parse_bool(row.get("motor_power_or_mapping_suspect")) is True:
            suspect = True
        if _parse_bool(row.get("ready_for_full_path_following")) is True:
            reasons.append("FULL_PATH_READY_SHOULD_BE_FALSE")

        left = _parse_float(row.get("final_left_cmd"))
        right = _parse_float(row.get("final_right_cmd"))
        if left is not None:
            final_left = left
        if right is not None:
            final_right = right

    if pulse_count > 0 and abs(final_left) > 1e-9 or pulse_count > 0 and abs(final_right) > 1e-9:
        reasons.append("FINAL_COMMANDS_NONZERO")
    if pulse_count > 0 and not any(_parse_bool(row.get("stop_seen")) is True for row in rows):
        reasons.append("NO_STOP_AFTER_PULSE")

    verdict = "PASS" if not reasons else "FAIL"
    return {
        "verdict": verdict,
        "pulse_count": pulse_count,
        "visible_motion_count": visible_count,
        "motor_power_or_mapping_suspect": suspect,
        "reasons": sorted(set(reasons)),
        "ready_for_full_path_following": False,
    }


def evaluate_path(path: Path, *, max_cmd: float = 0.08, max_ms: int = 800) -> dict[str, object]:
    return evaluate_rows(_load_rows(path), max_cmd=max_cmd, max_ms=max_ms)


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Check Stage 18 motor mapping probe output.")
    parser.add_argument("path", type=Path, help="Stage18 output dir or CSV")
    parser.add_argument("--max-cmd", type=float, default=0.08)
    parser.add_argument("--max-ms", type=int, default=800)
    args = parser.parse_args(argv)
    result = evaluate_path(args.path, max_cmd=args.max_cmd, max_ms=args.max_ms)
    reasons = ",".join(result["reasons"]) if result["reasons"] else "OK"
    print(
        f"{result['verdict']} pulse_count={result['pulse_count']} "
        f"visible_motion_count={result['visible_motion_count']} "
        f"motor_power_or_mapping_suspect={str(result['motor_power_or_mapping_suspect']).lower()} "
        f"reason={reasons} ready_for_full_path_following=false"
    )
    return 0 if result["verdict"] == "PASS" else 1


if __name__ == "__main__":
    raise SystemExit(main())
