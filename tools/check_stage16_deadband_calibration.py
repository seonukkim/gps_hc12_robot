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
        path = path / "deadband_calibration.csv"
    if not path.exists():
        raise FileNotFoundError(f"deadband_calibration.csv not found at {path}")
    with path.open("r", encoding="utf-8", newline="") as handle:
        return [{key: value for key, value in row.items() if key is not None} for row in csv.DictReader(handle)]


def evaluate_rows(rows: Sequence[dict[str, str]], *, max_cmd: float = 0.04, max_ms: int = 150) -> dict[str, object]:
    reasons: list[str] = []
    sent_count = 0
    visible_count = 0
    no_visible_at_limit = False
    final_left = 0.0
    final_right = 0.0

    if not rows:
        reasons.append("NO_CALIBRATION_ROWS")

    for row in rows:
        cmd = _parse_float(row.get("cmd"))
        pulse_ms = _parse_float(row.get("pulse_ms"))
        if cmd is None:
            reasons.append("COMMAND_NA")
        elif abs(cmd) > max_cmd + 1e-9:
            reasons.append("COMMAND_EXCEEDS_MAX_CMD")
        if pulse_ms is None:
            reasons.append("PULSE_MS_NA")
        elif pulse_ms > max_ms:
            reasons.append("PULSE_EXCEEDS_MAX_MS")

        command_sent = _parse_bool(row.get("ack_seen")) is True
        if command_sent:
            sent_count += 1
            if _parse_bool(row.get("arm_ack_seen")) is not True:
                reasons.append("ACK_WITHOUT_ARM")
            if _parse_bool(row.get("stop_seen")) is not True:
                reasons.append("STOP_MISSING_AFTER_PULSE")
            if _parse_bool(row.get("physical_output_active_after_stop")) is True:
                reasons.append("OUTPUT_ACTIVE_AFTER_STOP")
            if row.get("firmware_active_target_source") == "compile_time":
                reasons.append("COMPILE_TIME_TARGET_ACTIVE_DURING_STAGE16")

        if _parse_bool(row.get("physical_path_following_enable")) is True:
            reasons.append("NORMAL_PATH_FOLLOWING_ENABLED")
        if _parse_bool(row.get("allow_motor_output")) is True:
            reasons.append("ALLOW_MOTOR_OUTPUT_ENABLED")
        if _parse_bool(row.get("ready_for_full_path_following")) is True:
            reasons.append("FULL_PATH_READY_SHOULD_BE_FALSE")
        if _parse_bool(row.get("visible_motion_confirmed")) is True:
            visible_count += 1
        if _parse_bool(row.get("no_visible_motion_at_limit")) is True:
            no_visible_at_limit = True

        left = _parse_float(row.get("final_left_cmd"))
        right = _parse_float(row.get("final_right_cmd"))
        if left is not None:
            final_left = left
        if right is not None:
            final_right = right

    if sent_count > 0 and abs(final_left) > 1e-9 or sent_count > 0 and abs(final_right) > 1e-9:
        reasons.append("FINAL_COMMANDS_NONZERO")
    if visible_count == 0 and not no_visible_at_limit:
        reasons.append("NO_VISIBLE_RESULT_RECORDED")

    verdict = "PASS" if not reasons else "FAIL"
    return {
        "verdict": verdict,
        "sent_count": sent_count,
        "visible_motion_count": visible_count,
        "no_visible_motion_at_limit": no_visible_at_limit,
        "reasons": sorted(set(reasons)),
        "ready_for_full_path_following": False,
    }


def evaluate_path(path: Path, *, max_cmd: float = 0.04, max_ms: int = 150) -> dict[str, object]:
    return evaluate_rows(_load_rows(path), max_cmd=max_cmd, max_ms=max_ms)


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Check Stage 16 motor deadband calibration output.")
    parser.add_argument("path", type=Path, help="Calibration output dir or deadband_calibration.csv")
    parser.add_argument("--max-cmd", type=float, default=0.04)
    parser.add_argument("--max-ms", type=int, default=150)
    args = parser.parse_args(argv)
    result = evaluate_path(args.path, max_cmd=args.max_cmd, max_ms=args.max_ms)
    reasons = ",".join(result["reasons"]) if result["reasons"] else "OK"
    print(
        f"{result['verdict']} sent_count={result['sent_count']} "
        f"visible_motion_count={result['visible_motion_count']} "
        f"no_visible_motion_at_limit={str(result['no_visible_motion_at_limit']).lower()} "
        f"reason={reasons} ready_for_full_path_following=false"
    )
    return 0 if result["verdict"] == "PASS" else 1


if __name__ == "__main__":
    raise SystemExit(main())
