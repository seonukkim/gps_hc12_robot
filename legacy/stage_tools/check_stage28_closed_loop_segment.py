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
        value_float = float(text)
    except ValueError:
        return None
    return value_float if math.isfinite(value_float) else None


def _load_rows(path: Path) -> list[dict[str, str]]:
    if path.is_dir():
        path = path / "stage28_closed_loop_segment.csv"
    if not path.exists():
        raise FileNotFoundError(f"stage28_closed_loop_segment.csv not found at {path}")
    with path.open("r", encoding="utf-8", newline="") as handle:
        return [{key: value for key, value in row.items() if key is not None} for row in csv.DictReader(handle)]


def evaluate_rows(rows: Sequence[dict[str, str]]) -> dict[str, object]:
    reasons: list[str] = []
    valid_count = 0
    reject_count = 0
    rc_invalid_count = 0
    final_left = 0.0
    final_right = 0.0

    if not rows:
        reasons.append("NO_STAGE28_ROWS")

    for row in rows:
        if _parse_bool(row.get("ready_for_full_path_following")) is True:
            reasons.append("FULL_PATH_READY_SHOULD_BE_FALSE")
        if _parse_bool(row.get("arm_seen")) is not True:
            reasons.append("ARM_MISSING")
        if _parse_bool(row.get("ack_seen")) is not True:
            reasons.append("ACK_MISSING")
        if _parse_bool(row.get("stop_seen")) is not True:
            reasons.append("STOP_MISSING")
        if _parse_bool(row.get("reject_seen")) is True:
            reject_count += 1
            reasons.append("REJECT_SEEN")
        if _parse_bool(row.get("rc_invalid_seen")) is True:
            rc_invalid_count += 1
            reasons.append("RC_INVALID")
        if _parse_bool(row.get("physical_output_active_after_stop")) is True:
            reasons.append("OUTPUT_ACTIVE_AFTER_STOP")
        if _parse_bool(row.get("final_left_cmd_zero")) is not True or _parse_bool(row.get("final_right_cmd_zero")) is not True:
            reasons.append("FINAL_COMMANDS_NONZERO")
        if _parse_float(row.get("b_cmd")) is None:
            reasons.append("B_CMD_NA")
        if _parse_bool(row.get("valid_pulse")) is True:
            valid_count += 1
        else:
            reason = row.get("invalid_reason", "INVALID_PULSE")
            if reason and reason != "NONE":
                reasons.append(reason)
        left = _parse_float(row.get("final_left_cmd"))
        right = _parse_float(row.get("final_right_cmd"))
        if left is not None:
            final_left = left
        if right is not None:
            final_right = right

    if rows and (abs(final_left) > 1e-9 or abs(final_right) > 1e-9):
        reasons.append("FINAL_COMMANDS_NONZERO")

    return {
        "verdict": "PASS" if not reasons else "FAIL",
        "pulse_count": len(rows),
        "valid_pulse_count": valid_count,
        "reject_count": reject_count,
        "rc_invalid_count": rc_invalid_count,
        "reasons": sorted(set(reasons)),
        "ready_for_full_path_following": False,
    }


def evaluate_path(path: Path) -> dict[str, object]:
    return evaluate_rows(_load_rows(path))


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Check Stage28 closed-loop segment output.")
    parser.add_argument("path", type=Path, help="Stage28 output dir or CSV")
    args = parser.parse_args(argv)
    result = evaluate_path(args.path)
    reasons = ",".join(result["reasons"]) if result["reasons"] else "OK"
    print(
        f"{result['verdict']} pulse_count={result['pulse_count']} "
        f"valid_pulse_count={result['valid_pulse_count']} reject_count={result['reject_count']} "
        f"rc_invalid_count={result['rc_invalid_count']} reason={reasons} "
        "ready_for_full_path_following=false"
    )
    return 0 if result["verdict"] == "PASS" else 1


if __name__ == "__main__":
    raise SystemExit(main())
