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
        path = path / "stage26_calibrated_primitive_sequence.csv"
    if not path.exists():
        raise FileNotFoundError(f"stage26_calibrated_primitive_sequence.csv not found at {path}")
    with path.open("r", encoding="utf-8", newline="") as handle:
        return [{key: value for key, value in row.items() if key is not None} for row in csv.DictReader(handle)]


def evaluate_rows(rows: Sequence[dict[str, str]]) -> dict[str, object]:
    reasons: list[str] = []
    arm_count = 0
    ack_count = 0
    stop_count = 0
    reject_count = 0
    valid_count = 0
    final_left = 0.0
    final_right = 0.0

    if not rows:
        reasons.append("NO_STAGE26_ROWS")

    for row in rows:
        name = row.get("primitive_name", "")
        is_stop = name == "stop"
        if _parse_bool(row.get("ready_for_full_path_following")) is True:
            reasons.append("FULL_PATH_READY_SHOULD_BE_FALSE")
        if _parse_bool(row.get("arm_seen")) is True:
            arm_count += 1
        elif not is_stop:
            reasons.append("ARM_MISSING")
        if _parse_bool(row.get("ack_seen")) is True:
            ack_count += 1
        elif not is_stop:
            reasons.append("ACK_MISSING")
        if _parse_bool(row.get("stop_seen")) is True:
            stop_count += 1
        else:
            reasons.append("STOP_MISSING")
        if _parse_bool(row.get("reject_seen")) is True:
            reject_count += 1
            reasons.append("REJECT_SEEN")
        if str(row.get("reject_reason", "NONE")).upper() == "RC_INVALID":
            reasons.append("RC_INVALID")
        if _parse_bool(row.get("physical_output_active_after_stop")) is True:
            reasons.append("OUTPUT_ACTIVE_AFTER_STOP")
        if _parse_bool(row.get("final_left_cmd_zero")) is not True or _parse_bool(row.get("final_right_cmd_zero")) is not True:
            reasons.append("FINAL_COMMANDS_NONZERO")
        if _parse_bool(row.get("valid_primitive")) is True:
            valid_count += 1
        else:
            reasons.append(str(row.get("invalid_reason", "INVALID_PRIMITIVE")))
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
        "primitive_count": len(rows),
        "valid_primitive_count": valid_count,
        "arm_count": arm_count,
        "ack_count": ack_count,
        "stop_count": stop_count,
        "reject_count": reject_count,
        "reasons": sorted(set(reason for reason in reasons if reason and reason != "NONE")),
        "ready_for_full_path_following": False,
    }


def evaluate_path(path: Path) -> dict[str, object]:
    return evaluate_rows(_load_rows(path))


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Check Stage26 calibrated primitive sequence output.")
    parser.add_argument("path", type=Path, help="Stage26 output dir or CSV")
    args = parser.parse_args(argv)
    result = evaluate_path(args.path)
    reasons = ",".join(result["reasons"]) if result["reasons"] else "OK"
    print(
        f"{result['verdict']} primitive_count={result['primitive_count']} "
        f"valid_primitive_count={result['valid_primitive_count']} "
        f"arm_count={result['arm_count']} ack_count={result['ack_count']} "
        f"stop_count={result['stop_count']} reject_count={result['reject_count']} "
        f"reason={reasons} ready_for_full_path_following=false"
    )
    return 0 if result["verdict"] == "PASS" else 1


if __name__ == "__main__":
    raise SystemExit(main())
