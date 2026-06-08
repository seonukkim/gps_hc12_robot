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
        path = path / "stage22_straight_segment.csv"
    if not path.exists():
        raise FileNotFoundError(f"stage22_straight_segment.csv not found at {path}")
    with path.open("r", encoding="utf-8", newline="") as handle:
        return [{key: value for key, value in row.items() if key is not None} for row in csv.DictReader(handle)]


def evaluate_rows(
    rows: Sequence[dict[str, str]],
    *,
    expected_repeat_count: int = 3,
    required_visible_count: int = 2,
) -> dict[str, object]:
    reasons: list[str] = []
    effective_required_visible_count = min(required_visible_count, expected_repeat_count)
    if not rows:
        reasons.append("NO_STAGE22_ROWS")

    ack_count = 0
    stop_count = 0
    reject_count = 0
    visible_count = 0
    final_left = 0.0
    final_right = 0.0

    for row in rows:
        if _parse_bool(row.get("ack_seen")) is True:
            ack_count += 1
        else:
            reasons.append("ACK_MISSING")
        if _parse_bool(row.get("stop_seen")) is True:
            stop_count += 1
        else:
            reasons.append("STOP_MISSING")
        if _parse_bool(row.get("reject_seen")) is True:
            reject_count += 1
            reasons.append("REJECT_SEEN")
        if _parse_bool(row.get("physical_output_active_after_stop")) is True:
            reasons.append("OUTPUT_ACTIVE_AFTER_STOP")
        if _parse_bool(row.get("final_left_cmd_zero")) is not True or _parse_bool(row.get("final_right_cmd_zero")) is not True:
            reasons.append("FINAL_COMMANDS_NONZERO")
        selected_b = _parse_float(row.get("selected_b_cmd"))
        if selected_b is None or abs(selected_b) > 1e-12:
            reasons.append("B_NOT_ZERO")
        if _parse_bool(row.get("ready_for_full_path_following")) is True:
            reasons.append("FULL_PATH_READY_SHOULD_BE_FALSE")
        if row.get("body_motion_user_report") in {"forward", "twitch"} or _parse_bool(row.get("visible_forward_or_twitch")) is True:
            visible_count += 1
        left = _parse_float(row.get("final_left_cmd"))
        right = _parse_float(row.get("final_right_cmd"))
        if left is not None:
            final_left = left
        if right is not None:
            final_right = right

    if len(rows) != expected_repeat_count:
        reasons.append("REPEAT_COUNT_MISMATCH")
    if ack_count != len(rows):
        reasons.append("NOT_ALL_PULSES_ACKED")
    if stop_count != len(rows):
        reasons.append("NOT_ALL_PULSES_STOPPED")
    if reject_count:
        reasons.append("REJECT_COUNT_NONZERO")
    if visible_count < effective_required_visible_count:
        reasons.append("VISIBLE_FORWARD_OR_TWITCH_BELOW_THRESHOLD")
    if abs(final_left) > 1e-9 or abs(final_right) > 1e-9:
        reasons.append("FINAL_COMMANDS_NONZERO")

    return {
        "verdict": "PASS" if not reasons else "FAIL",
        "pulse_count": len(rows),
        "ack_count": ack_count,
        "stop_count": stop_count,
        "reject_count": reject_count,
        "visible_forward_or_twitch_count": visible_count,
        "reasons": sorted(set(reasons)),
        "ready_for_full_path_following": False,
    }


def evaluate_path(path: Path, *, expected_repeat_count: int = 3, required_visible_count: int = 2) -> dict[str, object]:
    return evaluate_rows(_load_rows(path), expected_repeat_count=expected_repeat_count, required_visible_count=required_visible_count)


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Check Stage 22 straight segment repeat output.")
    parser.add_argument("path", type=Path, help="Stage22 output dir or CSV")
    parser.add_argument("--expected-repeat-count", type=int, default=3)
    parser.add_argument("--required-visible-count", type=int, default=2)
    args = parser.parse_args(argv)
    result = evaluate_path(
        args.path,
        expected_repeat_count=args.expected_repeat_count,
        required_visible_count=args.required_visible_count,
    )
    reasons = ",".join(result["reasons"]) if result["reasons"] else "OK"
    print(
        f"{result['verdict']} pulse_count={result['pulse_count']} ack_count={result['ack_count']} "
        f"stop_count={result['stop_count']} reject_count={result['reject_count']} "
        f"visible_forward_or_twitch_count={result['visible_forward_or_twitch_count']} "
        f"reason={reasons} ready_for_full_path_following=false"
    )
    return 0 if result["verdict"] == "PASS" else 1


if __name__ == "__main__":
    raise SystemExit(main())
