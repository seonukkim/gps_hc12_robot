from __future__ import annotations

import argparse
import csv
from pathlib import Path
from typing import Sequence

try:
    from tools import _bootstrap  # type: ignore  # noqa: F401
except ImportError:
    import _bootstrap  # type: ignore  # noqa: F401


def _parse_bool(value: object) -> bool | None:
    text = str(value).strip().lower()
    if text in {"1", "true", "yes", "ok"}:
        return True
    if text in {"0", "false", "no", "off"}:
        return False
    return None


def _load_rows(path: Path) -> list[dict[str, str]]:
    if path.is_dir():
        path = path / "stage28_calibrated_motion_with_sensors.csv"
    with path.open("r", encoding="utf-8", newline="") as handle:
        return list(csv.DictReader(handle))


def evaluate_rows(rows: Sequence[dict[str, str]]) -> dict[str, object]:
    reasons: list[str] = []
    if not rows:
        reasons.append("NO_ROWS")
    for row in rows:
        if _parse_bool(row.get("ready_for_full_path_following")) is True:
            reasons.append("FULL_PATH_READY_SHOULD_BE_FALSE")
        for field, reason in (
            ("arm_seen", "ARM_MISSING"),
            ("ack_seen", "ACK_MISSING"),
            ("stop_seen", "STOP_MISSING"),
            ("final_left_cmd_zero", "FINAL_COMMANDS_NONZERO"),
            ("final_right_cmd_zero", "FINAL_COMMANDS_NONZERO"),
            ("valid_primitive", "INVALID_PRIMITIVE"),
        ):
            if _parse_bool(row.get(field)) is not True:
                reasons.append(reason)
        if _parse_bool(row.get("reject_seen")) is True:
            reasons.append("REJECT_SEEN")
        if _parse_bool(row.get("physical_output_active_after_stop")) is True:
            reasons.append("OUTPUT_ACTIVE_AFTER_STOP")
    return {
        "verdict": "PASS" if not reasons else "FAIL",
        "primitive_count": len(rows),
        "reasons": sorted(set(reasons)) or ["OK"],
        "ready_for_full_path_following": False,
    }


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Check Stage28C calibrated motion with sensors.")
    parser.add_argument("path", type=Path)
    args = parser.parse_args(argv)
    result = evaluate_rows(_load_rows(args.path))
    print(
        f"{result['verdict']} primitive_count={result['primitive_count']} "
        f"reason={','.join(result['reasons'])} ready_for_full_path_following=false"
    )
    return 0 if result["verdict"] == "PASS" else 1


if __name__ == "__main__":
    raise SystemExit(main())
