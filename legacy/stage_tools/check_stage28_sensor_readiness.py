from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Sequence

try:
    from tools import _bootstrap  # type: ignore  # noqa: F401
except ImportError:
    import _bootstrap  # type: ignore  # noqa: F401


PASS_LEVELS = {
    "READY_MANUAL_RC_ONLY",
    "READY_OPEN_LOOP_WITH_SENSORS",
    "READY_IMU_ONLY_HEADING",
    "READY_GPS_IMU_CLOSED_LOOP",
}


def load_summary(path: Path) -> dict[str, object]:
    if path.is_dir():
        path = path / "sensor_readiness_summary.json"
    return json.loads(path.read_text(encoding="utf-8"))


def evaluate_summary(summary: dict[str, object]) -> dict[str, object]:
    readiness = str(summary.get("sensor_readiness", "UNKNOWN"))
    reasons: list[str] = []
    if readiness not in PASS_LEVELS:
        reasons.append(readiness)
    if summary.get("ready_for_full_path_following") is True:
        reasons.append("FULL_PATH_READY_SHOULD_BE_FALSE")
    if summary.get("motor_command_generated") is True:
        reasons.append("MOTOR_COMMAND_GENERATED")
    return {
        "verdict": "PASS" if not reasons else "FAIL",
        "sensor_readiness": readiness,
        "reasons": reasons or ["OK"],
        "ready_for_full_path_following": False,
    }


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Check Stage28A sensor readiness output.")
    parser.add_argument("path", type=Path)
    args = parser.parse_args(argv)
    result = evaluate_summary(load_summary(args.path))
    print(
        f"{result['verdict']} sensor_readiness={result['sensor_readiness']} "
        f"reason={','.join(result['reasons'])} ready_for_full_path_following=false"
    )
    return 0 if result["verdict"] == "PASS" else 1


if __name__ == "__main__":
    raise SystemExit(main())
