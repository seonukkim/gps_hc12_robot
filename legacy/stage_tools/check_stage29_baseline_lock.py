from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Sequence

try:
    from tools import _bootstrap  # type: ignore  # noqa: F401
except ImportError:
    import _bootstrap  # type: ignore  # noqa: F401


REQUIRED = (
    "manual_rc_baseline",
    "gps_baseline",
    "imu_baseline",
    "station_pulse_baseline",
)


def load_payload(path: Path) -> dict[str, object]:
    if path.is_dir():
        path = path / "baseline_lock.json"
    return json.loads(path.read_text(encoding="utf-8"))


def evaluate_payload(payload: dict[str, object]) -> dict[str, object]:
    reasons: list[str] = []
    for key in REQUIRED:
        if payload.get(key) != "PASS":
            reasons.append(f"{key.upper()}_{payload.get(key, 'MISSING')}")
    ready_stage30 = payload.get("ready_for_stage30_physical_path_planning") is True
    if ready_stage30 and reasons:
        reasons.append("STAGE30_READY_WITH_FAILED_BASELINE")
    if not ready_stage30 and not reasons:
        reasons.append("STAGE30_NOT_READY_DESPITE_PASS_BASELINES")
    if payload.get("ready_for_full_path_following") is True:
        reasons.append("FULL_PATH_READY_SHOULD_BE_FALSE")
    return {
        "verdict": "PASS" if not reasons else "FAIL",
        "reasons": reasons or ["OK"],
        "ready_for_stage30_physical_path_planning": ready_stage30,
        "ready_for_full_path_following": False,
    }


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Check Stage29 baseline lock output.")
    parser.add_argument("path", type=Path)
    args = parser.parse_args(argv)
    result = evaluate_payload(load_payload(args.path))
    print(
        f"{result['verdict']} reason={','.join(result['reasons'])} "
        f"ready_for_stage30_physical_path_planning={str(result['ready_for_stage30_physical_path_planning']).lower()} "
        "ready_for_full_path_following=false"
    )
    return 0 if result["verdict"] == "PASS" else 1


if __name__ == "__main__":
    raise SystemExit(main())
