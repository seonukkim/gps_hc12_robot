from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Sequence

try:
    from tools import _bootstrap  # type: ignore  # noqa: F401
except ImportError:
    import _bootstrap  # type: ignore  # noqa: F401


def load_summary(path: Path) -> dict[str, object]:
    if path.is_dir():
        path = path / "manual_rc_passthrough_summary.json"
    return json.loads(path.read_text(encoding="utf-8"))


def evaluate_summary(summary: dict[str, object]) -> dict[str, object]:
    reasons: list[str] = []
    if summary.get("manual_rc_passthrough_pass") is not True:
        reasons.extend(str(item) for item in summary.get("reasons", ["MANUAL_RC_PASSTHROUGH_FAIL"]))
    if summary.get("station_command_used") is True:
        reasons.append("STATION_COMMAND_USED")
    if summary.get("ready_for_full_path_following") is True:
        reasons.append("FULL_PATH_READY_SHOULD_BE_FALSE")
    return {
        "verdict": "PASS" if not reasons else "FAIL",
        "reasons": reasons or ["OK"],
        "ready_for_full_path_following": False,
    }


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Check manual RC passthrough validation output.")
    parser.add_argument("path", type=Path)
    args = parser.parse_args(argv)
    result = evaluate_summary(load_summary(args.path))
    print(f"{result['verdict']} reason={','.join(result['reasons'])} ready_for_full_path_following=false")
    return 0 if result["verdict"] == "PASS" else 1


if __name__ == "__main__":
    raise SystemExit(main())
