from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path
from typing import Sequence

try:
    from tools import _bootstrap  # type: ignore  # noqa: F401
except ImportError:
    import _bootstrap  # type: ignore  # noqa: F401


def _parse_float(value: object, default: float = 0.0) -> float:
    try:
        return float(str(value).strip())
    except (TypeError, ValueError):
        return default


def _parse_int(value: object, default: int = 0) -> int:
    try:
        return int(float(str(value).strip()))
    except (TypeError, ValueError):
        return default


def _load_probe_rows(path: Path) -> list[dict[str, str]]:
    csv_path = path / "stage20_physical_ab_probe.csv" if path.is_dir() else path
    if not csv_path.exists():
        return []
    with csv_path.open("r", encoding="utf-8", newline="") as handle:
        return [{key: value for key, value in row.items() if key is not None} for row in csv.DictReader(handle)]


def motion_class(row: dict[str, str]) -> str:
    stored = row.get("motion_class", "").strip().lower()
    if stored in {"none", "twitch", "visible", "unknown"}:
        return stored
    body = row.get("body_motion_user_report", "unknown")
    if body in {"forward", "backward", "left", "right"}:
        return "visible"
    if body == "twitch" or row.get("left_wheel_user_report") in {"yes", "twitch"} or row.get("right_wheel_user_report") in {"yes", "twitch"}:
        return "twitch"
    if body == "unknown":
        return "unknown"
    return "none"


def _valid_rows(rows: Sequence[dict[str, str]]) -> list[dict[str, str]]:
    valid: list[dict[str, str]] = []
    for row in rows:
        if str(row.get("valid_trial", "true")).strip().lower() == "false":
            continue
        if str(row.get("invalid_reason", "NONE")).strip().upper() not in {"", "NONE", "NA"}:
            continue
        valid.append(row)
    return valid


def _first_by_class(rows: Sequence[dict[str, str]], wanted: str) -> dict[str, str] | None:
    for row in rows:
        if motion_class(row) == wanted:
            return row
    return None


def _first_visible_direction(rows: Sequence[dict[str, str]], body_direction: str) -> dict[str, str] | None:
    for row in rows:
        if motion_class(row) == "visible" and row.get("body_motion_user_report") == body_direction:
            return row
    return None


def _fallback_next_step(row: dict[str, str] | None, *, direction: str) -> tuple[float, int]:
    if row is None:
        return 0.0, 0
    cmd = _parse_float(row.get("requested_a_cmd"), 0.0)
    pulse = _parse_int(row.get("pulse_ms"), 0)
    if direction == "forward":
        return min(cmd + 0.02, 0.25), min(max(pulse + 150, 650), 800)
    return max(cmd - 0.02, -0.12), min(max(pulse, 300), 500)


def build_fine_calibration(
    *,
    forward_rows: Sequence[dict[str, str]],
    backward_rows: Sequence[dict[str, str]],
    base_calibration: dict[str, object],
) -> dict[str, object]:
    valid_forward_rows = _valid_rows(forward_rows)
    valid_backward_rows = _valid_rows(backward_rows)
    forward_twitch = _first_by_class(valid_forward_rows, "twitch")
    forward_visible = _first_visible_direction(valid_forward_rows, "forward")
    backward_visible = _first_visible_direction(valid_backward_rows, "backward")

    if forward_visible is not None:
        forward_recommended_cmd = _parse_float(forward_visible.get("requested_a_cmd"), 0.0)
        forward_recommended_ms = _parse_int(forward_visible.get("pulse_ms"), 0)
        motion_source = "forward_visible"
        forward_motion_class = "visible"
        forward_threshold_unresolved = False
        ready_for_stage21_forward = True
    else:
        forward_recommended_cmd, forward_recommended_ms = None, None
        motion_source = "forward_twitch_unresolved" if forward_twitch is not None else "forward_unresolved"
        forward_motion_class = "twitch" if forward_twitch is not None else "none"
        forward_threshold_unresolved = True
        ready_for_stage21_forward = False

    if backward_visible is not None:
        backward_cmd = _parse_float(backward_visible.get("requested_a_cmd"), 0.0)
        backward_ms = _parse_int(backward_visible.get("pulse_ms"), 0)
        ready_for_stage21_backward = True
    else:
        backward_cmd, backward_ms = _fallback_next_step(_first_by_class(valid_backward_rows, "twitch"), direction="backward")
        ready_for_stage21_backward = bool(backward_cmd)
    backward_recommended_cmd = max(min(backward_cmd, 0.0), -0.12)
    backward_recommended_ms = min(max(backward_ms, 250), 500) if backward_ms else 0

    forward_twitch_cmd = _parse_float(forward_twitch.get("requested_a_cmd") if forward_twitch else None, 0.0)
    forward_visible_cmd = _parse_float(forward_visible.get("requested_a_cmd") if forward_visible else None, 0.0) if forward_visible else None
    backward_visible_cmd = _parse_float(backward_visible.get("requested_a_cmd") if backward_visible else None, 0.0) if backward_visible else None
    backward_visible_ms = _parse_int(backward_visible.get("pulse_ms") if backward_visible else None, 0) if backward_visible else None
    motor_power_or_mapping_suspect = forward_threshold_unresolved and not ready_for_stage21_backward
    return {
        **base_calibration,
        "forward_twitch_a_cmd": forward_twitch_cmd,
        "forward_twitch_pulse_ms": _parse_int(forward_twitch.get("pulse_ms") if forward_twitch else None, 0),
        "forward_visible_a_cmd": forward_visible_cmd,
        "forward_visible_pulse_ms": _parse_int(forward_visible.get("pulse_ms") if forward_visible else None, 0) if forward_visible else None,
        "forward_recommended_crawl_a_cmd": forward_recommended_cmd,
        "forward_recommended_crawl_pulse_ms": forward_recommended_ms,
        "backward_visible_a_cmd": backward_visible_cmd,
        "backward_visible_pulse_ms": backward_visible_ms,
        "backward_recommended_crawl_a_cmd": backward_recommended_cmd,
        "backward_recommended_crawl_pulse_ms": backward_recommended_ms,
        "forward_motion_class": forward_motion_class,
        "forward_threshold_unresolved": forward_threshold_unresolved,
        "forward_specific_deadband_or_sign_suspect": forward_threshold_unresolved and ready_for_stage21_backward,
        "motor_power_or_mapping_suspect": motor_power_or_mapping_suspect,
        "forward_is_weaker_than_backward": bool(forward_recommended_cmd and backward_recommended_cmd and abs(forward_recommended_cmd) > abs(backward_recommended_cmd)),
        "backward_is_sensitive": bool(backward_recommended_cmd and abs(backward_recommended_cmd) <= 0.10),
        "stage21_forward_a_cmd": forward_recommended_cmd,
        "stage21_forward_pulse_ms": forward_recommended_ms,
        "stage21_backward_a_cmd": backward_recommended_cmd,
        "stage21_backward_pulse_ms": backward_recommended_ms,
        "motion_class_source": motion_source,
        "turn_disabled": True,
        "ready_for_stage21_forward": ready_for_stage21_forward,
        "ready_for_stage21_backward": ready_for_stage21_backward,
        "ready_for_stage21_straight_only": bool(ready_for_stage21_forward and ready_for_stage21_backward),
        "ready_for_stage21_turning": False,
        "ready_for_full_path_following": False,
    }


def write_calibration(path: Path, calibration: dict[str, object]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(calibration, indent=2, sort_keys=True) + "\n", encoding="utf-8")


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Build Stage20C fine forward/backward motion calibration.")
    parser.add_argument("--forward-fine-dir", type=Path, required=True)
    parser.add_argument("--backward-fine-dir", type=Path, required=True)
    parser.add_argument("--base-calibration-json", type=Path, required=True)
    parser.add_argument("--out", type=Path, required=True)
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    base = json.loads(args.base_calibration_json.read_text(encoding="utf-8"))
    calibration = build_fine_calibration(
        forward_rows=_load_probe_rows(args.forward_fine_dir),
        backward_rows=_load_probe_rows(args.backward_fine_dir),
        base_calibration=base,
    )
    write_calibration(args.out, calibration)
    print(f"fine_calibration_json={args.out}")
    print(f"stage21_forward_a_cmd={calibration['stage21_forward_a_cmd']}")
    print(f"stage21_forward_pulse_ms={calibration['stage21_forward_pulse_ms']}")
    print(f"stage21_backward_a_cmd={calibration['stage21_backward_a_cmd']}")
    print(f"stage21_backward_pulse_ms={calibration['stage21_backward_pulse_ms']}")
    print(f"forward_threshold_unresolved={calibration['forward_threshold_unresolved']}")
    print(f"ready_for_stage21_forward={calibration['ready_for_stage21_forward']}")
    print("ready_for_full_path_following=false")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
