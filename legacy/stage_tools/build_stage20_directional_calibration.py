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


VISIBLE_BODY = {"forward", "backward", "left", "right", "twitch"}
VISIBLE_WHEEL = {"yes", "twitch"}
TURN_ORDER = {
    "none": 0,
    "unknown": 0,
    "twitch": 1,
    "small_under_15deg": 2,
    "medium_15_45deg": 3,
    "big_over_45deg": 4,
}


def _parse_bool(value: object) -> bool:
    return str(value).strip().lower() in {"1", "true", "yes", "ok", "active"}


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


def _load_probe_rows(path: Path | None) -> list[dict[str, str]]:
    if path is None:
        return []
    csv_path = path / "stage20_physical_ab_probe.csv" if path.is_dir() else path
    if not csv_path.exists():
        return []
    with csv_path.open("r", encoding="utf-8", newline="") as handle:
        return [{key: value for key, value in row.items() if key is not None} for row in csv.DictReader(handle)]


def _motion_visible(row: dict[str, str]) -> bool:
    if _parse_bool(row.get("visible_motion_confirmed")):
        return True
    return (
        row.get("body_motion_user_report", "unknown") in VISIBLE_BODY
        or row.get("left_wheel_user_report", "unknown") in VISIBLE_WHEEL
        or row.get("right_wheel_user_report", "unknown") in VISIBLE_WHEEL
    )


def _first_visible(rows: Sequence[dict[str, str]]) -> dict[str, str] | None:
    for row in rows:
        if _motion_visible(row):
            return row
    return None


def motion_class(row: dict[str, str] | None) -> str:
    if row is None:
        return "unknown"
    body = row.get("body_motion_user_report", "unknown")
    if body == "twitch":
        return "twitch"
    if body in {"forward", "backward", "left", "right"}:
        return "visible"
    if row.get("left_wheel_user_report") == "twitch" or row.get("right_wheel_user_report") == "twitch":
        return "twitch"
    return "unknown"


def turn_gain_class(row: dict[str, str] | None, *, direction: str) -> str:
    if row is None:
        return "unknown"
    angle = row.get("turn_angle_report", "unknown")
    body = row.get("body_motion_user_report", "unknown")
    if direction == "right" and angle == "big_over_45deg":
        return "overstrong"
    if angle == "big_over_45deg":
        return "strong"
    if angle == "medium_15_45deg":
        return "normal"
    if angle == "small_under_15deg":
        return "weak"
    if angle == "twitch" or body == "twitch":
        return "weak"
    if body in {"left", "right"}:
        return "normal"
    return "unknown"


def _safe_turn_command(row: dict[str, str] | None, *, gain_class: str, key: str) -> float:
    if row is None:
        return 0.0
    value = _parse_float(row.get(key), 0.0)
    if gain_class == "overstrong":
        return max(-0.06, min(0.06, value))
    return value


def build_calibration(
    *,
    forward_rows: Sequence[dict[str, str]],
    backward_rows: Sequence[dict[str, str]],
    turn_left_rows: Sequence[dict[str, str]],
    turn_right_rows: Sequence[dict[str, str]],
) -> dict[str, object]:
    forward = _first_visible(forward_rows)
    backward = _first_visible(backward_rows)
    turn_left = _first_visible(turn_left_rows)
    turn_right = _first_visible(turn_right_rows)

    forward_min = _parse_float(forward.get("requested_a_cmd") if forward else None, 0.0)
    backward_min = _parse_float(backward.get("requested_a_cmd") if backward else None, 0.0)
    turn_left_min = _parse_float(turn_left.get("requested_b_cmd") if turn_left else None, 0.0)
    turn_right_min = _parse_float(turn_right.get("requested_b_cmd") if turn_right else None, 0.0)
    left_class = turn_gain_class(turn_left, direction="left")
    right_class = turn_gain_class(turn_right, direction="right")
    asymmetry = (
        left_class != right_class
        or (turn_left is not None and turn_right is not None and abs(abs(turn_left_min) - abs(turn_right_min)) > 0.02)
    )
    ready_straight = forward is not None and backward is not None
    ready_turning = (
        turn_left is not None
        and turn_right is not None
        and left_class in {"normal", "strong"}
        and right_class in {"normal", "strong"}
        and not asymmetry
    )
    return {
        "forward_sign": 1.0,
        "turn_sign": 1.0,
        "a_positive_direction": "forward",
        "a_negative_direction": "backward",
        "forward_min_a_cmd": forward_min,
        "forward_min_pulse_ms": _parse_int(forward.get("pulse_ms") if forward else None, 0),
        "forward_motion_class": motion_class(forward),
        "backward_min_a_cmd": backward_min,
        "backward_min_pulse_ms": _parse_int(backward.get("pulse_ms") if backward else None, 0),
        "backward_motion_class": motion_class(backward),
        "turn_left_min_b_cmd": turn_left_min,
        "turn_left_min_pulse_ms": _parse_int(turn_left.get("pulse_ms") if turn_left else None, 0),
        "turn_right_min_b_cmd": turn_right_min,
        "turn_right_min_pulse_ms": _parse_int(turn_right.get("pulse_ms") if turn_right else None, 0),
        "turn_left_gain_class": left_class,
        "turn_right_gain_class": right_class,
        "turn_asymmetry_detected": asymmetry,
        "safe_turn_left_cmd": _safe_turn_command(turn_left, gain_class=left_class, key="requested_b_cmd"),
        "safe_turn_right_cmd": _safe_turn_command(turn_right, gain_class=right_class, key="requested_b_cmd"),
        "safe_turn_left_pulse_ms": _parse_int(turn_left.get("pulse_ms") if turn_left else None, 0),
        "safe_turn_right_pulse_ms": min(_parse_int(turn_right.get("pulse_ms") if turn_right else None, 0), 250)
        if right_class == "overstrong"
        else _parse_int(turn_right.get("pulse_ms") if turn_right else None, 0),
        "max_forward_cmd": 0.20,
        "max_backward_cmd": 0.12,
        "max_turn_left_cmd": max(abs(turn_left_min), 0.0),
        "max_turn_right_cmd": min(abs(turn_right_min), 0.10) if right_class == "overstrong" else max(abs(turn_right_min), 0.0),
        "ready_for_stage21_straight_only": ready_straight,
        "ready_for_stage21_turning": ready_turning,
        "ready_for_full_path_following": False,
    }


def write_calibration(path: Path, calibration: dict[str, object]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(calibration, indent=2, sort_keys=True) + "\n", encoding="utf-8")


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Build Stage20B directional physical A/B calibration JSON.")
    parser.add_argument("--forward-dir", type=Path, required=True)
    parser.add_argument("--backward-dir", type=Path, required=True)
    parser.add_argument("--turn-left-dir", type=Path, required=True)
    parser.add_argument("--turn-right-dir", type=Path, required=True)
    parser.add_argument("--out", type=Path, required=True)
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    calibration = build_calibration(
        forward_rows=_load_probe_rows(args.forward_dir),
        backward_rows=_load_probe_rows(args.backward_dir),
        turn_left_rows=_load_probe_rows(args.turn_left_dir),
        turn_right_rows=_load_probe_rows(args.turn_right_dir),
    )
    write_calibration(args.out, calibration)
    print(f"calibration_json={args.out}")
    print(f"ready_for_stage21_straight_only={str(calibration['ready_for_stage21_straight_only']).lower()}")
    print(f"ready_for_stage21_turning={str(calibration['ready_for_stage21_turning']).lower()}")
    print("ready_for_full_path_following=false")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
