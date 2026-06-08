from __future__ import annotations

import argparse
import json
import math
import time
from pathlib import Path
from typing import Sequence

try:
    from tools import _bootstrap  # type: ignore  # noqa: F401
except ImportError:
    import _bootstrap  # type: ignore  # noqa: F401

from tools import station_path_package_tracker


def _bool(value: object) -> bool | None:
    text = str(value).strip().lower()
    if text in {"1", "true", "yes", "ok", "active"}:
        return True
    if text in {"0", "false", "no", "off", "inactive"}:
        return False
    return None


def _float(value: object) -> float | None:
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


def parse_rows(text: str) -> list[dict[str, str]]:
    return station_path_package_tracker.parse_usbdbg_rows(text)


def summarize_rows(rows: Sequence[dict[str, str]]) -> dict[str, object]:
    bmi_rows = [row for row in rows if row.get("imu_type") == "BMI160"]
    yaw_values = [value for row in rows if (value := _float(row.get("imu_relative_yaw_deg"))) is not None]
    present = any(_bool(row.get("imu_present")) is True for row in bmi_rows)
    pmu_normal = any(_bool(row.get("imu_pmu_normal")) is True for row in bmi_rows)
    plausible = any(_bool(row.get("imu_data_plausible")) is True for row in bmi_rows)
    calibrated = any(_bool(row.get("imu_calibrated")) is True for row in bmi_rows)
    heading_ready = any(_bool(row.get("imu_heading_ready")) is True for row in bmi_rows)
    diag_only = any(row.get("imu_heading_block_reason") == "RELATIVE_YAW_DIAG_ONLY" for row in bmi_rows)
    yaw_present = bool(yaw_values)
    if present and pmu_normal and plausible and calibrated and yaw_present and (heading_ready or diag_only):
        status = "IMU_READY"
    elif present and plausible and not calibrated:
        status = "IMU_CALIBRATING"
    elif not present:
        status = "IMU_NOT_PRESENT"
    else:
        status = "IMU_NOT_READY"
    return {
        "imu_status": status,
        "row_count": len(rows),
        "bmi160_rows": len(bmi_rows),
        "imu_type": "BMI160" if bmi_rows else "NA",
        "imu_present": present,
        "imu_pmu_normal": pmu_normal,
        "imu_data_plausible": plausible,
        "imu_calibrated": calibrated,
        "imu_heading_ready": heading_ready,
        "imu_relative_yaw_present": yaw_present,
        "imu_relative_yaw_min": min(yaw_values) if yaw_values else None,
        "imu_relative_yaw_max": max(yaw_values) if yaw_values else None,
        "imu_relative_yaw_span": (max(yaw_values) - min(yaw_values)) if yaw_values else None,
        "latest_imu_heading_block_reason": next((row.get("imu_heading_block_reason") for row in reversed(rows) if row.get("imu_heading_block_reason")), "NA"),
        "motor_command_generated": False,
        "ready_for_full_path_following": False,
    }


def write_summary(out_dir: Path, summary: dict[str, object], raw_lines: Sequence[str]) -> None:
    out_dir.mkdir(parents=True, exist_ok=True)
    (out_dir / "raw_usbdbg.log").write_text("\n".join(raw_lines) + ("\n" if raw_lines else ""), encoding="utf-8")
    (out_dir / "imu_readiness_summary.json").write_text(json.dumps(summary, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    lines = ["# IMU Readiness Watch", "", "No motion commands are sent.", ""]
    lines.extend(f"- {key}: `{value}`" for key, value in summary.items())
    (out_dir / "summary.md").write_text("\n".join(lines) + "\n", encoding="utf-8")


def run(args: argparse.Namespace) -> int:
    raw_lines: list[str] = []
    if args.log:
        raw_lines = Path(args.log).read_text(encoding="utf-8").splitlines()
    else:
        try:
            import serial  # type: ignore[import-not-found]
        except ImportError as exc:
            raise RuntimeError("pyserial is not available; cannot run IMU watch") from exc
        deadline = time.monotonic() + args.duration_s
        with serial.Serial(args.port, baudrate=115200, timeout=0.5) as handle:
            while time.monotonic() < deadline:
                raw = handle.readline()
                if raw:
                    line = raw.decode("utf-8", errors="replace").strip()
                    print(line)
                    raw_lines.append(line)
    summary = summarize_rows(parse_rows("\n".join(raw_lines)))
    write_summary(Path(args.out_dir), summary, raw_lines)
    print(f"imu_status={summary['imu_status']}")
    print("motor_command_generated=false")
    print("ready_for_full_path_following=false")
    return 0


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="BMI160 IMU readiness watch. No motor output.")
    parser.add_argument("--port", default="/dev/ttyACM0")
    parser.add_argument("--duration-s", type=float, default=60.0)
    parser.add_argument("--log")
    parser.add_argument("--out-dir", required=True)
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    try:
        return run(build_parser().parse_args(argv))
    except RuntimeError as exc:
        print(f"reason={exc}")
        print("motor_command_generated=false")
        print("ready_for_full_path_following=false")
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
