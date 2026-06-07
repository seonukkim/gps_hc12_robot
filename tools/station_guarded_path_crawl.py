from __future__ import annotations

import argparse
import csv
import json
import math
import re
import time
from pathlib import Path
from typing import Sequence

try:
    from tools import _bootstrap  # type: ignore  # noqa: F401
except ImportError:
    import _bootstrap  # type: ignore  # noqa: F401

from tools import station_path_package_tracker, station_virtual_path_controller
from tools.path_no_motion_validation import PathPackageResolutionError, resolve_path_package


CRAWL_FIELDS = (
    "row_index",
    "dry_run",
    "path_package_loaded",
    "georeference_available",
    "local_pose_available",
    "station_package_target_source",
    "firmware_active_target_source",
    "stage16_firmware_ready",
    "reason",
    "local_pose_source",
    "gps_used_for_local_pose",
    "gps_block_reason",
    "current_lat",
    "current_lon",
    "rc_ok",
    "active_primitive_index",
    "target_distance_m",
    "target_bearing_deg",
    "cross_track_error_m",
    "virtual_left_cmd",
    "virtual_right_cmd",
    "stage16_seq",
    "stage16_left_cmd",
    "stage16_right_cmd",
    "stage16_ms",
    "stage16_command_text",
    "command_sent",
    "arm_sent",
    "arm_ack_seen",
    "arm_reject_seen",
    "ack_seen",
    "reject_seen",
    "stop_seen",
    "latched_stop_before",
    "latched_stop_after",
    "command_rejected_reason",
    "estimated_distance_m",
    "motor_command_generated",
    "physical_output_active_seen",
    "ready_for_full_path_following",
)


def _parse_bool(value: object, default: bool = False) -> bool:
    if value is None:
        return default
    return str(value).strip().lower() in {"1", "true", "yes", "y", "ok"}


def _optional_float(value: object) -> float | None:
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


def _has_georeference(package: dict[str, object]) -> bool:
    georef = package.get("georeference")
    return isinstance(georef, dict) and _parse_bool(georef.get("georeference_available"))


def clamp_stage16_command(value: float, max_cmd: float) -> float:
    return max(-max_cmd, min(max_cmd, value))


def format_stage16_command(*, seq: int, left: float, right: float, pulse_ms: int) -> str:
    return f"STAGE16_CMD seq={seq} left={left:.3f} right={right:.3f} ms={pulse_ms}"


def parse_stage16_command_text(text: str, *, max_cmd: float = 0.04, max_ms: int = 150) -> dict[str, object]:
    if not text.startswith("STAGE16_CMD "):
        return {"valid": False, "reason": "NOT_STAGE16_CMD"}
    values = {key: value for key, value in re.findall(r"([A-Za-z0-9_]+)=([^,\s]+)", text)}
    try:
        seq = int(values["seq"])
        left = float(values["left"])
        right = float(values["right"])
        ms = int(values["ms"])
    except (KeyError, ValueError):
        return {"valid": False, "reason": "PARSE_ERROR"}
    if abs(left) > max_cmd or abs(right) > max_cmd:
        return {"valid": False, "reason": "COMMAND_EXCEEDS_MAX_CMD", "seq": seq}
    if ms < 1 or ms > max_ms:
        return {"valid": False, "reason": "COMMAND_EXCEEDS_MAX_MS", "seq": seq}
    return {"valid": True, "reason": "OK", "seq": seq, "left": left, "right": right, "ms": ms}


def parse_stage16_ready(rows: Sequence[dict[str, str]]) -> tuple[bool, bool, str, bool, bool, bool]:
    ready = False
    heartbeat_seen = False
    firmware_source = "unknown"
    ack_seen = False
    reject_seen = False
    stop_seen = False
    for row in rows:
        if _parse_bool(row.get("stage16_guarded_crawl")):
            if _parse_bool(row.get("stage16_firmware_ready")) or row.get("stage16_cmd_state", "").upper() == "STOPPED":
                ready = True
            if row.get("event", "").upper() == "HEARTBEAT":
                heartbeat_seen = True
        if "active_target_source" in row:
            firmware_source = row["active_target_source"]
        event = row.get("event", "").upper()
        state = row.get("stage16_cmd_state", "").upper()
        if event == "ACK" or state == "ACTIVE":
            ack_seen = True
        if event == "REJECT" or state == "REJECTED":
            reject_seen = True
        if event == "STOP" or state == "STOPPED":
            stop_seen = True
    return ready, heartbeat_seen, firmware_source, ack_seen, reject_seen, stop_seen


def parse_stage16_event_flags(rows: Sequence[dict[str, str]]) -> dict[str, object]:
    arm_ack_seen = False
    arm_reject_seen = False
    latched_values: list[bool] = []
    command_rejected_reason = "NONE"
    for row in rows:
        event = row.get("event", "").upper()
        state = row.get("stage16_cmd_state", "").upper()
        reason = row.get("stage16_reject_reason", "")
        if event == "ARM" or state == "ARMED":
            arm_ack_seen = True
        if event == "REJECT" and reason.startswith("ARM_"):
            arm_reject_seen = True
        if event == "REJECT" and reason and reason != "NONE":
            command_rejected_reason = reason
        if "latched_stop" in row:
            latched_values.append(_parse_bool(row.get("latched_stop")))
    return {
        "arm_ack_seen": arm_ack_seen,
        "arm_reject_seen": arm_reject_seen,
        "latched_stop_before": latched_values[0] if latched_values else "NA",
        "latched_stop_after": latched_values[-1] if latched_values else "NA",
        "command_rejected_reason": command_rejected_reason,
    }


def _gps_fields_present(rows: Sequence[dict[str, str]]) -> bool:
    return any("current_lat" in row or "current_lon" in row for row in rows)


def _gps_position_ready(rows: Sequence[dict[str, str]]) -> bool:
    return any(_optional_float(row.get("current_lat")) is not None and _optional_float(row.get("current_lon")) is not None for row in rows)


def _latest_value(rows: Sequence[dict[str, str]], key: str, default: str = "") -> str:
    for row in reversed(rows):
        if key in row:
            return row[key]
    return default


def diagnose_readiness(
    rows: Sequence[dict[str, str]],
    *,
    stage16_ready: bool,
    heartbeat_seen: bool,
    local_pose_available: bool,
    pose_mode: str,
) -> str:
    if not rows:
        return "NO_USBDBG_ROWS_READ"
    if not heartbeat_seen and not any(row.get("event", "").upper() in {"STOP", "ACK", "REJECT"} for row in rows):
        return "NO_STAGE16_HEARTBEAT"
    if not stage16_ready:
        return "STAGE16_READY_FALSE"
    rc_values = [_parse_bool(row.get("rc_ok")) for row in rows if "rc_ok" in row]
    if rc_values and rc_values[-1] is False:
        return "RC_NOT_OK"
    if pose_mode == "gps_georef":
        if not _gps_fields_present(rows):
            return "NO_GPS_POSITION_FIELDS_IN_STAGE16_LOG"
        if not _gps_position_ready(rows):
            return "GPS_POSITION_NOT_READY"
    if not local_pose_available:
        return "LOCAL_POSE_NOT_AVAILABLE"
    return "OK"


def bounded_command_from_virtual(row: dict[str, object], *, seq: int, max_cmd: float, pulse_ms: int) -> dict[str, object]:
    left = _optional_float(row.get("virtual_left_cmd")) or 0.0
    right = _optional_float(row.get("virtual_right_cmd")) or 0.0
    bounded_left = clamp_stage16_command(left, max_cmd)
    bounded_right = clamp_stage16_command(right, max_cmd)
    return {
        "stage16_seq": seq,
        "stage16_left_cmd": bounded_left,
        "stage16_right_cmd": bounded_right,
        "stage16_ms": pulse_ms,
        "stage16_command_text": format_stage16_command(
            seq=seq,
            left=bounded_left,
            right=bounded_right,
            pulse_ms=pulse_ms,
        ),
    }


def build_crawl_rows(
    package: dict[str, object],
    usbdbg_text: str,
    *,
    dry_run: bool,
    max_cmd: float,
    pulse_ms: int,
    max_total_distance_m: float,
    pose_mode: str = "gps_georef",
    current_x: float | None = None,
    current_y: float | None = None,
    current_heading_deg: float | None = None,
) -> list[dict[str, object]]:
    usbdbg_rows = station_path_package_tracker.parse_usbdbg_rows(usbdbg_text)
    stage16_ready, heartbeat_seen, firmware_source, ack_seen, reject_seen, stop_seen = parse_stage16_ready(usbdbg_rows)
    event_flags = parse_stage16_event_flags(usbdbg_rows)
    if pose_mode == "manual_local":
        if current_x is None or current_y is None:
            target_rows = [
                station_path_package_tracker.diagnostic_status(
                    mode="stage16_guarded_crawl",
                    row_index=0,
                    reason="MANUAL_LOCAL_POSE_REQUIRED",
                    firmware_active_target_source=firmware_source,
                )
            ]
        else:
            target_rows = [
                station_path_package_tracker.compute_target_status(
                    package,
                    current_x=current_x,
                    current_y=current_y,
                    current_heading_deg=current_heading_deg,
                    mode="stage16_guarded_crawl",
                    firmware_active_target_source=firmware_source,
                    local_pose_source="manual_local",
                )
            ]
    else:
        target_rows = station_path_package_tracker.build_rows_from_replay(package, usbdbg_text, "stage16_guarded_crawl")
    virtual_rows = station_virtual_path_controller.build_virtual_rows(
        target_rows,
        max_virtual_forward_cmd=max_cmd,
        max_virtual_turn_cmd=max_cmd,
    )
    output: list[dict[str, object]] = []
    estimated = 0.0
    for index, virtual in enumerate(virtual_rows):
        reason = diagnose_readiness(
            usbdbg_rows,
            stage16_ready=stage16_ready,
            heartbeat_seen=heartbeat_seen,
            local_pose_available=virtual.get("local_pose_available") is True,
            pose_mode=pose_mode,
        )
        command = bounded_command_from_virtual(virtual, seq=index + 1, max_cmd=max_cmd, pulse_ms=pulse_ms)
        average_cmd = (abs(float(command["stage16_left_cmd"])) + abs(float(command["stage16_right_cmd"]))) * 0.5
        estimated += average_cmd * pulse_ms / 1000.0
        command_allowed = (
            virtual.get("local_pose_available") is True
            and estimated <= max_total_distance_m + 1e-9
            and stage16_ready
            and reason == "OK"
            and (dry_run or event_flags["arm_ack_seen"] is True)
        )
        command_sent = bool(command_allowed and not dry_run)
        output.append(
            {
                "row_index": index,
                "dry_run": dry_run,
                "path_package_loaded": True,
                "georeference_available": _has_georeference(package),
                "local_pose_available": virtual.get("local_pose_available", False),
                "station_package_target_source": "path_package",
                "firmware_active_target_source": virtual.get("firmware_active_target_source", firmware_source),
                "stage16_firmware_ready": stage16_ready,
                "reason": reason,
                "local_pose_source": virtual.get("local_pose_source", "gps_georeference" if pose_mode == "gps_georef" else "manual_local"),
                "gps_used_for_local_pose": pose_mode == "gps_georef" and virtual.get("local_pose_available", False) is True,
                "gps_block_reason": _latest_value(usbdbg_rows, "gps_block_reason", "NA"),
                "current_lat": _latest_value(usbdbg_rows, "current_lat", "NA"),
                "current_lon": _latest_value(usbdbg_rows, "current_lon", "NA"),
                "rc_ok": _latest_value(usbdbg_rows, "rc_ok", "NA"),
                "active_primitive_index": virtual.get("active_primitive_index", "NA"),
                "target_distance_m": virtual.get("target_distance_m", "NA"),
                "target_bearing_deg": virtual.get("target_bearing_deg", "NA"),
                "cross_track_error_m": virtual.get("cross_track_error_m", "NA"),
                "virtual_left_cmd": virtual.get("virtual_left_cmd", 0.0),
                "virtual_right_cmd": virtual.get("virtual_right_cmd", 0.0),
                **command,
                "command_sent": command_sent,
                "arm_sent": False
                if dry_run
                else event_flags["arm_ack_seen"] is True or event_flags["arm_reject_seen"] is True,
                "arm_ack_seen": event_flags["arm_ack_seen"],
                "arm_reject_seen": event_flags["arm_reject_seen"],
                "ack_seen": ack_seen,
                "reject_seen": reject_seen,
                "stop_seen": stop_seen,
                "latched_stop_before": event_flags["latched_stop_before"],
                "latched_stop_after": event_flags["latched_stop_after"],
                "command_rejected_reason": event_flags["command_rejected_reason"],
                "estimated_distance_m": estimated,
                "motor_command_generated": command_sent,
                "physical_output_active_seen": any(
                    _parse_bool(row.get("physical_output_active")) for row in usbdbg_rows
                ),
                "ready_for_full_path_following": False,
            }
        )
    return output


def _write_csv(path: Path, rows: Sequence[dict[str, object]]) -> None:
    with path.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=CRAWL_FIELDS)
        writer.writeheader()
        for row in rows:
            writer.writerow({field: row.get(field, "") for field in CRAWL_FIELDS})


def _write_summary(path: Path, summary: dict[str, object]) -> None:
    lines = [
        "# Stage 16 Guarded Path Package Crawl",
        "",
        "USB-tethered station-supervised bounded crawl only. Not full path following.",
        "",
    ]
    for key, value in summary.items():
        lines.append(f"- {key}: `{value}`")
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def _summary(rows: Sequence[dict[str, object]], *, package: dict[str, object], dry_run: bool, max_cmd: float, pulse_ms: int, max_total_distance_m: float) -> dict[str, object]:
    first = rows[0] if rows else {}
    pulse_count = sum(1 for row in rows if row.get("command_sent") is True)
    arm_sent = any(row.get("arm_sent") is True for row in rows)
    arm_ack_seen = any(row.get("arm_ack_seen") is True for row in rows)
    arm_reject_seen = any(row.get("arm_reject_seen") is True for row in rows)
    ack_count = sum(1 for row in rows if row.get("ack_seen") is True)
    reject_count = sum(1 for row in rows if row.get("reject_seen") is True)
    stop_count = sum(1 for row in rows if row.get("stop_seen") is True)
    estimated = max((_optional_float(row.get("estimated_distance_m")) or 0.0 for row in rows), default=0.0)
    local_pose_available = bool(first.get("local_pose_available"))
    physical_seen = any(row.get("physical_output_active_seen") is True for row in rows)
    reason = first.get("reason", "NO_ROWS")
    ready_stage16 = (
        bool(first.get("path_package_loaded"))
        and bool(first.get("georeference_available"))
        and local_pose_available
        and bool(first.get("stage16_firmware_ready"))
        and max_cmd <= 0.04
        and pulse_ms <= 150
        and not physical_seen
        and reason == "OK"
        and (dry_run or stop_count > 0)
    )
    return {
        "path_package_loaded": bool(rows),
        "georeference_available": _has_georeference(package),
        "local_pose_available": local_pose_available,
        "station_package_target_source": "path_package",
        "firmware_active_target_source": first.get("firmware_active_target_source", "unknown"),
        "stage16_firmware_ready": first.get("stage16_firmware_ready", False),
        "reason": reason,
        "local_pose_source": first.get("local_pose_source", "unknown"),
        "gps_used_for_local_pose": first.get("gps_used_for_local_pose", False),
        "gps_block_reason": first.get("gps_block_reason", "NA"),
        "current_lat": first.get("current_lat", "NA"),
        "current_lon": first.get("current_lon", "NA"),
        "rc_ok": first.get("rc_ok", "NA"),
        "dry_run": dry_run,
        "arm_sent": arm_sent,
        "arm_ack_seen": arm_ack_seen,
        "arm_reject_seen": arm_reject_seen,
        "latched_stop_before": first.get("latched_stop_before", "NA"),
        "latched_stop_after": first.get("latched_stop_after", "NA"),
        "command_rejected_reason": first.get("command_rejected_reason", "NONE"),
        "pulse_count": pulse_count,
        "ack_count": ack_count,
        "reject_count": reject_count,
        "stop_count": stop_count,
        "max_cmd": max_cmd,
        "pulse_ms": pulse_ms,
        "max_total_distance_m": max_total_distance_m,
        "estimated_distance_m": estimated,
        "motor_command_generated": any(row.get("motor_command_generated") is True for row in rows),
        "physical_output_active_seen": physical_seen,
        "final_stop_confirmed": dry_run or stop_count > 0,
        "ready_for_stage16_guarded_crawl": ready_stage16,
        "ready_for_full_path_following": False,
    }


def _plot_trace(path: Path, rows: Sequence[dict[str, object]]) -> None:
    try:
        import matplotlib

        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError:
        return
    indices = [int(row["row_index"]) for row in rows]
    left = [float(row.get("stage16_left_cmd", 0.0)) for row in rows]
    right = [float(row.get("stage16_right_cmd", 0.0)) for row in rows]
    fig, ax = plt.subplots(figsize=(8, 4))
    ax.plot(indices, left, label="bounded left")
    ax.plot(indices, right, label="bounded right")
    ax.axhline(0.0, color="0.4", linewidth=0.8)
    ax.set_title("Stage 16 Guarded Crawl Proposed Commands")
    ax.set_xlabel("row")
    ax.set_ylabel("bounded diagnostic command")
    ax.grid(True, linestyle="--", alpha=0.3)
    ax.legend()
    ax.text(0.01, 0.01, "Not full path following", transform=ax.transAxes, fontsize=8)
    fig.tight_layout()
    fig.savefig(path, dpi=150)
    plt.close(fig)


def _read_live_lines(port: str, duration_s: float, *, startup_wait_s: float = 0.0, command_callback=None) -> str:
    try:
        import serial  # type: ignore[import-not-found]
    except ImportError as exc:
        raise RuntimeError("pyserial is not available; cannot open Stage 16 serial") from exc
    lines: list[str] = []
    with serial.Serial(port, baudrate=115200, timeout=0.5) as handle:
        if startup_wait_s > 0:
            time.sleep(startup_wait_s)
        deadline = time.monotonic() + max(duration_s, 0.1)
        while time.monotonic() < deadline:
            raw = handle.readline()
            if raw:
                line = raw.decode("utf-8", errors="replace").strip()
                print(line)
                lines.append(line)
                if command_callback is not None:
                    command_callback(handle, "\n".join(lines))
    return "\n".join(lines)


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Stage 16 guarded station-to-OpenRB path-package crawl.")
    parser.add_argument("--path-package", default="latest")
    parser.add_argument("--port", required=True)
    parser.add_argument("--duration-s", type=float, default=30.0)
    parser.add_argument("--max-cmd", type=float, default=0.03)
    parser.add_argument("--pulse-ms", type=int, default=100)
    parser.add_argument("--max-total-distance-m", type=float, default=0.20)
    parser.add_argument("--require-enter", default="true")
    parser.add_argument("--dry-run", default="true")
    parser.add_argument("--startup-wait-s", type=float, default=0.0)
    parser.add_argument("--pose-mode", choices=("gps_georef", "manual_local"), default="gps_georef")
    parser.add_argument("--current-x", type=float)
    parser.add_argument("--current-y", type=float)
    parser.add_argument("--current-heading-deg", type=float)
    parser.add_argument("--out-dir", required=True)
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    dry_run = _parse_bool(args.dry_run, default=True)
    require_enter = _parse_bool(args.require_enter, default=True)
    try:
        selected = resolve_path_package(args.path_package)
    except PathPackageResolutionError as exc:
        print(f"provided_path_package={exc.provided}")
        print("file_exists=false")
        print("nearest_candidates:")
        for candidate in exc.candidates:
            print(f"- {candidate}")
        return 2
    package = json.loads(selected.read_text(encoding="utf-8"))
    out_dir = Path(args.out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    seq_state = {"seq": 1, "arm_sent": False, "command_sent": False, "stop_sent": False}

    def maybe_send(handle: object, text: str) -> None:
        if dry_run:
            return
        usbdbg_rows = station_path_package_tracker.parse_usbdbg_rows(text)
        event_flags = parse_stage16_event_flags(usbdbg_rows)
        rows = build_crawl_rows(
            package,
            text,
            dry_run=False,
            max_cmd=args.max_cmd,
            pulse_ms=args.pulse_ms,
            max_total_distance_m=args.max_total_distance_m,
            pose_mode=args.pose_mode,
            current_x=args.current_x,
            current_y=args.current_y,
            current_heading_deg=args.current_heading_deg,
        )
        if not rows or rows[-1]["stage16_firmware_ready"] is not True:
            return
        if not seq_state["arm_sent"]:
            arm_command = f"STAGE16_ARM seq={seq_state['seq']}"
            if require_enter:
                input(f"Press Enter to arm Stage 16 guarded crawl: {arm_command} ")
            handle.write((arm_command + "\n").encode("ascii"))
            handle.flush()
            seq_state["arm_sent"] = True
            return
        if event_flags["arm_ack_seen"] is not True or seq_state["command_sent"]:
            return
        command = str(rows[-1]["stage16_command_text"])
        if require_enter:
            input(f"Press Enter to send one bounded Stage 16 pulse: {command} ")
        handle.write((command + "\n").encode("ascii"))
        handle.flush()
        seq_state["command_sent"] = True
        time.sleep(max(args.pulse_ms / 1000.0 + 0.05, 0.05))
        stop_command = f"STAGE16_STOP seq={seq_state['seq']}"
        handle.write((stop_command + "\n").encode("ascii"))
        handle.flush()
        seq_state["stop_sent"] = True

    log_text = _read_live_lines(
        args.port,
        args.duration_s,
        startup_wait_s=args.startup_wait_s,
        command_callback=maybe_send,
    )
    raw_log_path = out_dir / "raw_usbdbg.log"
    raw_log_path.write_text(log_text + ("\n" if log_text else ""), encoding="utf-8")
    rows = build_crawl_rows(
        package,
        log_text,
        dry_run=dry_run,
        max_cmd=args.max_cmd,
        pulse_ms=args.pulse_ms,
        max_total_distance_m=args.max_total_distance_m,
        pose_mode=args.pose_mode,
        current_x=args.current_x,
        current_y=args.current_y,
        current_heading_deg=args.current_heading_deg,
    )
    if not dry_run:
        try:
            import serial  # type: ignore[import-not-found]
            with serial.Serial(args.port, baudrate=115200, timeout=0.5) as handle:
                handle.write(b"STAGE16_STOP seq=999999\n")
                handle.flush()
        except Exception:
            pass
    csv_path = out_dir / "stage16_guarded_crawl.csv"
    summary_path = out_dir / "summary.md"
    preview_path = out_dir / "preview_crawl_trace.png"
    _write_csv(csv_path, rows)
    summary = _summary(
        rows,
        package=package,
        dry_run=dry_run,
        max_cmd=args.max_cmd,
        pulse_ms=args.pulse_ms,
        max_total_distance_m=args.max_total_distance_m,
    )
    _write_summary(summary_path, summary)
    _plot_trace(preview_path, rows)
    print("Stage 16 guarded path-package crawl complete.")
    print(f"selected_path_package={selected}")
    print(f"stage16_guarded_crawl_csv={csv_path}")
    print(f"raw_usbdbg_log={raw_log_path}")
    print(f"summary_md={summary_path}")
    for key in (
        "dry_run",
        "stage16_firmware_ready",
        "arm_sent",
        "arm_ack_seen",
        "arm_reject_seen",
        "command_rejected_reason",
        "pulse_count",
        "motor_command_generated",
        "ready_for_stage16_guarded_crawl",
        "ready_for_full_path_following",
    ):
        print(f"{key}={summary[key]}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
