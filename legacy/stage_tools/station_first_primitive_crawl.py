from __future__ import annotations

import argparse
import csv
import json
import math
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
    "station_package_target_source",
    "pose_mode",
    "local_pose_available",
    "firmware_mode",
    "firmware_ready",
    "firmware_active_target_source",
    "physical_path_following_enable",
    "allow_motor_output",
    "active_primitive_index",
    "active_tool_segment_id",
    "target_distance_m",
    "target_bearing_deg",
    "cross_track_error_m",
    "virtual_left_cmd",
    "virtual_right_cmd",
    "stage17_left_cmd",
    "stage17_right_cmd",
    "stage17_ms",
    "stage17_command_text",
    "force_straight_test",
    "diagnostic_only",
    "arm_sent",
    "arm_ack_seen",
    "ack_seen",
    "stop_seen",
    "reject_seen",
    "reject_reason",
    "pulse_count",
    "estimated_distance_m",
    "visible_motion_user_report",
    "final_left_cmd",
    "final_right_cmd",
    "final_left_cmd_zero",
    "final_right_cmd_zero",
    "physical_output_active_seen",
    "physical_output_active_after_stop",
    "ready_for_next_stage",
    "ready_for_full_path_following",
)

VISIBLE_RESPONSES = {"forward", "backward", "left", "right", "twitch"}
RESPONSE_CHOICES = VISIBLE_RESPONSES | {"none", "unknown"}


def _parse_bool(value: object, default: bool = False) -> bool:
    if value is None:
        return default
    return str(value).strip().lower() in {"1", "true", "yes", "y", "ok", "active"}


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


def _clamp(value: float, limit: float) -> float:
    return max(-limit, min(limit, value))


def _latest(rows: Sequence[dict[str, str]], key: str, default: str = "NA") -> str:
    for row in reversed(rows):
        if key in row:
            return row[key]
    return default


def format_stage_command(*, prefix: str, seq: int, left: float, right: float, pulse_ms: int) -> str:
    return f"{prefix}_CMD seq={seq} left={left:.3f} right={right:.3f} ms={pulse_ms}"


def detect_firmware_mode(rows: Sequence[dict[str, str]]) -> dict[str, object]:
    stage17 = any(_parse_bool(row.get("stage17_first_primitive_crawl")) for row in rows)
    stage16 = any(_parse_bool(row.get("stage16_guarded_crawl")) for row in rows)
    ready = any(
        _parse_bool(row.get("stage17_firmware_ready")) or _parse_bool(row.get("stage16_firmware_ready"))
        for row in rows
    )
    if stage17:
        return {"firmware_mode": "stage17", "command_prefix": "STAGE17", "firmware_ready": ready}
    if stage16:
        return {"firmware_mode": "stage16", "command_prefix": "STAGE16", "firmware_ready": ready}
    return {"firmware_mode": "unknown", "command_prefix": "STAGE17", "firmware_ready": False}


def command_from_virtual(
    virtual_row: dict[str, object],
    *,
    max_cmd: float,
    pulse_ms: int,
    seq: int,
    command_prefix: str = "STAGE17",
    force_straight_test: bool = False,
) -> dict[str, object]:
    if force_straight_test:
        left = max_cmd
        right = max_cmd
    else:
        left = _clamp(_optional_float(virtual_row.get("virtual_left_cmd")) or 0.0, max_cmd)
        right = _clamp(_optional_float(virtual_row.get("virtual_right_cmd")) or 0.0, max_cmd)
    return {
        "stage17_left_cmd": left,
        "stage17_right_cmd": right,
        "stage17_ms": pulse_ms,
        "stage17_command_text": format_stage_command(
            prefix=command_prefix,
            seq=seq,
            left=left,
            right=right,
            pulse_ms=pulse_ms,
        ),
        "arm_command_text": f"{command_prefix}_ARM seq={seq}",
        "stop_command_text": f"{command_prefix}_STOP seq={seq}",
    }


def _target_row(
    package: dict[str, object],
    usbdbg_text: str,
    *,
    pose_mode: str,
    current_x: float | None,
    current_y: float | None,
    current_heading_deg: float | None,
) -> dict[str, object]:
    if pose_mode == "manual_local":
        if current_x is None or current_y is None:
            return station_path_package_tracker.diagnostic_status(
                mode="stage17_first_primitive_crawl",
                row_index=0,
                reason="MANUAL_LOCAL_POSE_REQUIRED",
                firmware_active_target_source="unknown",
            )
        return station_path_package_tracker.compute_target_status(
            package,
            current_x=current_x,
            current_y=current_y,
            current_heading_deg=current_heading_deg,
            mode="stage17_first_primitive_crawl",
            firmware_active_target_source="station_first_primitive_guarded",
            local_pose_source="manual_local",
        )
    rows = station_path_package_tracker.build_rows_from_replay(package, usbdbg_text, "stage17_first_primitive_crawl")
    return rows[0] if rows else station_path_package_tracker.diagnostic_status(
        mode="stage17_first_primitive_crawl",
        row_index=0,
        reason="NO_USBDBG_ROWS_READ",
        firmware_active_target_source="unknown",
    )


def build_crawl_row(
    package: dict[str, object],
    usbdbg_text: str,
    *,
    dry_run: bool,
    pose_mode: str,
    current_x: float | None,
    current_y: float | None,
    current_heading_deg: float | None,
    max_cmd: float,
    pulse_ms: int,
    max_total_distance_m: float,
    seq: int = 1,
    force_straight_test: bool = False,
    visible_motion_user_report: str = "unknown",
) -> dict[str, object]:
    usb_rows = station_path_package_tracker.parse_usbdbg_rows(usbdbg_text)
    firmware = detect_firmware_mode(usb_rows)
    target = _target_row(
        package,
        usbdbg_text,
        pose_mode=pose_mode,
        current_x=current_x,
        current_y=current_y,
        current_heading_deg=current_heading_deg,
    )
    virtual = station_virtual_path_controller.compute_virtual_control(
        target,
        max_virtual_forward_cmd=max_cmd,
        max_virtual_turn_cmd=max_cmd,
    )
    command = command_from_virtual(
        virtual,
        max_cmd=max_cmd,
        pulse_ms=pulse_ms,
        seq=seq,
        command_prefix=str(firmware["command_prefix"]),
        force_straight_test=force_straight_test,
    )
    estimated_distance = (
        abs(float(command["stage17_left_cmd"])) + abs(float(command["stage17_right_cmd"]))
    ) * 0.5 * pulse_ms / 1000.0
    ack_seen = any(row.get("event", "").upper() == "ACK" for row in usb_rows)
    arm_ack_seen = any(row.get("event", "").upper() == "ARM" for row in usb_rows)
    stop_seen = any(row.get("event", "").upper() == "STOP" for row in usb_rows)
    reject_seen = any(row.get("event", "").upper() == "REJECT" for row in usb_rows)
    final_left = _optional_float(_latest(usb_rows, "final_left_cmd", "0.0")) or 0.0
    final_right = _optional_float(_latest(usb_rows, "final_right_cmd", "0.0")) or 0.0
    physical_active_after_stop = any(
        row.get("event", "").upper() == "STOP" and _parse_bool(row.get("physical_output_active"))
        for row in usb_rows
    )
    physical_seen = any(_parse_bool(row.get("physical_output_active")) for row in usb_rows)
    local_pose_available = target.get("local_pose_available") is True
    ready_next = (
        local_pose_available
        and bool(firmware["firmware_ready"])
        and estimated_distance <= max_total_distance_m + 1e-9
        and not physical_active_after_stop
        and abs(final_left) <= 1e-9
        and abs(final_right) <= 1e-9
    )
    active_source = _latest(usb_rows, "active_target_source", "unknown")
    for row in usb_rows:
        if row.get("event", "").upper() == "ACK" or row.get("stage16_cmd_state", "").upper() == "ACTIVE":
            active_source = row.get("active_target_source", active_source)
    return {
        "row_index": 0,
        "dry_run": dry_run,
        "path_package_loaded": True,
        "station_package_target_source": "path_package",
        "pose_mode": pose_mode,
        "local_pose_available": local_pose_available,
        "firmware_mode": firmware["firmware_mode"],
        "firmware_ready": firmware["firmware_ready"],
        "firmware_active_target_source": active_source,
        "physical_path_following_enable": _latest(usb_rows, "physical_path_following_enable", "false"),
        "allow_motor_output": _latest(usb_rows, "allow_motor_output", "false"),
        "active_primitive_index": target.get("active_primitive_index", "NA"),
        "active_tool_segment_id": target.get("active_tool_segment_id", "NA"),
        "target_distance_m": target.get("target_distance_m", "NA"),
        "target_bearing_deg": target.get("target_bearing_deg", "NA"),
        "cross_track_error_m": target.get("cross_track_error_m", "NA"),
        "virtual_left_cmd": virtual.get("virtual_left_cmd", 0.0),
        "virtual_right_cmd": virtual.get("virtual_right_cmd", 0.0),
        **command,
        "force_straight_test": force_straight_test,
        "diagnostic_only": force_straight_test,
        "arm_sent": False if dry_run else arm_ack_seen,
        "arm_ack_seen": arm_ack_seen,
        "ack_seen": ack_seen,
        "stop_seen": stop_seen,
        "reject_seen": reject_seen,
        "reject_reason": _latest(usb_rows, "stage16_reject_reason", "NONE"),
        "pulse_count": 1 if ack_seen else 0,
        "estimated_distance_m": estimated_distance if ack_seen else 0.0,
        "visible_motion_user_report": visible_motion_user_report,
        "final_left_cmd": f"{final_left:.3f}",
        "final_right_cmd": f"{final_right:.3f}",
        "final_left_cmd_zero": abs(final_left) <= 1e-9,
        "final_right_cmd_zero": abs(final_right) <= 1e-9,
        "physical_output_active_seen": physical_seen,
        "physical_output_active_after_stop": physical_active_after_stop,
        "ready_for_next_stage": ready_next and ack_seen and stop_seen,
        "ready_for_full_path_following": False,
    }


def _write_csv(path: Path, rows: Sequence[dict[str, object]]) -> None:
    with path.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=CRAWL_FIELDS)
        writer.writeheader()
        for row in rows:
            writer.writerow({field: row.get(field, "") for field in CRAWL_FIELDS})


def _write_summary(path: Path, summary: dict[str, object]) -> None:
    lines = [
        "# Stage 17 First-Primitive Crawl",
        "",
        "Bounded first-primitive crawl only. Not full path following.",
        "",
    ]
    for key, value in summary.items():
        lines.append(f"- {key}: `{value}`")
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def build_summary(rows: Sequence[dict[str, object]]) -> dict[str, object]:
    first = rows[0] if rows else {}
    estimated = sum(_optional_float(row.get("estimated_distance_m")) or 0.0 for row in rows)
    return {
        "path_package_loaded": bool(rows),
        "station_package_target_source": "path_package",
        "pose_mode": first.get("pose_mode", "unknown"),
        "active_primitive_index": first.get("active_primitive_index", "NA"),
        "target_distance_m": first.get("target_distance_m", "NA"),
        "target_bearing_deg": first.get("target_bearing_deg", "NA"),
        "virtual_left_cmd": first.get("virtual_left_cmd", 0.0),
        "virtual_right_cmd": first.get("virtual_right_cmd", 0.0),
        "stage17_left_cmd": first.get("stage17_left_cmd", 0.0),
        "stage17_right_cmd": first.get("stage17_right_cmd", 0.0),
        "pulse_count": sum(1 for row in rows if row.get("ack_seen") is True),
        "ack_count": sum(1 for row in rows if row.get("ack_seen") is True),
        "stop_count": sum(1 for row in rows if row.get("stop_seen") is True),
        "estimated_distance_m": estimated,
        "visible_motion_user_report": first.get("visible_motion_user_report", "unknown"),
        "final_stop_confirmed": bool(rows) and rows[-1].get("stop_seen") is True,
        "final_left_cmd_zero": bool(rows) and rows[-1].get("final_left_cmd_zero") is True,
        "final_right_cmd_zero": bool(rows) and rows[-1].get("final_right_cmd_zero") is True,
        "ready_for_next_stage": any(row.get("ready_for_next_stage") is True for row in rows),
        "ready_for_full_path_following": False,
    }


def _plot_trace(path: Path, rows: Sequence[dict[str, object]]) -> None:
    try:
        import matplotlib

        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError:
        return
    indices = [index for index, _ in enumerate(rows)]
    left = [float(row.get("stage17_left_cmd", 0.0)) for row in rows]
    right = [float(row.get("stage17_right_cmd", 0.0)) for row in rows]
    fig, ax = plt.subplots(figsize=(8, 4))
    ax.plot(indices, left, marker="o", label="stage17_left_cmd")
    ax.plot(indices, right, marker="o", label="stage17_right_cmd")
    ax.axhline(0.0, color="0.4", linewidth=0.8)
    ax.set_title("Stage 17 First-Primitive Bounded Pulses")
    ax.set_xlabel("pulse")
    ax.set_ylabel("bounded command")
    ax.grid(True, linestyle="--", alpha=0.3)
    ax.legend()
    ax.text(0.01, 0.01, "Not full path following", transform=ax.transAxes, fontsize=8)
    fig.tight_layout()
    fig.savefig(path, dpi=150)
    plt.close(fig)


def _wait_for_event(handle: object, raw_lines: list[str], wanted: set[str], timeout_s: float) -> list[dict[str, str]]:
    deadline = time.monotonic() + timeout_s
    start_index = len(raw_lines)
    while time.monotonic() < deadline:
        raw = handle.readline()
        if raw:
            line = raw.decode("utf-8", errors="replace").strip()
            print(line)
            raw_lines.append(line)
            new_rows = station_path_package_tracker.parse_usbdbg_rows("\n".join(raw_lines[start_index:]))
            if any(row.get("event", "").upper() in wanted for row in new_rows):
                return station_path_package_tracker.parse_usbdbg_rows("\n".join(raw_lines))
    return station_path_package_tracker.parse_usbdbg_rows("\n".join(raw_lines))


def _prompt_visible_motion() -> str:
    while True:
        response = input("Did the rover visibly move? [none/twitch/forward/backward/left/right/unknown] ").strip().lower()
        if response in RESPONSE_CHOICES:
            return response
        print("Use one of: none, twitch, forward, backward, left, right, unknown")


def _read_serial_session(args: argparse.Namespace, package: dict[str, object]) -> tuple[str, list[dict[str, object]]]:
    try:
        import serial  # type: ignore[import-not-found]
    except ImportError as exc:
        raise RuntimeError("pyserial is not available; cannot run Stage 17 first-primitive crawl") from exc
    raw_lines: list[str] = []
    rows: list[dict[str, object]] = []
    dry_run = _parse_bool(args.dry_run, default=True)
    require_enter = _parse_bool(args.require_enter, default=True)
    force_straight = _parse_bool(args.force_straight_test, default=False)
    allow_next = _parse_bool(args.allow_next_primitive, default=False)
    estimated_total = 0.0
    seq = 1
    with serial.Serial(args.port, baudrate=115200, timeout=0.5) as handle:
        start_deadline = time.monotonic() + max(args.duration_s, 0.1)
        _wait_for_event(handle, raw_lines, {"HEARTBEAT", "STOP"}, min(10.0, max(args.duration_s, 0.1)))
        while time.monotonic() < start_deadline and estimated_total < args.max_total_distance_m:
            text = "\n".join(raw_lines)
            proposed = build_crawl_row(
                package,
                text,
                dry_run=dry_run,
                pose_mode=args.pose_mode,
                current_x=args.current_x,
                current_y=args.current_y,
                current_heading_deg=args.current_heading_deg,
                max_cmd=args.max_cmd,
                pulse_ms=args.pulse_ms,
                max_total_distance_m=args.max_total_distance_m,
                seq=seq,
                force_straight_test=force_straight,
            )
            if dry_run:
                rows.append(proposed)
                break
            if proposed["local_pose_available"] is not True or proposed["firmware_ready"] is not True:
                rows.append(proposed)
                break
            if require_enter:
                input(f"Press Enter to send Stage17 first-primitive pulse: {proposed['stage17_command_text']} ")
            handle.write((str(proposed["arm_command_text"]) + "\n").encode("ascii"))
            handle.flush()
            _wait_for_event(handle, raw_lines, {"ARM", "REJECT"}, 5.0)
            armed_row = build_crawl_row(
                package,
                "\n".join(raw_lines),
                dry_run=False,
                pose_mode=args.pose_mode,
                current_x=args.current_x,
                current_y=args.current_y,
                current_heading_deg=args.current_heading_deg,
                max_cmd=args.max_cmd,
                pulse_ms=args.pulse_ms,
                max_total_distance_m=args.max_total_distance_m,
                seq=seq,
                force_straight_test=force_straight,
            )
            if armed_row["arm_ack_seen"] is not True:
                rows.append(armed_row)
                break
            handle.write((str(armed_row["stage17_command_text"]) + "\n").encode("ascii"))
            handle.flush()
            _wait_for_event(handle, raw_lines, {"ACK", "REJECT"}, 5.0)
            _wait_for_event(handle, raw_lines, {"STOP"}, max(5.0, args.pulse_ms / 1000.0 + 1.0))
            handle.write((str(armed_row["stop_command_text"]) + "\n").encode("ascii"))
            handle.flush()
            _wait_for_event(handle, raw_lines, {"STOP"}, 5.0)
            visible = _prompt_visible_motion()
            completed = build_crawl_row(
                package,
                "\n".join(raw_lines),
                dry_run=False,
                pose_mode=args.pose_mode,
                current_x=args.current_x,
                current_y=args.current_y,
                current_heading_deg=args.current_heading_deg,
                max_cmd=args.max_cmd,
                pulse_ms=args.pulse_ms,
                max_total_distance_m=args.max_total_distance_m,
                seq=seq,
                force_straight_test=force_straight,
                visible_motion_user_report=visible,
            )
            rows.append(completed)
            if completed["final_left_cmd_zero"] is not True or completed["final_right_cmd_zero"] is not True:
                raise RuntimeError("Stage17 final wheel commands were nonzero; aborting")
            if completed["physical_output_active_after_stop"] is True:
                raise RuntimeError("Stage17 output remained active after STOP; aborting")
            estimated_total += _optional_float(completed.get("estimated_distance_m")) or 0.0
            seq += 1
            if not allow_next:
                break
    return "\n".join(raw_lines), rows


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Stage 17 bounded first-primitive path-package crawl.")
    parser.add_argument("--path-package", default="latest")
    parser.add_argument("--port", required=True)
    parser.add_argument("--pose-mode", choices=("manual_local", "gps_georef"), default="gps_georef")
    parser.add_argument("--current-x", type=float)
    parser.add_argument("--current-y", type=float)
    parser.add_argument("--current-heading-deg", type=float)
    parser.add_argument("--max-cmd", type=float, default=0.045)
    parser.add_argument("--pulse-ms", type=int, default=300)
    parser.add_argument("--max-total-distance-m", type=float, default=0.20)
    parser.add_argument("--duration-s", type=float, default=30.0)
    parser.add_argument("--require-enter", default="true")
    parser.add_argument("--dry-run", default="true")
    parser.add_argument("--force-straight-test", default="false")
    parser.add_argument("--allow-next-primitive", default="false")
    parser.add_argument("--out-dir", required=True)
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    if args.max_cmd > 0.08:
        raise SystemExit("--max-cmd > 0.080 is not allowed without a separate dangerous override wrapper")
    if args.pulse_ms > 800:
        raise SystemExit("--pulse-ms > 800 is not allowed without a separate dangerous override wrapper")
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
    raw_text, rows = _read_serial_session(args, package)
    raw_path = out_dir / "raw_usbdbg.log"
    csv_path = out_dir / "stage17_first_primitive_crawl.csv"
    summary_path = out_dir / "summary.md"
    preview_path = out_dir / "preview_crawl_trace.png"
    raw_path.write_text(raw_text + ("\n" if raw_text else ""), encoding="utf-8")
    _write_csv(csv_path, rows)
    summary = build_summary(rows)
    summary["selected_path_package"] = str(selected)
    _write_summary(summary_path, summary)
    _plot_trace(preview_path, rows)
    print("Stage 17 first-primitive crawl complete.")
    print(f"selected_path_package={selected}")
    print(f"stage17_first_primitive_crawl_csv={csv_path}")
    print(f"raw_usbdbg_log={raw_path}")
    print(f"summary_md={summary_path}")
    print(f"ready_for_next_stage={summary['ready_for_next_stage']}")
    print("ready_for_full_path_following=false")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
