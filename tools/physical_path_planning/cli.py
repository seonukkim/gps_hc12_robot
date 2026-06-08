"""Unified physical-path-planning CLI: one entrypoint, five modes.

    preview         build the A->B serpentine (or direct) plan and render it -- no
                    serial, no firmware, no motion. Works even with NO calibration.
    calibrate-turn  shell out to scripts/run_stage20_physical_ab_probe.sh with
                    --imu-angle-compare true (that script compiles the BMI160 yaw
                    flags and uploads); this CLI never opens serial for it.
    execute-plan /  run the continuous-motion controller over a planned path
    run             (opens serial + drives guarded pulses when invoked at the field).
    diagnose        read-only telemetry summary, from a live port or --from-log FILE.

Every summary is routed through ``checks.assert_not_ready_for_full_path_following``
so no mode can ever claim full-path-following readiness. The hardware modes open
serial only when actually invoked; ``--print-plan`` / ``--print-cmd`` / ``--from-log``
give fully no-hardware paths for previewing exactly what would run.
"""
from __future__ import annotations

import argparse
import csv
import json
import shlex
import subprocess
import sys
import time
from pathlib import Path
from typing import Sequence

from tools.physical_path_planning import calibration, checks, controller, preview, telemetry

DEFAULT_PORT = "/dev/ttyACM0"
DEFAULT_BAUD = 115200
DEFAULT_STAGE20_SCRIPT = "scripts/run_stage20_physical_ab_probe.sh"
DEFAULT_TURN_CALIBRATION_OUT = (
    "outputs/stage23_turn_calibration/calibration/physical_ab_turn_angle_calibration.json"
)


# --- Pure, no-hardware helpers (directly unit-testable) -----------------------


def build_calibrate_turn_argv(
    *,
    script: str,
    port: str,
    mode: str,
    target_angle_deg: float,
    angle_tolerance_deg: float,
    save_turn_calibration: str,
    turn_calibration_out: str,
    out_dir: str,
) -> list[str]:
    """Build the argv that shells out to the Stage20 probe for turn-angle calibration.

    Always passes ``--imu-angle-compare true`` -- that is what makes the launcher
    append ``-DIMU_ENABLE=1 -DIMU_YAW_DIAG=1`` and measure before/after yaw. This
    CLI delegates the firmware compile/upload entirely to that script.
    """
    return [
        "bash",
        str(script),
        "--port",
        str(port),
        "--mode",
        str(mode),
        "--imu-angle-compare",
        "true",
        "--target-angle-deg",
        str(target_angle_deg),
        "--angle-tolerance-deg",
        str(angle_tolerance_deg),
        "--save-turn-calibration",
        str(save_turn_calibration),
        "--turn-calibration-out",
        str(turn_calibration_out),
        "--out-dir",
        str(out_dir),
    ]


def resolve_calibration(args: argparse.Namespace) -> dict[str, object]:
    """Resolve calibration honoring real on-disk files, with explicit overrides.

    Unspecified ``--*-calibration-json`` flags fall back to the resolver's default
    on-disk paths (so genuine calibration is used when present); a missing file
    degrades to the repeated-pulses fallback and never raises.
    """
    kwargs: dict[str, object] = {"calibration_mode": args.calibration_mode}
    for flag, key in (
        ("fine_calibration_json", "fine_calibration_json"),
        ("turn_calibration_json", "turn_calibration_json"),
        ("turn_angle_calibration_json", "turn_angle_calibration_json"),
        ("smooth_turn_calibration_json", "smooth_turn_calibration_json"),
    ):
        value = getattr(args, flag)
        if value is not None:
            kwargs[key] = Path(value)
    return calibration.resolve_physical_calibration(**kwargs)


def resolve_plan(args: argparse.Namespace, calibration_dict: dict[str, object]) -> dict[str, object]:
    """Build the no-motion plan (segments + goal) shared by preview and run."""
    return preview.build_preview(
        start_lat=args.start_lat,
        start_lon=args.start_lon,
        goal_mode=args.goal_mode,
        goal_lat=args.goal_lat,
        goal_lon=args.goal_lon,
        goal_east_m=args.goal_east_m,
        goal_north_m=args.goal_north_m,
        goal_dlat=args.goal_dlat,
        goal_dlon=args.goal_dlon,
        goal_bearing_deg=args.goal_bearing_deg,
        goal_distance_m=args.goal_distance_m,
        path_shape=args.path_shape,
        workspace_width_m=args.workspace_width_m,
        step_spacing_m=args.step_spacing_m,
        diagonal_orientation=args.diagonal_orientation,
        max_segment_pulses=args.max_segment_pulses,
        nominal_forward_pulse_m=args.nominal_forward_pulse_m,
        calibration=calibration_dict,
    )


def load_rows_from_log(path: Path) -> list[dict[str, str]]:
    """Parse USBDBG telemetry rows from a saved serial log (no serial needed)."""
    return telemetry.parse_usbdbg_rows(path.read_text())


def load_planner_config(path: Path) -> dict[str, object]:
    """Load a shipped JSON config, dropping ``_``-prefixed comment keys.

    Used for the ``configs/*.json`` starting points. A ``field_rectangle_example``
    config loads with keys that are exactly :func:`preview.build_preview` kwargs,
    so ``build_preview(**load_planner_config(path))`` runs it directly.
    """
    data = json.loads(path.read_text())
    if not isinstance(data, dict):
        raise ValueError(f"config {path} must be a JSON object")
    return {key: value for key, value in data.items() if not key.startswith("_")}


def diagnose_summary(rows: Sequence[dict[str, str]]) -> dict[str, object]:
    """Summarize telemetry rows into a read-only, never-ready diagnostic dict."""
    heartbeats = [r for r in rows if telemetry.event(r) == "HEARTBEAT"]
    last = heartbeats[-1] if heartbeats else None
    event_counts: dict[str, int] = {}
    for row in rows:
        name = telemetry.event(row) or "NONE"
        event_counts[name] = event_counts.get(name, 0) + 1
    summary: dict[str, object] = {
        "stage": "diagnose",
        "row_count": len(rows),
        "heartbeat_count": len(heartbeats),
        "event_counts": event_counts,
        "stage20_compatible": controller.stage20_compatible(last) if last else False,
        "physical_output_active": telemetry.physical_output_active(last) if last else False,
        "last_gps_block_reason": telemetry.gps_block_reason(last) if last else "NA",
        "last_gps_sats": telemetry._fmt(telemetry.gps_sats(last)) if last else "NA",
        "last_gps_hdop": telemetry._fmt(telemetry.gps_hdop(last)) if last else "NA",
        "last_imu_relative_yaw_deg": (
            telemetry._fmt(telemetry.imu_relative_yaw_deg(last)) if last else "NA"
        ),
        "ready_for_full_path_following": False,
    }
    return checks.assert_not_ready_for_full_path_following(summary)


# --- Output writers -----------------------------------------------------------


def _write_json(path: Path, obj: object) -> None:
    path.write_text(json.dumps(obj, indent=2, default=str) + "\n")


def _write_rows_csv(path: Path, rows: Sequence[dict[str, object]]) -> None:
    if not rows:
        path.write_text("")
        return
    fieldnames: list[str] = []
    for row in rows:
        for key in row.keys():
            if key not in fieldnames:
                fieldnames.append(key)
    with path.open("w", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        for row in rows:
            writer.writerow(row)


def _write_raw_log(path: Path, lines: Sequence[str]) -> None:
    path.write_text("\n".join(lines) + ("\n" if lines else ""))


def _fail(message: str) -> int:
    print(f"ABORT: {message}", file=sys.stderr)
    return 2


# --- Mode handlers ------------------------------------------------------------


def cmd_preview(args: argparse.Namespace) -> int:
    cal = resolve_calibration(args)
    try:
        plan = resolve_plan(args, cal)
    except ValueError as exc:
        return _fail(str(exc))
    out_dir = Path(args.out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    _write_json(out_dir / "preview_summary.json", plan)
    if args.png:
        png = preview.write_preview_png(
            out_dir / "preview.png",
            plan["segments"],  # type: ignore[arg-type]
            float(plan["start_lat"]),
            float(plan["start_lon"]),
            float(plan["goal_lat"]),
            float(plan["goal_lon"]),
            plan["workspace"],  # type: ignore[arg-type]
        )
        if png is None:
            print("preview: matplotlib unavailable; skipped PNG render")
    print(
        f"preview: {plan['segment_count']} segments, "
        f"{plan['lane_count']} lanes, goal_distance_m={float(plan['goal_distance_m']):.3f} -> {out_dir}"
    )
    return 0


def cmd_calibrate_turn(args: argparse.Namespace) -> int:
    argv = build_calibrate_turn_argv(
        script=args.script,
        port=args.port,
        mode=args.mode,
        target_angle_deg=args.target_angle_deg,
        angle_tolerance_deg=args.angle_tolerance_deg,
        save_turn_calibration=args.save_turn_calibration,
        turn_calibration_out=args.turn_calibration_out,
        out_dir=args.out_dir,
    )
    printable = " ".join(shlex.quote(part) for part in argv)
    if args.print_cmd:
        print(printable)
        return 0
    print(f"calibrate-turn: invoking {printable}")
    completed = subprocess.run(argv, check=False)
    return completed.returncode


def cmd_run(args: argparse.Namespace) -> int:
    cal = resolve_calibration(args)
    try:
        plan = resolve_plan(args, cal)
    except ValueError as exc:
        return _fail(str(exc))
    out_dir = Path(args.out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    if args.print_plan:
        _write_json(out_dir / "plan.json", plan)
        print(
            f"run --print-plan: {plan['segment_count']} segments, "
            f"fallback_to_repeated_pulses={cal['fallback_to_repeated_pulses']} "
            f"-> {out_dir}/plan.json (no serial opened)"
        )
        return 0

    import serial  # local import: preview/diagnose --from-log never need pyserial

    handle = serial.Serial(args.port, baudrate=args.baud, timeout=0.5)
    try:
        rows, raw_lines, abort_reason = controller.run_controller(
            handle,
            segments=plan["segments"],  # type: ignore[arg-type]
            resolved_calibration=cal,
            start_lat=float(plan["start_lat"]),
            start_lon=float(plan["start_lon"]),
            start_yaw_deg=args.start_yaw_deg,
            goal_lat=float(plan["goal_lat"]),
            goal_lon=float(plan["goal_lon"]),
            event_timeout_s=args.event_timeout_s,
            heartbeat_timeout_s=args.heartbeat_timeout_s,
            rc_neutral_wait_s=args.rc_neutral_wait_s,
            gps_degradation_policy=args.gps_degradation_policy,
            manual_override_mode=args.manual_override_mode,
            left_fixed_pulses=args.left_fixed_pulses,
            right_fixed_pulses=args.right_fixed_pulses,
        )
    finally:
        handle.close()

    summary = controller.build_controller_summary(
        rows,
        start_lat=float(plan["start_lat"]),
        start_lon=float(plan["start_lon"]),
        goal_lat=float(plan["goal_lat"]),
        goal_lon=float(plan["goal_lon"]),
        goal_distance_m=float(plan["goal_distance_m"]),
        fallback_to_repeated_pulses=bool(cal["fallback_to_repeated_pulses"]),
        abort_reason=abort_reason,
    )
    _write_json(out_dir / "run_summary.json", summary)
    _write_rows_csv(out_dir / "run_rows.csv", rows)
    _write_raw_log(out_dir / "run_serial.log", raw_lines)
    print(
        f"run: abort_reason={abort_reason}, pulses={summary['pulse_count']}, "
        f"valid={summary['valid_pulse_count']} -> {out_dir}"
    )
    return 1 if summary["aborted"] else 0


def cmd_diagnose(args: argparse.Namespace) -> int:
    out_dir = Path(args.out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    if args.from_log:
        log_path = Path(args.from_log)
        raw_lines = log_path.read_text().splitlines()
        rows = load_rows_from_log(log_path)
    else:
        import serial  # local import: --from-log path never needs pyserial

        raw_lines = []
        handle = serial.Serial(args.port, baudrate=args.baud, timeout=0.5)
        try:
            deadline = time.monotonic() + args.duration_s
            while time.monotonic() < deadline:
                raw = handle.readline()
                if raw:
                    line = raw.decode("utf-8", errors="replace").strip()
                    print(line)
                    raw_lines.append(line)
        finally:
            handle.close()
        rows = telemetry.parse_usbdbg_rows("\n".join(raw_lines))

    summary = diagnose_summary(rows)
    _write_json(out_dir / "diagnose_summary.json", summary)
    if raw_lines:
        _write_raw_log(out_dir / "diagnose_serial.log", raw_lines)
    print(
        f"diagnose: {summary['row_count']} rows, {summary['heartbeat_count']} heartbeats, "
        f"last_gps_block_reason={summary['last_gps_block_reason']} -> {out_dir}"
    )
    return 0


# --- Argument parser ----------------------------------------------------------


def _add_goal_arguments(parser: argparse.ArgumentParser) -> None:
    parser.add_argument("--start-lat", type=float, required=True)
    parser.add_argument("--start-lon", type=float, required=True)
    parser.add_argument(
        "--goal-mode",
        choices=["absolute", "relative_enu", "relative_latlon", "bearing_distance"],
        default="absolute",
    )
    parser.add_argument("--goal-lat", type=float, default=None)
    parser.add_argument("--goal-lon", type=float, default=None)
    parser.add_argument("--goal-east-m", type=float, default=None)
    parser.add_argument("--goal-north-m", type=float, default=None)
    parser.add_argument("--goal-dlat", type=float, default=None)
    parser.add_argument("--goal-dlon", type=float, default=None)
    parser.add_argument("--goal-bearing-deg", type=float, default=None)
    parser.add_argument("--goal-distance-m", type=float, default=None)
    parser.add_argument(
        "--path-shape",
        choices=["diagonal_rectangle_serpentine", "direct_line"],
        default="diagonal_rectangle_serpentine",
    )
    parser.add_argument("--workspace-width-m", type=float, default=None)
    parser.add_argument("--step-spacing-m", type=float, default=0.5)
    parser.add_argument(
        "--diagonal-orientation", default="A_top_left_to_B_bottom_right"
    )
    parser.add_argument("--max-segment-pulses", type=int, default=8)
    parser.add_argument("--nominal-forward-pulse-m", type=float, default=0.30)


def _add_calibration_arguments(parser: argparse.ArgumentParser) -> None:
    parser.add_argument("--calibration-mode", default="auto")
    parser.add_argument("--fine-calibration-json", default=None)
    parser.add_argument("--turn-calibration-json", default=None)
    parser.add_argument("--turn-angle-calibration-json", default=None)
    parser.add_argument("--smooth-turn-calibration-json", default=None)


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        prog="physical_path_planner",
        description="Integrated A->B serpentine path planning, calibration, and guarded motion.",
    )
    sub = parser.add_subparsers(dest="mode", required=True)

    preview_p = sub.add_parser("preview", help="build + render the plan (no serial, no motion)")
    _add_goal_arguments(preview_p)
    _add_calibration_arguments(preview_p)
    preview_p.add_argument("--out-dir", default="outputs/physical_path_planning/preview")
    preview_p.add_argument("--png", dest="png", action="store_true", default=True)
    preview_p.add_argument("--no-png", dest="png", action="store_false")
    preview_p.set_defaults(handler=cmd_preview)

    cal_p = sub.add_parser(
        "calibrate-turn",
        help="shell out to the Stage20 probe with --imu-angle-compare true",
    )
    cal_p.add_argument("--port", default=DEFAULT_PORT)
    cal_p.add_argument("--mode", default="turn_left")
    cal_p.add_argument("--target-angle-deg", type=float, default=90.0)
    cal_p.add_argument("--angle-tolerance-deg", type=float, default=10.0)
    cal_p.add_argument("--save-turn-calibration", default="false")
    cal_p.add_argument("--turn-calibration-out", default=DEFAULT_TURN_CALIBRATION_OUT)
    cal_p.add_argument("--out-dir", default="outputs/physical_path_planning/calibrate_turn")
    cal_p.add_argument("--script", default=DEFAULT_STAGE20_SCRIPT)
    cal_p.add_argument(
        "--print-cmd",
        action="store_true",
        help="print the shell-out command and exit (no firmware, no serial)",
    )
    cal_p.set_defaults(handler=cmd_calibrate_turn)

    for name in ("run", "execute-plan"):
        run_p = sub.add_parser(name, help="drive the continuous-motion controller over a plan")
        _add_goal_arguments(run_p)
        _add_calibration_arguments(run_p)
        run_p.add_argument("--port", default=DEFAULT_PORT)
        run_p.add_argument("--baud", type=int, default=DEFAULT_BAUD)
        run_p.add_argument("--start-yaw-deg", type=float, default=None)
        run_p.add_argument("--event-timeout-s", type=float, default=controller.DEFAULT_EVENT_TIMEOUT_S)
        run_p.add_argument(
            "--heartbeat-timeout-s", type=float, default=controller.DEFAULT_HEARTBEAT_TIMEOUT_S
        )
        run_p.add_argument(
            "--rc-neutral-wait-s", type=float, default=controller.DEFAULT_RC_NEUTRAL_WAIT_S
        )
        run_p.add_argument(
            "--gps-degradation-policy",
            choices=["continue", "pause", "abort"],
            default=controller.DEFAULT_GPS_DEGRADATION_POLICY,
        )
        run_p.add_argument(
            "--manual-override-mode",
            choices=["abort", "continue"],
            default=controller.DEFAULT_MANUAL_OVERRIDE_MODE,
        )
        run_p.add_argument("--left-fixed-pulses", type=int, default=12)
        run_p.add_argument("--right-fixed-pulses", type=int, default=12)
        run_p.add_argument("--out-dir", default="outputs/physical_path_planning/run")
        run_p.add_argument(
            "--print-plan",
            action="store_true",
            help="build + write the plan and exit (no serial opened)",
        )
        run_p.set_defaults(handler=cmd_run)

    diag_p = sub.add_parser("diagnose", help="read-only telemetry summary (live port or --from-log)")
    diag_p.add_argument("--port", default=DEFAULT_PORT)
    diag_p.add_argument("--baud", type=int, default=DEFAULT_BAUD)
    diag_p.add_argument("--from-log", default=None, help="parse a saved serial log instead of a port")
    diag_p.add_argument("--duration-s", type=float, default=5.0)
    diag_p.add_argument("--out-dir", default="outputs/physical_path_planning/diagnose")
    diag_p.set_defaults(handler=cmd_diagnose)

    return parser


def main(argv: Sequence[str] | None = None) -> int:
    parser = build_parser()
    args = parser.parse_args(argv)
    return int(args.handler(args))


if __name__ == "__main__":
    raise SystemExit(main())
