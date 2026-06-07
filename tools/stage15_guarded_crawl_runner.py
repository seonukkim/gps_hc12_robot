from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path
from typing import Sequence

try:
    from tools import _bootstrap  # type: ignore  # noqa: F401
except ImportError:
    import _bootstrap  # type: ignore  # noqa: F401


PULSE_SEQUENCE = (
    "test_forward_pulse",
    "test_backward_pulse",
    "test_rotate_left_pulse",
    "test_rotate_right_pulse",
)


def _parse_bool(value: str) -> bool:
    return value.strip().lower() in {"1", "true", "yes", "y"}


def _read_available(handle: object, log_handle: object, deadline: float, *, stop_when_confirmed: bool) -> None:
    while time.monotonic() < deadline:
        raw = handle.readline()
        if not raw:
            continue
        line = raw.decode("utf-8", errors="replace").rstrip()
        print(line)
        log_handle.write(line + "\n")
        log_handle.flush()
        if stop_when_confirmed and "pulse_type=test_stop" in line and "stop_confirmed=true" in line:
            return


def run_stage15_sequence(
    *,
    port: str,
    out_dir: Path,
    max_cmd: float,
    pulse_ms: int,
    inter_pulse_ms: int,
    require_enter: bool,
    baud: int = 115200,
) -> int:
    try:
        import serial  # type: ignore[import-not-found]
    except ImportError as exc:
        raise RuntimeError("pyserial is not available; cannot run Stage 15 crawl") from exc

    out_dir.mkdir(parents=True, exist_ok=True)
    log_path = out_dir / "guarded_crawl.log"
    summary_path = out_dir / "summary.md"

    with serial.Serial(port, baudrate=baud, timeout=0.2) as handle, log_path.open("w", encoding="utf-8") as log:
        header = (
            "station_stage15_runner=true "
            f"port={port} max_cmd={max_cmd:.3f} pulse_ms={pulse_ms} "
            f"inter_pulse_ms={inter_pulse_ms} require_enter={str(require_enter).lower()} "
            "path_following_enable=false path_package_used=false virtual_controller_used=false "
            "motor_command_generated=false\n"
        )
        print(header.rstrip())
        log.write(header)
        time.sleep(2.0)
        _read_available(handle, log, time.monotonic() + 1.0, stop_when_confirmed=False)

        for pulse in PULSE_SEQUENCE:
            if require_enter:
                input(f"Press Enter to send {pulse} ({max_cmd:.3f}, {pulse_ms} ms), or Ctrl-C to abort: ")
            pre_line = (
                f"station_trigger pulse_type={pulse} pulse_ms={pulse_ms} max_cmd={max_cmd:.3f} "
                "path_package_used=false virtual_controller_used=false motor_command_generated=false"
            )
            print(pre_line)
            log.write(pre_line + "\n")
            log.flush()
            handle.write((pulse + "\n").encode("ascii"))
            handle.flush()
            _read_available(
                handle,
                log,
                time.monotonic() + max(5.0, pulse_ms / 1000.0 + 2.0),
                stop_when_confirmed=True,
            )
            time.sleep(max(inter_pulse_ms, 0) / 1000.0)

        handle.write(b"test_stop\n")
        handle.flush()
        _read_available(handle, log, time.monotonic() + 2.0, stop_when_confirmed=True)

    summary_path.write_text(
        "\n".join(
            [
                "# Stage 15 Guarded Motor Sanity Crawl",
                "",
                "- stage15_guarded_crawl: `true`",
                "- path_following_enable: `false`",
                "- path_package_used: `false`",
                "- virtual_controller_used: `false`",
                "- motor_command_generated: `false`",
                "- log: `guarded_crawl.log`",
            ]
        )
        + "\n",
        encoding="utf-8",
    )
    print(f"log={log_path}")
    print(f"summary={summary_path}")
    return 0


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Interactive Stage 15 guarded crawl USB trigger runner.")
    parser.add_argument("--port", required=True)
    parser.add_argument("--max-cmd", type=float, default=0.06)
    parser.add_argument("--pulse-ms", type=int, default=200)
    parser.add_argument("--inter-pulse-ms", type=int, default=2000)
    parser.add_argument("--require-enter", default="true")
    parser.add_argument("--out-dir", type=Path, default=Path("outputs/stage15_guarded_crawl/latest"))
    args = parser.parse_args(argv)

    return run_stage15_sequence(
        port=args.port,
        out_dir=args.out_dir,
        max_cmd=args.max_cmd,
        pulse_ms=args.pulse_ms,
        inter_pulse_ms=args.inter_pulse_ms,
        require_enter=_parse_bool(args.require_enter),
    )


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except KeyboardInterrupt:
        print("Stage 15 guarded crawl interrupted by user.", file=sys.stderr)
        raise SystemExit(130)
