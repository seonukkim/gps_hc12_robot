from __future__ import annotations

import argparse
import re
import time
from pathlib import Path
from typing import Sequence

try:
    from tools import _bootstrap  # type: ignore  # noqa: F401
except ImportError:
    import _bootstrap  # type: ignore  # noqa: F401


def _parse_bool(value: object) -> bool:
    return str(value).strip().lower() in {"1", "true", "yes", "ok"}


def parse_rows(text: str) -> list[dict[str, str]]:
    rows = []
    for line in text.splitlines():
        values = {key.lower(): value for key, value in re.findall(r"([A-Za-z0-9_]+)=([^,\s]+)", line)}
        if values:
            rows.append(values)
    return rows


def evaluate_probe(text: str) -> dict[str, object]:
    rows = parse_rows(text)
    stage16_rows = [row for row in rows if _parse_bool(row.get("stage16_guarded_crawl"))]
    heartbeat_or_stop = [
        row for row in stage16_rows if row.get("event", "").upper() in {"HEARTBEAT", "STOP"}
    ]
    firmware_ready = any(_parse_bool(row.get("stage16_firmware_ready")) for row in stage16_rows)
    physical_safe = all(not _parse_bool(row.get("physical_output_active")) for row in stage16_rows)
    final_zero = all(
        row.get("final_left_cmd", "0.000") in {"0.000", "0", "0.0"}
        and row.get("final_right_cmd", "0.000") in {"0.000", "0", "0.0"}
        for row in stage16_rows
    )
    gps_fields_present = any("current_lat" in row and "current_lon" in row for row in stage16_rows)
    if not stage16_rows:
        verdict = "FAIL"
        reason = "NO_STAGE16_ROWS"
    elif not heartbeat_or_stop:
        verdict = "FAIL"
        reason = "NO_HEARTBEAT_OR_STOP"
    elif not firmware_ready:
        verdict = "FAIL"
        reason = "STAGE16_READY_FALSE"
    elif not physical_safe or not final_zero:
        verdict = "FAIL"
        reason = "PHYSICAL_OUTPUT_NOT_SAFE"
    elif not gps_fields_present:
        verdict = "WAIT"
        reason = "GPS_POSITION_FIELDS_MISSING"
    else:
        verdict = "PASS"
        reason = "OK"
    return {
        "verdict": verdict,
        "reason": reason,
        "stage16_row_count": len(stage16_rows),
        "heartbeat_or_stop_count": len(heartbeat_or_stop),
        "stage16_firmware_ready": firmware_ready,
        "physical_output_active": False if physical_safe else True,
        "final_commands_zero": final_zero,
        "gps_fields_present": gps_fields_present,
    }


def read_serial(port: str, duration_s: float) -> str:
    try:
        import serial  # type: ignore[import-not-found]
    except ImportError as exc:
        raise RuntimeError("pyserial is not available; cannot probe Stage 16 heartbeat") from exc
    lines: list[str] = []
    with serial.Serial(port, baudrate=115200, timeout=0.5) as handle:
        deadline = time.monotonic() + max(duration_s, 0.1)
        while time.monotonic() < deadline:
            raw = handle.readline()
            if raw:
                line = raw.decode("utf-8", errors="replace").strip()
                print(line)
                lines.append(line)
    return "\n".join(lines)


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Probe Stage 16 heartbeat without sending motor commands.")
    parser.add_argument("--port", required=True)
    parser.add_argument("--duration-s", type=float, default=15.0)
    parser.add_argument("--out-dir", type=Path, default=Path("outputs/stage16_heartbeat_probe/latest"))
    args = parser.parse_args(argv)
    args.out_dir.mkdir(parents=True, exist_ok=True)
    text = read_serial(args.port, args.duration_s)
    raw_path = args.out_dir / "raw_stage16_heartbeat.log"
    raw_path.write_text(text + ("\n" if text else ""), encoding="utf-8")
    result = evaluate_probe(text)
    summary_path = args.out_dir / "summary.md"
    summary_path.write_text(
        "# Stage 16 Heartbeat Probe\n\n"
        + "\n".join(f"- {key}: `{value}`" for key, value in result.items())
        + "\n",
        encoding="utf-8",
    )
    print(f"stage16_heartbeat_probe={result['verdict']}")
    print(f"reason={result['reason']}")
    print(f"raw_log={raw_path}")
    print(f"summary={summary_path}")
    return 0 if result["verdict"] in {"PASS", "WAIT"} else 1


if __name__ == "__main__":
    raise SystemExit(main())
