"""Generate a Markdown HC-12 diagnosis report from recent diagnostic logs.

Parses the most recent of each log type under a log directory and writes a
verdict + next-action report. It reads logs only; it opens no serial ports and
sends nothing.

Log sources (most recent of each):
  outputs/logs/uart_sweep_openrb*.log        (arduino-cli monitor | tee ...)
  outputs/logs/serial_raw_read*.log          (tools/serial_raw_read.py)
  outputs/logs/hc12_link_probe*.log          (tools/hc12_link_probe.py)
  outputs/logs/hc12_operational_diagnose*.log(tools/hc12_operational_diagnose.py)
"""

from __future__ import annotations

import argparse
import datetime as dt
import re
from pathlib import Path
from typing import Sequence

try:
    from tools import _bootstrap  # type: ignore  # noqa: F401
except ImportError:
    import _bootstrap  # type: ignore  # noqa: F401

from tools.serial_raw_read import detect_uart_ports

LOG_PATTERNS = {
    "uart_sweep": "uart_sweep_openrb*.log",
    "serial_raw": "serial_raw_read*.log",
    "link_probe": "hc12_link_probe*.log",
    "operational": "hc12_operational_diagnose*.log",
}

VERDICTS = (
    "USB_SERIAL_STABLE_NO_RF_BYTES",
    "UART_SWEEP_RECEIVED_ON_SERIAL3",
    "UART_SWEEP_RECEIVED_ON_SERIAL1",
    "UART_SWEEP_RECEIVED_ON_SERIAL2",
    "STATION_TO_OPENRB_RX_DETECTED",
    "STATION_TX_OK_NO_RX",
    "HC12_LINK_OK",
    "HC12_NO_RF_RX",
    "STATION_USB_UNSTABLE",
    "TEST_INVALID_STATION_OFF",
)


def parse_sweep_counters(text: str) -> dict:
    """Latest per-UART tx/rx counters and any RX_FIRST_DETECTED ports."""
    out: dict = {}
    for n in (1, 2, 3):
        tx = re.findall(rf"Serial{n}_tx=(\d+)", text)
        rx = re.findall(rf"Serial{n}_rx=(\d+)", text)
        out[f"serial{n}_tx"] = int(tx[-1]) if tx else None
        out[f"serial{n}_rx"] = int(rx[-1]) if rx else None
    out["rx_first_detected"] = sorted(
        {int(m) for m in re.findall(r"RX_FIRST_DETECTED port=Serial(\d)", text)}
    )
    return out


def parse_int_field(text: str, field: str) -> int | None:
    matches = re.findall(rf"{field}=(\d+)", text)
    return int(matches[-1]) if matches else None


def parse_total_bytes(text: str) -> int:
    vals = re.findall(r"total_bytes=(\d+)", text)
    return int(vals[-1]) if vals else 0


def parse_pong_count(text: str) -> int:
    # operational log: pong_rx=N ; link_probe log RX lines containing PONG
    pong_fields = re.findall(r"pong_rx=(\d+)", text)
    if pong_fields:
        return int(pong_fields[-1])
    return len(re.findall(r",RX,.*PONG", text)) + len(re.findall(r"type=PONG", text))


def report_verdict(evidence: dict) -> str:
    """Overall verdict (one of VERDICTS) from aggregated evidence."""
    if evidence.get("station_off"):
        return "TEST_INVALID_STATION_OFF"
    if evidence.get("link_ok"):
        return "HC12_LINK_OK"
    ports = evidence.get("detected_uart_ports") or []
    if 3 in ports:
        return "UART_SWEEP_RECEIVED_ON_SERIAL3"
    if 1 in ports:
        return "UART_SWEEP_RECEIVED_ON_SERIAL1"
    if 2 in ports:
        return "UART_SWEEP_RECEIVED_ON_SERIAL2"
    if evidence.get("openrb_rx_detected"):
        return "STATION_TO_OPENRB_RX_DETECTED"
    # A clean station write with no serial errors and no RX is NOT instability.
    if (
        evidence.get("station_tx_count", 0) > 0
        and evidence.get("serial_error_count", 0) == 0
        and evidence.get("station_total_bytes", 0) == 0
    ):
        return "STATION_TX_OK_NO_RX"
    if evidence.get("station_usb_unstable"):
        return "STATION_USB_UNSTABLE"
    if evidence.get("station_opened") and evidence.get("station_total_bytes", 0) == 0:
        return "USB_SERIAL_STABLE_NO_RF_BYTES"
    return "HC12_NO_RF_RX"


def next_action(verdict: str) -> str:
    return {
        "TEST_INVALID_STATION_OFF": (
            "Link test is INVALID until both HC-12 sides are powered and one side "
            "is transmitting. Power the station/counterpart on and re-run the same "
            "mode at the same baud."
        ),
        "USB_SERIAL_STABLE_NO_RF_BYTES": (
            "USB bridge is alive but no RF bytes arrived. Bring the counterpart "
            "side ON, confirm both HC-12s share the same baud/channel, and run the "
            "OpenRB uart sweep concurrently with a station read-only test."
        ),
        "UART_SWEEP_RECEIVED_ON_SERIAL3": (
            "HC-12 TX confirmed on Serial3. Lock the HC-12 port to Serial3 "
            "(PATH_FOLLOWING_HC12_SERIAL_PORT=3 / HC12_PROBE_SERIAL_PORT=3) and "
            "proceed to ping-pong, then the integrated dry-run."
        ),
        "UART_SWEEP_RECEIVED_ON_SERIAL1": (
            "HC-12 TX appears on Serial1, not Serial3. Re-map the HC-12 port to "
            "Serial1 in the probe/controller builds and re-validate."
        ),
        "UART_SWEEP_RECEIVED_ON_SERIAL2": (
            "Frames received on Serial2 (the GPS UART). Confirm this is not GPS "
            "NMEA; if it is the HC-12, move it off Serial2 (reserved for GPS)."
        ),
        "STATION_TX_OK_NO_RX": (
            "Station TX path is healthy (frames sent, no serial errors) but nothing "
            "came back. This is NOT a USB/code fault. Power the counterpart HC-12 "
            "side on, run the OpenRB uart sweep, and watch which SerialN_rx rises; "
            "then run ping-pong."
        ),
        "STATION_TO_OPENRB_RX_DETECTED": (
            "OpenRB received station bytes on a non-GPS UART. Confirm the same UART "
            "in both directions, then run ping-pong."
        ),
        "HC12_LINK_OK": (
            "Bidirectional link OK. Move to the integrated GPS + IMU + HC-12 "
            "dry-run (motors disabled) per firmware/README.md."
        ),
        "HC12_NO_RF_RX": (
            "Station confirmed ON but no RF bytes across the sweep: inspect the "
            "physical RF path and HC-12 settings (baud/channel/power), not "
            "code-level changes."
        ),
        "STATION_USB_UNSTABLE": (
            "USB-Serial bridge errored/disappeared. Re-seat the adapter, check the "
            "cable, and re-run the stability mode before any link test."
        ),
    }.get(verdict, "Re-run the diagnosis with both sides powered on.")


def _latest(log_dir: Path, pattern: str) -> Path | None:
    files = sorted(log_dir.glob(pattern), key=lambda p: p.stat().st_mtime)
    return files[-1] if files else None


def gather_logs(log_dir: Path) -> dict:
    logs: dict = {}
    for key, pattern in LOG_PATTERNS.items():
        path = _latest(log_dir, pattern)
        logs[key] = {
            "path": path,
            "text": path.read_text(encoding="utf-8", errors="replace") if path else "",
        }
    return logs


def build_evidence(logs: dict, *, station_off: bool) -> dict:
    sweep_text = logs["uart_sweep"]["text"]
    raw_text = logs["serial_raw"]["text"]
    link_text = logs["link_probe"]["text"]
    op_text = logs["operational"]["text"]

    sweep = parse_sweep_counters(sweep_text)
    detected = sorted(set(detect_uart_ports(raw_text)) | set(detect_uart_ports(op_text)))

    op_total = parse_total_bytes(op_text)
    raw_total = parse_total_bytes(raw_text)
    station_total = max(op_total, raw_total)
    serial_error_count = parse_int_field(op_text, "serial_error_count") or 0
    reconnect_count = parse_int_field(op_text, "reconnect_count") or 0
    station_tx_count = parse_int_field(op_text, "tx_count") or 0
    pong = parse_pong_count(op_text + "\n" + link_text)
    station_opened = bool(re.search(r"\bopened port=", op_text)) or station_total > 0 or bool(raw_text)

    # OpenRB received station bytes on a NON-GPS UART (Serial2 rx is just GPS NMEA).
    openrb_rx_detected = bool(
        (sweep.get("serial1_rx") or 0) > 0
        or (sweep.get("serial3_rx") or 0) > 0
        or any(n in (1, 3) for n in sweep.get("rx_first_detected", []))
    )

    evidence = {
        "station_off": station_off,
        "link_ok": pong > 0,
        "detected_uart_ports": detected,
        "openrb_rx_detected": openrb_rx_detected,
        "station_usb_unstable": (
            serial_error_count > 0
            and station_total == 0
            and station_tx_count == 0
            and not detected
        ),
        "station_opened": station_opened,
        "station_total_bytes": station_total,
        "station_tx_count": station_tx_count,
        "serial_error_count": serial_error_count,
        "reconnect_count": reconnect_count,
        "pong_rx": pong,
        "sweep": sweep,
    }
    evidence["verdict"] = report_verdict(evidence)
    return evidence


def render_markdown(evidence: dict, logs: dict) -> str:
    sweep = evidence["sweep"]
    verdict = evidence["verdict"]
    lines = [
        "# HC-12 Diagnosis Report",
        "",
        f"- generated_at_utc: `{dt.datetime.now(tz=dt.UTC).isoformat()}`",
        "",
        "## 1. Sources / commands run",
        "",
    ]
    for key, pattern in LOG_PATTERNS.items():
        path = logs[key]["path"]
        lines.append(f"- `{pattern}`: {('`' + str(path) + '`') if path else 'not found'}")
    op_header = re.search(r"^hc12_operational_diagnose .*$", logs["operational"]["text"], re.M)
    if op_header:
        lines += ["", f"- operational run: `{op_header.group(0)}`"]

    lines += [
        "",
        "## 2. OpenRB side evidence (uart sweep)",
        "",
        "| UART | tx | rx | rx_first_detected |",
        "|---|---|---|---|",
    ]
    for n in (1, 2, 3):
        rxfirst = "yes" if n in sweep.get("rx_first_detected", []) else "-"
        lines.append(
            f"| Serial{n} | {sweep.get(f'serial{n}_tx')} | {sweep.get(f'serial{n}_rx')} | {rxfirst} |"
        )
    lines.append("")
    lines.append("> Serial2 rx is normally GPS NMEA, not the HC-12.")

    lines += [
        "",
        "## 3. Station side evidence",
        "",
        f"- active/opened: `{evidence['station_opened']}`",
        f"- total_bytes: `{evidence['station_total_bytes']}`",
        f"- tx_count: `{evidence['station_tx_count']}`",
        f"- detected_uart_ports: `{evidence['detected_uart_ports'] or 'none'}`",
        f"- serial_error_count: `{evidence['serial_error_count']}`",
        f"- reconnect_count: `{evidence['reconnect_count']}`",
        f"- pong_rx: `{evidence['pong_rx']}`",
        "",
        "## 4. Verdict",
        "",
        f"**{verdict}**",
        "",
        "## 5. Next action",
        "",
        next_action(verdict),
        "",
    ]
    if evidence["station_off"]:
        lines += [
            "> Marked `--station-off`: with the counterpart HC-12 side off/idle, "
            "`total_bytes=0` / NO_RX is EXPECTED and does not prove a firmware or "
            "code failure.",
            "",
        ]
    return "\n".join(lines)


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Generate an HC-12 diagnosis Markdown report.")
    parser.add_argument("--log-dir", default="outputs/logs")
    parser.add_argument("--out-dir", default="outputs/reports")
    parser.add_argument(
        "--station-off",
        action="store_true",
        help="record that the counterpart HC-12 side was off -> verdict TEST_INVALID_STATION_OFF",
    )
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    log_dir = Path(args.log_dir)
    out_dir = Path(args.out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)

    logs = gather_logs(log_dir) if log_dir.exists() else {k: {"path": None, "text": ""} for k in LOG_PATTERNS}
    evidence = build_evidence(logs, station_off=args.station_off)
    report = render_markdown(evidence, logs)

    stamp = dt.datetime.now().strftime("%Y%m%d_%H%M%S")
    out_path = out_dir / f"hc12_diagnosis_{stamp}.md"
    out_path.write_text(report, encoding="utf-8")

    print(f"verdict={evidence['verdict']}")
    print(f"report={out_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
