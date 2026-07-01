"""Generate a Markdown HC-12 diagnosis report from recent diagnostic logs.

목적/역할 (Purpose):
  최근 진단 로그들을 파싱하여 HC-12 무선 UART 링크의 상태 판정(verdict)과
  다음 조치(next action)를 담은 Markdown 리포트를 생성한다. 로그만 읽는
  read-only 도구이며 시리얼 포트를 열거나 프레임을 전송하지 않는다.
  Aggregates the most recent of each diagnostic log type, derives a single
  verdict and a next-action recommendation, and writes a Markdown report.

시스템 내 위치 (Where it sits):
  진단 파이프라인의 최종 취합/보고 단계. 앞선 도구들(OpenRB uart sweep,
  tools/serial_raw_read, tools/hc12_link_probe, tools/hc12_operational_diagnose)이
  남긴 로그를 소비하며, 판정 어휘(VERDICTS)와 detect_uart_ports 규칙을
  hc12_operational_diagnose 및 serial_raw_read와 공유한다.
  Final "collector" stage of the diagnostics pipeline; consumes logs emitted
  by the upstream tools and reuses their vocabulary.

핵심 개념·불변식 (Key concepts / invariants):
  - Serial2 의 rx 는 보통 GPS NMEA 이므로 HC-12 수신 증거로 쓰지 않는다.
    (openrb_rx_detected 는 Serial1/Serial3 만 본다.)
  - verdict 문자열은 hc12_operational_diagnose.diagnose_verdict 와 어휘가
    일치해야 리포트/도구 간 해석이 어긋나지 않는다 (VERDICTS 참고).
  - --station-off 는 "상대 HC-12 가 꺼져 있었음"을 명시 -> NO_RX 는 정상,
    고장 증거가 아님. TEST_INVALID_STATION_OFF 로 강제 판정한다.

사용법/진입점 (Usage / entry point):
  CLI: python -m tools.hc12_diagnose_report [--log-dir DIR] [--out-dir DIR]
       [--station-off]. main() 이 진입점이며 outputs/reports 에 파일을 쓴다.

리팩토링 노트 (Refactoring notes):
  판정 우선순위는 report_verdict 에 집중되어 있다. 새 verdict 를 추가할 때는
  VERDICTS 튜플, report_verdict 분기, next_action 매핑을 함께 갱신하고,
  hc12_operational_diagnose 의 어휘와의 정합성을 유지할 것.

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

# 각 로그 종류별 glob 패턴. 키는 evidence/보고서에서 참조하는 논리적 이름.
# Glob per log type; keys are the logical names referenced when building evidence.
LOG_PATTERNS = {
    "uart_sweep": "uart_sweep_openrb*.log",
    "serial_raw": "serial_raw_read*.log",
    "link_probe": "hc12_link_probe*.log",
    "operational": "hc12_operational_diagnose*.log",
}

# 리포트가 낼 수 있는 판정 어휘 전체. hc12_operational_diagnose 와 공유하며,
# 여기 없는 문자열이 나오면 도구 간 해석이 어긋난 것이다.
# The full verdict vocabulary this report can emit; shared with the operational tool.
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
    """UART별 최신 tx/rx 카운터와 RX_FIRST_DETECTED 포트를 추출 / Parse sweep log.

    무엇을/왜 (What/why): OpenRB uart sweep 로그에서 Serial1..3 의 마지막 tx/rx
    값과 최초 수신이 감지된 포트 번호들을 뽑아 OpenRB 측 수신 증거로 삼는다.
    반환 (Returns): {"serial{n}_tx", "serial{n}_rx", "rx_first_detected": [ints]}.
    같은 필드가 여러 번 나오면 항상 마지막 값을 취한다 (최신 상태 반영).
    """
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
    """`field=<int>` 의 마지막 값을 정수로 반환 / Return last `field=<int>` as int, else None."""
    matches = re.findall(rf"{field}=(\d+)", text)
    return int(matches[-1]) if matches else None


def parse_total_bytes(text: str) -> int:
    """마지막 total_bytes 값(없으면 0) / Return last total_bytes value (0 if absent)."""
    vals = re.findall(r"total_bytes=(\d+)", text)
    return int(vals[-1]) if vals else 0


def parse_pong_count(text: str) -> int:
    """PONG 수신 횟수를 추정 / Estimate PONG count (link_status 판정의 근거).

    operational 로그의 pong_rx=N 을 우선 신뢰하고, 없으면 link_probe 의 RX 라인
    또는 type=PONG 출현 수를 센다. Prefer pong_rx=N; fall back to counting RX PONG lines.
    """
    # operational log: pong_rx=N ; link_probe log RX lines containing PONG
    pong_fields = re.findall(r"pong_rx=(\d+)", text)
    if pong_fields:
        return int(pong_fields[-1])
    return len(re.findall(r",RX,.*PONG", text)) + len(re.findall(r"type=PONG", text))


# ── 판정 로직 / Verdict + next-action logic ──


def report_verdict(evidence: dict) -> str:
    """취합 증거로부터 최종 판정을 결정 / Overall verdict (one of VERDICTS).

    무엇을/왜 (What/why): 여러 로그에서 모은 evidence 딕셔너리를 우선순위대로
    검사해 하나의 verdict 문자열을 고른다. 분기 순서가 곧 판정 우선순위이므로
    재배치 시 의미가 바뀐다 (station_off > link_ok > 포트별 수신 > ...).
    리팩토링 주의 (Refactor note): 새 verdict 는 VERDICTS 와 next_action 에도 추가.
    """
    # 상대측이 꺼져 있었다고 표시되면 다른 증거와 무관하게 무효 판정.
    # If the counterpart was marked off, the run is invalid regardless of evidence.
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
    # 프레임 전송 성공 + 시리얼 에러 0 + 수신 0 은 USB 불안정이 아니라 "송신 정상,
    # RF 무응답"이다. A clean station write with no serial errors and no RX is NOT instability.
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
    """판정별 다음 조치 문구를 반환 / Map a verdict to its human next-action text.

    무엇을/왜 (What/why): 각 verdict 에 대응하는 사람용 조치 안내를 돌려준다.
    미등록 verdict 는 안전한 기본 문구로 폴백. Unknown verdicts fall back to a generic hint.
    """
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


# ── 로그 수집 및 증거 구성 / Log gathering and evidence assembly ──


def _latest(log_dir: Path, pattern: str) -> Path | None:
    """패턴에 맞는 가장 최근(mtime) 파일 / Newest file (by mtime) matching pattern, or None."""
    files = sorted(log_dir.glob(pattern), key=lambda p: p.stat().st_mtime)
    return files[-1] if files else None


def gather_logs(log_dir: Path) -> dict:
    """각 로그 종류별 최신 파일을 읽어 텍스트로 반환 / Read newest of each log type.

    반환 (Returns): {key: {"path": Path|None, "text": str}}. 파일이 없으면
    text 는 빈 문자열이라 하류 파서들이 안전하게 동작한다.
    """
    logs: dict = {}
    for key, pattern in LOG_PATTERNS.items():
        path = _latest(log_dir, pattern)
        logs[key] = {
            "path": path,
            "text": path.read_text(encoding="utf-8", errors="replace") if path else "",
        }
    return logs


def build_evidence(logs: dict, *, station_off: bool) -> dict:
    """로그 텍스트들을 하나의 증거 딕셔너리로 종합 / Fuse all logs into one evidence dict.

    무엇을/왜 (What/why): 네 로그의 카운터/포트 감지 결과를 합쳐 report_verdict
    가 소비하는 표준 evidence 를 만든다. 마지막에 verdict 도 계산해 넣는다.
    인자 (Args): station_off=True 면 상대측 꺼짐으로 표시 -> 무효 판정 강제.
    부수효과 (Side effects): 없음(순수 함수). Pure; returns a new dict.
    """
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
    # 포트를 연 흔적이거나 실제 바이트/원시로그가 있으면 "열림"으로 간주.
    # Consider the station "opened" on any open marker OR any received bytes/raw log.
    station_opened = bool(re.search(r"\bopened port=", op_text)) or station_total > 0 or bool(raw_text)

    # Serial2 rx 는 GPS NMEA 이므로 제외하고, Serial1/Serial3 수신만 HC-12 증거로 본다.
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


# ── Markdown 렌더링 및 CLI 진입점 / Markdown rendering and CLI entry point ──


def render_markdown(evidence: dict, logs: dict) -> str:
    """증거/로그를 사람용 Markdown 리포트 문자열로 렌더링 / Render evidence into Markdown.

    무엇을/왜 (What/why): 소스 목록, OpenRB/스테이션 증거표, 판정, 다음 조치를
    담은 리포트 텍스트를 만든다. --station-off 였다면 NO_RX 가 정상임을 주석으로 명시.
    부수효과 (Side effects): 없음 — 문자열만 반환한다. Returns a string; no I/O.
    """
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
    """CLI 인자 파서를 구성 / Build the argparse parser for the report CLI."""
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
    """CLI 진입점: 로그 수집 -> 증거 구성 -> 리포트 파일 작성 / CLI entry point.

    부수효과 (Side effects): out-dir 를 만들고 hc12_diagnosis_<stamp>.md 를 쓰며
    verdict/report 경로를 stdout 에 출력. 반환 (Returns): 프로세스 종료 코드 0.
    """
    args = build_parser().parse_args(argv)
    log_dir = Path(args.log_dir)
    out_dir = Path(args.out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)

    # 로그 디렉터리가 없으면 빈 로그 세트로 진행 -> 하류 파서가 안전하게 동작.
    # No log dir -> empty log set so downstream parsers stay well-defined.
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
