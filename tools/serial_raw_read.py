"""OpenRB-150 UART 배선 진단용 스테이션 측 원시 시리얼 리더.
Station-side raw serial reader for OpenRB-150 UART mapping diagnostics.

목적/역할:
    USB-Serial 스테이션 어댑터(예: HC-12-USB 브리지)에서 원시 바이트를 읽어 그대로 출력한다.
    ``firmware/uart_port_sweep_probe`` 와 짝을 이룬다: HC-12 TX 가 배선된 OpenRB UART 만
    ``@UART,<n>,TX_TEST`` 프레임을 이 리더로 보내므로, 어떤 UART 포트 번호가 감지됐는지도
    함께 보고한다. 즉 "HC-12 TX 가 어느 Serial 포트에 연결됐나" 를 실측으로 확인하는 도구다.
    Reads raw bytes from the station adapter and prints them; pairs with the
    firmware ``uart_port_sweep_probe`` to reveal which OpenRB UART the HC-12 TX is
    wired to, by which ``@UART,<n>,TX_TEST`` frames arrive here.

시스템 내 위치:
    ``tools/station_serial`` 의 ``safe_open_serial`` / ``DTR_RTS_MODES`` 를 써서 포트를
    안전하게 연다. 상위 파이프라인에 속하지 않는 독립 진단 CLI 이며, 맨 위에서
    ``_bootstrap`` 을 import 해 repo 루트를 경로에 넣는다.
    Uses ``station_serial.safe_open_serial`` to open the port; a standalone
    diagnostic CLI, not part of a larger pipeline.

핵심 개념·불변식:
    - **읽기 전용(불변식)**: 포트에 절대 쓰지 않고 모터 명령도 보내지 않는다. 안전상 반드시
      유지할 것 — 리팩토링 시 write 경로를 추가하지 말 것 (함정 방지).
    - macOS 는 같은 장치를 ``/dev/cu.*`` 와 ``/dev/tty.*`` 둘 다로 노출한다. 자동 탐지는
      callout 논블로킹 노드인 ``cu.*`` 만 매칭해 한 물리 어댑터가 이중 매칭되지 않게 한다.
    - Read-only invariant: never writes, never sends motor commands. Auto-detect
      matches only ``/dev/cu.*`` (not ``tty.*``) to avoid double-matching a device.

사용법/진입점:
    ``python tools/serial_raw_read.py [--port ...] [--baud 9600] [--duration-s 10]``.
    ``--port`` 생략 시 ``/dev/cu.usbserial*`` 등에서 자동 탐지한다. ``main()`` 이 진입점.
    Entry point ``main()``; ``--port`` auto-detected from ``/dev/cu.usbserial*``.

리팩토링 노트:
    포트 glob 목록(``DEFAULT_PORT_GLOBS``)과 프레임 정규식(``UART_FRAME_RE``)이 결합 지점이다.
    프레임 포맷이 바뀌면 정규식과 펌웨어 프로브를 함께 갱신해야 한다.
    Coupling points: ``DEFAULT_PORT_GLOBS`` and ``UART_FRAME_RE``; keep the regex in
    sync with the firmware frame format.
"""

from __future__ import annotations

import argparse
import datetime as dt
import glob
import re
import sys
import time
from pathlib import Path
from typing import Iterable, Sequence

try:
    from tools import _bootstrap  # type: ignore  # noqa: F401
except ImportError:
    import _bootstrap  # type: ignore  # noqa: F401

from tools.station_serial import DTR_RTS_MODES, safe_open_serial

# macOS USB-serial bridges enumerate as /dev/cu.usbserial-*; keep a couple of
# common fallbacks for other adapter chipsets.
# macOS exposes both /dev/cu.* and /dev/tty.* for the same device; use cu.* only
# (the callout/non-blocking node) to avoid double-matching one physical adapter.
DEFAULT_PORT_GLOBS = (
    "/dev/cu.usbserial*",
    "/dev/cu.wchusbserial*",
    "/dev/cu.SLAB_USBtoUART*",
)

UART_FRAME_RE = re.compile(r"@UART,(\d+),TX_TEST")


def find_usbserial_ports(globs: Iterable[str] = DEFAULT_PORT_GLOBS) -> list[str]:
    """glob 패턴에 매칭되는 시리얼 장치 경로를 중복 제거해 안정적 순서로 반환한다.
    Return matching serial device paths, de-duplicated, in a stable order."""
    found: list[str] = []
    seen: set[str] = set()
    for pattern in globs:
        for path in sorted(glob.glob(pattern)):
            if path not in seen:
                seen.add(path)
                found.append(path)
    return found


def detect_uart_ports(text: str) -> list[int]:
    """캡처 텍스트에서 ``@UART,<n>,TX_TEST`` 로 관측된 UART 포트 번호 집합을 정렬해 반환.
    Return the sorted set of UART port numbers seen in ``@UART,<n>,TX_TEST``."""
    return sorted({int(match) for match in UART_FRAME_RE.findall(text)})


def build_parser() -> argparse.ArgumentParser:
    """CLI 인자 파서를 만든다(포트·보드레이트·읽기 시간·DTR/RTS·로그 디렉터리).
    Build the argparse parser (port, baud, duration, DTR/RTS, log dir)."""
    parser = argparse.ArgumentParser(
        description=(
            "Raw serial reader for UART-mapping diagnostics. Auto-detects a "
            "/dev/cu.usbserial* device, reads raw bytes, and reports which "
            "@UART,<n>,TX_TEST frames arrived. Read-only; never sends."
        )
    )
    parser.add_argument(
        "--port",
        default=None,
        help="USB-serial device; auto-detected from /dev/cu.usbserial* if omitted",
    )
    parser.add_argument("--baud", type=int, default=9600, help="Serial baudrate")
    parser.add_argument(
        "--duration-s", type=float, default=10.0, help="How long to read before exiting"
    )
    parser.add_argument(
        "--log-dir",
        default=None,
        help="Optional directory for a timestamped raw capture log",
    )
    parser.add_argument("--dtr", choices=DTR_RTS_MODES, default="low", help="DTR after open")
    parser.add_argument("--rts", choices=DTR_RTS_MODES, default="low", help="RTS after open")
    parser.add_argument("--write-timeout-s", type=float, default=1.0)
    return parser


def _resolve_port(requested: str | None) -> str:
    """요청 포트를 그대로 쓰거나, 없으면 자동 탐지한다. 후보가 0개면 ``SystemExit``.
    Use the requested port, else auto-detect; ``SystemExit`` if none found."""
    if requested:
        return requested
    candidates = find_usbserial_ports()
    if not candidates:
        raise SystemExit(
            "No /dev/cu.usbserial* device found. Plug in the HC-12-USB bridge, or "
            "pass --port explicitly (list with: ls /dev/cu.*)."
        )
    if len(candidates) > 1:
        print(f"Multiple serial devices found {candidates}; using {candidates[0]}.")
    return candidates[0]


def main(argv: Sequence[str] | None = None) -> int:
    """포트를 열어 지정 시간 동안 원시 바이트를 읽어 출력/로깅하고, 감지된 UART 를 요약한다.
    부수효과: 시리얼 포트 오픈, stdout 출력, 선택적 로그 파일 쓰기. 항상 0을 반환.
    Open the port, stream/log raw bytes for the duration, summarize detected
    UARTs. Side effects: opens serial, writes stdout and an optional log; returns 0."""
    args = build_parser().parse_args(argv)

    port = _resolve_port(args.port)
    print(
        f"Reading {port} @ {args.baud} for {args.duration_s:.1f}s (raw, read-only; "
        f"dtr={args.dtr} rts={args.rts} rtscts=False dsrdtr=False)."
    )

    log_handle = None
    if args.log_dir:
        log_dir = Path(args.log_dir)
        log_dir.mkdir(parents=True, exist_ok=True)
        log_path = log_dir / f"serial_raw_read_{dt.datetime.now():%Y%m%d_%H%M%S}.log"
        log_handle = log_path.open("w", encoding="utf-8")
        print(f"Logging raw capture to {log_path}")

    total_bytes = 0
    captured = ""
    start = time.monotonic()
    ser, ctl_state = safe_open_serial(
        port, args.baud, dtr=args.dtr, rts=args.rts, write_timeout_s=args.write_timeout_s
    )
    if ctl_state.get("warnings"):
        print("control_line_warning " + ",".join(ctl_state["warnings"]))
    try:
        while time.monotonic() - start < args.duration_s:
            chunk = ser.read(256)
            if not chunk:
                continue  # read timeout(빈 청크)일 뿐 — 배선/신호 문제 아님 / just a read timeout
            total_bytes += len(chunk)
            text = chunk.decode("ascii", errors="replace")
            captured += text
            sys.stdout.write(text)
            sys.stdout.flush()
            if log_handle is not None:
                log_handle.write(text)
                log_handle.flush()
    except KeyboardInterrupt:
        print("\nInterrupted.")
    finally:
        # close 실패해도 진단 결과(요약)는 내야 하므로 광범위 예외를 삼킨다 / never mask summary
        try:
            ser.close()
        except Exception:  # noqa: BLE001
            pass
        if log_handle is not None:
            log_handle.close()

    detected = detect_uart_ports(captured)
    print("\n--- summary ---")
    print(f"total_bytes={total_bytes}")
    print(f"detected_uart_ports={detected if detected else 'none'}")
    if detected:
        names = ", ".join(f"Serial{n}" for n in detected)
        print(f"verdict: HC-12 TX appears wired to {names} (frames received here).")
    elif total_bytes > 0:
        print("verdict: bytes received but no @UART frame; check baud or framing.")
    else:
        print("verdict: 0 bytes. HC-12 is not on a swept UART, or wiring/baud is off.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
