"""Safe station-side HC-12 link probe.

목적/역할 (Purpose):
  스테이션(지상국) 쪽에서 HC-12-USB 브리지로 PING 프레임만 보내고 모든 응답을
  타임스탬프 로그로 기록하는 "안전한" 링크 검증 도구. 모터 구동 프레임을 절대
  보내지 않으므로 로봇이 움직일 위험 없이 무선 링크 생존만 확인한다.
  Station-side link validator: emits only PING frames and logs every response.

시스템 내 위치 (Where it sits):
  진단 파이프라인의 능동 송신 도구. firmware/hc12_link_probe(또는 메인 컨트롤러)가
  PONG 으로 응답하는 것을 기대한다. tools/hc12_operational_diagnose 가 이 모듈의
  ping_frame / link_status 를 재사용하고, 남긴 로그는 hc12_diagnose_report 가 파싱한다.
  Active-TX stage; its PONG replies come from firmware/hc12_link_probe.

핵심 개념·불변식 (Key concepts / invariants):
  - 안전 불변식: 오직 PING(및 종료 시 1회 CLEAR)만 전송한다. CMD,MANUAL /
    CMD,AUTO 같은 모터 프레임은 절대 보내지 않는다 (도구의 존재 이유).
  - 프레임 형식은 gps_coverage_core.protocol 의 @TYPE,SEQ,PAYLOAD*CS 를 공유.
  - LINK_OK 판정은 마지막 수신이 LINK_OK_MAX_AGE_S 이내일 때만 성립.

사용법/진입점 (Usage / entry point):
  CLI: python -m tools.hc12_link_probe --port /dev/ttyUSB0 --baud 9600
       [--ping-interval-ms N] [--duration-s S]. main() 이 진입점.

리팩토링 노트 (Refactoring notes):
  포트 열기/제어선 처리는 tools.station_serial.safe_open_serial 에 위임되어 있다.
  프레임 인코딩/디코딩을 바꾸면 상대 펌웨어와 반드시 함께 맞출 것.

Sends simple PING frames over the HC-12-USB bridge and logs every response to a
timestamped file. This tool is for link validation only:

* It only ever emits ``PING`` frames (and an optional ``CLEAR`` on exit).
* It never sends motor-driving frames (no ``CMD,MANUAL`` / ``CMD,AUTO``).

It pairs with ``firmware/hc12_link_probe`` (which answers ``PONG``) or with the
main controller. Frames use the shared ``@TYPE,SEQ,PAYLOAD*CS`` format from
``gps_coverage_core.protocol``.
"""

from __future__ import annotations

import argparse
import datetime as dt
import time
from pathlib import Path
from typing import Sequence

try:
    from tools import _bootstrap  # type: ignore  # noqa: F401
except ImportError:
    import _bootstrap  # type: ignore  # noqa: F401

from tools.station_serial import DTR_RTS_MODES, safe_open_serial

from gps_coverage_core.protocol import decode_frame, encode_frame

# 마지막 수신이 이 시간(초) 이내여야 LINK_OK. 초과하면 LINK_STALE 로 본다.
# Max age of the last RX for a "fresh" link; older than this is LINK_STALE.
LINK_OK_MAX_AGE_S = 3.0


def build_parser() -> argparse.ArgumentParser:
    """CLI 인자 파서를 구성 / Build the argparse parser for the link-probe CLI."""
    parser = argparse.ArgumentParser(
        description=(
            "Safe HC-12 link probe. Sends PING frames and logs responses. "
            "Never sends motor commands."
        )
    )
    parser.add_argument(
        "--port",
        default="/dev/ttyACM0",
        help="USB serial device for the HC-12-USB bridge (e.g. /dev/ttyUSB0 or /dev/cu.usbserial-*)",
    )
    parser.add_argument("--baud", type=int, default=9600, help="HC-12 serial baudrate")
    parser.add_argument(
        "--ping-interval-ms", type=int, default=1000, help="Milliseconds between PING frames"
    )
    parser.add_argument(
        "--duration-s",
        type=float,
        default=None,
        help="Optional run duration in seconds; runs until Ctrl-C if omitted",
    )
    parser.add_argument(
        "--log-dir", default="outputs/logs", help="Directory for the timestamped probe log"
    )
    parser.add_argument("--dtr", choices=DTR_RTS_MODES, default="low", help="DTR after open")
    parser.add_argument("--rts", choices=DTR_RTS_MODES, default="low", help="RTS after open")
    parser.add_argument("--write-timeout-s", type=float, default=1.0)
    return parser


def ping_frame(seq: int, payload: str = "PROBE") -> bytes:
    """단일 PING 프레임 생성 / Build one PING frame — the only type this tool ever sends.

    hc12_operational_diagnose 도 이 함수를 재사용하므로 시그니처 변경 시 주의.
    Reused by hc12_operational_diagnose; keep the signature stable.
    """
    return encode_frame("PING", seq, payload)


def link_status(rx_count: int, last_rx_age_s: float | None) -> str:
    """수신 활동으로 링크 상태를 요약 / Summarize link health from RX activity.

    반환 (Returns): 수신이 없으면 "NO_RX_YET"; 최근 수신이 LINK_OK_MAX_AGE_S
    이내면 "LINK_OK"; 그보다 오래되면 "LINK_STALE".
    """
    if rx_count == 0 or last_rx_age_s is None:
        return "NO_RX_YET"
    return "LINK_OK" if last_rx_age_s <= LINK_OK_MAX_AGE_S else "LINK_STALE"


def _log_line(handle, direction: str, data: bytes) -> None:
    """TX/RX 한 줄을 `ts,DIR,text` 형식으로 기록하고 flush / Append one framed log line.

    부수효과 (Side effects): 파일 핸들에 즉시 flush 하여 크래시에도 로그가 남게 한다.
    """
    stamp = dt.datetime.now(dt.UTC).isoformat()
    text = data.decode("ascii", errors="replace").rstrip()
    handle.write(f"{stamp},{direction},{text}\n")
    handle.flush()


def main(argv: Sequence[str] | None = None) -> int:
    """CLI 진입점: 포트 열기 -> 주기적 PING/응답 로깅 루프 -> 종료 시 CLEAR / Entry point.

    무엇을/왜 (What/why): 포트를 열고 ping-interval 마다 PING 을 보내며 응답을
    파싱/기록한다. Ctrl-C 또는 --duration-s 로 종료하고, 종료 직전 정리용 CLEAR
    프레임을 1회 보낸다(모터 명령 아님). 반환 (Returns): 종료 코드 0.
    부수효과 (Side effects): 시리얼 포트 열기/쓰기, 타임스탬프 로그 파일 작성.
    """
    args = build_parser().parse_args(argv)

    log_dir = Path(args.log_dir)
    log_dir.mkdir(parents=True, exist_ok=True)
    log_path = log_dir / f"hc12_link_probe_{dt.datetime.now():%Y%m%d_%H%M%S}.log"

    ping_period_s = max(args.ping_interval_ms, 1) / 1000.0
    print(
        f"Opening {args.port} @ {args.baud} for HC-12 link probe (PING only; "
        f"dtr={args.dtr} rts={args.rts} rtscts=False dsrdtr=False)."
    )
    print(f"Logging to {log_path}")

    seq = 0
    tx_count = 0
    rx_count = 0
    parse_ok = 0
    parse_err = 0
    pong_rx = 0
    last_rx_monotonic: float | None = None
    last_ping = 0.0
    start = time.monotonic()

    ser, ctl_state = safe_open_serial(
        args.port, args.baud, dtr=args.dtr, rts=args.rts, write_timeout_s=args.write_timeout_s
    )
    if ctl_state.get("warnings"):
        print("control_line_warning " + ",".join(ctl_state["warnings"]))
    with log_path.open("w", encoding="utf-8") as log_handle:
        try:
            # ── PING 송신 / 응답 수신 루프 / PING-send and response-read loop ──
            while True:
                now = time.monotonic()
                if args.duration_s is not None and now - start >= args.duration_s:
                    break

                # ping-interval 마다 새 seq 로 PING 1회 전송 / Emit one PING per interval.
                if now - last_ping >= ping_period_s:
                    seq += 1
                    frame = ping_frame(seq)
                    ser.write(frame)
                    _log_line(log_handle, "TX", frame)
                    tx_count += 1
                    last_ping = now

                raw = ser.readline()
                if raw:
                    _log_line(log_handle, "RX", raw)
                    rx_count += 1
                    last_rx_monotonic = time.monotonic()
                    try:
                        frame = decode_frame(raw)
                    except ValueError as exc:
                        parse_err += 1
                        print(f"RX parse_error: {exc}: {raw!r}")
                    else:
                        parse_ok += 1
                        if frame["type"] == "PONG":
                            pong_rx += 1
                        print(
                            f"RX type={frame['type']} seq={frame['seq']} "
                            f"payload={frame['payload']}"
                        )

                age = None if last_rx_monotonic is None else time.monotonic() - last_rx_monotonic
                age_text = "NA" if age is None else f"{age * 1000:.0f}"
                print(
                    f"tx_count={tx_count} rx_count={rx_count} pong_rx={pong_rx} "
                    f"parse_ok={parse_ok} parse_error={parse_err} "
                    f"last_rx_age_ms={age_text} link_status={link_status(rx_count, age)}",
                    end="\r",
                    flush=True,
                )
        except KeyboardInterrupt:
            print("\nStopping HC-12 link probe.")

        # 종료 시 정리용 프레임. CLEAR 는 모터 명령을 담지 않는다(안전 불변식).
        # Best-effort tidy frame; CLEAR carries no motor command.
        seq += 1
        clear_frame = encode_frame("CLEAR", seq, "PROBE")
        ser.write(clear_frame)
        _log_line(log_handle, "TX", clear_frame)

    try:
        ser.close()
    except Exception:  # noqa: BLE001
        pass
    print(f"\nDone. tx_count={tx_count} rx_count={rx_count} pong_rx={pong_rx} log={log_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
