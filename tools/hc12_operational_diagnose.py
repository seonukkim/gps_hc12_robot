"""Operational HC-12 station-side diagnosis for the current fixed hardware setup.

목적/역할 (Purpose):
  현장에서 반복 실행 가능한 스테이션(지상국) 진단 도구. HC-12 를 뽑거나 루프백/
  AT 모드로 바꾸지 않고, 고정된 하드웨어 그대로 USB-Serial 브리지를 열어 네 가지
  모드 중 하나를 수행한 뒤 구조화된 상태와 단일 verdict 를 출력한다. 진단용 텍스트
  프레임의 송수신만 하며 모터 명령은 절대 보내지 않는다.
  Field-repeatable station diagnosis; no unplugging/loopback/AT mode required.

시스템 내 위치 (Where it sits):
  진단 파이프라인의 스테이션 측 주력 도구. ping_frame/link_status 는
  tools.hc12_link_probe, detect_uart_ports 는 tools.serial_raw_read, 포트 열기/
  제어선은 tools.station_serial 에서 재사용한다. 남긴 로그는 hc12_diagnose_report
  가 파싱하며, 여기의 verdict 어휘가 그 리포트와 반드시 일치해야 한다.
  Reuses helpers from link_probe / serial_raw_read / station_serial; feeds the report.

핵심 개념·불변식 (Key concepts / invariants):
  - verdict 문자열은 hc12_diagnose_report 의 VERDICTS 와 어휘가 일치해야 한다.
  - 상대측 HC-12 가 꺼져 있거나 송신하지 않으면 NO_RX/total_bytes=0 은 정상이며
    고장 증거가 아니다. 명시적 무효 기록을 원하면 --station-off 를 쓴다.
  - "깨끗한 송신(에러 0)이지만 무응답"은 USB 불안정이 아니라 RF 무응답으로 본다
    (diagnose_verdict 의 STATION_TX_OK_NO_RX / HC12_NO_RF_RX 분기).
  - OpenRB(usbmodem) 포트는 스테이션 브리지가 아니므로 자동 선택에서 제외한다.

사용법/진입점 (Usage / entry point):
  CLI: python -m tools.hc12_operational_diagnose --mode {read-only|write-only|
       ping-pong|stability} [--port auto|DEV] [--baud N] [--duration-s S]
       [--reconnect] [--station-off]. main() 이 진입점.

리팩토링 노트 (Refactoring notes):
  판정 로직은 diagnose_verdict 한 곳에 모여 있다. 새 모드/verdict 추가 시 MODES,
  diagnose_verdict, 그리고 리포트 측 어휘를 함께 갱신할 것. 에러 코드 분류는
  classify_serial_error 에 집중되어 pyserial 없이도 동작하도록 설계되었다.

This is the field-repeatable station tool. It does NOT require unplugging the
HC-12, a direct loopback, or AT mode. It opens the station USB-Serial bridge and
exercises one of four modes, then prints a concise structured status with a
verdict. It is read/write of diagnostic text frames only: no motor commands.

Modes:
  read-only   read raw bytes, print repr(), detect @UART,<n> sweep frames
  write-only  send @STATION,<seq>,TX_TEST repeatedly (check OpenRB sweep RX)
  ping-pong   send PING, read PONG (compatible with tools/hc12_link_probe.py)
  stability   open + poll in_waiting, never send PING, prove the USB bridge alive

Interpretation note: with the counterpart HC-12 side powered off or its firmware
not transmitting, NO_RX / total_bytes=0 is EXPECTED and does not prove a code or
firmware failure. Mark it with --station-off to record an explicitly invalid run.
"""

from __future__ import annotations

import argparse
import datetime as dt
import glob
import time
from pathlib import Path
from typing import Sequence

try:
    from tools import _bootstrap  # type: ignore  # noqa: F401
except ImportError:
    import _bootstrap  # type: ignore  # noqa: F401

from gps_coverage_core.protocol import decode_frame, encode_frame
from tools.hc12_link_probe import link_status, ping_frame
from tools.serial_raw_read import detect_uart_ports
from tools.station_serial import DTR_RTS_MODES, control_line_state, safe_open_serial

# 스테이션 USB-시리얼 어댑터만 대상으로 한다. /dev/cu.usbmodem* 은 OpenRB 자신이라
# 실수로 OpenRB USB-CDC 포트를 읽지 않도록 자동 탐색에서 명시적으로 제외한다.
# Station USB-serial adapters only. /dev/cu.usbmodem* is the OpenRB itself and is
# explicitly excluded so we never read the OpenRB USB-CDC port by mistake.
STATION_PORT_GLOBS = (
    "/dev/cu.usbserial*",
    "/dev/cu.wchusbserial*",
    "/dev/cu.SLAB_USBtoUART*",
)
# 지원하는 진단 모드(모듈 docstring 참조) / Supported diagnosis modes (see module docstring).
MODES = ("read-only", "write-only", "ping-pong", "stability")


def filter_station_ports(paths: Sequence[str]) -> list[str]:
    """OpenRB(usbmodem) 포트를 제거하고 순서 유지 중복 제거 / Drop OpenRB ports, de-dup.

    무엇을/왜 (What/why): 자동 포트 선택이 OpenRB 자신의 USB-CDD 포트를 잡지 않도록
    usbmodem 을 걸러낸다. 입력 순서를 보존해 첫 후보가 안정적으로 선택되게 한다.
    """
    out: list[str] = []
    seen: set[str] = set()
    for path in paths:
        if "usbmodem" in path:  # OpenRB USB-CDC 이지 스테이션 브리지가 아님 / OpenRB, not the bridge
            continue
        if path not in seen:
            seen.add(path)
            out.append(path)
    return out


def find_station_ports(globs: Sequence[str] = STATION_PORT_GLOBS) -> list[str]:
    """glob 패턴들로 스테이션 포트 후보를 수집 / Collect station port candidates via globs.

    반환 (Returns): usbmodem 을 제외한 정렬·중복제거된 경로 목록.
    """
    paths: list[str] = []
    for pattern in globs:
        paths.extend(sorted(glob.glob(pattern)))
    return filter_station_ports(paths)


def classify_serial_error(exc: BaseException) -> str:
    """시리얼/OS 예외를 안정적인 짧은 코드로 분류 / Classify an error into a stable code.

    무엇을/왜 (What/why): errno 와 메시지 문자열을 함께 보고 pyserial 의존 없이도
    DEVICE_NOT_CONFIGURED / PORT_NOT_FOUND / IO_ERROR 등으로 매핑한다. 로그와 verdict
    에서 원인을 일관되게 표기하기 위한 것. 미상 예외는 UNKNOWN_ERROR 로 폴백.
    """
    errno = getattr(exc, "errno", None)
    msg = str(exc)
    if errno == 6 or "Errno 6" in msg or "Device not configured" in msg:
        return "DEVICE_NOT_CONFIGURED"
    if errno == 2 or "Errno 2" in msg or "No such file or directory" in msg:
        return "PORT_NOT_FOUND"
    if errno == 5 or "Errno 5" in msg or "Input/output error" in msg:
        return "IO_ERROR"
    if type(exc).__name__ == "SerialException":
        return "SERIAL_EXCEPTION"
    if isinstance(exc, OSError):
        return "OS_ERROR"
    return "UNKNOWN_ERROR"


# ── 단일 실행 판정 로직 / Single-run verdict logic ──


def diagnose_verdict(
    *,
    mode: str,
    opened: bool,
    serial_error_count: int,
    total_bytes: int,
    detected_uart_ports: Sequence[int],
    pong_rx: int,
    tx_count: int = 0,
    rx_count: int = 0,
    station_off: bool = False,
) -> str:
    """관측값으로부터 이번 실행의 verdict 를 결정 / Single-run verdict from observations.

    무엇을/왜 (What/why): 한 번의 진단 실행에서 모은 관측치(열림 여부, 에러 수,
    수신 바이트, 감지된 UART, pong 수 등)를 우선순위대로 검사해 하나의 문자열을
    돌려준다. 분기 순서가 곧 판정 우선순위이다. 반환 문자열은 hc12_diagnose_report
    의 어휘와 반드시 일치해야 한다(리팩토링 시 함께 갱신).
    핵심 인자 (Key args): mode 는 모드별 특수 판정을, station_off 는 무효 강제를 좌우.
    """
    # 상대측 꺼짐 표시면 무조건 무효. If the counterpart was off, the run is invalid.
    if station_off:
        return "TEST_INVALID_STATION_OFF"
    # 포트를 한 번도 못 열었으면 USB 자체가 불안정. Never opened -> USB bridge unstable.
    if not opened:
        return "STATION_USB_UNSTABLE"
    if mode == "ping-pong" and pong_rx > 0:
        return "HC12_LINK_OK"
    if detected_uart_ports:
        if 3 in detected_uart_ports:
            return "UART_SWEEP_RECEIVED_ON_SERIAL3"
        if 1 in detected_uart_ports:
            return "UART_SWEEP_RECEIVED_ON_SERIAL1"
        if 2 in detected_uart_ports:
            return "UART_SWEEP_RECEIVED_ON_SERIAL2"
    no_rx = total_bytes == 0 and rx_count == 0
    # 프레임을 보냈고 에러가 없다면 USB 불안정이 아니라 "송신 정상, RF 무응답"이다.
    # A clean write (frames sent, no serial errors) is NOT an unstable USB bridge.
    # It means the station TX path is fine but nothing came back over RF.
    if tx_count > 0 and serial_error_count == 0 and no_rx:
        return "STATION_TX_OK_NO_RX" if mode == "write-only" else "HC12_NO_RF_RX"
    # 아무것도 동작 안 했고 에러까지 났을 때만 진짜 불안정으로 판정.
    # Genuinely unstable only when nothing ever worked AND errors occurred.
    if serial_error_count > 0 and tx_count == 0 and total_bytes == 0 and not detected_uart_ports:
        return "STATION_USB_UNSTABLE"
    if mode == "write-only":
        return "STATION_WRITE_ONLY_SENT_CHECK_OPENRB_RX"
    if total_bytes > 0:
        return "RF_BYTES_RECEIVED_UNRECOGNIZED"
    if mode in ("stability", "read-only"):
        return "USB_SERIAL_STABLE_NO_RF_BYTES"
    return "HC12_NO_RF_RX"


def build_parser() -> argparse.ArgumentParser:
    """CLI 인자 파서를 구성 / Build the argparse parser for the operational diagnosis CLI."""
    parser = argparse.ArgumentParser(
        description="Operational HC-12 station diagnosis (no unplugging required)."
    )
    parser.add_argument("--port", default="auto", help="'auto' or an explicit device path")
    parser.add_argument("--baud", type=int, default=9600)
    parser.add_argument("--duration-s", type=float, default=60.0)
    parser.add_argument("--mode", choices=MODES, default="read-only")
    parser.add_argument("--log-dir", default="outputs/logs")
    parser.add_argument("--raw-log", action="store_true", help="also dump raw received bytes")
    parser.add_argument("--reconnect", action="store_true", help="reopen the port after errors")
    parser.add_argument("--reconnect-delay-s", type=float, default=1.0)
    parser.add_argument(
        "--station-off",
        action="store_true",
        help="mark the counterpart HC-12 side as off/idle -> run is recorded as invalid",
    )
    parser.add_argument("--status-period-s", type=float, default=5.0)
    parser.add_argument(
        "--dtr",
        choices=DTR_RTS_MODES,
        default="low",
        help="DTR line after open: low (default, matches the known-good manual test)",
    )
    parser.add_argument(
        "--rts",
        choices=DTR_RTS_MODES,
        default="low",
        help="RTS line after open: low (default, matches the known-good manual test)",
    )
    parser.add_argument(
        "--write-timeout-s", type=float, default=1.0, help="serial write timeout in seconds"
    )
    return parser


# ── CLI 진입점 및 진단 실행 루프 / CLI entry point and diagnosis run-loop ──


def main(argv: Sequence[str] | None = None) -> int:
    """CLI 진입점: 포트 열기 -> 모드별 루프 -> 상태/요약/verdict 출력 / Entry point.

    무엇을/왜 (What/why): 선택된 모드로 duration 동안 포트를 열고 (필요시 재접속)
    송수신하며 주기적으로 status 를 남긴다. 종료 시 감지 포트를 종합해 diagnose_verdict
    로 최종 판정을 계산하고 로그에 기록한다. 반환 (Returns): 종료 코드 0.
    부수효과 (Side effects): 시리얼 포트 I/O, 로그 파일(및 --raw-log 시 raw .bin) 작성.
    """
    args = build_parser().parse_args(argv)
    ctl = control_line_state(args.dtr, args.rts)

    log_dir = Path(args.log_dir)
    log_dir.mkdir(parents=True, exist_ok=True)
    stamp = dt.datetime.now().strftime("%Y%m%d_%H%M%S")
    log_path = log_dir / f"hc12_operational_diagnose_{stamp}.log"
    raw_path = log_dir / f"hc12_operational_diagnose_{stamp}_raw.bin" if args.raw_log else None
    log_handle = log_path.open("w", encoding="utf-8")
    raw_handle = raw_path.open("wb") if raw_path is not None else None

    def emit(line: str) -> None:
        """한 줄을 stdout 과 로그 파일에 동시에 출력·flush / Print+log one line, flushed."""
        print(line)
        log_handle.write(line + "\n")
        log_handle.flush()

    emit(
        f"hc12_operational_diagnose mode={args.mode} baud={args.baud} "
        f"duration_s={args.duration_s} reconnect={args.reconnect} station_off={args.station_off} "
        f"dtr_mode={ctl['dtr_mode']} rts_mode={ctl['rts_mode']} rtscts={ctl['rtscts']} "
        f"dsrdtr={ctl['dsrdtr']} write_timeout_s={args.write_timeout_s}"
    )

    # 실행 중 변하는 상태들(카운터/포트/버퍼). Mutable run state (counters/port/buffers).
    active_port = None
    opened = False
    opened_once = False
    serial_error_count = 0
    reconnect_count = 0
    last_error = "none"
    total_bytes = 0
    tx_count = 0
    rx_count = 0
    pong_rx = 0
    parse_ok = 0
    parse_error = 0
    stability_alive_ticks = 0
    captured = ""
    rx_line = b""
    last_rx_monotonic: float | None = None
    seq = 0
    last_send = 0.0
    last_status = 0.0
    send_period = 1.0

    def status_fields() -> str:
        """현재 카운터들을 한 줄 key=value 문자열로 직렬화 / Serialize counters to a status line.

        이 형식은 hc12_diagnose_report 의 파서가 읽으므로 필드명 변경 시 함께 갱신.
        This layout is parsed by hc12_diagnose_report; keep field names in sync.
        """
        ports = detect_uart_ports(captured)
        return (
            f"active_port={active_port} total_bytes={total_bytes} tx_count={tx_count} "
            f"rx_count={rx_count} pong_rx={pong_rx} parse_ok={parse_ok} "
            f"parse_error={parse_error} serial_error_count={serial_error_count} "
            f"reconnect_count={reconnect_count} stability_alive_ticks={stability_alive_ticks} "
            f"detected_uart_ports={ports if ports else 'none'} last_error={last_error} "
            f"dtr_mode={ctl['dtr_mode']} rts_mode={ctl['rts_mode']} "
            f"rtscts={ctl['rtscts']} dsrdtr={ctl['dsrdtr']}"
        )

    ser = None
    start = time.monotonic()
    deadline = start + args.duration_s
    try:
        # ── 메인 진단 루프 (deadline 까지) / Main diagnosis loop (until deadline) ──
        while time.monotonic() < deadline:
            # 필요 시 포트 (재)열기. opened_once 여부로 최초/재접속 로그를 구분.
            # (Re)open the port as needed; opened_once distinguishes first open vs reconnect.
            if ser is None:
                try:
                    port = args.port
                    if port == "auto":
                        candidates = find_station_ports()
                        if not candidates:
                            raise OSError(2, "No such file or directory (no station port found)")
                        port = candidates[0]
                    ser, ctl_state = safe_open_serial(
                        port,
                        args.baud,
                        dtr=args.dtr,
                        rts=args.rts,
                        write_timeout_s=args.write_timeout_s,
                    )
                    active_port = port
                    if ctl_state.get("warnings"):
                        emit("control_line_warning " + ",".join(ctl_state["warnings"]))
                    if opened_once:
                        reconnect_count += 1
                        emit(f"reconnect ok port={port} reconnect_count={reconnect_count}")
                    else:
                        emit(
                            f"opened port={port} baud={args.baud} dtr_mode={args.dtr} "
                            f"rts_mode={args.rts} rtscts=False dsrdtr=False"
                        )
                    opened = True
                    opened_once = True
                except Exception as exc:  # noqa: BLE001 - field robustness
                    serial_error_count += 1
                    last_error = classify_serial_error(exc)
                    emit(f"open_error {last_error} serial_error_count={serial_error_count}")
                    if args.reconnect:
                        time.sleep(args.reconnect_delay_s)
                        continue
                    break

            # ── 선택된 모드의 1회 작업 / One iteration of work for the selected mode ──
            try:
                now = time.monotonic()
                if args.mode in ("write-only", "ping-pong") and now - last_send >= send_period:
                    seq += 1
                    if args.mode == "write-only":
                        ser.write(encode_frame("STATION", seq, "TX_TEST"))
                    else:
                        ser.write(ping_frame(seq))
                    tx_count += 1
                    last_send = now

                if args.mode == "stability":
                    # in_waiting 접근 자체가 브리지 생존 확인. 미구성 시 OSError 6 유발.
                    waiting = ser.in_waiting  # may raise OSError 6 if unconfigured
                    stability_alive_ticks += 1
                    if waiting:
                        chunk = ser.read(waiting)
                        total_bytes += len(chunk)
                        if raw_handle is not None:
                            raw_handle.write(chunk)
                    time.sleep(0.5)
                elif args.mode == "write-only":
                    time.sleep(0.05)  # 송신만 하고 읽지 않음(OpenRB RX 로 확인) / send only, do not read
                else:  # read-only, ping-pong
                    chunk = ser.read(256)
                    if chunk:
                        total_bytes += len(chunk)
                        rx_count += len(chunk)
                        last_rx_monotonic = time.monotonic()
                        if raw_handle is not None:
                            raw_handle.write(chunk)
                        text = chunk.decode("ascii", errors="replace")
                        captured += text
                        if args.mode == "read-only":
                            emit(f"rx repr={chunk!r}")
                        # 개행 단위로 완결된 프레임만 파싱해 PONG 집계(부분 프레임은 버퍼에 유지).
                        # Parse complete (newline-terminated) frames for ping-pong accounting.
                        rx_line += chunk
                        while b"\n" in rx_line:
                            raw_line, rx_line = rx_line.split(b"\n", 1)
                            line = raw_line.strip()
                            if not line:
                                continue
                            try:
                                frame = decode_frame(line)
                            except ValueError:
                                parse_error += 1
                            else:
                                parse_ok += 1
                                if frame["type"] == "PONG":
                                    pong_rx += 1
            except Exception as exc:  # noqa: BLE001 - 현장 견고성 / field robustness
                # I/O 중 예외는 포트를 닫고 재접속 옵션이면 재시도, 아니면 루프 종료.
                # On I/O error: close, then retry if --reconnect else stop the loop.
                serial_error_count += 1
                last_error = classify_serial_error(exc)
                emit(f"io_error {last_error} serial_error_count={serial_error_count}")
                try:
                    if ser is not None:
                        ser.close()
                except Exception:  # noqa: BLE001
                    pass
                ser = None
                opened = False
                if args.reconnect:
                    time.sleep(args.reconnect_delay_s)
                    continue
                break

            if now - last_status >= args.status_period_s:
                emit("status " + status_fields())
                last_status = now
    except KeyboardInterrupt:
        emit("interrupted")
    finally:
        try:
            if ser is not None:
                ser.close()
        except Exception:  # noqa: BLE001
            pass
        if raw_handle is not None:
            raw_handle.close()

    # ── 종료 후 최종 판정/요약 / Post-run verdict and summary ──
    detected = detect_uart_ports(captured)
    verdict = diagnose_verdict(
        mode=args.mode,
        opened=opened_once,
        serial_error_count=serial_error_count,
        total_bytes=total_bytes,
        detected_uart_ports=detected,
        pong_rx=pong_rx,
        tx_count=tx_count,
        rx_count=rx_count,
        station_off=args.station_off,
    )
    link = link_status(rx_count, None if last_rx_monotonic is None else time.monotonic() - last_rx_monotonic)
    emit("summary " + status_fields())
    emit(f"link_status={link}")
    emit(f"verdict={verdict}")
    log_handle.close()
    print(f"log={log_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
