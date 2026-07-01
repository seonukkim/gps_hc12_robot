"""위경도 기준 A/B 지점 캡처 도구 / Capture georeferenced A/B points (lat/lon).

목적/역할:
    Stage 13 무동작(no-motion) 경로 추종을 위해 필드 기준점 A·B를 위경도로 캡처한다.
    세 가지 입력 방식을 지원한다: 수동 입력(manual_latlon), 로그 재생(replay_log),
    실시간 USBDBG 시리얼(live_usbdbg). 결과는 `field_points_georef.json`과 `.csv`로 저장하며,
    로그·실시간 모드에서는 여러 샘플의 위경도를 평균 낸다.

    Captures field reference points A and B as latitude/longitude for Stage 13 no-motion
    path tracking, via three modes: manual entry, log replay, or a live USBDBG serial
    stream. Writes `field_points_georef.json`/`.csv`, averaging multiple samples in the
    log and live modes.

시스템 내 위치:
    `tools/` CLI 진입점 스크립트. 로컬 미터 좌표만 필요할 때는 짝 스크립트
    `tools/capture_field_ab_points.py`를 쓴다. 여기서 얻은 위경도 A/B는 이후 경로 패키지의
    georeference 메타데이터로 이어지며, `tools/check_georef_path_package.py`가 그 변환을 검증한다.

    Entry-point script in `tools/`; the sibling `tools/capture_field_ab_points.py` handles
    local-meter capture. The lat/lon A/B produced here feed the path package's
    georeference metadata, which `tools/check_georef_path_package.py` later validates.

핵심 개념·불변식:
    - live_usbdbg 모드는 pyserial이 있어야 하며, 없으면 RuntimeError를 던진다(선택적 의존성).
    - 로그/시리얼 텍스트는 `key=value` 토큰으로 파싱하고, current_lat/current_lon을 우선,
      없으면 lat/lon을 쓴다. 유효 샘플이 하나도 없으면 ValueError("NO_LAT_LON_SAMPLES").
    - 출력에는 안전 플래그 `motor_command_generated=False`, `physical_output_active=False`가
      항상 포함된다 — 캡처만 할 뿐 물리 출력을 만들지 않는다는 계약.
    - row-range는 "N"(단일) 또는 "start:end"(반열림 구간, end 미포함) 형식이다.

    - live_usbdbg needs pyserial (optional dependency; raises RuntimeError if missing).
    - Text is parsed into key=value tokens; current_lat/current_lon preferred over lat/lon;
      raises ValueError("NO_LAT_LON_SAMPLES") when no usable sample is found.
    - Output always carries `motor_command_generated=False` and
      `physical_output_active=False`. Row-range is "N" or half-open "start:end".

사용법/진입점:
    CLI 진입점은 main(). --mode는 필수다. 예:
      manual: `--mode manual_latlon --a-lat .. --a-lon .. --b-lat .. --b-lon ..`
      replay: `--mode replay_log --log F --a-row-range 0:10 --b-row-range 10:20`
      live:   `--mode live_usbdbg --port /dev/ttyUSB0 --sample-count 10`

    CLI entry point is main(); --mode is required (manual_latlon / replay_log / live_usbdbg).

리팩토링 노트:
    CSV 컬럼 순서는 FIELDS로 고정. 파싱 정규식(`key=value`)과 좌표 키 우선순위는 로그 포맷과
    결합되어 있으니 함께 바꿀 것. 안전 플래그를 제거·변경하지 말 것.

    CSV column order is pinned by FIELDS; the key=value regex and coordinate-key preference
    are coupled to the log format. Keep the safety flags intact when refactoring.
"""

from __future__ import annotations

import argparse
import csv
import json
import re
import time
from pathlib import Path
from typing import Sequence

try:
    from tools import _bootstrap  # type: ignore  # noqa: F401
except ImportError:
    import _bootstrap  # type: ignore  # noqa: F401


FIELDS = ("point_label", "lat", "lon", "sample_count")


def _parse_rows(text: str) -> list[dict[str, str]]:
    """텍스트를 줄 단위로 `key=value` 토큰 dict 리스트로 파싱한다(키는 소문자).

    Parse text line-by-line into a list of key=value token dicts (keys lowercased).
    토큰이 없는 줄은 건너뛴다 / lines with no tokens are skipped.
    """
    rows: list[dict[str, str]] = []
    for line in text.splitlines():
        values = {key.lower(): value for key, value in re.findall(r"([A-Za-z0-9_]+)=([^,\s]+)", line)}
        if values:
            rows.append(values)
    return rows


def _average_lat_lon(rows: Sequence[dict[str, str]]) -> tuple[float, float, int]:
    """여러 파싱 행에서 위경도를 뽑아 평균과 사용 샘플 수 (lat, lon, count)를 반환한다.

    Average lat/lon over parsed rows; return (mean_lat, mean_lon, sample_count).
    current_lat/lon 우선, 없으면 lat/lon 사용 / prefers current_lat/lon, falls back to lat/lon.
    유효 샘플이 없으면 ValueError("NO_LAT_LON_SAMPLES") / raises if no usable sample.
    """
    coords: list[tuple[float, float]] = []
    for row in rows:
        # 로그는 current_lat/lon을 쓰고 수동 CSV는 lat/lon을 쓰므로 둘 다 허용
        # logs emit current_lat/lon while manual CSV uses lat/lon — accept either
        lat = row.get("current_lat", row.get("lat"))
        lon = row.get("current_lon", row.get("lon"))
        if lat is None or lon is None:
            continue
        coords.append((float(lat), float(lon)))
    if not coords:
        raise ValueError("NO_LAT_LON_SAMPLES")
    return (
        sum(lat for lat, _ in coords) / len(coords),
        sum(lon for _, lon in coords) / len(coords),
        len(coords),
    )


def _parse_range(text: str) -> tuple[int, int]:
    """행 범위 문자열을 반열림 슬라이스 (start, end)로 변환한다.

    Parse a row-range string into a half-open (start, end) slice.
    "N"은 (N, N+1) 단일 행, "a:b"는 (a, b) / "N" -> single row, "a:b" -> (a, b).
    """
    if ":" not in text:
        index = int(text)
        return index, index + 1
    start, end = text.split(":", 1)
    return int(start), int(end)


def _write_outputs(out_dir: Path, *, a: tuple[float, float, int], b: tuple[float, float, int], mode: str) -> dict[str, Path]:
    """A/B 위경도 결과를 field_points_georef.json/csv로 기록하고 경로 dict를 반환한다.

    Write A/B lat/lon results to field_points_georef.json and .csv; return {"json","csv"}.
    부수효과: out_dir 생성 및 두 파일 기록, 안전 플래그 포함 / creates dir, writes files+flags.
    """
    out_dir.mkdir(parents=True, exist_ok=True)
    data = {
        "capture_mode": mode,
        "points": {
            "A": {"lat": a[0], "lon": a[1], "sample_count": a[2]},
            "B": {"lat": b[0], "lon": b[1], "sample_count": b[2]},
        },
        "georeference_available": True,
        "motor_command_generated": False,
        "physical_output_active": False,
    }
    json_path = out_dir / "field_points_georef.json"
    csv_path = out_dir / "field_points_georef.csv"
    json_path.write_text(json.dumps(data, indent=2) + "\n", encoding="utf-8")
    with csv_path.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=FIELDS)
        writer.writeheader()
        writer.writerow({"point_label": "A", "lat": a[0], "lon": a[1], "sample_count": a[2]})
        writer.writerow({"point_label": "B", "lat": b[0], "lon": b[1], "sample_count": b[2]})
    return {"json": json_path, "csv": csv_path}


def _capture_live(port: str, sample_count: int) -> tuple[tuple[float, float, int], tuple[float, float, int]]:
    """실시간 USBDBG 시리얼에서 A·B 지점을 각각 평균 캡처한다(읽기 전용).

    Capture A and B (each averaged) from a live USBDBG serial stream; read-only.
    pyserial 미설치 시 RuntimeError / raises RuntimeError if pyserial is unavailable.
    부수효과: 시리얼 포트 열기(읽기만), Enter 대기 프롬프트 / opens serial read-only, prompts.
    """
    try:
        import serial  # type: ignore[import-not-found]
    except ImportError as exc:
        # 선택적 의존성: 실시간 모드에서만 필요 / optional dep: only needed for live mode
        raise RuntimeError("pyserial is not available; cannot read live USBDBG") from exc

    def collect(handle, label: str) -> tuple[float, float, int]:
        """한 라벨(A/B)에 대해 목표 샘플 수만큼 읽어 평균을 낸다. deadline까지만 대기.

        Read up to sample_count rows for one label and average; bounded by a deadline.
        """
        input(f"Press Enter to capture {label} average...")
        rows: list[dict[str, str]] = []
        # 무한 대기 방지: 최소 30초, 샘플당 2초를 상한으로 둔다 / cap wait to avoid hanging
        deadline = time.monotonic() + max(30.0, sample_count * 2.0)
        while len(rows) < sample_count and time.monotonic() < deadline:
            raw = handle.readline()
            if not raw:
                continue
            parsed = _parse_rows(raw.decode("utf-8", errors="replace"))
            if parsed:
                rows.extend(parsed)
        return _average_lat_lon(rows)

    with serial.Serial(port, baudrate=115200, timeout=1.0) as handle:
        return collect(handle, "A"), collect(handle, "B")


def build_parser() -> argparse.ArgumentParser:
    """이 도구의 argparse 파서를 구성한다 / build the argparse parser for this tool."""
    parser = argparse.ArgumentParser(description="Capture georeferenced A/B points for Stage 13 no-motion path tracking.")
    parser.add_argument("--mode", choices=("manual_latlon", "replay_log", "live_usbdbg"), required=True)
    parser.add_argument("--a-lat", type=float)
    parser.add_argument("--a-lon", type=float)
    parser.add_argument("--b-lat", type=float)
    parser.add_argument("--b-lon", type=float)
    parser.add_argument("--log")
    parser.add_argument("--a-row-range")
    parser.add_argument("--b-row-range")
    parser.add_argument("--port")
    parser.add_argument("--sample-count", type=int, default=10)
    parser.add_argument("--out-dir", default="outputs/field_georef_capture/latest")
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    """CLI 진입점: 모드별로 A/B 위경도를 확보→파일 기록→요약 출력. 항상 0 반환.

    CLI entry point: obtain A/B lat/lon per mode, write files, print summary; returns 0.
    잘못된 인자 조합은 SystemExit로 종료 / raises SystemExit on invalid arg combinations.
    """
    args = build_parser().parse_args(argv)
    if args.mode == "manual_latlon":
        missing = [
            name
            for name in ("a_lat", "a_lon", "b_lat", "b_lon")
            if getattr(args, name) is None
        ]
        if missing:
            raise SystemExit("--a-lat/--a-lon/--b-lat/--b-lon are required in manual_latlon mode")
        a = (float(args.a_lat), float(args.a_lon), 1)
        b = (float(args.b_lat), float(args.b_lon), 1)
    elif args.mode == "replay_log":
        if not args.log or not args.a_row_range or not args.b_row_range:
            raise SystemExit("--log, --a-row-range, and --b-row-range are required in replay_log mode")
        rows = _parse_rows(Path(args.log).read_text(encoding="utf-8", errors="replace"))
        a_start, a_end = _parse_range(args.a_row_range)
        b_start, b_end = _parse_range(args.b_row_range)
        a = _average_lat_lon(rows[a_start:a_end])
        b = _average_lat_lon(rows[b_start:b_end])
    else:
        if not args.port:
            raise SystemExit("--port is required in live_usbdbg mode")
        a, b = _capture_live(args.port, args.sample_count)
    outputs = _write_outputs(Path(args.out_dir), a=a, b=b, mode=args.mode)
    print("Georeferenced A/B points captured.")
    print(f"field_points_georef_json={outputs['json']}")
    print(f"field_points_georef_csv={outputs['csv']}")
    print(f"A_lat={a[0]}")
    print(f"A_lon={a[1]}")
    print(f"B_lat={b[0]}")
    print(f"B_lon={b[1]}")
    print("motor_command_generated=false")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
