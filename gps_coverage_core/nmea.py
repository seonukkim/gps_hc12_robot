"""NMEA 문장 파서 / NMEA sentence parser.

목적/역할:
    원시 NMEA-0183 문장(예: `$GPGGA...`, `$GPRMC...`)을 `pynmea2`로 파싱해, 필요한 필드만 담은
    가벼운 `ParsedNMEA`(위치+선택적 고도/위성/HDOP)로 축약한다. GPS 수신기가 직접 뱉는 원시 문장을
    다루는 입력 어댑터이며, 로버 텔레메트리 프레임(telemetry.py)과는 다른 경로다.

시스템 내 위치:
    geo.py의 `GeoPoint`를 import 해 좌표를 담는다. NMEA 리플레이/로깅 도구가 로그 파일을 읽을 때
    `iter_nmea_file`을, 실시간/개별 문장 파싱에 `parse_nmea_sentence`를 사용한다. 파이프라인상 "센서
    원시 데이터 → 구조화 데이터" 변환 지점.

핵심 개념·불변식:
    - 지원 문장은 GGA(위치+고도+품질)와 상태 "A"(유효)인 RMC 두 가지뿐. 그 외/파싱 실패/무효 RMC는
      **None**을 반환한다(예외 아님) — 손상된 스트림에서도 계속 진행하기 위함. 이 관용을 유지할 것.
    - RMC는 고도/위성/HDOP를 제공하지 않으므로 해당 필드는 None.
    - `iter_nmea_file`은 `$`로 시작하지 않는 줄과 빈 줄을 건너뛰고, None이 아닌 결과만 yield 한다.

리팩토링 노트:
    새 문장 타입을 지원하려면 parse_nmea_sentence에 isinstance 분기를 추가한다. UTF-8 로그 인코딩을
    가정하며, 반환 dataclass는 frozen(불변).

Purpose: parse raw NMEA-0183 sentences (GGA/RMC) via ``pynmea2`` into a compact frozen
``ParsedNMEA`` (position + optional altitude/sats/HDOP). This is the input adapter for a GPS
receiver's own sentences — distinct from the rover telemetry-frame path in telemetry.py. Uses
geo.GeoPoint. Only GGA and status-"A" RMC are supported; anything else / parse failure / invalid RMC
returns **None** (not an exception) so a corrupt stream keeps flowing — preserve that leniency. RMC
carries no altitude/sats/HDOP (those fields stay None). ``iter_nmea_file`` skips blank and non-``$``
lines and yields only non-None results; assumes UTF-8 log files.
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Iterator

import pynmea2

from .geo import GeoPoint


@dataclass(frozen=True)
class ParsedNMEA:
    """NMEA 한 문장에서 뽑은 관심 필드(불변). / Fields of interest from one NMEA sentence (immutable)."""

    point: GeoPoint
    altitude_m: float | None
    satellites: int | None
    hdop: float | None
    sentence_type: str


def parse_nmea_sentence(sentence: str) -> ParsedNMEA | None:
    """지원 NMEA 문장을 ParsedNMEA로 파싱(미지원/실패 시 None). / Parse a supported NMEA sentence, else None.

    인자/Args: sentence=원시 NMEA 문장 문자열. 반환/Returns: GGA 또는 유효 RMC면 ParsedNMEA,
    그 외에는 None. 파싱 예외는 삼켜서 None으로 바꾼다(스트림 견고성). 부수효과 없음.
    Swallows parse errors into None for stream robustness; no side effects.
    """
    try:
        message = pynmea2.parse(sentence)
    except pynmea2.ParseError:
        # 손상된 문장은 예외 대신 None — 로그 반복 중 한 줄 오류로 전체가 멈추지 않게 한다.
        # / Corrupt sentence -> None instead of raising, so one bad line won't halt iteration.
        return None

    if isinstance(message, pynmea2.types.talker.GGA):
        return ParsedNMEA(
            point=GeoPoint(lat=message.latitude, lon=message.longitude),
            altitude_m=float(message.altitude) if message.altitude else None,
            satellites=int(message.num_sats) if message.num_sats else None,
            hdop=float(message.horizontal_dil) if message.horizontal_dil else None,
            sentence_type="GGA",
        )

    # 상태 "A"(Active/유효)인 RMC만 신뢰 — "V"(void)는 픽스 무효라 버린다.
    # / Only trust RMC with status "A" (active); "V" (void) means no valid fix, so drop it.
    if isinstance(message, pynmea2.types.talker.RMC) and message.status == "A":
        return ParsedNMEA(
            point=GeoPoint(lat=message.latitude, lon=message.longitude),
            altitude_m=None,
            satellites=None,
            hdop=None,
            sentence_type="RMC",
        )

    return None


def iter_nmea_file(path: str | Path) -> Iterator[ParsedNMEA]:
    """NMEA 로그 파일을 한 줄씩 파싱해 유효 레코드만 순회. / Iterate a NMEA log file, yielding valid records.

    인자/Args: path=NMEA 로그 경로. 산출/Yields: 파싱에 성공한 ParsedNMEA만(미지원·오류 줄은 건너뜀).
    부수효과/Side effects: 파일을 UTF-8로 읽는다(제너레이터 소비 시점에 I/O 발생).
    """
    for raw_line in Path(path).read_text(encoding="utf-8").splitlines():
        line = raw_line.strip()
        # NMEA 문장은 반드시 '$'로 시작 — 헤더/주석/빈 줄을 저렴하게 걸러낸다.
        # / NMEA sentences must begin with '$'; cheaply skip headers/comments/blank lines.
        if not line or not line.startswith("$"):
            continue
        parsed = parse_nmea_sentence(line)
        if parsed is not None:
            yield parsed
