"""로버 GPS 텔레메트리 레코드 / Rover GPS telemetry record.

목적/역할:
    로버가 보내온 `GPS` 프레임의 PAYLOAD(쉼표로 나뉜 문자열 필드)를 강타입 `GPSTelemetry`
    레코드로 파싱하고, 로그 저장을 위해 CSV 행(dict)으로 되돌린다. 수신 파이프라인의 마지막 단계다:
    protocol.decode_frame → payload 분리 → GPSTelemetry.from_payload → CSV.

시스템 내 위치:
    `gps_logger` 등 스테이션 측 도구가 `decode_frame`으로 프레임을 풀고 그 payload를
    `from_payload`에 넘긴다. 패키지 `__init__`이 이 클래스를 재수출한다. protocol.py(프레임 계층)
    바로 위에 있는 애플리케이션 데이터 계층.

핵심 개념·불변식:
    - PAYLOAD 필드 순서 = lat, lon, alt_m, satellites, hdop, fix_valid (총 6개). 이 순서는 로버
      펌웨어의 송신 순서와 반드시 일치해야 하는 **와이어 계약**이다. 바꾸면 양쪽을 동시에 고쳐야 한다.
    - `fix_valid`는 문자열을 불리언으로 해석하며 참으로 보는 토큰 집합은 {"1","true","TRUE","True","OK"}.
    - dataclass는 frozen(불변). CSV 출력 시 fix_valid는 int(0/1)로 직렬화된다.

리팩토링 노트:
    필드를 추가/변경할 때는 (1) from_payload의 인덱스, (2) as_csv_row의 키, (3) 로버 펌웨어 송신부를
    함께 맞춰야 한다. 세 곳이 결합되어 있다.

Purpose: parse a rover ``GPS`` frame payload (comma-split string fields) into a typed
``GPSTelemetry`` and re-emit it as a CSV-row dict. Last stage of the receive pipeline:
protocol.decode_frame -> split payload -> GPSTelemetry.from_payload -> CSV. Used by station tools
such as ``gps_logger``; re-exported by the package. The payload field ORDER
(lat, lon, alt_m, satellites, hdop, fix_valid) is a wire contract shared with the rover firmware —
changing it means changing both sides plus ``as_csv_row``. Frozen dataclass; fix_valid serializes as
int 0/1.
"""

from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class GPSTelemetry:
    """디코드된 GPS 픽스 한 건(불변). / One decoded GPS fix (immutable)."""

    seq: int
    lat: float
    lon: float
    alt_m: float
    satellites: int
    hdop: float
    fix_valid: bool

    @classmethod
    def from_payload(cls, seq: int, payload: list[str]) -> "GPSTelemetry":
        """GPS 프레임 payload 필드를 파싱해 인스턴스 생성. / Build an instance from GPS payload fields.

        인자/Args: seq=프레임 시퀀스 번호, payload=쉼표 분리된 문자열 필드 리스트
        (순서: lat, lon, alt, sats, hdop, fix_valid). 6개 미만이면 ValueError.
        Raises ValueError when fewer than 6 fields are supplied.
        """
        # 필드 수 부족은 손상된/불완전한 프레임 신호 — 조용히 무시하지 않고 즉시 실패시킨다.
        # / Too few fields signals a corrupt/partial frame; fail loudly rather than silently.
        if len(payload) < 6:
            raise ValueError("GPS payload requires lat, lon, alt, sats, hdop, fix_valid")

        return cls(
            seq=seq,
            lat=float(payload[0]),
            lon=float(payload[1]),
            alt_m=float(payload[2]),
            satellites=int(payload[3]),
            hdop=float(payload[4]),
            # 펌웨어 버전마다 fix 플래그 표기가 달라 여러 참(true) 토큰을 허용한다.
            # / Accept several truthy spellings because firmware variants encode the fix flag differently.
            fix_valid=payload[5].strip() in {"1", "true", "TRUE", "True", "OK"},
        )

    def as_csv_row(self) -> dict[str, object]:
        """로그용 CSV 행(dict)으로 직렬화. / Serialize to a CSV-row dict for logging.

        fix_valid는 CSV 친화적으로 int(0/1)로 변환된다. / fix_valid is emitted as int 0/1 for CSV.
        """
        return {
            "seq": self.seq,
            "lat": self.lat,
            "lon": self.lon,
            "alt_m": self.alt_m,
            "satellites": self.satellites,
            "hdop": self.hdop,
            "fix_valid": int(self.fix_valid),
        }
