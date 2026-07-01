"""Core protocol, geodesy, telemetry, and planning utilities.

목적/역할:
    `gps_coverage_core`는 이 로버 프로젝트의 **순수(하드웨어 비의존) 커버리지 경로 수학** 패키지다.
    위/경도 ↔ 미터 변환, 레인 오프셋, 사행(serpentine/보устрофедон) 웨이포인트 생성, NMEA/IMU/텔레메트리
    파싱, 그리고 PC↔로버 명령 프레임/체크섬 프로토콜을 담는다. 시리얼 포트·GPS 수신기·모터 같은
    실제 장치에는 전혀 의존하지 않으므로 단위 테스트와 오프라인 도구에서 그대로 재사용된다.

시스템 내 위치:
    이 `__init__`은 패키지의 공개 표면(façade)이다. `tools/`(예: path_preview, hc12_link_probe,
    hc12_operational_diagnose), `scripts/station/plan_coverage_path.py`, `archive/tools/`의 스테이션
    도구, 그리고 `tests/`가 여기서 심볼을 import 한다. 파이프라인 상 위치: (경로 계획) planner →
    (전송) protocol → (수신 파싱) nmea/telemetry/imu 순으로 흐르는 핵심 로직의 진입 지점.

핵심 개념:
    `__all__`은 안정 공개 API를 고정한다 — 여기 나열된 이름만 재수출(re-export)하며, 새 심볼을 노출할
    때는 하위 모듈 추가와 함께 이 목록도 갱신해야 한다. planner.py의 등거리 근사(latlon_to_xy)와
    geo.py의 측지선(GeoPoint) 두 좌표계가 공존한다는 점에 주의(각각의 모듈 문서 참고).

Purpose / role:
    Pure, hardware-independent coverage-path math for the rover: lat/lon<->meter conversions,
    lane offsets, serpentine/boustrophedon waypoint generation, NMEA/IMU/telemetry parsing, and
    the PC<->rover command frame + checksum protocol. No serial/GPS/motor dependencies, so it is
    reused directly by unit tests and offline tools.

System placement:
    This ``__init__`` is the package façade. ``tools/``, ``scripts/station/``, archived station
    tools, and ``tests/`` import symbols from here. Pipeline: planner (plan) -> protocol (send) ->
    nmea/telemetry/imu (parse received data).

Refactoring note:
    ``__all__`` pins the stable public API. Only names listed here are re-exported; keep it in sync
    when adding submodules. Note two coordinate systems coexist (planner's equirectangular
    approximation vs. geo's geodesic Geodesic.WGS84) — see each module's docstring before mixing them.
"""

from .geo import GeoPoint, LocalPoint, latlon_to_local, local_to_latlon
from .planner import generate_lawnmower_path, latlon_to_xy, xy_to_latlon
from .protocol import checksum_xor, decode_frame, encode_frame
from .telemetry import GPSTelemetry

# 공개 API 화이트리스트 — 여기 있는 이름만 `from gps_coverage_core import ...`로 노출된다.
# / Public API whitelist: only these names are re-exported at package level.
__all__ = [
    "GPSTelemetry",
    "GeoPoint",
    "LocalPoint",
    "checksum_xor",
    "decode_frame",
    "encode_frame",
    "generate_lawnmower_path",
    "latlon_to_local",
    "latlon_to_xy",
    "local_to_latlon",
    "xy_to_latlon",
]
