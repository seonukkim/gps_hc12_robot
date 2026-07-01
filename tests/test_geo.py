"""geo.py 좌표 왕복 변환 계약 테스트 / Contract test for geo.py coordinate round-trips.

목적/역할:
    `gps_coverage_core.geo`의 측지선(geodesic) 기반 변환이 위경도 ⇄ 로컬 East/North 미터
    사이를 손실 없이 왕복하는지 잠근다. latlon_to_local → local_to_latlon 을 이어 붙였을 때
    원래 점으로 (오차 1e-6도 이내) 되돌아와야 한다는 불변식을 검증한다.

시스템 내 위치:
    coverage 파이프라인의 "정밀 좌표 변환" 계층(geo.py, Karney 측지선)을 대상으로 한다.
    빠른 근사인 planner.py의 latlon_to_xy 왕복은 test_planner.py가 따로 검증하므로,
    이 파일은 정밀 경로 전용 계약이다.

핵심 개념·불변식:
    - 로컬 프레임 규약: x_m=East, y_m=North(geo.py와 동일). 이 테스트는 규약을 전제한다.
    - 허용 오차 1e-6도(≈부산 위도에서 수 cm 수준). 이 임계값을 완화하면 계약이 느슨해진다.

리팩토링 노트:
    geo.py의 변환식이나 프레임 규약을 바꾸면 이 왕복이 먼저 깨진다 — 조기 경보 역할.

Contract test: geo.py geodesic conversions must round-trip lat/lon <-> local East/North meters
losslessly (latlon_to_local then local_to_latlon returns the original point within 1e-6 deg). This
guards the precise-conversion layer (Karney geodesics); planner.py's fast approximate round-trip is
covered separately in test_planner.py.
"""

from gps_coverage_core.geo import GeoPoint, latlon_to_local, local_to_latlon


def test_roundtrip_conversion() -> None:
    """왕복 변환 후 원점 복원(≤1e-6도) 검증. / A point survives local<->latlon round-trip within 1e-6 deg.

    부산 인근 원점에서 약 50~60 m 떨어진 점을 로컬로 변환했다가 되돌려, 위/경도 오차가
    1e-6도 미만인지 확인한다. / Convert a nearby point to local and back; lat & lon must agree.
    """
    origin = GeoPoint(lat=35.123456, lon=129.123456)
    point = GeoPoint(lat=35.123956, lon=129.123956)

    local_point = latlon_to_local(origin, point)
    roundtrip = local_to_latlon(origin, local_point)

    assert abs(roundtrip.lat - point.lat) < 1e-6
    assert abs(roundtrip.lon - point.lon) < 1e-6
