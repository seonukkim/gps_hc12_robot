from gps_coverage_core.geo import GeoPoint, latlon_to_local, local_to_latlon


def test_roundtrip_conversion() -> None:
    origin = GeoPoint(lat=35.123456, lon=129.123456)
    point = GeoPoint(lat=35.123956, lon=129.123956)

    local_point = latlon_to_local(origin, point)
    roundtrip = local_to_latlon(origin, local_point)

    assert abs(roundtrip.lat - point.lat) < 1e-6
    assert abs(roundtrip.lon - point.lon) < 1e-6
