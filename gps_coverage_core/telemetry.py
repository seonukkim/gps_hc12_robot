from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class GPSTelemetry:
    seq: int
    lat: float
    lon: float
    alt_m: float
    satellites: int
    hdop: float
    fix_valid: bool

    @classmethod
    def from_payload(cls, seq: int, payload: list[str]) -> "GPSTelemetry":
        if len(payload) < 6:
            raise ValueError("GPS payload requires lat, lon, alt, sats, hdop, fix_valid")

        return cls(
            seq=seq,
            lat=float(payload[0]),
            lon=float(payload[1]),
            alt_m=float(payload[2]),
            satellites=int(payload[3]),
            hdop=float(payload[4]),
            fix_valid=payload[5].strip() in {"1", "true", "TRUE", "True", "OK"},
        )

    def as_csv_row(self) -> dict[str, object]:
        return {
            "seq": self.seq,
            "lat": self.lat,
            "lon": self.lon,
            "alt_m": self.alt_m,
            "satellites": self.satellites,
            "hdop": self.hdop,
            "fix_valid": int(self.fix_valid),
        }
