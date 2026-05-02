from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Iterator

import pynmea2

from .geo import GeoPoint


@dataclass(frozen=True)
class ParsedNMEA:
    point: GeoPoint
    altitude_m: float | None
    satellites: int | None
    hdop: float | None
    sentence_type: str


def parse_nmea_sentence(sentence: str) -> ParsedNMEA | None:
    """Parse a supported NMEA sentence into a compact data object."""
    try:
        message = pynmea2.parse(sentence)
    except pynmea2.ParseError:
        return None

    if isinstance(message, pynmea2.types.talker.GGA):
        return ParsedNMEA(
            point=GeoPoint(lat=message.latitude, lon=message.longitude),
            altitude_m=float(message.altitude) if message.altitude else None,
            satellites=int(message.num_sats) if message.num_sats else None,
            hdop=float(message.horizontal_dil) if message.horizontal_dil else None,
            sentence_type="GGA",
        )

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
    for raw_line in Path(path).read_text(encoding="utf-8").splitlines():
        line = raw_line.strip()
        if not line or not line.startswith("$"):
            continue
        parsed = parse_nmea_sentence(line)
        if parsed is not None:
            yield parsed
