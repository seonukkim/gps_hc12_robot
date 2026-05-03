from __future__ import annotations

import csv
import datetime as dt
import math
import os
import re
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable, Sequence

os.environ.setdefault("MPLCONFIGDIR", str(Path("/tmp") / "gps_hc12_matplotlib"))

import matplotlib

matplotlib.use("Agg")

import matplotlib.pyplot as plt
from matplotlib.axes import Axes
from matplotlib.figure import Figure

SCRIPT_DIR = Path(__file__).resolve().parent
REPO_ROOT = SCRIPT_DIR.parents[1]
TOOLS_DIR = REPO_ROOT / "tools"
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))
if str(TOOLS_DIR) not in sys.path:
    sys.path.insert(0, str(TOOLS_DIR))

DEFAULT_OUTPUT_DIRS = (
    REPO_ROOT / "docs/figures/generated",
    REPO_ROOT / "docs/reports/interim/figures/generated",
    REPO_ROOT / "docs/reports/final/figures/generated",
)

GPS_LOG_PATTERNS = (
    "data/nmea_logs/*.csv",
    "data/gps_logs/*.log",
)
SAFETY_LOG_PATTERNS = (
    "data/safety_logs/*.log",
    "data/gps_logs/*.log",
)
WAYPOINT_PATTERNS = (
    "data/mock_runs/**/waypoints.csv",
    "data/mock_runs/**/waypoints.json",
)

KEY_VALUE_RE = re.compile(r"([A-Za-z0-9_]+)=([^\s]+)")

COLOR_NAVY = "#243447"
COLOR_BLUE = "#2f6f9f"
COLOR_GREEN = "#3c8d5a"
COLOR_ORANGE = "#c77d2a"
COLOR_RED = "#b84a4a"
COLOR_GRAY = "#6f7782"
COLOR_LIGHT = "#eef2f5"
COLOR_YELLOW = "#d6a934"


@dataclass(frozen=True)
class FigureResult:
    filename: str
    script: str
    data_source: str
    recommended_use: str
    caption: str


@dataclass(frozen=True)
class GPSRecord:
    t_s: float
    fix_valid: bool
    lat: float | None
    lon: float | None
    satellites: int | None
    hdop: float | None


@dataclass(frozen=True)
class GPSDataset:
    records: list[GPSRecord]
    source_label: str
    data_source: str
    is_mock: bool


@dataclass(frozen=True)
class SafetyRecord:
    t_s: float
    mode: str
    rc_ok: bool
    auto_sw: bool
    ppm_age_ms: int | None
    steer_norm: float
    throttle_norm: float
    station_age_ms: int | None
    station_manual_valid: bool
    station_deadman: bool
    station_estop: bool
    control_source: str
    left_cmd: float
    right_cmd: float
    gps_fix: bool


@dataclass(frozen=True)
class SafetyDataset:
    records: list[SafetyRecord]
    source_label: str
    data_source: str
    is_mock: bool


@dataclass(frozen=True)
class WaypointDataset:
    waypoints: list[dict[str, float | int]]
    point_a: tuple[float, float]
    point_b: tuple[float, float]
    spacing_m: float
    source_label: str
    data_source: str
    is_mock: bool


def set_report_style() -> None:
    plt.rcParams.update(
        {
            "figure.facecolor": "white",
            "axes.facecolor": "white",
            "axes.edgecolor": "#2f3437",
            "axes.labelcolor": "#202427",
            "axes.titlecolor": "#202427",
            "axes.grid": True,
            "grid.color": "#c9d2d8",
            "grid.linestyle": "--",
            "grid.linewidth": 0.6,
            "grid.alpha": 0.55,
            "font.family": "DejaVu Sans",
            "font.size": 10,
            "axes.titlesize": 13,
            "axes.labelsize": 10,
            "legend.fontsize": 9,
            "xtick.labelsize": 9,
            "ytick.labelsize": 9,
            "lines.linewidth": 1.8,
            "savefig.facecolor": "white",
        }
    )


def repo_relative(path: Path) -> str:
    try:
        return path.resolve().relative_to(REPO_ROOT).as_posix()
    except ValueError:
        return path.as_posix()


def add_output_args(parser) -> None:
    parser.add_argument(
        "--output-dirs",
        nargs="*",
        default=None,
        help=(
            "Output directories. Defaults to docs/figures/generated and the interim/final "
            "report generated figure directories."
        ),
    )


def resolve_output_dirs(output_dirs: Sequence[str | Path] | None = None) -> list[Path]:
    if not output_dirs:
        return list(DEFAULT_OUTPUT_DIRS)
    resolved: list[Path] = []
    for raw_dir in output_dirs:
        path = Path(raw_dir)
        if not path.is_absolute():
            path = REPO_ROOT / path
        resolved.append(path)
    return resolved


def save_figure_all(
    fig: Figure,
    filename: str,
    output_dirs: Sequence[str | Path] | None = None,
    *,
    dpi: int = 220,
) -> list[Path]:
    saved_paths: list[Path] = []
    for output_dir in resolve_output_dirs(output_dirs):
        output_dir.mkdir(parents=True, exist_ok=True)
        output_path = output_dir / filename
        fig.savefig(output_path, dpi=dpi, bbox_inches="tight")
        saved_paths.append(output_path)
    plt.close(fig)
    return saved_paths


def discover_paths(patterns: Iterable[str]) -> list[Path]:
    paths: list[Path] = []
    for pattern in patterns:
        paths.extend(REPO_ROOT.glob(pattern))
    return sorted({path for path in paths if path.is_file()})


def add_source_note(ax: Axes, source: str, *, mock: bool = False) -> None:
    prefix = "MOCK/DEMO DATA" if mock else "Source"
    ax.text(
        0.0,
        -0.16,
        f"{prefix}: {source}",
        transform=ax.transAxes,
        ha="left",
        va="top",
        fontsize=8,
        color=COLOR_RED if mock else COLOR_GRAY,
    )


def add_panel_label(ax: Axes, text: str, *, color: str = COLOR_GRAY) -> None:
    ax.text(
        0.99,
        0.98,
        text,
        transform=ax.transAxes,
        ha="right",
        va="top",
        fontsize=8,
        color="white",
        bbox={
            "boxstyle": "round,pad=0.28",
            "facecolor": color,
            "edgecolor": color,
            "alpha": 0.95,
        },
    )


def finalize_axes(ax: Axes) -> None:
    ax.spines["top"].set_visible(False)
    ax.spines["right"].set_visible(False)


def equalize_xy(ax: Axes, xs: Sequence[float], ys: Sequence[float], *, pad_ratio: float = 0.12) -> None:
    if not xs or not ys:
        return
    min_x, max_x = min(xs), max(xs)
    min_y, max_y = min(ys), max(ys)
    span = max(max_x - min_x, max_y - min_y, 1.0)
    pad = span * pad_ratio
    center_x = (min_x + max_x) / 2.0
    center_y = (min_y + max_y) / 2.0
    ax.set_xlim(center_x - span / 2.0 - pad, center_x + span / 2.0 + pad)
    ax.set_ylim(center_y - span / 2.0 - pad, center_y + span / 2.0 + pad)
    ax.set_aspect("equal", adjustable="box")


def parse_optional_float(text: str | None) -> float | None:
    if text is None or text == "" or text == "NA":
        return None
    return float(text)


def parse_optional_int(text: str | None) -> int | None:
    if text is None or text == "" or text == "NA":
        return None
    return int(float(text))


def parse_bool_text(text: str | None) -> bool:
    if text is None:
        return False
    return text.strip().lower() in {"1", "true", "ok", "fix", "valid", "yes"}


def _parse_iso_timestamp(text: str) -> dt.datetime | None:
    try:
        parsed = dt.datetime.fromisoformat(text.replace("Z", "+00:00"))
    except ValueError:
        return None
    if parsed.tzinfo is None:
        parsed = parsed.replace(tzinfo=dt.UTC)
    return parsed


def _load_csv_gps_records(path: Path) -> list[GPSRecord]:
    records: list[GPSRecord] = []
    timestamps: list[dt.datetime | None] = []
    with path.open("r", encoding="utf-8", errors="replace", newline="") as handle:
        reader = csv.DictReader(handle)
        for row in reader:
            if not {"lat", "lon", "satellites", "hdop", "fix_valid"}.issubset(row):
                continue
            timestamps.append(_parse_iso_timestamp(row.get("timestamp_utc", "")))
            records.append(
                GPSRecord(
                    t_s=0.0,
                    fix_valid=parse_bool_text(row.get("fix_valid")),
                    lat=parse_optional_float(row.get("lat")),
                    lon=parse_optional_float(row.get("lon")),
                    satellites=parse_optional_int(row.get("satellites")),
                    hdop=parse_optional_float(row.get("hdop")),
                )
            )

    if not records:
        return records

    first_time = next((stamp for stamp in timestamps if stamp is not None), None)
    timed_records: list[GPSRecord] = []
    for index, record in enumerate(records):
        stamp = timestamps[index]
        if first_time is not None and stamp is not None:
            t_s = (stamp - first_time).total_seconds()
        else:
            t_s = float(index)
        timed_records.append(
            GPSRecord(
                t_s=t_s,
                fix_valid=record.fix_valid,
                lat=record.lat,
                lon=record.lon,
                satellites=record.satellites,
                hdop=record.hdop,
            )
        )
    return timed_records


def _load_usbdbg_gps_records(path: Path, sample_period_s: float) -> list[GPSRecord]:
    from analyze_gps_log import ParseError as GPSParseError
    from analyze_gps_log import parse_usbdbg_line as parse_gps_usbdbg_line

    records: list[GPSRecord] = []
    with path.open("r", encoding="utf-8", errors="replace") as handle:
        for line in handle:
            if "USBDBG" not in line:
                continue
            try:
                parsed = parse_gps_usbdbg_line(line)
            except (GPSParseError, ValueError):
                continue
            records.append(
                GPSRecord(
                    t_s=len(records) * sample_period_s,
                    fix_valid=parsed.gps_fix,
                    lat=parsed.gps_lat,
                    lon=parsed.gps_lon,
                    satellites=parsed.gps_sats,
                    hdop=parsed.gps_hdop,
                )
            )
    return records


def _load_serial3_gps_records(path: Path) -> list[GPSRecord]:
    records: list[GPSRecord] = []
    with path.open("r", encoding="utf-8", errors="replace") as handle:
        for line in handle:
            fields = dict(KEY_VALUE_RE.findall(line))
            if not {"status", "lat", "lon", "sats", "hdop"}.issubset(fields):
                continue
            fix_valid = fields["status"].upper() == "FIX"
            records.append(
                GPSRecord(
                    t_s=float(len(records)),
                    fix_valid=fix_valid,
                    lat=parse_optional_float(fields.get("lat")),
                    lon=parse_optional_float(fields.get("lon")),
                    satellites=parse_optional_int(fields.get("sats")),
                    hdop=parse_optional_float(fields.get("hdop")),
                )
            )
    return records


def load_gps_dataset(path: Path, *, usbdbg_sample_period_s: float = 0.5) -> GPSDataset | None:
    if path.suffix.lower() == ".csv":
        records = _load_csv_gps_records(path)
        kind = "GPS CSV"
    else:
        usbdbg_records = _load_usbdbg_gps_records(path, usbdbg_sample_period_s)
        serial3_records = _load_serial3_gps_records(path)
        if len(usbdbg_records) >= len(serial3_records):
            records = usbdbg_records
            kind = "USBDBG GPS log"
        else:
            records = serial3_records
            kind = "Serial3 GPS log"

    if not records:
        return None
    label = f"{repo_relative(path)} ({kind})"
    return GPSDataset(
        records=records,
        source_label=label,
        data_source=f"real log: {repo_relative(path)}",
        is_mock=False,
    )


def load_gps_datasets(paths: Sequence[Path] | None = None) -> list[GPSDataset]:
    candidate_paths = list(paths) if paths else discover_paths(GPS_LOG_PATTERNS)
    datasets: list[GPSDataset] = []
    for path in candidate_paths:
        dataset = load_gps_dataset(path)
        if dataset is not None:
            datasets.append(dataset)
    return datasets


def mock_gps_dataset() -> GPSDataset:
    records: list[GPSRecord] = []
    base_lat = 35.57323
    base_lon = 129.23986
    for index in range(90):
        fix = index >= 10
        sats = None if not fix else 4 + (index // 18) % 3
        hdop = None if not fix else 2.4 - min(index - 10, 50) * 0.018 + 0.04 * math.sin(index / 5)
        lat = None if not fix else base_lat + 0.000018 * math.sin(index / 7)
        lon = None if not fix else base_lon + 0.000014 * math.cos(index / 9)
        records.append(
            GPSRecord(
                t_s=float(index),
                fix_valid=fix,
                lat=lat,
                lon=lon,
                satellites=sats,
                hdop=hdop,
            )
        )
    return GPSDataset(
        records=records,
        source_label="deterministic GPS demo dataset",
        data_source="mock/demo",
        is_mock=True,
    )


def _load_usbdbg_safety_records(path: Path, sample_period_s: float) -> list[SafetyRecord]:
    from analyze_safety_log import ParseError as SafetyParseError
    from analyze_safety_log import parse_usbdbg_line as parse_safety_usbdbg_line

    records: list[SafetyRecord] = []
    with path.open("r", encoding="utf-8", errors="replace") as handle:
        for line in handle:
            if "USBDBG" not in line:
                continue
            try:
                parsed = parse_safety_usbdbg_line(line)
            except (SafetyParseError, ValueError):
                continue
            records.append(
                SafetyRecord(
                    t_s=len(records) * sample_period_s,
                    mode=parsed.mode,
                    rc_ok=parsed.rc_ok,
                    auto_sw=parsed.auto_sw,
                    ppm_age_ms=parsed.ppm_age_ms,
                    steer_norm=parsed.steer_norm,
                    throttle_norm=parsed.throttle_norm,
                    station_age_ms=parsed.station_age_ms,
                    station_manual_valid=parsed.station_manual_valid,
                    station_deadman=parsed.station_deadman,
                    station_estop=parsed.station_estop,
                    control_source=parsed.control_source,
                    left_cmd=parsed.left_cmd,
                    right_cmd=parsed.right_cmd,
                    gps_fix=parsed.gps_fix,
                )
            )
    return records


def load_safety_dataset(path: Path, *, sample_period_s: float = 0.5) -> SafetyDataset | None:
    records = _load_usbdbg_safety_records(path, sample_period_s)
    if not records:
        return None
    return SafetyDataset(
        records=records,
        source_label=f"{repo_relative(path)} (USBDBG safety log)",
        data_source=f"real log: {repo_relative(path)}",
        is_mock=False,
    )


def load_safety_datasets(paths: Sequence[Path] | None = None) -> list[SafetyDataset]:
    candidate_paths = list(paths) if paths else discover_paths(SAFETY_LOG_PATTERNS)
    datasets: list[SafetyDataset] = []
    for path in candidate_paths:
        dataset = load_safety_dataset(path)
        if dataset is not None:
            datasets.append(dataset)
    return datasets


def mock_safety_dataset() -> SafetyDataset:
    records: list[SafetyRecord] = []
    for index in range(100):
        if index < 20:
            mode = "FAILSAFE"
            rc_ok = False
            auto_sw = False
            control_source = "STOP"
            throttle = 0.0
            steer = 0.0
        elif index < 58:
            mode = "MANUAL"
            rc_ok = True
            auto_sw = False
            control_source = "RC_MANUAL"
            throttle = 0.55 * math.sin((index - 20) / 5)
            steer = 0.35 * math.sin((index - 20) / 8)
        elif index < 75:
            mode = "AUTO_READY"
            rc_ok = True
            auto_sw = True
            control_source = "STOP"
            throttle = 0.0
            steer = 0.0
        else:
            mode = "MANUAL"
            rc_ok = True
            auto_sw = False
            control_source = "RC_MANUAL"
            throttle = 0.0
            steer = 0.0
        left = 0.0 if control_source == "STOP" else max(-1.0, min(1.0, throttle - steer))
        right = 0.0 if control_source == "STOP" else max(-1.0, min(1.0, throttle + steer))
        records.append(
            SafetyRecord(
                t_s=index * 0.5,
                mode=mode,
                rc_ok=rc_ok,
                auto_sw=auto_sw,
                ppm_age_ms=None if not rc_ok else 12,
                steer_norm=steer,
                throttle_norm=throttle,
                station_age_ms=None,
                station_manual_valid=False,
                station_deadman=False,
                station_estop=False,
                control_source=control_source,
                left_cmd=left,
                right_cmd=right,
                gps_fix=index > 12,
            )
        )
    return SafetyDataset(
        records=records,
        source_label="deterministic manual/failsafe demo dataset",
        data_source="mock/demo",
        is_mock=True,
    )


def _read_waypoints_csv(path: Path) -> list[dict[str, float | int]]:
    waypoints: list[dict[str, float | int]] = []
    with path.open("r", encoding="utf-8", newline="") as handle:
        reader = csv.DictReader(handle)
        for row in reader:
            if not {"order", "lane", "lat", "lon", "x", "y"}.issubset(row):
                continue
            waypoints.append(
                {
                    "order": int(row["order"]),
                    "lane": int(row["lane"]),
                    "lat": float(row["lat"]),
                    "lon": float(row["lon"]),
                    "x": float(row["x"]),
                    "y": float(row["y"]),
                }
            )
    return sorted(waypoints, key=lambda waypoint: int(waypoint["order"]))


def _read_waypoints_json(path: Path) -> list[dict[str, float | int]]:
    import json

    with path.open("r", encoding="utf-8") as handle:
        raw_items = json.load(handle)
    waypoints: list[dict[str, float | int]] = []
    for item in raw_items:
        if not {"order", "lane", "lat", "lon", "x", "y"}.issubset(item):
            continue
        waypoints.append(
            {
                "order": int(item["order"]),
                "lane": int(item["lane"]),
                "lat": float(item["lat"]),
                "lon": float(item["lon"]),
                "x": float(item["x"]),
                "y": float(item["y"]),
            }
        )
    return sorted(waypoints, key=lambda waypoint: int(waypoint["order"]))


def _infer_spacing_m(waypoints: Sequence[dict[str, float | int]]) -> float:
    first_by_lane: dict[int, tuple[float, float]] = {}
    for waypoint in waypoints:
        lane = int(waypoint["lane"])
        first_by_lane.setdefault(lane, (float(waypoint["x"]), float(waypoint["y"])))
    lane_points = [first_by_lane[lane] for lane in sorted(first_by_lane)]
    if len(lane_points) < 2:
        return 0.0
    distances = [
        math.hypot(curr[0] - prev[0], curr[1] - prev[1])
        for prev, curr in zip(lane_points, lane_points[1:])
    ]
    return sum(distances) / len(distances)


def load_waypoint_dataset(paths: Sequence[Path] | None = None) -> WaypointDataset:
    from gps_coverage_core.planner import generate_lawnmower_path

    candidate_paths = list(paths) if paths else discover_paths(WAYPOINT_PATTERNS)
    for path in candidate_paths:
        try:
            waypoints = _read_waypoints_csv(path) if path.suffix == ".csv" else _read_waypoints_json(path)
        except (OSError, ValueError, KeyError):
            continue
        if len(waypoints) >= 2:
            point_a = (float(waypoints[0]["lat"]), float(waypoints[0]["lon"]))
            point_b = (float(waypoints[1]["lat"]), float(waypoints[1]["lon"]))
            return WaypointDataset(
                waypoints=waypoints,
                point_a=point_a,
                point_b=point_b,
                spacing_m=_infer_spacing_m(waypoints),
                source_label=f"{repo_relative(path)} (mock mission waypoints)",
                data_source=f"mock mission: {repo_relative(path)}",
                is_mock=True,
            )

    point_a_dict = {"lat": 35.573188, "lon": 129.239825}
    point_b_dict = {"lat": 35.573250, "lon": 129.240000}
    spacing_m = 8.0
    waypoints = generate_lawnmower_path(
        point_a=point_a_dict,
        point_b=point_b_dict,
        spacing_m=spacing_m,
        num_lanes=4,
    )
    return WaypointDataset(
        waypoints=waypoints,
        point_a=(point_a_dict["lat"], point_a_dict["lon"]),
        point_b=(point_b_dict["lat"], point_b_dict["lon"]),
        spacing_m=spacing_m,
        source_label="deterministic A/B mock mission",
        data_source="mock/demo",
        is_mock=True,
    )


def fixed_xy_records(records: Sequence[GPSRecord]) -> list[GPSRecord]:
    return [
        record
        for record in records
        if record.fix_valid and record.lat is not None and record.lon is not None
    ]


def gps_local_xy(records: Sequence[GPSRecord]) -> tuple[list[float], list[float]]:
    from gps_coverage_core.planner import latlon_to_xy

    fixed = fixed_xy_records(records)
    if not fixed:
        return [], []
    origin = fixed[0]
    assert origin.lat is not None and origin.lon is not None
    xs: list[float] = []
    ys: list[float] = []
    for record in fixed:
        assert record.lat is not None and record.lon is not None
        x_m, y_m = latlon_to_xy(record.lat, record.lon, origin.lat, origin.lon)
        xs.append(x_m)
        ys.append(y_m)
    return xs, ys


def category_runs(values: Sequence[str]) -> list[tuple[str, int, int]]:
    if not values:
        return []
    runs: list[tuple[str, int, int]] = []
    current = values[0]
    start = 0
    for index, value in enumerate(values[1:], start=1):
        if value == current:
            continue
        runs.append((current, start, index - 1))
        current = value
        start = index
    runs.append((current, start, len(values) - 1))
    return runs


def write_figure_captions(results: Sequence[FigureResult]) -> Path:
    output_path = REPO_ROOT / "docs/figures/generated/figure_captions.md"
    output_path.parent.mkdir(parents=True, exist_ok=True)
    lines = [
        "# Generated Figure Captions",
        "",
        "Generated by `python3 scripts/analysis/generate_all_figures.py`.",
        "Captions describe what each figure shows without treating mock or schematic data as measured results.",
        "",
    ]
    for result in results:
        lines.extend(
            [
                f"## {result.filename}",
                "",
                f"- Script: `{result.script}`",
                f"- Data source: {result.data_source}",
                f"- Recommended use: {result.recommended_use}",
                f"- Caption: {result.caption}",
                "",
            ]
        )
    output_path.write_text("\n".join(lines), encoding="utf-8")
    return output_path


set_report_style()
