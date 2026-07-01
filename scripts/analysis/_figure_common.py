"""공용 그림 생성 헬퍼 / Shared figure-generation helpers.

목적/역할:
    보고서용 그림(figure)을 만드는 여러 `generate_*_figures.py` 스크립트가 공통으로
    쓰는 도구 상자다. 로그·웨이포인트 데이터 로딩(GPS/안전/미션), matplotlib 스타일과
    캔버스 헬퍼(축 정리·패널 라벨·출처 표기), 여러 산출 디렉터리에 그림 저장, 결정론적
    mock/데모 데이터 생성, 그리고 그림 캡션 마크다운 작성까지 담당한다.
    Toolbox shared by the ``generate_*_figures.py`` scripts that build report figures:
    it loads GPS/safety/waypoint data, applies a consistent matplotlib style, provides
    canvas helpers, saves each figure to several output dirs, produces deterministic
    mock/demo datasets, and writes the figure-caption markdown.

시스템 내 위치 / Where this sits in the pipeline:
    - import 하는 쪽 / imported by: ``generate_all_figures.py``, ``generate_system_figures.py``,
      ``generate_gps_figures.py``, ``generate_manual_control_figures.py``,
      ``generate_path_figures.py`` (즉 그림 파이프라인 전반).
    - import 하는 대상 / imports from: ``analyze_gps_log`` / ``analyze_safety_log``
      (USBDBG 라인 파서, 지연 import), ``gps_coverage_core.planner``
      (``generate_lawnmower_path``, ``latlon_to_xy``, 지연 import).
    - 파이프라인 위치 / stage: 데이터 → (여기: 로딩·스타일·저장) → 개별 그림 스크립트 → PNG + 캡션.

핵심 개념·불변식 / Key concepts and invariants:
    - ``MPLCONFIGDIR``는 matplotlib를 import 하기 *전에* 설정해야 하고, backend는
      ``Agg``(headless, 화면 없음)이다. 이 두 줄의 위치·순서는 함정이므로 옮기지 말 것.
      ``MPLCONFIGDIR`` must be set *before* importing matplotlib; backend is headless ``Agg``.
    - ``REPO_ROOT`` = 이 파일에서 두 단계 상위(``scripts/analysis`` → repo 루트). ``REPO_ROOT``와
      ``tools/``를 ``sys.path`` 앞에 넣어 위 모듈들을 import 가능하게 만든다.
    - mock 데이터셋은 난수를 쓰지 않고 결정론적(sin/cos 기반)이라 그림이 재현 가능하다.
      Mock datasets are deterministic (sin/cos), so figures are reproducible.
    - ``is_mock`` / ``data_source`` 플래그로 mock·스키매틱 데이터를 실측 결과처럼 표기하지
      않는 것이 중요한 불변식이다. Never present mock/schematic data as measured results.

사용법/진입점 / Usage:
    이 파일 자체는 CLI 진입점이 없다. import 시 맨 아래에서 ``set_report_style()``이 한 번
    호출되어 전역 rcParams가 세팅된다(import 부수효과). 각 그림 스크립트가 여기 함수를 호출한다.
    No CLI of its own; importing it calls ``set_report_style()`` once (side effect).

리팩토링 노트 / Refactoring notes:
    - ``analyze_gps_log`` / ``analyze_safety_log`` / ``gps_coverage_core.planner`` 와 결합돼
      있고, 로그 포맷(CSV 헤더, USBDBG, Serial3 key=value)에 의존한다. 포맷이 바뀌면
      로더도 함께 고쳐야 한다.
    - 지연 import는 무거운/선택적 의존성을 top-level에서 피하기 위한 의도적 선택이다.
    - 색상 상수·``@dataclass(frozen=True)`` 레코드는 그림 스크립트 전반의 공용 계약이므로
      필드 변경 시 하위 호환에 주의.
"""

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

# ── matplotlib 부트스트랩 / matplotlib bootstrap ──
# MPLCONFIGDIR는 matplotlib import 전에 지정해야 캐시 위치가 고정된다(권한/HOME 없는 환경 대비).
# Set MPLCONFIGDIR before importing matplotlib so its cache dir is writable in headless CI.
os.environ.setdefault("MPLCONFIGDIR", str(Path("/tmp") / "gps_hc12_matplotlib"))

import matplotlib

# 화면 없는 서버/CI에서 그림을 파일로만 저장하므로 non-interactive Agg 백엔드를 강제한다.
# Force the non-interactive Agg backend: figures are saved to files, never displayed.
matplotlib.use("Agg")

import matplotlib.pyplot as plt
from matplotlib.axes import Axes
from matplotlib.figure import Figure

# ── 경로 해석 및 import 경로 설정 / Path resolution and sys.path setup ──
# 이 파일은 scripts/analysis/ 에 있으므로 parents[1] 이 repo 루트다.
# This file lives in scripts/analysis/, so parents[1] is the repo root.
SCRIPT_DIR = Path(__file__).resolve().parent
REPO_ROOT = SCRIPT_DIR.parents[1]
TOOLS_DIR = REPO_ROOT / "tools"
# analyze_gps_log / analyze_safety_log(tools/)와 gps_coverage_core(repo 루트) import 를
# 위해 두 경로를 sys.path 앞에 넣는다. Make repo-root and tools/ importable for the loaders.
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))
if str(TOOLS_DIR) not in sys.path:
    sys.path.insert(0, str(TOOLS_DIR))

# ── 기본 산출 경로 및 데이터 탐색 패턴 / Default outputs and data-discovery globs ──
# 그림은 문서 트리(공용 + 중간/최종 보고서)의 세 곳에 동시에 저장된다.
# Each figure is written to three doc locations at once (shared + interim/final report).
DEFAULT_OUTPUT_DIRS = (
    REPO_ROOT / "docs/figures/generated",
    REPO_ROOT / "docs/reports/interim/figures/generated",
    REPO_ROOT / "docs/reports/final/figures/generated",
)

# 데이터를 명시하지 않으면 REPO_ROOT 기준으로 아래 glob 패턴을 탐색한다.
# When no explicit paths are given, these globs are searched relative to REPO_ROOT.
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

# Serial3 GPS 로그의 `key=value` 토큰을 뽑는 정규식(예: `lat=35.5 lon=129.2 status=FIX`).
# Regex extracting `key=value` tokens from Serial3 GPS log lines.
KEY_VALUE_RE = re.compile(r"([A-Za-z0-9_]+)=([^\s]+)")

# ── 보고서 팔레트 / Report color palette ──
# 그림 스크립트들이 공유하는 색상 상수. 값 변경 시 모든 그림의 색이 바뀐다.
# Shared palette constants; changing a value re-colors every figure.
COLOR_NAVY = "#243447"
COLOR_BLUE = "#2f6f9f"
COLOR_GREEN = "#3c8d5a"
COLOR_ORANGE = "#c77d2a"
COLOR_RED = "#b84a4a"
COLOR_GRAY = "#6f7782"
COLOR_LIGHT = "#eef2f5"
COLOR_YELLOW = "#d6a934"


# ── 불변(frozen) 데이터 모델 / Immutable data models ──
# 아래 dataclass들은 그림 스크립트 전반의 공용 계약이다. frozen 이라 생성 후 수정 불가하며,
# 필드 추가/이름 변경은 모든 로더·그림 코드에 파급된다. These frozen records form the shared
# contract across figure scripts; renaming/adding fields ripples through all loaders.


@dataclass(frozen=True)
class FigureResult:
    """한 개 그림의 산출 메타데이터(파일명·스크립트·출처·용도·캡션).

    Metadata for one produced figure; consumed by ``write_figure_captions``.
    """

    filename: str
    script: str
    data_source: str
    recommended_use: str
    caption: str


@dataclass(frozen=True)
class GPSRecord:
    """한 개 GPS 샘플: 상대 시각(초), fix 유효성, 위경도, 위성 수, HDOP.

    One GPS sample; ``lat``/``lon``/``satellites``/``hdop`` are ``None`` when no fix.
    """

    t_s: float
    fix_valid: bool
    lat: float | None
    lon: float | None
    satellites: int | None
    hdop: float | None


@dataclass(frozen=True)
class GPSDataset:
    """GPS 레코드 묶음 + 출처 라벨/설명 + mock 여부.

    Bundle of GPS records with source labels and a mock flag.
    """

    records: list[GPSRecord]
    source_label: str
    data_source: str
    is_mock: bool


@dataclass(frozen=True)
class SafetyRecord:
    """한 개 안전(USBDBG) 샘플: 모드·RC 상태·명령·스테이션/데드맨/E-STOP·모터 출력.

    One safety-log sample decoded from a USBDBG debug line.
    """

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
    """안전 레코드 묶음 + 출처 라벨/설명 + mock 여부.

    Bundle of safety records with source labels and a mock flag.
    """

    records: list[SafetyRecord]
    source_label: str
    data_source: str
    is_mock: bool


@dataclass(frozen=True)
class WaypointDataset:
    """미션 웨이포인트 + A/B 지점 + 레인 간격(m) + 출처/ mock 여부.

    Mission waypoints plus A/B endpoints and inferred lane spacing.
    """

    waypoints: list[dict[str, float | int]]
    point_a: tuple[float, float]
    point_b: tuple[float, float]
    spacing_m: float
    source_label: str
    data_source: str
    is_mock: bool


# ── 스타일 & 캔버스 헬퍼 / Style and canvas helpers ──


def set_report_style() -> None:
    """보고서용 전역 matplotlib rcParams(색·폰트·격자)를 설정한다.

    Apply the report-wide matplotlib rcParams (colors, fonts, grid). Side effect:
    mutates global ``plt.rcParams``. Called once at import time (bottom of file).
    """
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
    """경로를 repo 루트 기준 상대 POSIX 문자열로 변환(불가하면 원본 그대로).

    Return ``path`` relative to the repo root; fall back to the raw path if outside it.
    """
    try:
        return path.resolve().relative_to(REPO_ROOT).as_posix()
    except ValueError:
        return path.as_posix()


def add_output_args(parser) -> None:
    """argparse 파서에 ``--output-dirs`` 옵션을 붙인다(그림 저장 위치 지정).

    Register the shared ``--output-dirs`` argument on an argparse parser.
    """
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
    """산출 디렉터리 목록을 확정한다(미지정 시 기본 3곳, 상대경로는 repo 루트 기준으로 절대화).

    Resolve the list of output dirs: default trio when empty; relative paths are
    anchored under the repo root.
    """
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
    """같은 그림을 모든 산출 디렉터리에 저장하고 저장 경로 목록을 반환한다.

    Save one figure to every resolved output dir and return the saved paths.
    부수효과 / Side effects: 디렉터리 생성(mkdir), 저장 후 ``plt.close(fig)`` 로 메모리 해제
    (반복 생성 시 figure 누수를 막기 위한 것이므로 제거 금지).
    """
    saved_paths: list[Path] = []
    for output_dir in resolve_output_dirs(output_dirs):
        output_dir.mkdir(parents=True, exist_ok=True)
        output_path = output_dir / filename
        fig.savefig(output_path, dpi=dpi, bbox_inches="tight")
        saved_paths.append(output_path)
    # 그림 핸들 해제 / release figure to avoid leaks across many saves
    plt.close(fig)
    return saved_paths


def discover_paths(patterns: Iterable[str]) -> list[Path]:
    """glob 패턴들을 repo 루트에서 확장해 실제 파일만 정렬·중복제거하여 반환.

    Expand glob patterns from the repo root; return sorted, de-duplicated files only.
    """
    paths: list[Path] = []
    for pattern in patterns:
        paths.extend(REPO_ROOT.glob(pattern))
    return sorted({path for path in paths if path.is_file()})


# ── 축(Axes) 주석 헬퍼 / Axes annotation helpers ──


def add_source_note(ax: Axes, source: str, *, mock: bool = False) -> None:
    """축 아래에 데이터 출처 캡션을 단다(mock이면 붉은색 'MOCK/DEMO DATA' 표기).

    Add a small source caption below the axes; mock data is flagged in red so it is
    never mistaken for a measured result.
    """
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
    """축 우상단에 둥근 배지 형태의 패널 라벨(예: 'A', 'B')을 그린다.

    Draw a rounded badge label in the top-right corner of the axes.
    """
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
    """상·우 스파인(테두리)을 숨겨 보고서 스타일로 정리한다.

    Hide the top/right spines for a cleaner report look.
    """
    ax.spines["top"].set_visible(False)
    ax.spines["right"].set_visible(False)


def equalize_xy(ax: Axes, xs: Sequence[float], ys: Sequence[float], *, pad_ratio: float = 0.12) -> None:
    """x·y 축을 같은 스케일(정사각 aspect)로 맞추고 데이터에 여백을 준다.

    Force an equal-aspect square view around the data (used for map-like XY plots).
    빈 입력이면 아무것도 하지 않는다 / no-op on empty input.
    """
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


# ── 필드 파싱 헬퍼 / Field-parsing helpers ──


def parse_optional_float(text: str | None) -> float | None:
    """빈 문자열/None/"NA"는 None으로, 그 외는 float로 파싱.

    Parse to float; treat ``None``/empty/``"NA"`` as missing (None).
    """
    if text is None or text == "" or text == "NA":
        return None
    return float(text)


def parse_optional_int(text: str | None) -> int | None:
    """빈 문자열/None/"NA"는 None으로, 그 외는 int(float(...))로 파싱(소수 표기 허용).

    Parse to int via float; treat ``None``/empty/``"NA"`` as missing (None).
    """
    if text is None or text == "" or text == "NA":
        return None
    return int(float(text))


def parse_bool_text(text: str | None) -> bool:
    """"1/true/ok/fix/valid/yes"(대소문자 무시)만 참으로 보는 관대한 불리언 파서.

    Lenient boolean parse; only the listed truthy tokens count as True.
    """
    if text is None:
        return False
    return text.strip().lower() in {"1", "true", "ok", "fix", "valid", "yes"}


def _parse_iso_timestamp(text: str) -> dt.datetime | None:
    """ISO-8601 문자열을 tz-aware datetime으로 파싱(Z→UTC, naive는 UTC로 간주). 실패 시 None.

    Parse an ISO-8601 timestamp to a UTC-aware datetime; ``None`` on failure.
    """
    try:
        parsed = dt.datetime.fromisoformat(text.replace("Z", "+00:00"))
    except ValueError:
        return None
    if parsed.tzinfo is None:
        parsed = parsed.replace(tzinfo=dt.UTC)
    return parsed


# ── GPS 데이터 로딩 / GPS data loading ──
# 세 가지 원본 포맷을 지원: (1) CSV 헤더, (2) USBDBG 디버그 라인, (3) Serial3 key=value 로그.
# Three source formats are supported: CSV, USBDBG debug lines, Serial3 key=value logs.


def _load_csv_gps_records(path: Path) -> list[GPSRecord]:
    """CSV(헤더 있는) GPS 로그를 읽어 상대 시각(t_s)을 채운 GPSRecord 목록을 만든다.

    Read a header-based GPS CSV into ``GPSRecord``s. 첫 유효 timestamp를 기준으로 t_s(초)를
    계산하며, timestamp가 없으면 인덱스를 초로 사용한다. Relative t_s comes from the first
    valid timestamp; falls back to row index when timestamps are absent.
    """
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

    # 2-패스: 먼저 레코드를 모은 뒤 첫 유효 timestamp 대비 경과초로 t_s를 다시 채운다.
    # Second pass: rewrite t_s as seconds elapsed from the first valid timestamp.
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
    """USBDBG 디버그 라인에서 GPS 필드를 파싱한다(파서는 tools/analyze_gps_log, 지연 import).

    Parse GPS fields from USBDBG lines. t_s는 실제 타임스탬프가 없어 인덱스×sample_period_s로
    합성한다. USBDBG lines carry no clock, so t_s is synthesized as index × sample period.
    파싱 실패 라인은 조용히 건너뛴다 / unparseable lines are skipped.
    """
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
    """Serial3 `key=value` 로그를 파싱한다(status=FIX 이면 유효, t_s는 인덱스).

    Parse Serial3 ``key=value`` GPS lines; ``status=FIX`` marks a valid fix, t_s is the
    row index. Lines missing required keys are skipped.
    """
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
    """한 파일을 포맷에 맞게 로드해 GPSDataset을 반환(레코드가 없으면 None).

    Load one GPS file, choosing the format by suffix. .csv는 CSV 로더를, 그 외는 USBDBG와
    Serial3 두 파서를 모두 시도해 레코드가 더 많은 쪽을 채택한다. For non-CSV files both the
    USBDBG and Serial3 parsers run and the one yielding more records wins.
    반환 데이터는 항상 실측(is_mock=False)으로 표시된다.
    """
    if path.suffix.lower() == ".csv":
        records = _load_csv_gps_records(path)
        kind = "GPS CSV"
    else:
        # 로그 종류를 미리 알 수 없어 두 파서를 돌리고 더 많이 파싱된 쪽을 채택.
        # We don't know the log type up front: run both parsers, keep the richer result.
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
    """여러 GPS 파일(또는 기본 패턴 탐색 결과)을 로드해 유효한 것만 목록으로 반환.

    Load multiple GPS files (or discovered defaults); skip files that yield no records.
    """
    candidate_paths = list(paths) if paths else discover_paths(GPS_LOG_PATTERNS)
    datasets: list[GPSDataset] = []
    for path in candidate_paths:
        dataset = load_gps_dataset(path)
        if dataset is not None:
            datasets.append(dataset)
    return datasets


def mock_gps_dataset() -> GPSDataset:
    """실측 로그가 없을 때 쓰는 결정론적 GPS 데모 데이터셋을 만든다(is_mock=True).

    Build a deterministic GPS demo dataset for when no real log is available.
    앞 10샘플은 fix 없음(획득 중)으로 두고 이후 sin/cos로 좌표·위성·HDOP를 흉내낸다.
    First 10 samples have no fix (acquisition); later values are sin/cos-synthesized.
    """
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


# ── 안전(Safety) 데이터 로딩 / Safety data loading ──


def _load_usbdbg_safety_records(path: Path, sample_period_s: float) -> list[SafetyRecord]:
    """USBDBG 라인에서 안전/모드/명령 필드를 파싱한다(파서는 tools/analyze_safety_log, 지연 import).

    Parse safety/mode/command fields from USBDBG lines. GPS 로더와 마찬가지로 t_s는
    인덱스×sample_period_s로 합성하고, 파싱 실패 라인은 건너뛴다.
    As with the GPS loader, t_s is synthesized from the index; bad lines are skipped.
    """
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
    """한 USBDBG 안전 로그를 로드해 SafetyDataset을 반환(레코드 없으면 None, is_mock=False).

    Load one USBDBG safety log into a ``SafetyDataset`` (real data); ``None`` if empty.
    """
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
    """여러 안전 로그(또는 기본 패턴 탐색 결과)를 로드해 유효한 것만 반환.

    Load multiple safety logs (or discovered defaults); skip empty ones.
    """
    candidate_paths = list(paths) if paths else discover_paths(SAFETY_LOG_PATTERNS)
    datasets: list[SafetyDataset] = []
    for path in candidate_paths:
        dataset = load_safety_dataset(path)
        if dataset is not None:
            datasets.append(dataset)
    return datasets


def mock_safety_dataset() -> SafetyDataset:
    """FAILSAFE→MANUAL→AUTO_READY→MANUAL 시나리오의 결정론적 안전 데모 데이터셋(is_mock=True).

    Build a deterministic safety demo dataset walking through
    FAILSAFE → MANUAL → AUTO_READY → MANUAL. left/right 모터 명령은 STOP이면 0,
    아니면 throttle±steer를 [-1,1]로 클램프해 산출한다(실제 로직 근사).
    Motor commands are 0 in STOP, else throttle±steer clamped to [-1, 1].
    """
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


# ── 웨이포인트/미션 데이터 로딩 / Waypoint & mission data loading ──


def _read_waypoints_csv(path: Path) -> list[dict[str, float | int]]:
    """웨이포인트 CSV를 읽어 order 기준으로 정렬된 dict 목록을 반환(필수 컬럼 없으면 그 행 무시).

    Read a waypoint CSV into dicts sorted by ``order``; rows missing required columns
    are skipped.
    """
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
    """웨이포인트 JSON을 읽어 order 기준 정렬된 dict 목록을 반환(CSV 판과 동일 스키마).

    Read a waypoint JSON list into dicts sorted by ``order`` (same schema as the CSV reader).
    """
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
    """레인별 첫 점 사이의 평균 거리로 레인 간격(m)을 추정(레인이 2개 미만이면 0).

    Estimate lane spacing (m) as the mean distance between consecutive lanes' first
    points; returns 0 when fewer than two lanes exist.
    """
    first_by_lane: dict[int, tuple[float, float]] = {}
    for waypoint in waypoints:
        lane = int(waypoint["lane"])
        # 각 레인의 대표점은 처음 등장한 점으로 고정 / one representative point per lane
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
    """미션 웨이포인트를 로드한다: 실제 파일이 있으면 그것을, 없으면 잔디깎기 경로를 합성한다.

    Load mission waypoints. 파일(CSV/JSON)에서 2개 이상 읽히면 그 첫 두 점을 A/B로 삼고,
    실패하면 ``generate_lawnmower_path``로 결정론적 A/B 데모 미션을 만든다. Uses a real file
    when ≥2 waypoints parse, otherwise synthesizes a deterministic lawnmower demo mission.
    항상 is_mock=True(실측 미션 로그가 아님) / always flagged mock.
    """
    from gps_coverage_core.planner import generate_lawnmower_path

    candidate_paths = list(paths) if paths else discover_paths(WAYPOINT_PATTERNS)
    for path in candidate_paths:
        try:
            waypoints = _read_waypoints_csv(path) if path.suffix == ".csv" else _read_waypoints_json(path)
        except (OSError, ValueError, KeyError):
            # 손상/스키마 불일치 파일은 건너뛰고 다음 후보로 / skip unreadable/mismatched files
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

    # 후보 파일이 없거나 모두 부적합할 때의 폴백: 고정 A/B에서 잔디깎기 경로 합성.
    # Fallback when no usable file: synthesize a lawnmower path from fixed A/B endpoints.
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


# ── 좌표 변환 & 시계열 유틸 / Coordinate transforms and series utilities ──


def fixed_xy_records(records: Sequence[GPSRecord]) -> list[GPSRecord]:
    """fix가 유효하고 위경도가 모두 있는 레코드만 골라 반환.

    Keep only records with a valid fix and non-null lat/lon.
    """
    return [
        record
        for record in records
        if record.fix_valid and record.lat is not None and record.lon is not None
    ]


def gps_local_xy(records: Sequence[GPSRecord]) -> tuple[list[float], list[float]]:
    """유효 GPS 레코드를 첫 점을 원점으로 하는 로컬 (x, y) 미터 좌표로 변환.

    Convert valid GPS fixes to local metric (x, y) with the first fix as origin, via
    ``latlon_to_xy``. 유효 레코드가 없으면 빈 리스트 쌍을 반환 / empty pair if none.
    """
    from gps_coverage_core.planner import latlon_to_xy

    fixed = fixed_xy_records(records)
    if not fixed:
        return [], []
    # 첫 유효 fix를 로컬 좌표 원점으로 사용 / first fix defines the origin
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
    """연속으로 같은 값이 이어지는 구간을 (값, 시작index, 끝index) 목록으로 압축.

    Compress a categorical series into runs of equal values as
    ``(value, start_index, end_index)`` (inclusive). 상태 타임라인의 색 띠 그리기에 쓰인다.
    Used to draw colored spans for state timelines.
    """
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


# ── 캡션 문서 생성 / Caption document generation ──


def write_figure_captions(results: Sequence[FigureResult]) -> Path:
    """모든 FigureResult를 모아 ``docs/figures/generated/figure_captions.md``를 작성하고 경로 반환.

    Write the aggregated caption markdown from all ``FigureResult``s and return its path.
    부수효과 / Side effect: 파일 덮어쓰기(overwrite). 각 그림의 출처·용도·캡션을 mock/스키매틱
    표기와 함께 기록한다. Records source/use/caption per figure, keeping mock labeling intact.
    """
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


# ── import 부수효과 / Import-time side effect ──
# 이 모듈을 import 하는 즉시 보고서 스타일을 적용해 모든 그림 스크립트가 일관된 룩을 갖는다.
# Applying the report style on import so every figure script shares one look.
set_report_style()
