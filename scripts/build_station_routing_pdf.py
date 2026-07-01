#!/usr/bin/env python3
"""Render the station routing/drive code guide as a diagram-heavy PDF.

Pure matplotlib (already a project dependency), so no extra tooling is needed.
Korean text uses WenQuanYi Zen Hei with a Unifont fallback for the few glyphs
it lacks. Output: docs/station_routing_code_guide.pdf.

    uv run python scripts/build_station_routing_pdf.py
"""
from __future__ import annotations

from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib import font_manager as fm
from matplotlib.backends.backend_pdf import PdfPages
from matplotlib.patches import FancyBboxPatch, FancyArrowPatch

# --- Fonts: register Korean-capable faces and enable per-glyph fallback -------
_FONT_CANDIDATES = [
    "/usr/share/fonts/truetype/wqy/wqy-zenhei.ttc",
    "/usr/share/fonts/opentype/unifont/unifont.otf",
]
for _p in _FONT_CANDIDATES:
    if Path(_p).exists():
        try:
            fm.fontManager.addfont(_p)
        except Exception:
            pass
plt.rcParams["font.family"] = "sans-serif"
plt.rcParams["font.sans-serif"] = ["WenQuanYi Zen Hei", "Unifont", "DejaVu Sans"]
plt.rcParams["axes.unicode_minus"] = False

# --- Palette ------------------------------------------------------------------
INK = "#0f172a"
MUTED = "#475569"
PC = "#2563eb"
PC_BG = "#dbeafe"
ROV = "#059669"
ROV_BG = "#d1fae5"
ROUTE = "#7c3aed"
ROUTE_BG = "#ede9fe"
WARN = "#b45309"
WARN_BG = "#fef3c7"
LINE = "#334155"
CARD = "#f1f5f9"
A4 = (8.27, 11.69)


def new_page():
    fig = plt.figure(figsize=A4)
    ax = fig.add_axes([0, 0, 1, 1])
    ax.set_xlim(0, 100)
    ax.set_ylim(0, 100)
    ax.axis("off")
    return fig, ax


def box(ax, x, y, w, h, *, fc="white", ec=LINE, lw=1.4, rounding=1.6, alpha=1.0, z=1):
    p = FancyBboxPatch(
        (x, y), w, h,
        boxstyle=f"round,pad=0,rounding_size={rounding}",
        linewidth=lw, edgecolor=ec, facecolor=fc, alpha=alpha, zorder=z,
        mutation_aspect=1.0,
    )
    ax.add_patch(p)


def text(ax, x, y, s, *, size=9.5, color=INK, weight="normal", ha="left", va="top",
         mono=False, z=3, style="normal"):
    # Monospace lines keep a CJK fallback so Korean mixed into a command string
    # still renders (ASCII stays fixed-width via DejaVu Sans Mono).
    fam = ["DejaVu Sans Mono", "WenQuanYi Zen Hei", "Unifont"] if mono else None
    ax.text(x, y, s, fontsize=size, color=color, fontweight=weight, ha=ha, va=va,
            family=fam, zorder=z, style=style)


def arrow(ax, x1, y1, x2, y2, *, color=LINE, lw=2.0, style="-|>", z=2, mut=16, rad=0.0):
    ax.add_patch(FancyArrowPatch(
        (x1, y1), (x2, y2), arrowstyle=style, mutation_scale=mut,
        linewidth=lw, color=color, zorder=z,
        connectionstyle=f"arc3,rad={rad}"))


def header(ax, title, subtitle=None):
    box(ax, 0, 92, 100, 8, fc=INK, ec=INK, rounding=0.0)
    text(ax, 6, 96.6, title, size=15, color="white", weight="bold", va="center")
    if subtitle:
        text(ax, 94, 96.6, subtitle, size=8.5, color="#cbd5e1", ha="right", va="center")


def footer(ax, page):
    text(ax, 50, 2.2, "GPS·HC-12 로버 · 스테이션 라우팅/구동 코드 가이드",
         size=7.5, color=MUTED, ha="center")
    text(ax, 94, 2.2, f"{page}/6", size=7.5, color=MUTED, ha="right")


def chip(ax, x, y, label, color, bg):
    w = 2.1 + len(label) * 1.65
    box(ax, x, y, w, 3.0, fc=bg, ec=color, lw=1.1, rounding=1.4)
    text(ax, x + w / 2, y + 1.5, label, size=8.5, color=color, weight="bold",
         ha="center", va="center")
    return w


# =============================================================================
# PAGE 1 — Cover + architecture
# =============================================================================
def page1(pdf):
    fig, ax = new_page()
    box(ax, 0, 88, 100, 12, fc=INK, ec=INK, rounding=0.0)
    text(ax, 6, 95.4, "스테이션(PC) 라우팅 · 구동 코드 가이드", size=19,
         color="white", weight="bold", va="center")
    text(ax, 6, 90.6, "github.com/seonukkim/gps_hc12_robot   ·   OpenRB-150 로버 + HC-12 + GPS/IMU",
         size=9, color="#cbd5e1", va="center")

    # One-line summary
    box(ax, 5, 78.3, 90, 8.0, fc=ROUTE_BG, ec=ROUTE, lw=1.4)
    text(ax, 8, 85.1, "한 줄 요약", size=9, color=ROUTE, weight="bold")
    text(ax, 8, 82.6,
         "로버 코드는 .ino 하나. PC의 ‘라우팅 파이썬’은 tools/physical_path_planning/ (실행) +",
         size=9, color=INK)
    text(ax, 8, 80.7,
         "gps_coverage_core/planner.py (경로 계산). 둘은 USB 시리얼 한 줄로 연결된다.",
         size=9, color=INK)

    # Architecture title
    text(ax, 5, 75.5, "전체 구조 — 누가 무엇을 실행하고 어떻게 연결되나", size=12,
         color=INK, weight="bold")

    # PC box
    box(ax, 4, 40, 40, 32, fc=PC_BG, ec=PC, lw=1.8)
    text(ax, 24, 69.4, "스테이션 PC (Mac)", size=11, color=PC, weight="bold", ha="center")
    box(ax, 7, 57.5, 34, 8.4, fc="white", ec=PC, lw=1.1)
    text(ax, 9, 63.9, "tools/physical_path_planning/", size=8.6, color=INK, weight="bold")
    text(ax, 9, 61.4, "진입: cli.py  ·  실행·보정·안전", size=8.2, color=MUTED)
    text(ax, 9, 59.2, "run_physical_path_planner.sh 로 호출", size=8.2, color=MUTED)
    box(ax, 7, 48.2, 34, 7.6, fc="white", ec=ROUTE, lw=1.1)
    text(ax, 9, 54.1, "gps_coverage_core/", size=8.6, color=ROUTE, weight="bold")
    text(ax, 9, 51.6, "planner.py = 경로 계산(라우팅)", size=8.2, color=MUTED)
    text(ax, 9, 49.6, "protocol.py = 명령 프레임", size=8.2, color=MUTED)
    text(ax, 24, 44.0, "파이썬 3.12 · pyserial · numpy/pyproj", size=7.8, color=PC, ha="center")
    text(ax, 24, 41.8, "matplotlib(미리보기 PNG)", size=7.8, color=PC, ha="center")

    # Rover box
    box(ax, 56, 40, 40, 32, fc=ROV_BG, ec=ROV, lw=1.8)
    text(ax, 76, 69.4, "로버: OpenRB-150", size=11, color=ROV, weight="bold", ha="center")
    text(ax, 76, 66.4, "firmware/openrb_robot_controller/*.ino", size=7.8, color=ROV, ha="center")
    for i, (t, d) in enumerate([
        ("모터 2개 (좌/우 ESC)", "명령 실행"),
        ("GPS (NMEA)", "위치"),
        ("IMU BMI160", "상대 yaw(방위)"),
        ("RC 수신기 (PPM)", "CH1조향·CH2스로틀·CH5모드"),
        ("HC-12 (무선 UART)", "옵션"),
    ]):
        yy = 62.0 - i * 4.2
        box(ax, 59, yy, 34, 3.4, fc="white", ec=ROV, lw=0.9)
        text(ax, 61, yy + 1.7, t, size=8.0, color=INK, weight="bold", va="center")
        text(ax, 91.5, yy + 1.7, d, size=7.2, color=MUTED, ha="right", va="center")

    # USB link in the middle: arrows sit behind the white box (cable-in look),
    # short direction labels above/below, full protocol caption under the diagram.
    arrow(ax, 43.8, 57.2, 56.2, 57.2, color=PC, lw=2.2, z=0)
    arrow(ax, 56.2, 54.0, 43.8, 54.0, color=ROV, lw=2.2, z=0)
    box(ax, 45.5, 52.3, 9, 7.4, fc="white", ec=LINE, lw=1.2)
    text(ax, 50, 57.7, "USB", size=8.2, color=LINE, weight="bold", ha="center", va="center")
    text(ax, 50, 55.4, "시리얼", size=7.2, color=MUTED, ha="center", va="center")
    text(ax, 50, 53.5, "115200", size=6.6, color=MUTED, ha="center", va="center")
    text(ax, 50, 61.0, "명령 →", size=7.4, color=PC, weight="bold", ha="center")
    text(ax, 50, 50.7, "← 텔레메트리", size=7.4, color=ROV, weight="bold", ha="center")
    text(ax, 50, 38.6,
         "연결: USB 시리얼(/dev/ttyACM0 · 115200)   ·   PC→로버 명령(SET/STOP/ARM…)   "
         "·   로버→PC 텔레메트리(USBDBG…, ~0.5초/줄)",
         size=6.9, color=MUTED, ha="center", va="center")

    # Role split banner
    box(ax, 5, 30.5, 90, 6.4, fc=CARD, ec=LINE, lw=1.1)
    text(ax, 8, 35.2, "역할 분리", size=9, color=INK, weight="bold")
    text(ax, 8, 32.6,
         "두뇌(경로·판단) = 파이썬     |     안전 실행(모터·센서) = 펌웨어(.ino)",
         size=9.2, color=INK)

    # Two operating modes teaser
    box(ax, 5, 12.5, 44, 15.5, fc=PC_BG, ec=PC, lw=1.4)
    text(ax, 7.5, 25.6, "① 유선 — 두뇌가 PC", size=9.5, color=PC, weight="bold")
    text(ax, 7.5, 22.6, "PC가 실시간으로 저수준 구동 명령 전송.", size=8.4, color=INK)
    text(ax, 7.5, 20.3, "모드: auto-relative-run / run / execute-plan", size=8.2, color=MUTED)
    text(ax, 7.5, 18.0, "USB를 뽑으면 데드맨이 즉시 정지(설계).", size=8.2, color=MUTED)
    text(ax, 7.5, 15.0, "→ 상대가 말한 ‘PC 라우팅 코드’는 보통 이것", size=8.2, color=PC,
         weight="bold")

    box(ax, 51, 12.5, 44, 15.5, fc=ROV_BG, ec=ROV, lw=1.4)
    text(ax, 53.5, 25.6, "② 무선 — 두뇌가 로버", size=9.5, color=ROV, weight="bold")
    text(ax, 53.5, 22.6, "PC로 온보드 패턴 펌웨어를 1회 업로드만.", size=8.4, color=INK)
    text(ax, 53.5, 20.3, "이후 USB 없이 RC 송신기만으로 동작.", size=8.2, color=MUTED)
    text(ax, 53.5, 18.0, "모드: rc-auto-pattern (ㄹ자 왕복)", size=8.2, color=MUTED)
    text(ax, 53.5, 15.0, "→ PC 파이썬은 빌드·업로드 도구로만 사용", size=8.2, color=ROV,
         weight="bold")

    text(ax, 50, 8.2, "안전 원칙: 모든 실행 요약에 ready_for_full_path_following=false 강제 (checks.py)",
         size=8.0, color=WARN, ha="center")
    footer(ax, 1)
    pdf.savefig(fig)
    plt.close(fig)


# =============================================================================
# PAGE 2 — install + run entry + mode table
# =============================================================================
def page2(pdf):
    fig, ax = new_page()
    header(ax, "실행 진입점 & 모드", "설치 → 한 줄 명령으로 모든 모드 실행")

    box(ax, 5, 79, 90, 9.5, fc=CARD, ec=LINE, lw=1.1)
    text(ax, 8, 86.6, "1) 설치 (PC, 최초 1회)", size=10, color=INK, weight="bold")
    text(ax, 8, 83.6, "uv sync --extra dev        # 또는  pip install -r requirements.txt",
         size=8.6, color=INK, mono=True)
    text(ax, 8, 80.8, "펌웨어 업로드 모드는 arduino-cli + OpenRB-150:samd:OpenRB-150 보드 코어 필요",
         size=8.0, color=MUTED)

    box(ax, 5, 67.5, 90, 9, fc=PC_BG, ec=PC, lw=1.3)
    text(ax, 8, 74.6, "2) 모든 실행의 공통 진입점", size=10, color=PC, weight="bold")
    text(ax, 8, 71.4, "bash scripts/run_physical_path_planner.sh <모드> [옵션]",
         size=9.4, color=INK, mono=True, weight="bold")
    text(ax, 8, 68.6, "얇은 디스패처 → 실제 로직은 tools/physical_path_planning/cli.py 의 cmd_* 함수",
         size=8.0, color=MUTED)

    # Mode table
    text(ax, 5, 63.0, "주요 모드", size=11, color=INK, weight="bold")
    rows = [
        ("diagnose", "×", "연결·센서 확인(텔레메트리만 요약)"),
        ("gps-wait", "×", "쓸 만한 GPS 픽스 대기"),
        ("rc-input-diagnose", "×", "RC 수신기 채널 입력 진단"),
        ("manual-control", "RC", "PPM 수동 조작 펌웨어 업로드·모니터"),
        ("preview", "×", "ㄹ자 커버리지 경로 계획 + PNG 렌더"),
        ("inspect-plan", "×", "저장된 플랜/이미지 확인"),
        ("tune-motion", "●", "대화형 모터 캘리브레이션"),
        ("calibrate-turn", "●", "회전 각도 캘리브레이션(IMU yaw)"),
        ("calibration-check", "×", "캘리브레이션 완성도 점검"),
        ("align-heading", "●", "첫 레인 방향으로 로버 정렬"),
        ("run / execute-plan", "●", "경로를 유선으로 실행"),
        ("auto-relative-run", "●", "AUTO 스위치 대기 후 상대경로 1회 유선 실행"),
        ("rc-auto-pattern", "●", "무선 온보드 ㄹ자 패턴 펌웨어 업로드"),
    ]
    y0 = 58.8
    rh = 3.7
    box(ax, 5, y0 - rh * len(rows) + 0.2, 90, rh * len(rows) + 2.4, fc="white", ec=LINE, lw=1.1)
    text(ax, 8, y0 + 1.3, "모드", size=8.6, color=MUTED, weight="bold")
    text(ax, 35, y0 + 1.3, "모터", size=8.6, color=MUTED, weight="bold", ha="center")
    text(ax, 42, y0 + 1.3, "역할", size=8.6, color=MUTED, weight="bold")
    for i, (m, mv, d) in enumerate(rows):
        yy = y0 - i * rh - 1.0
        if i % 2 == 0:
            box(ax, 6, yy - rh + 1.1, 88, rh, fc="#f8fafc", ec="#f8fafc", lw=0.1, rounding=0.4)
        col = ROV if mv == "●" else (PC if mv == "RC" else MUTED)
        text(ax, 8, yy, m, size=8.4, color=INK, weight="bold", mono=True, va="center")
        text(ax, 35, yy, mv, size=8.6, color=col, weight="bold", ha="center", va="center")
        text(ax, 42, yy, d, size=8.2, color=INK, va="center")
    text(ax, 8, 9.6, "● = 모터 구동  ·  RC = 수동 조작  ·  × = 모터 안 움직임",
         size=8.0, color=MUTED)
    text(ax, 8, 7.0, "산출물(트레이스 CSV·요약 JSON·PNG)은 outputs/ 아래에 저장(깃 커밋 안 함).",
         size=8.0, color=MUTED)
    footer(ax, 2)
    pdf.savefig(fig)
    plt.close(fig)


# =============================================================================
# PAGE 3 — file role map
# =============================================================================
def page3(pdf):
    fig, ax = new_page()
    header(ax, "파일별 역할 지도", "임포트 방향은 한 방향(리프 먼저)")

    text(ax, 5, 88.5,
         "geometry · calibration · telemetry · safety · checks  →  executor  →  controller  →  cli",
         size=8.4, color=MUTED, mono=True)

    # Group A: physical_path_planning
    box(ax, 4, 47.5, 92, 39, fc="#f8fafc", ec=PC, lw=1.6)
    text(ax, 7, 83.6, "tools/physical_path_planning/  —  PC 실행/구동의 본체", size=10.5,
         color=PC, weight="bold")
    ppp = [
        ("cli.py", "사용자 CLI·시리얼 오픈·펌웨어 빌드/업로드·요약(JSON) 기록 (가장 큼)"),
        ("geometry.py", "ㄹ자 경로 기하 생성. 코너를 회전→스텝→회전으로 분해"),
        ("calibration.py", "전진/후진/회전 캘리브레이션 값·회전 실제각(target_angle_deg)"),
        ("telemetry.py", "‘USBDBG …’ 라인 파싱 + GPS/IMU/RC/모터 필드 접근자"),
        ("executor.py", "시리얼에 명령 쓰기 · 가드 펄스 FSM(ARM→ACK→완료→STOP)"),
        ("controller.py", "경로 실행 감독 루프(stop_correct_go): 구동→정지→읽기→보정 반복"),
        ("alignment.py", "초기 헤딩 정렬(GPS 변위 프로브 + IMU 제자리 회전)"),
        ("tuning.py", "대화형 캘리브레이션 후보 조정·승인·백업/리셋"),
        ("preview.py", "계획 경로를 PNG로 렌더(현장에서 눈으로 확인)"),
        ("safety.py / checks.py", "ACK/STOP·출력0 확인 · ready_for_full_path_following=false 강제"),
    ]
    for i, (f, d) in enumerate(ppp):
        yy = 79.4 - i * 3.15
        text(ax, 8, yy, f, size=8.3, color=INK, weight="bold", mono=True, va="center")
        text(ax, 34, yy, d, size=8.0, color=MUTED, va="center")

    # Group B: gps_coverage_core
    box(ax, 4, 17.5, 45, 27.5, fc=ROUTE_BG, ec=ROUTE, lw=1.6)
    text(ax, 7, 42.0, "gps_coverage_core/", size=10, color=ROUTE, weight="bold")
    text(ax, 7, 39.4, "순수 경로 계산(하드웨어 비의존)", size=8.0, color=MUTED)
    core = [
        ("planner.py ★", "위경도↔미터, 레인 오프셋, 왕복 웨이포인트"),
        ("protocol.py", "PC↔로버 명령 프레임·XOR 체크섬·클램프"),
        ("geo.py", "거리·방위 계산"),
        ("nmea.py", "GPS NMEA 파싱"),
        ("imu.py", "IMU 계산 헬퍼"),
        ("side_tool_planner.py", "보조 도구 경로"),
    ]
    for i, (f, d) in enumerate(core):
        yy = 36.4 - i * 3.05
        text(ax, 7, yy, f, size=8.0, color=(ROUTE if "★" in f else INK),
             weight="bold", mono=True, va="center")
        text(ax, 7, yy - 1.45, d, size=7.3, color=MUTED, va="center")

    # Group C: firmware + ros2
    box(ax, 51, 30, 45, 15, fc=ROV_BG, ec=ROV, lw=1.6)
    text(ax, 54, 42.0, "firmware/…/openrb_robot_controller.ino", size=8.8,
         color=ROV, weight="bold")
    text(ax, 54, 39.2, "로버의 유일한 컨트롤러. 하나의 파일을", size=8.0, color=INK)
    text(ax, 54, 37.0, "컴파일 -D 플래그로 여러 모드로 빌드.", size=8.0, color=INK)
    text(ax, 54, 34.4, "PPM 디코드·GPS·IMU·모터·안전게이트", size=7.6, color=MUTED)
    text(ax, 54, 32.2, "USBDBG 텔레메트리 회신", size=7.6, color=MUTED)

    box(ax, 51, 16.0, 45, 11.5, fc=CARD, ec=MUTED, lw=1.2)
    text(ax, 54, 25.2, "ros2_ws/  (현재 스켈레톤, 미사용)", size=8.6, color=MUTED,
         weight="bold")
    text(ax, 54, 22.6, "coverage_planner / hc12_bridge /", size=7.8, color=MUTED)
    text(ax, 54, 20.6, "station_mission / waypoint_follower 노드", size=7.8, color=MUTED)
    text(ax, 54, 18.4, "→ 지금 전달할 라우팅 코드엔 불포함", size=7.8, color=WARN)

    text(ax, 8, 13.2, "firmware/*_probe, *_test 폴더는 GPS/IMU/PPM 배선 점검용 일회성 진단 스케치(운용과 무관).",
         size=8.0, color=MUTED)
    footer(ax, 3)
    pdf.savefig(fig)
    plt.close(fig)


# =============================================================================
# PAGE 4 — .py <-> .ino connection
# =============================================================================
def page4(pdf):
    fig, ax = new_page()
    header(ax, "파이썬 ↔ 아두이노 연결", "① 빌드타임  +  ② 런타임 시리얼 프로토콜")

    # Build-time
    box(ax, 5, 76, 90, 12, fc=WARN_BG, ec=WARN, lw=1.4)
    text(ax, 8, 85.6, "① 빌드타임 — 파이썬이 펌웨어를 빌드·업로드", size=10.5, color=WARN,
         weight="bold")
    text(ax, 8, 82.4, "cli.py  →  arduino-cli compile/upload  (같은 .ino, 모드별 -D 플래그)",
         size=8.8, color=INK, mono=True)
    text(ax, 8, 79.8, "예) 무선 패턴: -DRC_AUTO_PATTERN=1 -DMANUAL_CONTROL_PPM=1 -DIMU_ENABLE=1 …",
         size=8.2, color=MUTED, mono=True)
    text(ax, 8, 77.4, "‘어떤 파이썬 모드로 올렸나’ = ‘로버가 어떤 펌웨어 모드로 도나’",
         size=8.2, color=INK)

    # Run-time sequence
    text(ax, 5, 71.5, "② 런타임 — USB 시리얼 문자열 (/dev/ttyACM0 · 115200)", size=10.5,
         color=INK, weight="bold")
    # two lanes
    box(ax, 6, 46, 30, 22, fc=PC_BG, ec=PC, lw=1.5)
    text(ax, 21, 65.6, "PC (파이썬)", size=9.5, color=PC, weight="bold", ha="center")
    text(ax, 21, 63.0, "executor.write_command()", size=7.4, color=MUTED, ha="center", mono=True)
    box(ax, 64, 46, 30, 22, fc=ROV_BG, ec=ROV, lw=1.5)
    text(ax, 79, 65.6, "로버 (.ino)", size=9.5, color=ROV, weight="bold", ha="center")
    text(ax, 79, 63.0, "한 글자씩 읽어 \\n에서 해석", size=7.4, color=MUTED, ha="center")

    arrow(ax, 36, 60.5, 64, 60.5, color=PC, lw=2.0)
    text(ax, 50, 61.6, "명령", size=8.0, color=PC, ha="center")
    text(ax, 50, 58.9, "USB_DRIVE_LIVE_SET a=.. b=..", size=7.2, color=INK, ha="center", mono=True)

    arrow(ax, 64, 52.5, 36, 52.5, color=ROV, lw=2.0)
    text(ax, 50, 53.6, "텔레메트리 (0.5초/줄)", size=8.0, color=ROV, ha="center")
    text(ax, 50, 50.9, "USBDBG key=value …", size=7.2, color=INK, ha="center", mono=True)

    # Command table
    box(ax, 5, 24.5, 90, 19.5, fc="white", ec=LINE, lw=1.2)
    text(ax, 8, 41.6, "PC → 로버  (개행 종료 ASCII)", size=9.2, color=PC, weight="bold")
    cmds = [
        ("USB_DRIVE_LIVE_SET seq=N a=<전후진> b=<조향> ms=.. ttl=..", "연속 구동 setpoint(핵심)"),
        ("USB_DRIVE_LIVE_STOP seq=N", "연속 구동 정지"),
        ("USB_PULSE_TEST_ARM/…_STOP seq=N", "가드 펄스 1회"),
        ("STOP", "즉시 정지"),
    ]
    for i, (c, d) in enumerate(cmds):
        yy = 38.2 - i * 2.7
        text(ax, 8, yy, c, size=7.7, color=INK, mono=True, va="center")
        text(ax, 66, yy, d, size=7.7, color=MUTED, va="center")
    text(ax, 8, 27.0, "a = physical A(전/후진), b = physical B(좌/우 조향), 각각 [-1,1] 클램프 → 좌우 바퀴로 믹싱",
         size=7.6, color=MUTED)

    box(ax, 5, 7.5, 90, 15, fc="white", ec=LINE, lw=1.2)
    text(ax, 8, 20.4, "로버 → PC  (USBDBG 한 줄, 공백 구분 key=value)", size=9.2,
         color=ROV, weight="bold")
    text(ax, 8, 17.6,
         "USBDBG mode=AUTO_RUNNING event=ACK rc_ok=true auto_sw=true",
         size=7.6, color=INK, mono=True)
    text(ax, 8, 15.4,
         "  gps_sats=7 gps_hdop=1.2 imu_relative_yaw_deg=172.8",
         size=7.6, color=INK, mono=True)
    text(ax, 8, 13.2,
         "  final_left_cmd=0.31 final_right_cmd=0.29 physical_output_active=true …",
         size=7.6, color=INK, mono=True)
    text(ax, 8, 10.4,
         "telemetry.parse_usbdbg_rows()로 dict 파싱 · event=ARM/ACK/STOP는 가드 펄스 상태 확인용",
         size=7.6, color=MUTED)
    footer(ax, 4)
    pdf.savefig(fig)
    plt.close(fig)


# =============================================================================
# PAGE 5 — run flow (tethered), step cards
# =============================================================================
def page5(pdf):
    fig, ax = new_page()
    header(ax, "실행 순서 (유선, 차례대로)", "로버 USB 연결 상태에서")

    steps = [
        ("1", "연결·센서 확인 (모터 X)",
         "bash scripts/run_physical_path_planner.sh diagnose \\\n  --out-dir outputs/physical_path_planning/diagnose",
         "USBDBG가 올라오고 GPS/IMU/RC 필드가 보이는지 확인.", PC),
        ("2", "GPS 픽스 대기 (모터 X)",
         "bash scripts/run_physical_path_planner.sh gps-wait \\\n  --timeout-s 300 --min-sats 5 --max-hdop 2.5 \\\n  --out-dir outputs/.../gps_wait",
         "gps_sats≥5 · hdop가 임계값 이하로 안정될 때까지.", PC),
        ("3", "경로 미리보기 → PNG (모터 X)",
         "bash scripts/run_physical_path_planner.sh preview \\\n  --goal-mode relative_enu --goal-east-m 1.8 --goal-north-m 1.8 \\\n  --workspace-width-m 1.8 --step-spacing-m 0.6 --out-dir outputs/.../preview",
         "경로를 눈으로 먼저 확인(preview_*.png).", ROUTE),
        ("4", "(필요 시) 모터 캘리브레이션",
         "bash scripts/run_physical_path_planner.sh tune-motion \\\n  --primitive forward --out-dir outputs/.../tune_forward",
         "전진/후진/회전 각각. 회전은 calibrate-turn.", ROV),
        ("5", "첫 레인 방향 정렬",
         "bash scripts/run_physical_path_planner.sh align-heading \\\n  --out-dir outputs/.../align",
         "GPS 변위 프로브 + IMU 제자리 회전으로 정렬.", ROV),
        ("6", "유선 실행 — 상대경로 1회",
         "bash scripts/run_physical_path_planner.sh auto-relative-run \\\n  --goal-east-m 1.8 --goal-north-m 1.8 \\\n  --workspace-width-m 1.8 --step-spacing-m 0.6 --out-dir outputs/.../auto_run",
         "RC를 AUTO로 넘기면 현재 GPS를 원점으로 시작.", ROV),
    ]
    text(ax, 50, 90.3,
         "무선으로 쓰려면 위 대신 rc-auto-pattern 으로 1회 업로드 후 USB를 뽑고 RC 송신기로 운용",
         size=8.0, color=ROV, ha="center", va="center")
    # Note goes ABOVE the command block so the command lines are the last element
    # and cannot collide with the card's bottom border.
    y = 87.6
    for num, title_, cmd, note, col in steps:
        n_lines = cmd.count("\n") + 1
        h = 7.0 + n_lines * 2.3
        box(ax, 5, y - h, 90, h, fc=CARD, ec=col, lw=1.4)
        box(ax, 5, y - h, 6.5, h, fc=col, ec=col, rounding=1.6)
        text(ax, 8.25, y - h / 2, num, size=15, color="white", weight="bold",
             ha="center", va="center")
        text(ax, 14, y - 2.7, title_, size=9.5, color=INK, weight="bold")
        text(ax, 14, y - 4.8, "· " + note, size=7.7, color=MUTED)
        cy = y - 7.0
        for ln in cmd.split("\n"):
            text(ax, 14, cy, ln, size=7.5, color="#1e293b", mono=True)
            cy -= 2.3
        y -= h + 1.3
    footer(ax, 5)
    pdf.savefig(fig)
    plt.close(fig)


# =============================================================================
# PAGE 6 — routing-only files + refs
# =============================================================================
def page6(pdf):
    fig, ax = new_page()
    header(ax, "라우팅 코드만 원한다면 · 참고", "상대의 요구에 맞춰 짚어줄 곳")

    text(ax, 5, 88.0, "A. 순수 ‘경로 계산 로직’을 원하면 → 이 3파일 (우선순위 순)", size=11,
         color=INK, weight="bold")
    cards = [
        ("1", "gps_coverage_core/planner.py", ROUTE, ROUTE_BG,
         ["위경도 ↔ 로컬 x/y(미터) 변환", "스위프 폭 → 레인 오프셋 계산",
          "왕복(serpentine) 웨이포인트 생성", "= 라우팅 알고리즘의 심장"]),
        ("2", "tools/physical_path_planning/geometry.py", PC, PC_BG,
         ["그 경로를 로버가 밟을 수 있는", "ㄹ자 세그먼트로 분해",
          "(직진 / 코너 회전 / 스텝)", "각 세그먼트에 목표 바디 헤딩 부여"]),
        ("3", "tools/physical_path_planning/controller.py", ROV, ROV_BG,
         ["그 세그먼트를 GPS/IMU 피드백으로", "보정하며 실행하는 감독 루프",
          "stop_correct_go:", "구동→정지→읽기→보정 반복"]),
    ]
    for i, (n, f, col, bg, lines) in enumerate(cards):
        x = 5 + i * 30.7
        box(ax, x, 62.5, 29, 22.5, fc=bg, ec=col, lw=1.5)
        box(ax, x + 1.6, 80.8, 5, 3.4, fc=col, ec=col, rounding=1.2)
        text(ax, x + 4.1, 82.5, n, size=12, color="white", weight="bold", ha="center", va="center")
        # filename prominent, package path muted underneath (both fit the 29-wide card)
        fdir, _, fname = f.rpartition("/")
        text(ax, x + 1.6, 78.6, fname, size=8.4, color=col, weight="bold", mono=True)
        text(ax, x + 1.6, 76.4, fdir + "/", size=6.1, color=MUTED, mono=True)
        for j, ln in enumerate(lines):
            text(ax, x + 1.6, 73.0 - j * 2.7, ln, size=7.5, color=INK)

    box(ax, 5, 55.5, 90, 4.5, fc=CARD, ec=LINE, lw=1.1)
    text(ax, 8, 57.7,
         "B. ‘PC가 로버를 어떻게 움직이나(구동 인터페이스)’를 원하면  →  executor.py + gps_coverage_core/protocol.py",
         size=8.6, color=INK, va="center")

    # data flow recap strip
    text(ax, 5, 50.5, "데이터 흐름 요약", size=11, color=INK, weight="bold")
    flow = [("현장 GPS/목표", PC_BG, PC), ("planner.py\n웨이포인트", ROUTE_BG, ROUTE),
            ("geometry.py\nㄹ자 세그먼트", PC_BG, PC), ("controller.py\n보정 실행", ROV_BG, ROV),
            ("executor.py\nSET 명령", ROV_BG, ROV), ("로버 .ino\n모터", ROV_BG, ROV)]
    x = 5
    for i, (lab, bg, col) in enumerate(flow):
        w = 13.5
        box(ax, x, 42.5, w, 6.2, fc=bg, ec=col, lw=1.2)
        for j, ln in enumerate(lab.split("\n")):
            text(ax, x + w / 2, 46.6 - j * 2.2, ln, size=7.2, color=col, weight="bold",
                 ha="center", va="center")
        if i < len(flow) - 1:
            arrow(ax, x + w + 0.3, 45.6, x + w + 1.9, 45.6, color=LINE, lw=1.4, mut=10)
        x += w + 2.2

    # references
    text(ax, 5, 36.5, "참고 문서 (docs/)", size=11, color=INK, weight="bold")
    refs = [
        ("README_physical_path_planning.md", "현장 실행 런북(유선/무선)"),
        ("physical_path_planning_architecture.md", "모듈 아키텍처"),
        ("station_routing_code_guide.md", "이 PDF의 원본 텍스트(전체 상세)"),
        ("claude_collaboration_guide.md", "제어 의미론·트러블슈팅 결정 트리"),
        ("protocol.md · wiring.md · rc_channel_map.md", "프로토콜·배선·RC 채널"),
    ]
    box(ax, 5, 14.5, 90, 19.5, fc="white", ec=LINE, lw=1.1)
    for i, (f, d) in enumerate(refs):
        yy = 31.5 - i * 3.4
        text(ax, 8, yy, f, size=8.3, color=INK, weight="bold", mono=True, va="center")
        text(ax, 60, yy, d, size=8.0, color=MUTED, va="center")

    box(ax, 5, 6.5, 90, 5.5, fc=WARN_BG, ec=WARN, lw=1.2)
    text(ax, 50, 9.2,
         "안전: 이 스택은 완전 자율이 아니라 감독·가드 구동. 실제 주행 전 preview로 경로 확인, "
         "GPS 픽스·캘리브레이션 먼저.",
         size=8.0, color=WARN, ha="center", va="center")
    footer(ax, 6)
    pdf.savefig(fig)
    plt.close(fig)


def main():
    out = Path("docs/station_routing_code_guide.pdf")
    out.parent.mkdir(parents=True, exist_ok=True)
    with PdfPages(out) as pdf:
        page1(pdf)
        page2(pdf)
        page3(pdf)
        page4(pdf)
        page5(pdf)
        page6(pdf)
        d = pdf.infodict()
        d["Title"] = "Station Routing/Drive Code Guide"
        d["Author"] = "gps_hc12_robot"
    print(f"wrote {out} ({out.stat().st_size/1024:.0f} KB)")


if __name__ == "__main__":
    main()
