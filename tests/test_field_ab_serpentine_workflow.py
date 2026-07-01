"""필드 A/B → 지그재그 경로 패키지 → 무동작 검증 전체 워크플로 통합 테스트.

목적/역할:
    현장 캡처점(A,B)에서 시작해 스테이션 측 가상 제어 프리뷰까지 이어지는
    "무동작(no-motion)" 파이프라인 전 구간을 CLI/함수 단위로 엮어 검증한다. 각
    스테이지가 만든 산출물(주로 `path_package.json`, CSV, summary.md)을 다음
    스테이지가 다시 읽어 올바르게 소비하는지가 핵심이다. 이 워크플로가 잠그는
    **최상위 안전 계약**은: 캡처→계획→검증→추적→가상제어 어디에서도 로버 모터를
    돌리지 않는다(모든 산출물에서 `motor_command_generated`/`physical_output_active`
    가 False).

    Integration tests for the end-to-end *no-motion* pipeline: field A/B capture →
    normalized rectangle → serpentine path package → outdoor no-motion validation
    → station-side target tracking → virtual control preview. Verifies each stage
    consumes the previous stage's artifacts (mostly `path_package.json`) correctly,
    and that the whole chain never generates a motor command.

시스템 내 위치 (파이프라인 스테이지 순서):
    1. `capture_field_ab_points` / `capture_georef_ab_points` — A/B 캡처(로컬 or 위경도).
    2. `field_ab_to_serpentine` — 정규화(A'=좌상단, B'=우하단) + 지그재그 경로 패키지 생성.
    3. `inspect_path_package` / `check_georef_path_package` — 패키지 정합성 검사.
    4. `path_no_motion_validation` — 야외 GPS/헤딩 준비도 게이트(무동작).
    5. `physical_path_preview_from_package` — 패키지에서 오프라인 미리보기 재생성.
    6. `station_path_package_tracker` (Stage 12) — 현재 포즈 대비 다음 타깃 계산.
    7. `station_virtual_path_controller` (Stage 14) — 가상 전진/회전 명령 프리뷰.
    보조: `analyze_integrated_dryrun_for_path_package`(펌웨어 드라이런 로그 분석),
    `check_path_no_motion_summary`(요약 판정), `path_no_motion_validation`(패키지 탐색).

핵심 개념·불변식(invariant):
    - **A'/B' 정규화 규약**: raw A/B 의 min/max 로 축 정렬 사각형을 만들고 A'=좌상단,
      B'=우하단으로 고정한다. 다수 테스트가 A'=(0,1.2), B'=(8,0)을 직접 단언한다.
    - **툴 안전**: sweep 트랙만 tool_active=True(청소), 커넥터/접근(approach)은
      tool_active=False. 프리미티브는 move/rotate 허용 집합 안에서만.
    - **무모터/무전송**: 라이브 시리얼 모드조차 포트가 안 열리면 명확히 실패하고
      트레이스백을 흘리지 않는다. compile_time 하드코딩 타깃이면 모터 준비를 막는다.
    - **위경도 지원**: georeference 가 있으면 lat/lon→로컬(등장방형 ENU) 변환이 되고,
      없으면 진단 행으로 강등된다(NO_GEOREFERENCE_FOR_LAT_LON_TO_LOCAL).

사용법/진입점:
    Pytest 파일. 대부분의 테스트는 각 도구의 `main([...])` 를 인자 배열로 호출하고
    `tmp_path` 에 쓰인 산출물을 읽어 단언한다. 일부는 `build_path_package` 등 순수
    함수를 직접 호출해 CLI 오버헤드 없이 계약을 검사한다.

리팩토링 노트:
    - `build_path_package(...)` 시그니처와 반환 패키지의 키 구조(summary/tool path/
      georeference)는 이 파일 전반에서 재사용되므로 사실상 계약이다.
    - georef dict 리터럴(여러 테스트에 반복 등장)은 실제 캡처 산출물 스키마를 흉내낸
      것이다; `field_ab_to_serpentine` 의 georef 스키마가 바뀌면 여기도 갱신 필요.

Pytest module: end-to-end no-motion workflow integration. Drives each tool's
`main([...])` and reads back `tmp_path` artifacts; a few tests call pure
functions (e.g. `build_path_package`) directly. The A'=top-left / B'=bottom-right
normalization and the "never emits motor commands" rule are the locked contracts.
"""

from __future__ import annotations

import csv
import json
from pathlib import Path

from tools import capture_field_ab_points
from tools import capture_georef_ab_points
from tools import check_georef_path_package
from tools import check_path_no_motion_summary
from tools import field_ab_to_serpentine
from tools import inspect_path_package
from tools import path_no_motion_validation
from tools import physical_path_preview_from_package
from tools import analyze_integrated_dryrun_for_path_package
from tools import station_path_package_tracker
from tools import station_virtual_path_controller


# ── 1. A/B 캡처 & 정규화 / Stage 1: A/B capture & normalization ──


def test_capture_field_ab_points_writes_manual_outputs(tmp_path: Path) -> None:
    """수동 A/B 좌표 캡처가 JSON/CSV 로 저장되고 값·라벨·무모터 플래그가 맞는지 확인.

    Manual A/B capture writes field_points.{json,csv} with the given coords, A/B
    labels, and motor_command_generated False.
    """
    assert capture_field_ab_points.main(
        [
            "--a-x",
            "2",
            "--a-y",
            "0",
            "--b-x",
            "10",
            "--b-y",
            "1.2",
            "--out-dir",
            str(tmp_path),
        ]
    ) == 0

    data = json.loads((tmp_path / "field_points.json").read_text(encoding="utf-8"))
    assert data["points"]["A"]["x_m"] == 2.0
    assert data["points"]["B"]["y_m"] == 1.2
    assert data["motor_command_generated"] is False
    with (tmp_path / "field_points.csv").open(encoding="utf-8", newline="") as handle:
        rows = list(csv.DictReader(handle))
    assert [row["point_label"] for row in rows] == ["A", "B"]


def test_ab_normalization_produces_a_prime_top_left_and_b_prime_bottom_right() -> None:
    """A/B 정규화가 축 정렬 사각형과 A'=좌상단·B'=우하단 규약을 산출하는지 검증.

    입력 A=(8,0),B=(0,1.2)를 min/max 로 정규화하면 사각형 x:0..8, y:0..1.2 이고
    A'=(0,1.2), B'=(8,0)이 됨을 확인(코어 플래너가 요구하는 코너 규약).

    Normalizing raw A/B yields the axis-aligned rectangle plus the required
    A'=top-left / B'=bottom-right corner assignment.
    """
    workspace = field_ab_to_serpentine.normalize_field_ab(a_x=8.0, a_y=0.0, b_x=0.0, b_y=1.2)

    assert workspace["x_min_m"] == 0.0
    assert workspace["x_max_m"] == 8.0
    assert workspace["y_min_m"] == 0.0
    assert workspace["y_max_m"] == 1.2
    assert workspace["A_prime_top_left"] == {"x_m": 0.0, "y_m": 1.2}
    assert workspace["B_prime_bottom_right"] == {"x_m": 8.0, "y_m": 0.0}


# ── 2. 경로 패키지 생성 / Stage 2: serpentine path-package generation ──


def test_field_ab_to_serpentine_package_has_safe_tool_and_primitives(tmp_path: Path) -> None:
    """A/B → 경로 패키지 CLI 가 안전한 툴 활성·허용 프리미티브·정규화 산출물을 생성.

    summary 의 A'/B' 규약·경로 생성·무모터 플래그, tool_path.csv 의 시작(0,1.2)/끝
    (8,0)·sweep 트랙만 활성·커넥터 비활성, primitive_sequence.csv 의 허용 집합·접근
    구간 툴 비활성·무모터, 그리고 정규화 JSON·미리보기 PNG 존재를 종합 확인.

    The A/B→package CLI produces a package with A'/B' semantics, active sweep
    tracks but inactive connectors/approach, allowed motor-free primitives, and
    the normalized-workspace + preview artifacts.
    """
    assert field_ab_to_serpentine.main(
        [
            "--a-x",
            "8",
            "--a-y",
            "0",
            "--b-x",
            "0",
            "--b-y",
            "1.2",
            "--current-x",
            "8",
            "--current-y",
            "0",
            "--current-heading-deg",
            "0",
            "--step-spacing-m",
            "0.25",
            "--tool-side",
            "left",
            "--tool-lateral-offset-m",
            "0.24",
            "--tool-width-m",
            "0.30",
            "--tool-length-m",
            "0.18",
            "--robot-width-m",
            "0.18",
            "--robot-length-m",
            "0.18",
            "--out-dir",
            str(tmp_path),
        ]
    ) == 0

    package = json.loads((tmp_path / "path_package.json").read_text(encoding="utf-8"))
    summary = package["summary"]
    assert summary["A_prime_top_left"] == {"x_m": 0.0, "y_m": 1.2}
    assert summary["B_prime_bottom_right"] == {"x_m": 8.0, "y_m": 0.0}
    assert summary["approach_path_generated"] is True
    assert summary["serpentine_path_generated"] is True
    assert summary["tool_side"] == "left"
    assert summary["primitive_sequence_valid"] is True
    assert summary["motor_command_generated"] is False
    assert summary["physical_output_active"] is False
    assert summary["ready_for_outdoor_no_motion_validation"] is True

    with (tmp_path / "tool_path.csv").open(encoding="utf-8", newline="") as handle:
        tool_rows = list(csv.DictReader(handle))
    assert tool_rows[0]["tool_start_x_m"] == "0.0"
    assert tool_rows[0]["tool_start_y_m"] == "1.2"
    assert tool_rows[-1]["tool_end_x_m"] == "8.0"
    assert tool_rows[-1]["tool_end_y_m"] == "0.0"
    assert {row["tool_active"] for row in tool_rows if row["tool_segment_type"] == "tool_sweep_track"} == {"True"}
    assert {row["tool_active"] for row in tool_rows if row["tool_segment_type"] == "tool_spacing_connector"} == {"False"}

    with (tmp_path / "primitive_sequence.csv").open(encoding="utf-8", newline="") as handle:
        primitive_rows = list(csv.DictReader(handle))
    assert {row["primitive_type"] for row in primitive_rows} <= {
        "move_forward",
        "move_backward",
        "rotate_left",
        "rotate_right",
    }
    assert {row["tool_active"] for row in primitive_rows if row["segment_role"].startswith("approach")} == {"False"}
    assert {row["motor_command_generated"] for row in primitive_rows} == {"False"}
    assert (tmp_path / "normalized_workspace.json").exists()
    assert (tmp_path / "preview_workspace_ab_aprime_bprime.png").exists()
    assert (tmp_path / "preview_tool_path_primary.png").exists()
    assert (tmp_path / "preview_primitive_sequence.png").exists()
    assert (tmp_path / "preview_approach_then_serpentine.png").exists()


# ── 3. 무동작 검증 게이트 / Stage 3: no-motion validation gate ──


def test_no_motion_validation_parser_accepts_sample_log(tmp_path: Path) -> None:
    """무동작 검증기가 샘플 상태 로그를 파싱해 GPS OK·헤딩 대기·준비완료를 보고.

    `build_path_package` 로 패키지를 만들고 GPS/IMU/RC 가 정상인 상태 로그를 먹여
    package_check 모드로 실행. summary 에 gps_status OK·위성/HDOP·헤딩
    WAITING_FOR_MOTION_OR_DIAG_ONLY·ready True 가, CSV 에는 진행거리 열이 없고
    heading_error=NA_DIAG_ONLY·무모터/무물리출력이 담김을 확인.

    The validator parses a healthy status log and reports GPS OK, heading waiting
    (diag-only), readiness True, and motor-free diagnostic CSV rows.
    """
    package = field_ab_to_serpentine.build_path_package(
        raw_a=(8.0, 0.0),
        raw_b=(0.0, 1.2),
        current_pose=(8.0, 0.0, 0.0),
        step_spacing_m=0.25,
        tool_side="left",
        tool_lateral_offset_m=0.24,
        tool_width_m=0.30,
        tool_length_m=0.18,
        robot_width_m=0.18,
        robot_length_m=0.18,
    )
    package_path = tmp_path / "path_package.json"
    package_path.write_text(json.dumps(package), encoding="utf-8")
    sample_log = tmp_path / "status.log"
    sample_log.write_text(
        "position_source=gps gps_sats=6 gps_hdop=1.2 "
        "bmi160_ok=true rc_manual_ok=true x_m=8.0 y_m=0.0 "
        "gps_course_deg=NA physical_output_active=false\n",
        encoding="utf-8",
    )

    assert path_no_motion_validation.main(
        [
            "--path-package",
            str(package_path),
            "--sample-log",
            str(sample_log),
            "--port",
            "/dev/ttyACM0",
            "--concise",
            "true",
            "--no-motion-gps-mode",
            "position_only",
            "--min-sats",
            "4",
            "--max-hdop",
            "3.0",
            "--mode",
            "package_check",
            "--out-dir",
            str(tmp_path / "validation"),
        ]
    ) == 0

    summary = (tmp_path / "validation" / "summary.md").read_text(encoding="utf-8")
    assert "validation_mode: `package_check`" in summary
    assert "serial_opened: `False`" in summary
    assert f"selected_path_package: `{package_path}`" in summary
    assert "gps_status: `OK`" in summary
    assert "gps_sats: `6`" in summary
    assert "gps_hdop: `1.2`" in summary
    assert "position_source: `gps`" in summary
    assert "heading_status: `WAITING_FOR_MOTION_OR_DIAG_ONLY`" in summary
    assert "ready_for_outdoor_no_motion_validation: `True`" in summary
    with (tmp_path / "validation" / "no_motion_validation.csv").open(encoding="utf-8", newline="") as handle:
        rows = list(csv.DictReader(handle))
    assert rows
    assert "along_track_progress_m" not in rows[0]
    assert rows[0]["heading_error_deg"] == "NA_DIAG_ONLY"
    assert {row["motor_command_generated"] for row in rows} == {"False"}
    assert {row["physical_output_active"] for row in rows} == {"False"}


def test_path_package_latest_resolves_field_ab_latest(tmp_path: Path, monkeypatch) -> None:
    """`--path-package latest` 별칭이 field_ab_serpentine/latest 산출물로 해석되는지.

    The "latest" alias resolves to outputs/field_ab_serpentine/latest/
    path_package.json (package discovery convenience).
    """
    latest = tmp_path / "outputs" / "field_ab_serpentine" / "latest" / "path_package.json"
    latest.parent.mkdir(parents=True)
    latest.write_text("{}", encoding="utf-8")
    monkeypatch.chdir(tmp_path)

    assert path_no_motion_validation.resolve_path_package("latest") == latest


def test_nonexistent_path_package_fails_without_traceback(tmp_path: Path, capsys) -> None:
    """없는 패키지 경로는 트레이스백 없이 exit 2 + 진단 힌트(가까운 후보)로 실패.

    A missing package path exits 2 with a friendly diagnostic (provided path,
    file_exists=false, nearest_candidates) and no Python traceback.
    """
    assert path_no_motion_validation.main(
        [
            "--path-package",
            str(tmp_path / "missing" / "path_package.json"),
            "--out-dir",
            str(tmp_path / "validation"),
        ]
    ) == 2
    output = capsys.readouterr().out
    assert "provided_path_package=" in output
    assert "file_exists=false" in output
    assert "nearest_candidates:" in output
    assert "Traceback" not in output


def test_package_check_mode_labels_serial_not_opened(tmp_path: Path) -> None:
    """package_check 모드는 시리얼을 열지 않고 GPS 를 "패키지만 검사"로 표기함.

    package_check labels serial_opened False and gps_status
    NOT_CHECKED_PACKAGE_ONLY (offline package validation, no port).
    """
    package = field_ab_to_serpentine.build_path_package(
        raw_a=(8.0, 0.0),
        raw_b=(0.0, 1.2),
        current_pose=(8.0, 0.0, 0.0),
        step_spacing_m=0.25,
        tool_side="left",
        tool_lateral_offset_m=0.24,
        tool_width_m=0.30,
        tool_length_m=0.18,
        robot_width_m=0.18,
        robot_length_m=0.18,
    )
    package_path = tmp_path / "path_package.json"
    package_path.write_text(json.dumps(package), encoding="utf-8")

    assert path_no_motion_validation.main(
        [
            "--path-package",
            str(package_path),
            "--port",
            "/dev/ttyACM0",
            "--mode",
            "package_check",
            "--out-dir",
            str(tmp_path / "validation"),
        ]
    ) == 0
    summary = (tmp_path / "validation" / "summary.md").read_text(encoding="utf-8")
    assert "validation_mode: `package_check`" in summary
    assert "serial_opened: `False`" in summary
    assert "gps_status: `NOT_CHECKED_PACKAGE_ONLY`" in summary
    assert "Package-only check complete" in summary


def test_live_serial_mode_fails_clearly_if_port_cannot_open(tmp_path: Path, capsys) -> None:
    """live_serial 모드에서 포트를 못 열면 트레이스백 없이 exit 2 로 명확히 실패.

    live_serial with an unopenable port exits 2 with serial_opened=false and an
    error line, no traceback (graceful hardware-absent failure).
    """
    package = field_ab_to_serpentine.build_path_package(
        raw_a=(8.0, 0.0),
        raw_b=(0.0, 1.2),
        current_pose=(8.0, 0.0, 0.0),
        step_spacing_m=0.25,
        tool_side="left",
        tool_lateral_offset_m=0.24,
        tool_width_m=0.30,
        tool_length_m=0.18,
        robot_width_m=0.18,
        robot_length_m=0.18,
    )
    package_path = tmp_path / "path_package.json"
    package_path.write_text(json.dumps(package), encoding="utf-8")

    assert path_no_motion_validation.main(
        [
            "--path-package",
            str(package_path),
            "--port",
            str(tmp_path / "missing_port"),
            "--mode",
            "live_serial",
            "--duration-s",
            "0.1",
            "--out-dir",
            str(tmp_path / "validation"),
        ]
    ) == 2
    output = capsys.readouterr().out
    assert "validation_mode=live_serial" in output
    assert "serial_opened=false" in output
    assert "error=" in output
    assert "Traceback" not in output


def test_no_motion_summary_checker_reports_pass_wait_and_fail() -> None:
    """요약 판정기가 PASS/WAIT/FAIL 세 등급을 상황별로 올바로 반환하는지 검증.

    정상 요약→PASS, 위성 부족/헤딩 대기→WAIT(사유 포함), 물리출력 활성→FAIL 을 확인
    (사람이 읽는 go/no-go 판정 계약).

    The summary checker maps a healthy summary to PASS, low-sats/stationary to WAIT
    (with reasons), and active physical output to FAIL.
    """
    base = {
        "motor_command_generated": "False",
        "physical_output_active": "False",
        "path_package_loaded": "True",
        "serial_opened": "True",
        "validation_mode": "live_serial",
        "position_source": "gps",
        "gps_status": "OK",
        "gps_sats": "6",
        "gps_hdop": "1.2",
        "target_distance_finite": "True",
        "target_bearing_finite": "True",
        "heading_status": "OK",
    }
    assert check_path_no_motion_summary.evaluate_summary(base) == (
        "PASS",
        "target preview ready",
    )

    marginal = base | {"gps_sats": "3", "gps_status": "WAIT"}
    verdict, action = check_path_no_motion_summary.evaluate_summary(marginal)
    assert verdict == "WAIT"
    assert "satellites" in action

    stationary = base | {"heading_status": "WAITING_FOR_MOTION_OR_DIAG_ONLY"}
    verdict, action = check_path_no_motion_summary.evaluate_summary(stationary)
    assert verdict == "WAIT"
    assert "stationary" in action

    unsafe = base | {"physical_output_active": "True"}
    verdict, action = check_path_no_motion_summary.evaluate_summary(unsafe)
    assert verdict == "FAIL"
    assert "physical output" in action


# ── 4. 패키지 검사·미리보기·드라이런 분석 / Stage 4: inspect, preview, dryrun ──


def test_inspect_path_package_validates_known_good_package(tmp_path: Path) -> None:
    """패키지 검사기가 정상 패키지의 모든 검증 항목을 True 로 통과시키는지 확인.

    inspect_path_package 가 tool_side/시작·끝 코너/연속성/커넥터 비활성/트랙 활성/
    프리미티브 허용/무모터 등 검증 플래그를 모두 True 로 보고하고 리포트 MD 를 씀.

    The inspector marks a known-good package's checks (corners, continuity,
    connector/track activity, allowed primitives, motor-free) all True.
    """
    package = field_ab_to_serpentine.build_path_package(
        raw_a=(8.0, 0.0),
        raw_b=(0.0, 1.2),
        current_pose=(8.0, 0.0, 0.0),
        step_spacing_m=0.25,
        tool_side="left",
        tool_lateral_offset_m=0.24,
        tool_width_m=0.30,
        tool_length_m=0.18,
        robot_width_m=0.18,
        robot_length_m=0.18,
    )
    package_path = tmp_path / "path_package.json"
    package_path.write_text(json.dumps(package), encoding="utf-8")

    assert inspect_path_package.main(
        ["--path-package", str(package_path), "--out-dir", str(tmp_path / "inspection")]
    ) == 0
    inspection = json.loads((tmp_path / "inspection" / "path_package_inspection.json").read_text(encoding="utf-8"))
    validation = inspection["validation"]
    assert validation["tool_side_left"] is True
    assert validation["tool_path_starts_at_A_prime"] is True
    assert validation["tool_path_ends_at_B_prime"] is True
    assert validation["tool_path_continuous"] is True
    assert validation["connectors_inactive"] is True
    assert validation["sweep_tracks_active"] is True
    assert validation["primitive_sequence_allowed"] is True
    assert validation["motor_command_generated_false"] is True
    assert (tmp_path / "inspection" / "path_package_inspection.md").exists()


def test_physical_path_preview_from_package_generates_offline_outputs(tmp_path: Path) -> None:
    """패키지에서 오프라인 물리 미리보기(요약·검사된 프리미티브 CSV·PNG)를 재생성.

    physical_path_preview_from_package 가 summary/primitive_sequence_checked.csv/
    툴 경로 PNG 를 만들고, CSV 각 행이 primitive_allowed=True·무모터임을 확인.

    Regenerates offline preview artifacts from a package (summary, checked-primitive
    CSV, PNG) with every primitive allowed and motor-free.
    """
    package = field_ab_to_serpentine.build_path_package(
        raw_a=(8.0, 0.0),
        raw_b=(0.0, 1.2),
        current_pose=(8.0, 0.0, 0.0),
        step_spacing_m=0.25,
        tool_side="left",
        tool_lateral_offset_m=0.24,
        tool_width_m=0.30,
        tool_length_m=0.18,
        robot_width_m=0.18,
        robot_length_m=0.18,
    )
    package_path = tmp_path / "path_package.json"
    package_path.write_text(json.dumps(package), encoding="utf-8")

    assert physical_path_preview_from_package.main(
        ["--path-package", str(package_path), "--out-dir", str(tmp_path / "preview")]
    ) == 0
    assert (tmp_path / "preview" / "summary.md").exists()
    assert (tmp_path / "preview" / "primitive_sequence_checked.csv").exists()
    assert (tmp_path / "preview" / "preview_tool_path.png").exists()
    with (tmp_path / "preview" / "primitive_sequence_checked.csv").open(encoding="utf-8", newline="") as handle:
        rows = list(csv.DictReader(handle))
    assert rows
    assert {row["primitive_allowed"] for row in rows} == {"True"}
    assert {row["motor_command_generated"] for row in rows} == {"False"}


def test_integrated_dryrun_analyzer_detects_compile_time_target_and_course_skip(tmp_path: Path) -> None:
    """통합 드라이런 로그 분석기가 GPS/IMU/모터안전 OK 와 compile_time 타깃을 잡아냄.

    펌웨어 드라이런 로그에서 위치/IMU/모터 게이트가 정상이고 코스 출력은 무동작으로
    스킵됨을 판정하되, 활성 타깃이 compile_time(라이브 패키지 미연결)이라 사유
    ACTIVE_TARGET_SOURCE_COMPILE_TIME + "모터 시험 금지" 권고를 내는지 확인.

    The dryrun-log analyzer confirms GPS/IMU/motor-safety OK but flags a
    compile-time active target (no live package), recommending "do not proceed to
    motor tests".
    """
    log = tmp_path / "integrated.log"
    log.write_text(
        "gps_block_reason=OK gps_sats=5 gps_hdop=1.88 position_source=gps "
        "imu_type=BMI160 imu_data_plausible=true imu_calibrated=true "
        "physical_compile_gate=false physical_path_following_enable=false "
        "allow_motor_output=false physical_output_active=false "
        "gps_course_deg=NA gps_course_output_block_reason=NO_ACCEPTED_COURSE_YET "
        "path_following_block_reason=NO_HEADING active_target_source=compile_time wp_count=3 "
        "target_distance_m=68.0\n",
        encoding="utf-8",
    )
    result = analyze_integrated_dryrun_for_path_package.analyze_files([log])
    assert result["gps_position_ok"] is True
    assert result["imu_ok"] is True
    assert result["motor_safety_ok"] is True
    assert result["gps_course_status"] == "SKIPPED_NO_MOTION_OR_TETHERED"
    assert result["active_target_source"] == "compile_time"
    assert result["live_path_package_connected"] is False
    assert result["current_target_source_is_compile_time"] is True
    assert result["reason"] == "ACTIVE_TARGET_SOURCE_COMPILE_TIME"
    assert "Do not proceed to motor tests" in str(result["recommendation"])
    assert result["motor_command_generated"] is False


# ── 5. 스테이션 타깃 추적 (Stage 12) / Station target tracking ──


def test_station_tracker_offline_pose_computes_target_status(tmp_path: Path) -> None:
    """오프라인 포즈 입력에서 스테이션 추적기가 다음 타깃 거리/방위를 계산(무모터).

    offline_pose 모드로 현재 좌표를 주면 station_target_status.csv 에 패키지 기반
    타깃 소스·유한한 거리/방위·무모터가 담기고, summary 에 타깃 프리뷰 준비 True·
    모터 시험 준비 False 가 표기됨을 확인.

    In offline_pose mode the tracker computes a finite next-target distance/bearing
    from the package (motor-free), ready for target preview but not motor tests.
    """
    package = field_ab_to_serpentine.build_path_package(
        raw_a=(8.0, 0.0),
        raw_b=(0.0, 1.2),
        current_pose=(8.0, 0.0, 0.0),
        step_spacing_m=0.25,
        tool_side="left",
        tool_lateral_offset_m=0.24,
        tool_width_m=0.30,
        tool_length_m=0.18,
        robot_width_m=0.18,
        robot_length_m=0.18,
    )
    package_path = tmp_path / "path_package.json"
    package_path.write_text(json.dumps(package), encoding="utf-8")

    assert station_path_package_tracker.main(
        [
            "--path-package",
            str(package_path),
            "--mode",
            "offline_pose",
            "--current-x",
            "0",
            "--current-y",
            "1.2",
            "--current-heading-deg",
            "0",
            "--out-dir",
            str(tmp_path / "tracker"),
        ]
    ) == 0
    with (tmp_path / "tracker" / "station_target_status.csv").open(encoding="utf-8", newline="") as handle:
        rows = list(csv.DictReader(handle))
    assert rows
    assert rows[0]["station_package_target_source"] == "path_package"
    assert rows[0]["target_distance_m"] != "NA"
    assert rows[0]["target_bearing_deg"] != "NA"
    assert rows[0]["motor_command_generated"] == "False"
    summary = (tmp_path / "tracker" / "summary.md").read_text(encoding="utf-8")
    assert "ready_for_station_side_target_preview: `True`" in summary
    assert "ready_for_motor_test: `False`" in summary


def test_station_tracker_replay_log_reports_no_georeference_and_compile_time_source(tmp_path: Path) -> None:
    """georeference 없는 lat/lon 리플레이 로그는 로컬 포즈 불가로 강등되고 사유를 보고.

    replay_log 의 위경도 행은 패키지에 georef 가 없으면 local_pose_available=False,
    reason=NO_GEOREFERENCE_FOR_LAT_LON_TO_LOCAL, 거리 NA 로 진단되고, 펌웨어 타깃이
    여전히 compile_time 이라 타깃 프리뷰/모터 준비 모두 False 임을 확인.

    A lat/lon replay log without georeference degrades to a diagnostic row
    (NO_GEOREFERENCE..., distance NA) and stays not-ready (firmware still
    compile-time).
    """
    package = field_ab_to_serpentine.build_path_package(
        raw_a=(8.0, 0.0),
        raw_b=(0.0, 1.2),
        current_pose=(8.0, 0.0, 0.0),
        step_spacing_m=0.25,
        tool_side="left",
        tool_lateral_offset_m=0.24,
        tool_width_m=0.30,
        tool_length_m=0.18,
        robot_width_m=0.18,
        robot_length_m=0.18,
    )
    package_path = tmp_path / "path_package.json"
    package_path.write_text(json.dumps(package), encoding="utf-8")
    log = tmp_path / "usbdbg.log"
    log.write_text(
        "current_lat=37.1 current_lon=127.2 active_target_source=compile_time "
        "gps_block_reason=OK gps_sats=5 gps_hdop=1.8 physical_output_active=false\n",
        encoding="utf-8",
    )

    assert station_path_package_tracker.main(
        [
            "--path-package",
            str(package_path),
            "--mode",
            "replay_log",
            "--log",
            str(log),
            "--out-dir",
            str(tmp_path / "tracker"),
        ]
    ) == 0
    with (tmp_path / "tracker" / "station_target_status.csv").open(encoding="utf-8", newline="") as handle:
        rows = list(csv.DictReader(handle))
    assert rows[0]["local_pose_available"] == "False"
    assert rows[0]["reason"] == "NO_GEOREFERENCE_FOR_LAT_LON_TO_LOCAL"
    assert rows[0]["firmware_active_target_source"] == "compile_time"
    assert rows[0]["station_package_target_source"] == "path_package"
    assert rows[0]["target_distance_m"] == "NA"
    summary = (tmp_path / "tracker" / "summary.md").read_text(encoding="utf-8")
    assert "firmware_still_compile_time: `True`" in summary
    assert "ready_for_station_side_target_preview: `False`" in summary
    assert "ready_for_motor_test: `False`" in summary


def test_station_tracker_usbdbg_parser_handles_local_rows_without_serial() -> None:
    """USBDBG 파서가 로컬 x/y 좌표 행을 시리얼 없이도 처리해 로컬 포즈를 인식.

    build_rows_from_replay 에 local x/y 를 담은 라인을 주면 local_pose_available=True,
    타깃 소스=path_package, 펌웨어 타깃=compile_time, 무모터로 파싱됨을 확인.

    The USBDBG parser accepts rows with local x/y (no serial) and marks
    local_pose_available True with a package target source, motor-free.
    """
    package = field_ab_to_serpentine.build_path_package(
        raw_a=(8.0, 0.0),
        raw_b=(0.0, 1.2),
        current_pose=(8.0, 0.0, 0.0),
        step_spacing_m=0.25,
        tool_side="left",
        tool_lateral_offset_m=0.24,
        tool_width_m=0.30,
        tool_length_m=0.18,
        robot_width_m=0.18,
        robot_length_m=0.18,
    )
    rows = station_path_package_tracker.build_rows_from_replay(
        package,
        "current_x_m=0 current_y_m=1.2 current_heading_deg=0 active_target_source=compile_time\n",
        "live_usbdbg",
    )
    assert rows[0]["local_pose_available"] is True
    assert rows[0]["station_package_target_source"] == "path_package"
    assert rows[0]["firmware_active_target_source"] == "compile_time"
    assert rows[0]["motor_command_generated"] is False


# ── 6. 위경도(georeference) 경로 / Georeferenced (lat/lon) path ──


def test_manual_latlon_georef_capture_writes_json(tmp_path: Path) -> None:
    """수동 위경도 A/B 캡처가 georeference 가능 플래그와 lat/lon 을 JSON/CSV 로 저장.

    Manual lat/lon capture writes field_points_georef.{json,csv} with
    georeference_available True and the given A/B lat/lon, motor-free.
    """
    assert capture_georef_ab_points.main(
        [
            "--mode",
            "manual_latlon",
            "--a-lat",
            "35.0",
            "--a-lon",
            "129.0",
            "--b-lat",
            "35.00001",
            "--b-lon",
            "129.00010",
            "--out-dir",
            str(tmp_path / "capture"),
        ]
    ) == 0
    data = json.loads((tmp_path / "capture" / "field_points_georef.json").read_text(encoding="utf-8"))
    assert data["georeference_available"] is True
    assert data["points"]["A"]["lat"] == 35.0
    assert data["points"]["B"]["lon"] == 129.00010
    assert data["motor_command_generated"] is False
    assert (tmp_path / "capture" / "field_points_georef.csv").exists()


def test_field_ab_to_serpentine_accepts_georef_points(tmp_path: Path) -> None:
    """캡처한 위경도 파일을 경로 패키지 생성기가 입력으로 받아 georef 메타를 심는지 확인.

    캡처(georef JSON)→field_ab_to_serpentine(--field-points-georef-json) 2단계로
    만든 패키지에 georeference_available·local_frame_type=equirectangular_enu·
    origin_lat 이 담기고 무모터임을 확인(위경도 입력 경로의 엔드투엔드).

    The path generator accepts a captured georef file and embeds georeference
    metadata (equirectangular_enu, origin_lat) into the package, motor-free.
    """
    capture_dir = tmp_path / "capture"
    assert capture_georef_ab_points.main(
        [
            "--mode",
            "manual_latlon",
            "--a-lat",
            "35.0",
            "--a-lon",
            "129.0",
            "--b-lat",
            "35.00001",
            "--b-lon",
            "129.00010",
            "--out-dir",
            str(capture_dir),
        ]
    ) == 0
    assert field_ab_to_serpentine.main(
        [
            "--field-points-georef-json",
            str(capture_dir / "field_points_georef.json"),
            "--step-spacing-m",
            "0.25",
            "--tool-side",
            "left",
            "--tool-lateral-offset-m",
            "0.24",
            "--tool-width-m",
            "0.30",
            "--tool-length-m",
            "0.18",
            "--robot-width-m",
            "0.18",
            "--robot-length-m",
            "0.18",
            "--out-dir",
            str(tmp_path / "package"),
        ]
    ) == 0
    package = json.loads((tmp_path / "package" / "path_package.json").read_text(encoding="utf-8"))
    assert package["georeference"]["georeference_available"] is True
    assert package["georeference"]["local_frame_type"] == "equirectangular_enu"
    assert package["georeference"]["origin_lat"] == 35.0
    assert package["summary"]["georeference_available"] is True
    assert package["summary"]["motor_command_generated"] is False


def test_station_tracker_replay_log_converts_lat_lon_with_georef(tmp_path: Path) -> None:
    """georeference 가 있으면 리플레이 로그의 lat/lon 이 로컬 포즈로 변환되어 타깃이 계산됨.

    패키지에 georef 를 심고 원점(35.0,129.0)과 같은 위경도 로그를 주면
    local_pose_available=True, source=gps_georeference, reason=OK, 유한 거리로
    변환됨을 확인. 펌웨어는 여전히 compile_time 이라 모터 준비는 False.

    With georeference present, a lat/lon replay row converts to a local pose
    (gps_georeference, reason OK, finite distance); target preview is ready but
    motor test is not.
    """
    # georef 리터럴은 캡처 산출물 스키마를 흉내낸 것 / mimics the captured georef schema
    georef = {
        "georeference_available": True,
        "raw_A_lat": 35.0,
        "raw_A_lon": 129.0,
        "raw_B_lat": 35.00001,
        "raw_B_lon": 129.00010,
        "origin_lat": 35.0,
        "origin_lon": 129.0,
        "local_frame_type": "equirectangular_enu",
        "x_axis_source": "normalized_rectangle",
        "x_axis_bearing_deg": 90.0,
        "meters_per_deg_lat": field_ab_to_serpentine.METERS_PER_DEG_LAT,
        "meters_per_deg_lon": field_ab_to_serpentine._meters_per_deg_lon(35.000005),
        "meters_per_lat": field_ab_to_serpentine.METERS_PER_DEG_LAT,
        "meters_per_lon": field_ab_to_serpentine._meters_per_deg_lon(35.000005),
        "motor_command_generated": False,
    }
    package = field_ab_to_serpentine.build_path_package(
        raw_a=(0.0, 0.0),
        raw_b=(9.1, 1.1),
        current_pose=(0.0, 0.0, 0.0),
        step_spacing_m=0.25,
        tool_side="left",
        tool_lateral_offset_m=0.24,
        tool_width_m=0.30,
        tool_length_m=0.18,
        robot_width_m=0.18,
        robot_length_m=0.18,
        georeference=georef,
    )
    package_path = tmp_path / "path_package.json"
    package_path.write_text(json.dumps(package), encoding="utf-8")
    log = tmp_path / "usbdbg.log"
    log.write_text(
        "current_lat=35.0 current_lon=129.0 active_target_source=compile_time physical_output_active=false\n",
        encoding="utf-8",
    )
    assert station_path_package_tracker.main(
        [
            "--path-package",
            str(package_path),
            "--mode",
            "replay_log",
            "--log",
            str(log),
            "--out-dir",
            str(tmp_path / "tracker"),
        ]
    ) == 0
    with (tmp_path / "tracker" / "station_target_status.csv").open(encoding="utf-8", newline="") as handle:
        rows = list(csv.DictReader(handle))
    assert rows[0]["local_pose_available"] == "True"
    assert rows[0]["local_pose_source"] == "gps_georeference"
    assert rows[0]["reason"] == "OK"
    assert rows[0]["firmware_active_target_source"] == "compile_time"
    assert rows[0]["station_package_target_source"] == "path_package"
    assert rows[0]["target_distance_m"] != "NA"
    summary = (tmp_path / "tracker" / "summary.md").read_text(encoding="utf-8")
    assert "firmware_still_compile_time: `True`" in summary
    assert "ready_for_station_side_target_preview: `True`" in summary
    assert "ready_for_motor_test: `False`" in summary


def test_check_georef_path_package_reports_metadata(tmp_path: Path) -> None:
    """georef 패키지 검사기가 메타데이터와 변환 정합성(sanity)을 보고하는지 확인.

    check_georef_path_package.inspect_georef 가 georeference_available·
    local_frame_type·conversion_sanity_check=True·무모터를 보고함을 검증.

    The georef package checker reports metadata (frame type) and a passing
    conversion sanity check, motor-free.
    """
    georef = {
        "georeference_available": True,
        "raw_A_lat": 35.0,
        "raw_A_lon": 129.0,
        "raw_B_lat": 35.00001,
        "raw_B_lon": 129.00010,
        "origin_lat": 35.0,
        "origin_lon": 129.0,
        "local_frame_type": "equirectangular_enu",
        "x_axis_source": "normalized_rectangle",
        "x_axis_bearing_deg": 90.0,
        "meters_per_deg_lat": field_ab_to_serpentine.METERS_PER_DEG_LAT,
        "meters_per_deg_lon": field_ab_to_serpentine._meters_per_deg_lon(35.000005),
        "meters_per_lat": field_ab_to_serpentine.METERS_PER_DEG_LAT,
        "meters_per_lon": field_ab_to_serpentine._meters_per_deg_lon(35.000005),
        "motor_command_generated": False,
    }
    package = field_ab_to_serpentine.build_path_package(
        raw_a=(0.0, 0.0),
        raw_b=(9.1, 1.1),
        current_pose=(0.0, 0.0, 0.0),
        step_spacing_m=0.25,
        tool_side="left",
        tool_lateral_offset_m=0.24,
        tool_width_m=0.30,
        tool_length_m=0.18,
        robot_width_m=0.18,
        robot_length_m=0.18,
        georeference=georef,
    )
    package_path = tmp_path / "path_package.json"
    package_path.write_text(json.dumps(package), encoding="utf-8")
    result = check_georef_path_package.inspect_georef(package, package_path)
    assert result["georeference_available"] is True
    assert result["local_frame_type"] == "equirectangular_enu"
    assert result["conversion_sanity_check"] is True
    assert result["motor_command_generated"] is False


# ── 7. 스테이션 가상 제어 (Stage 14) / Station virtual control ──


def test_virtual_controller_offline_pose_produces_virtual_control(tmp_path: Path) -> None:
    """오프라인 포즈에서 가상 제어기가 클램프된 전진/회전 명령을 "가상으로" 산출.

    offline_pose 모드로 virtual_control.csv 를 만들고 virtual_control_generated=True,
    heading OK, forward<=0.10·|turn|<=0.05 로 클램프, 무모터/모터준비 False, PNG 존재를
    확인(진단 전용 가상 명령).

    In offline_pose the virtual controller emits clamped virtual forward/turn
    commands (heading OK) that are motor-free and not motor-test-ready.
    """
    package = field_ab_to_serpentine.build_path_package(
        raw_a=(8.0, 0.0),
        raw_b=(0.0, 1.2),
        current_pose=(8.0, 0.0, 0.0),
        step_spacing_m=0.25,
        tool_side="left",
        tool_lateral_offset_m=0.24,
        tool_width_m=0.30,
        tool_length_m=0.18,
        robot_width_m=0.18,
        robot_length_m=0.18,
    )
    package_path = tmp_path / "path_package.json"
    package_path.write_text(json.dumps(package), encoding="utf-8")
    assert station_virtual_path_controller.main(
        [
            "--path-package",
            str(package_path),
            "--mode",
            "offline_pose",
            "--current-x",
            "0",
            "--current-y",
            "1.2",
            "--current-heading-deg",
            "0",
            "--out-dir",
            str(tmp_path / "virtual"),
        ]
    ) == 0
    with (tmp_path / "virtual" / "virtual_control.csv").open(encoding="utf-8", newline="") as handle:
        rows = list(csv.DictReader(handle))
    assert rows
    assert rows[0]["virtual_control_generated"] == "True"
    assert rows[0]["virtual_heading_status"] == "OK"
    assert float(rows[0]["virtual_forward_cmd"]) <= 0.10
    assert abs(float(rows[0]["virtual_turn_cmd"])) <= 0.05
    assert rows[0]["motor_command_generated"] == "False"
    assert rows[0]["ready_for_motor_test"] == "False"
    assert (tmp_path / "virtual" / "preview_virtual_control.png").exists()


def test_virtual_controller_replay_log_with_georef_produces_rows(tmp_path: Path) -> None:
    """georef 리플레이 로그로 가상 제어기가 행을 생성하고 무모터/모터준비False 를 유지.

    replay_log + georef 로 virtual_control.csv 를 만들고 펌웨어 타깃 compile_time·
    virtual_control_generated=True·무모터/무물리출력을 확인, summary 에 가상 제어
    프리뷰 준비 True·모터 시험 준비 False 가 표기됨을 검증.

    A georeferenced replay log yields virtual-control rows (firmware still
    compile-time) that stay motor-free; preview ready, motor test not.
    """
    # georef 리터럴은 캡처 산출물 스키마를 흉내낸 것 / mimics the captured georef schema
    georef = {
        "georeference_available": True,
        "raw_A_lat": 35.0,
        "raw_A_lon": 129.0,
        "raw_B_lat": 35.00001,
        "raw_B_lon": 129.00010,
        "origin_lat": 35.0,
        "origin_lon": 129.0,
        "local_frame_type": "equirectangular_enu",
        "x_axis_source": "normalized_rectangle",
        "x_axis_bearing_deg": 90.0,
        "meters_per_deg_lat": field_ab_to_serpentine.METERS_PER_DEG_LAT,
        "meters_per_deg_lon": field_ab_to_serpentine._meters_per_deg_lon(35.000005),
        "meters_per_lat": field_ab_to_serpentine.METERS_PER_DEG_LAT,
        "meters_per_lon": field_ab_to_serpentine._meters_per_deg_lon(35.000005),
        "motor_command_generated": False,
    }
    package = field_ab_to_serpentine.build_path_package(
        raw_a=(0.0, 0.0),
        raw_b=(9.1, 1.1),
        current_pose=(0.0, 0.0, 0.0),
        step_spacing_m=0.25,
        tool_side="left",
        tool_lateral_offset_m=0.24,
        tool_width_m=0.30,
        tool_length_m=0.18,
        robot_width_m=0.18,
        robot_length_m=0.18,
        georeference=georef,
    )
    package_path = tmp_path / "path_package.json"
    package_path.write_text(json.dumps(package), encoding="utf-8")
    log = tmp_path / "usbdbg.log"
    log.write_text("current_lat=35.0 current_lon=129.0 active_target_source=compile_time\n", encoding="utf-8")
    assert station_virtual_path_controller.main(
        [
            "--path-package",
            str(package_path),
            "--mode",
            "replay_log",
            "--log",
            str(log),
            "--out-dir",
            str(tmp_path / "virtual"),
        ]
    ) == 0
    with (tmp_path / "virtual" / "virtual_control.csv").open(encoding="utf-8", newline="") as handle:
        rows = list(csv.DictReader(handle))
    assert rows[0]["firmware_active_target_source"] == "compile_time"
    assert rows[0]["virtual_control_generated"] == "True"
    assert rows[0]["motor_command_generated"] == "False"
    assert rows[0]["physical_output_active"] == "False"
    summary = (tmp_path / "virtual" / "summary.md").read_text(encoding="utf-8")
    assert "ready_for_station_virtual_control_preview: `True`" in summary
    assert "ready_for_motor_test: `False`" in summary


def test_virtual_controller_heading_missing_is_diag_only_not_motor_ready() -> None:
    """헤딩이 없으면 가상 제어가 DIAG_ONLY 로 표기되고 모터 준비는 절대 True 가 안 됨.

    현재 헤딩이 빈 문자열인 행에 compute_virtual_control 을 적용하면
    virtual_heading_status=DIAG_ONLY, 프리뷰 준비 True 지만 모터 준비/무모터는 False.

    Missing heading yields DIAG_ONLY virtual control: preview-ready but never
    motor-ready, motor-free.
    """
    row = {
        "local_pose_available": True,
        "target_distance_m": 2.0,
        "target_bearing_deg": 10.0,
        "cross_track_error_m": 0.2,
        "current_heading_deg": "",
        "heading_error_deg": "NA_DIAG_ONLY",
        "motor_command_generated": False,
        "physical_output_active": False,
    }
    control = station_virtual_path_controller.compute_virtual_control(row)
    assert control["virtual_control_generated"] is True
    assert control["virtual_heading_status"] == "DIAG_ONLY"
    assert control["ready_for_station_virtual_control_preview"] is True
    assert control["ready_for_motor_test"] is False
    assert control["motor_command_generated"] is False


def test_virtual_controller_live_parser_rows_are_diagnostic_without_serial() -> None:
    """시리얼 없이도 라이브 파서→타깃 행→가상 행이 진단(DIAG_ONLY)으로 생성되는지 확인.

    Stage 12 build_rows_from_replay(live_usbdbg)로 타깃 행을 만든 뒤
    build_virtual_rows 로 가상 행을 얹으면 virtual_control_generated=True·
    DIAG_ONLY·모터준비False·무모터임을 검증(Stage12→Stage14 연동, 하드웨어 불필요).

    Without serial, the live parser → target rows → virtual rows chain produces
    DIAG_ONLY virtual control that is motor-free and not motor-ready.
    """
    package = field_ab_to_serpentine.build_path_package(
        raw_a=(8.0, 0.0),
        raw_b=(0.0, 1.2),
        current_pose=(8.0, 0.0, 0.0),
        step_spacing_m=0.25,
        tool_side="left",
        tool_lateral_offset_m=0.24,
        tool_width_m=0.30,
        tool_length_m=0.18,
        robot_width_m=0.18,
        robot_length_m=0.18,
    )
    target_rows = station_path_package_tracker.build_rows_from_replay(
        package,
        "current_x_m=0 current_y_m=1.2 active_target_source=compile_time\n",
        "live_usbdbg",
    )
    virtual_rows = station_virtual_path_controller.build_virtual_rows(target_rows)
    assert virtual_rows[0]["virtual_control_generated"] is True
    assert virtual_rows[0]["virtual_heading_status"] == "DIAG_ONLY"
    assert virtual_rows[0]["ready_for_motor_test"] is False
    assert virtual_rows[0]["motor_command_generated"] is False
