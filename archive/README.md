# archive/ — 보관용 레거시 코드 (Archived legacy code)

**한국어**

이 폴더에는 더 이상 활동적으로 쓰이지 않는 레거시/단독 스크립트를 보관합니다.
현재 워크플로에서는 통합 CLI(`scripts/run_physical_path_planner.sh` →
`tools/physical_path_planning/cli.py`)나 분석 파이프라인이 이 스크립트들을 대체합니다.

여기 있는 파일들의 공통 성격:

- 어떤 모듈에서도 `import`되지 않음,
- 어떤 테스트(`tests/`)에서도 참조되지 않음,
- 통합 CLI(`tools/physical_path_planning/`)와 셸 스크립트(`scripts/*.sh`)에서 쓰이지 않음.

즉 **삭제해도 빌드/테스트에는 영향이 없지만**, 초기 현장·디버그 절차의 기록으로
남겨 두기 위해 삭제 대신 이곳으로 옮겼습니다. 다시 쓰려면 해당 파일을 `tools/`로
되돌리면 됩니다(경로 기반 `import`가 있을 수 있으니 함께 옮겨진 짝 파일도 확인).

**English**

This folder holds legacy, standalone scripts that are no longer part of the
active workflow. They were superseded by the unified CLI
(`scripts/run_physical_path_planner.sh` → `tools/physical_path_planning/cli.py`)
or by the analysis pipeline. Every file here is (a) imported by no module,
(b) referenced by no test under `tests/`, and (c) unused by the unified CLI and
shell scripts — so archiving them does not affect the build or the test suite.
They are kept for historical/reference value. To revive one, move it back to
`tools/` (mind any path-based imports and its companion files).

## 목록 (Contents) — `archive/tools/`

| 파일 | 원래 역할 (former role) |
|---|---|
| `analyze_ppm_log.py` | PPM 수신기 로그 오프라인 분석 (superseded by `rc-input-diagnose`) |
| `gps_logger.py` | GPS NMEA 원시 로깅 도구 |
| `nmea_replay.py` | 기록된 NMEA 문장 리플레이 |
| `serial_open_test.py` | 시리얼 포트 open 스모크 테스트 |
| `hc12_terminal.py` | HC-12 대화형 터미널 (ad-hoc) |
| `station_hc12_test.py` | HC-12 링크 수동 점검 스크립트 |
| `station_controller.py` | 통합 CLI 이전의 스테이션 제어 프로토타입 |
| `station_keyboard_manual.py` | 키보드 수동 조종 프로토타입 (`station_controller` 사용) |
| `station_mock_mission.py` | 스테이션 목(mock) 미션 러너 |
