# Claude 협업 가이드 (Physical Path Planning)

이 문서는 Claude(또는 다른 AI 어시스턴트)와 함께 이 저장소에서 작업을 이어갈 때
새 세션에 그대로 붙여 넣거나 참조시키기 위한 단일 진입점이다. 마지막 갱신:
2026-06-11 (lawnmower connector 리팩토링 직후).

## 1. 프로젝트 한 줄 요약

OpenRB-150 로버(GPS + BMI160 IMU + HC-12 + PPM RC)를 스테이션(Mac, USB 시리얼)이
감독하며, ㄹ자(lawnmower) 커버리지 경로를 물리적으로 주행시키는 프로젝트.
모든 필드 작업은 단일 진입점을 쓴다:

```bash
bash scripts/run_physical_path_planner.sh <mode> [options]
```

## 2. 새 세션을 시작할 때 Claude에게 먼저 읽힐 것

1. `AGENTS.md` — 저장소 불변 규칙(안전 기본값, STOP 우선, ROS2 금지 등)
2. `docs/claude_collaboration_guide.md` — 이 문서
3. `docs/physical_path_planning_architecture.md` — 모듈 구조와 제어 의미론
4. `docs/README_physical_path_planning.md` — 모드별 런북(ㄹ 필드 런북 포함)
5. `docs/known_issues.md` — 하드웨어 특이사항(특히 회전/모터 항목)
6. 직전 보고서: `docs/reports/interim/2026-06-11_lawnmower_connector_refactor.md`

## 3. 현재 하드웨어 상태(2026-06-11 기준, 코드가 가정하는 사실)

- 바퀴 모터가 비대칭이고 약하다. 특히 우회전이 약함.
- 승인된 모션 캘리브레이션(`outputs/physical_path_planning/calibration/motion_calibration.json`):
  - forward: `A=+0.30, 600ms` / backward: `A=-0.08, 300ms`
  - turn_left_90: `B=+0.24, 700ms` / turn_right_90: `B=-0.08, 600ms`
  - **중요**: 위 turn 펄스는 이름과 달리 실제로는 한 번에 약 15~45도만 돈다.
    실측 각도를 `target_angle_deg`에 저장해야 하며(예: 30), 코드는 그 값으로
    코너당 펄스 수를 계산한다. `assume_90` 정책은 레거시 호환용.
- BMI160 yaw는 상대값이고 드리프트가 있다(절대 나침반 아님). 몇 분짜리 미션
  안에서는 mission heading 기준으로 쓸 만하다.
- GPS는 m급 오차. `--cross-track-correction-threshold-m`은 레인 간격보다
  충분히 작아야 한다(1.2m 간격이면 0.35m 권장).
- RC CH5(모드 스위치)는 MANUAL/AUTO 전환에 쓰이며, USB 감독 모드에서도
  스테이션 루프가 MANUAL 전환을 감지해 즉시 정지한다.

## 4. 핵심 제어 의미론 (2026-06-11 리팩토링 결과)

### 4.1 ㄹ 코너 = turn_step_turn

`coverage_lawnmower`의 레인 전환은 기본적으로 3개 세그먼트로 분해된다:

```
connector_turn(제자리 ~90°) -> step_lane(간격만큼 직진/후진) -> connector_turn(~90° 복귀)
```

- 전진 레인 뒤 스텝은 전진, 후진 레인 뒤 스텝은 후진으로 주행
  (사용자가 원한 "전진→회전→약간 전진→회전→후진→회전→약간 후진→회전" 패턴).
- 각 세그먼트에 `body_heading_deg`(몸체가 향해야 할 방향)와 connector에
  `turn_angle_deg`(부호 있는 회전각, +좌/-우)가 들어 있다.
- 레거시 플랜(`path_connector` 단일 세그먼트)도 그대로 실행 가능.
  새 플랜을 옛 방식으로 만들려면 `--connector-style single_turn`.

### 4.2 connector 회전 = target_angle_deg 기반 다중 펄스 + IMU 폐루프

- 캘리브레이션의 `target_angle_deg`(펄스 1회의 실제 회전각)를 신뢰한다
  (`--turn-calibration-angle-policy from_json` 기본).
- connector 시작 시점의 yaw를 기준으로, 측정된 yaw 변화가 요청 각도에
  도달할 때까지 펄스 반복. 한도는 `--max-connector-pulses-per-turn`(기본 6),
  허용 오차는 `--connector-turn-tolerance-deg`(기본 10).
- IMU가 없으면 `ceil(|각도|/target_angle_deg)`개의 펄스만 개루프로 실행.
- JSON 수정 없이 실측 각도를 줄 때는 `--turn-angle-deg-override 30`.

### 4.3 heading 기준 = mission (기본)

- 미션 시작 시(첫 레인, 정렬된 상태) yaw 프레임을 한 번 잡고 끝까지 유지.
- 코너에서 덜 돈 각도가 다음 레인의 heading 오차로 그대로 드러나고,
  기존 IMU 제자리 보정 회전이 그것을 고친다.
- 후진 레인은 `body_heading_deg`(주행방향+180)를 유지한다.
- 옛 동작(레인마다 기준 재캡처, 오차 흡수)은 `--heading-reference per_lane`.

### 4.4 AUTO/MANUAL 스위치 (auto-relative-run)

- MANUAL→AUTO: 현재 GPS 위치를 원점으로 상대 ENU 플랜을 1회 실행.
- AUTO→MANUAL: 펄스 윈도/보정 회전 중에도 감지하여 즉시 정지
  (`stop_reason=USER_SWITCHED_TO_MANUAL`), RC 수동 조작으로 복귀.
- 다시 실행하려면 명령을 재실행(AUTO 플립마다 1회 실행, 현재 위치 재앵커).

## 5. 실행 결과를 판독하는 법 (트러블슈팅 결정 트리)

실행 후 `outputs/.../stop_correct_go_trace.csv`를 본다.

1. **코너가 둥글다/덜 돈다** → `phase=connector` 행에서
   `applied_turn_delta_deg` vs `requested_turn_angle_deg` 비교.
   - `turn_pulse_index`가 `turn_pulse_budget`까지 갔는데 미달이면 모터 출력
     부족: B를 키우거나 ms를 늘리고 `target_angle_deg` 재실측.
   - `turn_measured_by_imu=False`면 IMU 미수신 문제부터 해결.
2. **레인이 비스듬하다** → 레인 행 `heading_error_deg`가 큰데
   `phase=correction`이 안 나오면 `--heading-correction-threshold-deg`가
   너무 큰 것. `heading_reference`가 `per_lane`이면 오차가 0으로 숨겨진다.
3. **레인을 벗어나도 복귀 안 함** → `cross_track_error_m` 대비
   `--cross-track-correction-threshold-m`이 큰 것(레인 간격보다 작게).
4. **회전이 끝없이 반복** → summary의 `connector_pulse_count`와
   `rotation budget` 확인; `--max-connector-pulses-per-turn`이 가드.
5. **A와 B가 동시에 나가는 코너** → connector 펄스는 항상 `a=0.000`이어야
   한다(`move_a_cmd` 확인). 아니면 calibration JSON의 turn 항목 `a`를 0으로.

summary.json에서 보는 핵심 카운터:
`connector_turn_count / connector_completed_count / connector_incomplete_count`,
`heading_correction_count`, `turn_calibration_angle_policy`,
`turn_left_90_target_angle_deg`, `turn_right_90_target_angle_deg`,
`turn_angle_warnings`.

## 6. 코드 변경 시 지켜야 할 것 (Claude용 체크리스트)

- 모든 summary에 `ready_for_full_path_following=false`가 유지되어야 한다
  (`checks.assert_not_ready_for_full_path_following`이 강제).
- 기존 CLI 플래그 의미를 바꾸지 말 것. 새 동작은 새 플래그 + 안전한 기본값.
- 시리얼이 필요 없는 로직은 순수 함수로 빼고 `FakeSerial`
  (`tests/test_ppp_stop_correct_go.py` 패턴)으로 테스트를 붙인다.
- 검증 순서: `uv run pytest -q` 전체 통과 →
  `bash -n scripts/run_physical_path_planner.sh` →
  `uv run python -m tools.physical_path_planning.cli <mode> --help` →
  필요 시 `/tmp`에 preview 스모크 실행(PNG 확인).
- outputs/ 아래 산출물과 raw 로그는 커밋하지 않는다.
- 회전/방향 코드를 바꾸기 전 `docs/known_issues.md`의 방향 매핑 항목을 읽는다.

## 7. git 워크플로

```bash
# 스테이션(Mac)에서 최신 작업 받기
cd ~/Desktop/project-lab/gps_hc12_robot
git fetch origin
git checkout claude/epic-rubin-dbha7t   # 이 리팩토링이 들어있는 브랜치
git pull origin claude/epic-rubin-dbha7t

# main에 합쳐졌다면
git checkout main && git pull origin main
```

Claude 원격 세션은 같은 브랜치에 커밋/푸시한다. 필드 테스트 결과(트레이스
CSV, summary)는 필요한 부분만 발췌해 새 세션에 붙여 넣으면 된다.

## 8. 용어 사전 (대화 ↔ 코드)

| 대화 표현 | 코드/문서 용어 |
|---|---|
| ㄹ자 경로 | `coverage_lawnmower` (path_shape) |
| 코너/꺾기 | `connector_turn` 세그먼트, `phase=connector` |
| 약간 전진/후진(스텝) | `step_lane` 세그먼트 (`step_spacing_m` 길이) |
| 작은 회전 펄스 | turn primitive 1회 = `target_angle_deg`만큼 회전 |
| 멈추고-보정-가기 | `--path-control-mode stop_correct_go` |
| 스위치 자동 실행 | `auto-relative-run` (CH5 AUTO 플립) |
| 수동 전환 즉시 정지 | `USER_SWITCHED_TO_MANUAL` abort |
| 진행 기록 | `stop_correct_go_trace.csv`, `summary.md/json` |
