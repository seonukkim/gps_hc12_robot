# 2026-06-11 ㄹ자 코너 리팩토링 보고서

브랜치: `claude/epic-rubin-dbha7t`
커밋: connector 회전 의미론 수정 → turn_step_turn 분해 → 문서화 (3개)
검증: 전체 pytest 스위트 통과(기존 + 신규 테스트), CLI/스크립트 스모크 통과.
하드웨어 검증은 아직 안 됨 — 다음 필드 테스트에서 §5 체크리스트로 확인할 것.

## 1. 문제 (필드에서 관찰된 증상)

2.4m x 2.4m, 1.2m 간격 ㄹ자 실행 시 "ㄹ자로 보이긴 하는데 너무 완만하게 크게
돌아가는" 궤적. 코너가 직각이 아니라 넓은 호.

## 2. 원인 진단 (코드에서 확정)

세 가지가 겹쳐 있었다.

1. **connector가 작은 펄스 1회로 90도를 끝냈다고 가정.**
   - `controller._segment_pulse_budget`: angle-calibrated 모드에서 예산=1펄스.
   - `calibration.connector_primitive`: JSON의 `target_angle_deg`(실제 펄스당
     회전각, 이 로버에선 ~15-45도)를 버리고 전달하지 않음.
   - 회전 후 검증이 `--start-yaw-deg`(평소 없음) 기준이라 오차가 항상 0으로
     읽혀 1펄스 후 "성공" 처리.
2. **레인마다 yaw 기준 재캡처** (`reference_yaw_for_segment`): 코너에서 덜 돈
   45-75도 오차가 다음 레인의 "정상 방향"으로 흡수되어 보정 자체가 안 일어남.
3. **플랜의 connector가 물리적으로 주행 불가능**: `path_connector` 하나가
   1.2m 횡이동을 담고 있지만 실행은 제자리 회전만 함. 스텝오버 이동이 플랜에
   없었음.

추가 운용 문제: 실행 명령의 `--cross-track-correction-threshold-m 1.50`이
레인 간격(1.2m)보다 커서 레인 복귀 보정이 사실상 꺼져 있었다.

## 3. 변경 사항

### 커밋 1 — connector 회전 의미론 + mission heading + MANUAL 즉시 정지

- `calibration.connector_primitive`가 `target_angle_deg` 전달
  (없으면 실측 `imu_yaw_delta_deg`, 그다음 90 폴백).
- `run_stop_correct_go` connector: 시작 yaw 기준 IMU 폐루프 다중 펄스.
  새 플래그 `--max-connector-pulses-per-turn`(6),
  `--connector-turn-tolerance-deg`(10),
  `--turn-calibration-angle-policy from_json|assume_90`,
  `--turn-angle-deg-override`. IMU 없으면 `ceil(각도/펄스각)` 개루프.
- `--heading-reference mission|per_lane`(기본 mission): 미션 전체에 yaw
  프레임 1개를 유지해 코너 미달분이 다음 레인에서 보정되게 함.
- AUTO 실행 중 MANUAL 플립을 펄스 윈도/보정 회전 중에도 감지해 즉시 중단.
- 트레이스/summary에 회전 필드 추가(`requested_turn_angle_deg`,
  `applied_turn_delta_deg`, `connector_turn_completed` 등).
- `calibration-check`/`set-motion-calibration`이 회전각과
  `TURN_CALIBRATION_IS_SMALL_PULSE_NOT_90` 경고를 출력.

### 커밋 2 — turn_step_turn 플랜 분해

- `build_serpentine_segments(..., connector_style="turn_step_turn")` 기본:
  코너 = `connector_turn(±90, 길이0)` → `step_lane(간격, 전진/후진)` →
  `connector_turn(∓90)`. 전진 레인 뒤 스텝은 전진, 후진 레인 뒤는 후진.
- 모든 세그먼트에 `body_heading_deg`, connector에 부호 있는
  `turn_angle_deg` (실제 기하에서 유도 — 스텝 방향이 우측이면 우회전).
- `--connector-style single_turn`으로 레거시 유지; 옛 plan-dir도 실행 가능.
- 카운트 의미 정리: `lane_count`=풀 레인, `connector_count`=전환 수,
  `connector_turn_count`/`step_lane_count` 신설.

### 커밋 3 — 문서화

- `docs/physical_path_planning_architecture.md`: 새 의미론 3개 섹션.
- `docs/README_physical_path_planning.md`: ㄹ 필드 런북 + AUTO/MANUAL 런북.
- `docs/known_issues.md`: 본 이슈를 Resolved로 기록(운용 규칙 포함).
- `docs/claude_collaboration_guide.md`(신규, 한국어): 다음 세션용 협업 가이드.
- 본 보고서.

## 4. 테스트

- 신규: connector 30도 펄스 3회 폐루프 90도 도달 / IMU 없음 개루프 3펄스 /
  회전 루프 가드(예산 소진 시 중단·미완료 기록) / mission vs per_lane에서
  코너 미달 50도가 보정되는지·흡수되는지 비교 / 후진 레인 body heading 유지 /
  펄스 중 MANUAL 플립 즉시 abort / turn_step_turn 세그먼트 구조·각도·방향 /
  single_turn 레거시 구조 / target_angle_deg 전달·경고.
- 기존 스위트 전부 통과(의도적 변경 1건: lawnmower 구조 테스트를 새 기본
  구조로 갱신, 레거시 테스트 별도 추가).

## 5. 다음 필드 테스트 체크리스트

1. `set-motion-calibration --target-angle-deg <실측각>`으로 좌/우 실제 회전각
   저장 (대략 30이면 30).
2. `calibration-check` 출력에서 각도/경고 확인.
3. `preview` (기본 turn_step_turn) → PNG가 스텝오버 포함 ㄹ인지 확인.
4. `execute-plan --path-control-mode stop_correct_go` 권장 파라미터:
   `--heading-correction-threshold-deg 15 --heading-correction-tolerance-deg 8
   --cross-track-correction-threshold-m 0.35 --move-chunk-ms 800`.
5. 코너에서 로봇이 멈춰서 2-4회 끊어 돌고, 1.2m 직진(또는 후진) 후 되돌아
   도는지 관찰.
6. `stop_correct_go_trace.csv`의 `phase=connector` 행에서
   `applied_turn_delta_deg`가 ±90 근처인지, `connector_turn_completed=True`
   비율 확인.
7. `auto-relative-run`으로 CH5 AUTO→실행, MANUAL→즉시 정지 확인.

## 6. 남은 리스크 / 한계

- BMI160 yaw 드리프트: 긴 미션에서 mission heading 누적 오차 가능. 미션이
  수 분을 넘으면 `--heading-reference per_lane`과 비교 관찰 필요.
- 우회전 모터 약함: 펄스 예산(기본 6)으로도 90도 미달이면
  `connector_incomplete_count`로 드러남 → B/ms 재조정 필요.
- 후진 스텝/레인의 cross-track 트림 부호는 기존 동작을 유지했음(임계값
  이하로 운용 권장). 후진 중 B 보정 극성은 필드에서 한 번 확인할 것.
- 하드웨어 미검증: 본 변경은 모의 시리얼 기반 테스트까지만 검증됨.
