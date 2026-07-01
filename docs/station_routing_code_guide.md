# 스테이션(PC) 라우팅/구동 코드 안내

> 이 문서는 "PC에 들어가는 라우팅 파이썬 코드가 무엇이고, 어떻게 실행하며,
> 각 파일이 어떤 역할을 하고, 파이썬(.py)과 아두이노(.ino)가 어떻게
> 연결되는지"를 한 장으로 정리한 것입니다. 저장소:
> `github.com/seonukkim/gps_hc12_robot`.

---

## 0. 한 줄 요약

- **로버 본체**는 OpenRB-150(아두이노 호환 보드)이고, 그 안에서 도는 코드는
  `firmware/openrb_robot_controller/openrb_robot_controller.ino` **단 하나**입니다.
- **PC(스테이션, 보통 Mac)에서 도는 "라우팅 파이썬 코드"의 실체**는
  `tools/physical_path_planning/` 패키지이며, 진입점은
  `scripts/run_physical_path_planner.sh` → `tools/physical_path_planning/cli.py`
  입니다.
- **PC와 로버는 USB 시리얼 케이블 하나로 연결**됩니다
  (`/dev/ttyACM0`, 115200 baud). PC가 명령 문자열을 보내고, 로버가
  `USBDBG ...` 텔레메트리 한 줄씩을 돌려주는 구조입니다.
- 순수 경로 계산(위경도↔미터 변환, 왕복 레인 웨이포인트 생성)은
  `gps_coverage_core/planner.py`에 들어 있습니다. **"라우팅 알고리즘"만
  딱 떼서 보고 싶다면 이 파일 하나**를 보면 됩니다.

---

## 1. 큰 그림 (누가 무엇을 실행하고 어떻게 연결되나)

```
┌────────────────────────────┐        USB 시리얼 케이블        ┌──────────────────────────────┐
│  스테이션 PC (Mac)          │   /dev/ttyACM0 · 115200 baud   │  로버: OpenRB-150 (Arduino)   │
│                            │ ─────────────────────────────► │                              │
│  Python: tools/            │   PC→로버: 명령 문자열          │  firmware/openrb_robot_       │
│  physical_path_planning/   │   (SET / STOP / ARM …)          │  controller/*.ino            │
│  (cli.py 진입)             │                                │                              │
│                            │ ◄───────────────────────────── │   ├─ 모터 2개(좌/우 ESC)      │
│  gps_coverage_core/        │   로버→PC: "USBDBG key=val …"   │   ├─ GPS (NMEA)              │
│  (경로 계산 라이브러리)     │   텔레메트리 한 줄/0.5초         │   ├─ IMU BMI160 (상대 yaw)   │
└────────────────────────────┘                                │   ├─ RC 수신기 (PPM, CH1/2/5)│
                                                              │   └─ HC-12 (무선 UART, 옵션) │
                                                              └──────────────────────────────┘
```

두 가지 운용 방식이 있습니다. 다른 분이 말한 "PC 라우팅 코드"는 대개 **①번**입니다.

**① 유선 방식 — 두뇌가 PC에 있음 (`auto-relative-run`, `run`, `execute-plan`)**
- 경로 계산·판단·보정을 전부 PC의 파이썬이 하고, USB로 로버에 저수준
  구동 명령(`USB_DRIVE_LIVE_SET a=.. b=..`)을 실시간으로 보냅니다.
- 로버 펌웨어는 "받은 명령을 안전 범위 안에서 모터에 그대로 실행 +
  센서값을 텔레메트리로 회신"하는 역할만 합니다.
- USB를 뽑으면 데드맨(dead-man) 안전장치가 즉시 정지시킵니다(설계 의도).

**② 무선 방식 — 두뇌가 로버에 있음 (`rc-auto-pattern`)**
- PC의 파이썬으로 **온보드 패턴 펌웨어를 한 번 업로드**만 하면, 이후엔
  USB를 뽑고 RC 송신기만으로 동작합니다. 경로(ㄹ자 왕복)를 로버가
  스스로 계산·실행합니다.
- 이때 PC 파이썬은 "빌드+업로드 도구"로만 쓰이고, 주행 중에는 개입하지
  않습니다.

---

## 2. 설치와 실행 진입점

### 2-1. 의존성 설치 (PC, 최초 1회)

```bash
# Python 3.12, uv 패키지 매니저 사용
uv sync --extra dev          # 또는: pip install -r requirements.txt
```

핵심 파이썬 의존성: `pyserial`(시리얼), `pynmea2`(GPS NMEA 파싱),
`numpy/scipy/pyproj/geographiclib`(좌표·기하), `matplotlib`(경로 미리보기
PNG). 전체 목록은 `pyproject.toml` / `requirements.txt`.

로버에 펌웨어를 업로드하는 모드를 쓰려면 `arduino-cli`와 OpenRB-150 보드
코어(`OpenRB-150:samd:OpenRB-150`)가 PC에 설치돼 있어야 합니다.

### 2-2. 모든 실행의 공통 진입점

```bash
bash scripts/run_physical_path_planner.sh <모드> [옵션들]
```

이 셸 스크립트는 얇은 디스패처이고, 실제 로직은
`tools/physical_path_planning/cli.py`의 각 `cmd_*` 함수에 있습니다. 도움말:

```bash
bash scripts/run_physical_path_planner.sh --help
```

주요 모드(= 서브커맨드)와 역할:

| 모드 | 모터 구동? | 역할 |
|---|---|---|
| `diagnose` | ✗ | 텔레메트리만 읽어 요약(연결/센서 확인). |
| `gps-wait` | ✗ | 쓸 만한 GPS 픽스가 잡힐 때까지 대기. |
| `rc-input-diagnose` | ✗ | RC 수신기 채널 입력 진단. |
| `manual-control` | (RC) | PPM 수동 조작 펌웨어 업로드·모니터. |
| `preview` | ✗ | ㄹ자 커버리지 경로 계획 + PNG 렌더. |
| `inspect-plan` | ✗ | 저장된 플랜/이미지 확인. |
| `tune-motion` | ✓ | 대화형 모터 캘리브레이션(전진/후진/회전). |
| `calibrate-turn` | ✓ | 회전 각도 캘리브레이션(IMU yaw 비교). |
| `calibration-check` | ✗ | 캘리브레이션 완성도 점검. |
| `align-heading` | ✓ | 첫 레인 방향으로 로버를 정렬. |
| `run` | ✓ | 경로를 즉석 계획 후 유선 실행. |
| `execute-plan` | ✓ | 저장된 플랜을 유선 실행. |
| `auto-relative-run` | ✓ | RC의 AUTO 스위치를 기다렸다가, 현재 GPS를 원점으로 상대 경로를 1회 유선 실행. |
| `rc-auto-pattern` | ✓ | **무선** 온보드 ㄹ자 패턴 펌웨어 업로드(이후 USB 불필요). |

> 안전 원칙: 모든 실행 결과 요약에는 `ready_for_full_path_following=false`가
> 강제됩니다(`checks.py`). 즉 "완전 자율 주행"은 아직 활성화되지 않은
> 상태이며, 모든 구동은 감독·가드 하에서만 일어납니다.

---

## 3. 파일별 역할 지도

### 3-1. `tools/physical_path_planning/` — PC 라우팅/구동의 본체

임포트 방향은 한 방향(리프 먼저)입니다:
`geometry, calibration, telemetry, safety, checks → executor → controller → cli`

| 파일 | 역할 |
|---|---|
| `cli.py` | 사용자 대면 CLI. 모든 모드 구현, **시리얼 포트 오픈**, 펌웨어 빌드/업로드 호출, 결과 요약(JSON) 기록. (가장 큼) |
| `geometry.py` | ㄹ자/직사각형 커버리지 **경로 기하 생성**. 코너를 `connector_turn → step_lane → connector_turn`(turn_step_turn)로 분해. 각 세그먼트에 목표 바디 헤딩/회전각을 부여. |
| `calibration.py` | 전진/후진/회전 모터 캘리브레이션 값 정규화·해석. 회전 펄스의 실제 각도(`target_angle_deg`) 관리. |
| `telemetry.py` | 로버가 보내는 `USBDBG ...` 라인을 dict로 **파싱**하고, GPS/IMU/RC/모터 필드 접근자 제공. |
| `executor.py` | **시리얼에 명령을 쓰는 계층.** 가드 펄스 FSM(ARM→ACK→완료대기→STOP)과 명령/텔레메트리 대기 루프. |
| `controller.py` | **경로 실행 감독 루프(`stop_correct_go`).** "한 청크 구동→정지→안정된 GPS/IMU 읽기→헤딩 보정"을 반복하며 계획 경로를 따라감. |
| `alignment.py` | 초기 헤딩 정렬(GPS 변위 프로브로 절대 방위 추정 + IMU 피드백 제자리 회전). |
| `tuning.py` | 대화형 캘리브레이션 후보 조정·승인·백업/리셋. |
| `preview.py` | 계획 경로를 **PNG 이미지로 렌더**(현장에서 경로를 눈으로 확인). |
| `safety.py` | ACK/STOP·정지 후 출력 0 확인 등 안전 술어. |
| `checks.py` | 모든 요약에 `ready_for_full_path_following=false` 강제. |

### 3-2. `gps_coverage_core/` — 순수 경로 계산 라이브러리(하드웨어 비의존)

시리얼·모터와 무관한 **순수 함수**들입니다. `physical_path_planning`을 비롯한
여러 도구가 이걸 가져다 씁니다. **"라우팅 알고리즘"의 핵심은 여기입니다.**

| 파일 | 역할 |
|---|---|
| `planner.py` | 위경도↔로컬 x/y(미터) 변환, 스위프 폭에 맞는 **레인 오프셋 계산**, 왕복(serpentine) **웨이포인트 생성**. ← 가장 "라우팅"다운 코드 |
| `geo.py` | 거리·방위 등 지리 계산 헬퍼. |
| `imu.py` | IMU 관련 계산 헬퍼. |
| `nmea.py` | GPS NMEA 문장 파싱. |
| `protocol.py` | **PC↔로버 명령 프레임 포맷·XOR 체크섬·값 클램프**(아래 4장 프로토콜). |
| `telemetry.py` | 텔레메트리 관련 공용 파싱. |
| `side_tool_planner.py` | 보조 도구(사이드 툴) 경로 계획. |

### 3-3. `firmware/openrb_robot_controller/openrb_robot_controller.ino` — 로버 펌웨어

로버 안에서 도는 **유일한** 컨트롤러 소스입니다. 하나의 파일이지만
**컴파일 플래그(-D...)에 따라 여러 모드**로 빌드됩니다(수동 조작, 가드
펄스, 무선 패턴 등). 역할:
- RC 수신기 PPM 디코드(CH1=조향, CH2=스로틀, CH5=MANUAL/AUTO 스위치),
- GPS NMEA 수신, IMU(BMI160) 상대 yaw 적분,
- USB로 들어온 명령 파싱·안전 범위로 클램프 후 모터 실행,
- `USBDBG ...` 텔레메트리 회신,
- 안전 게이트(정지 우선, 출력 0 확인, 데드맨).

> 나머지 `firmware/*_probe`, `*_test` 폴더들은 GPS/IMU/PPM 배선을 점검할
> 때 쓰는 **일회성 진단 스케치**이며 평상시 운용과 무관합니다.

### 3-4. `ros2_ws/` — ROS2 노드(현재 스켈레톤, 평상시 미사용)

`coverage_planner_node`, `hc12_bridge_node`, `station_mission_node`,
`waypoint_follower_node`가 있으나 **뼈대만 있는 상태**(known_issues에
"skeleton-only"로 명시)입니다. 현재 현장 운용은 위의
`physical_path_planning` 경로로 하며, ROS2는 향후 확장용입니다. **지금
전달할 "라우팅 코드"에 ROS2는 포함하지 않아도 됩니다.**

---

## 4. `.py ↔ .ino` 연결 = ① 빌드타임 + ② 런타임 시리얼 프로토콜

두 지점에서 연결됩니다.

### 4-1. 빌드타임: 파이썬이 펌웨어를 빌드·업로드

업로드가 필요한 모드(`manual-control`, `rc-auto-pattern`, `tune-motion`
등)에서 `cli.py`가 `arduino-cli`를 호출해 **같은 `.ino`를 모드별 -D 플래그로
컴파일**하고 보드에 올립니다. 예: 무선 패턴 모드는
`-DRC_AUTO_PATTERN=1 -DMANUAL_CONTROL_PPM=1 -DIMU_ENABLE=1 …` 등을 부여.
즉 "어떤 파이썬 모드로 올렸는가"가 "로버가 어떤 펌웨어 모드로 도는가"를
결정합니다.

### 4-2. 런타임: USB 시리얼 문자열 프로토콜

- 포트/속도: **`/dev/ttyACM0` · 115200 baud**
  (Mac에서는 `/dev/tty.usbmodemXXXX` 형태. `--port`로 지정 가능.)
- `cli.py`가 `serial.Serial(port, 115200, timeout=0.5)`로 포트를 엽니다.

**PC → 로버 (개행 종료 ASCII 명령, `executor.write_command`가 전송)**

| 명령 문자열 | 의미 |
|---|---|
| `USB_DRIVE_LIVE_SET seq=N a=<전후진> b=<조향> ms=<지속> ttl=<타임아웃>` | 연속 구동 setpoint(유선 실시간 주행/보정 회전의 핵심). |
| `USB_DRIVE_LIVE_STOP seq=N` | 연속 구동 정지. |
| `USB_PULSE_TEST_ARM seq=N` → 명령 → `USB_PULSE_TEST_STOP seq=N` | 가드 펄스 1회(ARM→명령→완료→STOP). |
| `STOP` | 즉시 정지. |

- `a`(=physical A)는 전/후진, `b`(=physical B)는 좌/우 조향. 각각 [-1,1]로
  클램프됩니다. 로버 펌웨어가 이를 좌/우 바퀴 명령으로 믹싱합니다.
- 로버 펌웨어는 명령을 한 글자씩 읽어 `\n`에서 한 줄로 조립·해석하고,
  일정 시간 갱신이 없으면(TTL 초과) 스스로 정지합니다(안전).

**로버 → PC (텔레메트리, 약 0.5초마다 한 줄)**

```
USBDBG mode=AUTO_RUNNING control_source=AUTO event=ACK \
  rc_ok=true auto_sw=true manual_switch=AUTO \
  gps_sats=7 gps_hdop=1.2 imu_relative_yaw_deg=172.8 \
  final_left_cmd=0.31 final_right_cmd=0.29 physical_output_active=true …
```

- 공백으로 구분된 `key=value` 나열. `telemetry.parse_usbdbg_rows`가 dict로
  파싱하고, 접근자(`imu_relative_yaw_deg`, `gps_sats`, `event` 등)로 읽습니다.
- `event=ARM/ACK/STOP`은 가드 펄스 FSM의 상태 확인에 쓰입니다.
- 무선 패턴 모드에서는 추가로 `RC_AUTO_PATTERN state=DRIVE step=2/13 …`
  라인도 나옵니다.

정리하면 **파이썬(경로/판단)** 과 **아두이노(모터/센서 실행)** 는
"저수준 구동 명령 ↔ 텔레메트리 회신"이라는 얇은 시리얼 규약으로만
연결됩니다. 경로 계산의 두뇌는 파이썬, 안전한 실행은 펌웨어가 담당하는
역할 분리 구조입니다.

---

## 5. 대표 실행 시나리오 (유선, 차례대로)

로버 USB 연결 상태에서:

```bash
# 1) 연결/센서 확인 (모터 안 움직임)
bash scripts/run_physical_path_planner.sh diagnose \
  --out-dir outputs/physical_path_planning/diagnose

# 2) GPS 픽스 대기
bash scripts/run_physical_path_planner.sh gps-wait \
  --timeout-s 300 --min-sats 5 --max-hdop 2.5 \
  --out-dir outputs/physical_path_planning/gps_wait

# 3) 경로 미리보기(PNG 생성, 모터 안 움직임)
bash scripts/run_physical_path_planner.sh preview \
  --goal-mode relative_enu --goal-east-m 1.8 --goal-north-m 1.8 \
  --workspace-width-m 1.8 --step-spacing-m 0.6 \
  --out-dir outputs/physical_path_planning/preview

# 4) (필요 시) 모터 캘리브레이션
bash scripts/run_physical_path_planner.sh tune-motion --primitive forward \
  --out-dir outputs/physical_path_planning/tune_forward

# 5) 첫 레인 방향 정렬
bash scripts/run_physical_path_planner.sh align-heading \
  --out-dir outputs/physical_path_planning/align

# 6) 유선 실행: 현재 GPS를 원점으로 상대 경로 1회 (RC를 AUTO로 넘기면 시작)
bash scripts/run_physical_path_planner.sh auto-relative-run \
  --goal-east-m 1.8 --goal-north-m 1.8 \
  --workspace-width-m 1.8 --step-spacing-m 0.6 \
  --out-dir outputs/physical_path_planning/auto_run
```

산출물(트레이스 CSV, 요약 JSON, PNG)은 `outputs/` 아래에 쌓이며 저장소에는
커밋하지 않습니다.

무선으로 쓰려면 위 대신 `rc-auto-pattern` 모드로 펌웨어를 한 번 업로드한 뒤
USB를 뽑고 RC 송신기로 운용합니다(상세: `docs/README_physical_path_planning.md`
"Untethered MANUAL/AUTO" 절).

---

## 6. "라우팅 파이썬 코드만" 딱 원한다면 → 이 세 곳

상대가 순수 경로 계산 로직만 보고 싶어 하는 경우 우선순위:

1. **`gps_coverage_core/planner.py`** — 위경도↔미터 변환, 레인 오프셋,
   왕복 웨이포인트 생성(경로 라우팅 알고리즘의 심장).
2. **`tools/physical_path_planning/geometry.py`** — 그 경로를 로버가 실제로
   밟을 수 있는 ㄹ자 세그먼트(직진/코너 회전/스텝)로 분해.
3. **`tools/physical_path_planning/controller.py`** — 그 세그먼트를 GPS/IMU
   피드백으로 보정하며 실행하는 감독 루프.

상대가 "PC가 로버를 어떻게 움직이나(구동 인터페이스)"를 원하는 경우:
**`tools/physical_path_planning/executor.py` + `gps_coverage_core/protocol.py`**
(위 4장의 시리얼 프로토콜).

---

## 7. 참고 문서

- `docs/README_physical_path_planning.md` — 현장 실행 런북(유선/무선).
- `docs/physical_path_planning_architecture.md` — 모듈 아키텍처.
- `docs/claude_collaboration_guide.md` — 제어 의미론·트러블슈팅 결정 트리.
- `docs/protocol.md`, `docs/wiring.md`, `docs/rc_channel_map.md` — 프로토콜·배선·RC 채널.
