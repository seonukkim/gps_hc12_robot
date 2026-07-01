"""Integrated physical path-planning package.

Single home for A->B serpentine geometry, calibration resolution, telemetry and
safety parsing, guarded pulse execution, the continuous-motion controller, and the
unified CLI (modes: preview, calibrate-turn, execute-plan, run, diagnose).

Import edges are one-way (leaves first):
  geometry, calibration, telemetry, safety, checks -> executor -> controller -> cli
Nothing imports controller/cli back.

목적/역할 (KO):
    이 패키지는 PC(랩톱) 쪽에서 로버의 물리 경로를 "감독(supervised)" 방식으로
    실행한다. A->B 뱀형(serpentine)/직선 경로 기하, 보정(calibration) 해석,
    텔레메트리·안전 파싱, 가드된 펄스(guarded pulse) 실행, 연속 이동 컨트롤러,
    그리고 통합 CLI를 한 곳에 모아 둔 상위 패키지 네임스페이스이다. 이 파일
    자체는 실행 코드가 없는 패키지 마커이며, 하위 모듈들의 임포트 계약
    (import contract)과 배치를 문서화하는 역할만 한다.

시스템 내 위치 / 임포트 방향 (KO):
    임포트 간선은 한 방향(잎 모듈 먼저)이다:
        geometry, calibration, telemetry, safety, checks
            -> executor -> controller -> cli
    controller/cli 를 거꾸로 임포트하는 모듈은 없다. 이 단방향성은 순환 임포트
    (circular import)를 막고, 하위 계층(잎)이 상위 계층을 모른 채 단위 테스트될
    수 있게 하는 핵심 불변식이다. 리팩터링 시 이 방향을 절대 역전시키지 말 것.

핵심 불변식 (KO):
    - executor 는 가드된 펄스 직렬 FSM(ARM->ACK->STOP)을 소유한다.
    - safety/checks 는 ``ready_for_full_path_following=false`` 불변식을 강제한다.
      즉 어떤 모드도 "전체 경로 자율 추종 준비 완료"를 주장할 수 없다.
    - 실제 모터 출력 안전 게이트는 여전히 펌웨어 쪽에 있다. 이 패키지는 명령을
      순서화하고 텔레메트리를 감시할 뿐이다.

Purpose / role (EN):
    PC-side supervised path executor. This ``__init__`` is a code-free package
    marker whose sole job is to document the one-way import contract and the
    layout of the leaf/mid/top modules. The load-bearing invariants:
    ``executor`` owns the guarded-pulse serial FSM (ARM->ACK->STOP);
    ``safety``/``checks`` enforce ``ready_for_full_path_following=false`` so no
    mode can ever claim full-path-following readiness; the real motor-output
    safety gate stays in firmware.
"""
