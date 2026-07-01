"""station_mission_node — 스테이션 미션/운영 워크플로 노드 스켈레톤 / station mission-state node skeleton.

목적/역할:
    미래 ROS2(Jazzy) 스테이션 스택에서 운영자 워크플로와 미션 상태(점 A/B 선택, 스윕 폭
    입력, 프리뷰/로깅, dry-run 명령 생성 등)를 관리할 노드의 **자리표시자(스켈레톤)**.
    현재는 상태 기계가 전혀 없고, 1초 주기 타이머로 debug 로그만 찍어 생존을 표시한다.
    (README "Planned ROS2 migration": station_mission_node = "manage operator workflow and
    mission state".)

시스템 내 위치:
    - 지금의 실제 운영 워크플로는 통합 CLI(`scripts/run_physical_path_planner.sh`,
      `tools/station_mock_mission.py` 등)로 동작하며, 이 노드는 **그 경로에 포함되지 않는다**.
    - 미션 오케스트레이션 시 coverage_planner_node(경로 생성)와 hc12_bridge_node(라디오
      송수신)를 조율하는 상위 조정자 역할이 될 예정이다.
    - `setup.py`의 console_scripts 진입점 `station_mission_node`가 아래 `main()`을 가리킨다.

핵심 개념·불변식:
    - **dry-run/미무장(not armed) 기본값 유지**, 경로 생성 직후 로버를 즉시 움직이지 않음,
      `STOP`이 수동·스테이션·미래 자율 명령보다 우선(AGENTS.md). 스켈레톤은 아무 명령도
      내보내지 않아 이 불변식들을 자연히 만족한다.
    - 자율 실행은 A/B 선택·스윕 폭·GPS 텔레메트리·프리뷰 계획이 명시적으로 테스트된
      뒤에야 도입한다.

사용법/진입점:
    - `main()`이 rclpy 초기화→spin→정리를 수행한다. 현 단계에선 존재 확인용일 뿐 실제
      미션 관리 기능은 없다.

리팩토링 노트:
    - 상태 기계를 추가하더라도 안전 기본값(무장 해제, STOP 우선)을 코드 구조로 강제하고,
      계획/프로토콜 로직은 코어 모듈에 위임하는 얇은 조정자 형태를 유지할 것.

EN: Skeleton/placeholder for a future ROS2 (Jazzy) station-stack node meant to manage operator
    workflow and mission state (A/B point selection, sweep width, preview/logging, dry-run
    command generation). It currently does nothing but emit a 1 Hz debug heartbeat; it is NOT
    part of the current unified-CLI workflow. Safe defaults — dry-run/not-armed, no immediate
    motion after planning, STOP overrides everything — hold trivially since the skeleton issues
    no commands. `main()` is the console_scripts entry point.
"""

import rclpy
from rclpy.node import Node


class SkeletonNode(Node):
    """살아있음만 알리는 자리표시자 ROS2 노드 / placeholder ROS2 node that only signals liveness.

    1초 주기 타이머로 debug 로그를 출력할 뿐, 미션 상태 관리 로직은 없다. 실제 기능은
    ROS2 마이그레이션 때 채워진다.
    EN: Emits a periodic debug heartbeat only; no mission-state logic yet.
    """

    def __init__(self) -> None:
        """노드 초기화 및 1Hz 하트비트 타이머 등록 / init node and register a 1 Hz heartbeat timer."""
        super().__init__("station_mission_node")
        # 1초마다 생존 로그 / liveness log every second
        self.create_timer(1.0, self._tick)

    def _tick(self) -> None:
        """타이머 콜백: 생존 debug 로그 1건 / timer callback: one liveness debug log line."""
        self.get_logger().debug("station_mission_node alive")


def main() -> None:
    """콘솔 스크립트 진입점: rclpy 기동→spin→정리 / console-script entry: init, spin, teardown.

    KeyboardInterrupt(Ctrl-C)로 깔끔히 빠져나오도록 감싸며, finally에서 노드 파괴와
    rclpy 종료를 보장한다(정상·예외 경로 모두 자원 정리).
    EN: Standard rclpy lifecycle; Ctrl-C exits cleanly and teardown always runs.
    """
    rclpy.init()
    node = SkeletonNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        # Ctrl-C는 정상 종료로 취급 / treat Ctrl-C as a clean shutdown
        pass
    finally:
        # 예외 여부와 무관하게 자원 정리 보장 / always release resources, even on error
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
