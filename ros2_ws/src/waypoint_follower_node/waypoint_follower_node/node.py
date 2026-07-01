"""waypoint_follower_node — 웨이포인트 추종/명령 생성 노드 스켈레톤 / waypoint-follower node skeleton.

목적/역할:
    미래 ROS2(Jazzy) 스테이션 스택에서 계획된 경로(웨이포인트)를 소비해 로버 명령을
    산출할 추종 노드의 **자리표시자(스켈레톤)**. 현재는 추종 로직이 전혀 없고, 1초 주기
    타이머로 debug 로그만 찍어 생존을 표시한다. (README "Planned ROS2 migration":
    waypoint_follower_node = "consume planned paths and produce rover commands".)

시스템 내 위치:
    - 지금의 실제 워크플로는 통합 CLI 기반 dry-run 계획/프리뷰이며, 이 노드는 **그 경로에
      포함되지 않는다**. 실제 로버 구동은 아직 이 스택으로 수행하지 않는다.
    - 파이프라인상 하류(下流) 소비자: coverage_planner_node가 만든 경로를 받아
      hc12_bridge_node를 통해 로버로 나갈 명령으로 변환하는 위치가 될 예정이다.
    - `setup.py`의 console_scripts 진입점 `waypoint_follower_node`가 아래 `main()`을 가리킨다.

핵심 개념·불변식:
    - 이 노드는 실제 모터 구동 명령을 산출하는 곳이므로 **가장 민감**하다. AGENTS.md 제약:
      경로 생성 직후 즉시 이동 금지, `STOP` 최우선, 수동 제어 약화 금지, 모터 시험은
      바퀴를 띄운 상태로만. 스켈레톤은 아무 명령도 내보내지 않아 이를 자연히 만족한다.
    - HC-12 프로토콜·텔레메트리·STOP·dry-run 워크플로가 안정화되기 전에는 ROS2 런타임
      구동 동작을 도입하지 않는다.

사용법/진입점:
    - `main()`이 rclpy 초기화→spin→정리를 수행한다. 현 단계에선 존재 확인용일 뿐 실제
      추종/명령 산출 기능은 없다.

리팩토링 노트:
    - 구동 명령 경로를 구현할 때 STOP 우선순위와 무장(arming) 게이트를 코드 구조로 강제하고,
      헤딩/좌표 변환 등 계산은 코어 모듈에 위임할 것. 수동 제어 오버라이드는 반드시 보존.

EN: Skeleton/placeholder for a future ROS2 (Jazzy) station-stack node meant to consume planned
    paths (waypoints) and produce rover commands — the most safety-sensitive node since it is
    where drive commands would originate. It currently does nothing but emit a 1 Hz debug
    heartbeat; it is NOT part of the current unified-CLI (dry-run planning/preview) workflow.
    Safety invariants (no motion right after planning, STOP overrides all, preserve manual
    control, wheels-off-ground motor testing) hold trivially because the skeleton commands
    nothing. `main()` is the console_scripts entry point.
"""

import rclpy
from rclpy.node import Node


class SkeletonNode(Node):
    """살아있음만 알리는 자리표시자 ROS2 노드 / placeholder ROS2 node that only signals liveness.

    1초 주기 타이머로 debug 로그를 출력할 뿐, 웨이포인트 추종·명령 산출 로직은 없다. 실제
    기능은 ROS2 마이그레이션 때 채워진다.
    EN: Emits a periodic debug heartbeat only; no waypoint-following or command logic yet.
    """

    def __init__(self) -> None:
        """노드 초기화 및 1Hz 하트비트 타이머 등록 / init node and register a 1 Hz heartbeat timer."""
        super().__init__("waypoint_follower_node")
        # 1초마다 생존 로그 / liveness log every second
        self.create_timer(1.0, self._tick)

    def _tick(self) -> None:
        """타이머 콜백: 생존 debug 로그 1건 / timer callback: one liveness debug log line."""
        self.get_logger().debug("waypoint_follower_node alive")


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
