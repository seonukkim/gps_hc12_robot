"""coverage_planner_node — ROS2 커버리지 경로 노드 스켈레톤 / coverage-path ROS2 node skeleton.

목적/역할:
    미래 ROS2(Jazzy) 스테이션 스택에서 커버리지(도장/청소) 경로 생성을 ROS2
    토픽·서비스로 노출할 노드의 **자리표시자(스켈레톤)**. 현재는 실제 계획 로직이
    전혀 없고, 1초 주기 타이머로 debug 로그만 찍어 노드가 살아 있음을 표시한다.
    (README "Planned ROS2 migration" 참고: coverage_planner_node = "expose coverage
    path generation through ROS2".)

시스템 내 위치:
    - 지금의 실제 워크플로는 통합 CLI(`scripts/run_physical_path_planner.sh` /
      `gps_coverage_core`, `tools/`)로 동작하며, 이 노드는 **그 경로에 포함되지 않는다**.
    - 실 계획 로직은 ROS2와 독립적인 코어 모듈에 있고(AGENTS.md: "Keep core protocol
      and planning modules independent from ROS2"), 이 노드는 마이그레이션 시 그것을
      감싸 ROS2 인터페이스로 재노출하도록 채워질 예정이다.
    - `setup.py`의 console_scripts 진입점 `coverage_planner_node`가 아래 `main()`을 가리킨다.

핵심 개념·불변식:
    - 스켈레톤 단독으로는 부작용이 없다(모터 명령·시리얼 송신 없음). AGENTS.md의
      "Do not introduce ROS2 runtime behavior ... until the simple HC-12 protocol ...
      are stable" 제약을 지키기 위한 의도적 무동작 상태다.
    - `SkeletonNode`/`main`은 네 개 노드 패키지에서 노드 이름 문자열만 다르고 구조는 동일하다.

사용법/진입점:
    - `main()`이 rclpy 초기화 → spin → 정리를 수행한다. `ros2 run coverage_planner_node
      coverage_planner_node` 또는 `python node.py`로 실행할 수 있으나, 현 단계에선 존재
      확인용일 뿐 실제 계획 기능은 없다.

리팩토링 노트:
    - 실제 구현 시 퍼블리셔/서비스/파라미터를 추가하되, 계획 알고리즘 자체는 코어 모듈에
      두고 여기서는 얇은 ROS2 어댑터만 유지하는 결합 최소화 원칙을 지킬 것.

EN: Skeleton/placeholder for a future ROS2 (Jazzy) station-stack node meant to expose
    coverage-path generation over ROS2 topics/services. It currently does nothing but emit
    a 1 Hz debug heartbeat log; it is NOT part of the current unified-CLI workflow, which
    runs through the plain-Python core and `tools/`. Planning logic is kept ROS2-independent
    in core modules; when implemented this node should stay a thin ROS2 adapter over them.
    `main()` is the console_scripts entry point declared in setup.py.
"""

import rclpy
from rclpy.node import Node


class SkeletonNode(Node):
    """살아있음만 알리는 자리표시자 ROS2 노드 / placeholder ROS2 node that only signals liveness.

    1초 주기 타이머로 debug 로그를 출력할 뿐, 커버리지 계획·통신 로직은 없다. 실제 기능은
    ROS2 마이그레이션 때 채워진다.
    EN: Emits a periodic debug heartbeat only; no coverage-planning or comms logic yet.
    """

    def __init__(self) -> None:
        """노드 초기화 및 1Hz 하트비트 타이머 등록 / init node and register a 1 Hz heartbeat timer."""
        super().__init__("coverage_planner_node")
        # 1초마다 생존 로그 / liveness log every second
        self.create_timer(1.0, self._tick)

    def _tick(self) -> None:
        """타이머 콜백: 생존 debug 로그 1건 / timer callback: one liveness debug log line."""
        self.get_logger().debug("coverage_planner_node alive")


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
