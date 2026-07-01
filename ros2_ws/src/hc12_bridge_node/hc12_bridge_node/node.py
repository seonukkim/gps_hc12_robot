"""hc12_bridge_node — HC-12 시리얼↔ROS2 브리지 노드 스켈레톤 / HC-12 serial-to-ROS2 bridge skeleton.

목적/역할:
    미래 ROS2(Jazzy) 스테이션 스택에서 HC-12 UART 라디오 프레임을 ROS2 토픽·서비스로
    중계할 브리지 노드의 **자리표시자(스켈레톤)**. 현재는 실제 시리얼 입출력이 전혀 없고,
    1초 주기 타이머로 debug 로그만 찍어 생존을 표시한다. (README "Planned ROS2 migration":
    hc12_bridge_node = "bridge HC-12 serial frames into ROS2 topics/services".)

시스템 내 위치:
    - 지금의 실제 HC-12 통신은 통합 CLI와 `tools/`(예: `station_hc12_test.py`,
      `station_controller.py`)에서 이뤄지며, 이 노드는 **그 워크플로에 포함되지 않는다**.
    - 기본 USB 시리얼 장치는 `/dev/ttyACM0`이고 시리얼 도구는 `--port`를 노출한다
      (AGENTS.md). 실제 구현 시 이 노드도 동일한 포트/프로토콜 가정을 따라야 한다.
    - `setup.py`의 console_scripts 진입점 `hc12_bridge_node`가 아래 `main()`을 가리킨다.

핵심 개념·불변식:
    - **안전 기본값**: 스테이션 측은 heartbeat와 `STOP`만 보내야 하며, 시작 시 살아있는
      `AUTO`(모터 구동) 명령을 보내면 안 된다(AGENTS.md). 스켈레톤은 아무것도 송신하지
      않아 이 불변식을 자연히 만족한다.
    - HC-12 프로토콜·STOP·dry-run 워크플로가 안정화되기 전에는 ROS2 런타임 동작을
      도입하지 않는다는 제약 때문에 현재는 의도적 무동작이다.

사용법/진입점:
    - `main()`이 rclpy 초기화→spin→정리를 수행한다. 현 단계에선 존재 확인용일 뿐 실제
      브리지 기능은 없다.

리팩토링 노트:
    - 실제 구현 시 시리얼 프레이밍/파싱은 ROS2와 독립적인 코어 프로토콜 모듈을 재사용하고,
      이 노드는 그것을 토픽/서비스로 감싸는 얇은 어댑터로 유지할 것.

EN: Skeleton/placeholder for a future ROS2 (Jazzy) station-stack node meant to bridge HC-12
    UART radio frames into ROS2 topics/services. It currently does nothing but emit a 1 Hz
    debug heartbeat; it is NOT part of the current unified-CLI/`tools/` HC-12 workflow. Safe
    station defaults (heartbeat + STOP only, never live AUTO on start) are preserved trivially
    because the skeleton transmits nothing. `main()` is the console_scripts entry point.
"""

import rclpy
from rclpy.node import Node


class SkeletonNode(Node):
    """살아있음만 알리는 자리표시자 ROS2 노드 / placeholder ROS2 node that only signals liveness.

    1초 주기 타이머로 debug 로그를 출력할 뿐, HC-12 시리얼 브리지 로직은 없다. 실제 기능은
    ROS2 마이그레이션 때 채워진다.
    EN: Emits a periodic debug heartbeat only; no HC-12 serial-bridge logic yet.
    """

    def __init__(self) -> None:
        """노드 초기화 및 1Hz 하트비트 타이머 등록 / init node and register a 1 Hz heartbeat timer."""
        super().__init__("hc12_bridge_node")
        # 1초마다 생존 로그 / liveness log every second
        self.create_timer(1.0, self._tick)

    def _tick(self) -> None:
        """타이머 콜백: 생존 debug 로그 1건 / timer callback: one liveness debug log line."""
        self.get_logger().debug("hc12_bridge_node alive")


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
