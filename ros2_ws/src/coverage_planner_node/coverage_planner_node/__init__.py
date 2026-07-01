"""coverage_planner_node 패키지 초기화 / package init for the coverage_planner_node ROS2 skeleton.

목적/역할:
    ROS2(Jazzy) 커버리지 경로 노드 스켈레톤 패키지의 파이썬 패키지 마커. 실제 노드 로직은
    `coverage_planner_node.node` 모듈(진입점 `main`)에 있고, 이 패키지는 아직 공개 API를
    노출하지 않는 **자리표시자**다. 현재 통합 CLI 워크플로에는 포함되지 않는다.

시스템 내 위치:
    - `setup.py`가 이 디렉터리를 파이썬 패키지로 포장하고, console_scripts가
      `coverage_planner_node.node:main`을 실행 진입점으로 등록한다.

리팩토링 노트:
    - 공개 심볼을 내보낼 때 `__all__`에 추가해 명시적 공개 계약을 유지할 것.

EN: Package marker for the coverage_planner_node ROS2 (Jazzy) skeleton. Node logic lives in
    `coverage_planner_node.node` (entry point `main`); this package exposes no public API yet and
    is not part of the current unified-CLI workflow.
"""

# 공개 심볼 없음(스켈레톤) / no public symbols exported yet (skeleton)
__all__ = []
