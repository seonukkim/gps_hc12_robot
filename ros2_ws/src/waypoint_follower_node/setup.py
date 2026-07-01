"""waypoint_follower_node ament_python 패키징 설정 / ament_python packaging for the ROS2 skeleton.

목적/역할:
    웨이포인트 추종/명령 생성 노드 스켈레톤 패키지를 `colcon`/`ament_python`으로 빌드·설치
    하기 위한 setuptools 설정. 이 파일 자체는 노드가 아니라 **빌드 메타데이터**이며, 실제
    노드 스켈레톤은 `waypoint_follower_node.node`에 있다. 이 노드는 로버 구동 명령을 산출할
    위치라 가장 민감하며, 현재 dry-run 중심 통합 CLI 워크플로와 무관하다.

시스템 내 위치:
    - `package.xml`(build_type: ament_python)과 짝을 이루며, `colcon build`가 이 setup을
      호출한다. `ros2 run waypoint_follower_node waypoint_follower_node`로 실행할 수 있게
      console_scripts 진입점을 등록한다.

핵심 개념·불변식:
    - `data_files`의 ament resource index 등록과 `package.xml` 설치는 ROS2가 패키지를
      인식하는 데 필수다. 함부로 제거하면 `ros2 run`/`ament` 탐색이 깨진다.

리팩토링 노트:
    - 네 스켈레톤 패키지의 setup.py는 `package_name`만 다르고 구조가 동일하다. 한 곳을 바꾸면
      나머지도 일관되게 맞출 것.

EN: setuptools/ament_python build config so `colcon build` can package and install the
    waypoint_follower_node ROS2 (Jazzy) skeleton and register its `ros2 run` entry point. This is
    build metadata, not the node itself; it is unrelated to the current unified-CLI workflow.
"""

from setuptools import setup

package_name = "waypoint_follower_node"

setup(
    name=package_name,
    version="0.0.0",
    packages=[package_name],
    data_files=[
        # ament 리소스 인덱스 등록 + package.xml 설치: ROS2 패키지 탐색에 필수
        # / register ament resource index + install package.xml so ROS2 can discover the package
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Codex",
    maintainer_email="noreply@example.com",
    description="Minimal ROS2 Jazzy node skeleton for gps_hc12_robot.",
    license="MIT",
    entry_points={
        # `ros2 run <pkg> <exe>`가 호출할 콘솔 스크립트 → node.py의 main()
        # / console script invoked by `ros2 run`; points at node.py's main()
        "console_scripts": [
            package_name + " = " + package_name + ".node:main",
        ],
    },
)
