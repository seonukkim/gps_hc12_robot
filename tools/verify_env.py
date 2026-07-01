"""로컬 개발 환경 자가 점검 도구 — ``make verify`` 가 실행한다.
Environment self-check tool; the entry point behind ``make verify``.

목적/역할:
    작업 전 환경이 준비됐는지 한눈에 확인한다: Python/uv 버전, ROS_DISTRO, 꽂혀 있는
    시리얼 장치, 그리고 필수 파이썬 모듈(pyserial, pynmea2, matplotlib, pandas,
    gps_coverage_core.*)이 실제로 import 되는지. 하나라도 import 실패하면 종료 코드 1을
    반환하므로 스크립트/CI 에서 게이트로 쓸 수 있다.
    Prints Python/uv/ROS info, lists serial devices, and verifies each required
    module imports. Returns exit code 1 if any import fails, so it can gate CI.

시스템 내 위치:
    ``Makefile`` 의 ``verify`` 타깃이 ``uv run python tools/verify_env.py`` 로 호출한다.
    맨 위에서 ``import _bootstrap`` (bare) 를 수행해 repo 루트를 ``sys.path`` 에 넣고,
    그 덕분에 아래 ``gps_coverage_core.*`` import 점검이 성사된다.
    Invoked by the Makefile ``verify`` target; imports ``_bootstrap`` first to
    put the repo root on the path so the ``gps_coverage_core.*`` checks resolve.

핵심 개념·불변식:
    - ``import _bootstrap`` 는 bare(패키지 접두사 없음)라, 이 파일은 ``tools/`` 가 이미
      ``sys.path`` 에 있는 상태(예: ``python tools/verify_env.py`` 로 실행)에서 동작함을
      전제한다 — 다른 도구들의 try/except 관용구와 다르니 주의 (함정).
    - 반환값 계약: import 실패 목록이 비어 있으면 0, 아니면 1. 그 외 진단(버전·장치)은
      정보 출력일 뿐 종료 코드에 영향을 주지 않는다.
    - Invariant: uses a bare ``import _bootstrap`` (assumes ``tools/`` already on
      path); exit code is 0 only when every listed module imports.

리팩토링 노트:
    점검할 모듈 목록은 ``main`` 안에 하드코딩돼 있다. 새 런타임 의존성을 추가하면 여기에도
    넣어야 ``make verify`` 가 그 부재를 잡아낸다.
    The checked-module list is hardcoded in ``main``; add new runtime deps here so
    ``make verify`` catches their absence.
"""
from __future__ import annotations

import argparse
import glob
import importlib
import os
import subprocess
import sys

import _bootstrap  # noqa: F401


def build_parser() -> argparse.ArgumentParser:
    """``--port`` 기본값만 받는 인자 파서를 만든다. / Build the argparse parser (only ``--port``)."""
    parser = argparse.ArgumentParser(description="Print local environment diagnostics.")
    parser.add_argument("--port", default="/dev/ttyACM0", help="Expected default serial device")
    return parser


def main() -> int:
    """환경 진단을 출력하고, 필수 모듈 import 실패 시 1을 반환한다.
    Print diagnostics; return 1 if any required module fails to import, else 0."""
    args = build_parser().parse_args()
    print(f"Python: {sys.version.split()[0]}")

    try:
        uv_version = subprocess.run(
            ["uv", "--version"],
            check=False,
            capture_output=True,
            text=True,
        )
        print(f"uv: {uv_version.stdout.strip() or uv_version.stderr.strip() or 'not available'}")
    except FileNotFoundError:
        print("uv: not found")

    print(f"ROS_DISTRO: {os.getenv('ROS_DISTRO', 'unset')}")

    serial_devices = sorted(set(glob.glob("/dev/ttyACM*") + glob.glob("/dev/ttyUSB*")))
    print(f"Serial devices: {serial_devices or 'none found'}")
    print(f"Default expected port: {args.port}")

    modules = [
        "serial",
        "pynmea2",
        "matplotlib",
        "pandas",
        "gps_coverage_core.protocol",
        "gps_coverage_core.geo",
        "gps_coverage_core.planner",
    ]
    missing = []
    for module_name in modules:
        try:
            importlib.import_module(module_name)
            print(f"import OK: {module_name}")
        except Exception as exc:  # pragma: no cover - environment diagnostics
            missing.append(module_name)
            print(f"import FAIL: {module_name}: {exc}")

    return 1 if missing else 0


if __name__ == "__main__":
    raise SystemExit(main())
