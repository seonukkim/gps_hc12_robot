"""The shipped ``configs/*.json`` must load and stay in lockstep with the code.

Three guarantees: every shipped config loads via ``cli.load_planner_config``
(comment keys stripped); ``calibration_default.json`` matches the resolver's
in-code fallback set exactly (so the documented fallback can't silently drift);
and ``field_rectangle_example.json`` is a real, buildable A->B-diagonal plan --
its keys are ``build_preview`` kwargs, so it runs directly and yields a guarded,
never-ready summary.

목적/역할 (KO):
    저장소에 동봉된 ``configs/*.json`` 예시 파일들이 (1) 로더로 실제 로드되고,
    (2) 코드와 어긋나지 않게 동기 상태를 유지하는지 못 박는다. 문서/예시 설정이
    조용히 코드와 갈라져(drift) 사용자가 못 쓰는 예시를 받는 사고를 막는 회귀
    가드다.

핵심 계약·불변식 (KO):
    - 모든 동봉 설정은 ``cli.load_planner_config`` 로 dict 로 로드되며, ``_`` 로
      시작하는 주석용 키(JSON 에는 주석이 없어 이를 대용)는 로더가 제거한다.
    - ``calibration_default.json`` 의 값들은 리졸버의 in-code 폴백 집합
      (``geometry.FALLBACK_RESOLVED_CALIBRATION``) 과 정확히 일치해야 한다 --
      문서화된 폴백이 코드 폴백과 소리 없이 갈라지면 실패한다.
    - ``field_rectangle_example.json`` 의 키는 곧 ``preview.build_preview`` 의
      kwargs 이므로 그대로 언팩해 실행 가능하며, 결과는 항상 가드된(절대
      ready 아님) A->B 대각 직사각형 서펜타인 계획이다.

Purpose (EN):
    Regression guard that the bundled example configs both load and never drift
    out of lockstep with the code -- the calibration example must equal the
    in-code fallback set, and the field example must be a directly runnable
    ``build_preview(**config)`` that yields a guarded, never-ready plan.
"""
from __future__ import annotations

from pathlib import Path

import pytest

from tools.physical_path_planning import cli, geometry, preview

_CONFIGS = Path("configs")


@pytest.mark.parametrize(
    "name",
    [
        "physical_path_planning_default.json",
        "calibration_default.json",
        "field_rectangle_example.json",
    ],
)
def test_shipped_config_loads_as_object(name: str) -> None:
    """동봉된 각 설정 파일이 dict 로 로드되고 주석 키(``_``)는 제거된다 / each
    shipped config loads as a dict and comment keys (``_``-prefixed) are stripped."""
    config = cli.load_planner_config(_CONFIGS / name)
    assert isinstance(config, dict)
    # Comment keys are stripped by the loader.
    assert not any(key.startswith("_") for key in config)


def test_calibration_default_matches_resolver_fallback() -> None:
    """calibration_default.json 이 리졸버의 in-code 폴백과 정확히 일치한다 --
    문서화된 폴백의 소리 없는 drift 방지 / the calibration example equals the
    resolver's in-code fallback set exactly, preventing silent drift."""
    config = cli.load_planner_config(_CONFIGS / "calibration_default.json")
    fallback = geometry.FALLBACK_RESOLVED_CALIBRATION
    for motion in ("forward", "backward", "turn_left", "turn_right"):
        primitive = fallback[motion]
        assert config[motion]["a"] == pytest.approx(primitive["a"])  # type: ignore[index]
        assert config[motion]["b"] == pytest.approx(primitive["b"])  # type: ignore[index]
        assert config[motion]["ms"] == primitive["ms"]  # type: ignore[index]
    assert config["connector_mode_effective"] == fallback["connector_mode_effective"]
    assert config["fallback_to_repeated_pulses"] is True
    assert config["left_fixed_pulses"] == fallback["left_fixed_pulses"]
    assert config["right_fixed_pulses"] == fallback["right_fixed_pulses"]
    assert config["ready_for_full_path_following"] is False


def test_field_rectangle_example_builds_a_guarded_plan() -> None:
    """field_rectangle_example.json 을 build_preview 로 곧장 실행하면 가드된(절대
    ready 아님) 대각 직사각형 서펜타인 계획이 나온다 / the field example runs
    directly through ``build_preview`` and yields a guarded, never-ready
    diagonal-rectangle serpentine plan."""
    config = cli.load_planner_config(_CONFIGS / "field_rectangle_example.json")
    # Width must be the short side of the diagonal for the example to be valid.
    assert config["workspace_width_m"] < 8.0
    plan = preview.build_preview(**config)  # keys are build_preview kwargs
    assert plan["ready_for_full_path_following"] is False
    assert plan["path_shape"] == "diagonal_rectangle_serpentine"
    assert plan["lane_count"] >= 1
    assert float(plan["diagonal_length_m"]) > float(config["workspace_width_m"])
