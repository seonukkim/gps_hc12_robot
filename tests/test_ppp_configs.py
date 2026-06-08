"""The shipped ``configs/*.json`` must load and stay in lockstep with the code.

Three guarantees: every shipped config loads via ``cli.load_planner_config``
(comment keys stripped); ``calibration_default.json`` matches the resolver's
in-code fallback set exactly (so the documented fallback can't silently drift);
and ``field_rectangle_example.json`` is a real, buildable A->B-diagonal plan --
its keys are ``build_preview`` kwargs, so it runs directly and yields a guarded,
never-ready summary.
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
    config = cli.load_planner_config(_CONFIGS / name)
    assert isinstance(config, dict)
    # Comment keys are stripped by the loader.
    assert not any(key.startswith("_") for key in config)


def test_calibration_default_matches_resolver_fallback() -> None:
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
    config = cli.load_planner_config(_CONFIGS / "field_rectangle_example.json")
    # Width must be the short side of the diagonal for the example to be valid.
    assert config["workspace_width_m"] < 8.0
    plan = preview.build_preview(**config)  # keys are build_preview kwargs
    assert plan["ready_for_full_path_following"] is False
    assert plan["path_shape"] == "diagonal_rectangle_serpentine"
    assert plan["lane_count"] >= 1
    assert float(plan["diagonal_length_m"]) > float(config["workspace_width_m"])
