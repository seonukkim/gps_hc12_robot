from __future__ import annotations

from tools import check_imu_readiness_watch
from tools import imu_readiness_watch


def test_imu_readiness_passes_bmi160_relative_yaw_diag() -> None:
    summary = imu_readiness_watch.summarize_rows(
        imu_readiness_watch.parse_rows(
            "STAGE20 imu_type=BMI160 imu_present=true imu_pmu_normal=true "
            "imu_data_plausible=true imu_calibrated=true imu_heading_ready=false "
            "imu_heading_block_reason=RELATIVE_YAW_DIAG_ONLY imu_relative_yaw_deg=1.25\n"
            "STAGE20 imu_type=BMI160 imu_present=true imu_pmu_normal=true "
            "imu_data_plausible=true imu_calibrated=true imu_heading_ready=true "
            "imu_heading_block_reason=RELATIVE_YAW_DIAG_ONLY imu_relative_yaw_deg=2.00\n"
        )
    )
    assert summary["imu_status"] == "IMU_READY"
    assert summary["imu_type"] == "BMI160"
    assert summary["imu_relative_yaw_present"] is True
    result = check_imu_readiness_watch.evaluate_summary(summary)
    assert result["verdict"] == "PASS"
    assert result["ready_for_full_path_following"] is False


def test_imu_readiness_blocks_calibrating_bmi160() -> None:
    summary = imu_readiness_watch.summarize_rows(
        imu_readiness_watch.parse_rows(
            "STAGE20 imu_type=BMI160 imu_present=true imu_pmu_normal=true "
            "imu_data_plausible=true imu_calibrated=false imu_heading_block_reason=CALIBRATING "
            "imu_relative_yaw_deg=0.25\n"
        )
    )
    assert summary["imu_status"] == "IMU_CALIBRATING"
    result = check_imu_readiness_watch.evaluate_summary(summary)
    assert result["verdict"] == "FAIL"
    assert result["ready_for_full_path_following"] is False
