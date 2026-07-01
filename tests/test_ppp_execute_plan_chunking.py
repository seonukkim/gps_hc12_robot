"""``execute-plan`` CLI 의 청킹/제어 기본값이 "안전" 프로파일임을 고정하는 테스트.

목적/역할 (KO):
    ``cli.build_parser`` 로 만든 ``execute-plan`` 서브커맨드가 *인자 없이* 파싱될
    때 채택하는 기본값을 못 박는다. 이 기본값들은 자율 실행의 안전 프로파일 --
    폐루프 GPS/IMU 제어, 짧은 라이브 청크(700ms), 세그먼트당 청크 상한, 보정
    게인의 보수적 상한 등 -- 을 정의하므로, 누군가 기본값을 바꾸면 이 테스트가
    먼저 잡는다.

핵심 계약 (KO):
    ``path_control_mode=gps_imu_closed_loop``, ``live_chunk_ms=700``,
    ``max_segment_chunks=20``, ``max_ms=1000``, ``gps_degradation_policy=continue``,
    헤딩 홀드/크로스트랙 보정/GPS 재앵커는 모두 "true", 게인은
    ``k_heading=0.006``, ``k_cross_track=0.20``, ``max_correction_b=0.08``.

Purpose (EN):
    Pins the argparse defaults of the ``execute-plan`` subcommand parsed with no
    arguments -- the safe autonomy profile (closed-loop GPS/IMU control, short
    700ms live chunks, per-segment chunk cap, conservative correction-gain caps).
    A regression guard so nobody silently changes a safety-relevant default.
"""
from tools.physical_path_planning import cli


def test_execute_plan_chunking_defaults_are_safe() -> None:
    """인자 없이 파싱한 execute-plan 의 모든 안전 관련 기본값을 못 박는다 / pins every
    safety-relevant default of ``execute-plan`` parsed with no arguments."""
    parser = cli.build_parser()
    args = parser.parse_args(["execute-plan"])
    assert args.path_control_mode == "gps_imu_closed_loop"
    assert args.live_chunk_ms == 700
    assert args.max_segment_chunks == 20
    assert args.max_ms == 1000
    assert args.gps_degradation_policy == "continue"
    assert args.imu_heading_hold == "true"
    assert args.cross_track_correction == "true"
    assert args.gps_reanchor == "true"
    assert args.k_heading == 0.006
    assert args.k_cross_track == 0.20
    assert args.max_correction_b == 0.08
