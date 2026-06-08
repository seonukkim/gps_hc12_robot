from tools.physical_path_planning import cli


def test_execute_plan_chunking_defaults_are_safe() -> None:
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
