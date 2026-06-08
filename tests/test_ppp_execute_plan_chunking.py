from tools.physical_path_planning import cli


def test_execute_plan_chunking_defaults_are_safe() -> None:
    parser = cli.build_parser()
    args = parser.parse_args(["execute-plan"])
    assert args.live_chunk_ms == 700
    assert args.max_segment_chunks == 20
    assert args.max_ms == 1000
    assert args.gps_degradation_policy == "continue"
    assert args.imu_heading_hold == "true"
    assert args.cross_track_correction == "true"
