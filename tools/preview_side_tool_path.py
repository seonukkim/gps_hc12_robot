from __future__ import annotations

try:
    from tools.side_tool_path_preview import main as _side_tool_main
except ImportError:
    from side_tool_path_preview import main as _side_tool_main  # type: ignore


def main(argv=None) -> int:
    args = [] if argv is None else list(argv)
    if "--advanced" not in args and "--debug-planner-options" not in args:
        args.insert(0, "--advanced")
    return _side_tool_main(args)


if __name__ == "__main__":
    raise SystemExit(main())
