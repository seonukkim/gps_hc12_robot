"""side_tool_path_preview 실행을 감싸는 얇은 래퍼 / Thin wrapper around side_tool_path_preview.

목적/역할:
    사이드툴 경로 미리보기를 항상 '고급(advanced) 모드'로 실행하도록 보장하는 편의 진입점이다.
    사용자가 --advanced/--debug-planner-options를 주지 않았으면 자동으로 --advanced를 앞에 끼워
    넣고, 그대로 `tools.side_tool_path_preview.main`에 위임한다. 자체 로직은 없다.

    Convenience entry point that guarantees the side-tool path preview runs in "advanced"
    mode. If the user did not pass --advanced/--debug-planner-options, it prepends --advanced
    and delegates to `tools.side_tool_path_preview.main`. It contains no logic of its own.

시스템 내 위치:
    `tools/` CLI 진입점. 실제 미리보기·플래닝 로직은 모두 `tools/side_tool_path_preview.py`에
    있고, 이 파일은 인자만 손봐 그 main()을 호출한다. import는 패키지/스크립트 두 경로 모두를
    지원하기 위해 try/except로 감싼다.

    Entry-point script in `tools/`. All preview/planning logic lives in
    `tools/side_tool_path_preview.py`; this file only tweaks argv and calls its main(). The
    import is guarded with try/except to support both package and script execution.

핵심 개념·불변식:
    - 이미 --advanced나 --debug-planner-options가 있으면 중복 삽입하지 않는다.
    - 위임 대상 main()의 반환값(종료 코드)을 그대로 돌려준다.

    - Does not double-insert --advanced when either advanced flag is already present.
    - Returns the delegate main()'s exit code unchanged.

리팩토링 노트:
    실제 인자·동작은 side_tool_path_preview 쪽에서 정의된다. 기본 모드를 바꾸려면 여기서
    삽입하는 플래그만 고치고, 옵션 자체는 위임 대상에서 관리할 것.

    The real arguments/behavior are defined in side_tool_path_preview. To change the default
    mode, edit only the flag inserted here; manage the options in the delegate.
"""

from __future__ import annotations

try:
    from tools.side_tool_path_preview import main as _side_tool_main
except ImportError:
    from side_tool_path_preview import main as _side_tool_main  # type: ignore


def main(argv=None) -> int:
    """argv에 --advanced를 보장한 뒤 side_tool_path_preview.main에 위임한다.

    Ensure --advanced is present in argv, then delegate to side_tool_path_preview.main.
    반환값은 위임 대상의 종료 코드 / returns the delegate's exit code.
    """
    args = [] if argv is None else list(argv)
    # 고급 옵션이 하나도 없을 때만 기본으로 --advanced를 강제 / force advanced only if absent
    if "--advanced" not in args and "--debug-planner-options" not in args:
        args.insert(0, "--advanced")
    return _side_tool_main(args)


if __name__ == "__main__":
    raise SystemExit(main())
