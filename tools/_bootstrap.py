"""sys.path 부트스트랩 헬퍼 — 독립 실행 도구가 repo 루트를 import 경로에 넣도록 한다.
sys.path bootstrap helper so standalone tools can import the repo packages.

목적/역할:
    ``tools/`` 안의 스크립트를 ``python tools/xxx.py`` 처럼 직접 실행하면, sys.path[0]는
    ``tools/`` 디렉터리이지 repo 루트가 아니다. 그러면 ``from tools import ...`` 나
    ``import gps_coverage_core`` 가 실패한다. 이 모듈은 import 되는 부수효과만으로
    repo 루트를 ``sys.path`` 맨 앞에 삽입해 그 문제를 없앤다.
    Running a script directly puts ``tools/`` (not the repo root) on sys.path, so
    top-level package imports fail. Importing this module (for its side effect)
    inserts the repo root at the front of ``sys.path`` to fix that.

시스템 내 위치:
    ``tools/`` 하위의 거의 모든 독립 도구가 맨 위에서 이 모듈을 import 한다. 관용구는
    ``try: from tools import _bootstrap / except ImportError: import _bootstrap`` 로,
    루트가 이미 경로에 있으면 전자가, 아직 ``tools/`` 에서 실행 중이면 후자가 성사된다.
    Nearly every standalone tool imports this first, via the try/except idiom
    (``from tools import _bootstrap`` else bare ``import _bootstrap``).

핵심 개념·불변식:
    - ``ROOT_DIR`` 은 이 파일 기준 한 단계 위(= repo 루트)로 계산한다. 이 파일을 다른
      깊이로 옮기면 ``parents[1]`` 인덱스도 반드시 함께 고쳐야 한다 (함정).
    - ``sys.path`` 삽입은 멱등(idempotent)하다: 이미 있으면 다시 넣지 않는다.
    - ``MPLCONFIGDIR`` 은 ``setdefault`` 라 이미 설정돼 있으면 존중한다. 헤드리스/CI에서
      matplotlib 이 홈 캐시를 못 써서 경고/실패하는 것을 막는 안전장치다.
    - Invariant: ``parents[1]`` assumes this file lives one level under the repo
      root; keep it in sync if the file moves. Path insert is idempotent, and
      ``MPLCONFIGDIR`` uses setdefault so an existing value wins.

리팩토링 노트:
    부수효과 전용 모듈이라 공개 API가 없다. import 순서가 곧 계약이다 — 다른 무엇을
    import 하기 전에 먼저 import 되어야 경로 보정이 유효하다.
    Side-effect-only module with no public API; its contract is import ordering
    (import it before anything that needs the repo root on the path).
"""
from __future__ import annotations

import os
import sys
from pathlib import Path

ROOT_DIR = Path(__file__).resolve().parents[1]
if str(ROOT_DIR) not in sys.path:
    sys.path.insert(0, str(ROOT_DIR))

os.environ.setdefault("MPLCONFIGDIR", "/tmp/matplotlib")
