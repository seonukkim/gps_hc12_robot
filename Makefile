PYTHON ?= uv run python

.PHONY: sync test verify export path-preview

sync:
	uv sync --extra dev --extra web

test:
	uv run pytest -q

verify:
	uv run python tools/verify_env.py

export:
	./scripts/export_requirements.sh

path-preview:
	$(PYTHON) tools/path_preview.py --lat-a 35.123456 --lon-a 129.123456 --lat-b 35.124000 --lon-b 129.124000 --spacing 5.0
