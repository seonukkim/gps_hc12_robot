#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT_DIR"

export UV_CACHE_DIR="${UV_CACHE_DIR:-/tmp/uv-cache}"
mkdir -p "$UV_CACHE_DIR"

if ! uv pip compile pyproject.toml -o requirements.txt; then
  uv export --offline --format requirements.txt --no-hashes --no-emit-project --output-file requirements.txt
fi

if ! uv pip compile pyproject.toml --extra dev --extra web -o requirements-dev.txt; then
  uv export --offline --format requirements.txt --no-hashes --no-emit-project --extra dev --extra web --output-file requirements-dev.txt
fi
