#!/usr/bin/env bash
set -euo pipefail

PORT="/dev/ttyACM0"
DURATION_S="60"
OUT_DIR="outputs/imu_readiness_watch/latest"
LOG=""

while [[ $# -gt 0 ]]; do
  case "$1" in
    --port) PORT="$2"; shift 2 ;;
    --duration-s) DURATION_S="$2"; shift 2 ;;
    --out-dir) OUT_DIR="$2"; shift 2 ;;
    --log) LOG="$2"; shift 2 ;;
    -h|--help)
      cat <<'USAGE'
Usage:
  scripts/run_imu_readiness_watch.sh \
    --port "$PORT" \
    --duration-s 60 \
    --out-dir outputs/imu_readiness_watch/latest

BMI160 readiness watch. No motor commands and no upload.
USAGE
      exit 0
      ;;
    *) echo "Unknown argument: $1" >&2; exit 2 ;;
  esac
done

REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$REPO_DIR"
mkdir -p "$OUT_DIR"

ARGS=(legacy/stage_tools/imu_readiness_watch.py --port "$PORT" --duration-s "$DURATION_S" --out-dir "$OUT_DIR")
if [[ -n "$LOG" ]]; then
  ARGS+=(--log "$LOG")
fi
UV_NO_SYNC=1 uv run python "${ARGS[@]}"
UV_NO_SYNC=1 uv run python legacy/stage_tools/check_imu_readiness_watch.py "$OUT_DIR" || true
