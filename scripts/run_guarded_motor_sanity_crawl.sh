#!/usr/bin/env bash
set -euo pipefail

PORT="/dev/ttyACM0"
MAX_CMD="0.06"
PULSE_MS="200"
INTER_PULSE_MS="2000"
REQUIRE_ENTER="true"
OUT_DIR="outputs/stage15_guarded_crawl/latest"
DANGEROUSLY_ALLOW_LONGER_PULSE="false"
DANGEROUSLY_ALLOW_HIGHER_CMD="false"

while [[ $# -gt 0 ]]; do
  case "$1" in
    --port)
      PORT="$2"
      shift 2
      ;;
    --max-cmd)
      MAX_CMD="$2"
      shift 2
      ;;
    --pulse-ms)
      PULSE_MS="$2"
      shift 2
      ;;
    --inter-pulse-ms)
      INTER_PULSE_MS="$2"
      shift 2
      ;;
    --require-enter)
      REQUIRE_ENTER="$2"
      shift 2
      ;;
    --out-dir)
      OUT_DIR="$2"
      shift 2
      ;;
    --dangerously-allow-longer-pulse)
      DANGEROUSLY_ALLOW_LONGER_PULSE="true"
      shift
      ;;
    --dangerously-allow-higher-cmd)
      DANGEROUSLY_ALLOW_HIGHER_CMD="true"
      shift
      ;;
    -h|--help)
      cat <<'USAGE'
Usage:
  scripts/run_guarded_motor_sanity_crawl.sh \
    --port /dev/ttyACM0 \
    [--max-cmd 0.06] \
    [--pulse-ms 200] \
    [--inter-pulse-ms 2000] \
    [--require-enter true] \
    [--out-dir outputs/stage15_guarded_crawl/latest]

Stage 15 guarded motor sanity crawl only. This is not path following, does not
read a path package, does not use HC-12 target commands, and does not connect
station virtual controller values to firmware motor commands.

Refuses --pulse-ms > 300 unless --dangerously-allow-longer-pulse is present.
Refuses --max-cmd > 0.10 unless --dangerously-allow-higher-cmd is present.
USAGE
      exit 0
      ;;
    *)
      echo "Unknown argument: $1" >&2
      exit 2
      ;;
  esac
done

python3 - "$PULSE_MS" "$DANGEROUSLY_ALLOW_LONGER_PULSE" <<'PY'
import sys
pulse_ms = int(float(sys.argv[1]))
allow = sys.argv[2].lower() == "true"
if pulse_ms > 300 and not allow:
    print("ABORT: --pulse-ms > 300 requires --dangerously-allow-longer-pulse.", file=sys.stderr)
    raise SystemExit(2)
PY

python3 - "$MAX_CMD" "$DANGEROUSLY_ALLOW_HIGHER_CMD" <<'PY'
import sys
max_cmd = float(sys.argv[1])
allow = sys.argv[2].lower() == "true"
if max_cmd > 0.10 and not allow:
    print("ABORT: --max-cmd > 0.10 requires --dangerously-allow-higher-cmd.", file=sys.stderr)
    raise SystemExit(2)
PY

REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$REPO_DIR"

mkdir -p "$OUT_DIR"

FQBN="OpenRB-150:samd:OpenRB-150"
BUILD_PATH="/private/tmp/openrb-stage15-guarded-crawl"
SKETCH="firmware/openrb_robot_controller"
FLAGS="-DSTAGE15_GUARDED_CRAWL_TEST=1 \
-DPHYSICAL_PATH_FOLLOWING_ENABLE=0 \
-DPATH_FOLLOWING_ALLOW_MOTOR_OUTPUT=0 \
-DPATH_FOLLOWING_DRYRUN=0 \
-DGROUND_CRAWL_TEST_MODE=0 \
-DAUTO_MOTION_ARMED=0 \
-DMOTOR_PULSE_TEST_MODE=0 \
-DSTAGE15_MAX_CMD=${MAX_CMD} \
-DSTAGE15_PULSE_MS=${PULSE_MS} \
-DSTAGE15_INTER_PULSE_MS=${INTER_PULSE_MS}"

if ! printf '%s' "$FLAGS" | grep -q 'STAGE15_GUARDED_CRAWL_TEST=1'; then
  echo "ABORT: Stage 15 compile flag missing." >&2
  exit 2
fi
if printf '%s' "$FLAGS" | grep -Eq 'PHYSICAL_PATH_FOLLOWING_ENABLE=1|PATH_FOLLOWING_ALLOW_MOTOR_OUTPUT=1|PATH_FOLLOWING_DRYRUN=1|GROUND_CRAWL_TEST_MODE=1|AUTO_MOTION_ARMED=1'; then
  echo "ABORT: path-following or autonomous motion flag present." >&2
  exit 2
fi

echo "STAGE 15 GUARDED MOTOR SANITY CRAWL"
echo "This is not path following. Wheels off ground or physically restrained."
echo "port=${PORT}"
echo "out_dir=${OUT_DIR}"
echo "flags=${FLAGS}"
echo "path_following_enable=false"
echo "path_package_used=false"
echo "virtual_controller_used=false"

arduino-cli compile --fqbn "$FQBN" --build-path "$BUILD_PATH" \
  --build-property "compiler.cpp.extra_flags=${FLAGS}" "$SKETCH"
arduino-cli upload -p "$PORT" --fqbn "$FQBN" --build-path "$BUILD_PATH" "$SKETCH"

uv run python tools/stage15_guarded_crawl_runner.py \
  --port "$PORT" \
  --max-cmd "$MAX_CMD" \
  --pulse-ms "$PULSE_MS" \
  --inter-pulse-ms "$INTER_PULSE_MS" \
  --require-enter "$REQUIRE_ENTER" \
  --out-dir "$OUT_DIR"

uv run python tools/check_guarded_crawl_log.py \
  "$OUT_DIR/guarded_crawl.log" \
  --max-cmd "$MAX_CMD" \
  --max-duration-ms "$PULSE_MS"
