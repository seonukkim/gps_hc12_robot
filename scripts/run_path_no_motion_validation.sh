#!/usr/bin/env bash
set -euo pipefail

PORT="/dev/ttyACM0"
PATH_PACKAGE=""
SAMPLE_LOG=""
OUT_DIR="outputs/path_no_motion_validation/latest"

while [[ $# -gt 0 ]]; do
  case "$1" in
    --port)
      PORT="$2"
      shift 2
      ;;
    --path-package)
      PATH_PACKAGE="$2"
      shift 2
      ;;
    --sample-log)
      SAMPLE_LOG="$2"
      shift 2
      ;;
    --out-dir)
      OUT_DIR="$2"
      shift 2
      ;;
    -h|--help)
      cat <<'USAGE'
Usage:
  scripts/run_path_no_motion_validation.sh \
    --port /dev/ttyACM0 \
    --path-package outputs/field_ab_serpentine/latest/path_package.json \
    [--sample-log outputs/logs/no_motion_status_sample.log] \
    [--out-dir outputs/path_no_motion_validation/latest]

No-motion only: this wrapper does not upload firmware, does not send HC-12
frames, and does not generate rover motor commands. The Python validator reports
the port for traceability but does not open serial by default.
USAGE
      exit 0
      ;;
    *)
      echo "Unknown argument: $1" >&2
      exit 2
      ;;
  esac
done

if [[ -z "${PATH_PACKAGE}" ]]; then
  echo "ERROR: --path-package is required." >&2
  exit 2
fi

CMD=(uv run python tools/path_no_motion_validation.py --port "${PORT}" --path-package "${PATH_PACKAGE}" --out-dir "${OUT_DIR}")
if [[ -n "${SAMPLE_LOG}" ]]; then
  CMD+=(--sample-log "${SAMPLE_LOG}")
fi

echo "NO-MOTION VALIDATION ONLY"
echo "physical_output_active=false"
echo "motor_command_generated=false"
"${CMD[@]}"
