#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="${SCRIPT_DIR}/.."
BUILD_DIR="${PROJECT_DIR}/build"

DEFAULT_DATA_PREFIX="200"
PROBLEM_FILE="${1:-${PROJECT_DIR}/../../data/${DEFAULT_DATA_PREFIX}.10.2.txt}"

OUTPUTS_DIR="${OUTPUTS_DIR:-${PROJECT_DIR}/outputs}"
COMPACT_OUTPUT="${COMPACT_OUTPUT:-1}"
RUN_ID="${RUN_ID:-}"
SEED="${SEED:-}"
TIME_LIMIT="${TIME_LIMIT:-}"
VERBOSE="${VERBOSE:-0}"

case "${PROBLEM_FILE}" in
  /*) ;;
  *) PROBLEM_FILE="$(pwd)/${PROBLEM_FILE}" ;;
esac
case "${OUTPUTS_DIR}" in
  /*) ;;
  *) OUTPUTS_DIR="$(pwd)/${OUTPUTS_DIR}" ;;
esac

CMD=(
  "${BUILD_DIR}/ga-single-island" run
  "${PROBLEM_FILE}"
  --outputs "${OUTPUTS_DIR}"
)

if [ "${COMPACT_OUTPUT}" = "1" ]; then
  CMD+=(--compact-output)
fi

if [ -n "${RUN_ID}" ]; then
  CMD+=(--run-id "${RUN_ID}")
fi

if [ -n "${SEED}" ]; then
  CMD+=(--seed "${SEED}")
fi

if [ -n "${TIME_LIMIT}" ]; then
  CMD+=(--time-limit "${TIME_LIMIT}")
fi

if [ "${VERBOSE}" = "1" ]; then
  CMD+=(--verbose)
fi

# Extra arguments after the problem file are passed through, e.g.
#   ./script.sh ../../data/50.10.1.txt --initial-pop 100 --ls-neighbors 0
CMD+=("${@:2}")

cd "${PROJECT_DIR}"
"${CMD[@]}"
