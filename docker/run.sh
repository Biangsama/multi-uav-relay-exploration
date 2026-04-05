#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)

export DISPLAY="${DISPLAY:-:0}"

if command -v xhost >/dev/null 2>&1; then
  xhost +local:root >/dev/null 2>&1 || true
  trap 'xhost -local:root >/dev/null 2>&1 || true' EXIT
fi

cd "${ROOT_DIR}"

if [ "$#" -eq 0 ]; then
  docker compose run --rm relay-explore bash
else
  docker compose run --rm relay-explore "$@"
fi
