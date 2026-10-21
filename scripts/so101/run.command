#!/bin/zsh
set -e
PROJECT_ROOT="$(cd -- "$(dirname -- "$0")/../.." && pwd)"
cd -- "$PROJECT_ROOT"
exec uv run --locked mjpython "$PROJECT_ROOT/scripts/so101/run.py" "$@"
