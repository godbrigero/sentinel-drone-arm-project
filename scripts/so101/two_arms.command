#!/bin/zsh
set -e
PROJECT_ROOT="$(cd -- "$(dirname -- "$0")/../.." && pwd)"
cd -- "$PROJECT_ROOT"
exec uv run --locked mjpython -m pytest --sim-gui -s -q airtruck/testing_util/tests/test_two_arms_push.py -k two_arms_collision_push "$@"
