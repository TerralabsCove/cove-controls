#!/usr/bin/env bash
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

source /opt/ros/jazzy/setup.bash
rosdep install --from-paths "$REPO_ROOT/src" --ignore-src -r -y "$@"
