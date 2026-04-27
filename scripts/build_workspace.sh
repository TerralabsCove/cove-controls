#!/usr/bin/env bash
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

set +u
source /opt/ros/jazzy/setup.bash
set -u
cd "$REPO_ROOT"

# Wipe stale build/install for the packages most prone to picking up new launch
# files / installed scripts so a plain `colcon build` always sees them.
CLEAN_PKGS=(full_assembly simple_assembly_tracking simple_assembly_tracking_moveit_config)
for pkg in "${CLEAN_PKGS[@]}"; do
  rm -rf "build/$pkg" "install/$pkg"
done

colcon build --symlink-install "$@"
