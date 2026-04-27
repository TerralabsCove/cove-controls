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

colcon_args=("$@")

# If callers use --packages-select for a leaf package, colcon will not build
# dependencies. Since this script intentionally wipes these generated local
# packages first, make the common targeted rebuild commands include the wiped
# prerequisites too.
select_idx=-1
for i in "${!colcon_args[@]}"; do
  if [[ "${colcon_args[$i]}" == "--packages-select" ]]; then
    select_idx="$i"
    break
  fi
done

if (( select_idx >= 0 )); then
  select_end="${#colcon_args[@]}"
  for ((i = select_idx + 1; i < ${#colcon_args[@]}; i++)); do
    if [[ "${colcon_args[$i]}" == --* ]]; then
      select_end="$i"
      break
    fi
  done

  select_start=$((select_idx + 1))
  select_count=$((select_end - select_idx - 1))
  selected=("${colcon_args[@]:$select_start:$select_count}")

  has_selected_pkg() {
    local needle="$1"
    local pkg
    for pkg in "${selected[@]}"; do
      [[ "$pkg" == "$needle" ]] && return 0
    done
    return 1
  }

  add_selected_pkg() {
    local pkg="$1"
    has_selected_pkg "$pkg" || selected+=("$pkg")
  }

  if has_selected_pkg simple_assembly_tracking; then
    add_selected_pkg full_assembly
  fi

  if has_selected_pkg simple_assembly_tracking_moveit_config; then
    add_selected_pkg full_assembly
    add_selected_pkg simple_assembly_tracking
  fi

  colcon_args=(
    "${colcon_args[@]:0:$select_start}"
    "${selected[@]}"
    "${colcon_args[@]:$select_end}"
  )
fi

colcon build --symlink-install "${colcon_args[@]}"
