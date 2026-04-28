#!/usr/bin/env bash

warehouse_pid=""

start_moveit_warehouse() {
  local moveit_config_package="$1"
  local warehouse_args=()

  mkdir -p "$HOME/.ros"

  if [[ -n "${MOVEIT_WAREHOUSE_DATABASE_PATH:-}" ]]; then
    warehouse_args+=("moveit_warehouse_database_path:=$MOVEIT_WAREHOUSE_DATABASE_PATH")
  fi

  if ! ros2 pkg prefix warehouse_ros_sqlite >/dev/null 2>&1; then
    echo "Stored States database is disabled: sudo apt install ros-jazzy-warehouse-ros-sqlite" >&2
    return 0
  fi

  ros2 launch "$moveit_config_package" warehouse_db.launch.py "${warehouse_args[@]}" &
  warehouse_pid=$!

  sleep 1
  if ! kill -0 "$warehouse_pid" 2>/dev/null; then
    wait "$warehouse_pid"
  fi
}

stop_moveit_warehouse() {
  kill "${warehouse_pid:-}" 2>/dev/null || true
  wait "${warehouse_pid:-}" 2>/dev/null || true
}
