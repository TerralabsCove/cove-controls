#!/usr/bin/env bash

export QT_ACCESSIBILITY="${QT_ACCESSIBILITY:-1}"

warehouse_pid=""
MOVEIT_WAREHOUSE_DATABASE_PATH_EFFECTIVE=""

default_moveit_warehouse_path() {
  local moveit_config_package="$1"

  case "$moveit_config_package" in
    simple_assembly_tracking_moveit_config)
      printf '%s/.ros/simple_assembly_tracking_warehouse.sqlite\n' "$HOME"
      ;;
    *)
      printf '%s/.ros/simple_assembly_warehouse.sqlite\n' "$HOME"
      ;;
  esac
}

ensure_moveit_warehouse_sqlite_schema() {
  local db_path="$1"

  DB_PATH="$db_path" python3 - <<'PY'
import os
import sqlite3

db_path = os.environ["DB_PATH"]
if not os.path.exists(db_path):
    raise SystemExit(0)

table_name = "T_moveit_robot_states@robot_states"
con = sqlite3.connect(db_path)
try:
    tables = {
        row[0]
        for row in con.execute("select name from sqlite_master where type='table'")
    }
    if table_name not in tables:
        raise SystemExit(0)

    columns = {
        row[1]
        for row in con.execute(f'pragma table_info("{table_name}")')
    }
    for column in ("M_state_id", "M_robot_id"):
        if column not in columns:
            con.execute(f'alter table "{table_name}" add column {column} TEXT')
    con.commit()
finally:
    con.close()
PY
}

start_moveit_warehouse() {
  local moveit_config_package="$1"
  local warehouse_args=()
  local db_path

  mkdir -p "$HOME/.ros"
  db_path="${MOVEIT_WAREHOUSE_DATABASE_PATH:-$(default_moveit_warehouse_path "$moveit_config_package")}"
  MOVEIT_WAREHOUSE_DATABASE_PATH_EFFECTIVE="$db_path"

  warehouse_args+=("moveit_warehouse_database_path:=$db_path")

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

  ensure_moveit_warehouse_sqlite_schema "$db_path"
}

stop_moveit_warehouse() {
  kill "${warehouse_pid:-}" 2>/dev/null || true
  wait "${warehouse_pid:-}" 2>/dev/null || true
}
