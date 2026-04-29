#!/usr/bin/env python3
"""Replay MoveIt warehouse Stored States through the joint trajectory controller."""

from __future__ import annotations

import argparse
import sqlite3
import subprocess
import sys
import threading
import time
from pathlib import Path
from typing import Any

import rclpy
from builtin_interfaces.msg import Duration
from moveit_msgs.msg import RobotState
from rclpy.node import Node
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


JOINT_NAMES = [
    "revolute_1_0",
    "revolute_2_0",
    "revolute_3_0",
    "revolute_4_0",
    "revolute_5_0",
    "revolute_6_0",
    "revolute_7_0",
]

ROBOT_STATES_TABLE = "T_moveit_robot_states@robot_states"


class StoredState:
    def __init__(
        self,
        row_id: int,
        name: str,
        robot_id: str,
        creation_time: float,
        positions: list[float],
    ) -> None:
        self.row_id = row_id
        self.name = name
        self.robot_id = robot_id
        self.creation_time = creation_time
        self.positions = positions


class SequenceStep:
    def __init__(
        self,
        token: str,
        kind: str,
        state: StoredState | None = None,
        magnet_action: str | None = None,
        manual: bool = False,
    ) -> None:
        self.token = token
        self.kind = kind
        self.state = state
        self.magnet_action = magnet_action
        self.manual = manual


class MagnetController:
    def __init__(self, gpio_chip: str, gpio_line: int, dry_run: bool) -> None:
        self.gpio_chip = gpio_chip
        self.gpio_line = gpio_line
        self.dry_run = dry_run
        self.proc: subprocess.Popen | None = None

    def on(self) -> None:
        if self.dry_run:
            print("dry-run: magnet ON")
            return
        if self.proc is not None:
            print("magnet already ON")
            return
        self._drive_low()
        self.proc = subprocess.Popen(
            ["gpioset", "--mode=signal", self.gpio_chip, f"{self.gpio_line}=1"]
        )
        print("magnet ON")

    def off(self) -> None:
        if self.dry_run:
            print("dry-run: magnet OFF")
            return
        if self.proc is not None:
            self.proc.terminate()
            try:
                self.proc.wait(timeout=2.0)
            except subprocess.TimeoutExpired:
                self.proc.kill()
                self.proc.wait(timeout=1.0)
            self.proc = None
        self._drive_low()
        print("magnet OFF")

    def _drive_low(self) -> None:
        subprocess.run(
            ["gpioset", self.gpio_chip, f"{self.gpio_line}=0"],
            check=False,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
        )


class StoredStateRunner(Node):
    def __init__(self, trajectory_topic: str, joint_topic: str) -> None:
        super().__init__("canhat_stored_state_runner")
        self.traj_pub = self.create_publisher(JointTrajectory, trajectory_topic, 10)
        self._lock = threading.Lock()
        self._latest_joint_state: JointState | None = None
        self.create_subscription(JointState, joint_topic, self._on_joint_state, 10)

    def _on_joint_state(self, msg: JointState) -> None:
        with self._lock:
            self._latest_joint_state = msg

    def latest_positions(self) -> dict[str, float]:
        with self._lock:
            msg = self._latest_joint_state
        if msg is None:
            return {}
        return {
            name: float(position)
            for name, position in zip(msg.name, msg.position)
            if name in JOINT_NAMES
        }

    def wait_for_joint_states(self, timeout_s: float) -> bool:
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline and rclpy.ok():
            if self.latest_positions():
                return True
            time.sleep(0.05)
        return False

    def wait_for_controller_subscriber(self, timeout_s: float) -> bool:
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline and rclpy.ok():
            if self.traj_pub.get_subscription_count() > 0:
                return True
            time.sleep(0.05)
        return self.traj_pub.get_subscription_count() > 0

    def publish_state(self, positions: list[float], duration_s: float) -> None:
        traj = JointTrajectory()
        traj.header.stamp = self.get_clock().now().to_msg()
        traj.joint_names = list(JOINT_NAMES)

        point = JointTrajectoryPoint()
        point.positions = [float(value) for value in positions]
        point.velocities = [0.0] * len(JOINT_NAMES)
        point.time_from_start = seconds_to_duration(duration_s)
        traj.points = [point]
        self.traj_pub.publish(traj)


def seconds_to_duration(seconds: float) -> Duration:
    whole = int(seconds)
    nanosec = int(round((seconds - whole) * 1_000_000_000))
    if nanosec >= 1_000_000_000:
        whole += 1
        nanosec -= 1_000_000_000
    return Duration(sec=whole, nanosec=nanosec)


def decode_text(value: Any) -> str:
    if value is None:
        return ""
    if isinstance(value, bytes):
        return value.decode("utf-8", errors="replace")
    return str(value)


def joint_positions_from_robot_state(message: RobotState, label: str) -> list[float]:
    positions_by_name = {
        name: float(message.joint_state.position[index])
        for index, name in enumerate(message.joint_state.name)
        if index < len(message.joint_state.position)
    }
    missing = [name for name in JOINT_NAMES if name not in positions_by_name]
    if missing:
        raise ValueError(
            f"stored state '{label}' is missing joint positions for: "
            + ", ".join(missing)
        )
    return [positions_by_name[name] for name in JOINT_NAMES]


def load_stored_states(database_path: Path, robot_id_filter: str) -> list[StoredState]:
    con = sqlite3.connect(database_path)
    try:
        tables = {
            row[0]
            for row in con.execute("select name from sqlite_master where type='table'")
        }
        if ROBOT_STATES_TABLE not in tables:
            raise ValueError(f"table not found: {ROBOT_STATES_TABLE}")

        query = (
            f'select M_id, M_state_id, M_robot_id, M_creation_time, Data '
            f'from "{ROBOT_STATES_TABLE}" order by M_id'
        )
        states: list[StoredState] = []
        for row_id, state_id, robot_id, creation_time, data in con.execute(query):
            name = decode_text(state_id) or str(row_id)
            robot_id_text = decode_text(robot_id)
            if robot_id_filter and robot_id_text != robot_id_filter:
                continue
            message = deserialize_message(data, RobotState)
            states.append(
                StoredState(
                    row_id=int(row_id),
                    name=name,
                    robot_id=robot_id_text,
                    creation_time=float(creation_time or 0.0),
                    positions=joint_positions_from_robot_state(message, name),
                )
            )
        return states
    finally:
        con.close()


def parse_sequence(value: str) -> list[str]:
    return [item.strip() for item in value.split(",") if item.strip()]


def parse_magnet_action(token: str) -> str | None:
    text = token.strip().lower()
    if text.startswith("!"):
        text = text[1:].strip()
    compact = text.replace(" ", "").replace("-", "_").replace(":", "_")

    if compact in {"magnet_on", "magneton", "magnet(on)", "magnet=on"}:
        return "on"
    if compact in {"magnet_off", "magnetoff", "magnet(off)", "magnet=off"}:
        return "off"
    return None


def parse_state_token(token: str) -> tuple[str, bool]:
    text = token.strip()
    manual = False
    if text.endswith(")") and "(" in text:
        name, suffix = text.rsplit("(", 1)
        mode = suffix[:-1].strip().lower()
        if mode != "manual":
            raise ValueError(f"unsupported mode in sequence token '{token}': {mode}")
        text = name.strip()
        manual = True
    if not text:
        raise ValueError(f"empty stored state name in sequence token '{token}'")
    return text, manual


def build_state_indexes(
    states: list[StoredState],
) -> tuple[dict[str, list[StoredState]], dict[str, StoredState]]:
    by_name: dict[str, list[StoredState]] = {}
    by_id: dict[str, StoredState] = {}
    for state in states:
        by_name.setdefault(state.name, []).append(state)
        by_id[str(state.row_id)] = state
    return by_name, by_id


def resolve_state(
    token: str,
    by_name: dict[str, list[StoredState]],
    by_id: dict[str, StoredState],
) -> StoredState:
    if token in by_id:
        return by_id[token]
    matches = by_name.get(token, [])
    if not matches:
        raise ValueError(f"stored state not found: {token}")
    if len(matches) > 1:
        ids = ", ".join(str(match.row_id) for match in matches)
        raise ValueError(
            f"stored state name '{token}' is ambiguous; use one of row IDs: {ids}"
        )
    return matches[0]


def select_sequence(states: list[StoredState], sequence: list[str]) -> list[SequenceStep]:
    by_name, by_id = build_state_indexes(states)
    selected: list[SequenceStep] = []
    for token in sequence:
        magnet_action = parse_magnet_action(token)
        if magnet_action is not None:
            selected.append(
                SequenceStep(token=token, kind="magnet", magnet_action=magnet_action)
            )
            continue
        state_token, manual = parse_state_token(token)
        selected.append(
            SequenceStep(
                token=token,
                kind="state",
                state=resolve_state(state_token, by_name, by_id),
                manual=manual,
            )
        )
    return selected


def state_summary(state: StoredState) -> str:
    joint_text = " ".join(
        f"{name.replace('revolute_', 'j')}={position:+.3f}"
        for name, position in zip(JOINT_NAMES, state.positions)
    )
    return (
        f"stored state '{state.name}' row_id={state.row_id} robot='{state.robot_id}'\n"
        f"  {joint_text}"
    )


def sequence_step_label(step: SequenceStep) -> str:
    if step.kind == "magnet":
        return f"magnet_{step.magnet_action}"
    if step.state is None:
        return step.token
    suffix = "(manual)" if step.manual else ""
    return f"{step.state.name}{suffix}"


def print_available_states(states: list[StoredState]) -> None:
    if not states:
        print("No stored states found.")
        return
    print("Available stored states:")
    for state in states:
        print(f"  {state.row_id}: {state.name}  robot={state.robot_id}")


def wait_for_motion_review(
    node: StoredStateRunner,
    target: dict[str, float],
    duration_s: float,
    tolerance: float,
    timeout_s: float,
) -> None:
    deadline = time.monotonic() + max(timeout_s, duration_s)
    time.sleep(max(0.0, duration_s))

    while time.monotonic() < deadline and rclpy.ok():
        current = node.latest_positions()
        if not current:
            return
        errors = [
            abs(current[name] - target[name])
            for name in JOINT_NAMES
            if name in current
        ]
        if errors and max(errors) <= tolerance:
            print(f"reached within {max(errors):.4f} rad")
            return
        time.sleep(0.1)

    current = node.latest_positions()
    if not current:
        print("warning: no /joint_states feedback received after command")
        return
    errors = [
        abs(current[name] - target[name])
        for name in JOINT_NAMES
        if name in current
    ]
    if errors:
        print(f"warning: max joint error still {max(errors):.4f} rad")


def magnet_summary(action: str) -> str:
    return f"magnet command: {action.upper()}"


def prompt_for_step(step: int, total: int, item: SequenceStep) -> str:
    print()
    print(f"step {step}/{total}")
    if item.kind == "magnet":
        print(magnet_summary(item.magnet_action or ""))
        return input("Press Enter to run this command, s to skip, q to quit: ").strip().lower()
    if item.state is None:
        raise RuntimeError(f"invalid state sequence item: {item.token}")
    print(state_summary(item.state))
    if item.manual:
        print("mode: manual duration")
    return input("Press Enter to run this stored state, s to skip, q to quit: ").strip().lower()


def prompt_for_manual_duration(state: StoredState, default_duration: float) -> float | None:
    while True:
        reply = input(
            f"Duration for '{state.name}' in seconds [{default_duration:.2f}], q to quit: "
        ).strip().lower()
        if not reply:
            return default_duration
        if reply in {"q", "quit", "exit"}:
            return None
        try:
            duration = float(reply)
        except ValueError:
            print("Enter a positive number of seconds, or q to quit.")
            continue
        if duration <= 0.0:
            print("Duration must be greater than zero.")
            continue
        return duration


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Replay MoveIt warehouse Stored States by name or row ID."
    )
    parser.add_argument("--database", required=True, help="MoveIt warehouse SQLite database.")
    parser.add_argument(
        "--sequence",
        default="",
        help="Comma-separated stored state names or row IDs to execute in order.",
    )
    parser.add_argument(
        "--robot-id",
        default="simple_assembly_tracking",
        help="Only load stored states for this robot ID. Empty string disables filtering.",
    )
    parser.add_argument("--list", action="store_true", help="List available states and exit.")
    parser.add_argument(
        "--trajectory-topic",
        default="/arm_controller/joint_trajectory",
        help="JointTrajectory topic for the active ros2_control arm controller.",
    )
    parser.add_argument("--joint-topic", default="/joint_states")
    parser.add_argument("--duration", type=float, default=5.0)
    parser.add_argument("--goal-tolerance", type=float, default=0.05)
    parser.add_argument("--settle-timeout", type=float, default=8.0)
    parser.add_argument("--startup-wait", type=float, default=10.0)
    parser.add_argument(
        "--skip-confirmation",
        "--skip-confirm",
        "--no-confirm",
        "--yes",
        action="store_true",
        help="Run sequence entries without the per-step Enter approval prompt.",
    )
    parser.add_argument("--gpio-chip", default="gpiochip4")
    parser.add_argument("--gpio-line", type=int, default=17)
    parser.add_argument(
        "--keep-magnet-on-exit",
        action="store_true",
        help="Do not force the magnet low when the script exits.",
    )
    parser.add_argument("--dry-run", action="store_true")
    args = parser.parse_args()

    database_path = Path(args.database).expanduser().resolve()
    if not database_path.exists():
        print(f"warehouse database not found: {database_path}", file=sys.stderr)
        return 1

    try:
        states = load_stored_states(database_path, args.robot_id)
        sequence = parse_sequence(args.sequence)
        selected = select_sequence(states, sequence) if sequence else []
    except Exception as exc:
        print(f"failed to load stored states: {exc}", file=sys.stderr)
        return 1

    if args.list:
        print(f"warehouse database: {database_path}")
        print_available_states(states)
        return 0

    if not sequence:
        print(f"warehouse database: {database_path}")
        print_available_states(states)
        print()
        print("No sequence configured. Edit the wrapper script sequence or pass --sequence.")
        return 1

    print(f"warehouse database: {database_path}")
    print("replay sequence: " + ", ".join(sequence_step_label(item) for item in selected))
    print(f"duration per state: {args.duration:.2f}s")
    if args.skip_confirmation:
        print("confirmation prompts: skipped")

    rclpy.init()
    node = StoredStateRunner(args.trajectory_topic, args.joint_topic)
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()
    magnet = MagnetController(args.gpio_chip, args.gpio_line, args.dry_run)

    try:
        if not args.dry_run:
            if not node.wait_for_controller_subscriber(args.startup_wait):
                print(
                    f"no subscriber on {args.trajectory_topic}. Start the CAN HAT robot "
                    "launch first, for example scripts/pi/canhat/tracking_moveit.sh",
                    file=sys.stderr,
                )
                return 1
            if not node.wait_for_joint_states(args.startup_wait):
                print(
                    f"warning: no {args.joint_topic} feedback yet; commands will still publish",
                    file=sys.stderr,
                )

        for step, item in enumerate(selected, start=1):
            if args.skip_confirmation:
                print()
                print(f"step {step}/{len(selected)}")
                if item.kind == "magnet":
                    print(magnet_summary(item.magnet_action or ""))
                elif item.state is not None:
                    print(state_summary(item.state))
                    if item.manual:
                        print("mode: manual duration")
            else:
                reply = prompt_for_step(step, len(selected), item)
                if reply in {"q", "quit", "exit"}:
                    print("stopping stored-state replay")
                    return 0
                if reply in {"s", "skip"}:
                    print(f"skipped sequence item '{item.token}'")
                    continue

            if item.kind == "magnet":
                if item.magnet_action == "on":
                    magnet.on()
                elif item.magnet_action == "off":
                    magnet.off()
                continue

            if item.state is None:
                print(f"invalid sequence item: {item.token}", file=sys.stderr)
                return 1

            duration = args.duration
            if item.manual:
                manual_duration = prompt_for_manual_duration(item.state, args.duration)
                if manual_duration is None:
                    print("stopping stored-state replay")
                    return 0
                duration = manual_duration

            if args.dry_run:
                print(f"dry-run: would publish stored state '{item.state.name}' for {duration:.2f}s")
            else:
                target = dict(zip(JOINT_NAMES, item.state.positions))
                node.publish_state(item.state.positions, duration)
                print(f"sent stored state '{item.state.name}' for {duration:.2f}s")
                if item.manual:
                    time.sleep(max(0.0, duration))
                    print("manual duration elapsed; advancing to next sequence item")
                else:
                    wait_for_motion_review(
                        node,
                        target,
                        duration_s=duration,
                        tolerance=args.goal_tolerance,
                        timeout_s=args.settle_timeout,
                    )

        print("stored-state replay complete")
        return 0
    except KeyboardInterrupt:
        print("\nstopping stored-state replay")
        return 130
    finally:
        if not args.keep_magnet_on_exit:
            magnet.off()
        node.destroy_node()
        rclpy.shutdown()
        spin_thread.join(timeout=1.0)


if __name__ == "__main__":
    raise SystemExit(main())
