#!/usr/bin/env python3
"""Replay recorded JSONL arm waypoints through the joint trajectory controller."""

from __future__ import annotations

import argparse
import json
import subprocess
import sys
import threading
import time
from pathlib import Path
from typing import Any

import rclpy
from builtin_interfaces.msg import Duration
from rclpy.node import Node
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


class RecordedPathRunner(Node):
    def __init__(self, trajectory_topic: str, joint_topic: str) -> None:
        super().__init__("canhat_recorded_path_runner")
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

    def publish_waypoint(self, positions: list[float], duration_s: float) -> None:
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


def load_records(path: Path) -> list[dict[str, Any]]:
    records: list[dict[str, Any]] = []
    with path.open("r", encoding="utf-8") as stream:
        for line_no, line in enumerate(stream, start=1):
            line = line.strip()
            if not line:
                continue
            record = json.loads(line)
            record["_line_no"] = line_no
            record["_index"] = int(record.get("index", line_no - 1))
            records.append(record)
    return records


def parse_sequence(value: str) -> list[int]:
    if not value.strip():
        return []
    sequence = [int(item.strip(), 0) for item in value.split(",") if item.strip()]
    if not sequence:
        raise ValueError("sequence cannot be empty")
    return sequence


def select_waypoints(
    records: list[dict[str, Any]],
    sequence: list[int],
) -> list[tuple[int, dict[str, Any], list[float]]]:
    if sequence:
        by_index = {record["_index"]: record for record in records}
        return [
            (index, by_index[index], positions_from_record(by_index[index]))
            for index in sequence
        ]

    waypoints: list[tuple[int, dict[str, Any], list[float]]] = []
    for record in records:
        try:
            waypoints.append(
                (record["_index"], record, positions_from_record(record))
            )
        except ValueError:
            continue
    if not waypoints:
        raise ValueError("No records with full joint positions were found")
    return waypoints


def parse_magnet_actions(value: str, sequence_len: int) -> list[str | None]:
    if not value.strip():
        return []

    actions: list[str | None] = []
    for raw in value.split(","):
        action = raw.strip().lower()
        if action in {"", "none", "no", "skip", "-"}:
            actions.append(None)
        elif action in {"on", "enable"}:
            actions.append("on")
        elif action in {"off", "release", "disable"}:
            actions.append("off")
        elif action in {"comment", "infer", "auto"}:
            actions.append("comment")
        else:
            raise ValueError(
                "magnet actions must be on, off, none, or comment; got "
                f"'{raw.strip()}'"
            )

    if len(actions) != sequence_len:
        raise ValueError(
            f"magnet action count ({len(actions)}) must match sequence length "
            f"({sequence_len})"
        )
    return actions


def positions_from_record(record: dict[str, Any]) -> list[float]:
    joint_state = record.get("joint_state")
    if not joint_state:
        raise ValueError(f"waypoint #{record.get('index')} has no joint_state")

    by_name = joint_state.get("by_name") or {}
    if all(name in by_name and by_name[name].get("position") is not None for name in JOINT_NAMES):
        return [float(by_name[name]["position"]) for name in JOINT_NAMES]

    names = joint_state.get("name") or []
    positions = joint_state.get("position") or []
    position_by_name = {
        name: float(positions[i])
        for i, name in enumerate(names)
        if i < len(positions)
    }
    missing = [name for name in JOINT_NAMES if name not in position_by_name]
    if missing:
        raise ValueError(
            f"waypoint #{record.get('index')} is missing joint positions for: "
            + ", ".join(missing)
        )
    return [position_by_name[name] for name in JOINT_NAMES]


def infer_magnet_action(comment: str) -> str | None:
    text = comment.strip().lower()
    if not text.startswith("!"):
        return None
    tokens = text[1:].strip().split()
    if len(tokens) >= 2 and tokens[0] == "magnet" and tokens[1] == "on":
        return "on"
    if len(tokens) >= 2 and tokens[0] == "magnet" and tokens[1] == "off":
        return "off"
    raise ValueError(f"Unsupported command comment: {comment!r}")
    return None


def waypoint_summary(record: dict[str, Any], positions: list[float]) -> str:
    index = record.get("_index", record.get("index"))
    comment = str(record.get("comment", "")).strip()
    joint_text = " ".join(
        f"{name.replace('revolute_', 'j')}={position:+.3f}"
        for name, position in zip(JOINT_NAMES, positions)
    )
    transforms = record.get("transforms") or {}
    tf_name = next((name for name, value in transforms.items() if value is not None), "")
    tf = transforms.get(tf_name) if tf_name else None
    if tf:
        translation = tf["translation"]
        eef = (
            f"{tf_name}=({translation['x']:+.3f},"
            f"{translation['y']:+.3f},{translation['z']:+.3f})"
        )
    else:
        eef = "eef=(no tf)"
    return f"#{index} comment='{comment}' {eef}\n  {joint_text}"


def wait_for_motion_review(
    node: RecordedPathRunner,
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


def prompt_for_step(step: int, total: int, summary: str, action: str | None) -> str:
    print()
    print(f"step {step}/{total}")
    print(summary)
    print(f"magnet action after move: {action or 'none'}")
    return input("Press Enter to run this waypoint, s to skip, q to quit: ").strip().lower()


def main() -> int:
    parser = argparse.ArgumentParser(description="Replay recorded CAN HAT joint waypoints.")
    parser.add_argument(
        "--waypoint-file",
        required=True,
        help="Recorded JSONL waypoint file.",
    )
    parser.add_argument(
        "--sequence",
        default="",
        help="Comma-separated waypoint indices to replay. Default: all valid joint waypoints in file order.",
    )
    parser.add_argument(
        "--magnet-actions",
        default="",
        help=(
            "Optional comma-separated per-step magnet actions: on, off, none, "
            "or comment. When omitted, only !magnet on/off comments are commands."
        ),
    )
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
    parser.add_argument("--gpio-chip", default="gpiochip4")
    parser.add_argument("--gpio-line", type=int, default=17)
    parser.add_argument("--dry-run", action="store_true")
    parser.add_argument(
        "--keep-magnet-on-exit",
        action="store_true",
        help="Do not force the magnet low when the script exits.",
    )
    args = parser.parse_args()

    waypoint_path = Path(args.waypoint_file).expanduser().resolve()
    if not waypoint_path.exists():
        print(f"waypoint file not found: {waypoint_path}", file=sys.stderr)
        return 1

    try:
        sequence = parse_sequence(args.sequence)
        records = load_records(waypoint_path)
        waypoints = select_waypoints(records, sequence)
        magnet_actions = parse_magnet_actions(args.magnet_actions, len(waypoints))
    except KeyError as exc:
        print(f"waypoint index not found in {waypoint_path}: {exc}", file=sys.stderr)
        return 1
    except (ValueError, json.JSONDecodeError) as exc:
        print(f"invalid waypoint file or sequence: {exc}", file=sys.stderr)
        return 1

    print(f"waypoint file: {waypoint_path}")
    print(f"replay sequence: {', '.join(str(index) for index, _, _ in waypoints)}")
    if magnet_actions:
        action_text = [action if action != "comment" else "comment" for action in magnet_actions]
        print(f"magnet actions: {', '.join(action or 'none' for action in action_text)}")
    else:
        print("magnet actions: inferred from comments")
    print(f"duration per waypoint: {args.duration:.2f}s")

    rclpy.init()
    node = RecordedPathRunner(args.trajectory_topic, args.joint_topic)
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

        for step, (index, record, positions) in enumerate(waypoints, start=1):
            action = (
                magnet_actions[step - 1]
                if magnet_actions
                else infer_magnet_action(str(record.get("comment", "")))
            )
            if action == "comment":
                action = infer_magnet_action(str(record.get("comment", "")))
            reply = prompt_for_step(
                step,
                len(waypoints),
                waypoint_summary(record, positions),
                action,
            )
            if reply in {"q", "quit", "exit"}:
                print("stopping path replay")
                return 0
            if reply in {"s", "skip"}:
                print(f"skipped waypoint #{index}")
                continue

            if args.dry_run:
                print(f"dry-run: would publish waypoint #{index}")
            else:
                target = dict(zip(JOINT_NAMES, positions))
                node.publish_waypoint(positions, args.duration)
                print(f"sent waypoint #{index}")
                wait_for_motion_review(
                    node,
                    target,
                    duration_s=args.duration,
                    tolerance=args.goal_tolerance,
                    timeout_s=args.settle_timeout,
                )

            if action == "on":
                magnet.on()
            elif action == "off":
                magnet.off()

        print("path replay complete")
        return 0
    except KeyboardInterrupt:
        print("\nstopping path replay")
        return 130
    finally:
        if not args.keep_magnet_on_exit:
            magnet.off()
        node.destroy_node()
        rclpy.shutdown()
        spin_thread.join(timeout=1.0)


if __name__ == "__main__":
    raise SystemExit(main())
