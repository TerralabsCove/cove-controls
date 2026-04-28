#!/usr/bin/env python3
"""Append manual arm waypoint captures to a JSONL file.

Each row can contain either a full /joint_states sample plus TF transforms, or a
single end-effector pose. The script is intentionally interactive: type an
optional comment and press Enter to append a capture, then move the arm and
repeat.
"""

from __future__ import annotations

import argparse
import json
import sys
import threading
import time
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

import rclpy
from rclpy.duration import Duration
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import JointState
from tf2_ros import Buffer, TransformException, TransformListener


DEFAULT_TARGET_FRAMES = ("wrist_link", "camera_optical_frame")
DEFAULT_POSE_ONLY_FRAME = "wrist_link"


def stamp_to_dict(stamp: Any) -> dict[str, int]:
    return {"sec": int(stamp.sec), "nanosec": int(stamp.nanosec)}


def now_iso() -> str:
    return datetime.now(timezone.utc).isoformat()


def parse_target_frames(values: list[str]) -> list[str]:
    frames: list[str] = []
    for value in values:
        for frame in value.split(","):
            frame = frame.strip()
            if frame and frame not in frames:
                frames.append(frame)
    return frames


def joint_state_to_dict(msg: JointState | None) -> dict[str, Any] | None:
    if msg is None:
        return None

    names = list(msg.name)
    positions = list(msg.position)
    velocities = list(msg.velocity)
    efforts = list(msg.effort)

    by_name: dict[str, dict[str, float | None]] = {}
    for index, name in enumerate(names):
        by_name[name] = {
            "position": positions[index] if index < len(positions) else None,
            "velocity": velocities[index] if index < len(velocities) else None,
            "effort": efforts[index] if index < len(efforts) else None,
        }

    return {
        "header": {
            "frame_id": msg.header.frame_id,
            "stamp": stamp_to_dict(msg.header.stamp),
        },
        "name": names,
        "position": positions,
        "velocity": velocities,
        "effort": efforts,
        "by_name": by_name,
    }


class WaypointRecorder(Node):
    def __init__(
        self,
        joint_topic: str,
        fixed_frame: str,
        target_frames: list[str],
        tf_timeout: float,
        record_joint_state: bool,
    ) -> None:
        super().__init__("canhat_waypoint_recorder")
        self.fixed_frame = fixed_frame
        self.target_frames = target_frames
        self.tf_timeout = tf_timeout
        self.record_joint_state = record_joint_state
        self._joint_lock = threading.Lock()
        self._latest_joint_state: JointState | None = None

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        if self.record_joint_state:
            self.create_subscription(JointState, joint_topic, self._on_joint_state, 10)

    def _on_joint_state(self, msg: JointState) -> None:
        with self._joint_lock:
            self._latest_joint_state = msg

    def wait_for_joint_state(self, timeout_sec: float) -> bool:
        deadline = time.monotonic() + timeout_sec
        while time.monotonic() < deadline and rclpy.ok():
            with self._joint_lock:
                if self._latest_joint_state is not None:
                    return True
            time.sleep(0.05)
        return False

    def wait_for_tf(self, timeout_sec: float) -> bool:
        deadline = time.monotonic() + timeout_sec
        while time.monotonic() < deadline and rclpy.ok():
            if all(self._lookup_transform(frame)[0] is not None for frame in self.target_frames):
                return True
            time.sleep(0.05)
        return False

    def _lookup_transform(self, target_frame: str) -> tuple[Any | None, str | None]:
        try:
            transform = self.tf_buffer.lookup_transform(
                self.fixed_frame,
                target_frame,
                Time(),
                timeout=Duration(seconds=self.tf_timeout),
            )
            return transform, None
        except TransformException as exc:
            return None, f"TF lookup failed {self.fixed_frame}->{target_frame}: {exc}"

    def capture_transforms(self) -> tuple[dict[str, dict[str, Any] | None], list[str]]:
        errors: list[str] = []
        transforms: dict[str, dict[str, Any] | None] = {}
        for target_frame in self.target_frames:
            transform, error = self._lookup_transform(target_frame)
            if transform is None:
                transforms[target_frame] = None
                if error is not None:
                    errors.append(error)
                continue

            translation = transform.transform.translation
            rotation = transform.transform.rotation
            transforms[target_frame] = {
                "header": {
                    "frame_id": transform.header.frame_id,
                    "stamp": stamp_to_dict(transform.header.stamp),
                },
                "child_frame_id": transform.child_frame_id,
                "translation": {
                    "x": translation.x,
                    "y": translation.y,
                    "z": translation.z,
                },
                "quaternion": {
                    "x": rotation.x,
                    "y": rotation.y,
                    "z": rotation.z,
                    "w": rotation.w,
                },
            }
        return transforms, errors

    def capture(self, index: int, comment: str, pose_only: bool) -> dict[str, Any]:
        transforms, errors = self.capture_transforms()

        if pose_only:
            target_frame = self.target_frames[0]
            transform = transforms.get(target_frame)
            pose = None
            if transform is not None:
                pose = {
                    "frame_id": transform["header"]["frame_id"],
                    "child_frame_id": transform["child_frame_id"],
                    "stamp": transform["header"]["stamp"],
                    "position": transform["translation"],
                    "quaternion": transform["quaternion"],
                }

            ros_now = self.get_clock().now().to_msg()
            return {
                "schema_version": 2,
                "index": index,
                "comment": comment,
                "wall_time_utc": now_iso(),
                "ros_time": stamp_to_dict(ros_now),
                "fixed_frame": self.fixed_frame,
                "end_effector_frame": target_frame,
                "pose": pose,
                "errors": errors,
            }

        with self._joint_lock:
            joint_state = self._latest_joint_state

        if joint_state is None:
            errors.append("No /joint_states message has been received yet")

        ros_now = self.get_clock().now().to_msg()
        return {
            "schema_version": 1,
            "index": index,
            "comment": comment,
            "wall_time_utc": now_iso(),
            "ros_time": stamp_to_dict(ros_now),
            "fixed_frame": self.fixed_frame,
            "target_frames": list(self.target_frames),
            "joint_state": joint_state_to_dict(joint_state),
            "transforms": transforms,
            "errors": errors,
        }


def count_existing_records(path: Path) -> int:
    if not path.exists():
        return 0
    with path.open("r", encoding="utf-8") as stream:
        return sum(1 for line in stream if line.strip())


def print_saved_summary(record: dict[str, Any], output: Path) -> None:
    if record.get("schema_version") == 2:
        pose = record.get("pose")
        error_count = len(record["errors"])
        if pose is None:
            pose_text = "pose=none"
        else:
            position = pose["position"]
            quaternion = pose["quaternion"]
            pose_text = (
                f"pos=({position['x']:+.3f},{position['y']:+.3f},{position['z']:+.3f}) "
                f"quat=({quaternion['x']:+.4f},{quaternion['y']:+.4f},"
                f"{quaternion['z']:+.4f},{quaternion['w']:+.4f})"
            )
        print(
            f"saved #{record['index']} to {output} "
            f"({record['end_effector_frame']} {pose_text}, errors={error_count})",
            flush=True,
        )
        for error in record["errors"]:
            print(f"  warning: {error}", flush=True)
        return

    joint_state = record["joint_state"]
    joint_count = len(joint_state["name"]) if joint_state else 0
    ok_frames = [
        frame for frame, transform in record["transforms"].items() if transform is not None
    ]
    error_count = len(record["errors"])
    print(
        f"saved #{record['index']} to {output} "
        f"({joint_count} joints, tf={','.join(ok_frames) or 'none'}, "
        f"errors={error_count})",
        flush=True,
    )
    for error in record["errors"]:
        print(f"  warning: {error}", flush=True)


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Record manual joint-state or end-effector-pose waypoints on Enter."
    )
    parser.add_argument(
        "--output",
        required=True,
        help="JSONL file to append captures to.",
    )
    parser.add_argument(
        "--joint-topic",
        default="/joint_states",
        help="JointState topic to sample.",
    )
    parser.add_argument(
        "--fixed-frame",
        default="root",
        help="TF fixed/base frame.",
    )
    parser.add_argument(
        "--target-frame",
        action="append",
        default=[],
        help=(
            "End-effector frame to record. Can be repeated or comma-separated. "
            "Defaults to wrist_link in pose-only mode, otherwise "
            "wrist_link,camera_optical_frame."
        ),
    )
    parser.add_argument(
        "--tf-timeout",
        type=float,
        default=0.5,
        help="Seconds to wait for each TF lookup per capture.",
    )
    parser.add_argument(
        "--startup-wait",
        type=float,
        default=10.0,
        help="Seconds to wait for the first joint state or TF before accepting captures.",
    )
    parser.add_argument(
        "--pose-only",
        action="store_true",
        help="Record only one end-effector Cartesian position and quaternion.",
    )
    args = parser.parse_args()

    target_frames = parse_target_frames(args.target_frame)
    if not target_frames:
        target_frames = [DEFAULT_POSE_ONLY_FRAME] if args.pose_only else list(DEFAULT_TARGET_FRAMES)
    if args.pose_only and len(target_frames) != 1:
        print(
            "--pose-only records exactly one end-effector frame; pass one --target-frame",
            file=sys.stderr,
        )
        return 1

    output = Path(args.output).expanduser().resolve()
    output.parent.mkdir(parents=True, exist_ok=True)

    rclpy.init()
    node = WaypointRecorder(
        joint_topic=args.joint_topic,
        fixed_frame=args.fixed_frame,
        target_frames=target_frames,
        tf_timeout=args.tf_timeout,
        record_joint_state=not args.pose_only,
    )
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    try:
        if args.startup_wait > 0.0:
            if args.pose_only:
                frame_text = f"{args.fixed_frame}->{target_frames[0]}"
                print(f"waiting up to {args.startup_wait:.1f}s for TF {frame_text}...")
            else:
                print(f"waiting up to {args.startup_wait:.1f}s for {args.joint_topic}...")

            ready = (
                node.wait_for_tf(args.startup_wait)
                if args.pose_only
                else node.wait_for_joint_state(args.startup_wait)
            )
            if not ready:
                wait_target = f"TF {args.fixed_frame}->{target_frames[0]}" if args.pose_only else args.joint_topic
                print(
                    f"warning: no {wait_target} received yet; captures will "
                    "still append with available data",
                    file=sys.stderr,
                )

        index = count_existing_records(output)
        print(f"recording to {output}")
        print(f"fixed frame: {args.fixed_frame}")
        if args.pose_only:
            print(f"end-effector frame: {target_frames[0]}")
            print("recording mode: pose-only")
        else:
            print(f"target frames: {', '.join(target_frames)}")
        print("type an optional comment then press Enter to save")
        print("type q, quit, or exit to stop")

        while rclpy.ok():
            try:
                comment = input("> ")
            except EOFError:
                print()
                break
            except KeyboardInterrupt:
                print()
                break

            if comment.strip().lower() in {"q", "quit", "exit"}:
                break

            record = node.capture(index=index, comment=comment, pose_only=args.pose_only)
            with output.open("a", encoding="utf-8") as stream:
                stream.write(json.dumps(record, separators=(",", ":")) + "\n")
                stream.flush()
            print_saved_summary(record, output)
            index += 1

    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()
        spin_thread.join(timeout=1.0)

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
