#!/usr/bin/env python3
"""Execute pose-only JSONL waypoints through MoveIt with per-step approval."""

from __future__ import annotations

import argparse
import json
import math
import subprocess
import sys
import threading
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import rclpy
from geometry_msgs.msg import Pose, Quaternion
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    BoundingVolume,
    Constraints,
    MoveItErrorCodes,
    MotionPlanRequest,
    OrientationConstraint,
    PlanningOptions,
    PositionConstraint,
)
from rclpy.action import ActionClient
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import JointState
from shape_msgs.msg import SolidPrimitive


@dataclass
class PathStep:
    line_no: int
    index: int
    comment: str
    frame_id: str
    link_name: str
    pose: Pose | None
    command: str | None


class MagnetController:
    def __init__(self, gpio_chip: str, gpio_line: int, dry_run: bool) -> None:
        self.gpio_chip = gpio_chip
        self.gpio_line = gpio_line
        self.dry_run = dry_run
        self.proc: subprocess.Popen | None = None

    def run(self, command: str) -> None:
        if command == "magnet_on":
            self.on()
        elif command == "magnet_off":
            self.off()
        else:
            raise RuntimeError(f"Unsupported command: {command}")

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
        try:
            subprocess.run(
                ["gpioset", self.gpio_chip, f"{self.gpio_line}=0"],
                check=False,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
            )
        except FileNotFoundError as exc:
            raise RuntimeError("gpioset not found - install gpiod") from exc


class EefPathRunner(Node):
    def __init__(self, move_action: str, joint_topic: str) -> None:
        super().__init__("canhat_eef_path_runner")
        self.move_group_client = ActionClient(self, MoveGroup, move_action)
        self.latest_joint_state: JointState | None = None
        self.lock = threading.Lock()
        self.create_subscription(JointState, joint_topic, self._on_joint_state, 10)

    def _on_joint_state(self, msg: JointState) -> None:
        with self.lock:
            self.latest_joint_state = msg

    def get_joint_state(self) -> JointState | None:
        with self.lock:
            return self.latest_joint_state

    def wait_for_joint_state(self, timeout_s: float) -> bool:
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline and rclpy.ok():
            if self.get_joint_state() is not None:
                return True
            time.sleep(0.05)
        return False

    def execute_pose(
        self,
        step: PathStep,
        position_tolerance: float,
        orientation_tolerance: float,
        constrain_orientation: bool,
        velocity_scale: float,
        acceleration_scale: float,
        planning_time: float,
        timeout_s: float,
    ) -> bool:
        if step.pose is None:
            return True

        if not self.move_group_client.wait_for_server(timeout_sec=timeout_s):
            print("MoveGroup action server is not available", file=sys.stderr)
            return False

        constraints = build_pose_constraints(
            step=step,
            position_tolerance=position_tolerance,
            orientation_tolerance=orientation_tolerance,
            constrain_orientation=constrain_orientation,
            node=self,
        )

        req = MotionPlanRequest()
        req.group_name = "arm"
        req.num_planning_attempts = 5
        req.allowed_planning_time = planning_time
        req.max_velocity_scaling_factor = velocity_scale
        req.max_acceleration_scaling_factor = acceleration_scale
        req.goal_constraints.append(constraints)

        joint_state = self.get_joint_state()
        if joint_state is not None:
            req.start_state.joint_state = joint_state
            req.start_state.is_diff = True

        options = PlanningOptions()
        options.plan_only = False
        options.planning_scene_diff.is_diff = True

        goal = MoveGroup.Goal()
        goal.request = req
        goal.planning_options = options

        goal_future = self.move_group_client.send_goal_async(goal)
        if not wait_for_future(goal_future, timeout_s):
            print("MoveGroup goal request timed out", file=sys.stderr)
            return False

        goal_handle = goal_future.result()
        if not goal_handle.accepted:
            print("MoveGroup rejected the goal", file=sys.stderr)
            return False

        result_future = goal_handle.get_result_async()
        if not wait_for_future(result_future, timeout_s):
            print("MoveGroup execution timed out", file=sys.stderr)
            return False

        result = result_future.result().result
        code = result.error_code.val
        if code != MoveItErrorCodes.SUCCESS:
            print(f"MoveGroup failed with error code {code}", file=sys.stderr)
            return False
        return True


def wait_for_future(future: Any, timeout_s: float) -> bool:
    deadline = time.monotonic() + timeout_s
    while rclpy.ok() and not future.done():
        if time.monotonic() > deadline:
            return False
        time.sleep(0.05)
    return future.done()


def build_pose_constraints(
    step: PathStep,
    position_tolerance: float,
    orientation_tolerance: float,
    constrain_orientation: bool,
    node: Node,
) -> Constraints:
    assert step.pose is not None

    sphere = SolidPrimitive()
    sphere.type = SolidPrimitive.SPHERE
    sphere.dimensions = [position_tolerance]

    region = BoundingVolume()
    region.primitives.append(sphere)
    region.primitive_poses.append(step.pose)

    pc = PositionConstraint()
    pc.header.frame_id = step.frame_id
    pc.header.stamp = node.get_clock().now().to_msg()
    pc.link_name = step.link_name
    pc.constraint_region = region
    pc.weight = 1.0

    constraints = Constraints()
    constraints.name = f"jsonl_waypoint_{step.index}"
    constraints.position_constraints.append(pc)

    if constrain_orientation:
        oc = OrientationConstraint()
        oc.header.frame_id = step.frame_id
        oc.header.stamp = node.get_clock().now().to_msg()
        oc.link_name = step.link_name
        oc.orientation = step.pose.orientation
        oc.absolute_x_axis_tolerance = orientation_tolerance
        oc.absolute_y_axis_tolerance = orientation_tolerance
        oc.absolute_z_axis_tolerance = orientation_tolerance
        oc.weight = 1.0
        constraints.orientation_constraints.append(oc)

    return constraints


def normalize_quaternion(values: dict[str, Any]) -> Quaternion:
    x = float(values["x"])
    y = float(values["y"])
    z = float(values["z"])
    w = float(values["w"])
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if norm < 1e-9:
        raise ValueError("orientation quaternion has zero length")
    q = Quaternion()
    q.x = x / norm
    q.y = y / norm
    q.z = z / norm
    q.w = w / norm
    return q


def parse_command(comment: str) -> str | None:
    text = comment.strip()
    if not text.startswith("!"):
        return None
    tokens = text[1:].strip().lower().split()
    if len(tokens) >= 2 and tokens[0] == "magnet" and tokens[1] == "on":
        return "magnet_on"
    if len(tokens) >= 2 and tokens[0] == "magnet" and tokens[1] == "off":
        return "magnet_off"
    raise ValueError(f"Unsupported path command comment: {comment!r}")


def pose_from_record(record: dict[str, Any]) -> tuple[Pose | None, str, str]:
    pose_data = record.get("pose")
    if pose_data is None:
        return None, str(record.get("fixed_frame", "root")), str(
            record.get("end_effector_frame", "camera_optical_frame")
        )

    position = pose_data["position"]
    quaternion = pose_data["quaternion"]

    pose = Pose()
    pose.position.x = float(position["x"])
    pose.position.y = float(position["y"])
    pose.position.z = float(position["z"])
    pose.orientation = normalize_quaternion(quaternion)

    frame_id = str(pose_data.get("frame_id") or record.get("fixed_frame") or "root")
    link_name = str(
        pose_data.get("child_frame_id")
        or record.get("end_effector_frame")
        or "camera_optical_frame"
    )
    return pose, frame_id, link_name


def load_jsonl_path(path: Path) -> list[PathStep]:
    steps: list[PathStep] = []
    with path.open("r", encoding="utf-8") as stream:
        for line_no, line in enumerate(stream, start=1):
            line = line.strip()
            if not line:
                continue
            record = json.loads(line)
            comment = str(record.get("comment", ""))
            command = parse_command(comment)
            pose, frame_id, link_name = pose_from_record(record)
            if pose is None and command is None:
                print(
                    f"warning: skipping line {line_no}; no pose and no ! command",
                    file=sys.stderr,
                )
                continue
            steps.append(
                PathStep(
                    line_no=line_no,
                    index=int(record.get("index", line_no - 1)),
                    comment=comment,
                    frame_id=frame_id,
                    link_name=link_name,
                    pose=pose,
                    command=command,
                )
            )
    return steps


def describe_step(step: PathStep, step_no: int, total: int) -> None:
    print()
    print(f"step {step_no}/{total} line={step.line_no} index={step.index}")
    print(f"comment: {step.comment or '(none)'}")
    if step.pose is None:
        print("pose: none")
    else:
        p = step.pose.position
        q = step.pose.orientation
        print(
            f"target {step.frame_id}->{step.link_name}: "
            f"pos=({p.x:+.3f},{p.y:+.3f},{p.z:+.3f}) "
            f"quat=({q.x:+.4f},{q.y:+.4f},{q.z:+.4f},{q.w:+.4f})"
        )
    print(f"command after pose: {step.command or 'none'}")


def main() -> int:
    parser = argparse.ArgumentParser(description="Execute pose-only JSONL waypoints with MoveIt.")
    parser.add_argument("path_file", help="Pose JSONL file to execute.")
    parser.add_argument("--move-action", default="/move_action")
    parser.add_argument("--joint-topic", default="/joint_states")
    parser.add_argument("--position-tolerance", type=float, default=0.04)
    parser.add_argument("--orientation-tolerance", type=float, default=0.2)
    parser.add_argument("--no-orientation", action="store_true")
    parser.add_argument("--velocity-scale", type=float, default=0.3)
    parser.add_argument("--acceleration-scale", type=float, default=0.3)
    parser.add_argument("--planning-time", type=float, default=5.0)
    parser.add_argument("--move-timeout", type=float, default=30.0)
    parser.add_argument("--startup-wait", type=float, default=10.0)
    parser.add_argument("--gpio-chip", default="gpiochip4")
    parser.add_argument("--gpio-line", type=int, default=17)
    parser.add_argument("--dry-run", action="store_true")
    parser.add_argument("--keep-magnet-on-exit", action="store_true")
    args = parser.parse_args()

    path_file = Path(args.path_file).expanduser().resolve()
    if path_file.suffix != ".jsonl":
        print(f"path must be a .jsonl file: {path_file}", file=sys.stderr)
        return 1
    if not path_file.exists():
        print(f"path file not found: {path_file}", file=sys.stderr)
        return 1

    try:
        steps = load_jsonl_path(path_file)
    except (ValueError, KeyError, json.JSONDecodeError) as exc:
        print(f"failed to parse {path_file}: {exc}", file=sys.stderr)
        return 1

    if not steps:
        print(f"no executable steps in {path_file}", file=sys.stderr)
        return 1

    print(f"path file: {path_file}")
    print(f"steps: {len(steps)}")
    print("Only comments beginning with ! are commands.")

    rclpy.init()
    node = EefPathRunner(args.move_action, args.joint_topic)
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()
    magnet = MagnetController(args.gpio_chip, args.gpio_line, args.dry_run)

    try:
        if not args.dry_run:
            if not node.move_group_client.wait_for_server(timeout_sec=args.startup_wait):
                print(
                    f"MoveGroup action server not available at {args.move_action}. "
                    "Start scripts/pi/canhat/tracking_moveit.sh first.",
                    file=sys.stderr,
                )
                return 1
            if not node.wait_for_joint_state(args.startup_wait):
                print(
                    f"warning: no {args.joint_topic} received yet; MoveIt will plan "
                    "without an explicit current start state",
                    file=sys.stderr,
                )

        for step_no, step in enumerate(steps, start=1):
            describe_step(step, step_no, len(steps))
            reply = input("Press Enter to execute, s to skip, q to quit: ").strip().lower()
            if reply in {"q", "quit", "exit"}:
                print("stopping path")
                return 0
            if reply in {"s", "skip"}:
                print("skipped")
                continue

            if step.pose is not None:
                if args.dry_run:
                    print("dry-run: would execute MoveIt pose goal")
                else:
                    ok = node.execute_pose(
                        step,
                        position_tolerance=args.position_tolerance,
                        orientation_tolerance=args.orientation_tolerance,
                        constrain_orientation=not args.no_orientation,
                        velocity_scale=args.velocity_scale,
                        acceleration_scale=args.acceleration_scale,
                        planning_time=args.planning_time,
                        timeout_s=args.move_timeout,
                    )
                    if not ok:
                        print("stopping because pose execution failed", file=sys.stderr)
                        return 1
                    print("pose execution complete")

            if step.command is not None:
                magnet.run(step.command)

        print("path complete")
        return 0
    except KeyboardInterrupt:
        print("\nstopping path")
        return 130
    except RuntimeError as exc:
        print(str(exc), file=sys.stderr)
        return 1
    finally:
        if not args.keep_magnet_on_exit:
            try:
                magnet.off()
            except RuntimeError as exc:
                print(str(exc), file=sys.stderr)
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()
        spin_thread.join(timeout=1.0)


if __name__ == "__main__":
    raise SystemExit(main())
