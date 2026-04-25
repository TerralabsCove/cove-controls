#!/usr/bin/env python3
"""Conservative, step-by-step AprilTag sweep/autotune helper.

Workflow per profile (human-confirmed each step):
1) Move to home pose (all joints zero by default).
2) Sweep revolute_6_0 slowly from home toward +100 degrees.
3) If a tag is detected during sweep, run a conservative approach loop.
4) Return home.
5) Wait for human input before next profile.
"""

import math
import threading
import time
from typing import Optional, Tuple

import rclpy
from apriltag_msgs.msg import AprilTagDetectionArray
from builtin_interfaces.msg import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


ALL_JOINTS = [
    "revolute_1_0",
    "revolute_2_0",
    "revolute_3_0",
    "revolute_4_0",
    "revolute_5_0",
    "revolute_6_0",
    "revolute_7_0",
]

LIMITS = {
    "revolute_1_0": (-1.9199, 1.9199),
    "revolute_2_0": (-math.pi, math.pi),
    "revolute_3_0": (-1.9199, 1.9199),
    "revolute_4_0": (-math.pi, math.pi),
    "revolute_5_0": (-1.9199, 1.9199),
    "revolute_6_0": (-math.pi, math.pi),
    "revolute_7_0": (-1.9199, 1.9199),
}

HOME = {joint: 0.0 for joint in ALL_JOINTS}

# Conservative first; gradually increases aggressiveness.
PROFILES = [
    {
        "name": "safe-1",
        "gain": 0.00020,
        "deadband_px": 70.0,
        "max_step_rad": 0.008,
        "traj_duration_s": 0.60,
        "approach_timeout_s": 18.0,
    },
    {
        "name": "safe-2",
        "gain": 0.00030,
        "deadband_px": 60.0,
        "max_step_rad": 0.012,
        "traj_duration_s": 0.50,
        "approach_timeout_s": 20.0,
    },
    {
        "name": "safe-3",
        "gain": 0.00040,
        "deadband_px": 50.0,
        "max_step_rad": 0.016,
        "traj_duration_s": 0.42,
        "approach_timeout_s": 22.0,
    },
    {
        "name": "safe-4",
        "gain": 0.00055,
        "deadband_px": 42.0,
        "max_step_rad": 0.020,
        "traj_duration_s": 0.36,
        "approach_timeout_s": 24.0,
    },
]


def clamp(value: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, value))


class TrackingAutotune(Node):
    def __init__(self) -> None:
        super().__init__("tracking_autotune_sweep")

        self.sweep_joint = str(self.declare_parameter("sweep_joint", "revolute_6_0").value)
        self.sweep_max_deg = float(self.declare_parameter("sweep_max_deg", 100.0).value)
        self.scan_step_deg = float(self.declare_parameter("scan_step_deg", 2.5).value)
        self.scan_step_time = float(self.declare_parameter("scan_step_time", 0.80).value)
        self.center_x = float(self.declare_parameter("center_x", 320.0).value)
        self.center_y = float(self.declare_parameter("center_y", 240.0).value)
        self.target_tag_id = int(self.declare_parameter("tag_id", 0).value)
        self.track_any_tag_if_missing = bool(
            self.declare_parameter("track_any_tag_if_missing", True).value
        )
        self.approach_axis = str(self.declare_parameter("approach_axis", "y").value).lower()
        self.approach_sign = float(self.declare_parameter("approach_sign", -1.0).value)
        self.return_home_duration = float(
            self.declare_parameter("return_home_duration", 4.0).value
        )
        self.detection_stale_timeout = float(
            self.declare_parameter("detection_stale_timeout", 0.6).value
        )

        if self.sweep_joint not in ALL_JOINTS:
            raise ValueError(f"sweep_joint must be one of {ALL_JOINTS}, got {self.sweep_joint}")
        if self.approach_axis not in ("x", "y"):
            raise ValueError("approach_axis must be 'x' or 'y'")

        self.sweep_max_rad = math.radians(self.sweep_max_deg)

        self.positions = dict(HOME)
        self.joint_states_received = False
        self.latest_detection: Optional[Tuple[float, float, float]] = None  # (x, y, time)
        self.lock = threading.Lock()

        self.traj_pub = self.create_publisher(JointTrajectory, "/arm_controller/joint_trajectory", 10)
        self.create_subscription(JointState, "/joint_states", self.on_joint_state, 10)
        self.create_subscription(AprilTagDetectionArray, "/detections", self.on_detection, 10)

        self.get_logger().info(
            f"Autotune ready sweep_joint={self.sweep_joint} "
            f"sweep_max_deg={self.sweep_max_deg:.1f} "
            f"scan_step_deg={self.scan_step_deg:.2f} "
            f"scan_step_time={self.scan_step_time:.2f}s"
        )

    def on_joint_state(self, msg: JointState) -> None:
        with self.lock:
            for name, pos in zip(msg.name, msg.position):
                if name in self.positions:
                    self.positions[name] = pos
            self.joint_states_received = True

    def on_detection(self, msg: AprilTagDetectionArray) -> None:
        if not msg.detections:
            return
        det = self._select_detection(msg.detections)
        if det is None:
            return
        with self.lock:
            self.latest_detection = (float(det.centre.x), float(det.centre.y), time.monotonic())

    def _select_detection(self, detections):
        selected = next(
            (d for d in detections if self._detection_id(d) == self.target_tag_id),
            None,
        )
        if selected is not None:
            return selected
        if self.track_any_tag_if_missing:
            return detections[0]
        return None

    @staticmethod
    def _detection_id(detection):
        tag_id = detection.id
        if isinstance(tag_id, (list, tuple)):
            return int(tag_id[0]) if tag_id else None
        return int(tag_id)

    def _snapshot_positions(self):
        with self.lock:
            return dict(self.positions), self.joint_states_received

    def _recent_detection(self) -> Optional[Tuple[float, float]]:
        with self.lock:
            det = self.latest_detection
        if det is None:
            return None
        x, y, t = det
        if time.monotonic() - t > self.detection_stale_timeout:
            return None
        return x, y

    def _publish_positions(self, target_positions: dict[str, float], duration_s: float) -> None:
        traj = JointTrajectory()
        traj.header.stamp = self.get_clock().now().to_msg()
        traj.joint_names = ALL_JOINTS

        pt = JointTrajectoryPoint()
        pt.positions = [float(target_positions[j]) for j in ALL_JOINTS]
        pt.velocities = [0.0] * len(ALL_JOINTS)
        ns = int((duration_s % 1.0) * 1e9)
        pt.time_from_start = Duration(sec=int(duration_s), nanosec=ns)
        traj.points = [pt]
        self.traj_pub.publish(traj)

    def wait_for_joint_states(self, timeout_s: float = 10.0) -> bool:
        start = time.monotonic()
        while time.monotonic() - start < timeout_s:
            _, ready = self._snapshot_positions()
            if ready:
                return True
            time.sleep(0.05)
        return False

    def go_home(self) -> None:
        current, _ = self._snapshot_positions()
        target = dict(current)
        for joint, value in HOME.items():
            lo, hi = LIMITS[joint]
            target[joint] = clamp(value, lo, hi)
        self._publish_positions(target, self.return_home_duration)
        time.sleep(self.return_home_duration + 0.1)

    def sweep_until_detected(self) -> bool:
        start_positions, _ = self._snapshot_positions()
        start = start_positions[self.sweep_joint]
        end = clamp(self.sweep_max_rad, *LIMITS[self.sweep_joint])

        step_rad = math.radians(self.scan_step_deg)
        span = max(0.0, end - start)
        step_count = max(1, int(math.ceil(span / step_rad)))

        self.get_logger().info(
            f"Sweeping {self.sweep_joint} from {start:.3f} -> {end:.3f} rad "
            f"in {step_count} steps"
        )

        for step in range(1, step_count + 1):
            alpha = float(step) / float(step_count)
            target_val = start + alpha * (end - start)
            now_positions, _ = self._snapshot_positions()
            now_positions[self.sweep_joint] = target_val
            self._publish_positions(now_positions, self.scan_step_time)
            time.sleep(self.scan_step_time + 0.02)
            if self._recent_detection() is not None:
                self.get_logger().info("Tag detected during sweep")
                return True

        self.get_logger().warn("No tag seen on positive sweep path; returning to start")
        for step in range(step_count - 1, -1, -1):
            alpha = float(step) / float(step_count)
            target_val = start + alpha * (end - start)
            now_positions, _ = self._snapshot_positions()
            now_positions[self.sweep_joint] = target_val
            self._publish_positions(now_positions, self.scan_step_time)
            time.sleep(self.scan_step_time + 0.02)

        return False

    def _error_from_detection(self, det_xy: Tuple[float, float]) -> float:
        x, y = det_xy
        return (x - self.center_x) if self.approach_axis == "x" else (y - self.center_y)

    def approach_with_profile(self, profile: dict) -> None:
        gain = float(profile["gain"])
        deadband_px = float(profile["deadband_px"])
        max_step = float(profile["max_step_rad"])
        traj_duration = float(profile["traj_duration_s"])
        timeout_s = float(profile["approach_timeout_s"])

        self.get_logger().info(
            f"Approach {profile['name']} "
            f"gain={gain:.6f} deadband={deadband_px:.1f}px "
            f"max_step={max_step:.4f}rad traj={traj_duration:.2f}s"
        )

        start = time.monotonic()
        sign = self.approach_sign
        worsen_count = 0
        prev_abs_err = None
        best = float("inf")

        while time.monotonic() - start < timeout_s:
            det = self._recent_detection()
            if det is None:
                self.get_logger().warn("Detection lost during approach; stopping step")
                break

            err = self._error_from_detection(det)
            abs_err = abs(err)
            best = min(best, abs_err)

            if abs_err <= deadband_px:
                self.get_logger().info(
                    f"Approach complete in deadband ({abs_err:.1f}px <= {deadband_px:.1f}px)"
                )
                break

            # If error keeps growing, flip command sign automatically.
            if prev_abs_err is not None and abs_err > prev_abs_err * 1.15:
                worsen_count += 1
            else:
                worsen_count = 0
            if worsen_count >= 3:
                sign *= -1.0
                worsen_count = 0
                self.get_logger().warn(f"Error increasing; flipping approach sign to {sign:+.1f}")

            delta = clamp(sign * gain * err, -max_step, max_step)
            now_positions, _ = self._snapshot_positions()
            current = now_positions[self.sweep_joint]
            lo, hi = LIMITS[self.sweep_joint]
            now_positions[self.sweep_joint] = clamp(current + delta, lo, hi)
            self._publish_positions(now_positions, traj_duration)
            time.sleep(traj_duration + 0.02)

            prev_abs_err = abs_err

        self.get_logger().info(f"Best |error| in approach: {best:.1f}px")

    def run(self) -> None:
        if not self.wait_for_joint_states():
            raise RuntimeError("Did not receive /joint_states; is robot.launch running?")

        print("\nAutotune plan:")
        for idx, profile in enumerate(PROFILES, start=1):
            print(
                f"  {idx}. {profile['name']}: gain={profile['gain']:.6f}, "
                f"deadband={profile['deadband_px']:.1f}px, "
                f"max_step={profile['max_step_rad']:.4f}rad, "
                f"traj={profile['traj_duration_s']:.2f}s"
            )
        print("\nEach step waits for Enter before motion.\n")

        self.go_home()

        for idx, profile in enumerate(PROFILES, start=1):
            prompt = f"[Step {idx}/{len(PROFILES)}] Press Enter to start '{profile['name']}' (q to quit): "
            ans = input(prompt).strip().lower()
            if ans in ("q", "quit", "n", "no"):
                print("Autotune stopped by user.")
                break

            self.go_home()
            found = self.sweep_until_detected()
            if found:
                self.approach_with_profile(profile)
            else:
                self.get_logger().warn("Skipping approach for this step (no tag detected)")
            self.go_home()
            print(f"[Step {idx}] complete. Robot returned to home.\n")


def main(args=None):
    rclpy.init(args=args)
    node = TrackingAutotune()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    try:
        node.run()
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()
        spin_thread.join(timeout=1.0)


if __name__ == "__main__":
    main()

