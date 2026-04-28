from __future__ import annotations

import time
from typing import Callable, Iterable, Sequence, Tuple

from action_msgs.msg import GoalStatus
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient
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

WAYPOINTS = {
    "home": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    "pre_pick": [0.0, -1.0, 0.3, -0.5, -0.3, 0.0, 0.0],
    "pick": [0.0, -1.4, 0.6, -0.8, -0.3, 0.0, 0.0],
    "lift": [0.0, -0.8, 0.3, -0.4, -0.3, 0.0, 0.0],
    "transit": [1.2, -0.6, 0.2, -0.3, -0.2, 0.0, 0.0],
    "pre_place": [1.2, -1.0, 0.3, -0.5, -0.3, 0.0, 0.0],
    "place": [1.2, -1.4, 0.6, -0.8, -0.3, 0.0, 0.0],
    "retreat": [1.2, -0.6, 0.2, -0.3, -0.2, 0.0, 0.0],
}

POURING_SEQUENCE: Sequence[Tuple[str, str]] = (
    ("move", "home"),
    ("move", "pre_pick"),
    ("move", "pick"),
    ("gripper", "close"),
    ("move", "lift"),
)

MOVING_SEQUENCE: Sequence[Tuple[str, str]] = (
    ("move", "transit"),
    ("move", "pre_place"),
    ("move", "place"),
    ("gripper", "open"),
    ("move", "retreat"),
    ("move", "home"),
)

SIMULATED_DELAYS = {
    "home": 0.8,
    "pre_pick": 1.0,
    "pick": 1.1,
    "lift": 1.0,
    "transit": 1.2,
    "pre_place": 1.0,
    "place": 1.1,
    "retreat": 0.9,
    "close": 0.8,
    "open": 0.8,
}


class MotionExecutionError(RuntimeError):
    """Raised when the motion sequence fails or is aborted."""


class CoveMotionController:
    def __init__(self, node, simulate_only: bool = False, allow_simulation_fallback: bool = True):
        self._node = node
        self.simulate_only = simulate_only
        self.allow_simulation_fallback = allow_simulation_fallback
        self._goal_handle = None
        self._trajectory_client = None

        if self.simulate_only:
            self._node.get_logger().info("COVE motion controller starting in simulated mode.")
            return

        self._trajectory_client = ActionClient(
            self._node,
            FollowJointTrajectory,
            "/arm_controller/follow_joint_trajectory",
        )
        self._node.get_logger().info("COVE motion controller connected to arm_controller trajectory action.")

    def wait_until_ready(self, timeout_sec: float = 3.0) -> bool:
        if self.simulate_only or self._trajectory_client is None:
            return True
        return self._trajectory_client.wait_for_server(timeout_sec=timeout_sec)

    def maybe_cancel(self) -> None:
        if self._goal_handle is None:
            return
        try:
            self._goal_handle.cancel_goal_async()
        except Exception as exc:
            self._node.get_logger().warning(f"Failed to cancel trajectory execution: {exc}")
        finally:
            self._goal_handle = None

    def run_pouring_sequence(self, should_abort: Callable[[], bool]) -> None:
        self._run_sequence(POURING_SEQUENCE, should_abort)

    def run_moving_sequence(self, should_abort: Callable[[], bool]) -> None:
        self._run_sequence(MOVING_SEQUENCE, should_abort)

    def _run_sequence(
        self,
        sequence: Iterable[Tuple[str, str]],
        should_abort: Callable[[], bool],
    ) -> None:
        self._ensure_ready()
        for action, target in sequence:
            if should_abort():
                raise MotionExecutionError("Execution interrupted by operator fault.")

            if action == "gripper":
                self._sleep_with_abort(SIMULATED_DELAYS[target], should_abort)
                continue

            if self.simulate_only:
                self._sleep_with_abort(SIMULATED_DELAYS.get(target, 1.0), should_abort)
                continue

            self._execute_waypoint(target, should_abort)

    def _ensure_ready(self) -> None:
        if self.simulate_only:
            return
        if self.wait_until_ready(timeout_sec=5.0):
            return
        if self.allow_simulation_fallback:
            self.simulate_only = True
            self._node.get_logger().warning(
                "Trajectory action did not become ready in time; switching to simulated execution."
            )
            return
        raise MotionExecutionError("arm_controller trajectory action is not ready.")

    def _execute_waypoint(self, target: str, should_abort: Callable[[], bool]) -> None:
        if self._trajectory_client is None:
            raise MotionExecutionError("Trajectory action client is not configured.")

        self._node.get_logger().info(f"Moving arm to '{target}'.")
        trajectory = JointTrajectory()
        trajectory.joint_names = list(JOINT_NAMES)

        point = JointTrajectoryPoint()
        point.positions = list(WAYPOINTS[target])
        point.time_from_start = self._duration_msg(SIMULATED_DELAYS.get(target, 1.0))
        trajectory.points = [point]

        goal = FollowJointTrajectory.Goal()
        goal.trajectory = trajectory

        goal_future = self._trajectory_client.send_goal_async(goal)
        goal_handle = self._wait_for_future(goal_future, should_abort, "Timed out waiting for trajectory goal acceptance.")
        if goal_handle is None or not goal_handle.accepted:
            raise MotionExecutionError(f"Trajectory controller rejected waypoint '{target}'.")

        self._goal_handle = goal_handle
        result_future = goal_handle.get_result_async()
        result = self._wait_for_future(result_future, should_abort, f"Timed out executing waypoint '{target}'.")
        self._goal_handle = None

        if result is None:
            raise MotionExecutionError(f"Trajectory execution returned no result for '{target}'.")
        if result.status != GoalStatus.STATUS_SUCCEEDED:
            raise MotionExecutionError(f"Trajectory execution did not succeed for '{target}' (status {result.status}).")
        if getattr(result.result, "error_code", 0) != 0:
            raise MotionExecutionError(
                f"Trajectory controller reported error {result.result.error_code} for '{target}'."
            )

    def _wait_for_future(self, future, should_abort: Callable[[], bool], timeout_message: str):
        deadline = time.monotonic() + 20.0
        while not future.done():
            if should_abort():
                self.maybe_cancel()
                raise MotionExecutionError("Execution interrupted by operator fault.")
            if time.monotonic() >= deadline:
                self.maybe_cancel()
                raise MotionExecutionError(timeout_message)
            time.sleep(0.05)
        return future.result()

    @staticmethod
    def _duration_msg(duration_sec: float) -> Duration:
        seconds = max(0.1, float(duration_sec))
        whole = int(seconds)
        nanos = int((seconds - whole) * 1_000_000_000)
        return Duration(sec=whole, nanosec=nanos)

    @staticmethod
    def _sleep_with_abort(duration_sec: float, should_abort: Callable[[], bool]) -> None:
        deadline = time.monotonic() + duration_sec
        while time.monotonic() < deadline:
            if should_abort():
                raise MotionExecutionError("Execution interrupted by operator fault.")
            time.sleep(0.05)
