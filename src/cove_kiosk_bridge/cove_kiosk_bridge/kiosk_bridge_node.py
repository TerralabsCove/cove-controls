from __future__ import annotations

import json
import threading
import time
from dataclasses import dataclass, field
from datetime import datetime
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Optional
from urllib.parse import urlparse

import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import JointState

from .motion_controller import CoveMotionController, MotionExecutionError


@dataclass
class OrderRecord:
    id: int
    name: str
    status: str = "queued"
    slot: Optional[int] = None
    isMe: bool = False
    created_at: float = field(default_factory=time.time)
    completed_at: Optional[float] = None

    def as_public_dict(self) -> dict:
        return {
            "id": self.id,
            "name": self.name,
            "status": self.status,
            "slot": self.slot,
            "isMe": self.isMe,
        }


class CoveKioskBridge(Node):
    PHASE_DURATIONS = {
        "received": 2.5,
        "pouring": 12.0,
        "moving": 7.0,
        "arrived": 30.0,
    }

    def __init__(self) -> None:
        super().__init__("cove_kiosk_bridge")
        self.declare_parameter("bind_host", "0.0.0.0")
        self.declare_parameter("bind_port", 8080)
        self.declare_parameter("frontend_path", "")
        self.declare_parameter("simulate_only", False)
        self.declare_parameter("allow_simulation_fallback", True)
        self.declare_parameter("slot_open_seconds", 30.0)
        self.declare_parameter("initial_order_id", 2617)
        self.declare_parameter("cups_remaining", 148)
        self.declare_parameter("cups_capacity", 200)
        self.declare_parameter("served_today", 37)
        self.declare_parameter("last_calibration", "2026-04-19 08:14")

        self._bind_host = str(self.get_parameter("bind_host").value)
        self._bind_port = int(self.get_parameter("bind_port").value)
        self._slot_open_seconds = float(self.get_parameter("slot_open_seconds").value)
        self._next_order_id = int(self.get_parameter("initial_order_id").value)
        self._cups_remaining = int(self.get_parameter("cups_remaining").value)
        self._cups_capacity = int(self.get_parameter("cups_capacity").value)
        self._served_today = int(self.get_parameter("served_today").value)
        self._last_calibration = str(self.get_parameter("last_calibration").value)
        self._frontend_path = self._resolve_frontend_path(str(self.get_parameter("frontend_path").value))

        self._lock = threading.RLock()
        self._condition = threading.Condition(self._lock)
        self._shutdown_event = threading.Event()
        self._orders: list[OrderRecord] = []
        self._state = "idle"
        self._fault_message = ""
        self._phase_started = time.monotonic()
        self._phase_duration = 0.0
        self._boot_time = time.time()
        self._last_joint_state_wall = 0.0
        self._current_order_id: Optional[int] = None
        self._average_cycle_seconds = 43.0

        simulate_only = bool(self.get_parameter("simulate_only").value)
        allow_fallback = bool(self.get_parameter("allow_simulation_fallback").value)
        self._motion = CoveMotionController(
            self,
            simulate_only=simulate_only,
            allow_simulation_fallback=allow_fallback,
        )

        self.create_subscription(JointState, "/joint_states", self._joint_state_cb, 10)

        self._http_server = self._create_http_server()
        self._http_thread = threading.Thread(target=self._http_server.serve_forever, daemon=True)
        self._http_thread.start()

        self._worker_thread = threading.Thread(target=self._queue_worker, daemon=True)
        self._worker_thread.start()

        self.get_logger().info(
            f"COVE kiosk bridge serving {self._frontend_path} on http://{self._bind_host}:{self._bind_port}"
        )

    def destroy_node(self) -> bool:
        self._shutdown_event.set()
        with self._condition:
            self._condition.notify_all()
        self._motion.maybe_cancel()
        self._http_server.shutdown()
        self._http_server.server_close()
        if self._worker_thread.is_alive():
            self._worker_thread.join(timeout=2.0)
        if self._http_thread.is_alive():
            self._http_thread.join(timeout=2.0)
        return super().destroy_node()

    def enqueue_order(self, name: str, is_me: bool) -> dict:
        cleaned = " ".join(name.split()).strip()
        if not cleaned:
            raise ValueError("Order name is required.")
        if len(cleaned) > 20:
            raise ValueError("Order name must be 20 characters or fewer.")

        with self._condition:
            order = OrderRecord(
                id=self._next_order_id,
                name=cleaned,
                isMe=is_me,
            )
            self._next_order_id += 1
            self._orders.append(order)
            self._condition.notify_all()
            return order.as_public_dict()

    def inject_fault(self, message: str = "Operator simulated fault.") -> dict:
        with self._condition:
            self._fault_message = message
            self._set_phase_locked("error", 0.0)
            self._motion.maybe_cancel()
            self._condition.notify_all()
        return self.snapshot()

    def reset_fault(self, clear_queue: bool = False) -> dict:
        with self._condition:
            self._fault_message = ""
            if clear_queue:
                self._orders = [order for order in self._orders if order.status == "done"]
            else:
                for order in self._orders:
                    if order.status == "active":
                        order.status = "queued"
            self._current_order_id = None
            self._set_phase_locked("idle", 0.0)
            self._condition.notify_all()
        return self.snapshot()

    def snapshot(self) -> dict:
        with self._lock:
            current_order = self._find_order_locked(self._current_order_id)
            active_count = sum(1 for order in self._orders if order.status == "active")
            queued_count = sum(1 for order in self._orders if order.status == "queued")
            connected = self._motion.simulate_only or (time.time() - self._last_joint_state_wall) < 2.5
            return {
                "state": self._state,
                "phase_progress": self._phase_progress_locked(),
                "queue": [order.as_public_dict() for order in self._orders],
                "current_order": current_order.as_public_dict() if current_order else None,
                "served_today": self._served_today,
                "cups_remaining": self._cups_remaining,
                "cups_capacity": self._cups_capacity,
                "uptime_human": self._format_uptime(time.time() - self._boot_time),
                "connected": connected,
                "motion_mode": "simulated" if self._motion.simulate_only else "moveit",
                "fault_message": self._fault_message or None,
                "last_calibration": self._last_calibration,
                "active_count": active_count,
                "queued_count": queued_count,
                "queue_mode": "fault" if self._fault_message else "auto-run",
                "average_cycle_seconds": round(self._average_cycle_seconds, 1),
                "timestamp": datetime.utcnow().isoformat(timespec="seconds") + "Z",
            }

    def serve_frontend(self) -> bytes:
        return self._frontend_path.read_bytes()

    def _queue_worker(self) -> None:
        while not self._shutdown_event.is_set():
            with self._condition:
                while not self._shutdown_event.is_set():
                    if self._fault_message:
                        self._condition.wait(timeout=0.2)
                        continue
                    next_order = next((order for order in self._orders if order.status == "queued"), None)
                    if next_order is None:
                        if self._state != "idle":
                            self._set_phase_locked("idle", 0.0)
                        self._condition.wait(timeout=0.2)
                        continue
                    next_order.status = "active"
                    if next_order.slot is None:
                        next_order.slot = self._next_free_slot_locked()
                    self._current_order_id = next_order.id
                    self._set_phase_locked("received", self.PHASE_DURATIONS["received"])
                    break
                else:
                    return

            try:
                self._sleep_with_abort(self.PHASE_DURATIONS["received"])
                self._set_phase("pouring", self.PHASE_DURATIONS["pouring"])
                self._motion.run_pouring_sequence(self._should_abort)
                self._set_phase("moving", self.PHASE_DURATIONS["moving"])
                self._motion.run_moving_sequence(self._should_abort)
                completed = self._complete_current_order()
                hold_seconds = self._slot_open_seconds if completed and completed.isMe else 2.0
                self._sleep_with_abort(hold_seconds)
                self._release_current_order_slot()
            except MotionExecutionError as exc:
                self.get_logger().error(str(exc))
                with self._condition:
                    self._fault_message = str(exc)
                    self._set_phase_locked("error", 0.0)
            except Exception as exc:  # pragma: no cover
                self.get_logger().error(f"Unexpected queue worker failure: {exc}")
                with self._condition:
                    self._fault_message = str(exc)
                    self._set_phase_locked("error", 0.0)

    def _complete_current_order(self) -> Optional[OrderRecord]:
        with self._condition:
            order = self._find_order_locked(self._current_order_id)
            if order is None:
                return None
            order.status = "done"
            order.completed_at = time.time()
            self._served_today += 1
            self._cups_remaining = max(0, self._cups_remaining - 1)
            elapsed = max(1.0, order.completed_at - order.created_at)
            self._average_cycle_seconds = ((self._average_cycle_seconds * 4.0) + elapsed) / 5.0
            self._set_phase_locked("arrived", self._slot_open_seconds)
            return order

    def _release_current_order_slot(self) -> None:
        with self._condition:
            order = self._find_order_locked(self._current_order_id)
            if order is not None:
                order.slot = None
            self._current_order_id = None
            if not self._fault_message:
                self._set_phase_locked("idle", 0.0)
            self._condition.notify_all()

    def _joint_state_cb(self, _msg: JointState) -> None:
        with self._lock:
            self._last_joint_state_wall = time.time()

    def _resolve_frontend_path(self, configured_path: str) -> Path:
        if configured_path:
            return Path(configured_path).expanduser().resolve()
        package_share = Path(get_package_share_directory("cove_kiosk_bridge"))
        return package_share / "web" / "index.html"

    def _should_abort(self) -> bool:
        with self._lock:
            return self._shutdown_event.is_set() or bool(self._fault_message)

    def _sleep_with_abort(self, duration_sec: float) -> None:
        deadline = time.monotonic() + duration_sec
        while time.monotonic() < deadline:
            if self._should_abort():
                raise MotionExecutionError(self._fault_message or "Execution interrupted.")
            time.sleep(0.05)

    def _next_free_slot_locked(self) -> int:
        taken = {order.slot for order in self._orders if order.slot and order.status != "done"}
        for slot in range(1, 5):
            if slot not in taken:
                return slot
        return 1

    def _find_order_locked(self, order_id: Optional[int]) -> Optional[OrderRecord]:
        if order_id is None:
            return None
        return next((order for order in self._orders if order.id == order_id), None)

    def _set_phase(self, phase: str, duration_sec: float) -> None:
        with self._condition:
            self._set_phase_locked(phase, duration_sec)

    def _set_phase_locked(self, phase: str, duration_sec: float) -> None:
        self._state = phase
        self._phase_started = time.monotonic()
        self._phase_duration = duration_sec

    def _phase_progress_locked(self) -> float:
        if self._phase_duration <= 0.0:
            return 0.0
        elapsed = time.monotonic() - self._phase_started
        return max(0.0, min(1.0, elapsed / self._phase_duration))

    def _create_http_server(self) -> ThreadingHTTPServer:
        node = self

        class Handler(BaseHTTPRequestHandler):
            def log_message(self, fmt: str, *args) -> None:  # pragma: no cover
                node.get_logger().debug(fmt % args)

            def do_GET(self) -> None:
                parsed = urlparse(self.path)
                if parsed.path in ("/", "/index.html"):
                    self._send_bytes(node.serve_frontend(), "text/html; charset=utf-8")
                    return
                if parsed.path == "/api/state":
                    self._send_json(node.snapshot())
                    return
                if parsed.path == "/healthz":
                    self._send_json({"ok": True, "state": node.snapshot()["state"]})
                    return
                self.send_error(HTTPStatus.NOT_FOUND)

            def do_POST(self) -> None:
                parsed = urlparse(self.path)
                body = self._read_json_body()
                try:
                    if parsed.path == "/api/orders":
                        order = node.enqueue_order(str(body.get("name", "")), bool(body.get("isMe", True)))
                        self._send_json({"ok": True, "order": order}, status=HTTPStatus.CREATED)
                        return
                    if parsed.path == "/api/operator/error":
                        self._send_json({"ok": True, "state": node.inject_fault()})
                        return
                    if parsed.path == "/api/operator/reset":
                        self._send_json(
                            {
                                "ok": True,
                                "state": node.reset_fault(clear_queue=bool(body.get("clearQueue", True))),
                            }
                        )
                        return
                except ValueError as exc:
                    self._send_json({"ok": False, "error": str(exc)}, status=HTTPStatus.BAD_REQUEST)
                    return
                self.send_error(HTTPStatus.NOT_FOUND)

            def _read_json_body(self) -> dict:
                length = int(self.headers.get("Content-Length", "0") or 0)
                if length <= 0:
                    return {}
                raw = self.rfile.read(length)
                if not raw:
                    return {}
                try:
                    return json.loads(raw.decode("utf-8"))
                except json.JSONDecodeError:
                    return {}

            def _send_json(self, payload: dict, status: HTTPStatus = HTTPStatus.OK) -> None:
                self._send_bytes(
                    json.dumps(payload).encode("utf-8"),
                    "application/json; charset=utf-8",
                    status,
                )

            def _send_bytes(
                self,
                payload: bytes,
                content_type: str,
                status: HTTPStatus = HTTPStatus.OK,
            ) -> None:
                self.send_response(status)
                self.send_header("Content-Type", content_type)
                self.send_header("Cache-Control", "no-store")
                self.send_header("Content-Length", str(len(payload)))
                self.end_headers()
                self.wfile.write(payload)

        return ThreadingHTTPServer((self._bind_host, self._bind_port), Handler)

    @staticmethod
    def _format_uptime(total_seconds: float) -> str:
        seconds = int(total_seconds)
        days, seconds = divmod(seconds, 86400)
        hours, seconds = divmod(seconds, 3600)
        minutes, _ = divmod(seconds, 60)
        return f"{days}d {hours:02d}:{minutes:02d}"


def main() -> None:
    rclpy.init()
    node = CoveKioskBridge()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
