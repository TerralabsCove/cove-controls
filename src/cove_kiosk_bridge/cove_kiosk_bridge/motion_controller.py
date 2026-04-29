from __future__ import annotations

import json
import os
import shlex
import subprocess
import threading
import time
from collections import deque
from pathlib import Path
from typing import Callable


class MotionExecutionError(RuntimeError):
    """Raised when the arm script fails or is aborted."""


class CoveMotionController:
    """Run the current arm movement script and expose its progress.

    The kiosk bridge intentionally treats arm motion as an external Python
    script boundary. The final motion script can replace the dummy script as
    long as it writes JSON lines or plain status lines to stdout.
    """

    def __init__(self, node, script_command: str, script_cwd: str = "") -> None:
        self._node = node
        self._script_command = script_command
        self._script_cwd = Path(script_cwd).expanduser().resolve() if script_cwd else None
        self._lock = threading.RLock()
        self._process: subprocess.Popen[str] | None = None
        self._last_event: dict = {
            "phase": "idle",
            "step": "idle",
            "message": "Ready for order",
            "progress": 0.0,
            "running": False,
        }
        self._recent_output: deque[str] = deque(maxlen=20)
        self._last_started_at = 0.0
        self._last_finished_at = 0.0
        self._last_exit_code: int | None = None

        self._node.get_logger().info(
            f"COVE motion script command: {self._script_command}"
        )

    @property
    def simulate_only(self) -> bool:
        return False

    def snapshot(self) -> dict:
        with self._lock:
            event = dict(self._last_event)
            event.update(
                {
                    "mode": "script",
                    "command": self._script_command,
                    "running": self._process is not None,
                    "recent_output": list(self._recent_output),
                    "last_started_at": self._last_started_at,
                    "last_finished_at": self._last_finished_at,
                    "last_exit_code": self._last_exit_code,
                }
            )
            return event

    def maybe_cancel(self) -> None:
        with self._lock:
            process = self._process
        if process is None:
            return
        try:
            process.terminate()
            process.wait(timeout=2.0)
        except subprocess.TimeoutExpired:
            process.kill()
            process.wait(timeout=1.0)
        finally:
            with self._lock:
                if self._process is process:
                    self._process = None
                self._last_finished_at = time.time()

    def run_order(
        self,
        order_id: int,
        order_name: str,
        should_abort: Callable[[], bool],
        on_event: Callable[[dict], None],
    ) -> None:
        command = shlex.split(self._script_command)
        if not command:
            raise MotionExecutionError("No arm motion script command configured.")

        env = os.environ.copy()
        env.update(
            {
                "PYTHONUNBUFFERED": "1",
                "COVE_ORDER_ID": str(order_id),
                "COVE_ORDER_NAME": order_name,
            }
        )

        self._publish_event(
            {
                "phase": "received",
                "step": "starting script",
                "message": f"Starting arm script for order #{order_id}",
                "progress": 0.0,
            },
            on_event,
        )

        try:
            process = subprocess.Popen(
                command,
                cwd=str(self._script_cwd) if self._script_cwd else None,
                env=env,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
                bufsize=1,
            )
        except FileNotFoundError as exc:
            raise MotionExecutionError(f"Arm script command not found: {command[0]}") from exc
        except OSError as exc:
            raise MotionExecutionError(f"Failed to start arm script: {exc}") from exc

        with self._lock:
            self._process = process
            self._last_started_at = time.time()
            self._last_finished_at = 0.0
            self._last_exit_code = None

        try:
            assert process.stdout is not None
            while True:
                if should_abort():
                    self.maybe_cancel()
                    raise MotionExecutionError("Arm script interrupted by operator fault.")

                line = process.stdout.readline()
                if line:
                    event = self._event_from_line(line)
                    self._publish_event(event, on_event)
                    continue

                if process.poll() is not None:
                    break
                time.sleep(0.05)

            exit_code = process.wait(timeout=1.0)
            with self._lock:
                self._last_exit_code = exit_code
            if exit_code != 0:
                raise MotionExecutionError(f"Arm script exited with code {exit_code}.")

            self._publish_event(
                {
                    "phase": "arrived",
                    "step": "script complete",
                    "message": "Arm script completed",
                    "progress": 1.0,
                },
                on_event,
            )
        finally:
            with self._lock:
                if self._process is process:
                    self._process = None
                self._last_finished_at = time.time()

    def _event_from_line(self, line: str) -> dict:
        text = line.strip()
        with self._lock:
            self._recent_output.append(text)
        if not text:
            return {"message": "", "step": "", "progress": None}
        try:
            payload = json.loads(text)
        except json.JSONDecodeError:
            return self._plain_text_event(text)
        if not isinstance(payload, dict):
            return self._plain_text_event(text)
        return self._json_event(payload, text)

    def _json_event(self, payload: dict, original_text: str) -> dict:
        event = dict(payload)
        phase = event.get("phase") or event.get("state") or event.get("status")
        step = event.get("step") or event.get("action") or event.get("name")
        message = event.get("message") or event.get("msg") or event.get("status_text")
        progress = event.get("progress")
        if progress is None:
            progress = event.get("percent") or event.get("pct")
            if isinstance(progress, (int, float)) and progress > 1:
                progress = float(progress) / 100.0
        event["phase"] = self._normalize_phase(str(phase or original_text))
        event["step"] = step or phase or original_text
        event["message"] = message or step or phase or original_text
        if progress is not None:
            event["progress"] = progress
        return event

    def _plain_text_event(self, text: str) -> dict:
        event = {"message": text, "step": text}
        phase = self._normalize_phase(text)
        if phase:
            event["phase"] = phase
            event["progress"] = {
                "received": 0.15,
                "pouring": 0.45,
                "moving": 0.75,
                "arrived": 1.0,
            }[phase]
        return event

    @staticmethod
    def _normalize_phase(value: str) -> str | None:
        lowered = value.lower()
        if "fetching" in lowered:
            return "received"
        if "pouring" in lowered:
            return "pouring"
        if "moving" in lowered:
            return "moving"
        if "done" in lowered:
            return "arrived"
        if any(token in lowered for token in ("pre_pick", "pick", "lift", "gripper", "close")):
            return "pouring"
        if any(token in lowered for token in ("transit", "pre_place", "place", "retreat", "open")):
            return "moving"
        if "received" in lowered or "start" in lowered:
            return "received"
        if "arrived" in lowered or "complete" in lowered:
            return "arrived"
        return None

    def _publish_event(self, event: dict, on_event: Callable[[dict], None]) -> None:
        normalized = dict(event)
        phase = normalized.get("phase")
        if phase is not None:
            normalized["phase"] = str(phase)
        if "step" in normalized and normalized["step"] is not None:
            normalized["step"] = str(normalized["step"])
        if "message" in normalized and normalized["message"] is not None:
            normalized["message"] = str(normalized["message"])
        if "progress" in normalized and normalized["progress"] is not None:
            normalized["progress"] = max(0.0, min(1.0, float(normalized["progress"])))

        with self._lock:
            merged = dict(self._last_event)
            merged.update({k: v for k, v in normalized.items() if v is not None})
            merged["running"] = self._process is not None
            self._last_event = merged

        on_event(merged)
