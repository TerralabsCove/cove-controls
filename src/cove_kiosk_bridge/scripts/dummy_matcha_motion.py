#!/usr/bin/env python3
from __future__ import annotations

import json
import os
import sys
import time


STEPS = [
    ("received", "validate order", "Order accepted by arm script", 0.05, 0.7),
    ("pouring", "move home", "Moving arm to home pose", 0.12, 0.8),
    ("pouring", "move pre_pick", "Moving arm above cup pickup", 0.24, 0.9),
    ("pouring", "move pick", "Lowering arm to pickup pose", 0.36, 1.0),
    ("pouring", "close gripper", "Simulated gripper close", 0.48, 0.7),
    ("pouring", "move lift", "Lifting cup", 0.58, 0.9),
    ("moving", "move transit", "Carrying order toward pickup", 0.68, 1.0),
    ("moving", "move pre_place", "Approaching pickup slot", 0.78, 0.9),
    ("moving", "move place", "Placing order in pickup slot", 0.88, 1.0),
    ("moving", "open gripper", "Simulated gripper release", 0.94, 0.6),
    ("moving", "return home", "Returning arm to home pose", 0.98, 0.8),
    ("arrived", "complete", "Order is ready for pickup", 1.0, 0.2),
]


def emit(phase: str, step: str, message: str, progress: float) -> None:
    print(
        json.dumps(
            {
                "phase": phase,
                "step": step,
                "message": message,
                "progress": progress,
            }
        ),
        flush=True,
    )


def main() -> int:
    order_id = os.environ.get("COVE_ORDER_ID", "unknown")
    order_name = os.environ.get("COVE_ORDER_NAME", "guest")
    emit(
        "received",
        "script start",
        f"Dummy arm script started for order #{order_id} ({order_name})",
        0.0,
    )
    for phase, step, message, progress, delay in STEPS:
        emit(phase, step, message, progress)
        time.sleep(delay)
    return 0


if __name__ == "__main__":
    sys.exit(main())
