#!/usr/bin/env python3
import argparse
import sys
import time

import gi

gi.require_version("Atspi", "2.0")
from gi.repository import Atspi


def safe_call(fn, default=None):
    try:
        return fn()
    except Exception:
        return default


def accessible_name(obj):
    return safe_call(obj.get_name, "") or ""


def role_name(obj):
    return safe_call(obj.get_role_name, "") or ""


def child_count(obj):
    return safe_call(obj.get_child_count, 0) or 0


def child_at(obj, index):
    return safe_call(lambda: obj.get_child_at_index(index))


def has_action(obj):
    return safe_call(obj.get_n_actions, 0) or 0


def do_first_action(obj):
    return safe_call(lambda: obj.do_action(0), False)


def walk(root, max_nodes=12000):
    stack = [root]
    seen = 0
    while stack and seen < max_nodes:
        obj = stack.pop()
        seen += 1
        yield obj

        count = child_count(obj)
        for index in range(count - 1, -1, -1):
            child = child_at(obj, index)
            if child is not None:
                stack.append(child)


def find_in_rviz(name, role_substring=None):
    desktop = Atspi.get_desktop(0)
    for app_index in range(child_count(desktop)):
        app = child_at(desktop, app_index)
        if app is None:
            continue
        app_name = accessible_name(app).lower()
        if "rviz" not in app_name:
            continue

        for obj in walk(app):
            if accessible_name(obj) != name:
                continue
            if role_substring and role_substring.lower() not in role_name(obj).lower():
                continue
            return obj
    return None


def main():
    parser = argparse.ArgumentParser(
        description="Connect MoveIt RViz Stored States to the warehouse database."
    )
    parser.add_argument("--timeout", type=float, default=90.0)
    parser.add_argument("--poll-period", type=float, default=0.5)
    args = parser.parse_args()

    Atspi.init()
    deadline = time.monotonic() + args.timeout
    context_clicked = False

    while time.monotonic() < deadline:
        if find_in_rviz("Disconnect", "push button"):
            print("MoveIt RViz warehouse is already connected")
            return 0

        connect_button = find_in_rviz("Connect", "push button")
        if connect_button is not None and has_action(connect_button):
            if do_first_action(connect_button):
                time.sleep(1.0)
                if find_in_rviz("Disconnect", "push button"):
                    print("Connected MoveIt RViz warehouse")
                    return 0

        if not context_clicked:
            context_tab = find_in_rviz("Context")
            if context_tab is not None and has_action(context_tab):
                do_first_action(context_tab)
                context_clicked = True

        time.sleep(args.poll_period)

    print("Timed out waiting for the MoveIt RViz warehouse Connect button", file=sys.stderr)
    return 1


if __name__ == "__main__":
    raise SystemExit(main())
