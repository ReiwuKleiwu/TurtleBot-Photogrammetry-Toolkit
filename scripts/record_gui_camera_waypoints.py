#!/usr/bin/env python3
import argparse
import json
import math
import re
import subprocess
import sys
import threading
import time
from pathlib import Path



KEY_EVENT_RE = re.compile(r"EVENT type (\d+) \((RawKeyPress|RawKeyRelease)\)")
DETAIL_RE = re.compile(r"detail:\s+(\d+)")


def normalize_key_name(key: str) -> str:
    key = key.strip().lower()
    aliases = {
        "=": "equal",
        "+": "plus",
        "esc": "escape",
        "return": "enter",
    }
    return aliases.get(key, key)


def load_x11_keymap() -> dict[int, str]:
    result = subprocess.run(
        ["xmodmap", "-pke"],
        check=True,
        capture_output=True,
        text=True,
    )
    keymap: dict[int, str] = {}
    for line in result.stdout.splitlines():
        line = line.strip()
        if not line.startswith("keycode"):
            continue
        parts = line.split()
        if len(parts) < 4:
            continue
        try:
            keycode = int(parts[1])
        except ValueError:
            continue
        keymap[keycode] = parts[3].lower()
    return keymap


def start_global_key_listener(callback, stop_event: threading.Event) -> threading.Thread:
    keymap = load_x11_keymap()

    def worker() -> None:
        process = subprocess.Popen(
            ["xinput", "test-xi2", "--root"],
            stdout=subprocess.PIPE,
            stderr=subprocess.DEVNULL,
            text=True,
            bufsize=1,
        )
        if process.stdout is None:
            raise RuntimeError("Failed to subscribe to X11 keyboard events")

        pending_event: str | None = None
        try:
            for raw_line in process.stdout:
                if stop_event.is_set():
                    break
                line = raw_line.strip()
                event_match = KEY_EVENT_RE.search(line)
                if event_match:
                    pending_event = event_match.group(2)
                    continue
                if pending_event is None:
                    continue
                detail_match = DETAIL_RE.search(line)
                if not detail_match:
                    continue
                keycode = int(detail_match.group(1))
                key = keymap.get(keycode)
                if key and pending_event == "RawKeyPress":
                    callback(key)
                pending_event = None
        finally:
            process.terminate()
            try:
                process.wait(timeout=1.0)
            except subprocess.TimeoutExpired:
                process.kill()

    thread = threading.Thread(target=worker, daemon=True)
    thread.start()
    return thread


def append_current_pose(points: list[dict[str, float]]) -> None:
    pose = read_gui_camera_pose()
    points.append(
        {
            "x": round(pose["x"], 5),
            "y": round(pose["y"], 5),
            "z": round(pose["z"], 5),
            "qx": round(pose["qx"], 8),
            "qy": round(pose["qy"], 8),
            "qz": round(pose["qz"], 8),
            "qw": round(pose["qw"], 8),
            "roll": round(pose["roll"], 8),
            "pitch": round(pose["pitch"], 8),
            "yaw": round(pose["yaw"], 8),
        }
    )
    print(
        f"Saved waypoint {len(points)}: "
        f"x={pose['x']:.3f} y={pose['y']:.3f} z={pose['z']:.3f} "
        f"pitch={pose['pitch']:.3f} yaw={pose['yaw']:.3f}",
        flush=True,
    )

def quaternion_to_euler(x: float, y: float, z: float, w: float) -> tuple[float, float, float]:
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2.0 * (w * y - z * x)
    if abs(sinp) >= 1.0:
        pitch = math.copysign(math.pi / 2.0, sinp)
    else:
        pitch = math.asin(sinp)

    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return roll, pitch, yaw


def read_gui_camera_pose() -> dict[str, float]:
    result = subprocess.run(
        ["gz", "topic", "-e", "-n", "1", "-t", "/gui/camera/pose"],
        check=True,
        capture_output=True,
        text=True,
    )
    text = result.stdout

    def parse_block(block_name: str) -> dict[str, float]:
        values = {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0}
        in_block = False
        for line in text.splitlines():
            line = line.strip()
            if line.startswith(f"{block_name} {{"):
                in_block = True
                continue
            if in_block and line == "}":
                break
            if in_block:
                for key in values:
                    if line.startswith(f"{key}:"):
                        values[key] = float(line.split(":", 1)[1].strip())
        return values

    pos = parse_block("position")
    ori = parse_block("orientation")
    roll, pitch, yaw = quaternion_to_euler(ori["x"], ori["y"], ori["z"], ori["w"])
    return {
        "x": pos["x"],
        "y": pos["y"],
        "z": pos["z"],
        "qx": ori["x"],
        "qy": ori["y"],
        "qz": ori["z"],
        "qw": ori["w"],
        "roll": roll,
        "pitch": pitch,
        "yaw": yaw,
    }


def main() -> int:
    parser = argparse.ArgumentParser(description="Record Gazebo GUI camera poses as waypoints.")
    parser.add_argument("--output", type=Path, required=True, help="Path to write recorded GUI camera waypoints JSON")
    parser.add_argument("--capture-key", default="c", help="Global X11 key that saves the current GUI camera pose")
    parser.add_argument("--quit-key", default="=", help="Global X11 key that finishes recording and writes the JSON file")
    parser.add_argument("--terminal", action="store_true", help="Use terminal Enter/q input instead of global X11 hotkeys")
    args = parser.parse_args()

    output = args.output.resolve()
    output.parent.mkdir(parents=True, exist_ok=True)

    points: list[dict[str, float]] = []
    print("Move the Gazebo GUI camera to a viewpoint you want to save.")

    if args.terminal:
        print("Press Enter in this terminal to save the current GUI camera pose, or type `q` then Enter to finish.")
        while True:
            user_input = input("> ").strip().lower()
            if user_input == "q":
                break
            try:
                append_current_pose(points)
            except Exception as exc:
                print(f"Failed to read GUI camera pose: {exc}", file=sys.stderr)
                continue
    else:
        capture_key = normalize_key_name(args.capture_key)
        quit_key = normalize_key_name(args.quit_key)
        stop_event = threading.Event()
        queue: list[str] = []
        lock = threading.Lock()

        def on_key(key: str) -> None:
            with lock:
                queue.append(normalize_key_name(key))

        print(f"Keep Gazebo focused. Press `{args.capture_key}` to save a pose, `{args.quit_key}` to finish.")
        listener = start_global_key_listener(on_key, stop_event)
        try:
            while True:
                with lock:
                    key = queue.pop(0) if queue else None
                if key is None:
                    time.sleep(0.05)
                    continue
                if key == quit_key:
                    break
                if key != capture_key:
                    continue
                try:
                    append_current_pose(points)
                except Exception as exc:
                    print(f"Failed to read GUI camera pose: {exc}", file=sys.stderr)
        finally:
            stop_event.set()
            listener.join(timeout=1.0)

    if len(points) < 2:
        print("Need at least 2 waypoints. Nothing written.", file=sys.stderr)
        return 1

    output.write_text(json.dumps(points, indent=2))
    print(f"Wrote {len(points)} GUI camera waypoints to {output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
