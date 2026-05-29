#!/usr/bin/env python3
import argparse
import csv
import json
import math
import shutil
import subprocess
import time
from pathlib import Path

import numpy as np


def quaternion_from_rpy(roll: float, pitch: float, yaw: float) -> tuple[float, float, float, float]:
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    qw = cr * cp * cy + sr * sp * sy
    qx = sr * cp * cy - cr * sp * sy
    qy = cr * sp * cy + sr * cp * sy
    qz = cr * cp * sy - sr * sp * cy
    return qx, qy, qz, qw


def rotation_matrix_from_rpy(roll: float, pitch: float, yaw: float) -> np.ndarray:
    cx, sx = math.cos(roll), math.sin(roll)
    cy, sy = math.cos(pitch), math.sin(pitch)
    cz, sz = math.cos(yaw), math.sin(yaw)
    rx = np.array([[1, 0, 0], [0, cx, -sx], [0, sx, cx]])
    ry = np.array([[cy, 0, sy], [0, 1, 0], [-sy, 0, cy]])
    rz = np.array([[cz, -sz, 0], [sz, cz, 0], [0, 0, 1]])
    return rz @ ry @ rx


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


def normalize_waypoint(waypoint: dict) -> dict[str, float]:
    if all(key in waypoint for key in ("qx", "qy", "qz", "qw")):
        qx = float(waypoint["qx"])
        qy = float(waypoint["qy"])
        qz = float(waypoint["qz"])
        qw = float(waypoint["qw"])
        if all(key in waypoint for key in ("roll", "pitch", "yaw")):
            roll = float(waypoint["roll"])
            pitch = float(waypoint["pitch"])
            yaw = float(waypoint["yaw"])
        else:
            roll, pitch, yaw = quaternion_to_euler(qx, qy, qz, qw)
        return {
            "x": float(waypoint["x"]),
            "y": float(waypoint["y"]),
            "z": float(waypoint["z"]),
            "qx": qx,
            "qy": qy,
            "qz": qz,
            "qw": qw,
            "roll": roll,
            "pitch": pitch,
            "yaw": yaw,
        }

    if "roll" in waypoint and "pitch" in waypoint and "yaw" in waypoint:
        qx, qy, qz, qw = quaternion_from_rpy(
            float(waypoint["roll"]),
            float(waypoint["pitch"]),
            float(waypoint["yaw"]),
        )
        return {
            "x": float(waypoint["x"]),
            "y": float(waypoint["y"]),
            "z": float(waypoint["z"]),
            "qx": qx,
            "qy": qy,
            "qz": qz,
            "qw": qw,
            "roll": float(waypoint["roll"]),
            "pitch": float(waypoint["pitch"]),
            "yaw": float(waypoint["yaw"]),
        }

    # Compatibility with old survey_camera waypoints.
    qx, qy, qz, qw = quaternion_from_rpy(0.0, -0.12, float(waypoint["yaw"]))
    return {
        "x": float(waypoint["x"]),
        "y": float(waypoint["y"]),
        "z": float(waypoint.get("z", 1.05)),
        "qx": qx,
        "qy": qy,
        "qz": qz,
        "qw": qw,
        "roll": 0.0,
        "pitch": -0.12,
        "yaw": float(waypoint["yaw"]),
    }


def parse_pose_text(text: str) -> dict[str, float]:
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


def read_gui_camera_pose(timeout_sec: float = 0.5) -> dict[str, float] | None:
    try:
        result = subprocess.run(
            ["gz", "topic", "-e", "-n", "1", "-t", "/gui/camera/pose"],
            check=True,
            capture_output=True,
            text=True,
            timeout=timeout_sec,
        )
    except (subprocess.CalledProcessError, subprocess.TimeoutExpired):
        return None
    return parse_pose_text(result.stdout)


def quaternion_dot(a: dict[str, float], b: dict[str, float]) -> float:
    return abs(a["qx"] * b["qx"] + a["qy"] * b["qy"] + a["qz"] * b["qz"] + a["qw"] * b["qw"])


def pose_reached(current: dict[str, float], target: dict[str, float], position_tolerance: float, rotation_tolerance: float) -> bool:
    dx = current["x"] - target["x"]
    dy = current["y"] - target["y"]
    dz = current["z"] - target["z"]
    position_error = math.sqrt(dx * dx + dy * dy + dz * dz)
    dot = min(1.0, max(-1.0, quaternion_dot(current, target)))
    rotation_error = 2.0 * math.acos(dot)
    return position_error <= position_tolerance and rotation_error <= rotation_tolerance


def wait_for_gui_camera_pose(target: dict[str, float], timeout_sec: float, position_tolerance: float, rotation_tolerance: float) -> bool:
    deadline = time.time() + timeout_sec
    while time.time() < deadline:
        current = read_gui_camera_pose(timeout_sec=0.5)
        if current is not None and pose_reached(current, target, position_tolerance, rotation_tolerance):
            return True
    return False


def move_gui_camera(x: float, y: float, z: float, qx: float, qy: float, qz: float, qw: float) -> None:
    req = (
        "pose: {"
        f"position: {{x: {x:.6f}, y: {y:.6f}, z: {z:.6f}}} "
        f"orientation: {{x: {qx:.8f}, y: {qy:.8f}, z: {qz:.8f}, w: {qw:.8f}}}"
        "}"
    )
    subprocess.run(
        [
            "gz",
            "service",
            "-s",
            "/gui/move_to/pose",
            "--reqtype",
            "gz.msgs.GUICamera",
            "--reptype",
            "gz.msgs.Boolean",
            "--timeout",
            "2000",
            "--req",
            req,
        ],
        check=True,
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
    )


def take_screenshot(path: Path, search_dirs: list[Path], timeout_sec: float) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    before = {candidate.resolve() for candidate in path.parent.rglob("*.png")}
    requested = str(path.parent)
    requested_name = path.name
    subprocess.run(
        [
            "gz",
            "service",
            "-s",
            "/gui/screenshot",
            "--reqtype",
            "gz.msgs.StringMsg",
            "--reptype",
            "gz.msgs.Boolean",
            "--timeout",
            "4000",
            "--req",
            f'data: "{requested}"',
        ],
        check=True,
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
    )
    deadline = time.time() + timeout_sec
    while time.time() < deadline:
        if path.exists():
            return

        # Gazebo's screenshot service sometimes treats the requested name as a
        # directory-like prefix and writes an auto-named PNG underneath it.
        weird_dir = path
        if weird_dir.is_dir():
            nested_pngs = sorted(weird_dir.rglob("*.png"), key=lambda p: p.stat().st_mtime)
            if nested_pngs:
                shutil.move(str(nested_pngs[-1]), str(path.with_suffix(".tmp.png")))
                shutil.rmtree(weird_dir, ignore_errors=True)
                path.with_suffix(".tmp.png").replace(path)
                return

        after = {candidate.resolve() for candidate in path.parent.rglob("*.png")}
        new_files = sorted(after - before, key=lambda p: p.stat().st_mtime)
        if new_files:
            newest = new_files[-1]
            if newest != path.resolve():
                newest.replace(path)
            return

        candidates = []
        for directory in search_dirs:
            if not directory.exists():
                continue
            try:
                for candidate in directory.rglob(requested_name):
                    if candidate.is_file():
                        candidates.append(candidate)
            except PermissionError:
                continue
        candidates.sort(key=lambda p: p.stat().st_mtime)
        if candidates:
            candidates[-1].replace(path)
            return

        time.sleep(0.1)

    searched = ", ".join(str(directory) for directory in [path.parent, *search_dirs])
    raise RuntimeError(
        f"Gazebo screenshot service did not create {path}. "
        f"Searched: {searched}. If Gazebo writes screenshots elsewhere, rerun with "
        f"--screenshot-search-dir PATH for that directory."
    )


def main() -> int:
    parser = argparse.ArgumentParser(description="Replay Gazebo GUI camera waypoints and save screenshots.")
    parser.add_argument("--waypoints", type=Path, required=True, help="Input GUI camera waypoint JSON")
    parser.add_argument("--images-dir", type=Path, required=True, help="Directory where PNG screenshots are written")
    parser.add_argument("--poses-csv", type=Path, required=True, help="Path to write pose metadata CSV")
    parser.add_argument("--transforms-json", type=Path, required=True, help="Path to write transform metadata JSON")
    parser.add_argument("--screenshot-search-dir", type=Path, action="append", default=[], help="Extra directory to search if Gazebo writes the screenshot somewhere unexpected")
    parser.add_argument("--screenshot-timeout-sec", type=float, default=5.0)
    parser.add_argument("--settle-sec", type=float, default=0.4, help="Extra delay after the camera reaches a waypoint")
    parser.add_argument("--move-timeout-sec", type=float, default=3.0, help="Seconds to wait for /gui/camera/pose to reach the requested waypoint")
    parser.add_argument("--position-tolerance", type=float, default=0.03, help="Allowed camera position error in meters")
    parser.add_argument("--rotation-tolerance", type=float, default=0.03, help="Allowed camera rotation error in radians")
    args = parser.parse_args()

    images_dir = args.images_dir.resolve()
    poses_csv = args.poses_csv.resolve()
    transforms_json = args.transforms_json.resolve()
    search_dirs = [path.resolve() for path in args.screenshot_search_dir]
    images_dir.mkdir(parents=True, exist_ok=True)
    poses_csv.parent.mkdir(parents=True, exist_ok=True)
    transforms_json.parent.mkdir(parents=True, exist_ok=True)

    raw_waypoints = json.loads(args.waypoints.read_text())
    waypoints = [normalize_waypoint(waypoint) for waypoint in raw_waypoints]
    with poses_csv.open("w", newline="") as csv_file:
        writer = csv.writer(csv_file)
        writer.writerow(["image", "x", "y", "z", "roll", "pitch", "yaw", "qx", "qy", "qz", "qw"])

        for frame_index, waypoint in enumerate(waypoints):
            x = waypoint["x"]
            y = waypoint["y"]
            z = waypoint["z"]
            roll = waypoint["roll"]
            pitch = waypoint["pitch"]
            yaw = waypoint["yaw"]
            qx = waypoint["qx"]
            qy = waypoint["qy"]
            qz = waypoint["qz"]
            qw = waypoint["qw"]

            move_gui_camera(x, y, z, qx, qy, qz, qw)
            reached = wait_for_gui_camera_pose(
                waypoint,
                args.move_timeout_sec,
                args.position_tolerance,
                args.rotation_tolerance,
            )
            if not reached:
                print(
                    f"Warning: camera pose was not confirmed for waypoint {frame_index + 1}; "
                    "capturing after fallback settle delay"
                )
            time.sleep(args.settle_sec)

            image_name = f"{frame_index:05d}.png"
            image_path = images_dir / image_name
            take_screenshot(image_path, search_dirs, args.screenshot_timeout_sec)
            try:
                image_ref = image_path.relative_to(poses_csv.parent)
            except ValueError:
                image_ref = image_path
            writer.writerow([str(image_ref), x, y, z, roll, pitch, yaw, qx, qy, qz, qw])
            print(f"[{frame_index + 1}/{len(waypoints)}] -> {image_name}")

    transforms = {"camera_pose_convention": "gui_camera_in_world", "frames": []}
    with poses_csv.open() as csv_file:
        reader = csv.DictReader(csv_file)
        for row in reader:
            x = float(row["x"])
            y = float(row["y"])
            z = float(row["z"])
            roll = float(row["roll"])
            pitch = float(row["pitch"])
            yaw = float(row["yaw"])
            matrix = np.eye(4)
            matrix[:3, :3] = rotation_matrix_from_rpy(roll, pitch, yaw)
            matrix[:3, 3] = np.array([x, y, z])
            transforms["frames"].append(
                {
                    "file_path": row["image"],
                    "transform_matrix": matrix.tolist(),
                }
            )
    transforms_json.write_text(json.dumps(transforms, indent=2))
    print(f"Saved GUI-camera images to {images_dir}")
    print(f"Saved pose CSV to {poses_csv}")
    print(f"Saved transforms JSON to {transforms_json}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
