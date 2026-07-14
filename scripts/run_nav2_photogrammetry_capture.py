#!/usr/bin/env python3
from __future__ import annotations

import argparse
import copy
import csv
import json
import math
import threading
import time
from enum import IntEnum
from pathlib import Path
from typing import Any

import numpy as np
import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time as RclpyTime
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformException, TransformListener

from capture_physical_turtlebot_survey import (
    camera_info_to_dict,
    quaternion_to_matrix,
    save_image,
)


class NavOutcome(IntEnum):
    SUCCEEDED = 0
    FAILED = 1
    TIMEOUT = 2
    REJECTED = 3
    SERVER_UNAVAILABLE = 4


def quaternion_from_yaw(yaw: float) -> tuple[float, float, float, float]:
    half = yaw * 0.5
    return 0.0, 0.0, math.sin(half), math.cos(half)


def normalize_yaw(yaw: float) -> float:
    return math.atan2(math.sin(yaw), math.cos(yaw))


def load_waypoints(path: Path) -> list[dict[str, float]]:
    with path.open(newline="") as f:
        rows = list(csv.DictReader(f))
    waypoints = []
    for index, row in enumerate(rows):
        waypoints.append(
            {
                "index": index,
                "x": float(row["x"]),
                "y": float(row["y"]),
                "yaw": normalize_yaw(float(row.get("yaw", 0.0))),
            }
        )
    return waypoints


class Nav2PhotogrammetryRunner(Node):
    def __init__(self, args: argparse.Namespace):
        super().__init__("nav2_photogrammetry_capture")
        self.args = args
        self.waypoints = load_waypoints(args.waypoints_csv.resolve())
        self.images_dir = args.images_dir.resolve()
        self.poses_csv = args.poses_csv.resolve()
        self.transforms_json = args.transforms_json.resolve()
        self.camera_info_json = args.camera_info_json.resolve()
        self.progress_json = args.progress_json.resolve()
        self.images_dir.mkdir(parents=True, exist_ok=True)
        self.poses_csv.parent.mkdir(parents=True, exist_ok=True)
        self.transforms_json.parent.mkdir(parents=True, exist_ok=True)
        self.camera_info_json.parent.mkdir(parents=True, exist_ok=True)
        self.progress_json.parent.mkdir(parents=True, exist_ok=True)

        self.lock = threading.Lock()
        self.latest_image: Image | None = None
        self.latest_image_count = 0
        self.latest_camera_info: CameraInfo | None = None
        self.capture_rows: list[dict[str, Any]] = []
        self.next_capture_index = 0
        self.stop_requested = False

        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=args.qos_depth,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.create_subscription(Image, args.image_topic, self.on_image, qos)
        self.create_subscription(CameraInfo, args.camera_info_topic, self.on_camera_info, qos)

        self.tf_buffer = Buffer(cache_time=Duration(seconds=args.tf_cache_sec))
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.nav_client = ActionClient(self, NavigateToPose, args.navigate_action)
        self.resume_index = self.load_progress()
        self.get_logger().info(f"Loaded {len(self.waypoints)} waypoints from {args.waypoints_csv}")
        self.get_logger().info(f"Resume waypoint index: {self.resume_index}")
        self.get_logger().info(f"Nav2 action: {args.navigate_action}")

    def on_image(self, msg: Image) -> None:
        with self.lock:
            self.latest_image = msg
            self.latest_image_count += 1

    def on_camera_info(self, msg: CameraInfo) -> None:
        with self.lock:
            self.latest_camera_info = msg

    def load_progress(self) -> int:
        if not self.args.resume or not self.progress_json.exists():
            return self.args.start_index
        try:
            data = json.loads(self.progress_json.read_text())
        except json.JSONDecodeError:
            self.get_logger().warn(f"Could not parse progress file, starting from {self.args.start_index}")
            return self.args.start_index
        self.capture_rows = data.get("capture_rows", [])
        self.next_capture_index = int(data.get("next_capture_index", len(self.capture_rows)))
        return max(self.args.start_index, int(data.get("next_waypoint_index", self.args.start_index)))

    def write_progress(self, next_waypoint_index: int) -> None:
        data = {
            "waypoints_csv": str(self.args.waypoints_csv),
            "next_waypoint_index": next_waypoint_index,
            "next_capture_index": self.next_capture_index,
            "capture_rows": self.capture_rows,
            "updated_unix": time.time(),
        }
        self.progress_json.write_text(json.dumps(data, indent=2))

    def wait_for_inputs(self) -> bool:
        deadline = time.time() + self.args.startup_timeout_sec
        while rclpy.ok() and time.time() < deadline:
            with self.lock:
                has_image = self.latest_image is not None
                has_info = self.latest_camera_info is not None
            if has_image and has_info:
                return True
            rclpy.spin_once(self, timeout_sec=0.1)
        return False

    def wait_for_nav_server(self) -> bool:
        deadline = time.time() + self.args.nav_server_timeout_sec
        while rclpy.ok() and time.time() < deadline:
            if self.nav_client.wait_for_server(timeout_sec=1.0):
                return True
            self.get_logger().warn("Waiting for Nav2 NavigateToPose action server...")
        return False

    def navigate_once(self, waypoint: dict[str, float]) -> NavOutcome:
        if not self.wait_for_nav_server():
            return NavOutcome.SERVER_UNAVAILABLE

        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = self.args.target_frame
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = waypoint["x"]
        goal.pose.pose.position.y = waypoint["y"]
        goal.pose.pose.position.z = 0.0
        qx, qy, qz, qw = quaternion_from_yaw(waypoint["yaw"])
        goal.pose.pose.orientation.x = qx
        goal.pose.pose.orientation.y = qy
        goal.pose.pose.orientation.z = qz
        goal.pose.pose.orientation.w = qw

        send_future = self.nav_client.send_goal_async(goal)
        if not self.spin_until_future(send_future, self.args.action_response_timeout_sec):
            return NavOutcome.SERVER_UNAVAILABLE

        goal_handle = send_future.result()
        if goal_handle is None or not goal_handle.accepted:
            return NavOutcome.REJECTED

        result_future = goal_handle.get_result_async()
        start = time.time()
        while rclpy.ok() and not result_future.done():
            if time.time() - start > self.args.nav_timeout_sec:
                self.get_logger().error("Navigation timeout; canceling goal")
                cancel_future = goal_handle.cancel_goal_async()
                self.spin_until_future(cancel_future, 5.0)
                return NavOutcome.TIMEOUT
            rclpy.spin_once(self, timeout_sec=0.1)

        if not result_future.done():
            return NavOutcome.FAILED
        result = result_future.result()
        if result.status == GoalStatus.STATUS_SUCCEEDED:
            return NavOutcome.SUCCEEDED
        self.get_logger().error(f"Navigation failed with goal status {result.status}")
        return NavOutcome.FAILED

    def spin_until_future(self, future, timeout_sec: float) -> bool:
        deadline = time.time() + timeout_sec
        while rclpy.ok() and not future.done() and time.time() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
        return future.done()

    def navigate_with_recovery(self, waypoint_index: int, waypoint: dict[str, float]) -> bool:
        for attempt in range(1, self.args.nav_retries + 1):
            self.get_logger().info(
                f"Navigating waypoint {waypoint_index}/{len(self.waypoints) - 1} "
                f"attempt {attempt}/{self.args.nav_retries}: "
                f"x={waypoint['x']:.3f} y={waypoint['y']:.3f} yaw={waypoint['yaw']:.3f}"
            )
            outcome = self.navigate_once(waypoint)
            if outcome == NavOutcome.SUCCEEDED:
                return True
            self.get_logger().error(f"Navigation outcome: {outcome.name}")
            if attempt < self.args.nav_retries:
                self.get_logger().warn(f"Waiting {self.args.retry_delay_sec:.1f}s before retry")
                self.sleep_spin(self.args.retry_delay_sec)

        if self.args.non_interactive:
            return False
        return self.prompt_recovery(waypoint_index, waypoint)

    def prompt_recovery(self, waypoint_index: int, waypoint: dict[str, float]) -> bool:
        while rclpy.ok():
            print(
                "\nNav2 failed repeatedly. Fix Nav2/localization if needed, then choose:\n"
                "  r + Enter: retry this waypoint\n"
                "  s + Enter: skip this waypoint\n"
                "  q + Enter: stop run\n"
                "> ",
                end="",
                flush=True,
            )
            choice = input().strip().lower()
            if choice == "r":
                return self.navigate_with_recovery(waypoint_index, waypoint)
            if choice == "s":
                self.get_logger().warn(f"Skipping waypoint {waypoint_index}")
                return True
            if choice == "q":
                self.stop_requested = True
                return False

    def sleep_spin(self, seconds: float) -> None:
        deadline = time.time() + seconds
        while rclpy.ok() and time.time() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)

    def capture_at_current_pose(self, waypoint_index: int, waypoint: dict[str, float]) -> None:
        min_image_count = self.latest_image_count
        deadline = time.time() + self.args.capture_timeout_sec
        image_msg = None
        camera_info_msg = None
        while rclpy.ok() and time.time() < deadline:
            with self.lock:
                if self.latest_image is not None and self.latest_image_count > min_image_count:
                    image_msg = copy.deepcopy(self.latest_image)
                    camera_info_msg = copy.deepcopy(self.latest_camera_info)
                    break
            rclpy.spin_once(self, timeout_sec=0.05)

        if image_msg is None or camera_info_msg is None:
            raise RuntimeError("Timed out waiting for fresh image/camera_info")

        stamp = image_msg.header.stamp
        lookup_stamp = RclpyTime.from_msg(stamp) - Duration(seconds=self.args.tf_lookup_offset_sec)
        transform = self.tf_buffer.lookup_transform(
            self.args.target_frame,
            self.args.camera_frame,
            lookup_stamp,
            timeout=Duration(seconds=self.args.tf_timeout_sec),
        )

        image_name = f"{self.next_capture_index:05d}.png"
        image_path = self.images_dir / image_name
        save_image(image_path, image_msg)
        try:
            image_ref = image_path.relative_to(self.poses_csv.parent)
        except ValueError:
            image_ref = image_path

        t = transform.transform.translation
        q = transform.transform.rotation
        row = {
            "image": str(image_ref),
            "waypoint_index": waypoint_index,
            "stamp_sec": stamp.sec,
            "stamp_nanosec": stamp.nanosec,
            "target_frame": self.args.target_frame,
            "camera_frame": self.args.camera_frame,
            "x": t.x,
            "y": t.y,
            "z": t.z,
            "qx": q.x,
            "qy": q.y,
            "qz": q.z,
            "qw": q.w,
            "nav_goal_x": waypoint["x"],
            "nav_goal_y": waypoint["y"],
            "nav_goal_yaw": waypoint["yaw"],
            "image_width": image_msg.width,
            "image_height": image_msg.height,
            "encoding": image_msg.encoding,
        }
        self.capture_rows.append(row)
        self.next_capture_index += 1
        self.write_capture_outputs(camera_info_msg)
        self.get_logger().info(
            f"Captured {image_name} at waypoint {waypoint_index}: "
            f"camera xyz=({t.x:.3f}, {t.y:.3f}, {t.z:.3f})"
        )

    def write_capture_outputs(self, camera_info_msg: CameraInfo) -> None:
        fieldnames = [
            "image",
            "waypoint_index",
            "stamp_sec",
            "stamp_nanosec",
            "target_frame",
            "camera_frame",
            "x",
            "y",
            "z",
            "qx",
            "qy",
            "qz",
            "qw",
            "nav_goal_x",
            "nav_goal_y",
            "nav_goal_yaw",
            "image_width",
            "image_height",
            "encoding",
        ]
        with self.poses_csv.open("w", newline="") as f:
            writer = csv.DictWriter(f, fieldnames=fieldnames)
            writer.writeheader()
            writer.writerows(self.capture_rows)

        frames = []
        for row in self.capture_rows:
            matrix = np.eye(4)
            matrix[:3, :3] = np.array(
                quaternion_to_matrix(
                    float(row["qx"]),
                    float(row["qy"]),
                    float(row["qz"]),
                    float(row["qw"]),
                )
            )
            matrix[:3, 3] = np.array([float(row["x"]), float(row["y"]), float(row["z"])])
            frames.append(
                {
                    "file_path": row["image"],
                    "transform_matrix": matrix.tolist(),
                    "waypoint_index": int(row["waypoint_index"]),
                    "target_frame": row["target_frame"],
                    "camera_frame": row["camera_frame"],
                }
            )
        transforms = {
            "camera_pose_convention": "ros_optical_camera_in_target_frame",
            "target_frame": self.args.target_frame,
            "camera_frame": self.args.camera_frame,
            "frames": frames,
        }
        self.transforms_json.write_text(json.dumps(transforms, indent=2))
        self.camera_info_json.write_text(json.dumps(camera_info_to_dict(camera_info_msg), indent=2))

    def run(self) -> int:
        if not self.wait_for_inputs():
            self.get_logger().error("Timed out waiting for image and camera_info")
            return 1
        if not self.wait_for_nav_server():
            self.get_logger().error("Nav2 action server is unavailable")
            if self.args.non_interactive:
                return 1

        for waypoint_index in range(self.resume_index, len(self.waypoints)):
            waypoint = self.waypoints[waypoint_index]
            self.write_progress(waypoint_index)
            if not self.navigate_with_recovery(waypoint_index, waypoint):
                self.write_progress(waypoint_index)
                return 1
            self.sleep_spin(self.args.settle_sec)
            attempt = 1
            while attempt <= self.args.capture_retries:
                try:
                    self.capture_at_current_pose(waypoint_index, waypoint)
                    self.write_progress(waypoint_index + 1)
                    break
                except TransformException as exc:
                    self.get_logger().error(f"Capture TF failed attempt {attempt}: {exc}")
                except Exception as exc:
                    self.get_logger().error(f"Capture failed attempt {attempt}: {exc}")
                if attempt >= self.args.capture_retries:
                    if self.args.non_interactive:
                        return 1
                    print("Capture failed repeatedly. r=retry, s=skip, q=quit")
                    choice = input("> ").strip().lower()
                    if choice == "r":
                        attempt = 1
                        continue
                    if choice == "q":
                        return 1
                    if choice == "s":
                        break
                self.sleep_spin(self.args.retry_delay_sec)
                attempt += 1
        self.get_logger().info("Photogrammetry run complete")
        return 0


def parse_args() -> tuple[argparse.Namespace, list[str]]:
    parser = argparse.ArgumentParser(description="Navigate a Nav2 waypoint path and capture OAK-D images with TF poses.")
    parser.add_argument("--waypoints-csv", type=Path, required=True)
    parser.add_argument("--navigate-action", default="/robot_2/navigate_to_pose")
    parser.add_argument("--target-frame", default="map")
    parser.add_argument("--camera-frame", default="oakd_rgb_camera_optical_frame")
    parser.add_argument("--image-topic", default="/robot_2/oakd/rgb/image_raw")
    parser.add_argument("--camera-info-topic", default="/robot_2/oakd/rgb/camera_info")
    parser.add_argument("--images-dir", type=Path, required=True)
    parser.add_argument("--poses-csv", type=Path, required=True)
    parser.add_argument("--transforms-json", type=Path, required=True)
    parser.add_argument("--camera-info-json", type=Path, required=True)
    parser.add_argument("--progress-json", type=Path, required=True)
    parser.add_argument("--start-index", type=int, default=0)
    parser.add_argument("--resume", action="store_true", default=True)
    parser.add_argument("--no-resume", dest="resume", action="store_false")
    parser.add_argument("--settle-sec", type=float, default=1.0)
    parser.add_argument("--nav-timeout-sec", type=float, default=120.0)
    parser.add_argument("--nav-retries", type=int, default=3)
    parser.add_argument("--capture-retries", type=int, default=3)
    parser.add_argument("--retry-delay-sec", type=float, default=3.0)
    parser.add_argument("--startup-timeout-sec", type=float, default=30.0)
    parser.add_argument("--nav-server-timeout-sec", type=float, default=60.0)
    parser.add_argument("--action-response-timeout-sec", type=float, default=10.0)
    parser.add_argument("--capture-timeout-sec", type=float, default=10.0)
    parser.add_argument("--tf-cache-sec", type=float, default=30.0)
    parser.add_argument("--tf-timeout-sec", type=float, default=5.0)
    parser.add_argument("--tf-lookup-offset-sec", type=float, default=0.02)
    parser.add_argument("--qos-depth", type=int, default=5)
    parser.add_argument("--non-interactive", action="store_true")
    return parser.parse_known_args()


def main() -> int:
    args, ros_args = parse_args()
    rclpy.init(args=ros_args)
    node = Nav2PhotogrammetryRunner(args)
    try:
        return node.run()
    except KeyboardInterrupt:
        return 130
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
