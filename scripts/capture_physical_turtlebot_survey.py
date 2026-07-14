#!/usr/bin/env python3
from __future__ import annotations

import argparse
import copy
import csv
import json
import math
import queue
import threading
from pathlib import Path
from typing import Any

import numpy as np
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time as RclpyTime
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformException, TransformListener


IMAGE_WRITER_NAME: str | None = None
IMAGE_WRITER: Any | None = None


def get_image_writer() -> tuple[str, Any]:
    global IMAGE_WRITER_NAME, IMAGE_WRITER
    if IMAGE_WRITER_NAME is not None:
        return IMAGE_WRITER_NAME, IMAGE_WRITER

    try:
        import cv2

        IMAGE_WRITER_NAME, IMAGE_WRITER = "cv2", cv2
        return IMAGE_WRITER_NAME, IMAGE_WRITER
    except ImportError:
        pass

    try:
        from PIL import Image as PilImage

        IMAGE_WRITER_NAME, IMAGE_WRITER = "pil", PilImage
        return IMAGE_WRITER_NAME, IMAGE_WRITER
    except ImportError as exc:
        raise RuntimeError("Install python3-opencv or python3-pil to save raw ROS images") from exc


def quaternion_to_matrix(x: float, y: float, z: float, w: float) -> list[list[float]]:
    xx, yy, zz = x * x, y * y, z * z
    xy, xz, yz = x * y, x * z, y * z
    wx, wy, wz = w * x, w * y, w * z
    return [
        [1.0 - 2.0 * (yy + zz), 2.0 * (xy - wz), 2.0 * (xz + wy)],
        [2.0 * (xy + wz), 1.0 - 2.0 * (xx + zz), 2.0 * (yz - wx)],
        [2.0 * (xz - wy), 2.0 * (yz + wx), 1.0 - 2.0 * (xx + yy)],
    ]


def matrix_to_euler_xyz(r: list[list[float]]) -> tuple[float, float, float]:
    sy = -r[2][0]
    if abs(sy) < 1.0:
        pitch = math.asin(sy)
        roll = math.atan2(r[2][1], r[2][2])
        yaw = math.atan2(r[1][0], r[0][0])
    else:
        pitch = math.copysign(math.pi / 2.0, sy)
        roll = math.atan2(-r[0][1], r[1][1])
        yaw = 0.0
    return roll, pitch, yaw


def image_to_array(msg: Image) -> np.ndarray:
    dtype_by_encoding = {
        "rgb8": np.uint8,
        "bgr8": np.uint8,
        "rgba8": np.uint8,
        "bgra8": np.uint8,
        "mono8": np.uint8,
        "8UC1": np.uint8,
        "16UC1": np.uint16,
        "mono16": np.uint16,
    }
    channels_by_encoding = {
        "rgb8": 3,
        "bgr8": 3,
        "rgba8": 4,
        "bgra8": 4,
        "mono8": 1,
        "8UC1": 1,
        "16UC1": 1,
        "mono16": 1,
    }
    encoding = msg.encoding.lower()
    if encoding not in dtype_by_encoding:
        raise ValueError(f"Unsupported image encoding '{msg.encoding}'")

    dtype = dtype_by_encoding[encoding]
    channels = channels_by_encoding[encoding]
    data = np.frombuffer(msg.data, dtype=dtype)
    bytes_per_pixel = np.dtype(dtype).itemsize * channels
    row_pixels = msg.step // bytes_per_pixel
    if row_pixels < msg.width:
        raise ValueError(f"Image step {msg.step} is too small for width {msg.width} and encoding {msg.encoding}")

    if channels == 1:
        array = data.reshape((msg.height, row_pixels))[:, : msg.width]
    else:
        array = data.reshape((msg.height, row_pixels, channels))[:, : msg.width, :]
    if msg.is_bigendian and np.dtype(dtype).itemsize > 1:
        array = array.byteswap()
    return np.ascontiguousarray(array)


def save_image(path: Path, msg: Image) -> None:
    image_writer_name, image_writer = get_image_writer()
    array = image_to_array(msg)
    encoding = msg.encoding.lower()

    if image_writer_name == "cv2":
        cv2 = image_writer
        if encoding == "rgb8":
            output = cv2.cvtColor(array, cv2.COLOR_RGB2BGR)
        elif encoding == "rgba8":
            output = cv2.cvtColor(array, cv2.COLOR_RGBA2BGRA)
        else:
            output = array
        if not cv2.imwrite(str(path), output):
            raise RuntimeError(f"Failed to write image: {path}")
        return

    pil_image = image_writer
    if encoding == "bgr8":
        array = array[:, :, ::-1]
        mode = "RGB"
    elif encoding == "rgb8":
        mode = "RGB"
    elif encoding == "rgba8":
        mode = "RGBA"
    elif encoding == "bgra8":
        array = array[:, :, [2, 1, 0, 3]]
        mode = "RGBA"
    elif encoding in {"mono8", "8uc1"}:
        mode = "L"
    elif encoding in {"mono16", "16uc1"}:
        mode = "I;16"
    else:
        raise ValueError(f"Unsupported image encoding for Pillow: {msg.encoding}")
    pil_image.fromarray(array, mode=mode).save(path)


def camera_info_to_dict(msg: CameraInfo) -> dict[str, Any]:
    return {
        "header": {
            "stamp": {"sec": msg.header.stamp.sec, "nanosec": msg.header.stamp.nanosec},
            "frame_id": msg.header.frame_id,
        },
        "height": msg.height,
        "width": msg.width,
        "distortion_model": msg.distortion_model,
        "d": list(msg.d),
        "k": list(msg.k),
        "r": list(msg.r),
        "p": list(msg.p),
        "binning_x": msg.binning_x,
        "binning_y": msg.binning_y,
        "roi": {
            "x_offset": msg.roi.x_offset,
            "y_offset": msg.roi.y_offset,
            "height": msg.roi.height,
            "width": msg.roi.width,
            "do_rectify": msg.roi.do_rectify,
        },
    }


class PhysicalSurveyRecorder(Node):
    def __init__(self, args: argparse.Namespace):
        super().__init__("physical_turtlebot_survey_recorder")
        self.args = args
        self.images_dir = args.images_dir.resolve()
        self.poses_csv = args.poses_csv.resolve()
        self.transforms_json = args.transforms_json.resolve()
        self.camera_info_json = args.camera_info_json.resolve()

        self.images_dir.mkdir(parents=True, exist_ok=True)
        self.poses_csv.parent.mkdir(parents=True, exist_ok=True)
        self.transforms_json.parent.mkdir(parents=True, exist_ok=True)
        self.camera_info_json.parent.mkdir(parents=True, exist_ok=True)

        self.lock = threading.Lock()
        self.latest_image: Image | None = None
        self.latest_image_count = 0
        self.latest_camera_info: CameraInfo | None = None
        self.capture_queue: queue.Queue[str] = queue.Queue()
        self.pending_capture: tuple[str, int] | None = None
        self.stop_event = threading.Event()
        self.frame_index = args.start_index
        self.rows: list[dict[str, Any]] = []

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
        self.create_timer(0.05, self.process_capture_requests)
        self.get_logger().info(f"Image topic: {args.image_topic}")
        self.get_logger().info(f"CameraInfo topic: {args.camera_info_topic}")
        self.get_logger().info(f"TF target/source: {args.target_frame} -> {args.camera_frame}")
        image_writer_name, _ = get_image_writer()
        self.get_logger().info(f"Image writer: {image_writer_name}")

    def on_image(self, msg: Image) -> None:
        with self.lock:
            self.latest_image = msg
            self.latest_image_count += 1

    def on_camera_info(self, msg: CameraInfo) -> None:
        with self.lock:
            self.latest_camera_info = msg

    def request_capture(self, label: str = "") -> None:
        self.capture_queue.put(label)

    def process_capture_requests(self) -> None:
        if self.pending_capture is not None:
            label, min_image_count = self.pending_capture
            with self.lock:
                has_fresh_image = self.latest_image is not None and self.latest_image_count > min_image_count
            if has_fresh_image:
                self.pending_capture = None
                try:
                    self.capture(label)
                except Exception as exc:
                    self.get_logger().error(f"Capture failed: {exc}")
            return

        while True:
            try:
                label = self.capture_queue.get_nowait()
            except queue.Empty:
                return
            with self.lock:
                self.pending_capture = (label, self.latest_image_count)
            self.get_logger().info("Capture requested; waiting for next image frame")
            return

    def capture(self, label: str) -> None:
        with self.lock:
            image_msg = copy.deepcopy(self.latest_image)
            camera_info_msg = copy.deepcopy(self.latest_camera_info)

        if image_msg is None:
            raise RuntimeError("No image received yet")
        if camera_info_msg is None:
            raise RuntimeError("No camera_info received yet")

        camera_frame = self.args.camera_frame or image_msg.header.frame_id or camera_info_msg.header.frame_id
        stamp = image_msg.header.stamp
        lookup_stamp = RclpyTime.from_msg(stamp) - Duration(seconds=self.args.tf_lookup_offset_sec)
        try:
            transform = self.tf_buffer.lookup_transform(
                self.args.target_frame,
                camera_frame,
                lookup_stamp,
                timeout=Duration(seconds=self.args.tf_timeout_sec),
            )
        except TransformException as exc:
            if not self.args.allow_latest_tf:
                raise RuntimeError(
                    f"TF lookup failed near image stamp {stamp.sec}.{stamp.nanosec:09d}: {exc}"
                ) from exc
            self.get_logger().warn(
                f"TF lookup at image stamp failed, using latest transform instead: {exc}"
            )
            transform = self.tf_buffer.lookup_transform(
                self.args.target_frame,
                camera_frame,
                RclpyTime(),
                timeout=Duration(seconds=self.args.tf_timeout_sec),
            )

        image_name = f"{self.frame_index:05d}.{self.args.image_format}"
        image_path = self.images_dir / image_name
        save_image(image_path, image_msg)

        t = transform.transform.translation
        q = transform.transform.rotation
        rotation_matrix = quaternion_to_matrix(q.x, q.y, q.z, q.w)
        roll, pitch, yaw = matrix_to_euler_xyz(rotation_matrix)
        try:
            image_ref = image_path.relative_to(self.poses_csv.parent)
        except ValueError:
            image_ref = image_path

        row = {
            "image": str(image_ref),
            "stamp_sec": stamp.sec,
            "stamp_nanosec": stamp.nanosec,
            "target_frame": self.args.target_frame,
            "camera_frame": camera_frame,
            "x": t.x,
            "y": t.y,
            "z": t.z,
            "qx": q.x,
            "qy": q.y,
            "qz": q.z,
            "qw": q.w,
            "roll": roll,
            "pitch": pitch,
            "yaw": yaw,
            "image_width": image_msg.width,
            "image_height": image_msg.height,
            "encoding": image_msg.encoding,
            "label": label,
        }
        self.rows.append(row)
        self.write_outputs(camera_info_msg)
        self.frame_index += 1
        self.get_logger().info(
            f"Saved {image_name}: {self.args.target_frame}->{camera_frame} "
            f"xyz=({t.x:.3f}, {t.y:.3f}, {t.z:.3f}) stamp={stamp.sec}.{stamp.nanosec:09d}"
        )

    def write_outputs(self, camera_info_msg: CameraInfo) -> None:
        fieldnames = [
            "image",
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
            "roll",
            "pitch",
            "yaw",
            "image_width",
            "image_height",
            "encoding",
            "label",
        ]
        with self.poses_csv.open("w", newline="") as csv_file:
            writer = csv.DictWriter(csv_file, fieldnames=fieldnames)
            writer.writeheader()
            writer.writerows(self.rows)

        frames = []
        for row in self.rows:
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
                    "stamp": {
                        "sec": int(row["stamp_sec"]),
                        "nanosec": int(row["stamp_nanosec"]),
                    },
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


def start_terminal_listener(node: PhysicalSurveyRecorder) -> threading.Thread:
    def worker() -> None:
        print("Type `c` then Enter to capture. Type `q` then Enter to finish.", flush=True)
        while rclpy.ok():
            try:
                text = input("> ").strip()
            except EOFError:
                rclpy.shutdown()
                return
            if text.lower() in {"q", "quit", "exit"}:
                node.stop_event.set()
                return
            if not text:
                continue
            label = ""
            if text.lower() != "c":
                label = text
            node.request_capture(label)

    thread = threading.Thread(target=worker, daemon=True)
    thread.start()
    return thread


def parse_args() -> tuple[argparse.Namespace, list[str]]:
    parser = argparse.ArgumentParser(
        description="Capture physical TurtleBot OAK-D images and matching TF camera poses."
    )
    parser.add_argument("--image-topic", default="/robot_1/oakd/rgb/image_raw")
    parser.add_argument("--camera-info-topic", default="/robot_1/oakd/rgb/camera_info")
    parser.add_argument("--target-frame", default="map", help="World frame for saved camera poses, usually map or odom")
    parser.add_argument("--camera-frame", default="oakd_rgb_camera_optical_frame")
    parser.add_argument("--images-dir", type=Path, required=True)
    parser.add_argument("--poses-csv", type=Path, required=True)
    parser.add_argument("--transforms-json", type=Path, required=True)
    parser.add_argument("--camera-info-json", type=Path, required=True)
    parser.add_argument("--image-format", choices=["png"], default="png")
    parser.add_argument("--start-index", type=int, default=0)
    parser.add_argument("--qos-depth", type=int, default=5)
    parser.add_argument("--tf-cache-sec", type=float, default=30.0)
    parser.add_argument("--tf-timeout-sec", type=float, default=1.0)
    parser.add_argument(
        "--tf-lookup-offset-sec",
        type=float,
        default=0.0,
        help="Subtract this offset from the image timestamp before TF lookup",
    )
    parser.add_argument(
        "--allow-latest-tf",
        action="store_true",
        help="Use latest TF if exact image-stamp lookup is unavailable",
    )
    return parser.parse_known_args()


def main() -> int:
    args, ros_args = parse_args()
    rclpy.init(args=ros_args)
    node = PhysicalSurveyRecorder(args)
    start_terminal_listener(node)
    try:
        while rclpy.ok() and not node.stop_event.is_set():
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    print(f"Saved pose CSV to {args.poses_csv.resolve()}")
    print(f"Saved transforms JSON to {args.transforms_json.resolve()}")
    print(f"Saved camera_info JSON to {args.camera_info_json.resolve()}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
