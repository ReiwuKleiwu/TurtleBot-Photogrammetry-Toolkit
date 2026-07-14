#!/usr/bin/env python3
from __future__ import annotations

import argparse
import csv
import json
import math
import sqlite3
import struct
import sys
from pathlib import Path

IMAGE_EXTS = {".png", ".jpg", ".jpeg"}


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Export a COLMAP text model from physical TurtleBot capture outputs. "
            "Input poses must be ROS optical camera-to-world poses."
        )
    )
    parser.add_argument("--poses-csv", type=Path, required=True)
    parser.add_argument("--images-dir", type=Path, required=True)
    parser.add_argument("--camera-info-json", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--database-path", type=Path, help="Optional COLMAP database.db. If set, image IDs match database image IDs")
    parser.add_argument(
        "--database-connected-only",
        action="store_true",
        help="Only export images that have verified two-view geometry in the COLMAP database",
    )
    parser.add_argument("--camera-model", choices=["PINHOLE", "SIMPLE_PINHOLE", "OPENCV", "FULL_OPENCV"], default="PINHOLE")
    parser.add_argument("--single-camera", action="store_true", default=True)
    parser.add_argument("--per-image-camera", dest="single_camera", action="store_false")
    return parser.parse_args()


def read_png_size(path: Path) -> tuple[int, int]:
    with path.open("rb") as f:
        header = f.read(24)
    if len(header) < 24 or header[:8] != b"\x89PNG\r\n\x1a\n" or header[12:16] != b"IHDR":
        raise ValueError(f"Not a valid PNG: {path}")
    return struct.unpack(">II", header[16:24])


def read_jpeg_size(path: Path) -> tuple[int, int]:
    with path.open("rb") as f:
        if f.read(2) != b"\xff\xd8":
            raise ValueError(f"Not a valid JPEG: {path}")
        while True:
            marker_start = f.read(1)
            if marker_start == b"":
                break
            if marker_start != b"\xff":
                continue
            marker = f.read(1)
            while marker == b"\xff":
                marker = f.read(1)
            if marker in {b"\xc0", b"\xc1", b"\xc2", b"\xc3", b"\xc5", b"\xc6", b"\xc7", b"\xc9", b"\xca", b"\xcb", b"\xcd", b"\xce", b"\xcf"}:
                length = struct.unpack(">H", f.read(2))[0]
                data = f.read(length - 2)
                height, width = struct.unpack(">HH", data[1:5])
                return width, height
            if marker in {b"\xd8", b"\xd9"}:
                continue
            length_data = f.read(2)
            if len(length_data) != 2:
                break
            length = struct.unpack(">H", length_data)[0]
            f.seek(length - 2, 1)
    raise ValueError(f"Could not read JPEG size: {path}")


def image_size(path: Path) -> tuple[int, int]:
    suffix = path.suffix.lower()
    if suffix == ".png":
        return read_png_size(path)
    if suffix in {".jpg", ".jpeg"}:
        return read_jpeg_size(path)
    raise ValueError(f"Unsupported image extension '{path.suffix}' for {path}")


def transpose(m: list[list[float]]) -> list[list[float]]:
    return [[m[col][row] for col in range(3)] for row in range(3)]


def matvec(m: list[list[float]], v: list[float]) -> list[float]:
    return [sum(m[row][col] * v[col] for col in range(3)) for row in range(3)]


def quaternion_to_matrix(x: float, y: float, z: float, w: float) -> list[list[float]]:
    xx, yy, zz = x * x, y * y, z * z
    xy, xz, yz = x * y, x * z, y * z
    wx, wy, wz = w * x, w * y, w * z
    return [
        [1.0 - 2.0 * (yy + zz), 2.0 * (xy - wz), 2.0 * (xz + wy)],
        [2.0 * (xy + wz), 1.0 - 2.0 * (xx + zz), 2.0 * (yz - wx)],
        [2.0 * (xz - wy), 2.0 * (yz + wx), 1.0 - 2.0 * (xx + yy)],
    ]


def rotation_matrix_to_quaternion(r: list[list[float]]) -> list[float]:
    trace = r[0][0] + r[1][1] + r[2][2]
    if trace > 0.0:
        s = math.sqrt(trace + 1.0) * 2.0
        qw = 0.25 * s
        qx = (r[2][1] - r[1][2]) / s
        qy = (r[0][2] - r[2][0]) / s
        qz = (r[1][0] - r[0][1]) / s
    elif r[0][0] > r[1][1] and r[0][0] > r[2][2]:
        s = math.sqrt(1.0 + r[0][0] - r[1][1] - r[2][2]) * 2.0
        qw = (r[2][1] - r[1][2]) / s
        qx = 0.25 * s
        qy = (r[0][1] + r[1][0]) / s
        qz = (r[0][2] + r[2][0]) / s
    elif r[1][1] > r[2][2]:
        s = math.sqrt(1.0 + r[1][1] - r[0][0] - r[2][2]) * 2.0
        qw = (r[0][2] - r[2][0]) / s
        qx = (r[0][1] + r[1][0]) / s
        qy = 0.25 * s
        qz = (r[1][2] + r[2][1]) / s
    else:
        s = math.sqrt(1.0 + r[2][2] - r[0][0] - r[1][1]) * 2.0
        qw = (r[1][0] - r[0][1]) / s
        qx = (r[0][2] + r[2][0]) / s
        qy = (r[1][2] + r[2][1]) / s
        qz = 0.25 * s
    norm = math.sqrt(qw * qw + qx * qx + qy * qy + qz * qz)
    return [qw / norm, qx / norm, qy / norm, qz / norm]


def load_database_image_ids(database_path: Path) -> dict[str, int]:
    conn = sqlite3.connect(database_path)
    try:
        rows = conn.execute("SELECT image_id, name FROM images").fetchall()
    finally:
        conn.close()
    return {name: image_id for image_id, name in rows}


def load_connected_database_image_ids(database_path: Path) -> set[int]:
    pair_id_constant = 2147483647
    conn = sqlite3.connect(database_path)
    try:
        rows = conn.execute("SELECT pair_id, rows FROM two_view_geometries").fetchall()
    finally:
        conn.close()

    connected: set[int] = set()
    for pair_id, num_rows in rows:
        if num_rows <= 0:
            continue
        image_id2 = pair_id % pair_id_constant
        image_id1 = (pair_id - image_id2) // pair_id_constant
        connected.add(image_id1)
        connected.add(image_id2)
    return connected


def resolve_image_path(images_dir: Path, poses_csv: Path, image_ref: str) -> tuple[Path, str]:
    ref = Path(image_ref)
    candidates = []
    if ref.is_absolute():
        candidates.append(ref)
    else:
        candidates.append(images_dir / ref.name)
        candidates.append(images_dir / ref)
        candidates.append(poses_csv.parent / ref)
    for candidate in candidates:
        if candidate.exists():
            return candidate.resolve(), candidate.name
    raise FileNotFoundError(f"Could not resolve image '{image_ref}' under {images_dir}")


def camera_params(camera_model: str, camera_info: dict) -> list[float]:
    k = camera_info["k"]
    fx, fy, cx, cy = float(k[0]), float(k[4]), float(k[2]), float(k[5])
    d = [float(value) for value in camera_info.get("d", [])]
    if camera_model == "SIMPLE_PINHOLE":
        if abs(fx - fy) > 1e-6:
            print("Warning: SIMPLE_PINHOLE requested but fx != fy; using fx", file=sys.stderr)
        return [fx, cx, cy]
    if camera_model == "PINHOLE":
        return [fx, fy, cx, cy]
    if camera_model == "OPENCV":
        padded = (d + [0.0, 0.0, 0.0, 0.0])[:4]
        return [fx, fy, cx, cy, *padded]
    if camera_model == "FULL_OPENCV":
        padded = (d + [0.0] * 8)[:8]
        return [fx, fy, cx, cy, *padded]
    raise ValueError(f"Unsupported camera model: {camera_model}")


def colmap_pose_from_ros_optical_row(row: dict[str, str]) -> tuple[list[float], list[float]]:
    center = [float(row["x"]), float(row["y"]), float(row["z"])]
    rot_c2w = quaternion_to_matrix(
        float(row["qx"]),
        float(row["qy"]),
        float(row["qz"]),
        float(row["qw"]),
    )
    rot_w2c = transpose(rot_c2w)
    translation = [-value for value in matvec(rot_w2c, center)]
    quaternion = rotation_matrix_to_quaternion(rot_w2c)
    return quaternion, translation


def main() -> int:
    args = parse_args()
    poses_csv = args.poses_csv.resolve()
    images_dir = args.images_dir.resolve()
    output_dir = args.output_dir.resolve()
    camera_info_path = args.camera_info_json.resolve()

    if not poses_csv.exists():
        print(f"poses.csv does not exist: {poses_csv}", file=sys.stderr)
        return 1
    if not images_dir.exists():
        print(f"images directory does not exist: {images_dir}", file=sys.stderr)
        return 1
    if not camera_info_path.exists():
        print(f"camera_info JSON does not exist: {camera_info_path}", file=sys.stderr)
        return 1

    camera_info = json.loads(camera_info_path.read_text())
    database_path = args.database_path.resolve() if args.database_path else None
    db_image_ids = load_database_image_ids(database_path) if database_path else None
    connected_image_ids = (
        load_connected_database_image_ids(database_path)
        if database_path and args.database_connected_only
        else None
    )
    params = camera_params(args.camera_model, camera_info)
    camera_rows: list[tuple[int, int, int, list[float]]] = []
    image_rows: list[tuple[int, list[float], list[float], int, str]] = []

    with poses_csv.open(newline="") as f:
        rows = list(csv.DictReader(f))
    if not rows:
        print(f"No pose rows found in {poses_csv}", file=sys.stderr)
        return 1

    shared_camera_spec: tuple[int, int, int, list[float]] | None = None
    for fallback_image_id, row in enumerate(rows, start=1):
        image_path, image_name = resolve_image_path(images_dir, poses_csv, row["image"])
        if image_path.suffix.lower() not in IMAGE_EXTS:
            print(f"Unsupported image extension for {image_path}", file=sys.stderr)
            return 1
        width, height = image_size(image_path)

        image_id = fallback_image_id
        if db_image_ids is not None:
            image_id = db_image_ids.get(image_name, -1)
            if image_id < 0:
                print(f"Image '{image_name}' is not present in database '{args.database_path}'", file=sys.stderr)
                return 1
            if connected_image_ids is not None and image_id not in connected_image_ids:
                continue

        camera_id = 1 if args.single_camera else image_id
        if args.single_camera:
            if shared_camera_spec is None:
                shared_camera_spec = (1, width, height, params)
                camera_rows.append(shared_camera_spec)
            elif (width, height) != (shared_camera_spec[1], shared_camera_spec[2]):
                print("All images must have the same dimensions when using --single-camera", file=sys.stderr)
                return 1
        else:
            camera_rows.append((camera_id, width, height, params))

        quaternion, translation = colmap_pose_from_ros_optical_row(row)
        image_rows.append((image_id, quaternion, translation, camera_id, image_name))

    image_rows.sort(key=lambda row: row[0])
    output_dir.mkdir(parents=True, exist_ok=True)

    with (output_dir / "cameras.txt").open("w", encoding="ascii") as f:
        f.write("# Camera list with one line of data per camera:\n")
        f.write("#   CAMERA_ID, MODEL, WIDTH, HEIGHT, PARAMS[]\n")
        f.write(f"# Number of cameras: {len(camera_rows)}\n")
        for camera_id, width, height, params_row in camera_rows:
            param_str = " ".join(f"{value:.17g}" for value in params_row)
            f.write(f"{camera_id} {args.camera_model} {width} {height} {param_str}\n")

    with (output_dir / "images.txt").open("w", encoding="ascii") as f:
        f.write("# Image list with two lines of data per image:\n")
        f.write("#   IMAGE_ID, QW, QX, QY, QZ, TX, TY, TZ, CAMERA_ID, NAME\n")
        f.write("#   POINTS2D[] as (X, Y, POINT3D_ID)\n")
        f.write(f"# Number of images: {len(image_rows)}\n")
        for image_id, quaternion, translation, camera_id, image_name in image_rows:
            pose_str = " ".join(f"{value:.17g}" for value in quaternion + translation)
            f.write(f"{image_id} {pose_str} {camera_id} {image_name}\n\n")

    with (output_dir / "points3D.txt").open("w", encoding="ascii") as f:
        f.write("# 3D point list with one line of data per point:\n")
        f.write("#   POINT3D_ID, X, Y, Z, R, G, B, ERROR, TRACK[] as (IMAGE_ID, POINT2D_IDX)\n")
        f.write("# Number of points: 0\n")

    print(f"Wrote physical TurtleBot COLMAP text model to {output_dir}")
    print(f"Images referenced by name from {images_dir}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
