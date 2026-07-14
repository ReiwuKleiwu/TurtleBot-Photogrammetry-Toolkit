#!/usr/bin/env python3
from __future__ import annotations

import argparse
import csv
import math
from pathlib import Path
from typing import Any

import numpy as np


def parse_simple_yaml(path: Path) -> dict[str, Any]:
    values: dict[str, Any] = {}
    for raw_line in path.read_text().splitlines():
        line = raw_line.split("#", 1)[0].strip()
        if not line or ":" not in line:
            continue
        key, value = line.split(":", 1)
        key = key.strip()
        value = value.strip()
        if value.startswith("[") and value.endswith("]"):
            values[key] = [float(item.strip()) for item in value[1:-1].split(",") if item.strip()]
        else:
            try:
                values[key] = float(value)
            except ValueError:
                values[key] = value.strip("'\"")
    return values


def read_pgm(path: Path) -> np.ndarray:
    with path.open("rb") as f:
        magic = f.readline().strip()
        if magic != b"P5":
            raise ValueError(f"Unsupported PGM magic {magic!r}; expected P5")
        line = f.readline()
        while line.startswith(b"#"):
            line = f.readline()
        width, height = [int(value) for value in line.split()]
        max_value = int(f.readline())
        if max_value > 255:
            data = np.frombuffer(f.read(width * height * 2), dtype=">u2")
            data = (data / 256).astype(np.uint8)
        else:
            data = np.frombuffer(f.read(width * height), dtype=np.uint8)
        return data.reshape((height, width))


def world_to_pixel(x: float, y: float, height: int, resolution: float, origin: list[float]) -> tuple[int, int]:
    px = int((x - origin[0]) / resolution)
    py = int(height - 1 - (y - origin[1]) / resolution)
    return px, py


def load_waypoints(path: Path) -> list[dict[str, float]]:
    with path.open(newline="") as f:
        rows = list(csv.DictReader(f))
    return [
        {
            "x": float(row["x"]),
            "y": float(row["y"]),
            "yaw": float(row.get("yaw", 0.0)),
        }
        for row in rows
    ]


def render_overlay(
    map_image: np.ndarray,
    waypoints: list[dict[str, float]],
    resolution: float,
    origin: list[float],
    output_path: Path,
    scale: int,
    draw_labels: bool,
) -> None:
    try:
        import cv2
    except ImportError as exc:
        raise RuntimeError("python3-opencv is required to render overlays") from exc

    base = np.zeros((map_image.shape[0], map_image.shape[1], 3), dtype=np.uint8)
    base[map_image > 240] = (245, 245, 245)
    base[map_image < 80] = (35, 35, 35)
    unknown = (map_image >= 80) & (map_image <= 240)
    base[unknown] = (150, 150, 150)

    points = [world_to_pixel(wp["x"], wp["y"], map_image.shape[0], resolution, origin) for wp in waypoints]
    for a, b in zip(points, points[1:]):
        cv2.line(base, a, b, (70, 150, 255), 1, cv2.LINE_AA)

    for index, (wp, point) in enumerate(zip(waypoints, points)):
        cv2.circle(base, point, 3, (0, 95, 255), -1)
        end = (
            int(round(point[0] + math.cos(wp["yaw"]) * 10)),
            int(round(point[1] - math.sin(wp["yaw"]) * 10)),
        )
        cv2.line(base, point, end, (0, 180, 0), 1, cv2.LINE_AA)
        if draw_labels and (index % 10 == 0 or index == len(points) - 1):
            cv2.putText(base, str(index), (point[0] + 4, point[1] - 4), cv2.FONT_HERSHEY_SIMPLEX, 0.35, (0, 75, 220), 1)

    if scale != 1:
        base = cv2.resize(base, (base.shape[1] * scale, base.shape[0] * scale), interpolation=cv2.INTER_NEAREST)

    output_path.parent.mkdir(parents=True, exist_ok=True)
    if not cv2.imwrite(str(output_path), base):
        raise RuntimeError(f"Failed to write overlay: {output_path}")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Render a high-resolution PNG overlay for an existing photogrammetry path CSV.")
    parser.add_argument("--map-yaml", type=Path, required=True)
    parser.add_argument("--path-csv", type=Path, required=True)
    parser.add_argument("--overlay-png", type=Path, required=True)
    parser.add_argument("--scale", type=int, default=8)
    parser.add_argument("--no-labels", action="store_true")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    config = parse_simple_yaml(args.map_yaml.resolve())
    image_path = args.map_yaml.resolve().parent / str(config["image"])
    map_image = read_pgm(image_path)
    waypoints = load_waypoints(args.path_csv.resolve())
    render_overlay(
        map_image=map_image,
        waypoints=waypoints,
        resolution=float(config["resolution"]),
        origin=list(config["origin"]),
        output_path=args.overlay_png.resolve(),
        scale=args.scale,
        draw_labels=not args.no_labels,
    )
    print(f"Rendered {len(waypoints)} waypoints to {args.overlay_png.resolve()}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
