#!/usr/bin/env python3
from __future__ import annotations

import argparse
import csv
import json
import math
import struct
from collections import deque
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
        elif value.lower() in {"true", "false"}:
            values[key] = value.lower() == "true"
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
        else:
            data = np.frombuffer(f.read(width * height), dtype=np.uint8)
        return data.reshape((height, width))


def classify_free_cells(image: np.ndarray, map_config: dict[str, Any]) -> np.ndarray:
    negate = int(map_config.get("negate", 0))
    occupied_thresh = float(map_config.get("occupied_thresh", 0.65))
    free_thresh = float(map_config.get("free_thresh", 0.196))
    normalized = image.astype(np.float32) / 255.0
    if negate:
        occupancy = normalized
    else:
        occupancy = 1.0 - normalized
    return occupancy <= free_thresh


def distance_from_obstacles(free: np.ndarray) -> np.ndarray:
    height, width = free.shape
    large = height + width + 1
    dist = np.full((height, width), large, dtype=np.int32)
    q: deque[tuple[int, int]] = deque()
    obstacle_or_unknown = ~free
    ys, xs = np.nonzero(obstacle_or_unknown)
    for y, x in zip(ys, xs):
        dist[y, x] = 0
        q.append((y, x))
    while q:
        y, x = q.popleft()
        next_dist = dist[y, x] + 1
        for ny, nx in ((y - 1, x), (y + 1, x), (y, x - 1), (y, x + 1)):
            if 0 <= ny < height and 0 <= nx < width and next_dist < dist[ny, nx]:
                dist[ny, nx] = next_dist
                q.append((ny, nx))
    return dist


def pixel_to_world(x: int, y: int, height: int, resolution: float, origin: list[float]) -> tuple[float, float]:
    world_x = origin[0] + (x + 0.5) * resolution
    world_y = origin[1] + (height - y - 0.5) * resolution
    return world_x, world_y


def world_to_pixel(x: float, y: float, height: int, resolution: float, origin: list[float]) -> tuple[int, int]:
    px = int((x - origin[0]) / resolution)
    py = int(height - 1 - (y - origin[1]) / resolution)
    return px, py


def nearest_true(mask: np.ndarray, point: tuple[int, int]) -> tuple[int, int] | None:
    px, py = point
    ys, xs = np.nonzero(mask)
    if len(xs) == 0:
        return None
    distances = (xs - px) ** 2 + (ys - py) ** 2
    index = int(np.argmin(distances))
    return int(xs[index]), int(ys[index])


def connected_component_from(mask: np.ndarray, start: tuple[int, int]) -> np.ndarray:
    nearest = nearest_true(mask, start)
    if nearest is None:
        return np.zeros_like(mask, dtype=bool)

    component = np.zeros_like(mask, dtype=bool)
    q: deque[tuple[int, int]] = deque([nearest])
    component[nearest[1], nearest[0]] = True
    while q:
        x, y = q.popleft()
        for nx, ny in ((x - 1, y), (x + 1, y), (x, y - 1), (x, y + 1)):
            if 0 <= ny < mask.shape[0] and 0 <= nx < mask.shape[1] and mask[ny, nx] and not component[ny, nx]:
                component[ny, nx] = True
                q.append((nx, ny))
    return component


def sample_candidates(
    free: np.ndarray,
    clearance_px: int,
    spacing_px: int,
    border_px: int,
) -> list[tuple[int, int]]:
    dist = distance_from_obstacles(free)
    safe = free & (dist >= clearance_px)
    height, width = free.shape
    candidates: list[tuple[int, int]] = []
    for y in range(border_px, height - border_px, spacing_px):
        for x in range(border_px, width - border_px, spacing_px):
            if not safe[y, x]:
                window_radius = max(1, spacing_px // 3)
                y0 = max(border_px, y - window_radius)
                y1 = min(height - border_px, y + window_radius + 1)
                x0 = max(border_px, x - window_radius)
                x1 = min(width - border_px, x + window_radius + 1)
                local = np.argwhere(safe[y0:y1, x0:x1])
                if local.size == 0:
                    continue
                best_index = int(np.argmax(dist[y0:y1, x0:x1][safe[y0:y1, x0:x1]]))
                safe_points = local
                ly, lx = safe_points[best_index]
                candidates.append((x0 + int(lx), y0 + int(ly)))
                continue
            candidates.append((x, y))

    unique: list[tuple[int, int]] = []
    seen: set[tuple[int, int]] = set()
    min_sep = max(1, spacing_px // 2)
    for point in candidates:
        if point in seen:
            continue
        if any((point[0] - old[0]) ** 2 + (point[1] - old[1]) ** 2 < min_sep * min_sep for old in unique):
            continue
        seen.add(point)
        unique.append(point)
    return unique


def route_nearest_neighbor(points: list[tuple[int, int]], start: tuple[int, int] | None) -> list[tuple[int, int]]:
    if not points:
        return []
    remaining = points.copy()
    if start is None:
        current = min(remaining, key=lambda p: (p[1], p[0]))
    else:
        current = min(remaining, key=lambda p: (p[0] - start[0]) ** 2 + (p[1] - start[1]) ** 2)
    route = [current]
    remaining.remove(current)
    while remaining:
        current = min(remaining, key=lambda p: (p[0] - current[0]) ** 2 + (p[1] - current[1]) ** 2)
        route.append(current)
        remaining.remove(current)
    return route


def safe_mask(free: np.ndarray, clearance_px: int) -> np.ndarray:
    dist = distance_from_obstacles(free)
    return free & (dist >= clearance_px)


def safe_intervals(values: np.ndarray) -> list[tuple[int, int]]:
    intervals: list[tuple[int, int]] = []
    start: int | None = None
    for i, is_safe in enumerate(values):
        if is_safe and start is None:
            start = i
        elif not is_safe and start is not None:
            intervals.append((start, i - 1))
            start = None
    if start is not None:
        intervals.append((start, len(values) - 1))
    return intervals


def sample_segment(start: int, end: int, step_px: int) -> list[int]:
    if start == end:
        return [start]
    direction = 1 if end > start else -1
    values = list(range(start, end + direction, direction * step_px))
    if values[-1] != end:
        values.append(end)
    return values


def generate_lawnmower_route(
    free: np.ndarray,
    clearance_px: int,
    track_spacing_px: int,
    station_spacing_px: int,
    min_segment_px: int,
    start: tuple[int, int] | None,
    axis: str,
) -> list[tuple[int, int]]:
    safe = safe_mask(free, clearance_px)
    height, width = safe.shape
    scanlines: list[tuple[int, tuple[int, int]]] = []

    if axis == "x":
        for y in range(0, height, track_spacing_px):
            intervals = safe_intervals(safe[y, :])
            intervals = [interval for interval in intervals if interval[1] - interval[0] + 1 >= min_segment_px]
            if not intervals:
                continue
            interval = max(intervals, key=lambda item: item[1] - item[0])
            scanlines.append((y, interval))
    elif axis == "y":
        for x in range(0, width, track_spacing_px):
            intervals = safe_intervals(safe[:, x])
            intervals = [interval for interval in intervals if interval[1] - interval[0] + 1 >= min_segment_px]
            if not intervals:
                continue
            interval = max(intervals, key=lambda item: item[1] - item[0])
            scanlines.append((x, interval))
    else:
        raise ValueError(f"Unsupported lawnmower axis: {axis}")

    if not scanlines:
        return []

    if start is not None:
        start_coord = start[1] if axis == "x" else start[0]
        nearest_index = min(range(len(scanlines)), key=lambda i: abs(scanlines[i][0] - start_coord))
        forward = scanlines[nearest_index:]
        backward = list(reversed(scanlines[:nearest_index]))
        scanlines = forward + backward

    route: list[tuple[int, int]] = [start] if start is not None else []
    previous_endpoint = start
    for line_index, (fixed, interval) in enumerate(scanlines):
        lo, hi = interval
        if axis == "x":
            forward_points = [(x, fixed) for x in sample_segment(lo, hi, station_spacing_px)]
            backward_points = [(x, fixed) for x in sample_segment(hi, lo, station_spacing_px)]
        else:
            forward_points = [(fixed, y) for y in sample_segment(lo, hi, station_spacing_px)]
            backward_points = [(fixed, y) for y in sample_segment(hi, lo, station_spacing_px)]

        if previous_endpoint is None:
            points = forward_points if line_index % 2 == 0 else backward_points
        else:
            fdist = (forward_points[0][0] - previous_endpoint[0]) ** 2 + (forward_points[0][1] - previous_endpoint[1]) ** 2
            bdist = (backward_points[0][0] - previous_endpoint[0]) ** 2 + (backward_points[0][1] - previous_endpoint[1]) ** 2
            points = forward_points if fdist <= bdist else backward_points

        if route and points and route[-1] == points[0]:
            route.extend(points[1:])
        else:
            route.extend(points)
        if points:
            previous_endpoint = points[-1]
    return route


def line_is_free(free: np.ndarray, start: tuple[int, int], end: tuple[int, int]) -> bool:
    x0, y0 = start
    x1, y1 = end
    steps = max(abs(x1 - x0), abs(y1 - y0), 1)
    for i in range(steps + 1):
        t = i / steps
        x = int(round(x0 + (x1 - x0) * t))
        y = int(round(y0 + (y1 - y0) * t))
        if not (0 <= y < free.shape[0] and 0 <= x < free.shape[1]) or not free[y, x]:
            return False
    return True


def wall_facing_yaws(
    point: tuple[int, int],
    free: np.ndarray,
    resolution: float,
    max_range_m: float,
    min_hits: int,
) -> list[float]:
    max_range_px = max(1, int(max_range_m / resolution))
    hits: list[tuple[float, float]] = []
    px, py = point
    for deg in range(0, 360, 15):
        yaw = math.radians(deg)
        dx = math.cos(yaw)
        dy = -math.sin(yaw)
        for step in range(1, max_range_px + 1):
            x = int(round(px + dx * step))
            y = int(round(py + dy * step))
            if not (0 <= y < free.shape[0] and 0 <= x < free.shape[1]):
                break
            if not free[y, x]:
                hits.append((step * resolution, yaw))
                break
    hits.sort(key=lambda item: item[0])
    selected: list[float] = []
    for _, yaw in hits:
        if all(abs(math.atan2(math.sin(yaw - old), math.cos(yaw - old))) > math.radians(45) for old in selected):
            selected.append(yaw)
        if len(selected) >= min_hits:
            break
    return selected


def build_waypoints(
    route: list[tuple[int, int]],
    free: np.ndarray,
    resolution: float,
    origin: list[float],
    yaw_mode: str,
    yaws_per_point: int,
    wall_range_m: float,
) -> list[dict[str, float]]:
    waypoints: list[dict[str, float]] = []
    for index, point in enumerate(route):
        x, y = pixel_to_world(point[0], point[1], free.shape[0], resolution, origin)
        if yaw_mode == "route":
            if index + 1 < len(route):
                nxt = route[index + 1]
                nx, ny = pixel_to_world(nxt[0], nxt[1], free.shape[0], resolution, origin)
                yaw_values = [math.atan2(ny - y, nx - x)]
            elif waypoints:
                yaw_values = [waypoints[-1]["yaw"]]
            else:
                yaw_values = [0.0]
        elif yaw_mode == "cardinal":
            yaw_values = [2.0 * math.pi * i / yaws_per_point for i in range(yaws_per_point)]
        elif yaw_mode == "wall":
            yaw_values = wall_facing_yaws(point, free, resolution, wall_range_m, yaws_per_point)
            if len(yaw_values) < yaws_per_point:
                yaw_values.extend(2.0 * math.pi * i / yaws_per_point for i in range(yaws_per_point - len(yaw_values)))
        else:
            raise ValueError(f"Unsupported yaw mode: {yaw_mode}")

        for yaw in yaw_values[:yaws_per_point]:
            waypoints.append(
                {
                    "x": x,
                    "y": y,
                    "yaw": math.atan2(math.sin(yaw), math.cos(yaw)),
                    "grid_x": point[0],
                    "grid_y": point[1],
                }
            )
    return waypoints


def write_overlay(
    path: Path,
    image: np.ndarray,
    free: np.ndarray,
    route: list[tuple[int, int]],
    waypoints: list[dict[str, float]],
    resolution: float,
    origin: list[float],
    scale: int,
) -> None:
    try:
        import cv2
    except ImportError as exc:
        raise RuntimeError("python3-opencv is required to write the overlay image") from exc

    base = np.zeros((image.shape[0], image.shape[1], 3), dtype=np.uint8)
    base[free] = (245, 245, 245)
    base[~free] = (40, 40, 40)
    for a, b in zip(route, route[1:]):
        cv2.line(base, a, b, (80, 160, 255), 1, cv2.LINE_AA)
    for i, point in enumerate(route):
        cv2.circle(base, point, 3, (0, 100, 255), -1)
        if i % 5 == 0:
            cv2.putText(base, str(i), (point[0] + 4, point[1] - 4), cv2.FONT_HERSHEY_SIMPLEX, 0.35, (0, 80, 220), 1)
    for wp in waypoints:
        px, py = world_to_pixel(wp["x"], wp["y"], image.shape[0], resolution, origin)
        end = (
            int(round(px + math.cos(wp["yaw"]) * 10)),
            int(round(py - math.sin(wp["yaw"]) * 10)),
        )
        cv2.line(base, (px, py), end, (0, 180, 0), 1, cv2.LINE_AA)
    if scale != 1:
        base = cv2.resize(base, (base.shape[1] * scale, base.shape[0] * scale), interpolation=cv2.INTER_NEAREST)
    path.parent.mkdir(parents=True, exist_ok=True)
    cv2.imwrite(str(path), base)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Generate an offline photogrammetry waypoint path from a ROS map.")
    parser.add_argument("--map-yaml", type=Path, required=True)
    parser.add_argument("--output-json", type=Path, required=True)
    parser.add_argument("--output-csv", type=Path, required=True)
    parser.add_argument("--overlay-png", type=Path, required=True)
    parser.add_argument("--path-mode", choices=["sampled", "lawnmower"], default="sampled")
    parser.add_argument("--spacing-m", type=float, default=0.75)
    parser.add_argument("--track-spacing-m", type=float, help="Distance between lawnmower scanlines. Defaults to --spacing-m")
    parser.add_argument("--station-spacing-m", type=float, help="Distance between stops on one scanline. Defaults to --spacing-m")
    parser.add_argument("--min-segment-m", type=float, default=0.8)
    parser.add_argument("--lawnmower-axis", choices=["x", "y", "both"], default="x")
    parser.add_argument("--clearance-m", type=float, default=0.35)
    parser.add_argument("--border-m", type=float, default=0.3)
    parser.add_argument("--start-x", type=float)
    parser.add_argument("--start-y", type=float)
    parser.add_argument(
        "--restrict-to-start-component",
        action="store_true",
        help="Discard free pixels not connected to the start point",
    )
    parser.add_argument("--yaw-mode", choices=["route", "cardinal", "wall"], default="wall")
    parser.add_argument("--yaws-per-point", type=int, default=2)
    parser.add_argument("--wall-range-m", type=float, default=3.0)
    parser.add_argument("--overlay-scale", type=int, default=2)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    config = parse_simple_yaml(args.map_yaml)
    resolution = float(config["resolution"])
    origin = list(config["origin"])
    image_path = args.map_yaml.parent / str(config["image"])
    image = read_pgm(image_path)
    free = classify_free_cells(image, config)

    spacing_px = max(1, int(round(args.spacing_m / resolution)))
    track_spacing_px = max(1, int(round((args.track_spacing_m or args.spacing_m) / resolution)))
    station_spacing_px = max(1, int(round((args.station_spacing_m or args.spacing_m) / resolution)))
    min_segment_px = max(1, int(round(args.min_segment_m / resolution)))
    clearance_px = max(1, int(round(args.clearance_m / resolution)))
    border_px = max(0, int(round(args.border_m / resolution)))

    start_px = None
    if args.start_x is not None and args.start_y is not None:
        start_px = world_to_pixel(args.start_x, args.start_y, free.shape[0], resolution, origin)
        if args.restrict_to_start_component:
            component = connected_component_from(free, start_px)
            free = free & component

    if args.path_mode == "sampled":
        candidates = sample_candidates(free, clearance_px, spacing_px, border_px)
        route = route_nearest_neighbor(candidates, start_px)
    else:
        axes = ["x", "y"] if args.lawnmower_axis == "both" else [args.lawnmower_axis]
        route = []
        current_start = start_px
        for axis in axes:
            partial = generate_lawnmower_route(
                free,
                clearance_px,
                track_spacing_px,
                station_spacing_px,
                min_segment_px,
                current_start,
                axis,
            )
            if route and partial and route[-1] == partial[0]:
                route.extend(partial[1:])
            else:
                route.extend(partial)
            if partial:
                current_start = partial[-1]
        candidates = route

    waypoints = build_waypoints(route, free, resolution, origin, args.yaw_mode, args.yaws_per_point, args.wall_range_m)

    args.output_json.parent.mkdir(parents=True, exist_ok=True)
    args.output_csv.parent.mkdir(parents=True, exist_ok=True)
    args.output_json.write_text(json.dumps({"map": str(args.map_yaml), "waypoints": waypoints}, indent=2))
    with args.output_csv.open("w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=["x", "y", "yaw", "grid_x", "grid_y"])
        writer.writeheader()
        writer.writerows(waypoints)

    write_overlay(args.overlay_png, image, free, route, waypoints, resolution, origin, args.overlay_scale)
    print(f"Sampled {len(candidates)} camera stations")
    print(f"Wrote {len(waypoints)} pose waypoints")
    print(f"JSON: {args.output_json}")
    print(f"CSV: {args.output_csv}")
    print(f"Overlay: {args.overlay_png}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
