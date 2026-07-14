#!/usr/bin/env python3
from __future__ import annotations

import argparse
import csv
import json
import math
from pathlib import Path
from tkinter import (
    BOTH,
    BOTTOM,
    END,
    HORIZONTAL,
    LEFT,
    RIGHT,
    TOP,
    X,
    Button,
    Canvas,
    Checkbutton,
    DoubleVar,
    Entry,
    Frame,
    IntVar,
    Label,
    Listbox,
    Scale,
    StringVar,
    Tk,
    filedialog,
    messagebox,
)
from tkinter import ttk


def parse_simple_yaml(path: Path) -> dict:
    values = {}
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


def read_pgm(path: Path) -> tuple[int, int, bytes]:
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
            raise ValueError("Only 8-bit PGM maps are supported")
        data = f.read(width * height)
    return width, height, data


def normalize_yaw(yaw: float) -> float:
    return math.atan2(math.sin(yaw), math.cos(yaw))


def is_free_value(value: int) -> bool:
    return value > 240


def safe_intervals(values: list[bool]) -> list[tuple[int, int]]:
    intervals: list[tuple[int, int]] = []
    start = None
    for i, is_safe in enumerate(values):
        if is_safe and start is None:
            start = i
        elif not is_safe and start is not None:
            intervals.append((start, i - 1))
            start = None
    if start is not None:
        intervals.append((start, len(values) - 1))
    return intervals


def sample_segment(start: int, end: int, step: int) -> list[int]:
    if start == end:
        return [start]
    direction = 1 if end > start else -1
    values = list(range(start, end + direction, direction * step))
    if values[-1] != end:
        values.append(end)
    return values


class PathEditor:
    def __init__(self, args: argparse.Namespace):
        self.args = args
        self.map_yaml = args.map_yaml.resolve()
        self.path_csv = args.path_csv.resolve() if args.path_csv else None
        self.output_csv = (args.output_csv or args.path_csv or Path("data/path_plans/region_lawnmower.csv")).resolve()
        self.output_json = args.output_json.resolve() if args.output_json else self.output_csv.with_suffix(".json")

        self.map_config = parse_simple_yaml(self.map_yaml)
        self.resolution = float(self.map_config["resolution"])
        self.origin = list(self.map_config["origin"])
        image_path = self.map_yaml.parent / str(self.map_config["image"])
        self.map_width, self.map_height, self.map_data = read_pgm(image_path)
        self.waypoints = self.load_waypoints(self.path_csv) if self.path_csv else []

        self.root = Tk()
        self.root.title("Photogrammetry Path Editor")
        self.scale = IntVar(value=args.scale)
        self.add_mode = IntVar(value=0)
        self.region_mode = IntVar(value=0)
        self.start_mode = IntVar(value=0)
        self.selected_index: int | None = 0 if self.waypoints else None
        self.drag_index: int | None = None
        self.region_drag_start: tuple[int, int] | None = None
        self.region_rect: tuple[int, int, int, int] | None = None
        self.start_point: tuple[int, int] | None = None
        self.last_mouse = (0, 0)
        self.yaw_var = DoubleVar(value=0.0)
        self.grid_var = DoubleVar(value=0.45)
        self.clearance_var = DoubleVar(value=0.30)
        self.axis_var = StringVar(value="both")
        self.status_var = StringVar(value="")

        self.build_ui()
        self.bind_keys()
        self.refresh_all()

    def load_waypoints(self, path: Path) -> list[dict[str, float]]:
        with path.open(newline="") as f:
            rows = list(csv.DictReader(f))
        waypoints = []
        for row in rows:
            waypoints.append(
                {
                    "x": float(row["x"]),
                    "y": float(row["y"]),
                    "yaw": float(row.get("yaw", 0.0)),
                    "grid_x": float(row.get("grid_x", 0.0)),
                    "grid_y": float(row.get("grid_y", 0.0)),
                }
            )
        return waypoints

    def build_ui(self) -> None:
        root = self.root
        main = Frame(root)
        main.pack(fill=BOTH, expand=True)

        left = Frame(main)
        left.pack(side=LEFT, fill=BOTH, expand=True)
        right = Frame(main)
        right.pack(side=RIGHT, fill="y")

        toolbar = Frame(left)
        toolbar.pack(side=TOP, fill=X)
        Button(toolbar, text="Save", command=self.save).pack(side=LEFT)
        Button(toolbar, text="Save As", command=self.save_as).pack(side=LEFT)
        Button(toolbar, text="Delete", command=self.delete_selected).pack(side=LEFT)
        Button(toolbar, text="Insert After", command=self.insert_after_selected).pack(side=LEFT)
        Button(toolbar, text="Yaw -15", command=lambda: self.rotate_selected(math.radians(-15))).pack(side=LEFT)
        Button(toolbar, text="Yaw +15", command=lambda: self.rotate_selected(math.radians(15))).pack(side=LEFT)
        Checkbutton(toolbar, text="Add on click", variable=self.add_mode).pack(side=LEFT)
        Label(toolbar, text="Scale").pack(side=LEFT)
        Scale(toolbar, variable=self.scale, from_=2, to=8, orient=HORIZONTAL, command=lambda _: self.refresh_all(), length=120).pack(side=LEFT)

        self.canvas = Canvas(left, bg="#242424", highlightthickness=0)
        self.canvas.pack(fill=BOTH, expand=True)
        self.canvas.bind("<Button-1>", self.on_click)
        self.canvas.bind("<B1-Motion>", self.on_drag)
        self.canvas.bind("<ButtonRelease-1>", self.on_release)
        self.canvas.bind("<Motion>", self.on_motion)

        info = Frame(right)
        info.pack(side=TOP, fill=X, padx=8, pady=8)
        Label(info, text="Selected").pack(anchor="w")
        self.index_label = Label(info, text="-")
        self.index_label.pack(anchor="w")
        Label(info, text="x").pack(anchor="w")
        self.x_entry = Entry(info, width=14)
        self.x_entry.pack(anchor="w")
        Label(info, text="y").pack(anchor="w")
        self.y_entry = Entry(info, width=14)
        self.y_entry.pack(anchor="w")
        Label(info, text="yaw rad").pack(anchor="w")
        self.yaw_entry = Entry(info, width=14)
        self.yaw_entry.pack(anchor="w")
        Button(info, text="Apply Values", command=self.apply_entry_values).pack(anchor="w", pady=(4, 10))

        planner = Frame(right)
        planner.pack(side=TOP, fill=X, padx=8, pady=(0, 8))
        Label(planner, text="Region Planner").pack(anchor="w")
        Button(planner, text="Select Region", command=self.enable_region_mode).pack(anchor="w", fill=X)
        Button(planner, text="Set Start", command=self.enable_start_mode).pack(anchor="w", fill=X)
        Label(planner, text="grid m").pack(anchor="w")
        self.grid_entry = Entry(planner, width=10)
        self.grid_entry.insert(0, f"{self.grid_var.get():.2f}")
        self.grid_entry.pack(anchor="w")
        Label(planner, text="clearance m").pack(anchor="w")
        self.clearance_entry = Entry(planner, width=10)
        self.clearance_entry.insert(0, f"{self.clearance_var.get():.2f}")
        self.clearance_entry.pack(anchor="w")
        Label(planner, text="axis").pack(anchor="w")
        self.axis_combo = ttk.Combobox(planner, textvariable=self.axis_var, values=["x", "y", "both"], width=8, state="readonly")
        self.axis_combo.pack(anchor="w")
        Button(planner, text="Generate In Region", command=self.generate_region_lawnmower).pack(anchor="w", fill=X, pady=(4, 0))

        Label(right, text="Waypoints").pack(anchor="w", padx=8)
        self.listbox = Listbox(right, width=34, height=30)
        self.listbox.pack(fill=BOTH, expand=True, padx=8)
        self.listbox.bind("<<ListboxSelect>>", self.on_list_select)

        status = Label(root, textvariable=self.status_var, anchor="w")
        status.pack(side=BOTTOM, fill=X)

    def bind_keys(self) -> None:
        self.root.bind("<Delete>", lambda _event: self.delete_selected())
        self.root.bind("<BackSpace>", lambda _event: self.delete_selected())
        self.root.bind("<Control-s>", lambda _event: self.save())
        self.root.bind("a", lambda _event: self.toggle_add_mode())
        self.root.bind("q", lambda _event: self.root.destroy())
        self.root.bind("r", lambda _event: self.rotate_selected(math.radians(15)))
        self.root.bind("R", lambda _event: self.rotate_selected(math.radians(-15)))

    def map_to_screen(self, x: int, y: int) -> tuple[int, int]:
        s = self.scale.get()
        return x * s, y * s

    def screen_to_map(self, sx: int, sy: int) -> tuple[int, int]:
        s = self.scale.get()
        x = max(0, min(self.map_width - 1, int(round(sx / s))))
        y = max(0, min(self.map_height - 1, int(round(sy / s))))
        return x, y

    def world_to_pixel(self, x: float, y: float) -> tuple[int, int]:
        px = int((x - self.origin[0]) / self.resolution)
        py = int(self.map_height - 1 - (y - self.origin[1]) / self.resolution)
        return px, py

    def pixel_to_world(self, px: int, py: int) -> tuple[float, float]:
        x = self.origin[0] + (px + 0.5) * self.resolution
        y = self.origin[1] + (self.map_height - py - 0.5) * self.resolution
        return x, y

    def refresh_all(self) -> None:
        self.draw()
        self.refresh_list()
        self.refresh_entries()

    def draw(self) -> None:
        s = self.scale.get()
        width = self.map_width * s
        height = self.map_height * s
        self.canvas.config(scrollregion=(0, 0, width, height), width=min(width, 900), height=min(height, 900))
        self.canvas.delete("all")

        for y in range(self.map_height):
            run_start = None
            run_color = None
            for x in range(self.map_width):
                value = self.map_data[y * self.map_width + x]
                if value > 240:
                    color = "#f4f4f4"
                elif value < 80:
                    color = "#202020"
                else:
                    color = "#9a9a9a"
                if color != run_color:
                    if run_start is not None:
                        self.canvas.create_rectangle(run_start * s, y * s, x * s, (y + 1) * s, fill=run_color, outline="")
                    run_start = x
                    run_color = color
            if run_start is not None:
                self.canvas.create_rectangle(run_start * s, y * s, self.map_width * s, (y + 1) * s, fill=run_color, outline="")

        points = [self.world_to_pixel(wp["x"], wp["y"]) for wp in self.waypoints]
        for a, b in zip(points, points[1:]):
            ax, ay = self.map_to_screen(*a)
            bx, by = self.map_to_screen(*b)
            self.canvas.create_line(ax, ay, bx, by, fill="#ff9d2e", width=2)

        for i, (wp, point) in enumerate(zip(self.waypoints, points)):
            sx, sy = self.map_to_screen(*point)
            radius = 6 if i == self.selected_index else 4
            color = "#00b050" if i == self.selected_index else "#ff6600"
            self.canvas.create_oval(sx - radius, sy - radius, sx + radius, sy + radius, fill=color, outline="#111111", width=1)
            hx = sx + math.cos(wp["yaw"]) * 18
            hy = sy - math.sin(wp["yaw"]) * 18
            self.canvas.create_line(sx, sy, hx, hy, fill="#00c040", width=2, arrow="last")
            if i % 10 == 0 or i == self.selected_index:
                self.canvas.create_text(sx + 12, sy - 10, text=str(i), fill="#cc4b00", anchor="w")

        if self.region_rect is not None:
            x0, y0, x1, y1 = self.region_rect
            sx0, sy0 = self.map_to_screen(x0, y0)
            sx1, sy1 = self.map_to_screen(x1, y1)
            self.canvas.create_rectangle(sx0, sy0, sx1, sy1, outline="#00a6ff", width=3, dash=(6, 4))

        if self.start_point is not None:
            sx, sy = self.map_to_screen(*self.start_point)
            self.canvas.create_oval(sx - 8, sy - 8, sx + 8, sy + 8, outline="#ff00ff", width=3)
            self.canvas.create_text(sx + 12, sy + 12, text="START", fill="#ff00ff", anchor="w")

    def refresh_list(self) -> None:
        self.listbox.delete(0, END)
        for i, wp in enumerate(self.waypoints):
            self.listbox.insert(END, f"{i:03d}: x={wp['x']:.2f} y={wp['y']:.2f} yaw={wp['yaw']:.2f}")
        if self.selected_index is not None and self.selected_index < len(self.waypoints):
            self.listbox.selection_set(self.selected_index)
            self.listbox.see(self.selected_index)

    def refresh_entries(self) -> None:
        for entry in (self.x_entry, self.y_entry, self.yaw_entry):
            entry.delete(0, END)
        if self.selected_index is None or self.selected_index >= len(self.waypoints):
            self.index_label.config(text="-")
            return
        wp = self.waypoints[self.selected_index]
        self.index_label.config(text=str(self.selected_index))
        self.x_entry.insert(0, f"{wp['x']:.6f}")
        self.y_entry.insert(0, f"{wp['y']:.6f}")
        self.yaw_entry.insert(0, f"{wp['yaw']:.6f}")

    def nearest_waypoint(self, sx: int, sy: int) -> int | None:
        best_index = None
        best_dist = 14 * 14
        for i, wp in enumerate(self.waypoints):
            px, py = self.world_to_pixel(wp["x"], wp["y"])
            wx, wy = self.map_to_screen(px, py)
            dist = (wx - sx) ** 2 + (wy - sy) ** 2
            if dist < best_dist:
                best_index = i
                best_dist = dist
        return best_index

    def on_click(self, event) -> None:
        self.last_mouse = (event.x, event.y)
        if self.region_mode.get():
            self.region_drag_start = self.screen_to_map(event.x, event.y)
            self.region_rect = (*self.region_drag_start, *self.region_drag_start)
            self.refresh_all()
            return
        if self.start_mode.get():
            self.start_point = self.screen_to_map(event.x, event.y)
            self.start_mode.set(0)
            self.refresh_all()
            return
        if self.add_mode.get():
            self.add_at_screen(event.x, event.y)
            return
        index = self.nearest_waypoint(event.x, event.y)
        if index is not None:
            self.selected_index = index
            self.drag_index = index
            self.refresh_all()

    def on_drag(self, event) -> None:
        if self.region_mode.get() and self.region_drag_start is not None:
            x0, y0 = self.region_drag_start
            x1, y1 = self.screen_to_map(event.x, event.y)
            self.region_rect = (min(x0, x1), min(y0, y1), max(x0, x1), max(y0, y1))
            self.draw()
            return
        if self.drag_index is None:
            return
        px, py = self.screen_to_map(event.x, event.y)
        x, y = self.pixel_to_world(px, py)
        wp = self.waypoints[self.drag_index]
        wp["x"] = x
        wp["y"] = y
        wp["grid_x"] = px
        wp["grid_y"] = py
        self.draw()
        self.refresh_entries()

    def on_release(self, _event) -> None:
        if self.region_mode.get() and self.region_drag_start is not None:
            self.region_drag_start = None
            self.region_mode.set(0)
            self.refresh_all()
            return
        self.drag_index = None
        self.refresh_all()

    def on_motion(self, event) -> None:
        px, py = self.screen_to_map(event.x, event.y)
        x, y = self.pixel_to_world(px, py)
        self.status_var.set(f"map pixel=({px}, {py}) world=({x:.3f}, {y:.3f}) points={len(self.waypoints)}")

    def on_list_select(self, _event) -> None:
        selection = self.listbox.curselection()
        if not selection:
            return
        self.selected_index = int(selection[0])
        self.refresh_all()

    def toggle_add_mode(self) -> None:
        self.add_mode.set(0 if self.add_mode.get() else 1)

    def enable_region_mode(self) -> None:
        self.region_mode.set(1)
        self.start_mode.set(0)
        self.add_mode.set(0)
        self.status_var.set("Drag a rectangle on the map to select the planning region")

    def enable_start_mode(self) -> None:
        self.start_mode.set(1)
        self.region_mode.set(0)
        self.add_mode.set(0)
        self.status_var.set("Click inside the selected region to set the start point")

    def add_at_screen(self, sx: int, sy: int) -> None:
        px, py = self.screen_to_map(sx, sy)
        x, y = self.pixel_to_world(px, py)
        yaw = self.waypoints[self.selected_index]["yaw"] if self.selected_index is not None and self.waypoints else 0.0
        waypoint = {"x": x, "y": y, "yaw": yaw, "grid_x": px, "grid_y": py}
        insert_at = len(self.waypoints) if self.selected_index is None else self.selected_index + 1
        self.waypoints.insert(insert_at, waypoint)
        self.selected_index = insert_at
        self.refresh_all()

    def insert_after_selected(self) -> None:
        if not self.waypoints:
            return
        index = self.selected_index if self.selected_index is not None else len(self.waypoints) - 1
        base = dict(self.waypoints[index])
        self.waypoints.insert(index + 1, base)
        self.selected_index = index + 1
        self.refresh_all()

    def delete_selected(self) -> None:
        if self.selected_index is None or not (0 <= self.selected_index < len(self.waypoints)):
            return
        del self.waypoints[self.selected_index]
        if not self.waypoints:
            self.selected_index = None
        else:
            self.selected_index = min(self.selected_index, len(self.waypoints) - 1)
        self.refresh_all()

    def rotate_selected(self, delta: float) -> None:
        if self.selected_index is None or not (0 <= self.selected_index < len(self.waypoints)):
            return
        self.waypoints[self.selected_index]["yaw"] = normalize_yaw(self.waypoints[self.selected_index]["yaw"] + delta)
        self.refresh_all()

    def apply_entry_values(self) -> None:
        if self.selected_index is None or not (0 <= self.selected_index < len(self.waypoints)):
            return
        try:
            x = float(self.x_entry.get())
            y = float(self.y_entry.get())
            yaw = normalize_yaw(float(self.yaw_entry.get()))
        except ValueError:
            messagebox.showerror("Invalid values", "x, y, and yaw must be numbers")
            return
        px, py = self.world_to_pixel(x, y)
        self.waypoints[self.selected_index].update({"x": x, "y": y, "yaw": yaw, "grid_x": px, "grid_y": py})
        self.refresh_all()

    def free_mask_in_region(self, clearance_px: int) -> list[list[bool]]:
        if self.region_rect is None:
            raise ValueError("Select a region first")
        x0, y0, x1, y1 = self.region_rect
        raw = [[False for _ in range(self.map_width)] for _ in range(self.map_height)]
        for y in range(y0, y1 + 1):
            for x in range(x0, x1 + 1):
                raw[y][x] = is_free_value(self.map_data[y * self.map_width + x])

        if clearance_px <= 0:
            return raw

        safe = [[False for _ in range(self.map_width)] for _ in range(self.map_height)]
        offsets = []
        for dy in range(-clearance_px, clearance_px + 1):
            for dx in range(-clearance_px, clearance_px + 1):
                if dx * dx + dy * dy <= clearance_px * clearance_px:
                    offsets.append((dx, dy))
        for y in range(y0, y1 + 1):
            for x in range(x0, x1 + 1):
                if not raw[y][x]:
                    continue
                ok = True
                for dx, dy in offsets:
                    nx, ny = x + dx, y + dy
                    if not (x0 <= nx <= x1 and y0 <= ny <= y1) or not raw[ny][nx]:
                        ok = False
                        break
                safe[y][x] = ok
        return safe

    def generate_axis_route(
        self,
        safe: list[list[bool]],
        axis: str,
        track_step: int,
        station_step: int,
        min_segment_px: int,
        start: tuple[int, int],
    ) -> list[tuple[int, int]]:
        assert self.region_rect is not None
        x0, y0, x1, y1 = self.region_rect
        scanlines = []
        if axis == "x":
            for y in range(y0, y1 + 1, track_step):
                intervals = safe_intervals([safe[y][x] for x in range(x0, x1 + 1)])
                intervals = [(a + x0, b + x0) for a, b in intervals if b - a + 1 >= min_segment_px]
                if intervals:
                    scanlines.append((y, max(intervals, key=lambda item: item[1] - item[0])))
        else:
            for x in range(x0, x1 + 1, track_step):
                intervals = safe_intervals([safe[y][x] for y in range(y0, y1 + 1)])
                intervals = [(a + y0, b + y0) for a, b in intervals if b - a + 1 >= min_segment_px]
                if intervals:
                    scanlines.append((x, max(intervals, key=lambda item: item[1] - item[0])))

        if not scanlines:
            return []

        start_coord = start[1] if axis == "x" else start[0]
        nearest_index = min(range(len(scanlines)), key=lambda i: abs(scanlines[i][0] - start_coord))
        scanlines = scanlines[nearest_index:] + list(reversed(scanlines[:nearest_index]))

        route = [start]
        previous = start
        for fixed, (lo, hi) in scanlines:
            if axis == "x":
                forward = [(x, fixed) for x in sample_segment(lo, hi, station_step)]
                backward = [(x, fixed) for x in sample_segment(hi, lo, station_step)]
            else:
                forward = [(fixed, y) for y in sample_segment(lo, hi, station_step)]
                backward = [(fixed, y) for y in sample_segment(hi, lo, station_step)]
            fdist = (forward[0][0] - previous[0]) ** 2 + (forward[0][1] - previous[1]) ** 2
            bdist = (backward[0][0] - previous[0]) ** 2 + (backward[0][1] - previous[1]) ** 2
            points = forward if fdist <= bdist else backward
            if route and route[-1] == points[0]:
                route.extend(points[1:])
            else:
                route.extend(points)
            previous = route[-1]
        return route

    def route_to_waypoints(self, route: list[tuple[int, int]]) -> list[dict[str, float]]:
        waypoints = []
        for i, point in enumerate(route):
            x, y = self.pixel_to_world(*point)
            if i + 1 < len(route):
                nx, ny = self.pixel_to_world(*route[i + 1])
                yaw = math.atan2(ny - y, nx - x)
            elif waypoints:
                yaw = waypoints[-1]["yaw"]
            else:
                yaw = 0.0
            waypoints.append({"x": x, "y": y, "yaw": normalize_yaw(yaw), "grid_x": point[0], "grid_y": point[1]})
        return waypoints

    def generate_region_lawnmower(self) -> None:
        if self.region_rect is None:
            messagebox.showerror("Missing region", "Select a region first")
            return
        if self.start_point is None:
            messagebox.showerror("Missing start", "Set a start point first")
            return
        try:
            grid_m = float(self.grid_entry.get())
            clearance_m = float(self.clearance_entry.get())
        except ValueError:
            messagebox.showerror("Invalid values", "Grid and clearance must be numbers")
            return
        step_px = max(1, int(round(grid_m / self.resolution)))
        clearance_px = max(0, int(round(clearance_m / self.resolution)))
        min_segment_px = max(1, int(round(max(0.6, grid_m * 1.5) / self.resolution)))
        safe = self.free_mask_in_region(clearance_px)
        axes = ["x", "y"] if self.axis_var.get() == "both" else [self.axis_var.get()]
        route: list[tuple[int, int]] = []
        current_start = self.start_point
        for axis in axes:
            partial = self.generate_axis_route(safe, axis, step_px, step_px, min_segment_px, current_start)
            if route and partial and route[-1] == partial[0]:
                route.extend(partial[1:])
            else:
                route.extend(partial)
            if partial:
                current_start = partial[-1]
        self.waypoints = self.route_to_waypoints(route)
        self.selected_index = 0 if self.waypoints else None
        self.refresh_all()
        self.status_var.set(f"Generated {len(self.waypoints)} waypoints in selected region")

    def save_as(self) -> None:
        filename = filedialog.asksaveasfilename(
            title="Save CSV",
            defaultextension=".csv",
            filetypes=[("CSV", "*.csv"), ("All files", "*.*")],
        )
        if not filename:
            return
        self.output_csv = Path(filename)
        self.output_json = self.output_csv.with_suffix(".json")
        self.save()

    def save(self) -> None:
        self.output_csv.parent.mkdir(parents=True, exist_ok=True)
        self.output_json.parent.mkdir(parents=True, exist_ok=True)
        with self.output_csv.open("w", newline="") as f:
            writer = csv.DictWriter(f, fieldnames=["x", "y", "yaw", "grid_x", "grid_y"])
            writer.writeheader()
            writer.writerows(self.waypoints)
        self.output_json.write_text(json.dumps({"map": str(self.map_yaml), "waypoints": self.waypoints}, indent=2))
        self.status_var.set(f"Saved {len(self.waypoints)} waypoints to {self.output_csv}")

    def run(self) -> None:
        self.root.mainloop()


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Edit generated photogrammetry paths on top of a ROS PGM map.")
    parser.add_argument("--map-yaml", type=Path, required=True)
    parser.add_argument("--path-csv", type=Path)
    parser.add_argument("--output-csv", type=Path)
    parser.add_argument("--output-json", type=Path)
    parser.add_argument("--scale", type=int, default=4)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    editor = PathEditor(args)
    editor.run()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
