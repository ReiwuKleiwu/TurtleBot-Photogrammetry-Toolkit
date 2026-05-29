# TurtleBot Gazebo Camera Tools

This repository contains a small, portable toolchain for working with TurtleBot 4 simulations in Gazebo when you need repeatable GUI-camera screenshots from manually chosen viewpoints. It is meant for workflows where you fly through a simulated world, save the Gazebo GUI camera poses you care about, and later replay those exact poses to generate a consistent image dataset with pose metadata.

The repo provides:

- launch scripts for starting TurtleBot 4 Gazebo worlds with or without the custom WASD GUI-camera controller
- a ROS/Gazebo overlay containing the WASD camera plugin and bundled TurtleBot 4 world/model assets
- a waypoint recorder that saves the current Gazebo GUI camera pose to JSON
- a screenshot replay tool that moves the GUI camera back to recorded poses and writes PNG images, `poses.csv`, and `transforms.json`
- config files and examples for keeping local paths, maps, worlds, and GUI settings out of the scripts

The bundled defaults target the included small-house world, but the scripts are intentionally parameterized so other Gazebo worlds, model folders, and Nav2 maps can be used by changing config values or passing command-line options.

## Contents

```text
assets/maps/                         Nav2/localization maps
config/project.env.example           Example local path/config overrides
scripts/build_overlay.sh             Builds the ROS/Gazebo overlay
scripts/run_turtlebot_world.sh        Launches TurtleBot simulation
scripts/run_wasd_world.sh            Launches Gazebo with WASD GUI camera plugin
scripts/record_gui_camera_waypoints.py
scripts/capture_gui_camera_survey.py
tb4_overlay_ws/src/                  Overlay source packages, worlds, models, GUI plugin
```

Generated `tb4_overlay_ws/build`, `tb4_overlay_ws/install`, and `tb4_overlay_ws/log` directories are intentionally ignored by git.

## Requirements

- ROS 2 Jazzy installed at `/opt/ros/jazzy`
- Gazebo Harmonic / `gz` CLI
- TurtleBot 4 simulator dependencies installed on the machine
- `colcon` and normal ROS build tooling
- Python 3 with `numpy` for `capture_gui_camera_survey.py`

## Configure

The scripts read configuration from `config/project.env` by default. For a clean clone:

```bash
cp config/project.env.example config/project.env
```

All relative paths in the config are resolved from the repository root. You can also use a different config file:

```bash
CONFIG_FILE=/path/to/project.env bash scripts/run_wasd_world.sh
```

## Build

Build the overlay before launching:

```bash
bash scripts/build_overlay.sh
```

This creates `tb4_overlay_ws/install/setup.bash` and compiles the custom WASD Gazebo GUI plugin. Re-run this command after changing plugin source files.

## Launch Defaults

Launch the default TurtleBot simulation with RViz, localization, and Nav2:

```bash
bash scripts/run_turtlebot_world.sh
```

Launch the default world with WASD Gazebo GUI camera controls:

```bash
bash scripts/run_wasd_world.sh
```

## Record Camera Poses

This workflow records poses from the Gazebo GUI camera, not the TurtleBot robot camera. Use it when you want to manually fly through a world with WASD controls and save viewpoints for later screenshot capture.

1. Start Gazebo with the WASD camera plugin in one terminal:

```bash
bash scripts/run_wasd_world.sh --world small_house
```

For another world, pass its name or SDF path:

```bash
bash scripts/run_wasd_world.sh --world warehouse
bash scripts/run_wasd_world.sh --world path/to/custom_world.sdf
```

2. In a second terminal, start the pose recorder and choose the output JSON path explicitly:

```bash
python3 scripts/record_gui_camera_waypoints.py \
  --output data/my_world/waypoints.json
```

3. Keep Gazebo focused and move the GUI camera using the WASD panel controls.

4. Press `C` while Gazebo is focused to save the current camera pose.

5. Press `=` while Gazebo is focused to finish. The script writes a JSON list of saved GUI camera poses to the `--output` path.

If global X11 hotkeys are not available, use terminal mode instead:

```bash
python3 scripts/record_gui_camera_waypoints.py \
  --output data/my_world/waypoints.json \
  --terminal
```

In terminal mode, press `Enter` in the recorder terminal to save a pose, and type `q` then `Enter` to finish.

## Capture Screenshots From Recorded Poses

This workflow replays a waypoint JSON file, moves the Gazebo GUI camera to each saved pose exactly as recorded, captures screenshots, and writes pose metadata.

1. Start Gazebo with the same world used for recording through the WASD launch script:

```bash
bash scripts/run_wasd_world.sh --world small_house
```

The replay script depends on the GUI camera behavior exposed by this WASD launch/config path. Launching the normal TurtleBot simulation with `run_turtlebot_world.sh` can still show the world, but camera movement during screenshot replay may not work reliably there.

2. In a second terminal, replay the recorded poses and choose every output path explicitly:

```bash
python3 scripts/capture_gui_camera_survey.py \
  --waypoints data/my_world/waypoints.json \
  --images-dir data/my_world/screenshots/images \
  --poses-csv data/my_world/screenshots/poses.csv \
  --transforms-json data/my_world/screenshots/transforms.json
```

The outputs are:

```text
data/my_world/screenshots/images/       PNG screenshots
data/my_world/screenshots/poses.csv     pose metadata table
data/my_world/screenshots/transforms.json
```

Before each screenshot, the script asks Gazebo to move the GUI camera and waits until `/gui/camera/pose` reports the requested pose. If your machine is slow, increase the wait or settle times:

```bash
python3 scripts/capture_gui_camera_survey.py \
  --waypoints data/my_world/waypoints.json \
  --images-dir data/my_world/screenshots/images \
  --poses-csv data/my_world/screenshots/poses.csv \
  --transforms-json data/my_world/screenshots/transforms.json \
  --move-timeout-sec 6.0 \
  --settle-sec 1.0
```

If Gazebo writes screenshots somewhere unexpected, pass one or more explicit search directories:

```bash
python3 scripts/capture_gui_camera_survey.py \
  --waypoints data/my_world/waypoints.json \
  --images-dir data/my_world/screenshots/images \
  --poses-csv data/my_world/screenshots/poses.csv \
  --transforms-json data/my_world/screenshots/transforms.json \
  --screenshot-search-dir /tmp \
  --screenshot-search-dir /path/to/gazebo/screenshots
```

## Use Another Gazebo World

Worlds can be launched by name if the SDF exists under:

```text
tb4_overlay_ws/src/turtlebot4_gz_bringup/worlds/
```

Example:

```bash
bash scripts/run_wasd_world.sh --world warehouse
bash scripts/run_turtlebot_world.sh --world warehouse --localization false --nav2 false
```

You can also pass an SDF path. Relative paths are resolved from the repository root:

```bash
bash scripts/run_wasd_world.sh --world worlds/my_world.sdf
bash scripts/run_wasd_world.sh --world /absolute/path/to/my_world.sdf
```

For TurtleBot launch, `--world` is passed to `turtlebot4_gz.launch.py` as a world name, so place the world SDF in the overlay world's directory and pass the name without `.sdf`.

## Add A New World

1. Add the SDF file:

```text
tb4_overlay_ws/src/turtlebot4_gz_bringup/worlds/my_world.sdf
```

2. Add any custom models under:

```text
tb4_overlay_ws/src/turtlebot4_gz_bringup/models/
```

3. Make sure model URIs use `model://model_name/...` and that the model directory contains `model.config` and `model.sdf`.

4. Rebuild the overlay if launch/package files changed:

```bash
bash scripts/build_overlay.sh
```

5. Launch with WASD camera controls:

```bash
bash scripts/run_wasd_world.sh --world my_world
```

6. Launch with TurtleBot, if the world is compatible with TurtleBot spawning:

```bash
bash scripts/run_turtlebot_world.sh --world my_world --localization false --nav2 false
```

If you do not have a map for the new world yet, keep localization and Nav2 disabled.

## Use Another Nav2 Map

A Nav2 map normally has at least:

```text
assets/maps/my_world/map.yaml
assets/maps/my_world/map.pgm
```

The YAML should reference the image file, usually with a relative path:

```yaml
image: map.pgm
resolution: 0.05
origin: [0.0, 0.0, 0.0]
negate: 0
occupied_thresh: 0.65
free_thresh: 0.25
```

Launch TurtleBot with that map:

```bash
bash scripts/run_turtlebot_world.sh \
  --world my_world \
  --map assets/maps/my_world/map.yaml \
  --localization true \
  --nav2 true
```

Set the initial robot pose to match the map/world coordinate frame:

```bash
bash scripts/run_turtlebot_world.sh \
  --world my_world \
  --map assets/maps/my_world/map.yaml \
  --x 0.0 --y 0.0 --z 0.35 --yaw 0.0
```

## Create A Map For A New World

Typical workflow:

1. Launch the world without localization/Nav2:

```bash
bash scripts/run_turtlebot_world.sh --world my_world --localization false --nav2 false
```

2. Run SLAM using your preferred TurtleBot/Nav2 SLAM workflow.

3. Drive the robot through the environment.

4. Save the map as `map.yaml` and `map.pgm` under `assets/maps/my_world/`.

5. Relaunch with `--map assets/maps/my_world/map.yaml --localization true --nav2 true`.

## Config Defaults

The default config lives at:

```text
config/project.env
```

Useful values:

```bash
WORLD=tb4_overlay_ws/src/turtlebot4_gz_bringup/worlds/small_house.sdf
GUI_CONFIG=tb4_overlay_ws/src/turtlebot4_gz_bringup/gui/wasd_camera/gui.config
MAP_YAML=assets/maps/turtlebot3_waffle_pi/map.yaml
MODEL_PATH=tb4_overlay_ws/src/turtlebot4_gz_bringup/models
```

Use command-line arguments for one-off launches and edit `config/project.env` for local defaults.
