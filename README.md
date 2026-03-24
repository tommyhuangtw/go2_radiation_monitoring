# Go2 Radiation Monitoring System

Autonomous environmental sensor monitoring using the Unitree Go2 quadruped robot. The robot builds a 3D LiDAR map of an environment, learns a monitoring route via manual walk-through, then autonomously navigates the route while collecting sensor data at each waypoint. Results are visualized as 2D/3D heatmaps overlaid on the LiDAR map.

![LiDAR floor plan with numbered monitoring waypoints](docs/images/floor_plan.png)

## System Architecture

**Workflow** (teach-and-repeat):

```
1. Map            2. Record Route     3. Auto-Navigate       4. Visualize
lidar_mapper.py → route_recorder.py → auto_navigator.py   → sensor_heatmap.py
(walk + scan)     (walk + mark)       (autonomous + log)     (3D/2D heatmap)
                                      ↓
                                      auto_navigator.py --web
                                      (web UI + live camera)
```

**Sensor data flow** (optional external sensor):

```
Arduino (serial) → sensor_bridge.py (Jetson) → TCP:9876 → auto_navigator.py (PC)
```

**Gantry control** (optional vertical sensor positioning):

```
PC ──HTTP:8090──→ gantry_control.py (Jetson) ──USB Serial──→ Arduino Mega ──A4988──→ stepper motor
```

At each waypoint, the gantry raises/lowers the sensor for dual-altitude measurement
(ground level + elevated), then logs average + stddev at each height.

**WiFi mode** (no ethernet cable needed):

```
PC ──WiFi/TCP──→ dds_bridge.py (Jetson) ──DDS/eth0──→ Robot
                 port 9877: commands + state (10Hz)
                 port 9878: video stream (JPEG via DDS VideoClient)
                 port 9879: LiDAR point clouds
```

## Prerequisites

### Hardware
- Unitree Go2 with L1 LiDAR
- PC on the same WiFi network as the robot
- (Optional) Arduino sensor board connected to Go2's Jetson via USB
- (Optional) Gantry system: Arduino Mega + A4988 stepper driver + NEMA17 motor + leadscrew, connected to Jetson via USB

### Software
- Python 3.10 with conda (required — cyclonedds does not support Python > 3.10)
- Unitree SDK2 Python (included in this repo as `unitree_sdk2py/`)

### Connecting to Jetson Nano via WiFi

The Go2's onboard Jetson Nano is accessible over WiFi once the robot is powered on
and connected to your WiFi network.

| Network | IP | Purpose |
|---------|-----|---------|
| WiFi (same as your PC) | `192.168.0.207` | SSH access, bridge TCP ports |
| Internal ethernet (to robot) | `192.168.123.18` | DDS communication with robot |

```bash
# SSH into the Jetson (password: 123)
ssh unitree@192.168.0.207
```

## Installation

### SDK setup

```bash
git clone git@github.com:tommyhuangtw/go2_radiation_monitoring.git
cd go2_radiation_monitoring

# Create conda env (cyclonedds requires Python ≤ 3.10)
conda create -n go2 python=3.10 -y
conda activate go2

# Install SDK + dependencies
pip install -r requirements.txt
pip install -e .
```

### Troubleshooting: cyclonedds

If `pip install -e .` fails with:

```
Could not locate cyclonedds. Try to set CYCLONEDDS_HOME or CMAKE_PREFIX_PATH
```

Build cyclonedds from source first:

```bash
cd ~
git clone https://github.com/eclipse-cyclonedds/cyclonedds -b releases/0.10.x
cd cyclonedds && mkdir build install && cd build
cmake .. -DCMAKE_INSTALL_PREFIX=../install
cmake --build . --target install
```

Then re-install with the path set:

```bash
cd ~/go2_radiation_monitoring
export CYCLONEDDS_HOME="~/cyclonedds/install"
pip install -e .
```

See [cyclonedds PyPI docs](https://pypi.org/project/cyclonedds/#installing-with-pre-built-binaries) for details.

### WiFi bridge deployment

The DDS protocol used by the Unitree SDK only works over ethernet. `dds_bridge.py`
runs on the Jetson and bridges DDS to TCP, so all PC scripts can work over WiFi.

**One-time setup:**

```bash
# Copy SDK + bridge script to Jetson
scp -r unitree_sdk2py sensor_monitor/dds_bridge.py unitree@192.168.0.207:~/

# SSH in and install cyclonedds
ssh unitree@192.168.0.207
pip3 install cyclonedds==0.10.2
```

**Start the bridge (run each session):**

```bash
ssh unitree@192.168.0.207
python3 ~/dds_bridge.py --interface eth0 --video --lidar
```

This starts all three relay services:
- **Port 9877** — robot state (10 Hz) + commands (move, standup, etc.)
- **Port 9878** — camera video stream (JPEG frames via DDS VideoClient)
- **Port 9879** — LiDAR point cloud stream

You can omit `--video` or `--lidar` if you don't need camera or LiDAR relay.

**Safety:** The bridge has a 250ms deadman timeout — if no move command arrives
(e.g. WiFi drops), the robot stops automatically. It also stops on client
disconnect and when a new client connects.

### Sensor bridge deployment

If using an external Arduino sensor connected to the Go2's Jetson:

```bash
# Copy bridge script to Jetson
scp sensor_monitor/sensor_bridge.py unitree@192.168.0.207:~/

# SSH in and add user to dialout group (one-time)
ssh unitree@192.168.0.207
sudo usermod -aG dialout unitree
# Log out and back in for group change to take effect
```

The Arduino sketch should print sensor values via `Serial.println()`. Both bare
numbers (`"123"`) and prefixed lines (`"Sound Intensity: 123"`) are supported.

### Gantry deployment

The gantry uses an Arduino Mega + A4988 stepper driver for vertical sensor positioning.
The Arduino handles step generation and position tracking; the Jetson sends commands
over USB serial.

**Arduino setup:**

1. Open `sensor_monitor/arduino/gantry_arduino_control.ino` in the Arduino IDE
2. Select **Board: Arduino Mega 2560** and the correct port
3. Upload the sketch
4. Verify in Serial Monitor (115200 baud): you should see `INFO A4988_GANTRY_READY`

**Jetson setup:**

```bash
# Copy gantry controller to Jetson
scp sensor_monitor/gantry_control.py unitree@192.168.0.207:~/

# SSH in and install dependencies
ssh unitree@192.168.0.207
pip3 install pyserial fastapi uvicorn

# Verify Arduino is detected (Arduino Mega shows as /dev/ttyACM*)
ls /dev/ttyACM*

# If permission denied:
sudo usermod -aG dialout unitree
# Log out and back in
```

**Test the gantry standalone:**

```bash
# Auto-detect port (tries each /dev/ttyACM* until it finds the gantry Arduino):
python3 ~/gantry_control.py

# Or specify port explicitly:
python3 ~/gantry_control.py --port /dev/ttyACM0

# Commands: home (set home), 9 (move to top), 0 (move to bottom), state, stop, q

# Web UI test (also auto-detects port):
python3 ~/gantry_control.py --web
# Open http://192.168.0.207:8090 — click "Set Home Here", then use Move/Jog buttons
```

> **Port auto-detection:** When `--port` is omitted, the script tries each
> `/dev/ttyACM*` port and identifies the gantry Arduino by its `A4988_GANTRY_READY`
> boot message. This handles the case where the port number changes between reboots
> (e.g. `/dev/ttyACM0` vs `/dev/ttyACM1` when multiple Arduinos are connected).
> Use `--port` to skip auto-detection if you know the port.

The gantry auto-homes on startup (assumes sensor arm starts at bottom position).
Sensor arm range: **40 cm** (home) to **120 cm** (top) from the ground. Internal
motor travel is 0–9 mm mapped to this physical range.

## Quick Start

> **Important — shared origin:** The robot's odometry resets to (0, 0, 0) on
> every boot. All three scripts (mapping, route recording, auto-navigation)
> must use the **same physical starting position and heading**. Mark the spot
> and direction with tape before you begin. If you reboot between steps,
> re-place the robot on the mark facing the same way.

> **WiFi assumed:** The commands below use WiFi mode (`--bridge`) with
> `dds_bridge.py` running on the Jetson (see [WiFi bridge deployment](#wifi-bridge-deployment)).
> For direct ethernet, omit `--bridge` and use `--interface enp0s2`.

### 0. Start the bridge on Jetson

```bash
ssh unitree@192.168.0.207
python3 ~/dds_bridge.py --interface eth0 --video --lidar
```

Leave this running for the entire session. All steps below connect to it.

### 1. Build the LiDAR map

Place the robot on the taped starting mark (see above). Walk it slowly through the environment while the script records 3D point cloud data.

```bash
python sensor_monitor/lidar_mapper.py --bridge 192.168.0.207
```

The script continuously accumulates LiDAR scans into memory at 5 Hz (configurable
via `--throttle`). Just walk the robot around — no button presses needed during
mapping. Press `[q]` or Ctrl+C when done; the point cloud auto-saves on exit.

Generates `sensor_monitor/data/point_cloud.ply` and `sensor_monitor/data/floor_plan.png`.

### 2. Record monitoring waypoints

**Same boot session, same starting position.** Walk the robot to each monitoring location and press Enter to mark it.

```bash
python sensor_monitor/route_recorder.py --bridge 192.168.0.207
```

Press `[q]` to save and quit. Generates `sensor_monitor/data/waypoints.json`.

### 3. Verify alignment

Check that waypoints align with the LiDAR map:

```bash
python sensor_monitor/sensor_heatmap.py --cloud-only
```

This runs locally on PC (no robot connection needed). The red spheres and line should overlay the point cloud correctly. If waypoints appear outside the map, re-record both in the same boot session.

### 4a. Run autonomous monitoring (CLI)

Place the robot on the taped starting mark (same position + heading as mapping), then:

```bash
python sensor_monitor/auto_navigator.py --bridge 192.168.0.207 --no-sensor
```

Remove `--no-sensor` if an Arduino sensor is connected. Add `--loop` for continuous patrol.

### 4b. Run autonomous monitoring (Web UI)

```bash
# Without sensor/gantry:
python sensor_monitor/auto_navigator.py --web --bridge 192.168.0.207 --no-sensor

# With sensor only:
python sensor_monitor/auto_navigator.py --web \
    --bridge 192.168.0.207 \
    --sensor-host 192.168.0.207

# With sensor + gantry (dual-altitude measurement):
python sensor_monitor/auto_navigator.py --web \
    --jetson 192.168.0.207 \
    --gantry-height 9 \
    --measure-duration 5
```

> **Note:** `--sensor-host` must be the Jetson's **WiFi IP** (not the internal
> `192.168.123.18`). All three services (DDS bridge, sensor bridge, gantry) use
> the same WiFi IP.

Open `http://localhost:8080` in your browser. The web UI provides:

- **Live floor plan** with robot position and waypoint status
- **Live camera** stream from the robot's front camera
- **Activity banner** showing current phase with countdown timers
- **Step pipeline** showing per-waypoint cycle progress (Navigate → Measure Low → Gantry Up → Measure High → Next)
- **Start Patrol** / **Next Point** / **Emergency Stop** controls
- **Sensor arm status** with height bar (40–120 cm) and direction indicator
- **Dual-altitude readings** (Low/High columns with avg and std dev) in waypoint table
- **Live sensor data** with user-configurable safe/warning/danger thresholds
- **3D/2D visualization** generation from collected data

### 5. Visualize results

```bash
# Both 3D + 2D
python sensor_monitor/sensor_heatmap.py

# 2D heatmap only
python sensor_monitor/sensor_heatmap.py --mode 2d
```

![2D sensor heatmap overlaid on floor plan](docs/images/sensor_heatmap_2d.png)

## Running the Full System (E2E)

Once mapping and waypoint recording are done, here's the complete procedure to run
the full patrol system with sensor + gantry. You need **3 SSH terminals to the Jetson**
and **1 terminal on your PC**.

### Jetson — Terminal 1: DDS Bridge

```bash
ssh unitree@192.168.0.207
python3 ~/dds_bridge.py --interface eth0 --video
```

### Jetson — Terminal 2: Sensor Bridge

```bash
ssh unitree@192.168.0.207
python3 ~/sensor_bridge.py --port /dev/ttyACM0
```

Verify you see `value=...` lines before proceeding.

### Jetson — Terminal 3: Gantry Controller

```bash
ssh unitree@192.168.0.207
python3 ~/gantry_control.py --web
```

The script auto-detects the gantry Arduino (identifies it by its `A4988_GANTRY_READY`
boot message, so it won't accidentally connect to the sensor Arduino). If auto-detect
fails, specify the port explicitly: `--port /dev/ttyACM0` or `--port /dev/ttyACM1`.

Verify with `curl http://192.168.0.207:8090/api/state` from PC. You should see
`"connected": true` and `"homed": true` (auto-homed on startup).

### PC — Auto Navigator + Web UI

Place the robot at its starting mark, then:

```bash
cd ~/Desktop/unitree_sdk2_python
conda activate go2

# Full system (sensor + gantry):
python sensor_monitor/auto_navigator.py --web \
    --jetson 192.168.0.207 \
    --gantry-height 9 \
    --measure-duration 5

# Or without sensor:
python sensor_monitor/auto_navigator.py --web \
    --bridge 192.168.0.207 \
    --gantry-host 192.168.0.207 \
    --gantry-height 9 \
    --measure-duration 5 \
    --no-sensor
```

`--jetson 192.168.0.207` is shorthand for `--bridge`, `--sensor-host`, and
`--gantry-host` all pointing to the same Jetson IP. `--gantry-height 9` is
the full motor travel (sensor arm from 40 cm to 120 cm).

### Browser

Open **http://localhost:8080** and click **Start Patrol**.

The patrol cycle at each waypoint:
1. Robot navigates to waypoint
2. Sensor measures at ground level (40 cm) for 5s (avg + stddev logged)
3. Sensor arm raises to 120 cm
4. Sensor measures at elevated position for 5s
5. UI waits for operator to click **Next Point**
6. Sensor arm lowers while robot navigates to next waypoint

After all waypoints, the sensor arm returns home and the robot stands down.

## Script Reference

| Script | Purpose | Run on |
|--------|---------|--------|
| `lidar_mapper.py` | Walk robot to build 3D LiDAR point cloud + 2D floor plan | PC |
| `route_recorder.py` | Walk robot, press Enter to mark monitoring waypoints | PC |
| `auto_navigator.py` | Auto-navigate waypoints, collect sensor data at each stop | PC |
| `auto_navigator.py --web` | Web UI with live map, camera, controls, and heatmap | PC |
| `sensor_heatmap.py` | Visualize sensor readings on 3D point cloud + 2D floor plan | PC |
| `sensor_bridge.py` | Arduino serial to TCP bridge (stdlib only, no pip needed) | Jetson |
| `dds_bridge.py` | DDS-to-TCP bridge for WiFi control, video relay, LiDAR relay | Jetson |
| `gantry_control.py` | Arduino gantry controller with FastAPI (HTTP:8090) | Jetson |
| `gantry_client.py` | HTTP client for gantry_control.py (used by auto_navigator) | PC |
| `wifi_bridge_client.py` | TCP client for dds_bridge (SportClient-compatible interface) | PC |
| `web_server.py` | FastAPI backend for web UI (used internally by `--web` flag) | PC |

All scripts are in `sensor_monitor/` and write data to `sensor_monitor/data/`.

## Data Files

| File | Generated by | Description |
|------|-------------|-------------|
| `point_cloud.ply` | lidar_mapper.py | 3D point cloud (PLY format) |
| `floor_plan.png` | lidar_mapper.py | 2D top-down occupancy image |
| `floor_plan.npy` | lidar_mapper.py | Raw 2D occupancy grid (numpy) |
| `map_meta.json` | lidar_mapper.py | Grid metadata (resolution, origin, size) |
| `waypoints.json` | route_recorder.py | Monitoring waypoints (x, y, yaw) |
| `sensor_data.csv` | auto_navigator.py | Timestamped position + sensor readings (with altitude_mm, measurement_phase, sensor_stddev for dual-altitude mode) |

Data files are large and excluded from git via `.gitignore`.

## Important Notes

- **Shared origin**: All scripts share one odometry frame anchored at the robot's boot-up position. Steps 1-2 (mapping + waypoint recording) must happen in the same boot session. Step 4 (auto-navigation) can be a later session, but the robot must start at the same taped position and heading. The navigator applies a small rigid-body correction, but large offsets will cause navigation errors.
- **WiFi vs ethernet**: Use `--bridge 192.168.0.207` for WiFi mode (recommended). For direct ethernet without the bridge, omit `--bridge` and use `--interface enp0s2`.

## Known Limitations

- Robot odometry drifts over long routes (no loop closure)
- No obstacle avoidance during autonomous navigation (uses `SportClient.Move`)
- Sensor bridge supports one TCP client at a time
- 3D visualization requires a display (X11/Wayland) — use `--mode 2d` over SSH
- WiFi video/LiDAR relay adds latency compared to direct ethernet DDS

## Upstream SDK

This project is built on top of [`unitree_sdk2_python`](https://github.com/unitreerobotics/unitree_sdk2_python). The `unitree_sdk2py/` directory is the unmodified upstream SDK. See the upstream repo for SDK documentation, DDS setup, and low-level API examples.
