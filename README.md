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
PC ──HTTP:8090──→ gantry_control.py (Jetson) ──USB-UART──→ TMC2209 → stepper motor
```

In **2D mode** (default), the gantry raises once at the first waypoint and stays elevated
for all measurements — fast single-height scan. In **3D mode**, the gantry raises/lowers
at each waypoint for dual-altitude measurement. Both modes log average + stddev.

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
- (Optional) Gantry system: TMC2209 stepper driver + NEMA17 motor + leadscrew, CP2102 USB-UART adapter connected to Jetson

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
# Copy SDK + bridge scripts to Jetson
scp -r unitree_sdk2py sensor_monitor/dds_bridge.py sensor_monitor/jetson_launch.sh unitree@192.168.0.207:~/

# SSH in, make launch script executable, and install cyclonedds
ssh unitree@192.168.0.207
chmod +x ~/jetson_launch.sh
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
# Copy bridge script to Jetson (if not already deployed via jetson_launch.sh setup)
scp sensor_monitor/sensor_bridge.py unitree@192.168.0.207:~/

# SSH in and add user to dialout group (one-time)
ssh unitree@192.168.0.207
sudo usermod -aG dialout unitree
# Log out and back in for group change to take effect
```

The Arduino sketch should print sensor values via `Serial.println()`. Both bare
numbers (`"123"`) and prefixed lines (`"Sound Intensity: 123"`) are supported.

### Gantry deployment

If using the TMC2209 gantry for dual-altitude measurements:

```bash
# Copy gantry controller to Jetson
scp sensor_monitor/gantry_control.py unitree@192.168.0.207:~/

# SSH in and install pyserial
ssh unitree@192.168.0.207
pip3 install pyserial

# Verify CP2102 adapter is detected
ls /dev/ttyUSB*
```

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

> **Tip:** For the full E2E flow (sensor + gantry), use `jetson_launch.sh` instead —
> it starts all Jetson services in one command. See [Running the Full System](#running-the-full-system-e2e).

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

# With sensor + gantry (2D mode — default, single elevated height):
python sensor_monitor/auto_navigator.py --web \
    --bridge 192.168.0.207 \
    --sensor-host 192.168.0.207 \
    --gantry-host 192.168.0.207 \
    --gantry-height 150 \
    --measure-duration 5

# With sensor + gantry (3D mode — dual altitude, low + high per waypoint):
python sensor_monitor/auto_navigator.py --web \
    --bridge 192.168.0.207 \
    --sensor-host 192.168.0.207 \
    --gantry-host 192.168.0.207 \
    --gantry-height 150 \
    --measure-duration 5 \
    --measure-mode 3d
```

> **Note:** `--sensor-host` must be the Jetson's **WiFi IP** (not the internal
> `192.168.123.18`). All three services (DDS bridge, sensor bridge, gantry) use
> the same WiFi IP.

Open `http://localhost:8080` in your browser. The web UI provides:

- **Interactive floor plan** with robot position, waypoint status, zoom/pan (scroll + drag)
- **Live camera** stream from the robot's front camera
- **2D/3D scan mode selector** — choose before starting patrol
- **Activity banner** showing current phase with countdown timers
- **Start Patrol** / **Next Point** / **Emergency Stop** controls
- **Gantry status** with height bar and direction indicator
- **Sensor readings** with user-configurable safe/warning/danger thresholds
- **3D/2D visualization** tabs with auto-generation after patrol completes

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
the full patrol system with sensor + gantry. You need **1 SSH terminal to the Jetson**
and **1 terminal on your PC**.

### Jetson — Launch all services

```bash
ssh unitree@192.168.0.207
./jetson_launch.sh
```

This single script starts all three Jetson services:
- **DDS bridge** (port 9877) — robot state + commands + video + LiDAR relay
- **Sensor bridge** (port 9876) — Arduino sensor data relay (skipped if no Arduino detected)
- **Gantry controller** (port 8090) — stepper motor HTTP API (skipped if no CP2102 detected)

It auto-detects hardware (`/dev/ttyACM*` for Arduino, `/dev/ttyUSB*` for CP2102),
prints status for each service, and logs output to `/tmp/go2_logs/`. Press Ctrl+C to stop all.

<details>
<summary>Manual alternative (3 separate terminals)</summary>

```bash
# Terminal 1: DDS bridge
python3 ~/dds_bridge.py --interface eth0 --video --lidar

# Terminal 2: Sensor bridge (if Arduino connected)
python3 ~/sensor_bridge.py --port /dev/ttyACM0

# Terminal 3: Gantry controller (if CP2102 connected)
python3 ~/gantry_control.py --web --port /dev/ttyUSB0
```

</details>

### PC — Auto Navigator + Web UI

Place the robot at its starting mark, then:

```bash
cd ~/Desktop/unitree_sdk2_python
conda activate go2

python sensor_monitor/auto_navigator.py --web \
    --bridge 192.168.0.207 \
    --sensor-host 192.168.0.207 \
    --gantry-host 192.168.0.207 \
    --gantry-height 150 \
    --measure-duration 5
```

### Browser

Open **http://localhost:8080**, select scan mode (2D or 3D), and click **Start Patrol**.

**2D Scan mode (default):**
1. Gantry raises sensor at first waypoint
2. Robot navigates to each waypoint → measures for 5s at elevated height
3. Operator clicks **Next Point** to proceed
4. Gantry lowers after last waypoint

**3D Scan mode** (`--measure-mode 3d`):
1. Robot navigates to waypoint
2. Sensor measures at ground level for 5s
3. Gantry raises → sensor measures at elevated position for 5s
4. Operator clicks **Next Point**
5. Gantry lowers while robot navigates to next waypoint

After all waypoints, the gantry returns home, the robot stands down, and 3D/2D visualizations are auto-generated.

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
| `gantry_control.py` | TMC2209 stepper motor controller with FastAPI (HTTP:8090) | Jetson |
| `gantry_client.py` | HTTP client for gantry_control.py (used by auto_navigator) | PC |
| `wifi_bridge_client.py` | TCP client for dds_bridge (SportClient-compatible interface) | PC |
| `web_server.py` | FastAPI backend for web UI (used internally by `--web` flag) | PC |
| `sensor_surface_sphere_overlay.py` | 3D/2D sensor visualization generator (Plotly + matplotlib) | PC |
| `jetson_launch.sh` | Single launcher for all Jetson services (DDS + sensor + gantry) | Jetson |

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
