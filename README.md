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
                                                           --sensor {sound,radiation}
```

The system supports multiple sensor types via `--sensor`. Currently available:
- **sound** — analog sound level (0–1023 ADC), for testing without a radiation source
- **radiation** — Geiger counter (J305) outputting µSv/h dose rate

The `--sensor` flag configures labels, units, display precision, and default alert
thresholds throughout the web UI and heatmap visualizations. The data pipeline itself
is sensor-agnostic — it passes through raw numeric values regardless of sensor type.

**Gantry control** (optional vertical sensor positioning):

```
PC ──HTTP:8090──→ gantry_control.py (Jetson) ──USB-UART──→ TMC2209 → stepper motor
```

In **2D mode**, the gantry raises once at the first waypoint and stays elevated
for all measurements — fast single-height scan. In **3D mode** (default), the gantry
measures at mid-height then raises to the top at each waypoint for dual-altitude
profiling. Both modes log average + stddev.

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
scp -r unitree_sdk2py \
    sensor_monitor/dds_bridge.py \
    sensor_monitor/sensor_bridge.py \
    sensor_monitor/gantry_control.py \
    sensor_monitor/jetson_launch.sh \
    unitree@192.168.0.207:~/

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

Two Arduino sketches are provided in `sensor_monitor/arduino/`:

| Sketch | Directory | Output format | Use case |
|--------|-----------|---------------|----------|
| `sound_sensor.ino` | `arduino/sound_sensor/` | `"Sound Intensity: 512"` (0–1023) | Testing without radiation source |
| `radiation_sensor.ino` | `arduino/radiation_sensor/` | `"0.0234"` (µSv/h float) | Production radiation monitoring |

Flash the appropriate sketch to your Arduino. The sensor bridge accepts both bare
numbers (`"123"`) and prefixed lines (`"Sound Intensity: 123"`).

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

## Getting Started

> **Important — shared origin:** The robot's odometry resets to (0, 0, 0) on
> every boot. Steps 1-3 (mapping, route recording, verification) must happen in the
> **same boot session** from the **same physical starting position and heading**.
> Mark the spot and direction with tape. Step 4+ can be a later session — just
> re-place the robot on the mark facing the same way.

### 1. Start Jetson services

SSH into the Jetson and launch all bridge services:

```bash
ssh unitree@192.168.0.207
./jetson_launch.sh
```

This starts all services automatically:
- **DDS bridge** (port 9877) — robot state, commands, video, LiDAR relay
- **Sensor bridge** (port 9876) — Arduino sensor relay (skipped if no Arduino)
- **Gantry controller** (port 8090) — motor control API (skipped if no CP2102)

It auto-detects hardware, prints status, and logs to `/tmp/go2_logs/`. Press Ctrl+C to stop all.

> **Important:** All scripts (`dds_bridge.py`, `sensor_bridge.py`, `gantry_control.py`) must be
> in the **same directory** as `jetson_launch.sh` on the Jetson (typically `~/`).

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

Leave the Jetson running for all following steps.

### 2. Build the LiDAR map

On your PC, place the robot on the taped starting mark. Walk it slowly through the environment:

```bash
cd ~/Desktop/unitree_sdk2_python
conda activate go2
python sensor_monitor/lidar_mapper.py --bridge 192.168.0.207
```

Just walk the robot around — the script records 3D scans at 5 Hz automatically.
Press `[q]` or Ctrl+C when done. Generates `sensor_monitor/data/point_cloud.ply` and `floor_plan.png`.

### 3. Record monitoring waypoints

**Same boot session, same starting position.** Walk the robot to each monitoring location and press Enter to mark it.

```bash
python sensor_monitor/route_recorder.py --bridge 192.168.0.207
```

Press `[q]` to save and quit. Generates `sensor_monitor/data/waypoints.json`.

### 4. Verify alignment

Check that waypoints align with the LiDAR map (runs locally, no robot needed):

```bash
python sensor_monitor/sensor_heatmap.py --cloud-only
```

Red spheres and route line should overlay the point cloud correctly. If waypoints appear outside the map, re-record both in the same boot session.

### 5. Run autonomous patrol

Place the robot on the taped starting mark (same position + heading as mapping), then:

```bash
# Full system — radiation sensor + gantry, dual-altitude, 10s averaging:
python sensor_monitor/auto_navigator.py --web \
    --jetson 192.168.0.207 \
    --sensor radiation \
    --gantry-height 3600 \
    --measure-duration 10

# Sound sensor for testing (default if --sensor omitted):
python sensor_monitor/auto_navigator.py --web \
    --jetson 192.168.0.207 \
    --sensor sound
```

> `--jetson 192.168.0.207` is the Jetson's WiFi IP. It sets `--bridge`, `--sensor-host`,
> and `--gantry-host` to the same address in one flag.
>
> `--sensor radiation` sets the web UI title to "Radiation Patrol Monitor", units to
> µSv/h, and alert thresholds to 0.3/1.0 µSv/h. `--sensor sound` (default) uses raw
> ADC values with thresholds 300/600.
>
> `--measure-duration N` sets how long the robot dwells at each waypoint to average
> sensor readings (default 5s). Used in all modes including no-gantry. Each individual
> reading (~5Hz) is also saved to `sensor_data_raw.csv` for analysis.

**Other configurations:**

```bash
# Navigation only (test robot movement, no sensor/gantry):
python sensor_monitor/auto_navigator.py --web --jetson 192.168.0.207 --no-sensor --no-gantry

# Sensor only, no gantry (average 10s per waypoint):
python sensor_monitor/auto_navigator.py --web --jetson 192.168.0.207 --sensor radiation --no-gantry --measure-duration 10

# Gantry only, no sensor (test motor without Arduino):
python sensor_monitor/auto_navigator.py --web --jetson 192.168.0.207 --no-sensor

# Single-height 2D scan mode (faster):
python sensor_monitor/auto_navigator.py --web --jetson 192.168.0.207 --sensor radiation --measure-mode 2d

# CLI mode (no web UI):
python sensor_monitor/auto_navigator.py --bridge 192.168.0.207 --sensor radiation --no-gantry
```

> **Tip:** Add `--no-sensor` or `--no-gantry` to skip those connections.
> Individual flags (e.g., `--gantry-height 300`) work alongside `--jetson`.

**Individual IP flags** (when services are on different hosts, or for debugging):

```bash
# Equivalent to --jetson 192.168.0.207 (all three on same host):
python sensor_monitor/auto_navigator.py --web \
    --bridge 192.168.0.207 \
    --sensor-host 192.168.0.207 \
    --gantry-host 192.168.0.207

# Bridge on Jetson WiFi, sensor on Jetson ethernet (direct DDS):
python sensor_monitor/auto_navigator.py --web \
    --bridge 192.168.0.207 \
    --sensor-host 192.168.123.18

# Bridge only, no sensor or gantry:
python sensor_monitor/auto_navigator.py --web \
    --bridge 192.168.0.207 \
    --no-sensor --no-gantry
```

> `--bridge` sets the DDS bridge host (port 9877). `--sensor-host` sets the sensor
> bridge host (port 9876, default `192.168.123.18`). `--gantry-host` sets the gantry
> controller host (port 8090). `--jetson` sets all three to the same IP.

### 6. Open the Web UI

Open **http://localhost:8080** in your browser, then click **Start Patrol**.

**3D Scan mode (default):**
1. Gantry raises to mid-height before first waypoint
2. Robot navigates to waypoint → measures at mid-height for `--measure-duration` seconds
3. Gantry raises to top → measures for `--measure-duration` seconds
4. Operator clicks **Continue Patrol**
5. Gantry lowers to mid-height while robot navigates to next waypoint

**2D Scan mode** (`--measure-mode 2d`):
1. Gantry raises sensor at first waypoint
2. Robot navigates to each waypoint → measures at elevated height for `--measure-duration` seconds
3. Operator clicks **Continue Patrol** to proceed
4. Gantry lowers after last waypoint

**No-gantry mode** (`--no-gantry`):
1. Robot navigates to each waypoint → averages sensor readings for `--measure-duration` seconds
2. No gantry movement, single altitude per waypoint

After all waypoints, the robot stands down and 3D/2D visualizations auto-generate.

Web UI features:
- **Interactive floor plan** with robot position, waypoint status, zoom/pan
- **Live camera** stream
- **Status banner** showing current phase in plain English
- **Start Patrol** / **Continue** / **STOP ROBOT** controls
- **Settings drawer** (gear icon) for thresholds, scan mode, advanced options
- **3D Map** and **Heat Map** tabs with auto-generation after patrol

### 7. Visualize results (CLI alternative)

The web UI auto-generates visualizations after patrol. For CLI-only use:

```bash
python sensor_monitor/sensor_heatmap.py --sensor radiation          # 3D + 2D with radiation labels
python sensor_monitor/sensor_heatmap.py --sensor sound --mode 2d    # 2D only with sound labels
```

![2D sensor heatmap overlaid on floor plan](docs/images/sensor_heatmap_2d.png)

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
| `sensor_data.csv` | auto_navigator.py | Averaged sensor readings (1 row per waypoint per phase, with sensor_stddev) |
| `sensor_data_raw.csv` | auto_navigator.py | Raw individual readings at ~5Hz (every sample within `--measure-duration`) |
| `history_data/` | auto_navigator.py | Timestamped copies of each patrol run (CSVs + waypoints + heatmap + 3D HTML) |

Data files are large and excluded from git via `.gitignore`.

### Patrol history

Each patrol automatically saves a timestamped snapshot to `sensor_monitor/data/history_data/`:

```
YYYYMMDD_HHMMSS_sensor_data.csv          # averaged results
YYYYMMDD_HHMMSS_sensor_data_raw.csv      # raw ~5Hz readings
YYYYMMDD_HHMMSS_waypoints.json           # route used
YYYYMMDD_HHMMSS_sensor_heatmap_2d.png    # 2D visualization (after auto-generate)
YYYYMMDD_HHMMSS_sensor_3d_interactive.html  # 3D visualization (after auto-generate)
```

List past runs:

```bash
python sensor_monitor/auto_navigator.py --list-history
```

Disable history with `--no-history`.

## Testing Individual Components

You can test each Jetson service independently before running the full patrol.

### Gantry (motor control)

**On Jetson** — start the gantry controller:

```bash
ssh unitree@192.168.0.207
python3 ~/gantry_control.py --web --port /dev/ttyUSB0
```

**On PC** — test with curl:

```bash
# Check state
curl http://192.168.0.207:8090/api/state

# Set current position as home (0mm)
curl -X POST http://192.168.0.207:8090/api/home_here

# Move up 150mm
curl -X POST http://192.168.0.207:8090/api/move_to \
     -H "Content-Type: application/json" -d '{"target_mm": 150}'

# Check state again (should show moving=true, then position updating)
curl http://192.168.0.207:8090/api/state

# Move back to home
curl -X POST http://192.168.0.207:8090/api/move_to \
     -H "Content-Type: application/json" -d '{"target_mm": 0}'

# Emergency stop
curl -X POST http://192.168.0.207:8090/api/stop
```

**On PC** — test with Python (`gantry_client.py`):

```python
import sys; sys.path.insert(0, "sensor_monitor")
from gantry_client import GantryClient

c = GantryClient("192.168.0.207")
print("Reachable:", c.is_reachable())
print("State:", c.get_state())

c.home_here()
c.move_to(150)
c.wait_for_arrival(timeout=60)
print("At 150mm:", c.get_state())

c.move_to(0)
c.wait_for_arrival(timeout=60)
print("Back home:", c.get_state())
```

The gantry controller also has its own web UI at `http://192.168.0.207:8090` with
manual jog buttons and live status.

### Sensor bridge (Arduino)

**Step 1 — Check Arduino is detected on Jetson:**

```bash
ssh unitree@192.168.0.207
ls /dev/ttyACM*
# Should show /dev/ttyACM0 (or similar). If not, check USB connection.
```

**Step 2 — Test Arduino serial directly (optional):**

```bash
# Read raw serial output to confirm Arduino is sending data
cat /dev/ttyACM0
# Sound sensor: "Sound Intensity: 42" (0-1023 range)
# Radiation sensor: "0.0234" (µSv/h, bare float)
# Press Ctrl+C to stop
```

**Step 3 — Start the sensor bridge:**

```bash
python3 ~/sensor_bridge.py --port /dev/ttyACM0
```

You should see `value=...` lines printed as the Arduino sends readings.
If you see `Permission denied`, run `sudo usermod -aG dialout unitree`, log out and back in.

**Step 4 — Verify from PC over WiFi:**

```bash
# Quick test with netcat (should print sensor values, one per line)
nc 192.168.0.207 9876
```

Or with Python:

```python
import socket
s = socket.socket()
s.connect(("192.168.0.207", 9876))
for _ in range(5):
    data = s.recv(1024).decode().strip()
    print(f"Sensor reading: {data}")
s.close()
```

If this works, the sensor is ready for auto_navigator. You can now run:
```bash
# Use --sensor radiation for Geiger counter, or --sensor sound (default) for sound sensor
python sensor_monitor/auto_navigator.py --web --jetson 192.168.0.207 --sensor radiation
```

### DDS bridge (robot state + commands)

**On Jetson:**

```bash
ssh unitree@192.168.0.207
python3 ~/dds_bridge.py --interface eth0
```

**On PC** — verify state stream:

```python
import socket, json
s = socket.socket()
s.connect(("192.168.0.207", 9877))
s.sendall(json.dumps({"type": "subscribe"}).encode() + b"\n")
for _ in range(5):
    line = b""
    while not line.endswith(b"\n"):
        line += s.recv(4096)
    state = json.loads(line)
    print(f"Robot at ({state['x']:.3f}, {state['y']:.3f}), yaw={state['yaw']:.2f}")
s.close()
```

If the robot is powered on and connected via ethernet to the Jetson, you should see
position updates at ~10 Hz.

## Important Notes

- **Shared origin**: All scripts share one odometry frame anchored at the robot's boot-up position. Steps 1-2 (mapping + waypoint recording) must happen in the same boot session. Step 4 (auto-navigation) can be a later session, but the robot must start at the same taped position and heading. The navigator applies a small rigid-body correction, but large offsets will cause navigation errors.
- **WiFi vs ethernet**: Use `--jetson 192.168.0.207` (or `--bridge 192.168.0.207`) for WiFi mode (recommended). For direct ethernet without the bridge, omit `--bridge`/`--jetson` and use `--interface enp0s2`.

## Known Limitations

- Robot odometry drifts over long routes (no loop closure)
- No obstacle avoidance during autonomous navigation (uses `SportClient.Move`)
- Sensor bridge supports one TCP client at a time
- 3D visualization requires a display (X11/Wayland) — use `--mode 2d` over SSH
- WiFi video/LiDAR relay adds latency compared to direct ethernet DDS

## Upstream SDK

This project is built on top of [`unitree_sdk2_python`](https://github.com/unitreerobotics/unitree_sdk2_python). The `unitree_sdk2py/` directory is the unmodified upstream SDK. See the upstream repo for SDK documentation, DDS setup, and low-level API examples.
