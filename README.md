# Go2 Radiation Monitoring System

Autonomous environmental sensor monitoring using the Unitree Go2 quadruped robot. The robot builds a 3D LiDAR map of an environment, learns a monitoring route via manual walk-through, then autonomously navigates the route while collecting sensor data at each waypoint. Results are visualized as 2D/3D heatmaps overlaid on the LiDAR map.

![LiDAR floor plan with numbered monitoring waypoints](docs/images/floor_plan.png)

## System Architecture

**Workflow** (teach-and-repeat):

```
1. Map            2. Record Route     3. Auto-Navigate       4. Visualize
lidar_mapper.py → route_recorder.py → auto_navigator.py   → sensor_heatmap.py
(walk + scan)     (walk + mark)       (autonomous + log)     (3D/2D heatmap)
```

**Sensor data flow** (optional external sensor):

```
Arduino (serial) → sensor_bridge.py (Jetson) → TCP → auto_navigator.py (PC)
```

## Prerequisites

### Hardware
- Unitree Go2 with L1 LiDAR
- PC with Ethernet connection to robot (default interface: `enp0s2`)
- (Optional) Arduino sensor board connected to Go2's Jetson via USB

### Software
- Python 3.10 with conda (required — cyclonedds does not support Python > 3.10)
- Unitree SDK2 Python (included in this repo as `unitree_sdk2py/`)

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

### Sensor bridge deployment

If using an external Arduino sensor connected to the Go2's Jetson:

```bash
# Copy bridge script to Jetson
scp sensor_monitor/sensor_bridge.py unitree@192.168.123.18:~/

# SSH in and add user to dialout group (one-time)
ssh unitree@192.168.123.18
sudo usermod -aG dialout unitree
# Log out and back in for group change to take effect
```

## Quick Start

> **Important — shared origin:** The robot's odometry resets to (0, 0, 0) on
> every boot. All three scripts (mapping, route recording, auto-navigation)
> must use the **same physical starting position and heading**. Mark the spot
> and direction with tape before you begin. If you reboot between steps,
> re-place the robot on the mark facing the same way.

### 1. Build the LiDAR map

Place the robot on the taped starting mark (see above). Walk it slowly through the environment while the script records 3D point cloud data.

```bash
python sensor_monitor/lidar_mapper.py
```

The script continuously accumulates LiDAR scans into memory at 5 Hz (configurable
via `--throttle`). Just walk the robot around — no button presses needed during
mapping. Press `[q]` or Ctrl+C when done; the point cloud auto-saves on exit.

Generates `sensor_monitor/data/point_cloud.ply` and `sensor_monitor/data/floor_plan.png`.

### 2. Record monitoring waypoints

**Same boot session, same starting position.** Walk the robot to each monitoring location and press Enter to mark it.

```bash
python sensor_monitor/route_recorder.py
```

Press `[q]` to save and quit. Generates `sensor_monitor/data/waypoints.json`.

### 3. Verify alignment

Check that waypoints align with the LiDAR map:

```bash
python sensor_monitor/sensor_heatmap.py --cloud-only
```

The red spheres and line should overlay the point cloud correctly. If waypoints appear outside the map, re-record both in the same boot session.

### 4. Run autonomous monitoring

Place the robot on the taped starting mark (same position + heading as mapping), then:

```bash
# Without external sensor (positions only)
python sensor_monitor/auto_navigator.py

# With sensor bridge running on Jetson
python sensor_monitor/auto_navigator.py --sensor-host 192.168.123.18

# Loop continuously
python sensor_monitor/auto_navigator.py --loop
```

### 5. Visualize results

```bash
# Both 3D + 2D
python sensor_monitor/sensor_heatmap.py

# 2D heatmap only
python sensor_monitor/sensor_heatmap.py --mode 2d
```

![2D sensor heatmap overlaid on floor plan](docs/images/sensor_heatmap_2d.png)

## Script Reference

| Script | Purpose | Run on |
|--------|---------|--------|
| `lidar_mapper.py` | Walk robot to build 3D LiDAR point cloud + 2D floor plan | PC |
| `route_recorder.py` | Walk robot, press Enter to mark monitoring waypoints | PC |
| `auto_navigator.py` | Auto-navigate waypoints, collect sensor data at each stop | PC |
| `sensor_heatmap.py` | Visualize sensor readings on 3D point cloud + 2D floor plan | PC |
| `sensor_bridge.py` | Arduino serial to TCP bridge (stdlib only, no pip needed) | Jetson |

All scripts are in `sensor_monitor/` and write data to `sensor_monitor/data/`.

## Data Files

| File | Generated by | Description |
|------|-------------|-------------|
| `point_cloud.ply` | lidar_mapper.py | 3D point cloud (PLY format) |
| `floor_plan.png` | lidar_mapper.py | 2D top-down occupancy image |
| `floor_plan.npy` | lidar_mapper.py | Raw 2D occupancy grid (numpy) |
| `map_meta.json` | lidar_mapper.py | Grid metadata (resolution, origin, size) |
| `waypoints.json` | route_recorder.py | Monitoring waypoints (x, y, yaw) |
| `sensor_data.csv` | auto_navigator.py | Timestamped position + sensor readings |

Data files are large and excluded from git via `.gitignore`.

## Important Notes

- **Shared origin**: All scripts share one odometry frame anchored at the robot's boot-up position. Steps 1-2 (mapping + waypoint recording) must happen in the same boot session. Step 4 (auto-navigation) can be a later session, but the robot must start at the same taped position and heading. The navigator applies a small rigid-body correction, but large offsets will cause navigation errors.
- **Network interface**: All scripts default to `enp0s2`. Use `--interface` to change.

## Known Limitations

- Robot odometry drifts over long routes (no loop closure)
- No obstacle avoidance during autonomous navigation (uses `SportClient.Move`)
- Sensor bridge supports one TCP client at a time
- 3D visualization requires a display (X11/Wayland) — use `--mode 2d` over SSH

## Upstream SDK

This project is built on top of [`unitree_sdk2_python`](https://github.com/unitreerobotics/unitree_sdk2_python). The `unitree_sdk2py/` directory is the unmodified upstream SDK. See the upstream repo for SDK documentation, DDS setup, and low-level API examples.
