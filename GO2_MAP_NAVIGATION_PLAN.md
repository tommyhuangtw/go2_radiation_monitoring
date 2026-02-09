# Go2 Patrol & 3D Sensor Mapping System

## Overview

A "teach and repeat" patrol system for the Unitree Go2 robot that:
1. Records a patrol route by manually walking the robot (with LiDAR 3D mapping)
2. Auto-navigates the same route on subsequent runs
3. Collects sensor data (radiation/temperature via Arduino on Jetson) at each waypoint
4. Visualizes sensor readings overlaid on the 3D point cloud map + 2D heatmap

**Why this approach**: No public API exists to load the Unitree App's saved 3D map. "Teach and repeat" with the Python SDK is the simplest effective approach.

**Constraint**: Robot odometry resets to (0,0,0) on boot — must start from the same physical spot each time (marked on floor).

---

## Current Status

| Step | Script | Status |
|------|--------|--------|
| 1a — Record waypoints | `patrol_system/route_recorder.py` | Done |
| 1b — Build 3D LiDAR map | `patrol_system/lidar_mapper.py` | Done |
| 2 — Auto-navigate + sensor logging | `patrol_system/patrol_navigator.py` | Done |
| 3 — 3D/2D visualization | `patrol_system/sensor_heatmap.py` | Code ready — needs re-recorded aligned data (see [Known Bug](#known-bug-coordinate-misalignment-in-sensor_heatmappy)) |
| 4 — Arduino TCP bridge (stdlib only) | `patrol_system/sensor_bridge.py` | Done (untested on hardware) |
| Stretch — Camera color enhancement | — | Not started |

**Data collected** (in `patrol_system/data/`):
- `point_cloud.ply` — 3D point cloud from LiDAR
- `waypoints.json` — 10 recorded waypoints (recorded in a separate boot session from LiDAR)
- `floor_plan.npy` / `floor_plan.png` — 2D occupancy grid (from LiDAR session, shows 12 older waypoints)
- `map_meta.json` — grid metadata (resolution=0.05 m/pixel, 249x232 grid)
- `patrol_data.csv` — 10 sensor readings from patrol run (values ~1021-1023)

---

## Architecture

### System Diagram

```
Step 1a: RECORD WAYPOINTS               Step 1b: BUILD 3D MAP
┌───────────────────────────┐            ┌───────────────────────────┐
│ route_recorder.py         │            │ lidar_mapper.py           │
│ Walk robot via app        │            │ Walk robot via app        │
│ Press Enter at key spots  │            │ LiDAR scans accumulate    │
│ ↓                         │            │ ↓                         │
│ waypoints.json            │            │ point_cloud.ply           │
└───────────────────────────┘            │ floor_plan.npy/.png       │
                                         │ map_meta.json             │
                                         └───────────────────────────┘

  !! IMPORTANT: Steps 1a and 1b MUST run in the SAME boot session    !!
  !! so that waypoints and point cloud share the same odom frame.     !!

Step 2: PATROL (every run)               Step 3: VISUALIZE
┌───────────────────────────┐            ┌───────────────────────────┐
│ patrol_navigator.py       │            │ sensor_heatmap.py         │
│ Load waypoints.json       │            │                           │
│ Rigid-body transform to   │            │ --cloud-only: 3D viewer   │
│   current odom frame      │            │ --mode 3d: cloud+sensors  │
│ Auto-navigate each point  │            │ --mode 2d: floor+heatmap  │
│ Read sensor at each stop  │            │ --mode both: all views    │
│ ↓                         │            │                           │
│ patrol_data.csv           │            │ Aligns patrol_data to map │
│ (positions in patrol      │            │ frame via waypoints.json  │
│  session's odom frame)    │            │ waypoint index snapping   │
└───────────────────────────┘            └───────────────────────────┘
```

### Sensor Data Flow (Arduino → Jetson → PC → Visualization)

```
┌──────────────┐    USB Serial     ┌──────────────────────────────┐
│   Arduino    │ ──────────────→   │  Jetson (192.168.123.18)     │
│ (sensor hw)  │  "1023\n"         │                              │
│              │  9600 baud        │  sensor_bridge.py            │
└──────────────┘                   │  - reads /dev/ttyACM0 via    │
                                   │    stdlib termios (no pyserial│
                                   │  - parses numeric lines       │
                                   │  - TCP server on port 9876   │
                                   │  - sends JSON: {"value":X,   │
                                   │    "ts":Y}\n                  │
                                   └───────────┬──────────────────┘
                                               │ TCP (LAN)
                                               ▼
                                   ┌──────────────────────────────┐
                                   │  Host PC (your computer)     │
                                   │                              │
                                   │  patrol_navigator.py         │
                                   │  - TCP client connects to    │
                                   │    192.168.123.18:9876       │
                                   │  - background thread reads   │
                                   │    JSON lines continuously   │
                                   │  - stores latest value in    │
                                   │    self.latest_sensor_value  │
                                   │  - at each waypoint stop:    │
                                   │    logs (x, y, yaw, value)   │
                                   │    to patrol_data.csv        │
                                   └───────────┬──────────────────┘
                                               │ CSV file
                                               ▼
                                   ┌──────────────────────────────┐
                                   │  sensor_heatmap.py           │
                                   │  - loads patrol_data.csv     │
                                   │  - aligns to map frame via   │
                                   │    waypoints.json snapping   │
                                   │  - 3D: colored spheres on    │
                                   │    point cloud               │
                                   │  - 2D: heatmap on floor plan │
                                   └──────────────────────────────┘
```

---

## Detailed Script Review

### 1. `route_recorder.py` — Waypoint Recording (Step 1a)

**Purpose**: Walk the robot manually via the Unitree App joystick while this script displays the live odometry position. Press Enter at key locations to record waypoints.

**How it works**:
- Subscribes to `rt/sportmodestate` via DDS to get `(x, y, yaw)` at ~100Hz
- Interactive terminal loop with non-blocking `select.select()` input
- Commands: Enter=record waypoint, `s`=save, `p`=print, `c`=clear, `q`=quit
- Saves waypoints as `[{x, y, yaw}, ...]` JSON array to `data/waypoints.json`
- Backs up existing file as `.bak` before overwriting

**Output**: `waypoints.json` — array of `{x, y, yaw}` in the robot's odom frame for that boot session.

### 2. `lidar_mapper.py` — LiDAR 3D Map Builder (Step 1b)

**Purpose**: Walk the robot manually while accumulating LiDAR point cloud scans into a dense 3D map. Generates a PLY point cloud and a 2D floor plan.

**How it works**:
- Subscribes to `rt/utlidar/cloud_deskewed` (PointCloud2_) for LiDAR data at ~10Hz
- Subscribes to `rt/sportmodestate` for position display
- Parses PointCloud2 messages: extracts x/y/z float32 fields from raw bytes using numpy
- Filters invalid points: `np.isfinite()` AND `dist_sq > 0.01` (rejects zero-points)
- Throttles to configurable Hz (default 10), downsamples every Nth point (default 2)
- Accumulates all valid points in a list of Nx3 numpy arrays
- On save:
  - Writes ASCII PLY file (`point_cloud.ply`)
  - Builds 2D occupancy grid by projecting points in height range [z_min, z_max] onto XY plane
  - Saves floor plan as PNG (matplotlib) and NPY (raw grid)
  - Overlays waypoints from `waypoints.json` on floor plan if available
  - Saves grid metadata (resolution, origin, dimensions) to `map_meta.json`

**Output**: `point_cloud.ply`, `floor_plan.npy`, `floor_plan.png`, `map_meta.json`

### 3. `patrol_navigator.py` — Auto-Navigation + Sensor Logging (Step 2)

**Purpose**: Load recorded waypoints and autonomously navigate the robot through the patrol route, logging sensor data at each stop.

**How it works**:
- **Initialization**:
  - Subscribes to `rt/sportmodestate` for position tracking
  - Starts a background TCP client thread connecting to sensor bridge on Jetson
  - Creates a `SportClient` for movement commands
- **Waypoint alignment** (critical for cross-session use):
  - Loads `waypoints.json` (recorded in mapping session)
  - Applies a rigid-body transform (translate + rotate) to align recorded waypoints to the current boot session's odom frame
  - Uses the first waypoint as reference: computes yaw offset between current heading and recorded heading, then rotates + translates all waypoints
- **Navigation** (two-phase per waypoint):
  1. **Rotate**: Turn to face target using bang-bang yaw control (vyaw = +/-0.8 rad/s), with stuck detection (5s no-progress skip)
  2. **Move forward**: P-control steering with `vx = min(speed, dist*0.8)` and `vyaw = clamp(yaw_error*0.8, -0.3, 0.3)`, with stuck detection (6s of <5cm movement)
- **Sensor logging**: After arriving at each waypoint, pauses 1s then reads `latest_sensor_value` from the TCP background thread
- **Cleanup**: StopMove → RecoveryStand → StandDown → save CSV

**Sensor TCP client** (background thread):
- Connects to `sensor_host:sensor_port` (default `192.168.123.18:9876`)
- Reads newline-delimited JSON `{"value": X, "ts": Y}`
- Stores `latest_sensor_value` and `latest_sensor_ts` for main thread to read
- Auto-reconnects on disconnect (3s retry)

**Output**: `patrol_data.csv` with columns: `timestamp, waypoint, x, y, z, yaw, sensor_value`

### 4. `sensor_bridge.py` — Arduino Serial → TCP Bridge (Jetson)

**Purpose**: Runs on the Go2's Jetson board. Reads sensor values from Arduino via USB serial and serves them over TCP to the host PC.

**How it works**:
- **Serial**: Opens `/dev/ttyACM0` (or auto-detected) using `os.open()` + `termios` — zero external dependencies
  - Configures 8N1, no flow control, raw mode at specified baud rate
  - Reads line-buffered from the file descriptor
- **Parsing**: Accepts bare numbers (`"1023"`) or prefixed lines (`"Sound Intensity: 1023"`)
- **TCP server**: Listens on `0.0.0.0:9876`, accepts one client at a time
  - Background thread handles `accept()` via `select.select()`
  - Main thread reads serial lines, parses values, sends JSON to connected client
  - Format: `{"value": 1023.0, "ts": 1770637912.9}\n`
- **Graceful handling**: Client disconnect detected via `BrokenPipeError`, old client replaced on new connection

**Deployment**: `scp` to Jetson, run with `python3 ~/sensor_bridge.py --port /dev/ttyACM0`

### 5. `sensor_heatmap.py` — Visualization (Step 3) [PARTIALLY BROKEN]

**Purpose**: Visualize sensor readings overlaid on the 3D point cloud (open3d) and 2D floor plan (matplotlib).

**How it works**:
- **Data loading**: Reads `patrol_data.csv`, then "aligns" positions to the map frame by snapping each waypoint's (x, y) to the corresponding entry in `waypoints.json` (by waypoint index)
- **3D mode**: Loads `point_cloud.ply`, colors by height (blue→green→red), adds colored spheres at sensor positions with vertical stems, patrol path as red line set
- **2D mode**: Loads `floor_plan.npy` as background, overlays sensor readings as colored scatter points with colorbar
- **Cloud-only mode**: Just the point cloud with waypoint markers (no sensor data needed)

**Current bug**: See [Known Bug section](#known-bug-coordinate-misalignment-in-sensor_heatmappy) below.

---

## File Reference

### patrol_system/ (main scripts)

| File | Purpose | Run on |
|------|---------|--------|
| `route_recorder.py` | Walk robot, press Enter to mark waypoints → `waypoints.json` | PC |
| `lidar_mapper.py` | Walk robot, accumulate LiDAR → `point_cloud.ply` + floor plan | PC |
| `patrol_navigator.py` | Auto-navigate waypoints, log sensor data → `patrol_data.csv` | PC |
| `sensor_heatmap.py` | 3D point cloud viewer + sensor overlay + 2D heatmap | PC |
| `sensor_bridge.py` | Arduino serial → TCP bridge (stdlib only) | Jetson |

### my_scripts/ (earlier prototypes, kept for reference)

| File | What it was |
|------|-------------|
| `position_recorder.py` | Original waypoint recorder (position subscriber pattern) |
| `test_auto_navigate_abs_position.py` | Navigation prototype (rotate + move + stuck detection) |
| `test_auto_navigate.py` | Earlier nav attempt |
| `test_basic_control_action.py` | Low-level motor control tests |
| `test_move.py` | Simple movement tests |
| `test_navigation.py` | Extended navigation tests |
| `test_position.py` | Position reading tests |

### patrol_system/data/ (generated outputs)

| File | From | Contents |
|------|------|----------|
| `waypoints.json` | route_recorder | 10 waypoints `{x, y, yaw}` — X: 0.04-2.27m, Y: -0.97-0.64m |
| `point_cloud.ply` | lidar_mapper | 3D point cloud, full room structure |
| `floor_plan.npy` | lidar_mapper | 2D occupancy grid (numpy, 249x232) |
| `floor_plan.png` | lidar_mapper | Visual floor plan image (shows 12 older waypoints) |
| `map_meta.json` | lidar_mapper | Grid: 0.05 m/pixel, origin=(-0.05, -6.09), 249x232 |
| `patrol_data.csv` | patrol_navigator | 10 rows: `timestamp, waypoint, x, y, z, yaw, sensor_value` |
| `misalignment_diagnostic.png` | diagnostic | Shows the waypoint vs map misalignment bug |

---

## How to Run

### One-time setup (TEACH)
1. Mark a starting position on the floor (tape/sticker)
2. Place robot at the mark, boot it up
3. Record waypoints:
   ```bash
   python patrol_system/route_recorder.py
   # Walk via app joystick, press Enter at key positions, press 's' to save
   ```
4. Build 3D map (**same boot session** — do NOT reboot between steps 3 and 4):
   ```bash
   python patrol_system/lidar_mapper.py
   # Walk the same route, press 's' to save, 'q' to quit
   ```
5. **Verify alignment** (critical — do this before any patrol run):
   ```bash
   python patrol_system/sensor_heatmap.py --cloud-only
   ```
   Check that the red waypoint markers appear **inside the building** in the 3D viewer.
   If you see a `WARNING: Waypoints appear OUTSIDE the LiDAR map!` message, the data
   is misaligned — re-record both steps 3 and 4 in the same boot session.

### Each patrol run (REPEAT)
1. Place robot at the same marked starting position
2. (If using real sensor) Start bridge on Jetson:
   ```bash
   ssh unitree@192.168.123.18 "python3 ~/sensor_bridge.py --port /dev/ttyACM0"
   ```
   Verify with: `nc 192.168.123.18 9876` (should see JSON lines)
3. Run patrol (use `--sensor-host` / `--sensor-port` if non-default):
   ```bash
   python patrol_system/patrol_navigator.py
   ```
4. Visualize results:
   ```bash
   python patrol_system/sensor_heatmap.py --cloud-only    # just the 3D map
   python patrol_system/sensor_heatmap.py --mode 3d       # map + sensor spheres
   python patrol_system/sensor_heatmap.py --mode 2d       # 2D floor plan heatmap
   python patrol_system/sensor_heatmap.py --mode both     # everything (default)
   # Use --vmin/--vmax for custom color scale (e.g. full ADC range):
   python patrol_system/sensor_heatmap.py --mode 2d --vmin 0 --vmax 1023
   ```

### Deploy sensor bridge to Jetson (once)
```bash
# Ensure the unitree user can access serial ports
ssh unitree@192.168.123.18 "sudo usermod -aG dialout unitree"
# Re-login required after adding to dialout group
scp patrol_system/sensor_bridge.py unitree@192.168.123.18:~/
```

---

## Recording Best Practices

### The Same-Session Rule

**`route_recorder.py` and `lidar_mapper.py` MUST run in the same boot session.** The Go2's odometry
resets to (0,0,0) on every boot, so data from different boots uses incompatible coordinate frames.

`sensor_heatmap.py` now warns automatically if it detects misalignment (waypoints landing on empty
areas of the floor plan grid), but it's best to get it right during recording.

### Recommended Recording Workflow

```
1. Mark starting position on the floor (tape/sticker)
2. Boot the robot, place at the mark
3. DO NOT REBOOT from this point until step 7
4. Run route_recorder.py:
   - Walk the patrol route via App joystick
   - Press Enter at each key location to record waypoints
   - Press 's' to save, then 'q' to quit
5. Run lidar_mapper.py:
   - Walk the same route (or broader area) via App joystick
   - LiDAR scans accumulate automatically
   - Press 's' to save, then 'q' to quit
6. Verify alignment immediately:
   python patrol_system/sensor_heatmap.py --cloud-only
   → Red waypoint dots should appear INSIDE the building
   → No "WARNING: Waypoints appear OUTSIDE the LiDAR map!" message
7. If verification passes, recording is done. Safe to reboot now.
```

### Common Pitfalls

| Mistake | Symptom | Fix |
|---------|---------|-----|
| Rebooted between steps 4 and 5 | Waypoints outside map in visualization | Re-record both in same session |
| Only re-ran route_recorder, kept old point cloud | Same as above | Re-record both |
| Stale `floor_plan.png` shows old waypoints | PNG has wrong number/position of waypoint dots | Re-run lidar_mapper (it regenerates PNG) |
| Forgot to save in route_recorder (`s` key) | `waypoints.json` missing or empty | Re-run route_recorder, remember to press `s` |
| Robot moved before script started | First waypoint far from origin | Use `route_recorder.py` first while robot is still at start mark |

---

## Known Bug: Coordinate Misalignment in sensor_heatmap.py

### Problem

The patrol route appears **outside the LiDAR-mapped building** in the 2D heatmap and 3D visualization.

![Misalignment Diagnostic](patrol_system/data/misalignment_diagnostic.png)

### What's happening

The current `waypoints.json` (10 waypoints, X: 0.04-2.27m) and `point_cloud.ply` (occupied region X: 0.45-11.85m) were recorded in **different boot sessions**. Since the Go2's odometry resets to (0,0,0) on every boot, these files are in **incompatible coordinate frames**:

| Data | Odom Frame | X range | Y range |
|------|-----------|---------|---------|
| `waypoints.json` (10 pts) | Session B | 0.04 - 2.27 | -0.97 - 0.64 |
| `point_cloud.ply` / `floor_plan` | Session A | 0.45 - 11.85 | -5.59 - 4.96 |

The `floor_plan.png` still shows 12 older waypoints (from session A) that correctly align with the building, but these were overwritten in `waypoints.json` by the 10 new waypoints from session B.

`sensor_heatmap.py` snaps `patrol_data.csv` positions to `waypoints.json` coordinates (line 80-84), which puts them in session B's frame — still misaligned with the session A point cloud.

### Root cause

Steps 1a (route_recorder) and 1b (lidar_mapper) were run in **different boot sessions**, violating the requirement that they share the same odom frame.

### Code-level detection (added)

`sensor_heatmap.py` now checks whether waypoints land on occupied cells in the floor plan grid. If <30% of waypoints are near any LiDAR data, it prints:
```
WARNING: Waypoints appear OUTSIDE the LiDAR map!
    Only 0/10 waypoints are near occupied cells.
    ...
```

### Fix needed

**Re-record**: Re-run both `route_recorder.py` and `lidar_mapper.py` in the **same boot session** so all data shares one odom frame. See [Recording Best Practices](#recording-best-practices) for the step-by-step workflow.

### Current heatmap output (showing the bug)

The sensor heatmap with misaligned data — waypoints clustered at bottom-left, building structure at center-right:

![Sensor Heatmap with Bug](patrol_system/data/sensor_heatmap_2d.png)

---

## Key Technical Details

### SDK Patterns
- Network init: `ChannelFactoryInitialize(0, "enp0s2")` (ethernet to robot)
- State: `ChannelSubscriber("rt/sportmodestate", SportModeState_)` → `msg.position[0/1]`, `msg.imu_state.rpy[2]`
- LiDAR: `ChannelSubscriber("rt/utlidar/cloud_deskewed", PointCloud2_)` → points in odom frame, ~10Hz
- Movement: `SportClient.Move(vx, vy, vyaw)`, `SportClient.StandUp()`, `SportClient.StandDown()`

### PointCloud2 Parsing
```python
point_step = msg.point_step
raw = bytes(msg.data)
n_points = len(raw) // point_step
buf = np.frombuffer(raw, np.uint8).reshape(n_points, point_step)
# Extract xyz at field offsets (typically 0, 4, 8) as float32
```
- Filter `(0,0,0)` points (invalid/no-return sentinels) with `dist_sq > 0.01`

### Navigation Algorithm (patrol_navigator.py)
Two-phase approach per waypoint:
1. **Rotate**: Bang-bang yaw control at +/-0.8 rad/s, tolerance=0.25 rad (~14 deg). Stuck detection: skip if no progress in 5s.
2. **Move forward**: P-control with `vx = clamp(dist*0.8, 0.15, 0.3)`, `vyaw = clamp(yaw_error*0.8, -0.3, 0.3)`. Stuck detection: accept position if <5cm movement over 6s.

Cross-session alignment: On startup, computes rigid-body transform from first recorded waypoint to current position (yaw offset + rotate all waypoints around first point).

### open3d 0.18.0 Workarounds (Ubuntu 24.04)
These functions segfault and must be avoided:
- `paint_uniform_color()` → use `np.tile()` + manual `.colors` assignment
- `create_coordinate_frame()` → skip
- `create_sphere()` → use Fibonacci-sphere PointCloud helper (`_make_sphere_pcd()`)
- `Vector3dVector(list)` / `Vector2iVector(list)` → always pass numpy arrays, never Python lists

### 3D Visualization
- Point cloud colored by height: blue (floor) → green (walls) → red (ceiling)
- Normals estimated for Phong shading (`estimate_normals` with KDTree hybrid search)
- Sensor readings shown as colored PointCloud spheres (avoids mesh segfaults)

### Sensor Bridge Protocol
- Arduino → Jetson: USB serial at 9600 baud, one numeric value per line (e.g. `"1023\n"` or `"Sound Intensity: 1023\n"`)
- Jetson → PC: TCP on port 9876, newline-delimited JSON: `{"value": 1023.0, "ts": 1770637912.9}\n`
- `patrol_navigator.py` TCP client: background thread, auto-reconnect every 3s, stores latest value for main thread

---

## Dependencies

```bash
conda activate go2  # Python 3.10
pip install numpy matplotlib open3d
# opencv-python needed only for stretch goal (camera color)

# sensor_bridge.py on Jetson: stdlib only (no pip install needed)
```

---

## Known Limitations
- Robot odometry resets on boot → must start from same physical spot
- **Steps 1a and 1b MUST be done in the same boot session** (or data will be misaligned)
- No public API to load the Unitree App's saved 3D map
- `TrajectoryFollow` (API 1018) not available in Python SDK for Go2 (B2 only)
- open3d 0.18.0 has multiple segfault bugs (see workarounds above)
- Sensor bridge untested on actual Jetson hardware
- Jetson user must be in `dialout` group for serial access (`sudo usermod -aG dialout unitree`)
- Navigation tolerance is ~25cm per waypoint; accuracy depends on floor surface and odometry drift

---

## Stretch Goal: Camera Color Enhancement

Not yet started. Would add RGB color from the Go2's front camera to the point cloud by projecting LiDAR points onto camera frames during recording.

**SDK support**: `VideoClient` at `unitree_sdk2py/go2/video/video_client.py`
**Limitation**: Only front-facing points get color; sides/rear remain height-colored.
