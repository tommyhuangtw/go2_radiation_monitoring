"""
Auto Navigator with Sensor Logging (Step 2)

Loads waypoints from route_recorder.py output and auto-navigates the robot
through the monitoring route, collecting sensor data at each waypoint.

Usage:
    python sensor_monitor/auto_navigator.py
    python sensor_monitor/auto_navigator.py --web          # Launch web UI
    python sensor_monitor/auto_navigator.py --web --web-port 8080

    # WiFi mode (requires dds_bridge.py running on Jetson):
    python sensor_monitor/auto_navigator.py --bridge 192.168.0.207

Prerequisites:
    - waypoints.json in sensor_monitor/data/ (from route_recorder.py)
    - Robot placed at the same starting position as during recording
    - (Optional) sensor_bridge.py running on Jetson for sensor data
    - (WiFi mode) dds_bridge.py running on Jetson
"""

import sys
import os
import time
import math
import json
import csv
import socket
import threading
import shutil
import statistics
from datetime import datetime

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelSubscriber
from unitree_sdk2py.go2.sport.sport_client import SportClient
from unitree_sdk2py.idl.unitree_go.msg.dds_ import SportModeState_

DATA_DIR = os.path.join(os.path.dirname(__file__), "data")


class AutoNavigator:
    def __init__(self, network_interface="enp0s2",
                 sensor_host="192.168.123.18", sensor_port=9876,
                 bridge_host=None, bridge_port=9877,
                 no_sensor=False,
                 save_history=True,
                 rotate_tolerance=0.05, move_tolerance=0.05,
                 gantry_host=None, gantry_port=8090,
                 gantry_height=1500.0, measure_duration=5.0,
                 measure_mode="2d"):
        self._bridge_mode = bridge_host is not None
        self._bridge_client = None

        # Navigation tolerances
        self.rotate_tolerance = rotate_tolerance
        self.move_tolerance = move_tolerance

        # Position tracking
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0
        self.state_ready = False

        # Sensor data (from TCP bridge on Jetson)
        self.latest_sensor_value = None
        self.latest_sensor_ts = 0.0
        self.sensor_connected = False

        # Sensor data log
        self.sensor_log = []
        self.save_history = save_history

        # Navigation state (for web mode)
        self.nav_state = "idle"          # idle|standing_up|navigating|collecting|collecting_low|gantry_raising|collecting_high|waiting|done|error
        self.current_waypoint_idx = 0    # 0-based index into waypoints_adjusted
        self.total_waypoints = 0
        self.waypoints_adjusted = []     # populated by prepare_patrol()
        self.waypoint_statuses = []      # per-waypoint status strings
        self.patrol_error = ""

        # Web mode synchronization
        self._continue_event = threading.Event()
        self._stop_event = threading.Event()
        self._web_mode = False

        # Gantry control (optional — runs on Jetson, accessed via HTTP)
        self.gantry_client = None
        self.gantry_height = float(gantry_height)
        self.measure_duration = float(measure_duration)
        self.measure_mode = measure_mode  # "2d" or "3d"
        if gantry_host:
            from gantry_client import GantryClient
            self.gantry_client = GantryClient(gantry_host, gantry_port)
            print(f"Gantry client: {gantry_host}:{gantry_port}")

        if self._bridge_mode:
            # WiFi mode: connect to DDS bridge on Jetson
            from wifi_bridge_client import WiFiBridgeClient
            print(f"Connecting to DDS bridge at {bridge_host}:{bridge_port}...")
            self._bridge_client = WiFiBridgeClient(bridge_host, bridge_port)
            self._bridge_client.connect()
            self.sport_client = self._bridge_client  # same interface
            print("Bridge connected!")
        else:
            # Ethernet mode: direct DDS
            ChannelFactoryInitialize(0, network_interface)

            # Subscribe to state
            self.sub = ChannelSubscriber("rt/sportmodestate", SportModeState_)
            self.sub.Init(self._on_state, 10)

            # Sport Client
            self.sport_client = SportClient()
            self.sport_client.SetTimeout(10.0)
            self.sport_client.Init()

        # Start sensor TCP client thread
        self._no_sensor = no_sensor
        self._sensor_host = sensor_host
        self._sensor_port = sensor_port
        if not no_sensor:
            self._sensor_thread = threading.Thread(
                target=self._sensor_tcp_loop, daemon=True)
            self._sensor_thread.start()

        # Wait for state
        print("Waiting for state updates...")
        t0 = time.time()
        while not self.state_ready and time.time() - t0 < 5.0:
            if self._bridge_mode:
                # Poll bridge client for state
                self.state_ready = self._bridge_client.state_ready
                if self.state_ready:
                    x, y, yaw = self._bridge_client.get_position()
                    self.current_x, self.current_y, self.current_yaw = x, y, yaw
            time.sleep(0.1)
        if self.state_ready:
            print(f"State ready! Position: ({self.current_x:.2f}, {self.current_y:.2f}), "
                  f"yaw={math.degrees(self.current_yaw):.1f}")
        else:
            print("WARNING: State update timeout! Robot may not be connected.")

        # Start bridge state polling thread (WiFi mode)
        if self._bridge_mode:
            self._bridge_poll_thread = threading.Thread(
                target=self._bridge_state_poll, daemon=True)
            self._bridge_poll_thread.start()

        # Wait for sensor connection (up to 8s to cover connect timeout + retry)
        if no_sensor:
            print("Sensor disabled (--no-sensor).")
        elif not self.sensor_connected:
            print(f"Waiting for sensor connection ({self._sensor_host}:{self._sensor_port})...")
            t0 = time.time()
            while not self.sensor_connected and time.time() - t0 < 8.0:
                time.sleep(0.2)
            if self.sensor_connected:
                print(f"Sensor connected!")
            else:
                print(f"WARNING: Sensor not connected. Values will be N/A.")

        # --- System status banner ---
        self._print_system_status()

    def _print_system_status(self):
        """Print a loud summary of all subsystem connections."""
        print("\n" + "=" * 60)
        print("  SYSTEM STATUS")
        print("=" * 60)

        # DDS / Bridge
        if self._bridge_mode:
            status = "CONNECTED" if self.state_ready else "*** NOT READY ***"
            host = self._bridge_client._host if hasattr(self._bridge_client, '_host') else "bridge"
            print(f"  DDS Bridge:  {status}")
        else:
            status = "CONNECTED" if self.state_ready else "*** NOT READY ***"
            print(f"  DDS Direct:  {status}")

        # Sensor
        if self._no_sensor:
            print(f"  Sensor:      DISABLED (--no-sensor)")
        elif self.sensor_connected:
            print(f"  Sensor:      CONNECTED ({self._sensor_host}:{self._sensor_port})")
        else:
            print(f"  Sensor:      *** NOT CONNECTED *** ({self._sensor_host}:{self._sensor_port})")
            print(f"               Make sure: python3 ~/sensor_bridge.py --port /dev/ttyACM0")

        # Gantry
        if self.gantry_client:
            if self.gantry_client.is_reachable():
                print(f"  Gantry:      CONNECTED ({self.gantry_client.base_url})")
            else:
                print(f"  Gantry:      *** NOT REACHABLE *** ({self.gantry_client.base_url})")
                print(f"               Make sure: python3 ~/gantry_control.py --web --port /dev/ttyUSB0")
        else:
            print(f"  Gantry:      NOT CONFIGURED (no --gantry-host)")

        # Measure mode
        mode_label = "2D (single elevated height)" if self.measure_mode == "2d" else "3D (dual altitude)"
        print(f"  Mode:        {mode_label}")

        print("=" * 60 + "\n")

    def _on_state(self, msg):
        self.current_x = msg.position[0]
        self.current_y = msg.position[1]
        self.current_yaw = msg.imu_state.rpy[2]
        self.state_ready = True

    def _bridge_state_poll(self):
        """Poll bridge client for state updates (WiFi mode)."""
        while self._bridge_mode and self._bridge_client._connected:
            x, y, yaw = self._bridge_client.get_position()
            self.current_x = x
            self.current_y = y
            self.current_yaw = yaw
            self.state_ready = self._bridge_client.state_ready
            time.sleep(0.02)  # 50Hz polling

    def _sensor_tcp_loop(self):
        """Background thread: connect to sensor bridge via TCP, read JSON lines."""
        while True:
            try:
                sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                sock.settimeout(5.0)
                sock.connect((self._sensor_host, self._sensor_port))
                sock.settimeout(None)
                print(f"Sensor TCP connected to {self._sensor_host}:{self._sensor_port}")
                self.sensor_connected = True
                buf = b""
                while True:
                    chunk = sock.recv(4096)
                    if not chunk:
                        break
                    buf += chunk
                    while b"\n" in buf:
                        line, buf = buf.split(b"\n", 1)
                        try:
                            data = json.loads(line.decode('utf-8', errors='ignore'))
                            self.latest_sensor_value = data.get("value", 0.0)
                            self.latest_sensor_ts = time.time()
                        except (json.JSONDecodeError, ValueError):
                            pass
            except (ConnectionRefusedError, socket.timeout, OSError) as e:
                print(f"Sensor TCP connect failed: {e} — retrying in 3s...")
            finally:
                self.sensor_connected = False
                try:
                    sock.close()
                except OSError:
                    pass
            time.sleep(3)  # retry after 3 seconds

    def get_position(self):
        return self.current_x, self.current_y, self.current_yaw

    def normalize_angle(self, angle):
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle

    def distance_to(self, target_x, target_y):
        x, y, _ = self.get_position()
        return math.sqrt((target_x - x)**2 + (target_y - y)**2)

    def angle_to(self, target_x, target_y):
        x, y, _ = self.get_position()
        return math.atan2(target_y - y, target_x - x)

    def _is_stopped(self):
        """Check if emergency stop was requested."""
        return self._stop_event.is_set()

    def rotate_to_target(self, target_x, target_y, tolerance=0.05, timeout=30.0):
        """Rotate to face target point using P-control with damping."""
        print("  Rotating to face target...")

        target_yaw = self.angle_to(target_x, target_y)
        print(f"    [Rotate] Target heading: {math.degrees(target_yaw):.1f}")

        start_time = time.time()
        iteration = 0
        last_progress_check_time = start_time
        last_yaw_error = None
        prev_yaw_error = None

        while True:
            if self._is_stopped():
                self.sport_client.StopMove()
                return

            _, _, yaw = self.get_position()
            yaw_error = self.normalize_angle(target_yaw - yaw)

            if iteration % 50 == 0:
                print(f"    [Rotate] yaw={math.degrees(yaw):.1f}, "
                      f"target={math.degrees(target_yaw):.1f}, "
                      f"error={math.degrees(yaw_error):.1f}")

            if abs(yaw_error) < tolerance:
                print(f"    [Rotate] Complete! Error: {math.degrees(yaw_error):.1f}")
                break

            # Stuck detection
            if time.time() - last_progress_check_time > 5.0:
                if last_yaw_error is not None and abs(abs(yaw_error) - abs(last_yaw_error)) < 0.02:
                    print(f"    [Rotate] No progress, skipping (error: {math.degrees(yaw_error):.1f})")
                    break
                last_yaw_error = abs(yaw_error)
                last_progress_check_time = time.time()

            if time.time() - start_time > timeout:
                print(f"    [Rotate] Timeout! Error: {math.degrees(yaw_error):.1f}")
                break

            # PD control: proportional + derivative damping
            Kp = 2.0
            Kd = 0.3
            vyaw = Kp * yaw_error
            if prev_yaw_error is not None:
                d_error = (yaw_error - prev_yaw_error) / 0.02
                vyaw += Kd * d_error
            prev_yaw_error = yaw_error
            # Clamp to safe range
            vyaw = max(min(vyaw, 0.8), -0.8)
            # Dead zone: stop if error is tiny to avoid jitter
            if abs(yaw_error) < 0.02:
                vyaw = 0
            self.sport_client.Move(0, 0, vyaw)
            time.sleep(0.02)
            iteration += 1

        self.sport_client.StopMove()
        time.sleep(0.1)

    def move_forward_to_target(self, target_x, target_y, tolerance=0.05, speed=0.3, timeout=60.0):
        """Move forward to target with PD-control steering and smooth deceleration."""
        print("  Moving forward...")

        start_time = time.time()
        iteration = 0
        last_x, last_y = None, None
        last_progress_time = None
        stuck_counter = 0
        prev_yaw_error = None

        while self.distance_to(target_x, target_y) > tolerance:
            if self._is_stopped():
                self.sport_client.StopMove()
                return

            x, y, yaw = self.get_position()
            distance = self.distance_to(target_x, target_y)

            if iteration % 50 == 0:
                print(f"    [Move] pos=({x:.2f}, {y:.2f}), dist={distance:.3f}m")

            if time.time() - start_time > timeout:
                print(f"    [Move] Timeout! Distance: {distance:.3f}m")
                break

            # Stuck detection: check every 2s if robot moved at least 2cm
            if last_x is None:
                last_x, last_y = x, y
                last_progress_time = time.time()
            elif time.time() - last_progress_time > 2.0:
                position_change = math.sqrt((x - last_x)**2 + (y - last_y)**2)
                if position_change < 0.02:
                    stuck_counter += 1
                    if stuck_counter >= 3:  # 6 seconds of no real progress
                        print(f"    [Move] Stuck, accepting position (dist: {distance:.3f}m)")
                        break
                else:
                    stuck_counter = 0
                last_x, last_y = x, y
                last_progress_time = time.time()

            target_yaw = self.angle_to(target_x, target_y)
            yaw_error = self.normalize_angle(target_yaw - yaw)

            # Smooth deceleration: slow down proportionally as we approach
            vx = min(speed, distance * 1.0)
            vx = max(vx, 0.08)
            # Extra slow for final approach
            if distance < 0.15:
                vx = min(vx, 0.08)

            # PD steering control
            Kp_steer = 1.2
            Kd_steer = 0.2
            vyaw = Kp_steer * yaw_error
            if prev_yaw_error is not None:
                d_error = (yaw_error - prev_yaw_error) / 0.02
                vyaw += Kd_steer * d_error
            prev_yaw_error = yaw_error
            vyaw = max(min(vyaw, 0.4), -0.4)

            self.sport_client.Move(vx, 0, vyaw)
            time.sleep(0.02)
            iteration += 1

        self.sport_client.StopMove()
        print(f"    [Move] Complete! Final dist: {self.distance_to(target_x, target_y):.3f}m")

    def move_to_point(self, target_x, target_y, rotate_tolerance=0.05, move_tolerance=0.05):
        """Two-phase navigation: rotate then move forward."""
        x, y, _ = self.get_position()
        print(f"  Current: ({x:.2f}, {y:.2f}), Target: ({target_x:.2f}, {target_y:.2f})")

        if self.distance_to(target_x, target_y) < move_tolerance:
            print("  Already at target!")
            return

        self.rotate_to_target(target_x, target_y, tolerance=rotate_tolerance)
        if self._is_stopped():
            return
        self.move_forward_to_target(target_x, target_y, tolerance=move_tolerance)
        if not self._is_stopped():
            print("  Arrived!")

    def read_sensor(self):
        """Read the latest sensor value. Returns float or None if unavailable."""
        if self.sensor_connected and self.latest_sensor_value is not None:
            # Only use if fresh (within last 5 seconds)
            if time.time() - self.latest_sensor_ts < 5.0:
                return self.latest_sensor_value
        return None

    def log_data_point(self, waypoint_idx):
        """Log current position + sensor value (single-measurement, no gantry)."""
        x, y, yaw = self.get_position()
        val = self.read_sensor()

        entry = {
            "timestamp": time.time(),
            "waypoint": waypoint_idx,
            "x": round(x, 4),
            "y": round(y, 4),
            "z": 0.0,
            "yaw": round(yaw, 4),
            "sensor_value": val if val is not None else "",
            "sensor_stddev": "",
            "altitude_mm": 0,
            "measurement_phase": "",
        }
        self.sensor_log.append(entry)

        v_str = f"{val}" if val is not None else "N/A"
        print(f"    Logged: pos=({x:.3f}, {y:.3f}), sensor={v_str}")

    def _collect_sensor_averaged(self, waypoint_idx, duration=5.0, altitude_mm=0, phase="low"):
        """Collect sensor readings over `duration` seconds, log the average and stddev."""
        readings = []
        t0 = time.time()
        while time.time() - t0 < duration:
            if self._is_stopped():
                break
            val = self.read_sensor()
            if val is not None:
                readings.append(val)
            time.sleep(0.2)  # ~5Hz sampling

        x, y, yaw = self.get_position()
        if readings:
            avg = statistics.mean(readings)
            std = statistics.stdev(readings) if len(readings) > 1 else 0.0
        else:
            avg = None
            std = None

        entry = {
            "timestamp": time.time(),
            "waypoint": waypoint_idx,
            "x": round(x, 4),
            "y": round(y, 4),
            "z": 0.0,
            "yaw": round(yaw, 4),
            "sensor_value": round(avg, 2) if avg is not None else "",
            "sensor_stddev": round(std, 2) if std is not None else "",
            "altitude_mm": altitude_mm,
            "measurement_phase": phase,
        }
        self.sensor_log.append(entry)

        v_str = f"{avg:.1f} (std={std:.1f}, n={len(readings)})" if avg is not None else "N/A"
        print(f"    Logged [{phase}@{altitude_mm}mm]: pos=({x:.3f}, {y:.3f}), sensor={v_str}")

    def _ensure_gantry_home(self, timeout=60.0):
        """Block until gantry is at home position (0mm). Send move_to(0) if needed."""
        if not self.gantry_client:
            return
        if self.gantry_client.is_at_home():
            return
        # Gantry might still be lowering from previous waypoint — wait
        print("    Waiting for gantry to reach home...")
        if not self.gantry_client.wait_for_arrival(timeout=timeout):
            # Still not at home — force it
            print("    Gantry not at home, forcing move to 0mm...")
            self.gantry_client.move_to(0)
            if not self.gantry_client.wait_for_arrival(timeout=timeout):
                raise RuntimeError("Gantry failed to reach home position")

    def save_sensor_data(self):
        """Save sensor log to CSV."""
        if not self.sensor_log:
            print("No sensor data to save.")
            return

        os.makedirs(DATA_DIR, exist_ok=True)
        csv_path = os.path.join(DATA_DIR, "sensor_data.csv")

        fieldnames = ["timestamp", "waypoint", "x", "y", "z", "yaw", "sensor_value",
                      "sensor_stddev", "altitude_mm", "measurement_phase"]
        with open(csv_path, 'w', newline='') as f:
            writer = csv.DictWriter(f, fieldnames=fieldnames)
            writer.writeheader()
            writer.writerows(self.sensor_log)

        print(f"Saved {len(self.sensor_log)} data points to {csv_path}")

        if not self.save_history:
            return

        # Keep a per-run historical snapshot with timestamp.
        history_dir = os.path.join(DATA_DIR, "history_data")
        os.makedirs(history_dir, exist_ok=True)
        ts = datetime.now().strftime("%Y%m%d_%H%M%S")
        history_path = os.path.join(history_dir, f"{ts}_sensor_data.csv")
        try:
            shutil.copy2(csv_path, history_path)
            print(f"Saved history snapshot to {history_path}")
        except OSError as e:
            print(f"WARNING: Failed to save history snapshot: {e}")

    def load_waypoints(self):
        """Load waypoints from JSON file."""
        wp_path = os.path.join(DATA_DIR, "waypoints.json")
        if not os.path.exists(wp_path):
            print(f"ERROR: {wp_path} not found!")
            print("Run route_recorder.py first to create waypoints.")
            return None

        with open(wp_path, 'r') as f:
            wp_data = json.load(f)

        waypoints = []
        for wp in wp_data:
            waypoints.append((wp["x"], wp["y"], wp["yaw"]))

        print(f"Loaded {len(waypoints)} waypoints from {wp_path}")
        return waypoints

    # ------------------------------------------------------------------
    # Web-mode control methods
    # ------------------------------------------------------------------

    def web_continue(self):
        """Signal navigation to proceed to next waypoint (replaces Enter key)."""
        self._continue_event.set()

    def web_emergency_stop(self):
        """Emergency stop: halt movement immediately."""
        self._stop_event.set()
        self.sport_client.StopMove()
        if self.gantry_client:
            try:
                self.gantry_client.stop()
            except Exception:
                pass
        self.nav_state = "idle"

    def get_state_snapshot(self):
        """Return current state as a dict for WebSocket broadcast."""
        gantry_state = None
        if self.gantry_client:
            try:
                gantry_state = self.gantry_client.get_state()
            except Exception:
                gantry_state = {"connected": False, "status": "unreachable"}
        return {
            "nav_state": self.nav_state,
            "position": {
                "x": round(self.current_x, 4),
                "y": round(self.current_y, 4),
                "yaw": round(self.current_yaw, 4),
            },
            "current_waypoint": self.current_waypoint_idx,
            "total_waypoints": self.total_waypoints,
            "waypoint_statuses": list(self.waypoint_statuses),
            "sensor_value": self.latest_sensor_value,
            "sensor_connected": self.sensor_connected,
            "sensor_log": list(self.sensor_log),
            "error": self.patrol_error,
            "gantry": gantry_state,
            "measure_mode": self.measure_mode,
        }

    # ------------------------------------------------------------------
    # Patrol logic (factored for both CLI and web mode)
    # ------------------------------------------------------------------

    def prepare_patrol(self):
        """Load waypoints, apply rigid-body transform. Returns True on success."""
        waypoints = self.load_waypoints()
        if waypoints is None or len(waypoints) == 0:
            self.patrol_error = "No waypoints found"
            return False

        self.total_waypoints = len(waypoints)
        self.waypoint_statuses = ["pending"] * len(waypoints)
        self.sensor_log = []
        self.patrol_error = ""

        # Stand up
        self.nav_state = "standing_up"
        print("\nStanding up...")
        self.sport_client.StandUp()
        time.sleep(1.5)

        if self._is_stopped():
            return False

        # Enable FreeWalk mode
        print("Activating FreeWalk...")
        self.sport_client.FreeWalk()
        time.sleep(0.5)

        if self._is_stopped():
            return False

        start_x, start_y, start_yaw = self.get_position()
        print(f"\nStart position: ({start_x:.3f}, {start_y:.3f}), yaw={math.degrees(start_yaw):.1f}")

        # Rigid-body transform to current pose
        base_x, base_y = waypoints[0][0], waypoints[0][1]
        base_yaw = waypoints[0][2]
        yaw_offset = self.normalize_angle(start_yaw - base_yaw)
        cos_a = math.cos(yaw_offset)
        sin_a = math.sin(yaw_offset)

        adjusted = []
        for wx, wy, wyaw in waypoints:
            dx = wx - base_x
            dy = wy - base_y
            new_x = start_x + dx * cos_a - dy * sin_a
            new_y = start_y + dx * sin_a + dy * cos_a
            adjusted.append((new_x, new_y, self.normalize_angle(wyaw + yaw_offset)))

        self.waypoints_adjusted = adjusted
        print(f"\nAdjusted waypoints: yaw_offset={math.degrees(yaw_offset):+.1f}deg")

        # Auto-home gantry at patrol start (retry up to 3 times)
        if self.gantry_client:
            gantry_ok = False
            for attempt in range(1, 4):
                try:
                    if self.gantry_client.is_reachable():
                        self.gantry_client.home_here()
                        print("Gantry: auto-homed (current position = 0mm)")
                        gantry_ok = True
                        break
                    else:
                        print(f"  Gantry not reachable (attempt {attempt}/3)...")
                except Exception as e:
                    print(f"  Gantry connection error (attempt {attempt}/3): {e}")
                if attempt < 3:
                    time.sleep(2)

            if not gantry_ok:
                print("\n" + "!" * 60)
                print(f"  ERROR: Gantry at {self.gantry_client.base_url} not reachable!")
                print(f"  Make sure on Jetson: python3 ~/gantry_control.py --web --port /dev/ttyUSB0")
                print("!" * 60)
                print("Continuing WITHOUT gantry (single-altitude mode).\n")
                self.gantry_client = None

        return True

    def start_patrol(self):
        """Run the waypoint iteration loop. In web mode, waits for continue signal."""
        waypoints = self.waypoints_adjusted

        try:
            for i, wp in enumerate(waypoints):
                if self._is_stopped():
                    break

                target_x, target_y = wp[0], wp[1]
                self.current_waypoint_idx = i
                self.nav_state = "navigating"
                self.waypoint_statuses[i] = "navigating"
                print(f"\n>>> Waypoint {i+1}/{len(waypoints)}: ({target_x:.3f}, {target_y:.3f})")

                self.move_to_point(target_x, target_y,
                                   rotate_tolerance=self.rotate_tolerance,
                                   move_tolerance=self.move_tolerance)

                if self._is_stopped():
                    break

                if self.gantry_client and self.measure_mode == "2d":
                    # --- 2D MODE: single elevated measurement ---

                    # First waypoint: raise gantry once
                    if i == 0:
                        self.nav_state = "gantry_raising"
                        self.waypoint_statuses[i] = "gantry_raising"
                        print(f"    Raising gantry to {self.gantry_height}mm...")
                        self.gantry_client.move_to(self.gantry_height)
                        if not self.gantry_client.wait_for_arrival(timeout=120):
                            state = self.gantry_client.get_state()
                            raise RuntimeError(f"Gantry failed to reach {self.gantry_height}mm: {state.get('last_error', 'timeout')}")
                        if self._is_stopped():
                            break

                    # Measure at elevated height
                    self.nav_state = "collecting"
                    self.waypoint_statuses[i] = "collecting"
                    print(f"    Collecting sensor data at {self.gantry_height}mm ({self.measure_duration}s)...")
                    self._collect_sensor_averaged(i + 1, duration=self.measure_duration,
                                                  altitude_mm=self.gantry_height, phase="elevated")
                    if self._is_stopped():
                        break

                    self.waypoint_statuses[i] = "done"

                    # Wait for user confirmation
                    if i < len(waypoints) - 1:
                        if self._web_mode:
                            self.nav_state = "waiting"
                            print(f"    Waiting for user to continue...")
                            self._continue_event.clear()
                            while not self._continue_event.wait(timeout=0.2):
                                if self._is_stopped():
                                    break
                            if self._is_stopped():
                                break

                    # Last waypoint: start lowering gantry (cleanup will wait)
                    if i == len(waypoints) - 1:
                        print(f"    Lowering gantry to home...")
                        self.gantry_client.move_to(0)

                elif self.gantry_client and self.measure_mode == "3d":
                    # --- 3D MODE: dual-altitude measurement cycle ---

                    # Ensure gantry is at home (may be lowering from previous waypoint)
                    self._ensure_gantry_home(timeout=120)
                    if self._is_stopped():
                        break

                    # Low altitude measurement
                    self.nav_state = "collecting_low"
                    self.waypoint_statuses[i] = "collecting_low"
                    print(f"    Collecting low-altitude sensor data ({self.measure_duration}s)...")
                    self._collect_sensor_averaged(i + 1, duration=self.measure_duration,
                                                  altitude_mm=0, phase="low")
                    if self._is_stopped():
                        break

                    # Raise gantry
                    self.nav_state = "gantry_raising"
                    self.waypoint_statuses[i] = "gantry_raising"
                    print(f"    Raising gantry to {self.gantry_height}mm...")
                    self.gantry_client.move_to(self.gantry_height)
                    if not self.gantry_client.wait_for_arrival(timeout=120):
                        state = self.gantry_client.get_state()
                        raise RuntimeError(f"Gantry failed to reach {self.gantry_height}mm: {state.get('last_error', 'timeout')}")
                    if self._is_stopped():
                        break

                    # High altitude measurement
                    self.nav_state = "collecting_high"
                    self.waypoint_statuses[i] = "collecting_high"
                    print(f"    Collecting high-altitude sensor data ({self.measure_duration}s)...")
                    self._collect_sensor_averaged(i + 1, duration=self.measure_duration,
                                                  altitude_mm=self.gantry_height, phase="high")
                    if self._is_stopped():
                        break

                    self.waypoint_statuses[i] = "done"

                    # Wait for user confirmation
                    if i < len(waypoints) - 1:
                        if self._web_mode:
                            self.nav_state = "waiting"
                            print(f"    Waiting for user to continue...")
                            self._continue_event.clear()
                            while not self._continue_event.wait(timeout=0.2):
                                if self._is_stopped():
                                    break
                            if self._is_stopped():
                                break

                        # Start lowering gantry (async — runs in parallel with navigation)
                        print(f"    Lowering gantry to home (parallel with navigation)...")
                        self.gantry_client.move_to(0)

                else:
                    # --- ORIGINAL SINGLE-MEASUREMENT (no gantry) ---
                    self.nav_state = "collecting"
                    self.waypoint_statuses[i] = "collecting"
                    time.sleep(0.3)
                    self.log_data_point(i + 1)

                    if self._is_stopped():
                        break

                    self.waypoint_statuses[i] = "done"

                    # Wait for user (web mode: wait for continue event; CLI: input())
                    if i < len(waypoints) - 1:
                        if self._web_mode:
                            self.nav_state = "waiting"
                            print(f"    Waiting for user to continue...")
                            self._continue_event.clear()
                            while not self._continue_event.wait(timeout=0.2):
                                if self._is_stopped():
                                    break
                            if self._is_stopped():
                                break

            if not self._is_stopped():
                self.nav_state = "done"
                print(f"\n>>> Patrol complete!")

        except Exception as e:
            self.nav_state = "error"
            self.patrol_error = str(e)
            print(f"\nPatrol error: {e}")

        finally:
            print("\nStopping...")
            self.sport_client.StopMove()
            time.sleep(0.5)

            # Return gantry to home position before standing down
            if self.gantry_client:
                try:
                    print("Returning gantry to home...")
                    self.gantry_client.move_to(0)
                    if self.gantry_client.wait_for_arrival(timeout=120):
                        print("Gantry returned to home.")
                    else:
                        print("WARNING: Gantry return home timed out.")
                except Exception as e:
                    print(f"WARNING: Gantry return home failed: {e}")

            print("Recovery stand...")
            self.sport_client.RecoveryStand()
            time.sleep(0.5)

            print("Standing down...")
            self.sport_client.StandDown()

            print("\nSaving sensor data...")
            self.save_sensor_data()

            if self._is_stopped():
                self.nav_state = "idle"
                self._stop_event.clear()

            print("Done!")

    def run_patrol(self, loop=False, wait_for_enter=False):
        """Execute monitoring route (CLI mode entry point)."""
        waypoints = self.load_waypoints()
        if waypoints is None or len(waypoints) == 0:
            return

        print("\n" + "=" * 50)
        print("Auto Navigator")
        print("=" * 50)
        print(f"Waypoints: {len(waypoints)}")
        print(f"Sensor: {'Connected' if self.sensor_connected else 'Not connected (values will be N/A)'}")
        print(f"Loop: {'Yes' if loop else 'No'}")
        print("=" * 50)

        # Print waypoints
        for i, wp in enumerate(waypoints):
            print(f"  #{i+1}: x={wp[0]:.3f}, y={wp[1]:.3f}, yaw={math.degrees(wp[2]):.1f}")

        input("\nPress Enter to start (robot will stand up and move)...")

        # Stand up
        print("\nStanding up...")
        self.sport_client.StandUp()
        time.sleep(3)

        # Enable FreeWalk mode (allows SportClient.Move to work)
        print("Activating FreeWalk...")
        self.sport_client.FreeWalk()
        time.sleep(1)

        start_x, start_y, start_yaw = self.get_position()
        print(f"\nStart position: ({start_x:.3f}, {start_y:.3f}), yaw={math.degrees(start_yaw):.1f}")

        # Adjust waypoints: rigid-body transform (translate + rotate) to current pose
        base_x, base_y = waypoints[0][0], waypoints[0][1]
        base_yaw = waypoints[0][2]
        yaw_offset = self.normalize_angle(start_yaw - base_yaw)
        cos_a = math.cos(yaw_offset)
        sin_a = math.sin(yaw_offset)

        adjusted = []
        for wx, wy, wyaw in waypoints:
            dx = wx - base_x
            dy = wy - base_y
            new_x = start_x + dx * cos_a - dy * sin_a
            new_y = start_y + dx * sin_a + dy * cos_a
            adjusted.append((new_x, new_y, self.normalize_angle(wyaw + yaw_offset)))
        waypoints = adjusted
        print(f"\nAdjusted waypoints: yaw_offset={math.degrees(yaw_offset):+.1f}deg")

        run_count = 0
        try:
            while True:
                run_count += 1
                print(f"\n{'='*50}")
                print(f"RUN #{run_count}")
                print(f"{'='*50}")

                for i, wp in enumerate(waypoints):
                    target_x, target_y = wp[0], wp[1]
                    print(f"\n>>> Waypoint {i+1}/{len(waypoints)}: ({target_x:.3f}, {target_y:.3f})")

                    self.move_to_point(target_x, target_y,
                                   rotate_tolerance=self.rotate_tolerance,
                                   move_tolerance=self.move_tolerance)

                    # Log sensor data at this waypoint
                    time.sleep(1)  # brief pause for sensor reading to stabilize
                    self.log_data_point(i + 1)

                    if wait_for_enter and i < len(waypoints) - 1:
                        input("  Press Enter to continue to next waypoint...")

                print(f"\n>>> Run #{run_count} complete!")

                if not loop:
                    break

                print("\nStarting next loop in 5 seconds... (Ctrl+C to stop)")
                time.sleep(5)

        except KeyboardInterrupt:
            print("\n\nInterrupted!")

        finally:
            # Stop and clean up
            print("\nStopping...")
            self.sport_client.StopMove()
            time.sleep(1)

            print("Recovery stand...")
            self.sport_client.RecoveryStand()
            time.sleep(1)

            print("Standing down...")
            self.sport_client.StandDown()

            # Save data
            print("\nSaving sensor data...")
            self.save_sensor_data()

            print("Done!")


if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser(description="Auto Navigator - autonomous sensor monitoring")
    parser.add_argument("--loop", action="store_true", help="Loop route continuously")
    parser.add_argument("--wait", action="store_true", help="Wait for Enter between waypoints")
    parser.add_argument("--interface", default="enp0s2", help="Network interface")
    parser.add_argument("--bridge", default=None, metavar="HOST",
                        help="WiFi bridge host (e.g. 192.168.0.207). Uses TCP bridge instead of DDS.")
    parser.add_argument("--bridge-port", type=int, default=9877,
                        help="WiFi bridge TCP port (default: 9877)")
    parser.add_argument("--sensor-host", default="192.168.123.18",
                        help="Sensor bridge TCP host (default: 192.168.123.18)")
    parser.add_argument("--sensor-port", type=int, default=9876,
                        help="Sensor bridge TCP port (default: 9876)")
    parser.add_argument("--no-sensor", action="store_true",
                        help="Skip sensor connection entirely")
    parser.add_argument("--no-history", action="store_true",
                        help="Do not save timestamped history CSV copies")
    parser.add_argument("--rotate-tolerance", type=float, default=0.05,
                        help="Rotation tolerance in radians (default: 0.05, ~3 deg)")
    parser.add_argument("--move-tolerance", type=float, default=0.05,
                        help="Move-to-point tolerance in meters (default: 0.05, 5cm)")
    parser.add_argument("--video-port", type=int, default=9878,
                        help="Video relay TCP port on bridge host (default: 9878)")
    parser.add_argument("--gantry-host", default=None, metavar="HOST",
                        help="Gantry controller host (e.g. 192.168.0.207). Enables dual-altitude measurement.")
    parser.add_argument("--gantry-port", type=int, default=8090,
                        help="Gantry controller HTTP port (default: 8090)")
    parser.add_argument("--gantry-height", type=float, default=1500.0,
                        help="Gantry raised height in mm (default: 1500)")
    parser.add_argument("--measure-duration", type=float, default=5.0,
                        help="Sensor measurement duration in seconds (default: 5.0)")
    parser.add_argument("--measure-mode", choices=["2d", "3d"], default="2d",
                        help="2d: single elevated measurement (fast), 3d: dual-altitude (default: 2d)")
    parser.add_argument("--web", action="store_true",
                        help="Launch web UI instead of CLI mode")
    parser.add_argument("--web-port", type=int, default=8080,
                        help="Web server port (default: 8080)")
    args = parser.parse_args()

    nav = AutoNavigator(args.interface,
                          sensor_host=args.sensor_host,
                          sensor_port=args.sensor_port,
                          bridge_host=args.bridge,
                          bridge_port=args.bridge_port,
                          no_sensor=args.no_sensor,
                          save_history=not args.no_history,
                          rotate_tolerance=args.rotate_tolerance,
                          move_tolerance=args.move_tolerance,
                          gantry_host=args.gantry_host,
                          gantry_port=args.gantry_port,
                          gantry_height=args.gantry_height,
                          measure_duration=args.measure_duration,
                          measure_mode=args.measure_mode)

    if args.web:
        from web_server import run_web_server
        run_web_server(nav, port=args.web_port,
                       bridge_mode=nav._bridge_mode,
                       bridge_host=args.bridge,
                       video_port=args.video_port)
    else:
        nav.run_patrol(loop=args.loop, wait_for_enter=args.wait)
