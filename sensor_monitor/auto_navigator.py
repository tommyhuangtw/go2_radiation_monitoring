"""
Auto Navigator with Sensor Logging (Step 2)

Loads waypoints from route_recorder.py output and auto-navigates the robot
through the monitoring route, collecting sensor data at each waypoint.

Usage:
    python sensor_monitor/auto_navigator.py
    python sensor_monitor/auto_navigator.py --web          # Launch web UI
    python sensor_monitor/auto_navigator.py --web --web-port 8080

Prerequisites:
    - waypoints.json in sensor_monitor/data/ (from route_recorder.py)
    - Robot placed at the same starting position as during recording
    - (Optional) sensor_bridge.py running on Jetson for sensor data
"""

import sys
import os
import time
import math
import json
import csv
import socket
import threading

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelSubscriber
from unitree_sdk2py.go2.sport.sport_client import SportClient
from unitree_sdk2py.idl.unitree_go.msg.dds_ import SportModeState_

DATA_DIR = os.path.join(os.path.dirname(__file__), "data")


class AutoNavigator:
    def __init__(self, network_interface="enp0s2",
                 sensor_host="192.168.123.18", sensor_port=9876):
        ChannelFactoryInitialize(0, network_interface)

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

        # Navigation state (for web mode)
        self.nav_state = "idle"          # idle|standing_up|navigating|collecting|waiting|done|error
        self.current_waypoint_idx = 0    # 0-based index into waypoints_adjusted
        self.total_waypoints = 0
        self.waypoints_adjusted = []     # populated by prepare_patrol()
        self.waypoint_statuses = []      # per-waypoint status strings
        self.patrol_error = ""

        # Web mode synchronization
        self._continue_event = threading.Event()
        self._stop_event = threading.Event()
        self._web_mode = False

        # Subscribe to state
        self.sub = ChannelSubscriber("rt/sportmodestate", SportModeState_)
        self.sub.Init(self._on_state, 10)

        # Start sensor TCP client thread
        self._sensor_host = sensor_host
        self._sensor_port = sensor_port
        self._sensor_thread = threading.Thread(
            target=self._sensor_tcp_loop, daemon=True)
        self._sensor_thread.start()

        # Sport Client
        self.sport_client = SportClient()
        self.sport_client.SetTimeout(10.0)
        self.sport_client.Init()

        # Wait for state
        print("Waiting for state updates...")
        t0 = time.time()
        while not self.state_ready and time.time() - t0 < 5.0:
            time.sleep(0.1)
        if self.state_ready:
            print(f"State ready! Position: ({self.current_x:.2f}, {self.current_y:.2f}), "
                  f"yaw={math.degrees(self.current_yaw):.1f}")
        else:
            print("WARNING: State update timeout! Robot may not be connected.")

        # Wait for sensor connection (up to 8s to cover connect timeout + retry)
        if not self.sensor_connected:
            print(f"Waiting for sensor connection ({self._sensor_host}:{self._sensor_port})...")
            t0 = time.time()
            while not self.sensor_connected and time.time() - t0 < 8.0:
                time.sleep(0.2)
        if self.sensor_connected:
            print(f"Sensor connected!")
        else:
            print(f"WARNING: Sensor not connected. Values will be N/A.")

    def _on_state(self, msg):
        self.current_x = msg.position[0]
        self.current_y = msg.position[1]
        self.current_yaw = msg.imu_state.rpy[2]
        self.state_ready = True

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

    def rotate_to_target(self, target_x, target_y, tolerance=0.25, timeout=30.0):
        """Rotate to face target point using SportClient.Move."""
        print("  Rotating to face target...")

        target_yaw = self.angle_to(target_x, target_y)
        print(f"    [Rotate] Target heading: {math.degrees(target_yaw):.1f}")

        start_time = time.time()
        iteration = 0
        last_progress_check_time = start_time
        last_yaw_error = None

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
                if last_yaw_error is not None and abs(abs(yaw_error) - abs(last_yaw_error)) < 0.05:
                    print(f"    [Rotate] No progress, skipping (error: {math.degrees(yaw_error):.1f})")
                    break
                last_yaw_error = abs(yaw_error)
                last_progress_check_time = time.time()

            if time.time() - start_time > timeout:
                print(f"    [Rotate] Timeout! Error: {math.degrees(yaw_error):.1f}")
                break

            if abs(yaw_error) < 0.05:
                vyaw = 0
            else:
                vyaw = 0.8 if yaw_error > 0 else -0.8
            self.sport_client.Move(0, 0, vyaw)
            time.sleep(0.02)
            iteration += 1

        self.sport_client.StopMove()
        time.sleep(0.3)

    def move_forward_to_target(self, target_x, target_y, tolerance=0.25, speed=0.3, timeout=60.0):
        """Move forward to target with P-control steering."""
        print("  Moving forward...")

        start_time = time.time()
        iteration = 0
        last_x, last_y = None, None
        last_progress_time = None
        stuck_counter = 0

        while self.distance_to(target_x, target_y) > tolerance:
            if self._is_stopped():
                self.sport_client.StopMove()
                return

            x, y, yaw = self.get_position()
            distance = self.distance_to(target_x, target_y)

            if iteration % 50 == 0:
                print(f"    [Move] pos=({x:.2f}, {y:.2f}), dist={distance:.2f}m")

            if time.time() - start_time > timeout:
                print(f"    [Move] Timeout! Distance: {distance:.2f}m")
                break

            # Stuck detection: check every 2s if robot moved at least 5cm
            if last_x is None:
                last_x, last_y = x, y
                last_progress_time = time.time()
            elif time.time() - last_progress_time > 2.0:
                position_change = math.sqrt((x - last_x)**2 + (y - last_y)**2)
                if position_change < 0.05:
                    stuck_counter += 1
                    if stuck_counter >= 3:  # 6 seconds of no real progress
                        print(f"    [Move] Stuck, accepting position (dist: {distance:.2f}m)")
                        break
                else:
                    stuck_counter = 0
                last_x, last_y = x, y
                last_progress_time = time.time()

            target_yaw = self.angle_to(target_x, target_y)
            yaw_error = self.normalize_angle(target_yaw - yaw)

            vx = min(speed, distance * 0.8)
            vx = max(vx, 0.15)
            vyaw = max(min(yaw_error * 0.8, 0.3), -0.3)

            self.sport_client.Move(vx, 0, vyaw)
            time.sleep(0.02)
            iteration += 1

        self.sport_client.StopMove()
        print(f"    [Move] Complete! Final dist: {self.distance_to(target_x, target_y):.3f}m")

    def move_to_point(self, target_x, target_y, tolerance=0.25):
        """Two-phase navigation: rotate then move forward."""
        x, y, _ = self.get_position()
        print(f"  Current: ({x:.2f}, {y:.2f}), Target: ({target_x:.2f}, {target_y:.2f})")

        if self.distance_to(target_x, target_y) < tolerance:
            print("  Already at target!")
            return

        self.rotate_to_target(target_x, target_y)
        if self._is_stopped():
            return
        self.move_forward_to_target(target_x, target_y, tolerance)
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
        """Log current position + sensor value."""
        x, y, yaw = self.get_position()
        val = self.read_sensor()

        entry = {
            "timestamp": time.time(),
            "waypoint": waypoint_idx,
            "x": round(x, 4),
            "y": round(y, 4),
            "z": 0.0,  # ground level (could use body_height if needed)
            "yaw": round(yaw, 4),
            "sensor_value": val if val is not None else "",
        }
        self.sensor_log.append(entry)

        v_str = f"{val}" if val is not None else "N/A"
        print(f"    Logged: pos=({x:.3f}, {y:.3f}), sensor={v_str}")

    def save_sensor_data(self):
        """Save sensor log to CSV."""
        if not self.sensor_log:
            print("No sensor data to save.")
            return

        os.makedirs(DATA_DIR, exist_ok=True)
        csv_path = os.path.join(DATA_DIR, "sensor_data.csv")

        fieldnames = ["timestamp", "waypoint", "x", "y", "z", "yaw", "sensor_value"]
        with open(csv_path, 'w', newline='') as f:
            writer = csv.DictWriter(f, fieldnames=fieldnames)
            writer.writeheader()
            writer.writerows(self.sensor_log)

        print(f"Saved {len(self.sensor_log)} data points to {csv_path}")

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
        self.nav_state = "idle"

    def get_state_snapshot(self):
        """Return current state as a dict for WebSocket broadcast."""
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
        time.sleep(3)

        if self._is_stopped():
            return False

        # Enable FreeWalk mode
        print("Activating FreeWalk...")
        self.sport_client.FreeWalk()
        time.sleep(1)

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

                self.move_to_point(target_x, target_y)

                if self._is_stopped():
                    break

                # Log sensor data
                self.nav_state = "collecting"
                self.waypoint_statuses[i] = "collecting"
                time.sleep(1)
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
                        # Wait with periodic stop-event checks
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
            time.sleep(1)

            print("Recovery stand...")
            self.sport_client.RecoveryStand()
            time.sleep(1)

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

                    self.move_to_point(target_x, target_y)

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
    parser.add_argument("--sensor-host", default="192.168.123.18",
                        help="Sensor bridge TCP host (default: 192.168.123.18)")
    parser.add_argument("--sensor-port", type=int, default=9876,
                        help="Sensor bridge TCP port (default: 9876)")
    parser.add_argument("--web", action="store_true",
                        help="Launch web UI instead of CLI mode")
    parser.add_argument("--web-port", type=int, default=8080,
                        help="Web server port (default: 8080)")
    args = parser.parse_args()

    nav = AutoNavigator(args.interface,
                          sensor_host=args.sensor_host,
                          sensor_port=args.sensor_port)

    if args.web:
        from web_server import run_web_server
        run_web_server(nav, port=args.web_port)
    else:
        nav.run_patrol(loop=args.loop, wait_for_enter=args.wait)
