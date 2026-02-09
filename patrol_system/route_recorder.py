"""
Route Recorder (Step 1a)

Walk the robot manually while this script records waypoints (x, y, yaw)
when you press Enter. For 3D LiDAR mapping, use lidar_mapper.py separately.

Usage:
    python patrol_system/route_recorder.py

Controls:
    [Enter] - Record current position as waypoint
    [p]     - Print all recorded waypoints
    [c]     - Clear all waypoints
    [s]     - Save waypoints
    [q]     - Quit (auto-saves if waypoints exist)
"""

import sys
import os
import time
import math
import json
import select
import shutil

# Add parent directory to path for unitree_sdk2py imports
sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelSubscriber
from unitree_sdk2py.idl.unitree_go.msg.dds_ import SportModeState_

# Data output directory
DATA_DIR = os.path.join(os.path.dirname(__file__), "data")


class RouteRecorder:
    def __init__(self, network_interface="enp0s2"):
        ChannelFactoryInitialize(0, network_interface)

        # Position tracking
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0
        self.state_ready = False

        # Waypoints
        self.waypoints = []

        # Subscribe to robot state
        self.state_sub = ChannelSubscriber("rt/sportmodestate", SportModeState_)
        self.state_sub.Init(self._on_state, 10)

        # Wait for state
        print("Waiting for robot state...")
        t0 = time.time()
        while not self.state_ready and time.time() - t0 < 5.0:
            time.sleep(0.1)
        if self.state_ready:
            print(f"State ready! Position: ({self.current_x:.3f}, {self.current_y:.3f}), yaw={math.degrees(self.current_yaw):.1f}")
        else:
            print("WARNING: No state update received. Robot may not be connected.")

    def _on_state(self, msg):
        self.current_x = msg.position[0]
        self.current_y = msg.position[1]
        self.current_yaw = msg.imu_state.rpy[2]
        self.state_ready = True

    def save_all(self):
        """Save waypoints to JSON."""
        os.makedirs(DATA_DIR, exist_ok=True)

        # Backup existing waypoints file before overwriting
        wp_path = os.path.join(DATA_DIR, "waypoints.json")
        if os.path.exists(wp_path):
            shutil.copy2(wp_path, wp_path + ".bak")
            print("  Backed up existing waypoints.json (.bak)")

        wp_data = []
        for wp in self.waypoints:
            wp_data.append({"x": wp[0], "y": wp[1], "yaw": wp[2]})
        with open(wp_path, 'w') as f:
            json.dump(wp_data, f, indent=2)
        print(f"  Saved {len(self.waypoints)} waypoints to {wp_path}")

    def run(self):
        """Interactive recording loop."""
        print("\n" + "=" * 50)
        print("Route Recorder")
        print("=" * 50)
        print("Commands:")
        print("  [Enter] - Record current position as waypoint")
        print("  [p]     - Print all recorded waypoints")
        print("  [c]     - Clear all waypoints")
        print("  [s]     - Save waypoints")
        print("  [q]     - Quit (auto-saves if waypoints exist)")
        print("=" * 50)
        print()

        try:
            while True:
                # Show current position
                yaw_deg = math.degrees(self.current_yaw)
                status = (f"\rPos: x={self.current_x:7.3f}, y={self.current_y:7.3f}, "
                          f"yaw={yaw_deg:6.1f}\u00b0 | "
                          f"WP: {len(self.waypoints)}  ")
                print(status, end="", flush=True)

                # Non-blocking input check
                if select.select([sys.stdin], [], [], 0.5)[0]:
                    cmd = input().strip().lower()

                    if cmd == "":
                        # Record waypoint
                        wp = (round(self.current_x, 3),
                              round(self.current_y, 3),
                              round(self.current_yaw, 3))
                        self.waypoints.append(wp)
                        yaw_deg = math.degrees(self.current_yaw)
                        print(f"\n>>> Recorded waypoint #{len(self.waypoints)}: "
                              f"x={wp[0]}, y={wp[1]}, yaw={yaw_deg:.1f}\u00b0")

                    elif cmd == "p":
                        print(f"\n>>> Recorded waypoints ({len(self.waypoints)}):")
                        for i, wp in enumerate(self.waypoints):
                            print(f"  #{i+1}: x={wp[0]:.3f}, y={wp[1]:.3f}, "
                                  f"yaw={math.degrees(wp[2]):.1f}\u00b0")

                    elif cmd == "c":
                        self.waypoints = []
                        print("\n>>> Cleared all waypoints")

                    elif cmd == "s":
                        print("\n>>> Saving...")
                        self.save_all()
                        print(">>> Save complete!")

                    elif cmd == "q":
                        break

        except KeyboardInterrupt:
            print("\n\nInterrupted!")

        # Auto-save on quit if there's data
        if self.waypoints:
            print("\nAuto-saving before exit...")
            self.save_all()

        print("\n" + "=" * 50)
        print("Route recording finished!")
        if self.waypoints:
            print(f"Recorded {len(self.waypoints)} waypoints")
            print(f"Files saved to: {DATA_DIR}/")
        print("=" * 50)


if __name__ == "__main__":
    recorder = RouteRecorder("enp0s2")
    recorder.run()
