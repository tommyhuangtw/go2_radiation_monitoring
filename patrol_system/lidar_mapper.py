"""
LiDAR Mapper (Step 1b)

Walk the robot slowly while this script builds a dense 3D point cloud
from LiDAR data. Generates a point cloud (PLY) and 2D floor plan.

For waypoint recording, use route_recorder.py separately.

Usage:
    python patrol_system/lidar_mapper.py
    python patrol_system/lidar_mapper.py --throttle 10 --downsample 2
    python patrol_system/lidar_mapper.py --interface enp0s2

Controls:
    [Enter] - Toggle mapping on/off (pause/resume)
    [s]     - Save point cloud + floor plan
    [r]     - Reset accumulated points
    [i]     - Print detailed stats
    [q]     - Quit (auto-saves if data exists)
"""

import sys
import os
import time
import math
import json
import select
import shutil

import numpy as np

# Add parent directory to path for unitree_sdk2py imports
sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelSubscriber
from unitree_sdk2py.idl.unitree_go.msg.dds_ import SportModeState_
from unitree_sdk2py.idl.sensor_msgs.msg.dds_ import PointCloud2_

# PointField datatype constants
FLOAT32 = 7
DATATYPE_SIZE = {1: 1, 2: 1, 3: 2, 4: 2, 5: 4, 6: 4, 7: 4, 8: 8}

# Data output directory
DATA_DIR = os.path.join(os.path.dirname(__file__), "data")


class LidarMapper:
    def __init__(self, network_interface="enp0s2", throttle_hz=10, downsample=2):
        ChannelFactoryInitialize(0, network_interface)

        # Position tracking (for display)
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0
        self.state_ready = False

        # LiDAR point cloud accumulation
        self.all_points = []  # list of Nx3 numpy arrays
        self.total_points = 0
        self.lidar_frames = 0
        self.last_lidar_time = 0.0
        self.lidar_ready = False
        self.mapping_active = True  # can be paused/resumed

        # Configurable parameters
        self.throttle_interval = 1.0 / throttle_hz
        self.downsample_factor = downsample

        # Floor plan parameters
        self.resolution = 0.05  # 5cm per pixel
        self.z_min = 0.1  # filter ground
        self.z_max = 1.5  # filter ceiling

        # Timing
        self.start_time = time.time()

        # Subscribe to robot state (for position display)
        self.state_sub = ChannelSubscriber("rt/sportmodestate", SportModeState_)
        self.state_sub.Init(self._on_state, 10)

        # Subscribe to LiDAR point cloud (deskewed = in odom frame)
        self.lidar_sub = ChannelSubscriber("rt/utlidar/cloud_deskewed", PointCloud2_)
        self.lidar_sub.Init(self._on_lidar, 10)

        # Wait for state
        print("Waiting for robot state...")
        t0 = time.time()
        while not self.state_ready and time.time() - t0 < 5.0:
            time.sleep(0.1)
        if self.state_ready:
            print(f"State ready! Position: ({self.current_x:.3f}, {self.current_y:.3f}), yaw={math.degrees(self.current_yaw):.1f}")
        else:
            print("WARNING: No state update received. Robot may not be connected.")

        # Wait for LiDAR
        print("Waiting for LiDAR data...")
        t0 = time.time()
        while not self.lidar_ready and time.time() - t0 < 5.0:
            time.sleep(0.1)
        if self.lidar_ready:
            print("LiDAR data received!")
        else:
            print("WARNING: No LiDAR data received. LiDAR may be off.")
            print("  Tip: Enable LiDAR via Unitree App or rt/utlidar/switch topic")

    def _on_state(self, msg):
        self.current_x = msg.position[0]
        self.current_y = msg.position[1]
        self.current_yaw = msg.imu_state.rpy[2]
        self.state_ready = True

    def _on_lidar(self, msg):
        """Parse PointCloud2 message and accumulate 3D points."""
        self.lidar_ready = True

        if not self.mapping_active:
            return

        # Throttle processing rate
        now = time.time()
        if now - self.last_lidar_time < self.throttle_interval:
            return
        self.last_lidar_time = now

        points = self._parse_pointcloud2(msg)
        if points is None or len(points) == 0:
            return

        # Downsample
        if self.downsample_factor > 1:
            points = points[::self.downsample_factor]

        self.all_points.append(points)
        self.total_points += len(points)
        self.lidar_frames += 1

    def _parse_pointcloud2(self, msg):
        """Extract xyz points from a PointCloud2 message."""
        point_step = msg.point_step
        raw = bytes(msg.data)
        n_points = len(raw) // point_step

        if n_points == 0:
            return None

        # Find x, y, z field offsets
        field_offsets = {}
        for f in msg.fields:
            if f.name in ('x', 'y', 'z'):
                field_offsets[f.name] = f.offset

        if not all(k in field_offsets for k in ('x', 'y', 'z')):
            return None

        x_off = field_offsets['x']
        y_off = field_offsets['y']
        z_off = field_offsets['z']

        # Parse all points using numpy for speed
        raw_array = np.frombuffer(raw, dtype=np.uint8)
        # Reshape into per-point blocks
        if len(raw_array) < n_points * point_step:
            n_points = len(raw_array) // point_step
        raw_array = raw_array[:n_points * point_step].reshape(n_points, point_step)

        # Extract float32 at each offset
        x = raw_array[:, x_off:x_off+4].copy().view(np.float32).flatten()
        y = raw_array[:, y_off:y_off+4].copy().view(np.float32).flatten()
        z = raw_array[:, z_off:z_off+4].copy().view(np.float32).flatten()

        # Filter out NaN and inf
        valid = np.isfinite(x) & np.isfinite(y) & np.isfinite(z)
        # Filter out zero/near-zero points (LiDAR invalid returns)
        dist_sq = x*x + y*y + z*z
        valid &= dist_sq > 0.01  # reject points within 10cm of origin
        points = np.column_stack([x[valid], y[valid], z[valid]])

        return points

    def save_all(self):
        """Save point cloud, floor plan, and metadata."""
        os.makedirs(DATA_DIR, exist_ok=True)

        # Backup existing files before overwriting
        backup_files = ["point_cloud.ply", "floor_plan.png", "floor_plan.npy", "map_meta.json"]
        backed_up = False
        for fname in backup_files:
            fpath = os.path.join(DATA_DIR, fname)
            if os.path.exists(fpath):
                shutil.copy2(fpath, fpath + ".bak")
                backed_up = True
        if backed_up:
            print("  Backed up existing data files (.bak)")

        if self.total_points > 0:
            all_pts = np.vstack(self.all_points)
            print(f"  Total points collected: {len(all_pts):,}")

            # Save PLY
            ply_path = os.path.join(DATA_DIR, "point_cloud.ply")
            self._save_ply(ply_path, all_pts)
            print(f"  Saved point cloud to {ply_path}")

            # Build and save 2D floor plan
            self._save_floor_plan(all_pts)
        else:
            print("  WARNING: No LiDAR data collected. Nothing to save.")

    def _save_ply(self, filename, points):
        """Save Nx3 numpy array as ASCII PLY file."""
        with open(filename, 'w') as f:
            f.write("ply\n")
            f.write("format ascii 1.0\n")
            f.write(f"element vertex {len(points)}\n")
            f.write("property float x\n")
            f.write("property float y\n")
            f.write("property float z\n")
            f.write("end_header\n")
            for p in points:
                f.write(f"{p[0]:.4f} {p[1]:.4f} {p[2]:.4f}\n")

    def _save_floor_plan(self, points):
        """Build 2D occupancy grid from 3D points and save as image."""
        # Filter points by height (keep walls/furniture, remove floor/ceiling)
        mask = (points[:, 2] > self.z_min) & (points[:, 2] < self.z_max)
        pts_2d = points[mask, :2]  # keep only x, y

        if len(pts_2d) == 0:
            print("  WARNING: No points in height range for floor plan.")
            return

        # Compute grid bounds
        x_min, y_min = pts_2d.min(axis=0) - 0.5
        x_max, y_max = pts_2d.max(axis=0) + 0.5

        width = int((x_max - x_min) / self.resolution) + 1
        height = int((y_max - y_min) / self.resolution) + 1

        # Cap grid size to prevent memory issues
        max_dim = 2000
        if width > max_dim or height > max_dim:
            scale = max_dim / max(width, height)
            self.resolution = self.resolution / scale
            width = int((x_max - x_min) / self.resolution) + 1
            height = int((y_max - y_min) / self.resolution) + 1
            print(f"  Adjusted resolution to {self.resolution:.3f}m (grid: {width}x{height})")

        # Build occupancy grid (count points per cell)
        grid = np.zeros((height, width), dtype=np.float32)
        xi = ((pts_2d[:, 0] - x_min) / self.resolution).astype(int)
        yi = ((pts_2d[:, 1] - y_min) / self.resolution).astype(int)

        # Clip to grid bounds
        xi = np.clip(xi, 0, width - 1)
        yi = np.clip(yi, 0, height - 1)

        np.add.at(grid, (yi, xi), 1)

        # Normalize to 0-255 (occupied = dark, free = white)
        grid_norm = np.clip(grid, 0, np.percentile(grid[grid > 0], 95) if np.any(grid > 0) else 1)
        grid_img = 255 - (grid_norm / grid_norm.max() * 255).astype(np.uint8)

        # Save floor plan as PNG using matplotlib (no opencv needed)
        try:
            import matplotlib
            matplotlib.use('Agg')
            import matplotlib.pyplot as plt

            fig, ax = plt.subplots(1, 1, figsize=(10, 10))
            ax.imshow(grid_img, cmap='gray', origin='lower',
                      extent=[x_min, x_max, y_min, y_max])

            # Overlay waypoints if waypoints.json exists
            wp_path = os.path.join(DATA_DIR, "waypoints.json")
            if os.path.exists(wp_path):
                with open(wp_path, 'r') as f:
                    wp_data = json.load(f)
                if wp_data:
                    wx = [wp["x"] for wp in wp_data]
                    wy = [wp["y"] for wp in wp_data]
                    ax.plot(wx, wy, 'r-o', markersize=8, linewidth=2, label='Patrol route')
                    for i, wp in enumerate(wp_data):
                        ax.annotate(f'{i+1}', (wp["x"], wp["y"]), fontsize=10,
                                    fontweight='bold', color='red',
                                    ha='center', va='bottom', xytext=(0, 10),
                                    textcoords='offset points')
                    ax.legend()

            ax.set_xlabel('X (m)')
            ax.set_ylabel('Y (m)')
            ax.set_title('Floor Plan (top-down LiDAR projection)')
            ax.set_aspect('equal')

            png_path = os.path.join(DATA_DIR, "floor_plan.png")
            fig.savefig(png_path, dpi=150, bbox_inches='tight')
            plt.close(fig)
            print(f"  Saved floor plan to {png_path}")
        except ImportError:
            print("  WARNING: matplotlib not available, skipping floor plan PNG.")

        # Save raw grid as numpy
        npy_path = os.path.join(DATA_DIR, "floor_plan.npy")
        np.save(npy_path, grid)
        print(f"  Saved floor plan grid to {npy_path}")

        # Save metadata
        meta = {
            "resolution": self.resolution,
            "origin_x": float(x_min),
            "origin_y": float(y_min),
            "width": width,
            "height": height,
            "z_min": self.z_min,
            "z_max": self.z_max,
        }
        meta_path = os.path.join(DATA_DIR, "map_meta.json")
        with open(meta_path, 'w') as f:
            json.dump(meta, f, indent=2)
        print(f"  Saved map metadata to {meta_path}")

    def reset(self):
        """Clear accumulated points."""
        self.all_points = []
        self.total_points = 0
        self.lidar_frames = 0
        self.start_time = time.time()

    def run(self):
        """Interactive mapping loop."""
        print("\n" + "=" * 50)
        print("LiDAR Mapper")
        print("=" * 50)
        print("Commands:")
        print("  [Enter] - Toggle mapping on/off (pause/resume)")
        print("  [s]     - Save point cloud + floor plan")
        print("  [r]     - Reset accumulated points")
        print("  [i]     - Print detailed stats")
        print("  [q]     - Quit (auto-saves if data exists)")
        print("=" * 50)
        print()

        try:
            while True:
                # Show current position and LiDAR stats
                yaw_deg = math.degrees(self.current_yaw)
                elapsed = time.time() - self.start_time
                minutes = int(elapsed) // 60
                seconds = int(elapsed) % 60
                state_str = "MAPPING" if self.mapping_active else "PAUSED "
                status = (f"\rPos: x={self.current_x:7.3f}, y={self.current_y:7.3f}, "
                          f"yaw={yaw_deg:6.1f}\u00b0 | "
                          f"{state_str} | "
                          f"{self.lidar_frames} frames, {self.total_points:,} pts | "
                          f"{minutes}m {seconds:02d}s  ")
                print(status, end="", flush=True)

                # Non-blocking input check
                if select.select([sys.stdin], [], [], 0.5)[0]:
                    cmd = input().strip().lower()

                    if cmd == "":
                        # Toggle mapping
                        self.mapping_active = not self.mapping_active
                        state = "RESUMED" if self.mapping_active else "PAUSED"
                        print(f"\n>>> Mapping {state}")

                    elif cmd == "s":
                        print("\n>>> Saving...")
                        self.save_all()
                        print(">>> Save complete!")

                    elif cmd == "r":
                        self.reset()
                        print("\n>>> Reset! Accumulated points cleared.")

                    elif cmd == "i":
                        elapsed = time.time() - self.start_time
                        mem_mb = self.total_points * 12 / 1e6  # 3 x float32
                        print(f"\n>>> Stats:")
                        print(f"  LiDAR frames: {self.lidar_frames}")
                        print(f"  Total points: {self.total_points:,}")
                        print(f"  Memory usage: ~{mem_mb:.1f} MB")
                        print(f"  Elapsed time: {elapsed:.0f}s")
                        print(f"  Throttle: {1.0/self.throttle_interval:.0f} Hz")
                        print(f"  Downsample: 1/{self.downsample_factor}")
                        print(f"  Mapping: {'active' if self.mapping_active else 'paused'}")

                    elif cmd == "q":
                        break

        except KeyboardInterrupt:
            print("\n\nInterrupted!")

        # Auto-save on quit if there's data
        if self.total_points > 0:
            print("\nAuto-saving before exit...")
            self.save_all()

        print("\n" + "=" * 50)
        print("LiDAR mapping finished!")
        if self.total_points > 0:
            print(f"Collected {self.lidar_frames} frames, {self.total_points:,} points")
            print(f"Files saved to: {DATA_DIR}/")
        print("=" * 50)


if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser(description="LiDAR Mapper - Build 3D point cloud maps")
    parser.add_argument("--interface", default="enp0s2", help="Network interface (default: enp0s2)")
    parser.add_argument("--throttle", type=float, default=10.0, help="LiDAR processing rate in Hz (default: 10)")
    parser.add_argument("--downsample", type=int, default=2, help="Keep every Nth point (default: 2)")
    parser.add_argument("--resolution", type=float, default=0.05, help="Floor plan resolution m/pixel (default: 0.05)")
    parser.add_argument("--z-min", type=float, default=0.1, help="Floor plan min height in meters (default: 0.1)")
    parser.add_argument("--z-max", type=float, default=1.5, help="Floor plan max height in meters (default: 1.5)")
    args = parser.parse_args()

    mapper = LidarMapper(args.interface, args.throttle, args.downsample)
    mapper.resolution = args.resolution
    mapper.z_min = args.z_min
    mapper.z_max = args.z_max
    mapper.run()
