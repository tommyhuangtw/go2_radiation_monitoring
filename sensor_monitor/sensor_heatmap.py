"""
3D Sensor Map Visualization (Step 3)

Creates both a 3D visualization (open3d) and a 2D heatmap (matplotlib)
from sensor data overlaid on the LiDAR point cloud / floor plan.

Usage:
    python sensor_monitor/sensor_heatmap.py                 # full sensor heatmap
    python sensor_monitor/sensor_heatmap.py --mode 2d       # 2D only
    python sensor_monitor/sensor_heatmap.py --mode 3d       # 3D only
    python sensor_monitor/sensor_heatmap.py --mode both     # both (default)
    python sensor_monitor/sensor_heatmap.py --cloud-only    # point cloud viewer

Prerequisites:
    - point_cloud.ply in sensor_monitor/data/ (from lidar_mapper.py)
    - floor_plan.npy + map_meta.json in sensor_monitor/data/ (from lidar_mapper.py)
    - sensor_data.csv in sensor_monitor/data/ (from auto_navigator.py, not needed for --cloud-only)
"""

import sys
import os
import csv
import json

import numpy as np

DATA_DIR = os.path.join(os.path.dirname(__file__), "data")


def _check_alignment(waypoints, source_label="waypoints"):
    """Check if waypoint positions fall on occupied cells in the LiDAR floor plan.

    Returns True if aligned (or if map data is unavailable), False if misaligned.
    """
    meta_path = os.path.join(DATA_DIR, "map_meta.json")
    npy_path = os.path.join(DATA_DIR, "floor_plan.npy")

    if not (os.path.exists(meta_path) and os.path.exists(npy_path)):
        return True  # can't check without map data

    with open(meta_path, 'r') as f:
        meta = json.load(f)
    grid = np.load(npy_path)
    res = meta["resolution"]
    ox, oy = meta["origin_x"], meta["origin_y"]
    h, w = grid.shape

    # Check how many waypoints land on or near occupied cells
    near_occupied = 0
    radius = 5  # check 5-cell radius (~25cm at 0.05 res)
    for wp in waypoints:
        wx = wp["x"] if isinstance(wp, dict) else wp[0]
        wy = wp["y"] if isinstance(wp, dict) else wp[1]
        xi = int((wx - ox) / res)
        yi = int((wy - oy) / res)
        # Check neighborhood
        y_lo, y_hi = max(0, yi - radius), min(h, yi + radius + 1)
        x_lo, x_hi = max(0, xi - radius), min(w, xi + radius + 1)
        if y_lo < y_hi and x_lo < x_hi and grid[y_lo:y_hi, x_lo:x_hi].max() > 0:
            near_occupied += 1

    if near_occupied < len(waypoints) * 0.3:  # <30% of waypoints near any LiDAR data
        wp_xs = [wp["x"] if isinstance(wp, dict) else wp[0] for wp in waypoints]
        wp_ys = [wp["y"] if isinstance(wp, dict) else wp[1] for wp in waypoints]
        print(f"\n  WARNING: {source_label} appear OUTSIDE the LiDAR map!")
        print(f"    Only {near_occupied}/{len(waypoints)} waypoints are near occupied cells.")
        print(f"    {source_label}: X=[{min(wp_xs):.2f}, {max(wp_xs):.2f}], Y=[{min(wp_ys):.2f}, {max(wp_ys):.2f}]")
        print(f"    Were route_recorder.py and lidar_mapper.py run in the SAME boot session?")
        print(f"    Re-record both in one session so they share the same odom frame.\n")
        return False

    return True


def _make_sphere_pcd(center, radius, color, n_points=200):
    """Create a small colored PointCloud sphere (avoids buggy TriangleMesh factories)."""
    import open3d as o3d
    # Fibonacci sphere for even point distribution
    indices = np.arange(n_points, dtype=float)
    phi = np.arccos(1 - 2 * (indices + 0.5) / n_points)
    theta = np.pi * (1 + 5**0.5) * indices
    pts = np.column_stack([
        np.sin(phi) * np.cos(theta),
        np.sin(phi) * np.sin(theta),
        np.cos(phi),
    ]) * radius + np.array(center)
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(pts)
    pcd.colors = o3d.utility.Vector3dVector(np.tile(color, (n_points, 1)))
    return pcd


def load_sensor_data():
    """Load sensor data CSV, aligning positions to map frame via waypoints.json."""
    csv_path = os.path.join(DATA_DIR, "sensor_data.csv")
    if not os.path.exists(csv_path):
        print(f"ERROR: {csv_path} not found!")
        print("Run auto_navigator.py first.")
        return None

    data = []
    with open(csv_path, 'r') as f:
        reader = csv.DictReader(f)
        for row in reader:
            raw = row.get("sensor_value", "")
            entry = {
                "waypoint": int(row.get("waypoint", 0)),
                "x": float(row["x"]),
                "y": float(row["y"]),
                "z": float(row["z"]),
                "yaw": float(row["yaw"]),
                "sensor_value": float(raw) if raw else None,
            }
            data.append(entry)

    # Align sensor positions to map frame using waypoints.json
    # (waypoints.json and point_cloud.ply share the same odom frame,
    #  but sensor_data.csv is from a different boot session)
    wp_path = os.path.join(DATA_DIR, "waypoints.json")
    if os.path.exists(wp_path):
        with open(wp_path, 'r') as f:
            wp_data = json.load(f)
        aligned = 0
        for d in data:
            idx = d["waypoint"] - 1  # waypoint column is 1-indexed
            if 0 <= idx < len(wp_data):
                d["x"] = wp_data[idx]["x"]
                d["y"] = wp_data[idx]["y"]
                aligned += 1
        print(f"Aligned {aligned}/{len(data)} positions to map frame via waypoints.json")
        _check_alignment(wp_data, source_label="Waypoints")
    else:
        print("WARNING: waypoints.json not found — using sensor_data positions (may be misaligned)")

    valid = [d for d in data if d["sensor_value"] is not None]
    print(f"Loaded {len(data)} data points ({len(valid)} with sensor values)")
    return data


def sensor_to_color(value, vmin, vmax):
    """Map sensor value to color: blue (low) -> yellow (mid) -> red (high)."""
    if vmax == vmin:
        t = 0.5
    else:
        t = (value - vmin) / (vmax - vmin)
    t = max(0.0, min(1.0, t))

    # Blue -> Yellow -> Red
    if t < 0.5:
        r = t * 2
        g = t * 2
        b = 1.0 - t * 2
    else:
        r = 1.0
        g = 1.0 - (t - 0.5) * 2
        b = 0.0

    return (r, g, b)


def visualize_cloud_only():
    """3D point cloud viewer using open3d (no sensor data needed)."""
    try:
        import open3d as o3d
    except ImportError:
        print("ERROR: open3d not installed. Install with: pip install open3d")
        return

    ply_path = os.path.join(DATA_DIR, "point_cloud.ply")
    if not os.path.exists(ply_path):
        print(f"ERROR: {ply_path} not found!")
        print("Run lidar_mapper.py first to record a point cloud.")
        return

    cloud = o3d.io.read_point_cloud(ply_path)
    n_pts = len(cloud.points)
    print(f"Loaded point cloud: {n_pts:,} points")

    # Height-based coloring: blue (floor) → green (mid) → red (ceiling)
    pts = np.asarray(cloud.points)
    z = pts[:, 2]
    z_norm = (z - z.min()) / (z.max() - z.min() + 1e-9)
    colors = np.zeros((n_pts, 3))
    colors[:, 0] = np.clip(2 * z_norm - 0.5, 0, 1)              # R: ramps up in upper half
    colors[:, 1] = np.clip(1 - 2 * np.abs(z_norm - 0.5), 0, 1)  # G: peaks at mid
    colors[:, 2] = np.clip(1 - 2 * z_norm, 0, 1)                # B: high at bottom, fades
    cloud.colors = o3d.utility.Vector3dVector(colors)

    # Estimate normals for Phong shading
    cloud.estimate_normals(
        search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.1, max_nn=30)
    )

    geometries = [cloud]

    # Overlay waypoints if available
    wp_path = os.path.join(DATA_DIR, "waypoints.json")
    if os.path.exists(wp_path):
        with open(wp_path, 'r') as f:
            wp_data = json.load(f)
        if wp_data:
            print(f"Overlaying {len(wp_data)} waypoints")
            _check_alignment(wp_data, source_label="Waypoints")
            # Derive floor height from point cloud
            floor_z = float(np.percentile(z, 10)) + 0.3
            # Red point-cloud spheres at waypoint positions
            for i, wp in enumerate(wp_data):
                geometries.append(
                    _make_sphere_pcd([wp["x"], wp["y"], floor_z], 0.08, [1.0, 0.0, 0.0])
                )

            # Red line connecting waypoints
            if len(wp_data) >= 2:
                pts = np.array([[wp["x"], wp["y"], floor_z] for wp in wp_data])
                idx = np.array([[i, i+1] for i in range(len(pts) - 1)], dtype=np.int32)
                clr = np.tile([1.0, 0.0, 0.0], (len(idx), 1))
                line_set = o3d.geometry.LineSet()
                line_set.points = o3d.utility.Vector3dVector(pts)
                line_set.lines = o3d.utility.Vector2iVector(idx)
                line_set.colors = o3d.utility.Vector3dVector(clr)
                geometries.append(line_set)

    print("Opening 3D viewer... (close window to continue)")
    o3d.visualization.draw_geometries(
        geometries,
        window_name="3D Point Cloud",
        width=1280,
        height=720,
    )


def visualize_3d(sensor_data, sensor_vmin=None, sensor_vmax=None):
    """3D visualization using open3d: point cloud + sensor spheres."""
    try:
        import open3d as o3d
    except ImportError:
        print("ERROR: open3d not installed. Install with: pip install open3d")
        return

    ply_path = os.path.join(DATA_DIR, "point_cloud.ply")
    if not os.path.exists(ply_path):
        print(f"WARNING: {ply_path} not found. Showing sensor data only.")
        cloud = None
    else:
        cloud = o3d.io.read_point_cloud(ply_path)
        n_pts = len(cloud.points)
        print(f"Loaded point cloud: {n_pts:,} points")

        # Height-based coloring: blue (floor) → green (mid) → red (ceiling)
        pts = np.asarray(cloud.points)
        z = pts[:, 2]
        z_norm = (z - z.min()) / (z.max() - z.min() + 1e-9)
        h_colors = np.zeros((n_pts, 3))
        h_colors[:, 0] = np.clip(2 * z_norm - 0.5, 0, 1)              # R: ramps up in upper half
        h_colors[:, 1] = np.clip(1 - 2 * np.abs(z_norm - 0.5), 0, 1)  # G: peaks at mid
        h_colors[:, 2] = np.clip(1 - 2 * z_norm, 0, 1)                # B: high at bottom, fades
        cloud.colors = o3d.utility.Vector3dVector(h_colors)

        # Estimate normals for Phong shading
        cloud.estimate_normals(
            search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.1, max_nn=30)
        )

    # Get sensor values for color scaling
    valid_data = [d for d in sensor_data if d["sensor_value"] is not None]
    if not valid_data:
        print("No sensor data to visualize.")
        if cloud is not None:
            o3d.visualization.draw_geometries([cloud], window_name="3D Point Cloud")
        return

    sensor_values = [d["sensor_value"] for d in valid_data]
    vmin = sensor_vmin if sensor_vmin is not None else min(sensor_values)
    vmax = sensor_vmax if sensor_vmax is not None else max(sensor_values)
    print(f"Sensor value range: {min(sensor_values):.1f} - {max(sensor_values):.1f}"
          f" (color scale: {vmin:.1f} - {vmax:.1f})")

    constant_values = (vmax - vmin) < 5.0
    if constant_values:
        print("WARNING: Sensor values nearly constant — spheres will be at uniform height")

    # Create colored spheres at each sensor measurement position
    # Sphere Z-height represents sensor magnitude (3D bar chart effect)
    geometries = []
    if cloud is not None:
        geometries.append(cloud)

    # Derive floor height from point cloud if available
    if cloud is not None:
        cloud_z = np.asarray(cloud.points)[:, 2]
        floor_z = float(np.percentile(cloud_z, 10))
    else:
        floor_z = 0.0

    Z_MIN, Z_MAX = floor_z + 0.3, floor_z + 2.0  # height range for sensor spheres
    stem_pts = []  # for vertical stem lines

    for d in valid_data:
        color = sensor_to_color(d["sensor_value"], vmin, vmax)

        if constant_values:
            z_height = floor_z + 1.0  # fixed height when values don't vary
        else:
            t = (d["sensor_value"] - vmin) / (vmax - vmin)
            z_height = Z_MIN + t * (Z_MAX - Z_MIN)

        geometries.append(
            _make_sphere_pcd([d["x"], d["y"], z_height], 0.1, list(color))
        )
        # Vertical stem from ground to sphere
        stem_pts.append(([d["x"], d["y"], floor_z], [d["x"], d["y"], z_height]))

    # Add vertical stem lines (ground to sphere)
    if stem_pts:
        all_pts = []
        all_idx = []
        for i, (p0, p1) in enumerate(stem_pts):
            all_pts.append(p0)
            all_pts.append(p1)
            all_idx.append([i * 2, i * 2 + 1])
        all_pts = np.array(all_pts)
        all_idx = np.array(all_idx, dtype=np.int32)
        stem_clr = np.tile([0.5, 0.5, 0.5], (len(all_idx), 1))
        stem_lines = o3d.geometry.LineSet()
        stem_lines.points = o3d.utility.Vector3dVector(all_pts)
        stem_lines.lines = o3d.utility.Vector2iVector(all_idx)
        stem_lines.colors = o3d.utility.Vector3dVector(stem_clr)
        geometries.append(stem_lines)

    # Add route path as a line set (at ground level)
    if len(sensor_data) >= 2:
        pts = np.array([[d["x"], d["y"], floor_z + 0.15] for d in sensor_data])
        idx = np.array([[i, i+1] for i in range(len(pts) - 1)], dtype=np.int32)
        clr = np.tile([1.0, 0.0, 0.0], (len(idx), 1))

        line_set = o3d.geometry.LineSet()
        line_set.points = o3d.utility.Vector3dVector(pts)
        line_set.lines = o3d.utility.Vector2iVector(idx)
        line_set.colors = o3d.utility.Vector3dVector(clr)
        geometries.append(line_set)

    print("Opening 3D viewer... (close window to continue)")
    o3d.visualization.draw_geometries(
        geometries,
        window_name="3D Sensor Map",
        width=1280,
        height=720,
    )


def visualize_2d(sensor_data, sensor_vmin=None, sensor_vmax=None):
    """2D heatmap visualization using matplotlib."""
    try:
        import matplotlib
        matplotlib.use('TkAgg')  # interactive backend
        import matplotlib.pyplot as plt
        import matplotlib.cm as cm
    except ImportError:
        print("ERROR: matplotlib not installed. Install with: pip install matplotlib")
        return

    # Load floor plan if available
    npy_path = os.path.join(DATA_DIR, "floor_plan.npy")
    meta_path = os.path.join(DATA_DIR, "map_meta.json")
    has_floor_plan = os.path.exists(npy_path) and os.path.exists(meta_path)

    fig, ax = plt.subplots(1, 1, figsize=(12, 10))

    if has_floor_plan:
        grid = np.load(npy_path)
        with open(meta_path, 'r') as f:
            meta = json.load(f)

        x_min = meta["origin_x"]
        y_min = meta["origin_y"]
        x_max = x_min + meta["width"] * meta["resolution"]
        y_max = y_min + meta["height"] * meta["resolution"]

        # Normalize grid for display
        grid_norm = np.clip(grid, 0, np.percentile(grid[grid > 0], 95) if np.any(grid > 0) else 1)
        grid_img = 255 - (grid_norm / grid_norm.max() * 255).astype(np.uint8)

        ax.imshow(grid_img, cmap='gray', origin='lower',
                  extent=[x_min, x_max, y_min, y_max], alpha=0.6)
        print("Floor plan loaded as background.")
    else:
        print("No floor plan found. Showing sensor data on blank background.")

    # Plot route path
    all_x = [d["x"] for d in sensor_data]
    all_y = [d["y"] for d in sensor_data]
    ax.plot(all_x, all_y, 'k--', linewidth=1, alpha=0.5, label='Route')

    # Plot sensor readings as colored circles
    valid_data = [d for d in sensor_data if d["sensor_value"] is not None]
    if valid_data:
        sx = [d["x"] for d in valid_data]
        sy = [d["y"] for d in valid_data]
        sv = [d["sensor_value"] for d in valid_data]

        scatter = ax.scatter(sx, sy, c=sv, cmap='RdYlBu_r', s=200,
                             edgecolors='black', linewidths=1.5, zorder=5,
                             vmin=sensor_vmin, vmax=sensor_vmax)
        cbar = plt.colorbar(scatter, ax=ax, shrink=0.8)
        cbar.set_label('Sensor Value', fontsize=12)

        # Label waypoints
        for d in valid_data:
            ax.annotate(f'{d["waypoint"]}', (d["x"], d["y"]), fontsize=9,
                        fontweight='bold', ha='center', va='center',
                        color='white', zorder=6)
    else:
        # Just plot waypoints without sensor colors
        ax.scatter(all_x, all_y, c='red', s=100, edgecolors='black',
                   linewidths=1, zorder=5, label='Waypoints')

    ax.set_xlabel('X (m)', fontsize=12)
    ax.set_ylabel('Y (m)', fontsize=12)
    ax.set_title('2D Sensor Heatmap', fontsize=14)
    ax.set_aspect('equal')
    ax.legend(loc='upper right')
    ax.grid(True, alpha=0.3)

    # Save and show
    png_path = os.path.join(DATA_DIR, "sensor_heatmap_2d.png")
    fig.savefig(png_path, dpi=150, bbox_inches='tight')
    print(f"Saved 2D heatmap to {png_path}")

    plt.show()


def main():
    import argparse
    parser = argparse.ArgumentParser(description="Sensor Map Visualization")
    parser.add_argument("--mode", choices=["2d", "3d", "both"], default="both",
                        help="Visualization mode (default: both)")
    parser.add_argument("--cloud-only", action="store_true",
                        help="View point cloud only (no sensor data needed)")
    parser.add_argument("--vmin", type=float, default=None,
                        help="Min sensor value for color scale (default: auto from data)")
    parser.add_argument("--vmax", type=float, default=None,
                        help="Max sensor value for color scale (default: auto from data)")
    args = parser.parse_args()

    if args.cloud_only:
        visualize_cloud_only()
        return

    sensor_data = load_sensor_data()
    if sensor_data is None:
        return

    # Print sensor value summary table
    print(f"\n  {'Waypoint':<10} {'Position':<16} {'Sensor Value'}")
    print(f"  {'--------':<10} {'--------':<16} {'------------'}")
    for d in sensor_data:
        v_str = f"{d['sensor_value']:.1f}" if d["sensor_value"] is not None else "N/A"
        print(f"  #{d['waypoint']:<9} ({d['x']:.2f}, {d['y']:.2f}){'':>4} {v_str}")
    print()

    if args.mode in ("3d", "both"):
        visualize_3d(sensor_data, sensor_vmin=args.vmin, sensor_vmax=args.vmax)

    if args.mode in ("2d", "both"):
        visualize_2d(sensor_data, sensor_vmin=args.vmin, sensor_vmax=args.vmax)


if __name__ == "__main__":
    main()
