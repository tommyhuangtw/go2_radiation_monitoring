"""
3D Sensor Overlay + 2D Heatmap — unified visualisation.

Renders sensor data as:
  3D: Colored spheres + cylinders + interpolated mesh surface + LiDAR cloud
  2D: IDW heatmap over LiDAR floor plan (matplotlib)

Both views share the same 4-stop colormap (blue → cyan → yellow → red).

Usage:
    python sensor_monitor/sensor_surface_sphere_overlay.py                  # both (default)
    python sensor_monitor/sensor_surface_sphere_overlay.py --mode 3d
    python sensor_monitor/sensor_surface_sphere_overlay.py --mode 2d
    python sensor_monitor/sensor_surface_sphere_overlay.py --surface-lift 0.3 --z-scale 0.003
"""

import os
import csv
import json

import numpy as np

DATA_DIR = os.path.join(os.path.dirname(__file__), "data")


# ---------------------------------------------------------------------------
# Color mapping  (shared by 2D and 3D)
# ---------------------------------------------------------------------------

_CSTOP1 = 1.0 / 3.0
_CSTOP2 = 2.0 / 3.0
_SENSOR_COLOR_STOPS = [
    (0.0, (0.0, 0.0, 1.0)),      # blue
    (_CSTOP1, (0.0, 1.0, 1.0)),  # cyan
    (_CSTOP2, (1.0, 1.0, 0.0)),  # yellow
    (1.0, (1.0, 0.0, 0.0)),      # red
]

def sensor_to_color(value, vmin, vmax):
    """Map sensor value to RGB: blue (low) -> cyan -> yellow -> red (high)."""
    if vmax == vmin:
        t = 0.5
    else:
        t = (value - vmin) / (vmax - vmin)
    t = max(0.0, min(1.0, t))

    if t < _CSTOP1:
        s = t / _CSTOP1
        r, g, b = 0.0, s, 1.0 - s
    elif t < _CSTOP2:
        s = (t - _CSTOP1) / (_CSTOP2 - _CSTOP1)
        r, g, b = s, 1.0, 0.0
    else:
        s = (t - _CSTOP2) / (1.0 - _CSTOP2)
        r, g, b = 1.0, 1.0 - s, 0.0

    return (r, g, b)


def _sensor_to_color_array(values, vmin, vmax):
    """Vectorised version of sensor_to_color for numpy arrays."""
    t = np.clip((values - vmin) / max(vmax - vmin, 1e-9), 0.0, 1.0)
    r = np.zeros_like(t)
    g = np.zeros_like(t)
    b = np.zeros_like(t)

    m1 = t < _CSTOP1
    s1 = t[m1] / _CSTOP1
    r[m1], g[m1], b[m1] = 0.0, s1, 1.0 - s1

    m2 = (t >= _CSTOP1) & (t < _CSTOP2)
    s2 = (t[m2] - _CSTOP1) / (_CSTOP2 - _CSTOP1)
    r[m2], g[m2], b[m2] = s2, 1.0, 0.0

    m3 = t >= _CSTOP2
    s3 = (t[m3] - _CSTOP2) / (1.0 - _CSTOP2)
    r[m3], g[m3], b[m3] = 1.0, 1.0 - s3, 0.0

    return np.column_stack([r, g, b])


def _make_matplotlib_cmap():
    """Build a matplotlib LinearSegmentedColormap matching the 3D colormap."""
    from matplotlib.colors import LinearSegmentedColormap
    positions = [c[0] for c in _SENSOR_COLOR_STOPS]
    rgb = [c[1] for c in _SENSOR_COLOR_STOPS]
    return LinearSegmentedColormap.from_list("sensor_cmap", list(zip(positions, rgb)))


# ---------------------------------------------------------------------------
# 3D Geometry builders
# ---------------------------------------------------------------------------

def _make_sphere_mesh(center, radius, color, resolution=12):
    """Create a colored TriangleMesh sphere with normals."""
    import open3d as o3d
    mesh = o3d.geometry.TriangleMesh.create_sphere(radius=radius, resolution=resolution)
    mesh.translate(center)
    mesh.paint_uniform_color(color)
    mesh.compute_vertex_normals()
    return mesh


def _make_cylinder_mesh(x, y, z_bottom, z_top, radius=0.015, color=(0.5, 0.5, 0.5)):
    """Create a vertical cylinder TriangleMesh between two Z levels."""
    import open3d as o3d
    height = abs(z_top - z_bottom)
    if height < 1e-4:
        height = 1e-4
    mesh = o3d.geometry.TriangleMesh.create_cylinder(radius=radius, height=height,
                                                      resolution=10, split=1)
    mesh.translate([x, y, (z_bottom + z_top) / 2.0])
    mesh.paint_uniform_color(color)
    mesh.compute_vertex_normals()
    return mesh


def _build_surface_mesh(valid_data, vmin, vmax, base_z,
                        grid_res=0.06, influence_radius=1.2, z_scale=0.002):
    """Build a triangulated mesh surface coloured and shaped by sensor values."""
    import open3d as o3d

    xs = np.array([d["x"] for d in valid_data], dtype=float)
    ys = np.array([d["y"] for d in valid_data], dtype=float)
    vals = np.array([d["sensor_value"] for d in valid_data], dtype=float)

    if len(xs) == 0:
        return None

    pad = max(grid_res * 3.0, 0.3)
    gx = np.arange(xs.min() - pad, xs.max() + pad + grid_res, grid_res)
    gy = np.arange(ys.min() - pad, ys.max() + pad + grid_res, grid_res)
    XX, YY = np.meshgrid(gx, gy)
    ny, nx = XX.shape

    dx = XX[..., None] - xs[None, None, :]
    dy = YY[..., None] - ys[None, None, :]
    dist = np.sqrt(dx * dx + dy * dy)

    min_dist = dist.min(axis=2)
    mask = min_dist <= influence_radius

    weights = 1.0 / np.maximum(dist, 1e-6) ** 2
    weights_sum = weights.sum(axis=2)
    VV = (weights * vals[None, None, :]).sum(axis=2) / np.maximum(weights_sum, 1e-9)

    z_range = max(vmax - vmin, 1e-9)
    t = np.clip((VV - vmin) / z_range, 0.0, 1.0)
    ZZ = base_z + t * z_scale * z_range

    idx_map = np.full((ny, nx), -1, dtype=int)
    idx_map[mask] = np.arange(mask.sum())

    verts = np.column_stack([XX[mask].ravel(), YY[mask].ravel(), ZZ[mask].ravel()])
    colors = _sensor_to_color_array(VV[mask].ravel(), vmin, vmax)

    triangles = []
    for iy in range(ny - 1):
        for ix in range(nx - 1):
            i00 = idx_map[iy, ix]
            i10 = idx_map[iy, ix + 1]
            i01 = idx_map[iy + 1, ix]
            i11 = idx_map[iy + 1, ix + 1]
            if i00 < 0 or i10 < 0 or i01 < 0 or i11 < 0:
                continue
            triangles.append([i00, i01, i10])
            triangles.append([i10, i01, i11])

    if not triangles:
        return None

    mesh = o3d.geometry.TriangleMesh()
    mesh.vertices = o3d.utility.Vector3dVector(verts)
    mesh.triangles = o3d.utility.Vector3iVector(np.array(triangles, dtype=np.int32))
    mesh.vertex_colors = o3d.utility.Vector3dVector(colors)
    mesh.compute_vertex_normals()
    return mesh


def _make_route_tube(p0, p1, radius, color, o3d):
    """Create a cylinder tube mesh connecting two 3D points."""
    seg = p1 - p0
    length = np.linalg.norm(seg)
    if length < 1e-5:
        return None
    mid = (p0 + p1) / 2.0
    seg_dir = seg / length

    cyl = o3d.geometry.TriangleMesh.create_cylinder(
        radius=radius, height=length, resolution=8, split=1)

    z_axis = np.array([0.0, 0.0, 1.0])
    v = np.cross(z_axis, seg_dir)
    s = np.linalg.norm(v)
    c_dot = np.dot(z_axis, seg_dir)
    if s > 1e-6:
        vx = np.array([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]])
        R = np.eye(3) + vx + vx @ vx * (1.0 - c_dot) / (s * s)
        cyl.rotate(R, center=[0, 0, 0])
    elif c_dot < 0:
        cyl.rotate(o3d.geometry.get_rotation_matrix_from_xyz([np.pi, 0, 0]),
                   center=[0, 0, 0])
    cyl.translate(mid)
    cyl.paint_uniform_color(color)
    cyl.compute_vertex_normals()
    return cyl


# ---------------------------------------------------------------------------
# Data loading
# ---------------------------------------------------------------------------

def _load_sensor_data_aligned(phase_filter=None):
    """Load sensor data and align XY to waypoints.json when available.

    Args:
        phase_filter: "low", "high", or None (all data). Filters by
                      measurement_phase column if present.
    """
    csv_path = os.path.join(DATA_DIR, "sensor_data.csv")
    if not os.path.exists(csv_path):
        print(f"ERROR: {csv_path} not found")
        return None

    data = []
    with open(csv_path, "r") as f:
        reader = csv.DictReader(f)
        for row in reader:
            phase = row.get("measurement_phase", "")
            if phase_filter and phase and phase != phase_filter:
                continue
            raw = row.get("sensor_value", "")
            # Map phase to fixed physical heights for visualization
            if phase == "mid":
                z = 0.56
            elif phase == "elevated":
                z = 1.2
            else:
                z = float(row.get("altitude_mm", 0)) / 1000.0
            data.append({
                "waypoint": int(row.get("waypoint", 0)),
                "x": float(row["x"]),
                "y": float(row["y"]),
                "z": z,
                "yaw": float(row["yaw"]),
                "sensor_value": float(raw) if raw else None,
            })

    wp_path = os.path.join(DATA_DIR, "waypoints.json")
    if os.path.exists(wp_path):
        with open(wp_path, "r") as f:
            wp_data = json.load(f)
        aligned = 0
        for d in data:
            idx = d["waypoint"] - 1
            if 0 <= idx < len(wp_data):
                d["x"] = wp_data[idx]["x"]
                d["y"] = wp_data[idx]["y"]
                aligned += 1
        print(f"Aligned {aligned}/{len(data)} positions to waypoints frame")

    valid = [d for d in data if d["sensor_value"] is not None]
    print(f"Loaded {len(data)} points ({len(valid)} with sensor values)")
    return data


# ---------------------------------------------------------------------------
# 2D IDW interpolation (shared with 2D heatmap)
# ---------------------------------------------------------------------------

def _idw_interpolate_2d(valid_data, grid_res=0.06, influence_radius=1.2):
    """IDW interpolation on XY grid. Returns XX, YY, VV, mask."""
    xs = np.array([d["x"] for d in valid_data], dtype=float)
    ys = np.array([d["y"] for d in valid_data], dtype=float)
    vals = np.array([d["sensor_value"] for d in valid_data], dtype=float)

    pad = max(grid_res * 3.0, 0.3)
    gx = np.arange(xs.min() - pad, xs.max() + pad + grid_res, grid_res)
    gy = np.arange(ys.min() - pad, ys.max() + pad + grid_res, grid_res)
    XX, YY = np.meshgrid(gx, gy)

    dx = XX[..., None] - xs[None, None, :]
    dy = YY[..., None] - ys[None, None, :]
    dist = np.sqrt(dx * dx + dy * dy)

    min_dist = dist.min(axis=2)
    mask = min_dist <= influence_radius

    weights = 1.0 / np.maximum(dist, 1e-6) ** 2
    weights_sum = weights.sum(axis=2)
    VV = (weights * vals[None, None, :]).sum(axis=2) / np.maximum(weights_sum, 1e-9)

    return XX, YY, VV, mask


# ---------------------------------------------------------------------------
# 3D visualisation
# ---------------------------------------------------------------------------

def visualize_3d(sensor_data, sensor_vmin=None, sensor_vmax=None,
                 sphere_radius=0.05, surface_grid=0.06,
                 surface_radius=1.2, surface_lift=0.15,
                 surface_gap=0.10, z_scale=0.002,
                 hide_cloud=False):
    """Render cloud + spheres + cylinders + mesh surface in one Open3D window."""
    try:
        import open3d as o3d
    except ImportError:
        print("ERROR: open3d not installed.  pip install open3d")
        return

    # --- Load LiDAR point cloud ---
    cloud = None
    ply_path = os.path.join(DATA_DIR, "point_cloud.ply")
    if os.path.exists(ply_path):
        cloud = o3d.io.read_point_cloud(ply_path)
        n_pts = len(cloud.points)
        print(f"Loaded point cloud: {n_pts:,} points")

        pts = np.asarray(cloud.points)
        z = pts[:, 2]
        z_norm = (z - z.min()) / (z.max() - z.min() + 1e-9)
        # Rainbow coloring by height
        c = np.zeros((n_pts, 3))
        m1 = z_norm < 0.25
        s1 = z_norm[m1] / 0.25
        c[m1] = np.column_stack([np.zeros_like(s1), s1, np.ones_like(s1)])
        m2 = (z_norm >= 0.25) & (z_norm < 0.5)
        s2 = (z_norm[m2] - 0.25) / 0.25
        c[m2] = np.column_stack([np.zeros_like(s2), np.ones_like(s2), 1.0 - s2])
        m3 = (z_norm >= 0.5) & (z_norm < 0.75)
        s3 = (z_norm[m3] - 0.5) / 0.25
        c[m3] = np.column_stack([s3, np.ones_like(s3), np.zeros_like(s3)])
        m4 = z_norm >= 0.75
        s4 = (z_norm[m4] - 0.75) / 0.25
        c[m4] = np.column_stack([np.ones_like(s4), 1.0 - s4, np.zeros_like(s4)])
        cloud.colors = o3d.utility.Vector3dVector(c)
        cloud.estimate_normals(
            search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.1, max_nn=30))

    # --- Sensor value range ---
    valid_data = [d for d in sensor_data if d["sensor_value"] is not None]
    if not valid_data:
        print("No sensor data to visualize")
        if cloud is not None:
            o3d.visualization.draw_geometries([cloud], window_name="3D Overlay")
        return

    sensor_values = [d["sensor_value"] for d in valid_data]
    vmin = sensor_vmin if sensor_vmin is not None else min(sensor_values)
    vmax = sensor_vmax if sensor_vmax is not None else max(sensor_values)

    if cloud is not None:
        cloud_z = np.asarray(cloud.points)[:, 2]
        floor_z = float(np.percentile(cloud_z, 10))
    else:
        floor_z = 0.0

    # --- Spheres & cylinders ---
    sphere_meshes = []
    cylinder_meshes = []
    sphere_zs = []
    for d in valid_data:
        color = list(sensor_to_color(d["sensor_value"], vmin, vmax))
        z_height = d["z"]
        sphere_zs.append(z_height)
        sphere_meshes.append(
            _make_sphere_mesh([d["x"], d["y"], z_height], sphere_radius, color))
        cyl_color = [ch * 0.6 for ch in color]
        cylinder_meshes.append(
            _make_cylinder_mesh(d["x"], d["y"], floor_z, z_height,
                                radius=0.015, color=cyl_color))

    # --- Surface mesh ---
    top_sphere_z = max(sphere_zs) if sphere_zs else floor_z
    base_z = top_sphere_z + surface_lift

    surface_mesh = _build_surface_mesh(
        valid_data, vmin, vmax,
        base_z=base_z,
        grid_res=surface_grid,
        influence_radius=surface_radius,
        z_scale=z_scale,
    )

    # --- Assemble geometries ---
    geometries = []
    if cloud is not None and not hide_cloud:
        geometries.append(cloud)
    geometries.extend(cylinder_meshes)
    geometries.extend(sphere_meshes)

    if surface_mesh is not None:
        geometries.append(surface_mesh)
        print(f"Surface base_z={base_z:.3f} m  (floor={floor_z:.3f}, "
              f"top_sphere={top_sphere_z:.3f}, lift={surface_lift:.3f})")

    # Route: thick red tube on the floor plane
    if len(valid_data) >= 2:
        route_z = floor_z + 0.1
        for i in range(len(valid_data) - 1):
            p0 = np.array([valid_data[i]["x"], valid_data[i]["y"], route_z])
            p1 = np.array([valid_data[i + 1]["x"], valid_data[i + 1]["y"], route_z])
            tube = _make_route_tube(p0, p1, radius=0.02, color=[1.0, 0.0, 0.0], o3d=o3d)
            if tube is not None:
                geometries.append(tube)

    # Coordinate frame
    axes = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.3, origin=[0, 0, floor_z])
    geometries.append(axes)

    # --- Export PLY (merged mesh) ---
    _export_ply(geometries, o3d)

    # --- Export interactive HTML (plotly) ---
    _export_html(valid_data, vmin, vmax, floor_z, surface_mesh,
                 sphere_radius, cloud, hide_cloud)

    # --- Viewer ---
    print("Opening 3D viewer... (close window to continue)")
    vis = o3d.visualization.Visualizer()
    vis.create_window(window_name="3D Sensor Overlay", width=1400, height=900)
    for g in geometries:
        vis.add_geometry(g)

    opt = vis.get_render_option()
    opt.background_color = np.array([1.0, 1.0, 1.0])
    opt.point_size = 1.5
    opt.mesh_show_back_face = True
    opt.light_on = True

    cx = np.mean([d["x"] for d in valid_data])
    cy = np.mean([d["y"] for d in valid_data])
    ctr = vis.get_view_control()
    ctr.set_lookat([cx, cy, floor_z + 0.3])
    ctr.set_front([0.0, -0.5, -0.85])
    ctr.set_up([0.0, 0.0, 1.0])
    ctr.set_zoom(0.45)

    vis.run()
    vis.destroy_window()


# ---------------------------------------------------------------------------
# 3D export helpers
# ---------------------------------------------------------------------------

def _export_ply(geometries, o3d):
    """Merge all meshes and point clouds into a single PLY file."""
    merged_mesh = o3d.geometry.TriangleMesh()
    merged_pcd = o3d.geometry.PointCloud()

    for g in geometries:
        if isinstance(g, o3d.geometry.TriangleMesh):
            merged_mesh += g
        elif isinstance(g, o3d.geometry.PointCloud):
            merged_pcd += g

    # Save mesh PLY
    mesh_path = os.path.join(DATA_DIR, "sensor_3d_scene.ply")
    if len(merged_mesh.vertices) > 0:
        # Convert point cloud to mesh vertices so everything is in one file
        if len(merged_pcd.points) > 0:
            # Append point cloud as zero-area triangles (degenerate but preserves color)
            pcd_mesh = o3d.geometry.TriangleMesh()
            pcd_pts = np.asarray(merged_pcd.points)
            pcd_colors = np.asarray(merged_pcd.colors) if merged_pcd.has_colors() else None
            n_existing = len(merged_mesh.vertices)
            all_verts = np.vstack([np.asarray(merged_mesh.vertices), pcd_pts])
            merged_mesh.vertices = o3d.utility.Vector3dVector(all_verts)
            if pcd_colors is not None and merged_mesh.has_vertex_colors():
                all_colors = np.vstack([np.asarray(merged_mesh.vertex_colors), pcd_colors])
                merged_mesh.vertex_colors = o3d.utility.Vector3dVector(all_colors)

        o3d.io.write_triangle_mesh(mesh_path, merged_mesh, write_vertex_colors=True)
        print(f"Exported PLY: {mesh_path}  ({len(merged_mesh.vertices):,} vertices)")
    elif len(merged_pcd.points) > 0:
        o3d.io.write_point_cloud(mesh_path, merged_pcd)
        print(f"Exported PLY: {mesh_path}  ({len(merged_pcd.points):,} points)")


def _export_html(valid_data, vmin, vmax, floor_z, surface_mesh,
                 sphere_radius, cloud, hide_cloud, html_filename="sensor_3d_interactive.html"):
    """Export an interactive HTML file using plotly."""
    try:
        import plotly.graph_objects as go
    except ImportError:
        print("WARNING: plotly not installed, skipping HTML export.  pip install plotly")
        return

    fig = go.Figure()

    # --- LiDAR point cloud (downsampled for performance) ---
    if cloud is not None and not hide_cloud:
        pts = np.asarray(cloud.points)
        colors = np.asarray(cloud.colors) if cloud.has_colors() else None

        # Downsample to ~50K points for reasonable file size / render speed
        max_pts = 50000
        if len(pts) > max_pts:
            idx = np.random.default_rng(42).choice(len(pts), max_pts, replace=False)
            pts = pts[idx]
            if colors is not None:
                colors = colors[idx]

        cloud_colors = [f"rgb({int(c[0]*255)},{int(c[1]*255)},{int(c[2]*255)})"
                        for c in colors] if colors is not None else "grey"
        fig.add_trace(go.Scatter3d(
            x=pts[:, 0], y=pts[:, 1], z=pts[:, 2],
            mode="markers",
            marker=dict(size=1, color=cloud_colors, opacity=0.25),
            name="LiDAR",
            hoverinfo="skip",
        ))

    # --- Interpolated surface mesh (sensor heatmap draped over environment) ---
    if surface_mesh is not None:
        import open3d as o3d
        verts = np.asarray(surface_mesh.vertices)
        tris = np.asarray(surface_mesh.triangles)
        v_colors = np.asarray(surface_mesh.vertex_colors)

        fig.add_trace(go.Mesh3d(
            x=verts[:, 0], y=verts[:, 1], z=verts[:, 2],
            i=tris[:, 0], j=tris[:, 1], k=tris[:, 2],
            vertexcolor=[f"rgb({int(c[0]*255)},{int(c[1]*255)},{int(c[2]*255)})"
                         for c in v_colors],
            opacity=0.8,
            name="Sensor Surface",
            hoverinfo="skip",
        ))

    # --- Sensor markers (large, prominent) ---
    sx = [d["x"] for d in valid_data]
    sy = [d["y"] for d in valid_data]
    sz = [d["z"] for d in valid_data]
    sv = [d["sensor_value"] for d in valid_data]
    sphere_colors = [f"rgb({int(c[0]*255)},{int(c[1]*255)},{int(c[2]*255)})"
                     for c in [sensor_to_color(v, vmin, vmax) for v in sv]]

    fig.add_trace(go.Scatter3d(
        x=sx, y=sy, z=sz,
        mode="markers+text",
        marker=dict(
            size=14,
            color=sv,
            colorscale=[[0, "rgb(0,0,255)"], [0.333, "rgb(0,255,255)"],
                        [0.667, "rgb(255,255,0)"], [1, "rgb(255,0,0)"]],
            cmin=vmin, cmax=vmax,
            colorbar=dict(title="Sensor", thickness=15, len=0.5),
            line=dict(width=2, color="white"),
            opacity=0.95,
        ),
        text=[f"WP{d['waypoint']}: {d['sensor_value']:.0f}" for d in valid_data],
        textposition="top center",
        textfont=dict(size=11, color="white"),
        name="Sensor Points",
    ))

    # --- Vertical stems from floor to sensor point ---
    for d in valid_data:
        color = sensor_to_color(d["sensor_value"], vmin, vmax)
        rgb = f"rgb({int(color[0]*255)},{int(color[1]*255)},{int(color[2]*255)})"
        fig.add_trace(go.Scatter3d(
            x=[d["x"], d["x"]], y=[d["y"], d["y"]], z=[floor_z, d["z"]],
            mode="lines",
            line=dict(width=5, color=rgb),
            showlegend=False,
            hoverinfo="skip",
        ))

    # --- Route line ---
    route_z = floor_z + 0.1
    rx = [d["x"] for d in valid_data]
    ry = [d["y"] for d in valid_data]
    rz = [route_z] * len(valid_data)
    fig.add_trace(go.Scatter3d(
        x=rx, y=ry, z=rz,
        mode="lines",
        line=dict(width=5, color="red"),
        name="Route",
    ))

    # --- Layout ---
    fig.update_layout(
        title="3D Sensor Overlay (Interactive)",
        scene=dict(
            xaxis_title="X (m)",
            yaxis_title="Y (m)",
            zaxis_title="Z (m)",
            aspectmode="data",
            bgcolor="white",
        ),
        legend=dict(x=0.02, y=0.98),
        margin=dict(l=0, r=0, t=40, b=0),
    )

    html_path = os.path.join(DATA_DIR, html_filename)
    fig.write_html(html_path, include_plotlyjs=True)
    # Fix: ensure 100% height works in standalone and iframe contexts
    with open(html_path, "r") as f:
        html_content = f.read()
    html_content = html_content.replace(
        "<head>", "<head><style>html,body{height:100%;margin:0;}</style>", 1)
    with open(html_path, "w") as f:
        f.write(html_content)
    print(f"Exported HTML: {html_path}")


# ---------------------------------------------------------------------------
# Headless 3D HTML generation (no GUI window)
# ---------------------------------------------------------------------------

def generate_3d_html_only(sensor_vmin=None, sensor_vmax=None,
                          sphere_radius=0.12, surface_grid=0.06,
                          surface_radius=1.2, surface_lift=0.15,
                          z_scale=0.002, hide_cloud=False,
                          phase=None):
    """Generate interactive HTML (and PLY) without opening a viewer window.

    Designed to be called from a web server backend thread.
    Args:
        phase: "low", "high", or None (all data).
    Returns the path to the generated HTML file, or raises on error.
    """
    import open3d as o3d

    sensor_data = _load_sensor_data_aligned(phase_filter=phase)
    if sensor_data is None:
        raise RuntimeError("sensor_data.csv not found")

    valid_data = [d for d in sensor_data if d["sensor_value"] is not None]
    if not valid_data:
        raise RuntimeError("No valid sensor data")

    sensor_values = [d["sensor_value"] for d in valid_data]
    vmin = sensor_vmin if sensor_vmin is not None else min(sensor_values)
    vmax = sensor_vmax if sensor_vmax is not None else max(sensor_values)

    # Load point cloud
    cloud = None
    ply_path = os.path.join(DATA_DIR, "point_cloud.ply")
    if os.path.exists(ply_path):
        cloud = o3d.io.read_point_cloud(ply_path)
        pts = np.asarray(cloud.points)
        z = pts[:, 2]
        floor_z = float(np.percentile(z, 10))

        # Remove floor points for cleaner 3D view
        keep = z > (floor_z + 0.15)
        pts = pts[keep]
        cloud.points = o3d.utility.Vector3dVector(pts)

        # Neutral gray with subtle height shading (dark → lighter)
        z = pts[:, 2]
        z_norm = (z - z.min()) / (z.max() - z.min() + 1e-9)
        gray = 0.3 + 0.4 * z_norm
        c = np.column_stack([gray, gray, gray])
        cloud.colors = o3d.utility.Vector3dVector(c)
    else:
        floor_z = 0.0

    # Build surface mesh
    sphere_zs = [d["z"] for d in valid_data]
    top_sphere_z = max(sphere_zs) if sphere_zs else floor_z
    base_z = top_sphere_z + surface_lift

    surface_mesh = _build_surface_mesh(
        valid_data, vmin, vmax,
        base_z=base_z,
        grid_res=surface_grid,
        influence_radius=surface_radius,
        z_scale=z_scale,
    )

    suffix = f"_{phase}" if phase else ""
    html_filename = f"sensor_3d_interactive{suffix}.html"
    _export_html(valid_data, vmin, vmax, floor_z, surface_mesh,
                 sphere_radius, cloud, hide_cloud, html_filename=html_filename)

    html_path = os.path.join(DATA_DIR, html_filename)
    return html_path


# ---------------------------------------------------------------------------
# 2D visualisation
# ---------------------------------------------------------------------------

def visualize_2d(sensor_data, sensor_vmin=None, sensor_vmax=None,
                 surface_grid=0.06, surface_radius=1.2,
                 show_window=True, backend="TkAgg",
                 output_filename="sensor_heatmap_2d.png"):
    """2D heatmap with IDW interpolation over LiDAR floor plan."""
    try:
        import matplotlib
        matplotlib.use(backend)
        import matplotlib.pyplot as plt
    except ImportError:
        print("ERROR: matplotlib not installed.  pip install matplotlib")
        return

    cmap = _make_matplotlib_cmap()

    valid_data = [d for d in sensor_data if d["sensor_value"] is not None]
    if not valid_data:
        print("No sensor data for 2D heatmap")
        return

    sensor_values = [d["sensor_value"] for d in valid_data]
    vmin = sensor_vmin if sensor_vmin is not None else min(sensor_values)
    vmax = sensor_vmax if sensor_vmax is not None else max(sensor_values)

    fig, ax = plt.subplots(1, 1, figsize=(12, 10))

    # --- Floor plan background ---
    npy_path = os.path.join(DATA_DIR, "floor_plan.npy")
    meta_path = os.path.join(DATA_DIR, "map_meta.json")
    if os.path.exists(npy_path) and os.path.exists(meta_path):
        grid = np.load(npy_path)
        with open(meta_path, "r") as f:
            meta = json.load(f)
        x_min = meta["origin_x"]
        y_min = meta["origin_y"]
        x_max = x_min + meta["width"] * meta["resolution"]
        y_max = y_min + meta["height"] * meta["resolution"]

        positive = grid[grid > 0]
        clip_val = np.percentile(positive, 95) if positive.size > 0 else 1.0
        grid_norm = np.clip(grid, 0, clip_val)
        grid_img = 255 - (grid_norm / max(grid_norm.max(), 1e-9) * 255).astype(np.uint8)

        ax.imshow(grid_img, cmap="gray", origin="lower",
                  extent=[x_min, x_max, y_min, y_max], alpha=0.5)
        print("Floor plan loaded as background.")

    # --- IDW heatmap layer ---
    XX, YY, VV, mask = _idw_interpolate_2d(valid_data, grid_res=surface_grid,
                                            influence_radius=surface_radius)
    heatmap = np.full_like(VV, np.nan)
    heatmap[mask] = VV[mask]
    extent = [XX.min(), XX.max(), YY.min(), YY.max()]
    ax.imshow(heatmap, cmap=cmap, origin="lower", extent=extent,
              alpha=0.55, vmin=vmin, vmax=vmax, interpolation="bilinear")

    # --- Route ---
    all_x = [d["x"] for d in sensor_data]
    all_y = [d["y"] for d in sensor_data]
    ax.plot(all_x, all_y, color="red", linewidth=2, alpha=0.7, label="Route")

    # --- Scatter points ---
    sx = [d["x"] for d in valid_data]
    sy = [d["y"] for d in valid_data]
    sv = [d["sensor_value"] for d in valid_data]

    scatter = ax.scatter(sx, sy, c=sv, cmap=cmap, s=180,
                         edgecolors="black", linewidths=1.2, zorder=5,
                         vmin=vmin, vmax=vmax)
    cbar = plt.colorbar(scatter, ax=ax, shrink=0.8)
    cbar.set_label("Sensor Value", fontsize=12)

    for d in valid_data:
        ax.annotate(f'{d["waypoint"]}', (d["x"], d["y"]), fontsize=8,
                    fontweight="bold", ha="center", va="center",
                    color="white", zorder=6)

    ax.set_xlabel("X (m)", fontsize=12)
    ax.set_ylabel("Y (m)", fontsize=12)
    ax.set_title("2D Sensor Heatmap (IDW)", fontsize=14)
    ax.set_aspect("equal")
    ax.legend(loc="upper right")
    ax.grid(True, alpha=0.3)

    png_path = os.path.join(DATA_DIR, output_filename)
    fig.savefig(png_path, dpi=150, bbox_inches="tight")
    print(f"Saved 2D heatmap to {png_path}")

    if show_window:
        plt.show()
    else:
        plt.close(fig)

    return png_path


def generate_2d_heatmap_only(sensor_vmin=None, sensor_vmax=None,
                             surface_grid=0.06, surface_radius=1.2,
                             phase=None):
    """Generate 2D heatmap PNG without opening a GUI window.

    Designed to be called from web backend threads.
    Args:
        phase: "low", "high", or None (all data).
    Returns the path to the generated PNG, or raises on error.
    """
    sensor_data = _load_sensor_data_aligned(phase_filter=phase)
    if sensor_data is None:
        raise RuntimeError("sensor_data.csv not found")

    suffix = f"_{phase}" if phase else ""
    output_filename = f"sensor_heatmap_2d{suffix}.png"
    png_path = visualize_2d(
        sensor_data,
        sensor_vmin=sensor_vmin,
        sensor_vmax=sensor_vmax,
        surface_grid=surface_grid,
        surface_radius=surface_radius,
        show_window=False,
        backend="Agg",
        output_filename=output_filename,
    )
    if not png_path or not os.path.exists(png_path):
        raise RuntimeError("Failed to generate 2D heatmap PNG")
    return png_path


# ---------------------------------------------------------------------------
# CLI entry point
# ---------------------------------------------------------------------------

def main():
    import argparse

    parser = argparse.ArgumentParser(
        description="Sensor Visualisation: 3D overlay + 2D heatmap")
    parser.add_argument("--mode", choices=["2d", "3d", "both"], default="both",
                        help="Visualisation mode (default: both)")
    parser.add_argument("--vmin", type=float, default=None,
                        help="Min sensor value for color scale")
    parser.add_argument("--vmax", type=float, default=None,
                        help="Max sensor value for color scale")
    parser.add_argument("--sphere-radius", type=float, default=0.12,
                        help="Sphere radius in meters")
    parser.add_argument("--surface-grid", type=float, default=0.06,
                        help="Grid resolution for surface / heatmap")
    parser.add_argument("--surface-radius", type=float, default=1.2,
                        help="IDW influence radius")
    parser.add_argument("--surface-lift", type=float, default=0.15,
                        help="Base lift above top sphere (meters)")
    parser.add_argument("--surface-gap", type=float, default=0.10,
                        help="Extra minimum gap above top sphere")
    parser.add_argument("--z-scale", type=float, default=0.002,
                        help="Height scale for 3D surface curvature")
    parser.add_argument("--hide-cloud", action="store_true",
                        help="Hide LiDAR point cloud in 3D view")
    args = parser.parse_args()

    sensor_data = _load_sensor_data_aligned()
    if sensor_data is None:
        return

    if args.mode in ("3d", "both"):
        visualize_3d(
            sensor_data,
            sensor_vmin=args.vmin,
            sensor_vmax=args.vmax,
            sphere_radius=args.sphere_radius,
            surface_grid=args.surface_grid,
            surface_radius=args.surface_radius,
            surface_lift=args.surface_lift,
            surface_gap=args.surface_gap,
            z_scale=args.z_scale,
            hide_cloud=args.hide_cloud,
        )

    if args.mode in ("2d", "both"):
        visualize_2d(
            sensor_data,
            sensor_vmin=args.vmin,
            sensor_vmax=args.vmax,
            surface_grid=args.surface_grid,
            surface_radius=args.surface_radius,
        )


if __name__ == "__main__":
    main()
