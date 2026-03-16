"""
Web server for Go2 Patrol Monitor.

Provides a FastAPI backend with:
- WebSocket for real-time state updates and commands
- MJPEG camera stream
- REST endpoints for map data (floor plan, waypoints, metadata)

Launched via: python sensor_monitor/auto_navigator.py --web
"""

import os
import sys
import json
import struct
import asyncio
import logging
import threading
from queue import Queue, Empty
from pathlib import Path

# Ensure sensor_monitor package is importable
sys.path.insert(0, str(Path(__file__).parent))

from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.responses import HTMLResponse, FileResponse, StreamingResponse, JSONResponse
import uvicorn

DATA_DIR = Path(__file__).parent / "data"
HTML_PATH = Path(__file__).parent / "web_ui.html"

app = FastAPI(title="Go2 Patrol Monitor")

# Set by run_web_server()
navigator = None
video_client = None
_video_lock = threading.Lock()
_bridge_mode = False
_bridge_host = None
_webrtc_frame_queue = Queue(maxsize=2)

# Connected WebSocket clients
connected_clients: set[WebSocket] = set()


# ------------------------------------------------------------------
# REST endpoints
# ------------------------------------------------------------------

@app.get("/", response_class=HTMLResponse)
async def index():
    return HTMLResponse(content=HTML_PATH.read_text(), status_code=200)


@app.get("/api/map_meta")
async def get_map_meta():
    meta_path = DATA_DIR / "map_meta.json"
    if not meta_path.exists():
        return JSONResponse({"error": "map_meta.json not found"}, status_code=404)
    with open(meta_path) as f:
        data = json.load(f)
    return JSONResponse(data, headers={"Cache-Control": "no-store"})


@app.get("/api/waypoints")
async def get_waypoints():
    wp_path = DATA_DIR / "waypoints.json"
    if not wp_path.exists():
        return JSONResponse({"error": "waypoints.json not found"}, status_code=404)
    with open(wp_path) as f:
        data = json.load(f)
    return JSONResponse(data, headers={"Cache-Control": "no-store"})


@app.get("/api/floor_plan")
async def get_floor_plan():
    img_path = DATA_DIR / "floor_plan.png"
    if not img_path.exists():
        return JSONResponse({"error": "floor_plan.png not found"}, status_code=404)
    return FileResponse(str(img_path), media_type="image/png",
                        headers={"Cache-Control": "no-store"})


# ------------------------------------------------------------------
# 3D visualization generation
# ------------------------------------------------------------------

_3d_gen_status = "idle"   # idle | generating | ready | error
_3d_gen_message = ""
_3d_gen_lock = threading.Lock()

_2d_gen_status = "idle"   # idle | generating | ready | error
_2d_gen_message = ""
_2d_gen_lock = threading.Lock()


@app.post("/api/generate_3d")
async def generate_3d():
    global _3d_gen_status, _3d_gen_message
    with _3d_gen_lock:
        if _3d_gen_status == "generating":
            return JSONResponse({"status": "already_running"})
        _3d_gen_status = "generating"
        _3d_gen_message = ""

    def worker():
        global _3d_gen_status, _3d_gen_message
        try:
            from sensor_surface_sphere_overlay import generate_3d_html_only
            generate_3d_html_only()
            _3d_gen_status = "ready"
            _3d_gen_message = "3D visualization generated"
        except Exception as e:
            _3d_gen_status = "error"
            _3d_gen_message = str(e)
            print(f"3D generation error: {e}")

    threading.Thread(target=worker, daemon=True).start()
    return JSONResponse({"status": "generating"})


@app.get("/api/3d_status")
async def get_3d_status():
    return JSONResponse({"status": _3d_gen_status, "message": _3d_gen_message})


@app.get("/api/sensor_3d")
async def get_sensor_3d():
    html_path = DATA_DIR / "sensor_3d_interactive.html"
    if not html_path.exists():
        return JSONResponse({"error": "3D visualization not generated yet"}, status_code=404)
    return FileResponse(str(html_path), media_type="text/html",
                        headers={"Cache-Control": "no-store"})


@app.post("/api/generate_2d")
async def generate_2d():
    global _2d_gen_status, _2d_gen_message
    with _2d_gen_lock:
        if _2d_gen_status == "generating":
            return JSONResponse({"status": "already_running"})
        _2d_gen_status = "generating"
        _2d_gen_message = ""

    def worker():
        global _2d_gen_status, _2d_gen_message
        try:
            from sensor_surface_sphere_overlay import generate_2d_heatmap_only
            generate_2d_heatmap_only()
            _2d_gen_status = "ready"
            _2d_gen_message = "2D heatmap generated"
        except Exception as e:
            _2d_gen_status = "error"
            _2d_gen_message = str(e)
            print(f"2D generation error: {e}")

    threading.Thread(target=worker, daemon=True).start()
    return JSONResponse({"status": "generating"})


@app.get("/api/2d_status")
async def get_2d_status():
    return JSONResponse({"status": _2d_gen_status, "message": _2d_gen_message})


@app.get("/api/sensor_heatmap_2d")
async def get_sensor_heatmap_2d():
    png_path = DATA_DIR / "sensor_heatmap_2d.png"
    if not png_path.exists():
        return JSONResponse({"error": "2D heatmap not generated yet"}, status_code=404)
    return FileResponse(str(png_path), media_type="image/png",
                        headers={"Cache-Control": "no-store"})


@app.get("/api/sensor_data")
async def get_sensor_data():
    """Return sensor_data.csv as JSON for the frontend."""
    csv_path = DATA_DIR / "sensor_data.csv"
    if not csv_path.exists():
        return JSONResponse({"error": "sensor_data.csv not found"}, status_code=404)
    import csv as csv_mod
    data = []
    with open(csv_path, "r") as f:
        reader = csv_mod.DictReader(f)
        for row in reader:
            data.append(row)
    return JSONResponse(data, headers={"Cache-Control": "no-store"})


# ------------------------------------------------------------------
# MJPEG camera stream
# ------------------------------------------------------------------

def _get_video_client():
    """Lazy-init VideoClient (thread-safe). Skipped in bridge/WiFi mode."""
    global video_client
    if _bridge_mode:
        return None
    if video_client is not None:
        return video_client
    with _video_lock:
        if video_client is not None:
            return video_client
        try:
            from unitree_sdk2py.go2.video.video_client import VideoClient
            vc = VideoClient()
            vc.SetTimeout(3.0)
            vc.Init()
            video_client = vc
            print("VideoClient initialized for camera stream")
            return video_client
        except Exception as e:
            print(f"VideoClient init failed: {e}")
            return None


def _init_video_bridge(bridge_host, video_port=9878):
    """Background thread: receive JPEG frames from dds_bridge video relay over TCP."""
    import socket
    while True:
        try:
            sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            sock.settimeout(5.0)
            print(f"Video bridge: connecting to {bridge_host}:{video_port}...")
            sock.connect((bridge_host, video_port))
            sock.settimeout(None)
            print(f"Video bridge: connected!")

            while True:
                # Read 4-byte big-endian length prefix
                header = b""
                while len(header) < 4:
                    chunk = sock.recv(4 - len(header))
                    if not chunk:
                        raise ConnectionError("Video bridge disconnected")
                    header += chunk
                length = struct.unpack('>I', header)[0]

                # Read JPEG data
                data = b""
                while len(data) < length:
                    chunk = sock.recv(min(length - len(data), 65536))
                    if not chunk:
                        raise ConnectionError("Video bridge disconnected")
                    data += chunk

                # Put into frame queue (drop old if full)
                if _webrtc_frame_queue.full():
                    try:
                        _webrtc_frame_queue.get_nowait()
                    except Empty:
                        pass
                _webrtc_frame_queue.put(data)

        except (ConnectionRefusedError, socket.timeout, OSError, ConnectionError) as e:
            print(f"Video bridge: {e} — retrying in 3s...")
        finally:
            try:
                sock.close()
            except OSError:
                pass
        import time
        time.sleep(3)


def _capture_frame():
    """Blocking call to get one JPEG frame from the robot camera."""
    if _bridge_mode:
        # WiFi mode: read from WebRTC frame queue
        try:
            return _webrtc_frame_queue.get(timeout=0.5)
        except Empty:
            return None

    vc = _get_video_client()
    if vc is None:
        return None
    code, data = vc.GetImageSample()
    if code != 0:
        return None
    return bytes(data)


async def _generate_mjpeg():
    """Async generator yielding MJPEG frames."""
    loop = asyncio.get_event_loop()
    while True:
        frame = await loop.run_in_executor(None, _capture_frame)
        if frame is None:
            await asyncio.sleep(0.5)
            continue
        yield (b"--frame\r\n"
               b"Content-Type: image/jpeg\r\n\r\n" + frame + b"\r\n")
        await asyncio.sleep(0.05)  # ~20 FPS cap


@app.get("/video")
async def video_stream():
    return StreamingResponse(
        _generate_mjpeg(),
        media_type="multipart/x-mixed-replace; boundary=frame",
    )


# ------------------------------------------------------------------
# WebSocket: state broadcast + command handling
# ------------------------------------------------------------------

@app.websocket("/ws")
async def websocket_endpoint(ws: WebSocket):
    await ws.accept()
    connected_clients.add(ws)
    try:
        while True:
            data = await ws.receive_text()
            msg = json.loads(data)
            cmd = msg.get("cmd")
            if cmd == "start":
                _handle_start_patrol()
            elif cmd == "next":
                navigator.web_continue()
            elif cmd == "stop":
                navigator.web_emergency_stop()
    except WebSocketDisconnect:
        pass
    finally:
        connected_clients.discard(ws)


async def _broadcast_state():
    """Push navigator state to all connected clients at 5 Hz."""
    while True:
        if navigator and connected_clients:
            state = navigator.get_state_snapshot()
            msg = json.dumps({"type": "state", **state})
            disconnected = set()
            for ws in connected_clients:
                try:
                    await ws.send_text(msg)
                except Exception:
                    disconnected.add(ws)
            connected_clients.difference_update(disconnected)
        await asyncio.sleep(0.2)


@app.on_event("startup")
async def on_startup():
    asyncio.create_task(_broadcast_state())


# ------------------------------------------------------------------
# Patrol thread management
# ------------------------------------------------------------------

_patrol_thread = None


def _handle_start_patrol():
    """Launch patrol in a background thread."""
    global _patrol_thread
    if _patrol_thread and _patrol_thread.is_alive():
        print("Patrol already running, ignoring start command")
        return

    navigator._stop_event.clear()
    navigator._web_mode = True

    def worker():
        try:
            if not navigator.prepare_patrol():
                navigator.nav_state = "error"
                if not navigator.patrol_error:
                    navigator.patrol_error = "Failed to prepare patrol"
                return
            navigator.start_patrol()
        except Exception as e:
            navigator.nav_state = "error"
            navigator.patrol_error = str(e)
            print(f"Patrol thread error: {e}")

    _patrol_thread = threading.Thread(target=worker, daemon=True)
    _patrol_thread.start()
    print("Patrol thread started")


# ------------------------------------------------------------------
# Entry point
# ------------------------------------------------------------------

def run_web_server(nav, port=8080, bridge_mode=False, bridge_host=None,
                   video_port=9878):
    """Start the web server with the given AutoNavigator instance."""
    global navigator, _bridge_mode, _bridge_host
    navigator = nav
    _bridge_mode = bridge_mode
    _bridge_host = bridge_host

    print(f"\n{'='*50}")
    print(f"  Go2 Patrol Monitor — Web UI")
    print(f"  http://localhost:{port}")
    if bridge_mode:
        print(f"  Mode: WiFi (bridge → {bridge_host})")
    print(f"{'='*50}\n")

    # Start video bridge receiver in bridge mode
    if bridge_mode and bridge_host:
        video_thread = threading.Thread(
            target=_init_video_bridge, args=(bridge_host, video_port), daemon=True)
        video_thread.start()

    uvicorn.run(app, host="0.0.0.0", port=port, log_level="info")
