"""
Web server for Go2 Patrol Monitor.

Provides a FastAPI backend with:
- WebSocket for real-time state updates and commands
- MJPEG camera stream
- REST endpoints for map data (floor plan, waypoints, metadata)

Launched via: python sensor_monitor/auto_navigator.py --web
"""

import os
import json
import asyncio
import threading
from pathlib import Path

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
        return json.load(f)


@app.get("/api/waypoints")
async def get_waypoints():
    wp_path = DATA_DIR / "waypoints.json"
    if not wp_path.exists():
        return JSONResponse({"error": "waypoints.json not found"}, status_code=404)
    with open(wp_path) as f:
        return json.load(f)


@app.get("/api/floor_plan")
async def get_floor_plan():
    img_path = DATA_DIR / "floor_plan.png"
    if not img_path.exists():
        return JSONResponse({"error": "floor_plan.png not found"}, status_code=404)
    return FileResponse(str(img_path), media_type="image/png")


# ------------------------------------------------------------------
# MJPEG camera stream
# ------------------------------------------------------------------

def _get_video_client():
    """Lazy-init VideoClient (thread-safe)."""
    global video_client
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


def _capture_frame():
    """Blocking call to get one JPEG frame from the robot camera."""
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

def run_web_server(nav, port=8080):
    """Start the web server with the given AutoNavigator instance."""
    global navigator
    navigator = nav

    print(f"\n{'='*50}")
    print(f"  Go2 Patrol Monitor — Web UI")
    print(f"  http://localhost:{port}")
    print(f"{'='*50}\n")

    uvicorn.run(app, host="0.0.0.0", port=port, log_level="info")
