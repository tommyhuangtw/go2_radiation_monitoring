"""
WiFi Bridge Client — connects to dds_bridge.py over TCP.

Provides a SportClient-like interface so auto_navigator.py and
route_recorder.py can work over WiFi without changes to navigation logic.
Also provides LidarBridgeReceiver for streaming LiDAR data over WiFi.

Usage:
    client = WiFiBridgeClient("192.168.0.207")
    client.connect()
    print(client.get_position())  # (x, y, yaw)
    client.StandUp()
    client.FreeWalk()
    client.Move(0.3, 0.0, 0.1)
    client.StopMove()
    client.close()

    # LiDAR streaming:
    lidar = LidarBridgeReceiver("192.168.0.207", callback=on_points)
    lidar.start()
"""

import json
import struct
import socket
import threading
import time

import numpy as np


class WiFiBridgeClient:
    def __init__(self, host, port=9877):
        self.host = host
        self.port = port
        self._sock = None
        self._connected = False

        # State from bridge
        self._x = 0.0
        self._y = 0.0
        self._yaw = 0.0
        self._state_ready = False
        self._state_lock = threading.Lock()

        self._recv_buf = b""
        self._recv_thread = None

    def connect(self, timeout=5.0):
        """Connect to the DDS bridge TCP server."""
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self._sock.settimeout(timeout)
        self._sock.connect((self.host, self.port))
        self._sock.settimeout(None)
        self._connected = True

        # Start background receiver
        self._recv_thread = threading.Thread(target=self._recv_loop, daemon=True)
        self._recv_thread.start()

    def close(self):
        """Disconnect from the bridge."""
        self._connected = False
        if self._sock:
            try:
                self._sock.close()
            except OSError:
                pass
            self._sock = None

    @property
    def state_ready(self):
        with self._state_lock:
            return self._state_ready

    def get_position(self):
        """Returns (x, y, yaw) from the bridge's state stream."""
        with self._state_lock:
            return (self._x, self._y, self._yaw)

    def _send_cmd(self, cmd_dict):
        """Send a JSON command line to the bridge."""
        if not self._connected or self._sock is None:
            return
        payload = json.dumps(cmd_dict) + "\n"
        try:
            self._sock.sendall(payload.encode("utf-8"))
        except (BrokenPipeError, ConnectionResetError, OSError):
            self._connected = False

    def _recv_loop(self):
        """Background thread: receive state updates from bridge."""
        while self._connected:
            try:
                data = self._sock.recv(4096)
                if not data:
                    self._connected = False
                    break
                self._recv_buf += data
            except (OSError, ConnectionResetError):
                self._connected = False
                break

            while b"\n" in self._recv_buf:
                line, self._recv_buf = self._recv_buf.split(b"\n", 1)
                line = line.strip()
                if not line:
                    continue
                try:
                    msg = json.loads(line.decode("utf-8"))
                    if msg.get("type") == "state":
                        with self._state_lock:
                            self._x = msg["x"]
                            self._y = msg["y"]
                            self._yaw = msg["yaw"]
                            self._state_ready = True
                except (json.JSONDecodeError, UnicodeDecodeError, KeyError):
                    pass

    # --- SportClient-compatible interface ---

    def Move(self, vx, vy, vyaw):
        self._send_cmd({"type": "cmd", "action": "move",
                        "vx": vx, "vy": vy, "vyaw": vyaw})

    def StopMove(self):
        self._send_cmd({"type": "cmd", "action": "stopmove"})

    def FreeWalk(self):
        self._send_cmd({"type": "cmd", "action": "freewalk"})

    def StandUp(self):
        self._send_cmd({"type": "cmd", "action": "standup"})

    def StandDown(self):
        self._send_cmd({"type": "cmd", "action": "standdown"})

    def RecoveryStand(self):
        self._send_cmd({"type": "cmd", "action": "recovery"})

    # Stubs for compatibility (no-ops over bridge)
    def SetTimeout(self, t):
        pass

    def Init(self):
        pass


class LidarBridgeReceiver:
    """Receives LiDAR point cloud data from dds_bridge.py LiDAR relay over TCP.

    Parses length-prefixed packets containing a JSON header + raw point bytes,
    extracts xyz points as numpy arrays, and delivers them via a callback.
    """

    def __init__(self, host, port=9879, callback=None):
        self.host = host
        self.port = port
        self._callback = callback
        self._connected = False
        self.lidar_ready = False

    def _recv_exact(self, sock, n):
        """Read exactly n bytes from socket."""
        buf = b""
        while len(buf) < n:
            chunk = sock.recv(min(n - len(buf), 65536))
            if not chunk:
                raise ConnectionError("LiDAR bridge disconnected")
            buf += chunk
        return buf

    def _recv_loop(self):
        """Background thread: connect to LiDAR relay and receive point clouds."""
        while True:
            try:
                sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                sock.settimeout(5.0)
                print(f"LiDAR bridge: connecting to {self.host}:{self.port}...")
                sock.connect((self.host, self.port))
                sock.settimeout(None)
                self._connected = True
                print("LiDAR bridge: connected!")

                while True:
                    # Read 4-byte length prefix
                    header_raw = self._recv_exact(sock, 4)
                    payload_len = struct.unpack('>I', header_raw)[0]

                    # Read full payload
                    payload = self._recv_exact(sock, payload_len)

                    # Split JSON header and raw point data
                    newline_idx = payload.index(b"\n")
                    meta = json.loads(payload[:newline_idx].decode("utf-8"))
                    raw_data = payload[newline_idx + 1:]

                    # Parse xyz points
                    points = self._parse_points(
                        raw_data, meta["point_step"], meta["n_points"],
                        meta["x_off"], meta["y_off"], meta["z_off"])

                    self.lidar_ready = True
                    if points is not None and self._callback:
                        self._callback(points)

            except (ConnectionRefusedError, socket.timeout, OSError, ConnectionError) as e:
                print(f"LiDAR bridge: {e} — retrying in 3s...")
            finally:
                self._connected = False
                try:
                    sock.close()
                except OSError:
                    pass
            time.sleep(3)

    def _parse_points(self, raw, point_step, n_points, x_off, y_off, z_off):
        """Parse raw point cloud bytes into Nx3 numpy array."""
        expected = n_points * point_step
        if len(raw) < expected:
            n_points = len(raw) // point_step
        if n_points == 0:
            return None

        raw_array = np.frombuffer(raw, dtype=np.uint8)
        raw_array = raw_array[:n_points * point_step].reshape(n_points, point_step)

        x = raw_array[:, x_off:x_off+4].copy().view(np.float32).flatten()
        y = raw_array[:, y_off:y_off+4].copy().view(np.float32).flatten()
        z = raw_array[:, z_off:z_off+4].copy().view(np.float32).flatten()

        # Filter invalid points
        valid = np.isfinite(x) & np.isfinite(y) & np.isfinite(z)
        dist_sq = x*x + y*y + z*z
        valid &= dist_sq > 0.01
        points = np.column_stack([x[valid], y[valid], z[valid]])

        return points if len(points) > 0 else None

    def start(self):
        """Start background receiver thread."""
        threading.Thread(target=self._recv_loop, daemon=True).start()
