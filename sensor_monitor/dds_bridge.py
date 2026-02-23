"""
DDS Bridge for Jetson — bridges robot DDS (internal network) to TCP (WiFi)

Runs on the Go2's Jetson board. Subscribes to DDS topics on the internal
192.168.123.x network and serves state data over TCP so PC scripts can
connect over WiFi. Also accepts commands from TCP clients. Optionally
relays the robot camera feed via DDS VideoClient.

Deployment:
    # Copy SDK + bridge to Jetson (over WiFi):
    scp -r unitree_sdk2py sensor_monitor/dds_bridge.py unitree@192.168.0.207:~/

    # SSH in and run:
    ssh unitree@192.168.0.207
    python3 ~/dds_bridge.py --interface eth0

    # With video relay (streams robot camera to PC over WiFi):
    python3 ~/dds_bridge.py --interface eth0 --video

    # With LiDAR relay (streams point cloud to PC over WiFi):
    python3 ~/dds_bridge.py --interface eth0 --lidar

    # Both video + LiDAR:
    python3 ~/dds_bridge.py --interface eth0 --video --lidar

    # From PC (over WiFi):
    python3 sensor_monitor/auto_navigator.py --web --bridge 192.168.0.207 --no-sensor
    python3 sensor_monitor/lidar_mapper.py --bridge 192.168.0.207

Prerequisites:
    - unitree_sdk2py directory on the Jetson (copy from PC)
    - cyclonedds Python package: pip3 install cyclonedds==0.10.2
    - Jetson's eth0 on the 192.168.123.x internal network
    - For --video: uses SDK VideoClient over DDS (no extra deps needed)

Protocol (JSON lines over TCP, port 9877):
    Bridge → Client: {"type":"state","x":0.1,"y":0.2,"yaw":1.5}
    Client → Bridge: {"type":"cmd","action":"standup"}
    Client → Bridge: {"type":"cmd","action":"standdown"}
    Client → Bridge: {"type":"cmd","action":"recovery"}
    Client → Bridge: {"type":"cmd","action":"freewalk"}
    Client → Bridge: {"type":"cmd","action":"stopmove"}
    Client → Bridge: {"type":"cmd","action":"move","vx":0.3,"vy":0.0,"vyaw":0.1}

Video relay protocol (TCP, port 9878):
    4-byte big-endian length prefix + JPEG bytes, repeated continuously (~20 FPS)

LiDAR relay protocol (TCP, port 9879):
    4-byte big-endian total payload length
    JSON header line: {"point_step":16,"n_points":5000,"x_off":0,"y_off":4,"z_off":8}\n
    Raw point data bytes (n_points * point_step)
"""

import sys
import os
import time
import json
import math
import struct
import socket
import select
import argparse
import threading

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
# Also check home dir (when deployed as ~/dds_bridge.py with ~/unitree_sdk2py/)
sys.path.insert(0, os.path.expanduser("~"))

from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelSubscriber
from unitree_sdk2py.idl.unitree_go.msg.dds_ import SportModeState_
from unitree_sdk2py.idl.sensor_msgs.msg.dds_ import PointCloud2_
from unitree_sdk2py.go2.sport.sport_client import SportClient

DEADMAN_TIMEOUT = 0.25  # seconds — stop if no move command received


class VideoRelay:
    """Captures video from robot via DDS VideoClient and serves
    JPEG frames over TCP to remote clients (WiFi).

    Protocol: 4-byte big-endian length prefix + JPEG bytes, repeated.
    Requires ChannelFactoryInitialize() to be called first.
    """

    def __init__(self, tcp_port=9878):
        self.tcp_port = tcp_port
        self._latest_frame = None
        self._frame_lock = threading.Lock()

    def _video_capture_thread(self):
        """Capture JPEG frames from robot via DDS VideoClient."""
        from unitree_sdk2py.go2.video.video_client import VideoClient
        vc = VideoClient()
        vc.SetTimeout(3.0)
        vc.Init()
        print("Video relay: VideoClient initialized (DDS)")
        while True:
            try:
                code, data = vc.GetImageSample()
                if code == 0:
                    with self._frame_lock:
                        self._latest_frame = bytes(data)
            except Exception as e:
                print(f"Video relay: capture error — {e}")
            time.sleep(0.05)  # ~20 FPS

    def _tcp_server_thread(self):
        """Serve latest JPEG frame to TCP clients."""
        srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        srv.bind(('0.0.0.0', self.tcp_port))
        srv.listen(2)
        print(f"Video relay: TCP server on port {self.tcp_port}")

        while True:
            try:
                conn, addr = srv.accept()
                print(f"Video relay: client connected {addr[0]}:{addr[1]}")
                threading.Thread(target=self._handle_client, args=(conn,), daemon=True).start()
            except OSError:
                break

    def _handle_client(self, conn):
        """Stream frames to a single TCP client."""
        try:
            while True:
                with self._frame_lock:
                    frame = self._latest_frame
                if frame is None:
                    time.sleep(0.1)
                    continue
                # 4-byte big-endian length + JPEG data
                header = struct.pack('>I', len(frame))
                conn.sendall(header + frame)
                time.sleep(0.05)  # ~20 FPS
        except (BrokenPipeError, ConnectionResetError, OSError):
            pass
        finally:
            try:
                conn.close()
            except OSError:
                pass
            print("Video relay: client disconnected")

    def start(self):
        """Start video capture and TCP server threads."""
        threading.Thread(target=self._video_capture_thread, daemon=True).start()
        threading.Thread(target=self._tcp_server_thread, daemon=True).start()


class LidarRelay:
    """Subscribes to LiDAR PointCloud2 via DDS and serves raw point data
    over TCP to remote clients (WiFi).

    Protocol per packet:
      [4 bytes] big-endian total payload length
      [N bytes] JSON header: {"point_step":16,"n_points":5000,"x_off":0,"y_off":4,"z_off":8}\n
      [M bytes] raw point data (n_points * point_step bytes)

    Requires ChannelFactoryInitialize() to be called first.
    """

    def __init__(self, tcp_port=9879, throttle_hz=5):
        self.tcp_port = tcp_port
        self.throttle_interval = 1.0 / throttle_hz
        self._latest_packet = None
        self._packet_lock = threading.Lock()
        self._last_time = 0.0
        self._frame_count = 0

    def _on_lidar(self, msg):
        """DDS callback: serialize PointCloud2 into a binary packet."""
        now = time.time()
        if now - self._last_time < self.throttle_interval:
            return
        self._last_time = now

        raw = bytes(msg.data)
        point_step = msg.point_step
        n_points = len(raw) // point_step
        if n_points == 0:
            return

        # Extract field offsets for x, y, z
        field_offsets = {}
        for f in msg.fields:
            if f.name in ('x', 'y', 'z'):
                field_offsets[f.name] = f.offset
        if not all(k in field_offsets for k in ('x', 'y', 'z')):
            return

        # Build header + raw data packet
        header = json.dumps({
            "point_step": point_step,
            "n_points": n_points,
            "x_off": field_offsets['x'],
            "y_off": field_offsets['y'],
            "z_off": field_offsets['z'],
        }).encode("utf-8") + b"\n"

        payload = header + raw[:n_points * point_step]
        packet = struct.pack('>I', len(payload)) + payload

        with self._packet_lock:
            self._latest_packet = packet

        self._frame_count += 1
        if self._frame_count % 50 == 0:
            print(f"LiDAR relay: {self._frame_count} frames relayed")

    def _tcp_server_thread(self):
        """Serve latest LiDAR packet to TCP clients."""
        srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        srv.bind(('0.0.0.0', self.tcp_port))
        srv.listen(2)
        print(f"LiDAR relay: TCP server on port {self.tcp_port}")

        while True:
            try:
                conn, addr = srv.accept()
                print(f"LiDAR relay: client connected {addr[0]}:{addr[1]}")
                threading.Thread(target=self._handle_client, args=(conn,), daemon=True).start()
            except OSError:
                break

    def _handle_client(self, conn):
        """Stream LiDAR packets to a single TCP client."""
        try:
            while True:
                with self._packet_lock:
                    packet = self._latest_packet
                if packet is None:
                    time.sleep(0.1)
                    continue
                conn.sendall(packet)
                time.sleep(self.throttle_interval)
        except (BrokenPipeError, ConnectionResetError, OSError):
            pass
        finally:
            try:
                conn.close()
            except OSError:
                pass
            print("LiDAR relay: client disconnected")

    def start(self):
        """Start DDS subscriber and TCP server threads."""
        # Subscribe to LiDAR point cloud
        self.lidar_sub = ChannelSubscriber("rt/utlidar/cloud_deskewed", PointCloud2_)
        self.lidar_sub.Init(self._on_lidar, 10)
        print("LiDAR relay: subscribed to rt/utlidar/cloud_deskewed")

        threading.Thread(target=self._tcp_server_thread, daemon=True).start()


class DDSBridge:
    def __init__(self, interface="eth0", tcp_port=9877):
        # Initialize DDS on internal network
        print(f"Initializing DDS on interface: {interface}")
        ChannelFactoryInitialize(0, interface)

        self.position = [0.0, 0.0, 0.0]  # x, y, yaw
        self.state_ready = False
        self._state_lock = threading.Lock()

        # Subscribe to robot state
        self.state_sub = ChannelSubscriber("rt/sportmodestate", SportModeState_)
        self.state_sub.Init(self._on_state, 10)

        # Sport client for commands
        self.sport_client = SportClient()
        self.sport_client.SetTimeout(10.0)
        self.sport_client.Init()

        # Move state (for 50Hz move loop + deadman)
        self._move_lock = threading.Lock()
        self._move_vx = 0.0
        self._move_vy = 0.0
        self._move_vyaw = 0.0
        self._move_active = False  # True while receiving move commands
        self._last_move_time = 0.0  # timestamp of last move command

        # TCP server
        self.tcp_port = tcp_port
        self.server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.server.bind(('0.0.0.0', tcp_port))
        self.server.listen(2)
        self.server.setblocking(False)

        self.client_sock = None
        self.client_lock = threading.Lock()
        self._recv_buf = b""

    def _on_state(self, msg):
        with self._state_lock:
            self.position[0] = msg.position[0]
            self.position[1] = msg.position[1]
            self.position[2] = msg.imu_state.rpy[2]
            self.state_ready = True

    def _handle_command(self, cmd_data):
        """Execute a command received from TCP client."""
        action = cmd_data.get("action", "")
        try:
            if action == "move":
                vx = float(cmd_data.get("vx", 0.0))
                vy = float(cmd_data.get("vy", 0.0))
                vyaw = float(cmd_data.get("vyaw", 0.0))
                with self._move_lock:
                    self._move_vx = vx
                    self._move_vy = vy
                    self._move_vyaw = vyaw
                    self._move_active = True
                    self._last_move_time = time.time()
            elif action == "stopmove":
                print(f"  CMD: {action}")
                with self._move_lock:
                    self._move_active = False
                    self._move_vx = 0.0
                    self._move_vy = 0.0
                    self._move_vyaw = 0.0
                self.sport_client.StopMove()
            elif action == "freewalk":
                print(f"  CMD: {action}")
                self.sport_client.FreeWalk()
            elif action == "standup":
                print(f"  CMD: {action}")
                self.sport_client.StandUp()
            elif action == "standdown":
                print(f"  CMD: {action}")
                self.sport_client.StandDown()
            elif action == "recovery":
                print(f"  CMD: {action}")
                self.sport_client.RecoveryStand()
            else:
                print(f"  Unknown action: {action}")
        except Exception as e:
            print(f"  Command error: {e}")

    def _move_loop(self):
        """50Hz loop that continuously sends Move() while active.

        The SDK expects Move() to be called repeatedly (not just once).
        Also enforces deadman timeout — stops robot if no new move command
        arrives within DEADMAN_TIMEOUT seconds.
        """
        while True:
            with self._move_lock:
                active = self._move_active
                vx = self._move_vx
                vy = self._move_vy
                vyaw = self._move_vyaw
                last_time = self._last_move_time

            if active:
                # Deadman check
                if time.time() - last_time > DEADMAN_TIMEOUT:
                    print("  DEADMAN: No move command received, stopping.")
                    with self._move_lock:
                        self._move_active = False
                        self._move_vx = 0.0
                        self._move_vy = 0.0
                        self._move_vyaw = 0.0
                    try:
                        self.sport_client.StopMove()
                    except Exception:
                        pass
                else:
                    try:
                        self.sport_client.Move(vx, vy, vyaw)
                    except Exception:
                        pass

            time.sleep(0.02)  # 50Hz

    def _process_recv(self):
        """Read and process any commands from TCP client."""
        with self.client_lock:
            if self.client_sock is None:
                return
            try:
                readable, _, _ = select.select([self.client_sock], [], [], 0)
                if readable:
                    data = self.client_sock.recv(4096)
                    if not data:
                        print("TCP client disconnected.")
                        self.client_sock.close()
                        self.client_sock = None
                        self._recv_buf = b""
                        # Stop robot on disconnect
                        with self._move_lock:
                            self._move_active = False
                        try:
                            self.sport_client.StopMove()
                        except Exception:
                            pass
                        return
                    self._recv_buf += data
            except (OSError, ConnectionResetError):
                self.client_sock = None
                self._recv_buf = b""
                with self._move_lock:
                    self._move_active = False
                try:
                    self.sport_client.StopMove()
                except Exception:
                    pass
                return

        # Process complete lines outside the lock
        while b"\n" in self._recv_buf:
            line, self._recv_buf = self._recv_buf.split(b"\n", 1)
            line = line.strip()
            if not line:
                continue
            try:
                msg = json.loads(line.decode("utf-8"))
                if msg.get("type") == "cmd":
                    self._handle_command(msg)
            except (json.JSONDecodeError, UnicodeDecodeError) as e:
                print(f"  Bad message: {e}")

    def _accept_clients(self):
        """Accept new TCP connections (background thread)."""
        while True:
            try:
                readable, _, _ = select.select([self.server], [], [], 1.0)
                if readable:
                    sock, addr = self.server.accept()
                    with self.client_lock:
                        if self.client_sock is not None:
                            try:
                                self.client_sock.close()
                            except OSError:
                                pass
                        self.client_sock = sock
                        self._recv_buf = b""
                    # Stop any active movement when a new client connects
                    with self._move_lock:
                        self._move_active = False
                    print(f"TCP client connected: {addr[0]}:{addr[1]}")
            except OSError:
                break

    def run(self):
        print(f"TCP server listening on 0.0.0.0:{self.tcp_port}")
        print("Waiting for DDS state...")

        # Accept thread
        accept_thread = threading.Thread(target=self._accept_clients, daemon=True)
        accept_thread.start()

        # 50Hz move loop thread
        move_thread = threading.Thread(target=self._move_loop, daemon=True)
        move_thread.start()

        # Wait for first state
        t0 = time.time()
        while not self.state_ready and time.time() - t0 < 10.0:
            time.sleep(0.1)

        if self.state_ready:
            with self._state_lock:
                x, y, yaw = self.position
            print(f"DDS connected! Position: ({x:.3f}, {y:.3f}), yaw={math.degrees(yaw):.1f} deg")
        else:
            print("WARNING: No DDS state yet (will keep trying)...")

        print("=" * 50)
        print("DDS bridge running. Ctrl+C to stop.")
        print("=" * 50)

        last_send = 0
        try:
            while True:
                # Process incoming commands
                self._process_recv()

                # Send state at ~10Hz
                now = time.time()
                if now - last_send >= 0.1 and self.state_ready:
                    with self._state_lock:
                        x, y, yaw = self.position
                    payload = json.dumps({
                        "type": "state", "x": x, "y": y, "yaw": yaw
                    }) + "\n"

                    with self.client_lock:
                        if self.client_sock is not None:
                            try:
                                self.client_sock.sendall(payload.encode("utf-8"))
                            except (BrokenPipeError, ConnectionResetError, OSError):
                                print("TCP client disconnected.")
                                try:
                                    self.client_sock.close()
                                except OSError:
                                    pass
                                self.client_sock = None
                                # Stop on disconnect
                                with self._move_lock:
                                    self._move_active = False
                    last_send = now

                time.sleep(0.01)  # ~100Hz loop
        except KeyboardInterrupt:
            print("\nStopping robot...")
            with self._move_lock:
                self._move_active = False
            try:
                self.sport_client.StopMove()
            except Exception:
                pass
            print("DDS bridge stopped.")
        finally:
            self.server.close()
            with self.client_lock:
                if self.client_sock is not None:
                    try:
                        self.client_sock.close()
                    except OSError:
                        pass
            print("Cleaned up.")


def main():
    parser = argparse.ArgumentParser(
        description="DDS Bridge (runs on Jetson, bridges DDS to TCP over WiFi)")
    parser.add_argument("--interface", default="eth0",
                        help="Internal network interface for DDS (default: eth0)")
    parser.add_argument("--tcp-port", type=int, default=9877,
                        help="TCP server port (default: 9877)")
    parser.add_argument("--video", action="store_true",
                        help="Enable video relay (DDS VideoClient → TCP to PC)")
    parser.add_argument("--video-port", type=int, default=9878,
                        help="Video relay TCP port (default: 9878)")
    parser.add_argument("--lidar", action="store_true",
                        help="Enable LiDAR point cloud relay (DDS → TCP to PC)")
    parser.add_argument("--lidar-port", type=int, default=9879,
                        help="LiDAR relay TCP port (default: 9879)")
    args = parser.parse_args()

    bridge = DDSBridge(interface=args.interface, tcp_port=args.tcp_port)

    if args.video:
        video_relay = VideoRelay(tcp_port=args.video_port)
        video_relay.start()

    if args.lidar:
        lidar_relay = LidarRelay(tcp_port=args.lidar_port)
        lidar_relay.start()

    bridge.run()


if __name__ == "__main__":
    main()
