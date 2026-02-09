"""
Sensor Bridge for Jetson (Step 2 helper) — stdlib-only version

Runs on the Go2's Jetson board. Reads sensor data from Arduino via USB serial
and serves it over TCP so auto_navigator.py on the PC can connect.

Uses ONLY Python stdlib (no pyserial, no unitree_sdk2py).

Deployment:
    # Copy to Jetson:
    scp sensor_monitor/sensor_bridge.py unitree@192.168.123.18:~/

    # SSH in and run:
    ssh unitree@192.168.123.18
    python3 ~/sensor_bridge.py --port /dev/ttyACM0

    # Verify from PC:
    nc 192.168.123.18 9876

Prerequisites:
    - Arduino connected to Jetson via USB (usually /dev/ttyACM0 or /dev/ttyUSB0)
    - Arduino sending a numeric value via Serial.println()
      (accepts bare numbers or prefixed lines like "Sound Intensity: 123")
    - User in dialout group: sudo usermod -aG dialout unitree (then re-login)
"""

import sys
import os
import time
import json
import glob
import socket
import termios
import argparse
import threading
import select


def find_arduino_port():
    """Try common Arduino serial ports."""
    candidates = glob.glob('/dev/ttyACM*') + glob.glob('/dev/ttyUSB*')
    if candidates:
        return candidates[0]
    return None


def open_serial(port, baud_rate=9600):
    """Open serial port using os/termios (no pyserial needed).

    Returns a file object for line-buffered reading.
    """
    baud_map = {
        9600: termios.B9600,
        19200: termios.B19200,
        38400: termios.B38400,
        57600: termios.B57600,
        115200: termios.B115200,
    }
    baud_const = baud_map.get(baud_rate)
    if baud_const is None:
        print(f"ERROR: Unsupported baud rate {baud_rate}. Use one of: {list(baud_map.keys())}")
        sys.exit(1)

    fd = os.open(port, os.O_RDONLY | os.O_NOCTTY)
    attrs = termios.tcgetattr(fd)

    # Input flags: no parity check, no strip, no flow control
    attrs[0] = 0
    # Output flags: raw
    attrs[1] = 0
    # Control flags: 8N1, enable receiver, local mode
    attrs[2] = termios.CS8 | termios.CREAD | termios.CLOCAL
    # Local flags: raw (no echo, no canonical, no signals)
    attrs[3] = 0
    # Input speed
    attrs[4] = baud_const
    # Output speed
    attrs[5] = baud_const
    # cc: VMIN=1 (block until at least 1 byte), VTIME=0
    attrs[6][termios.VMIN] = 1
    attrs[6][termios.VTIME] = 0

    termios.tcsetattr(fd, termios.TCSAFLUSH, attrs)
    return os.fdopen(fd, 'rb')


def parse_value(line):
    """Parse a sensor value from a serial line.

    Accepts bare numbers ("123") or prefixed lines ("Sound Intensity: 123").
    Returns float or None.
    """
    try:
        return float(line)
    except ValueError:
        pass
    try:
        return float(line.split()[-1])
    except (ValueError, IndexError):
        return None


def main():
    parser = argparse.ArgumentParser(description="Sensor Bridge (runs on Jetson, stdlib only)")
    parser.add_argument("--port", default=None, help="Serial port (default: auto-detect)")
    parser.add_argument("--baud", type=int, default=9600, help="Baud rate (default: 9600)")
    parser.add_argument("--tcp-port", type=int, default=9876, help="TCP server port (default: 9876)")
    args = parser.parse_args()

    # Find serial port
    port = args.port or find_arduino_port()
    if port is None:
        print("ERROR: No Arduino found. Specify port with --port /dev/ttyACM0")
        sys.exit(1)

    print(f"Opening serial port: {port} at {args.baud} baud")
    ser = open_serial(port, args.baud)
    time.sleep(2)  # wait for Arduino reset

    # TCP server
    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server.bind(('0.0.0.0', args.tcp_port))
    server.listen(1)
    server.setblocking(False)
    print(f"TCP server listening on 0.0.0.0:{args.tcp_port}")

    client_sock = None
    client_lock = threading.Lock()

    def accept_clients():
        """Background thread to accept TCP connections (one at a time)."""
        nonlocal client_sock
        while True:
            try:
                readable, _, _ = select.select([server], [], [], 1.0)
                if readable:
                    sock, addr = server.accept()
                    with client_lock:
                        if client_sock is not None:
                            try:
                                client_sock.close()
                            except OSError:
                                pass
                        client_sock = sock
                    print(f"TCP client connected: {addr[0]}:{addr[1]}")
            except OSError:
                break

    accept_thread = threading.Thread(target=accept_clients, daemon=True)
    accept_thread.start()

    print("=" * 40)
    print("Sensor bridge running. Ctrl+C to stop.")
    print("=" * 40)

    try:
        for raw_line in ser:
            line = raw_line.decode('utf-8', errors='ignore').strip()
            if not line:
                continue

            value = parse_value(line)
            if value is None:
                print(f"  (skipped: {line})")
                continue

            ts = time.time()
            payload = json.dumps({"value": value, "ts": ts}) + "\n"
            print(f"  value={value}")

            with client_lock:
                if client_sock is not None:
                    try:
                        client_sock.sendall(payload.encode('utf-8'))
                    except (BrokenPipeError, ConnectionResetError, OSError):
                        print("  TCP client disconnected.")
                        try:
                            client_sock.close()
                        except OSError:
                            pass
                        client_sock = None

    except KeyboardInterrupt:
        print("\nSensor bridge stopped.")
    finally:
        ser.close()
        server.close()
        with client_lock:
            if client_sock is not None:
                try:
                    client_sock.close()
                except OSError:
                    pass
        print("Cleaned up.")


if __name__ == "__main__":
    main()
