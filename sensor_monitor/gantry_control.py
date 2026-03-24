#!/usr/bin/env python3
"""
Gantry control for Jetson/Go2 payload via Arduino Mega + A4988 stepper driver.

Design assumptions
------------------
- Jetson talks to an Arduino Mega over USB serial (/dev/ttyACM*).
- The Arduino runs gantry_arduino_control.ino which handles STEP/DIR/EN
  pulses for an A4988 stepper driver.
- Motor power is supplied separately (12V rail).
- Motion is open-loop. Position is tracked in software on the Arduino.
- The Arduino accepts text commands (SET_HOME, MOVE_ABS, STOP, STATUS, etc.)
  and returns text responses (OK ..., ERR ..., STATUS state=...).
"""

from __future__ import annotations

import argparse
import glob
import json
import os
import threading
import time
from dataclasses import dataclass, asdict
from typing import Optional

import serial


# ----------------------------
# Arduino serial config
# ----------------------------
DEFAULT_BAUD = 115200
SERIAL_TIMEOUT = 0.1           # readline timeout (seconds)
ARDUINO_BOOT_WAIT = 2.5        # wait for Arduino reset after serial open
STATUS_POLL_INTERVAL = 0.05    # poll interval during move_to

# Physical height mapping (motor mm → cm from ground)
GROUND_HEIGHT_CM = 40.0    # height when motor at 0mm (home)
TOP_HEIGHT_CM = 120.0      # height when motor at 9mm (top)
MOTOR_RANGE_MM = 9.0       # motor travel range


@dataclass
class GantryState:
    connected: bool = False
    serial_port: Optional[str] = None
    uart_address: int = 0
    current_position_mm: float = 0.0
    last_target_mm: Optional[float] = None
    moving: bool = False
    homed: bool = False
    vactual: int = 0
    ifcnt: Optional[int] = None
    last_error: str = ""
    status: str = "idle"
    motor_enabled: bool = False


class ArduinoSerial:
    """Serial interface to the Arduino gantry controller."""

    def __init__(self, port: str, baudrate: int = DEFAULT_BAUD) -> None:
        self.port = port
        self.baudrate = baudrate
        self._ser = serial.Serial(port=port, baudrate=baudrate, timeout=SERIAL_TIMEOUT)
        self._io_lock = threading.Lock()

    def wait_for_boot(self, timeout: float = ARDUINO_BOOT_WAIT) -> list[str]:
        """Drain boot messages, waiting for A4988_GANTRY_READY."""
        lines: list[str] = []
        deadline = time.time() + timeout
        while time.time() < deadline:
            line = self._ser.readline().decode("utf-8", errors="replace").strip()
            if line:
                lines.append(line)
                if "A4988_GANTRY_READY" in line:
                    # Keep draining briefly for any remaining boot lines
                    time.sleep(0.1)
                    while self._ser.in_waiting:
                        extra = self._ser.readline().decode("utf-8", errors="replace").strip()
                        if extra:
                            lines.append(extra)
                    return lines
        return lines

    def send_command(self, cmd: str) -> list[str]:
        """Send a command and collect response lines until OK/ERR or timeout."""
        with self._io_lock:
            # Drain any stale data
            self._ser.reset_input_buffer()
            self._ser.write((cmd + "\n").encode("utf-8"))
            self._ser.flush()

            lines: list[str] = []
            deadline = time.time() + 1.0  # timeout for response
            while time.time() < deadline:
                line = self._ser.readline().decode("utf-8", errors="replace").strip()
                if not line:
                    continue
                lines.append(line)
                # Terminal responses: OK, ERR, or TARGET LOCATION ARRIVED
                if line.startswith("OK ") or line.startswith("ERR ") or line.startswith("TARGET "):
                    # Also grab any trailing STATUS line
                    time.sleep(0.005)
                    while self._ser.in_waiting:
                        extra = self._ser.readline().decode("utf-8", errors="replace").strip()
                        if extra:
                            lines.append(extra)
                    break
            return lines

    def send_and_check(self, cmd: str) -> list[str]:
        """Send command, raise RuntimeError on ERR response."""
        lines = self.send_command(cmd)
        for line in lines:
            if line.startswith("ERR "):
                raise RuntimeError(line[4:])
        return lines

    @staticmethod
    def parse_status_line(line: str) -> dict:
        """Parse 'STATUS state=X pos_mm=Y target_mm=Z home=H enabled=E' into dict."""
        # Remove 'STATUS ' prefix
        body = line[7:] if line.startswith("STATUS ") else line
        result = {}
        for part in body.split():
            key, _, val = part.partition("=")
            if key:
                result[key] = val
        return {
            "state": result.get("state", "UNKNOWN"),
            "pos_mm": float(result.get("pos_mm", "0")),
            "target_mm": float(result.get("target_mm", "0")),
            "home": result.get("home", "0") == "1",
            "enabled": result.get("enabled", "0") == "1",
        }

    def get_status(self) -> dict:
        """Send STATUS and return parsed result."""
        lines = self.send_command("STATUS")
        for line in lines:
            if line.startswith("STATUS "):
                return self.parse_status_line(line)
        raise RuntimeError(f"No STATUS reply from Arduino: {lines}")

    def ping(self) -> bool:
        """Send PING and check for PONG."""
        lines = self.send_command("PING")
        return any("PONG" in line for line in lines)

    def close(self) -> None:
        if self._ser and self._ser.is_open:
            self._ser.close()


class GantryController:
    def __init__(
        self,
        port: Optional[str] = None,
        baudrate: int = DEFAULT_BAUD,
    ) -> None:
        self.port = port or self._auto_detect_port()
        self.baudrate = baudrate
        self.arduino: Optional[ArduinoSerial] = None
        self.state = GantryState(serial_port=self.port)
        self._move_lock = threading.Lock()
        self._stop_event = threading.Event()
        self._thread: Optional[threading.Thread] = None

    @staticmethod
    def _list_candidate_ports() -> list[str]:
        """List all candidate serial ports (ttyACM* first, then ttyUSB*)."""
        ports = sorted(glob.glob("/dev/ttyACM*")) + sorted(glob.glob("/dev/ttyUSB*"))
        return ports

    def _try_connect_port(self, port: str) -> bool:
        """Try to connect to a port and verify it's the gantry Arduino.
        Returns True if this port has the gantry controller."""
        try:
            arduino = ArduinoSerial(port, self.baudrate)
            boot_lines = arduino.wait_for_boot()
            # Check if boot messages identify this as the gantry controller
            is_gantry = any("A4988_GANTRY_READY" in line for line in boot_lines)
            if not is_gantry:
                # Might have missed boot message — try PING
                if arduino.ping():
                    # Verify it responds to STATUS (gantry-specific command)
                    try:
                        status = arduino.get_status()
                        is_gantry = "state" in status
                    except Exception:
                        is_gantry = False
            if is_gantry:
                self.arduino = arduino
                self.port = port
                return True
            else:
                arduino.close()
                return False
        except (serial.SerialException, OSError):
            return False

    def connect(self) -> None:
        if self.port:
            # Specific port requested — use it directly
            self.arduino = ArduinoSerial(self.port, self.baudrate)
            boot_lines = self.arduino.wait_for_boot()
            if not self.arduino.ping():
                raise RuntimeError(f"Arduino not responding on {self.port}")
        else:
            # Auto-detect: try each port until we find the gantry Arduino
            candidates = self._list_candidate_ports()
            if not candidates:
                raise RuntimeError("No Arduino serial port found (/dev/ttyACM* or /dev/ttyUSB*)")
            for port in candidates:
                print(f"Trying {port}...")
                if self._try_connect_port(port):
                    print(f"Found gantry Arduino on {port}")
                    break
            else:
                raise RuntimeError(
                    f"Gantry Arduino not found on any port: {candidates}. "
                    f"Use --port to specify manually."
                )
        self._sync_state()
        self.state.connected = True
        self.state.serial_port = self.port
        # Auto-home: assume gantry starts at physical bottom (0mm)
        self.set_home_here()

    def close(self) -> None:
        try:
            self.stop()
        finally:
            if self.arduino is not None:
                self.arduino.close()

    def _sync_state(self) -> None:
        """Pull current state from Arduino and update GantryState."""
        if self.arduino is None:
            return
        try:
            status = self.arduino.get_status()
            self.state.current_position_mm = status["pos_mm"]
            self.state.last_target_mm = status["target_mm"]
            self.state.moving = (status["state"] == "MOVING")
            self.state.homed = status["home"]
            self.state.motor_enabled = status["enabled"]
            state_map = {
                "IDLE": "idle",
                "MOVING": "moving",
                "STOPPED": "stopped",
                "ERROR": "error",
            }
            self.state.status = state_map.get(status["state"], status["state"].lower())
        except Exception as exc:
            self.state.last_error = str(exc)

    def set_home_here(self) -> None:
        if self.arduino is None:
            raise RuntimeError("Arduino not connected")
        self.arduino.send_and_check("SET_HOME")
        self._sync_state()
        self.state.homed = True
        self.state.status = "homed"
        self.state.last_error = ""

    def stop(self) -> None:
        self._stop_event.set()
        if self.arduino is not None:
            try:
                self.arduino.send_command("STOP")
                self._sync_state()
            except Exception:
                pass
        self.state.moving = False
        if self.state.status == "moving":
            self.state.status = "stopped"

    def move_to_async(self, target_mm: float) -> None:
        if self._thread and self._thread.is_alive():
            raise RuntimeError("A move is already in progress")
        self._thread = threading.Thread(target=self.move_to, args=(target_mm,), daemon=True)
        self._thread.start()

    def move_to(self, target_mm: float) -> None:
        if self.arduino is None:
            raise RuntimeError("Arduino not connected")

        with self._move_lock:
            self._stop_event.clear()
            self.state.moving = True
            self.state.status = "moving"
            self.state.last_target_mm = float(target_mm)
            self.state.last_error = ""

            try:
                lines = self.arduino.send_and_check(f"MOVE_ABS {target_mm:.2f}")

                # Check if already at target
                already_arrived = any(
                    "ARRIVED" in line for line in lines
                    if line.startswith("OK ") or line.startswith("TARGET ")
                )
                if already_arrived:
                    self._sync_state()
                    self.state.status = "at_target"
                    return

                # Poll until Arduino finishes moving
                while True:
                    if self._stop_event.is_set():
                        self.arduino.send_command("STOP")
                        self._sync_state()
                        self.state.status = "stopped"
                        return

                    time.sleep(STATUS_POLL_INTERVAL)
                    status = self.arduino.get_status()
                    self.state.current_position_mm = status["pos_mm"]

                    if status["state"] == "IDLE":
                        self._sync_state()
                        self.state.status = "at_target"
                        return
                    elif status["state"] == "STOPPED":
                        self._sync_state()
                        self.state.status = "stopped"
                        return
                    elif status["state"] == "ERROR":
                        self._sync_state()
                        self.state.status = "error"
                        return

            except Exception as exc:
                self.state.last_error = str(exc)
                self.state.status = "error"
                raise
            finally:
                self.state.moving = False

    def jog(self, delta_mm: float) -> None:
        self.move_to_async(self.state.current_position_mm + float(delta_mm))

    @staticmethod
    def motor_to_display_cm(motor_mm: float) -> float:
        """Convert internal motor mm to user-visible height in cm from ground."""
        return GROUND_HEIGHT_CM + (motor_mm / MOTOR_RANGE_MM) * (TOP_HEIGHT_CM - GROUND_HEIGHT_CM)

    def get_state_dict(self) -> dict:
        return asdict(self.state)


HTML_UI = """
<!DOCTYPE html>
<html lang=\"en\">
<head>
<meta charset=\"utf-8\" />
<meta name=\"viewport\" content=\"width=device-width,initial-scale=1\" />
<title>Go2 Gantry Control</title>
<style>
body { font-family: Arial, sans-serif; background:#111827; color:#e5e7eb; margin:0; padding:20px; }
.wrap { max-width: 900px; margin:0 auto; }
.grid { display:grid; grid-template-columns: repeat(2,minmax(0,1fr)); gap:16px; }
.card { background:#1f2937; padding:16px; border-radius:12px; border:1px solid #374151; }
button { padding:10px 16px; border:0; border-radius:10px; cursor:pointer; font-weight:600; }
button.primary { background:#2563eb; color:white; }
button.warn { background:#dc2626; color:white; }
button.secondary { background:#374151; color:white; }
input { width:100%; padding:10px; border-radius:8px; border:1px solid #4b5563; background:#111827; color:#fff; }
.row { display:flex; gap:10px; flex-wrap:wrap; }
.kv { line-height:1.8; }
pre { white-space:pre-wrap; background:#0b1220; padding:12px; border-radius:10px; }
.small { font-size: 0.9rem; color:#9ca3af; }
</style>
</head>
<body>
<div class=\"wrap\">
  <h1>Go2 Gantry Control</h1>
  <p class=\"small\">Arduino Mega + A4988 gantry control over USB serial.</p>
  <div class=\"grid\">
    <div class=\"card\">
      <h3>Move to absolute position (mm)</h3>
      <input id=\"target\" type=\"number\" step=\"0.1\" value=\"0\" />
      <div class=\"row\" style=\"margin-top:12px\">
        <button class=\"primary\" onclick=\"moveTo()\">Move</button>
        <button class=\"secondary\" onclick=\"homeHere()\">Set Home Here</button>
        <button class=\"warn\" onclick=\"stopMove()\">STOP</button>
      </div>
      <h3 style=\"margin-top:20px\">Jog</h3>
      <div class=\"row\">
        <button class=\"secondary\" onclick=\"jog(-10)\">-10 mm</button>
        <button class=\"secondary\" onclick=\"jog(-1)\">-1 mm</button>
        <button class=\"secondary\" onclick=\"jog(1)\">+1 mm</button>
        <button class=\"secondary\" onclick=\"jog(10)\">+10 mm</button>
      </div>
    </div>
    <div class=\"card\">
      <h3>Status</h3>
      <div class=\"kv\" id=\"status\">Loading...</div>
    </div>
  </div>
  <div class=\"card\" style=\"margin-top:16px\">
    <h3>Notes</h3>
    <pre>1) Home is set automatically when this server starts (assumes sensor arm is at bottom).
2) Sensor arm range: 40 cm (home/bottom) to 120 cm (top) from ground.
3) Use absolute mm moves (internal) or jog buttons.
4) Position resets on Arduino power-off. Restart this server to re-home.
5) If arm is NOT at bottom on startup, manually press \"Set Home Here\" first.</pre>
  </div>
</div>
<script>
async function api(path, method='GET', body=null) {
  const res = await fetch(path, {
    method,
    headers: {'Content-Type':'application/json'},
    body: body ? JSON.stringify(body) : null
  });
  return await res.json();
}
function toDisplayCm(mm) { return 40 + (Number(mm) / 9) * 80; }
async function refresh() {
  const s = await api('/api/state');
  const heightCm = toDisplayCm(s.current_position_mm);
  const targetCm = s.last_target_mm === null ? null : toDisplayCm(s.last_target_mm);
  document.getElementById('status').innerHTML = `
    <b>Connected:</b> ${s.connected}<br>
    <b>Height:</b> ${heightCm.toFixed(0)} cm<br>
    <b>Target:</b> ${targetCm === null ? 'N/A' : targetCm.toFixed(0) + ' cm'}<br>
    <b>Status:</b> ${s.status}<br>
    <b>Moving:</b> ${s.moving}<br>
    <b>Homed:</b> ${s.homed}<br>
    <b>Motor enabled:</b> ${s.motor_enabled}<br>
    <b>Last error:</b> ${s.last_error || 'None'}`;
}
async function moveTo() {
  const target = Number(document.getElementById('target').value);
  await api('/api/move_to', 'POST', {target_mm: target});
  refresh();
}
async function jog(delta) {
  await api('/api/jog', 'POST', {delta_mm: delta});
  refresh();
}
async function homeHere() {
  await api('/api/home_here', 'POST');
  refresh();
}
async function stopMove() {
  await api('/api/stop', 'POST');
  refresh();
}
setInterval(refresh, 500);
refresh();
</script>
</body>
</html>
"""


def run_web_server(controller: GantryController, host: str = "0.0.0.0", port: int = 8090) -> None:
    from fastapi import FastAPI
    from fastapi.responses import HTMLResponse, JSONResponse
    import uvicorn

    app = FastAPI(title="Go2 Gantry Control")

    @app.get("/", response_class=HTMLResponse)
    async def index() -> HTMLResponse:
        return HTMLResponse(HTML_UI)

    @app.get("/api/state")
    async def api_state() -> JSONResponse:
        return JSONResponse(controller.get_state_dict())

    @app.post("/api/home_here")
    async def api_home_here() -> JSONResponse:
        controller.set_home_here()
        return JSONResponse({"ok": True, "state": controller.get_state_dict()})

    @app.post("/api/stop")
    async def api_stop() -> JSONResponse:
        controller.stop()
        return JSONResponse({"ok": True, "state": controller.get_state_dict()})

    @app.post("/api/jog")
    async def api_jog(payload: dict) -> JSONResponse:
        try:
            controller.jog(float(payload["delta_mm"]))
            return JSONResponse({"ok": True, "state": controller.get_state_dict()})
        except Exception as exc:
            controller.state.last_error = str(exc)
            controller.state.status = "error"
            return JSONResponse({"ok": False, "error": str(exc), "state": controller.get_state_dict()}, status_code=400)

    @app.post("/api/move_to")
    async def api_move_to(payload: dict) -> JSONResponse:
        try:
            controller.move_to_async(float(payload["target_mm"]))
            return JSONResponse({"ok": True, "state": controller.get_state_dict()})
        except Exception as exc:
            controller.state.last_error = str(exc)
            controller.state.status = "error"
            return JSONResponse({"ok": False, "error": str(exc), "state": controller.get_state_dict()}, status_code=400)

    uvicorn.run(app, host=host, port=port, log_level="info")


def main() -> None:
    parser = argparse.ArgumentParser(description="Jetson Arduino gantry controller")
    parser.add_argument("--port", default=None, help="Arduino serial port, e.g. /dev/ttyACM0")
    parser.add_argument("--baud", type=int, default=DEFAULT_BAUD)
    parser.add_argument("--web", action="store_true", help="Run the browser UI server")
    parser.add_argument("--web-port", type=int, default=8090)
    args = parser.parse_args()

    controller = GantryController(port=args.port, baudrate=args.baud)
    controller.connect()

    if args.web:
        run_web_server(controller, port=args.web_port)
        return

    print("Connected to Arduino on", controller.port)
    print("Commands: home, stop, state, <mm target>, q")
    try:
        while True:
            height_cm = controller.motor_to_display_cm(controller.state.current_position_mm)
            cmd = input(f"gantry[{height_cm:.0f} cm]> ").strip().lower()
            if cmd in {"q", "quit", "exit"}:
                break
            if cmd == "home":
                controller.set_home_here()
                print(json.dumps(controller.get_state_dict(), indent=2))
                continue
            if cmd == "stop":
                controller.stop()
                print(json.dumps(controller.get_state_dict(), indent=2))
                continue
            if cmd == "state":
                print(json.dumps(controller.get_state_dict(), indent=2))
                continue
            controller.move_to(float(cmd))
            print(json.dumps(controller.get_state_dict(), indent=2))
    finally:
        controller.close()


if __name__ == "__main__":
    main()
