#!/usr/bin/env python3
"""
Gantry control for Jetson/Go2 payload using a TMC2209 over a USB-UART bridge.

Design assumptions
------------------
- Jetson talks to a CP2102/CP2102N USB-UART adapter as /dev/ttyUSB*.
- The adapter talks to the TMC2209 UART pins.
- Motor power (VM) is supplied separately from a suitable 12V rail.
- Logic power (VIO) is 3.3V. Do not assume 5V logic is safe with a CP2102.
- Motion is open-loop. Position is tracked in software only.

This script uses the TMC2209 internal pulse generator (VACTUAL), so it can move a
motor over UART without STEP/DIR pulses. That is useful for a simple Jetson-only
prototype, but it is not a true closed-loop positioning system.
"""

from __future__ import annotations

import argparse
import glob
import json
import math
import os
import threading
import time
from dataclasses import dataclass, asdict
from typing import Optional

import serial


# ----------------------------
# User-tunable gantry config
# ----------------------------
MOTOR_STEPS_PER_REV = 200.0          # NEMA17, 1.8 deg/step
MICROSTEPS = 16                      # native driver microsteps configured in CHOPCONF
LEADSCREW_MM_PER_REV = 8.0           # physical leadscrew pitch
STEPS_PER_MM = (MOTOR_STEPS_PER_REV * MICROSTEPS) / LEADSCREW_MM_PER_REV  # = 400

# Empirical scale: 33 motor-mm = ~900mm physical travel
# Converts user-facing mm to internal motor-mm
MM_SCALE = 33.0 / 900.0

# TMC2209 UART / motion config
DEFAULT_BAUD = 115200
DEFAULT_UART_ADDRESS = 0             # MS1=GND, MS2=GND
DEFAULT_VACTUAL = 18000              # conservative default for prototype use
DEFAULT_REPLY_TIMEOUT = 0.08
DEFAULT_SEND_DELAY = 0.005
VACTUAL_TO_USTEPS_PER_SEC = 12_000_000 / (2 ** 24)  # 0.715255...
VACTUAL_MAX = (2 ** 23) - 1

# TMC2209 registers used here
REG_GCONF = 0x00
REG_IFCNT = 0x02
REG_IHOLD_IRUN = 0x10
REG_TPOWERDOWN = 0x11
REG_VACTUAL = 0x22
REG_CHOPCONF = 0x6C
REG_DRV_STATUS = 0x6F

# Minimal config values
BIT_PDN_DISABLE = 6
BIT_MSTEP_REG_SELECT = 7
BIT_INTPOL = 28
# MRES code in CHOPCONF[27:24]: 0=256,1=128,2=64,3=32,4=16,5=8,6=4,7=2,8=fullstep
MRES_CODE_BY_MICROSTEPS = {256: 0, 128: 1, 64: 2, 32: 3, 16: 4, 8: 5, 4: 6, 2: 7, 1: 8}
DEFAULT_CHOPCONF_BASE = 0x00000053   # TOFF non-zero so the driver is enabled
DEFAULT_IRUN = 20                    # 0..31 current scale (still depends on VREF / board)
DEFAULT_IHOLD = 8
DEFAULT_IHOLDDELAY = 6
DEFAULT_TPOWERDOWN = 20


@dataclass
class GantryState:
    connected: bool = False
    serial_port: Optional[str] = None
    uart_address: int = DEFAULT_UART_ADDRESS
    current_position_mm: float = 0.0
    last_target_mm: Optional[float] = None
    moving: bool = False
    homed: bool = False
    vactual: int = 0
    ifcnt: Optional[int] = None
    last_error: str = ""
    status: str = "idle"
    motor_enabled: bool = False


class TMC2209UART:
    """Very small TMC2209 UART helper using the datasheet datagram format."""

    def __init__(
        self,
        port: str,
        baudrate: int = DEFAULT_BAUD,
        address: int = DEFAULT_UART_ADDRESS,
        timeout: float = DEFAULT_REPLY_TIMEOUT,
    ) -> None:
        self.port = port
        self.baudrate = baudrate
        self.address = address & 0xFF
        self.timeout = timeout
        self._ser = serial.Serial(port=port, baudrate=baudrate, timeout=timeout)
        self._io_lock = threading.Lock()

    @staticmethod
    def _crc(datagram: bytes) -> int:
        crc = 0
        for byte in datagram:
            current = byte
            for _ in range(8):
                if ((crc >> 7) ^ (current & 0x01)):
                    crc = ((crc << 1) ^ 0x07) & 0xFF
                else:
                    crc = (crc << 1) & 0xFF
                current >>= 1
        return crc

    @staticmethod
    def _u32_to_bytes(value: int) -> bytes:
        return int(value & 0xFFFFFFFF).to_bytes(4, byteorder="big", signed=False)

    @staticmethod
    def _bytes_to_u32(data: bytes) -> int:
        return int.from_bytes(data, byteorder="big", signed=False)

    @staticmethod
    def _to_signed_24(value: int) -> int:
        value &= 0xFFFFFF
        if value & 0x800000:
            return value - (1 << 24)
        return value

    @staticmethod
    def _from_signed_24(value: int) -> int:
        if value < 0:
            value = (1 << 24) + value
        return value & 0xFFFFFF

    def close(self) -> None:
        if self._ser and self._ser.is_open:
            self._ser.close()

    def write_register(self, reg: int, value: int) -> None:
        payload = bytes([0x05, self.address, (reg & 0x7F) | 0x80]) + self._u32_to_bytes(value)
        frame = payload + bytes([self._crc(payload)])
        with self._io_lock:
            self._ser.reset_input_buffer()
            self._ser.write(frame)
            self._ser.flush()
            time.sleep(DEFAULT_SEND_DELAY)
            # read and discard possible echo bytes from the single-wire arrangement
            _ = self._ser.read(len(frame))

    def read_register(self, reg: int) -> int:
        payload = bytes([0x05, self.address, reg & 0x7F])
        frame = payload + bytes([self._crc(payload)])
        with self._io_lock:
            self._ser.reset_input_buffer()
            self._ser.write(frame)
            self._ser.flush()
            time.sleep(DEFAULT_SEND_DELAY)
            raw = self._ser.read(32)

        if not raw:
            raise RuntimeError("No UART reply from TMC2209")

        # Remove echoed request if present.
        if raw.startswith(frame):
            raw = raw[len(frame):]

        # Search for an 8-byte reply frame: [0x05, 0xFF, reg, d3, d2, d1, d0, crc]
        for i in range(max(0, len(raw) - 7)):
            candidate = raw[i:i + 8]
            if len(candidate) < 8:
                continue
            if candidate[0] != 0x05:
                continue
            if candidate[1] != 0xFF:
                continue
            if (candidate[2] & 0x7F) != (reg & 0x7F):
                continue
            if self._crc(candidate[:-1]) != candidate[-1]:
                continue
            return self._bytes_to_u32(candidate[3:7])

        raise RuntimeError(f"Invalid UART reply for reg 0x{reg:02X}: {raw.hex(' ')}")

    def set_vactual(self, value: int) -> None:
        value = max(-VACTUAL_MAX, min(VACTUAL_MAX, int(value)))
        reg_value = self._from_signed_24(value)
        self.write_register(REG_VACTUAL, reg_value)

    def get_ifcnt(self) -> int:
        return self.read_register(REG_IFCNT) & 0xFF


class GantryController:
    def __init__(
        self,
        port: Optional[str] = None,
        baudrate: int = DEFAULT_BAUD,
        uart_address: int = DEFAULT_UART_ADDRESS,
        steps_per_mm: float = STEPS_PER_MM,
        default_vactual: int = DEFAULT_VACTUAL,
    ) -> None:
        self.steps_per_mm = steps_per_mm
        self.default_vactual = int(default_vactual)
        self.port = port or self._auto_detect_port()
        self.baudrate = baudrate
        self.uart_address = uart_address
        self.driver: Optional[TMC2209UART] = None
        self.state = GantryState(serial_port=self.port, uart_address=uart_address)
        self._move_lock = threading.Lock()
        self._stop_event = threading.Event()
        self._thread: Optional[threading.Thread] = None

    @staticmethod
    def _auto_detect_port() -> Optional[str]:
        ports = sorted(glob.glob("/dev/ttyUSB*"))
        return ports[0] if ports else None

    def connect(self) -> None:
        if not self.port:
            raise RuntimeError("No /dev/ttyUSB* port found for the CP2102/USB-UART adapter")
        self.driver = TMC2209UART(self.port, self.baudrate, self.uart_address)
        self._basic_init()
        self.state.connected = True
        self.state.serial_port = self.port
        self.state.status = "ready"

    def close(self) -> None:
        try:
            self.stop()
        finally:
            if self.driver is not None:
                self.driver.close()

    def _basic_init(self) -> None:
        if self.driver is None:
            raise RuntimeError("Driver not connected")

        # 1) Enable UART mode and select register-controlled microstep resolution.
        gconf = (1 << BIT_PDN_DISABLE) | (1 << BIT_MSTEP_REG_SELECT)
        self.driver.write_register(REG_GCONF, gconf)

        # 2) Set run/hold current scales. These are driver scale values, not absolute amps.
        ihold_irun = (
            (DEFAULT_IHOLD & 0x1F)
            | ((DEFAULT_IRUN & 0x1F) << 8)
            | ((DEFAULT_IHOLDDELAY & 0x0F) << 16)
        )
        self.driver.write_register(REG_IHOLD_IRUN, ihold_irun)
        self.driver.write_register(REG_TPOWERDOWN, DEFAULT_TPOWERDOWN)

        # 3) Set microstep resolution and interpolation; keep TOFF non-zero so the bridge is enabled.
        mres_code = MRES_CODE_BY_MICROSTEPS.get(int(MICROSTEPS))
        if mres_code is None:
            raise RuntimeError(f"Unsupported MICROSTEPS value: {MICROSTEPS}")
        chopconf = DEFAULT_CHOPCONF_BASE | (1 << BIT_INTPOL) | (mres_code << 24)
        self.driver.write_register(REG_CHOPCONF, chopconf)

        # 4) Stop motion.
        self.driver.set_vactual(0)
        self.state.motor_enabled = True

        # 5) Communication sanity check.
        self.state.ifcnt = self.driver.get_ifcnt()

    def mm_to_microsteps(self, mm: float) -> float:
        return mm * self.steps_per_mm

    def microsteps_to_mm(self, microsteps: float) -> float:
        return microsteps / self.steps_per_mm

    def _vactual_to_microsteps_per_sec(self, vactual: int) -> float:
        return abs(vactual) * VACTUAL_TO_USTEPS_PER_SEC

    def set_home_here(self) -> None:
        self.state.current_position_mm = 0.0
        self.state.homed = True
        self.state.status = "homed"
        self.state.last_error = ""

    def stop(self) -> None:
        self._stop_event.set()
        if self.driver is not None:
            try:
                self.driver.set_vactual(0)
                self.state.vactual = 0
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
        if self.driver is None:
            raise RuntimeError("Driver not connected")

        with self._move_lock:
            self._stop_event.clear()
            start_mm = self.state.current_position_mm
            delta_mm = float(target_mm) - start_mm
            if abs(delta_mm) < 0.5:
                self.state.last_target_mm = float(target_mm)
                self.state.status = "at_target"
                return

            # Scale user-facing mm to internal motor-mm
            motor_delta = delta_mm * MM_SCALE
            direction = 1 if motor_delta > 0 else -1
            total_usteps = abs(self.mm_to_microsteps(motor_delta))
            target_v = max(1000, min(VACTUAL_MAX, abs(self.default_vactual)))
            target_usteps_s = self._vactual_to_microsteps_per_sec(target_v)
            if target_usteps_s <= 0:
                raise RuntimeError("Invalid VACTUAL; computed speed is zero")

            # TMC2209 VACTUAL has no ramp generator. Use a small software ramp to reduce missed steps.
            ramp_time = 0.25
            ramp_steps = 5
            ramp_distance = 0.5 * target_usteps_s * ramp_time  # per ramp
            cruise_distance = max(0.0, total_usteps - 2 * ramp_distance)
            cruise_time = cruise_distance / target_usteps_s if cruise_distance > 0 else 0.0

            self.state.moving = True
            self.state.status = "moving"
            self.state.last_target_mm = float(target_mm)
            self.state.last_error = ""

            try:
                # Ramp up
                for i in range(1, ramp_steps + 1):
                    if self._stop_event.is_set():
                        break
                    v = int(direction * target_v * (i / ramp_steps))
                    self.driver.set_vactual(v)
                    self.state.vactual = v
                    time.sleep(ramp_time / ramp_steps)

                # Cruise
                if not self._stop_event.is_set() and cruise_time > 0:
                    self.driver.set_vactual(direction * target_v)
                    self.state.vactual = direction * target_v
                    t0 = time.time()
                    while time.time() - t0 < cruise_time:
                        if self._stop_event.is_set():
                            break
                        time.sleep(0.01)

                # Ramp down
                if not self._stop_event.is_set():
                    for i in range(ramp_steps - 1, -1, -1):
                        v = int(direction * target_v * (i / ramp_steps)) if i > 0 else 0
                        self.driver.set_vactual(v)
                        self.state.vactual = v
                        time.sleep(ramp_time / ramp_steps)
                else:
                    self.driver.set_vactual(0)
                    self.state.vactual = 0

                if self._stop_event.is_set():
                    self.state.status = "stopped"
                else:
                    self.state.current_position_mm = float(target_mm)
                    self.state.status = "at_target"

                # Read IFCNT after a motion command sequence as a basic UART health check.
                self.state.ifcnt = self.driver.get_ifcnt()

            except Exception as exc:
                self.state.last_error = str(exc)
                self.state.status = "error"
                try:
                    self.driver.set_vactual(0)
                    self.state.vactual = 0
                except Exception:
                    pass
                raise
            finally:
                self.state.moving = False
                try:
                    self.driver.set_vactual(0)
                    self.state.vactual = 0
                except Exception:
                    pass

    def jog(self, delta_mm: float) -> None:
        self.move_to_async(self.state.current_position_mm + float(delta_mm))

    def read_driver_status(self) -> Optional[int]:
        if self.driver is None:
            return None
        return self.driver.read_register(REG_DRV_STATUS)

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
  <p class=\"small\">Open-loop TMC2209 UART control over CP2102 on the Jetson.</p>
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
    <pre>1) Put the carriage at your real mechanical zero.
2) Press \"Set Home Here\" once.
3) Use absolute mm moves.
4) This system does not remember position after power-off unless the carriage stays exactly where it was.
5) Add a real limit switch later for repeatable startup homing.</pre>
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
async function refresh() {
  const s = await api('/api/state');
  document.getElementById('status').innerHTML = `
    <b>Connected:</b> ${s.connected}<br>
    <b>Port:</b> ${s.serial_port || 'N/A'}<br>
    <b>Position:</b> ${Number(s.current_position_mm).toFixed(3)} mm<br>
    <b>Target:</b> ${s.last_target_mm === null ? 'N/A' : Number(s.last_target_mm).toFixed(3) + ' mm'}<br>
    <b>Status:</b> ${s.status}<br>
    <b>Moving:</b> ${s.moving}<br>
    <b>Homed:</b> ${s.homed}<br>
    <b>IFCNT:</b> ${s.ifcnt === null ? 'N/A' : s.ifcnt}<br>
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
    parser = argparse.ArgumentParser(description="Jetson TMC2209 gantry controller")
    parser.add_argument("--port", default=None, help="CP2102 serial port, e.g. /dev/ttyUSB0")
    parser.add_argument("--baud", type=int, default=DEFAULT_BAUD)
    parser.add_argument("--address", type=int, default=DEFAULT_UART_ADDRESS)
    parser.add_argument("--web", action="store_true", help="Run the browser UI server")
    parser.add_argument("--web-port", type=int, default=8090)
    args = parser.parse_args()

    controller = GantryController(port=args.port, baudrate=args.baud, uart_address=args.address)
    controller.connect()

    if args.web:
        run_web_server(controller, port=args.web_port)
        return

    print("Connected to TMC2209 on", controller.port)
    print("Commands: home, stop, state, <mm target>, q")
    try:
        while True:
            cmd = input(f"gantry[{controller.state.current_position_mm:.3f} mm]> ").strip().lower()
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
