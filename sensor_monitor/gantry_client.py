"""
HTTP client for gantry_control.py's FastAPI running on the Jetson.

Uses only stdlib (urllib) so no extra dependencies are needed on the PC.

Usage:
    client = GantryClient("192.168.0.207", port=8090)
    client.home_here()
    client.move_to(1500)
    client.wait_for_arrival(timeout=120)
    state = client.get_state()
"""

from __future__ import annotations

import json
import time
import urllib.request
import urllib.error
from typing import Optional


class GantryClient:
    """Thin HTTP wrapper around gantry_control.py FastAPI endpoints."""

    def __init__(self, host: str, port: int = 8090, timeout: float = 5.0) -> None:
        self.base_url = f"http://{host}:{port}"
        self.timeout = timeout

    def _get(self, path: str) -> dict:
        url = f"{self.base_url}{path}"
        req = urllib.request.Request(url)
        with urllib.request.urlopen(req, timeout=self.timeout) as resp:
            return json.loads(resp.read().decode())

    def _post(self, path: str, body: Optional[dict] = None) -> dict:
        url = f"{self.base_url}{path}"
        data = json.dumps(body or {}).encode()
        req = urllib.request.Request(url, data=data, method="POST",
                                     headers={"Content-Type": "application/json"})
        with urllib.request.urlopen(req, timeout=self.timeout) as resp:
            return json.loads(resp.read().decode())

    def get_state(self) -> dict:
        """GET /api/state → GantryState dict."""
        return self._get("/api/state")

    def move_to(self, target_mm: float) -> dict:
        """POST /api/move_to — starts an async move on the server."""
        return self._post("/api/move_to", {"target_mm": float(target_mm)})

    def jog(self, delta_mm: float) -> dict:
        """POST /api/jog — relative move."""
        return self._post("/api/jog", {"delta_mm": float(delta_mm)})

    def stop(self) -> dict:
        """POST /api/stop — emergency stop motor."""
        return self._post("/api/stop")

    def home_here(self) -> dict:
        """POST /api/home_here — set current position as 0mm."""
        return self._post("/api/home_here")

    def wait_for_arrival(self, timeout: float = 120.0, poll_interval: float = 0.5) -> bool:
        """Poll state until gantry stops moving. Returns True if at_target, False on timeout/error.

        Handles the race condition where move_to_async() hasn't set moving=True yet
        by requiring we see moving=True at least once before trusting moving=False.
        """
        time.sleep(0.15)  # let async thread start and set moving=True
        saw_moving = False
        t0 = time.time()
        while time.time() - t0 < timeout:
            try:
                state = self.get_state()
            except (urllib.error.URLError, OSError):
                time.sleep(poll_interval)
                continue
            if state.get("moving", False):
                saw_moving = True
            if not state.get("moving", True) and saw_moving:
                # Only trust "not moving" after we've confirmed it was moving
                return state.get("status", "") != "error"
            time.sleep(poll_interval)
        return False

    def is_at_home(self, tolerance_mm: float = 2.0) -> bool:
        """Return True if gantry is near 0mm and not moving."""
        try:
            state = self.get_state()
        except (urllib.error.URLError, OSError):
            return False
        if state.get("moving", True):
            return False
        pos = state.get("current_position_mm", 999.0)
        return abs(pos) <= tolerance_mm

    def is_reachable(self) -> bool:
        """Quick connectivity check."""
        try:
            self.get_state()
            return True
        except (urllib.error.URLError, OSError):
            return False
