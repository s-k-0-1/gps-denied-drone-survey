"""
drone_link/simulator.py — SIMULATOR / mock link.

Replays a survey's coordinates.csv as an autonomous flight so the whole
dashboard can be developed and demoed with NO real drone:

  Idle → Takeoff → Survey (fly the CSV waypoints, "capture" each photo)
       → Returning → Landed → Data Transfer → Charging → Done

Battery drains while flying and recharges on the pad. Position, altitude,
yaw, speed and the live camera photo all update in real time.
"""
from __future__ import annotations

import csv
import math
import time
from pathlib import Path
from typing import Optional

from .base import DroneLink, MissionState

try:
    from .. import config
except ImportError:                       # allow running as a script
    import config


class SimulatorLink(DroneLink):
    link_type = "simulator"

    def __init__(self, rate_hz: float = 5.0):
        super().__init__(rate_hz=rate_hz)
        self._waypoints: list[dict] = []
        self._data_dir: Optional[Path] = None
        self._load_waypoints()

        # flight state
        self._phase = MissionState.IDLE
        self._pos = [0.0, 0.0, 0.0]        # x, y, z
        self._wp_idx = 0
        self._phase_t0 = 0.0
        self._cruise = config.CRUISE_ALT_M
        self._active = False

        with self._lock:
            self._tel.connected = True
            self._tel.waypoints_total = len(self._waypoints)
            self._tel.mode = "SIM"

    # ── data ─────────────────────────────────────────────────────────────
    def _load_waypoints(self) -> None:
        self._data_dir = config.simulator_data_dir()
        csv_path = config.active_coordinates_csv()
        if csv_path is None or not csv_path.exists():
            self._log("error", "no coordinates.csv found — simulator idle")
            return
        rows = []
        with csv_path.open(newline="", encoding="utf-8-sig") as f:
            for r in csv.DictReader(f):
                try:
                    rows.append({
                        "x": float(r["x_enu"]),
                        "y": float(r["y_enu"]),
                        "z": float(r.get("z_enu", config.CRUISE_ALT_M)),
                        "yaw": float(r.get("yaw_deg", 0.0)),
                        "image": r.get("image_file", ""),
                    })
                except (KeyError, ValueError, TypeError):
                    continue
        self._waypoints = rows
        self._log("info", f"loaded {len(rows)} waypoints from "
                          f"{csv_path.parent.name}/{csv_path.name}")

    def photo_dir(self) -> Optional[Path]:
        return self._data_dir

    # ── movement helper ──────────────────────────────────────────────────
    def _move_towards(self, tx: float, ty: float, tz: float,
                      speed: float, dt: float) -> bool:
        dx, dy, dz = tx - self._pos[0], ty - self._pos[1], tz - self._pos[2]
        dist = math.sqrt(dx * dx + dy * dy + dz * dz)
        step = speed * dt
        if dist <= max(step, 0.05):
            self._pos = [tx, ty, tz]
            self._set(speed_mps=0.0)
            return True
        self._pos[0] += dx / dist * step
        self._pos[1] += dy / dist * step
        self._pos[2] += dz / dist * step
        self._set(speed_mps=speed)
        return False

    def _publish_pos(self, yaw: Optional[float] = None) -> None:
        kw = dict(x=self._pos[0], y=self._pos[1], z=self._pos[2],
                  altitude_m=self._pos[2])
        if yaw is not None:
            kw["yaw_deg"] = yaw
        self._set(**kw)

    def _enter(self, phase: MissionState) -> None:
        self._phase = phase
        self._phase_t0 = time.time()
        self._set_state(phase)

    # ── main tick ────────────────────────────────────────────────────────
    def _tick(self, dt: float) -> None:
        speed = config.SIM_GROUND_SPEED

        # battery model
        with self._lock:
            bat = self._tel.battery_pct
        if self._phase == MissionState.CHARGING:
            bat = min(100.0, bat + config.SIM_BATTERY_CHARGE * dt)
        elif self._active and self._phase not in (MissionState.IDLE,
                                                  MissionState.DONE,
                                                  MissionState.LANDED):
            bat = max(0.0, bat - config.SIM_BATTERY_DRAIN * dt)
        self._set(battery_pct=bat)

        if self._phase == MissionState.TAKEOFF:
            if self._move_towards(0.0, 0.0, self._cruise, speed, dt):
                self._log("info", "reached cruise altitude")
                if self._waypoints:
                    self._wp_idx = 0
                    self._enter(MissionState.SURVEY)
                else:
                    self._enter(MissionState.RETURNING)
            self._publish_pos()

        elif self._phase == MissionState.SURVEY:
            if self._wp_idx >= len(self._waypoints):
                self._enter(MissionState.RETURNING)
                return
            wp = self._waypoints[self._wp_idx]
            reached = self._move_towards(wp["x"], wp["y"], wp["z"], speed, dt)
            self._publish_pos(yaw=wp["yaw"])
            self._set(waypoint=self._wp_idx + 1)
            if reached:
                self._capture(wp)
                self._wp_idx += 1

        elif self._phase == MissionState.RETURNING:
            if self._move_towards(0.0, 0.0, self._cruise, speed * 1.3, dt):
                self._enter(MissionState.LANDED)
            self._publish_pos()

        elif self._phase == MissionState.LANDED:
            if self._move_towards(0.0, 0.0, 0.0, speed, dt):
                self._publish_pos()
                if time.time() - self._phase_t0 > 1.5:
                    self._enter(MissionState.DATA_TRANSFER)
            self._publish_pos()

        elif self._phase == MissionState.DATA_TRANSFER:
            pct = min(100.0, (time.time() - self._phase_t0) / 4.0 * 100.0)
            self._set(transfer_pct=pct)
            if pct >= 100.0:
                self._log("info", "data transfer complete")
                self._enter(MissionState.CHARGING)

        elif self._phase == MissionState.CHARGING:
            if bat >= 99.9:
                self._log("info", "battery full")
                self._active = False
                self._enter(MissionState.DONE)

    # ── capture ──────────────────────────────────────────────────────────
    def _capture(self, wp: Optional[dict] = None) -> Optional[str]:
        if wp is None:
            if not self._waypoints:
                return None
            idx = min(self._wp_idx, len(self._waypoints) - 1)
            wp = self._waypoints[idx]
        name = wp.get("image") or ""
        with self._lock:
            self._tel.photos_captured += 1
            self._tel.last_photo = name
        if name:
            self._log("info", f"📷 captured {name}")
        return name

    # ── commands ─────────────────────────────────────────────────────────
    def start_mission(self) -> None:
        self._active = True
        self._pos = [0.0, 0.0, 0.0]
        self._wp_idx = 0
        self._mark_mission_start()
        with self._lock:
            self._tel.photos_captured = 0
            self._tel.transfer_pct = 0.0
            self._tel.armed = True
        self._log("info", "START MISSION — autonomous takeoff + survey")
        self._enter(MissionState.TAKEOFF)

    def takeoff(self) -> None:
        self._active = True
        self._mark_mission_start()
        self._set(armed=True)
        self._enter(MissionState.TAKEOFF)

    def survey(self) -> None:
        self._active = True
        if not self._waypoints:
            self._log("warn", "no waypoints to survey")
            return
        self._wp_idx = 0
        self._enter(MissionState.SURVEY)

    def return_land(self) -> None:
        self._enter(MissionState.RETURNING)

    def start_data_transfer(self) -> None:
        self._set(transfer_pct=0.0)
        self._enter(MissionState.DATA_TRANSFER)

    def start_charging(self) -> None:
        self._enter(MissionState.CHARGING)

    def capture_photo(self) -> Optional[str]:
        return self._capture()
