"""
drone_link/mavlink_link.py — MAVLink link (PX4 / ArduPilot via pymavlink).

This is the PRIMARY link. It connects to a MAVLink endpoint (UDP/serial),
reads live telemetry, and maps the dashboard commands to MAVLink commands.

If pymavlink is not installed, or no heartbeat is seen within the timeout,
``_connect()`` returns False and the link manager transparently falls back
to the simulator — so the UI always comes up.

Connection string (config.MAVLINK_CONN), examples:
    udpin:0.0.0.0:14550     # listen for telemetry (default, works with SITL)
    udp:127.0.0.1:14550
    tcp:127.0.0.1:5760
    /dev/ttyUSB0,57600      # serial radio
"""
from __future__ import annotations

import time
from typing import Optional

from .base import DroneLink, MissionState

try:
    from .. import config
except ImportError:
    import config

try:
    from pymavlink import mavutil
    _HAVE_PYMAVLINK = True
except Exception:                          # pragma: no cover - optional dep
    mavutil = None
    _HAVE_PYMAVLINK = False


class MavlinkLink(DroneLink):
    link_type = "mavlink"

    def __init__(self, rate_hz: float = 5.0,
                 conn_str: Optional[str] = None,
                 heartbeat_timeout: Optional[float] = None):
        super().__init__(rate_hz=rate_hz)
        self._conn_str = conn_str or config.MAVLINK_CONN
        self._hb_timeout = (heartbeat_timeout if heartbeat_timeout is not None
                            else config.MAVLINK_HEARTBEAT_TIMEOUT)
        self._m = None                      # mavutil connection
        self._target_sys = 1
        self._target_comp = 1
        self._home_set = False
        self._home = (0.0, 0.0, 0.0)

    @staticmethod
    def available() -> bool:
        return _HAVE_PYMAVLINK

    # ── connection ───────────────────────────────────────────────────────
    def _connect(self) -> bool:
        if not _HAVE_PYMAVLINK:
            self._log("warn", "pymavlink not installed — cannot use MAVLink")
            return False
        try:
            self._log("info", f"connecting to {self._conn_str} ...")
            self._m = mavutil.mavlink_connection(self._conn_str, autoreconnect=True)
            hb = self._m.wait_heartbeat(timeout=self._hb_timeout)
            if hb is None:
                self._log("warn", f"no heartbeat within {self._hb_timeout:.0f}s")
                return False
            self._target_sys = self._m.target_system
            self._target_comp = self._m.target_component
            self._log("info", f"heartbeat from sys {self._target_sys} "
                              f"comp {self._target_comp}")
            self._request_streams()
            self._set(connected=True)
            return True
        except Exception as exc:
            self._log("error", f"connect failed: {exc}")
            return False

    def _disconnect(self) -> None:
        if self._m is not None:
            try:
                self._m.close()
            except Exception:
                pass
            self._m = None

    def _request_streams(self) -> None:
        """Ask the autopilot for a reasonable telemetry stream rate."""
        try:
            self._m.mav.request_data_stream_send(
                self._target_sys, self._target_comp,
                mavutil.mavlink.MAV_DATA_STREAM_ALL, 5, 1)
        except Exception as exc:
            self._log("warn", f"stream request failed: {exc}")

    # ── telemetry ────────────────────────────────────────────────────────
    def _tick(self, dt: float) -> None:
        if self._m is None:
            return
        # drain all pending messages this tick
        for _ in range(200):
            msg = self._m.recv_match(blocking=False)
            if msg is None:
                break
            self._handle(msg)

    def _handle(self, msg) -> None:
        t = msg.get_type()
        if t == "HEARTBEAT":
            armed = bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
            mode = mavutil.mode_string_v10(msg) if hasattr(mavutil, "mode_string_v10") else ""
            self._set(armed=armed, mode=mode or self._tel_mode())
            self._set(connected=True)
        elif t == "LOCAL_POSITION_NED":
            # NED → ENU (east=y, north=x, up=-z) relative to EKF origin/home
            self._set(x=float(msg.y), y=float(msg.x), z=float(-msg.z),
                      altitude_m=float(-msg.z),
                      speed_mps=float((msg.vx ** 2 + msg.vy ** 2) ** 0.5))
        elif t == "GLOBAL_POSITION_INT":
            self._set(altitude_m=float(msg.relative_alt) / 1000.0,
                      yaw_deg=float(msg.hdg) / 100.0 if msg.hdg != 65535 else self._tel.yaw_deg)
        elif t == "VFR_HUD":
            self._set(speed_mps=float(msg.groundspeed),
                      altitude_m=float(msg.alt),
                      yaw_deg=float(msg.heading))
        elif t == "ATTITUDE":
            import math
            self._set(yaw_deg=(math.degrees(msg.yaw) % 360.0))
        elif t in ("BATTERY_STATUS", "SYS_STATUS"):
            rem = getattr(msg, "battery_remaining", -1)
            if rem is not None and rem >= 0:
                self._set(battery_pct=float(rem))
            else:
                # FC ne % nahi diya (-1 = battery capacity unconfigured) -> VOLTAGE se estimate.
                volt_mv = getattr(msg, "voltage_battery", 0) or 0        # SYS_STATUS (mV)
                if (not volt_mv or volt_mv == 65535) and hasattr(msg, "voltages"):
                    try:                                                 # BATTERY_STATUS voltages[] (mV)
                        vs = [x for x in msg.voltages if x not in (0, 65535)]
                        volt_mv = vs[0] if vs else 0
                    except Exception:
                        volt_mv = 0
                v = float(volt_mv) / 1000.0
                if v > 1.0:
                    cells = max(1, round(v / 3.7))                       # auto cell-count (~3.7V/cell)
                    vpc = v / cells
                    pct = max(0.0, min(100.0, (vpc - 3.30) / (4.20 - 3.30) * 100.0))
                    self._set(battery_pct=round(pct, 1))
        elif t == "STATUSTEXT":
            try:
                self._log("info", f"FC: {msg.text}")
            except Exception:
                pass

    def _tel_mode(self) -> str:
        with self._lock:
            return self._tel.mode

    # ── command helpers ──────────────────────────────────────────────────
    def _cmd_long(self, command, *params) -> None:
        if self._m is None:
            return
        p = list(params) + [0.0] * (7 - len(params))
        try:
            self._m.mav.command_long_send(
                self._target_sys, self._target_comp, command, 0,
                p[0], p[1], p[2], p[3], p[4], p[5], p[6])
        except Exception as exc:
            self._log("error", f"command {command} failed: {exc}")

    def _set_mode(self, mode_name: str) -> None:
        if self._m is None:
            return
        try:
            mapping = self._m.mode_mapping() or {}
            if mode_name in mapping:
                self._m.set_mode(mapping[mode_name])
                self._log("info", f"set mode {mode_name}")
            else:
                self._log("warn", f"mode {mode_name} not available on this FC")
        except Exception as exc:
            self._log("error", f"set_mode {mode_name} failed: {exc}")

    def _arm(self) -> None:
        self._cmd_long(mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 1)
        self._log("info", "arm command sent")

    # ── commands ─────────────────────────────────────────────────────────
    def start_mission(self) -> None:
        self._mark_mission_start()
        self._set(armed=True)
        self._set_state(MissionState.TAKEOFF)
        self._arm()
        self._cmd_long(mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0,
                       config.CRUISE_ALT_M)
        # AUTO runs the uploaded survey mission; MISSION_START kicks it off
        self._set_mode("AUTO")
        self._cmd_long(mavutil.mavlink.MAV_CMD_MISSION_START)
        self._set_state(MissionState.SURVEY)
        self._log("info", "START MISSION — arm + takeoff + AUTO survey")

    def takeoff(self) -> None:
        self._mark_mission_start()
        self._arm()
        self._cmd_long(mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0,
                       config.CRUISE_ALT_M)
        self._set_state(MissionState.TAKEOFF)

    def survey(self) -> None:
        self._set_mode("AUTO")
        self._cmd_long(mavutil.mavlink.MAV_CMD_MISSION_START)
        self._set_state(MissionState.SURVEY)

    def return_land(self) -> None:
        self._cmd_long(mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH)
        self._set_mode("RTL")
        self._set_state(MissionState.RETURNING)

    def start_data_transfer(self) -> None:
        # Post-landing base-station op — typically custom (no standard MAVLink).
        self._set(transfer_pct=0.0)
        self._set_state(MissionState.DATA_TRANSFER)
        self._log("info", "data transfer (custom payload command hook)")

    def start_charging(self) -> None:
        self._set_state(MissionState.CHARGING)
        self._log("info", "charging (custom base-station command hook)")

    def capture_photo(self) -> Optional[str]:
        # Best-effort camera trigger.
        self._cmd_long(mavutil.mavlink.MAV_CMD_IMAGE_START_CAPTURE,
                       0, 0, 1, 0, 0, 0, 0)
        with self._lock:
            self._tel.photos_captured += 1
        self._log("info", "image capture command sent")
        return None
