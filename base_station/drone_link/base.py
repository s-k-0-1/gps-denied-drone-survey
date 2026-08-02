"""
drone_link/base.py — Pluggable drone-link interface.

Every link (simulator, MAVLink, or your own custom radio) subclasses
``DroneLink`` and implements the small set of command + tick hooks. The
server only ever talks to this interface, so swapping links is a one-line
change in the factory (see drone_link/__init__.py).
"""
from __future__ import annotations

import abc
import threading
import time
from dataclasses import asdict, dataclass
from enum import Enum
from typing import Callable, Optional


class MissionState(str, Enum):
    IDLE          = "Idle"
    TAKEOFF       = "Takeoff"
    SURVEY        = "Survey"
    RETURNING     = "Returning"
    LANDED        = "Landed"
    DATA_TRANSFER = "Data Transfer"
    CHARGING      = "Charging"
    DONE          = "Done"


# Ordered phases — used by the UI to light up the state pipeline.
MISSION_PHASES = [s.value for s in (
    MissionState.IDLE, MissionState.TAKEOFF, MissionState.SURVEY,
    MissionState.RETURNING, MissionState.LANDED, MissionState.DATA_TRANSFER,
    MissionState.CHARGING, MissionState.DONE,
)]


@dataclass
class Telemetry:
    connected: bool = False
    link_type: str = "none"
    state: str = MissionState.IDLE.value
    mode: str = "MANUAL"
    armed: bool = False

    battery_pct: float = 100.0
    altitude_m: float = 0.0
    x: float = 0.0          # ENU east, metres, relative to base station
    y: float = 0.0          # ENU north
    z: float = 0.0          # up / altitude
    yaw_deg: float = 0.0
    speed_mps: float = 0.0

    sortie: int = 0
    mission_elapsed_s: float = 0.0
    photos_captured: int = 0
    last_photo: Optional[str] = None
    waypoint: int = 0
    waypoints_total: int = 0
    transfer_pct: float = 0.0
    ts: float = 0.0

    def to_dict(self) -> dict:
        d = asdict(self)
        # round floats for a tidy wire format
        for k, v in d.items():
            if isinstance(v, float):
                d[k] = round(v, 3)
        return d


LogCallback = Callable[[str, str], None]   # (level, message)


class DroneLink(abc.ABC):
    """
    Base class. Runs a background thread that calls ``_tick(dt)`` at
    ``TELEMETRY_HZ``. Subclasses update ``self._tel`` (guarded by ``self._lock``)
    inside ``_tick`` and implement the command hooks.
    """
    link_type: str = "base"

    def __init__(self, rate_hz: float = 5.0):
        self._rate_hz = max(1.0, rate_hz)
        self._lock = threading.RLock()
        self._tel = Telemetry(link_type=self.link_type)
        self._running = False
        self._thread: Optional[threading.Thread] = None
        self._log_cb: Optional[LogCallback] = None
        self._mission_t0: Optional[float] = None

    # ── lifecycle ────────────────────────────────────────────────────────
    def set_log_callback(self, cb: LogCallback) -> None:
        self._log_cb = cb

    def start(self) -> bool:
        ok = self._connect()
        self._running = True
        self._thread = threading.Thread(target=self._run, daemon=True,
                                        name=f"{self.link_type}-link")
        self._thread.start()
        self._log("info", f"{self.link_type} link started "
                          f"({'connected' if ok else 'no hardware'})")
        return ok

    def stop(self) -> None:
        self._running = False
        if self._thread is not None:
            self._thread.join(timeout=2.0)
        self._disconnect()
        self._log("info", f"{self.link_type} link stopped")

    def _run(self) -> None:
        period = 1.0 / self._rate_hz
        last = time.time()
        while self._running:
            now = time.time()
            dt = now - last
            last = now
            try:
                self._tick(dt)
            except Exception as exc:                       # never kill the loop
                self._log("error", f"tick error: {exc}")
            with self._lock:
                self._tel.ts = now
                if self._mission_t0 is not None:
                    self._tel.mission_elapsed_s = now - self._mission_t0
            time.sleep(max(0.0, period - (time.time() - now)))

    # ── telemetry access ────────────────────────────────────────────────
    def get_telemetry(self) -> dict:
        with self._lock:
            return self._tel.to_dict()

    def _set(self, **kwargs) -> None:
        with self._lock:
            for k, v in kwargs.items():
                setattr(self._tel, k, v)

    def _set_state(self, state: MissionState) -> None:
        with self._lock:
            if self._tel.state != state.value:
                self._log("info", f"state → {state.value}")
            self._tel.state = state.value

    def _mark_mission_start(self) -> None:
        self._mission_t0 = time.time()
        with self._lock:
            self._tel.mission_elapsed_s = 0.0

    def _log(self, level: str, msg: str) -> None:
        if self._log_cb is not None:
            try:
                self._log_cb(level, f"[{self.link_type}] {msg}")
            except Exception:
                pass

    # ── hooks subclasses implement ───────────────────────────────────────
    def _connect(self) -> bool:
        return True

    def _disconnect(self) -> None:
        pass

    @abc.abstractmethod
    def _tick(self, dt: float) -> None:
        ...

    # ── commands (default = log + state; override as needed) ─────────────
    @abc.abstractmethod
    def start_mission(self) -> None: ...

    @abc.abstractmethod
    def takeoff(self) -> None: ...

    @abc.abstractmethod
    def survey(self) -> None: ...

    @abc.abstractmethod
    def return_land(self) -> None: ...

    @abc.abstractmethod
    def start_data_transfer(self) -> None: ...

    @abc.abstractmethod
    def start_charging(self) -> None: ...

    @abc.abstractmethod
    def capture_photo(self) -> Optional[str]: ...

    def new_sortie(self) -> None:
        with self._lock:
            self._tel.sortie += 1
            self._tel.state = MissionState.IDLE.value
            self._tel.transfer_pct = 0.0
            self._tel.waypoint = 0
        self._log("info", f"sortie #{self._tel.sortie} armed")
