"""
drone_link — pluggable drone telemetry/command links.

LinkManager owns the *currently active* link and can hot-swap between the
MAVLink link and the simulator at runtime (the UI exposes this). MAVLink is
tried first; if it can't connect, we fall back to the simulator so the
dashboard is always usable.
"""
from __future__ import annotations

from typing import Optional

from .base import DroneLink, MissionState, Telemetry, MISSION_PHASES, LogCallback
from .simulator import SimulatorLink
from .mavlink_link import MavlinkLink

__all__ = ["DroneLink", "MissionState", "Telemetry", "MISSION_PHASES",
           "SimulatorLink", "MavlinkLink", "LinkManager"]


class LinkManager:
    """Holds the active link and supports runtime switching with sim fallback."""

    def __init__(self, log_cb: Optional[LogCallback] = None,
                 rate_hz: float = 5.0):
        self._log_cb = log_cb
        self._rate_hz = rate_hz
        self.link: Optional[DroneLink] = None
        self.mode: str = "simulator"

    def _make(self, mode: str) -> tuple[DroneLink, str]:
        """Return (link, effective_mode). Falls back to simulator on failure."""
        mode = (mode or "simulator").lower()
        if mode in ("mavlink", "auto"):
            if MavlinkLink.available():
                link = MavlinkLink(rate_hz=self._rate_hz)
                link.set_log_callback(self._log_cb)
                if link.start():
                    return link, "mavlink"
                # connect() failed → stop and fall back
                link.stop()
                if self._log_cb:
                    self._log_cb("warn", "MAVLink unavailable → simulator fallback")
            else:
                if self._log_cb:
                    self._log_cb("warn", "pymavlink missing → simulator fallback")
            sim = SimulatorLink(rate_hz=self._rate_hz)
            sim.set_log_callback(self._log_cb)
            sim.start()
            return sim, "simulator"

        # explicit simulator
        sim = SimulatorLink(rate_hz=self._rate_hz)
        sim.set_log_callback(self._log_cb)
        sim.start()
        return sim, "simulator"

    def start(self, mode: str) -> str:
        self.stop()
        self.link, self.mode = self._make(mode)
        return self.mode

    def switch(self, mode: str) -> str:
        return self.start(mode)

    def stop(self) -> None:
        if self.link is not None:
            try:
                self.link.stop()
            except Exception:
                pass
            self.link = None

    # convenience pass-throughs
    def telemetry(self) -> dict:
        if self.link is None:
            return Telemetry().to_dict()
        return self.link.get_telemetry()
