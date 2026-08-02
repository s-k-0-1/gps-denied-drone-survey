"""
pipeline_runner.py — runs your existing pipeline scripts as subprocesses and
streams their stdout/stderr line-by-line to a log callback (→ WebSocket).

Used by START MISSION (when the "Run pipeline" toggle is on) and by the
manual pipeline buttons. Only one pipeline job runs at a time.
"""
from __future__ import annotations

import os
import subprocess
import threading
import time
from typing import Callable, Optional

try:
    from . import config
except ImportError:
    import config

LogCallback = Callable[[str, str], None]   # (level, message)


# Named jobs → (script, extra args, extra env). Uses the FIXED pipeline (config.PIPELINE_MAIN) so
# START MISSION runs with base-station origin + LR + HD-720 + stage3_robust; match_only uses the
# robust matcher (config.MATCHER_MAIN). Both fall back to originals if the fixed files are missing.
# extra env: e.g. MATCH_LR=64 for the 64×64 LR-to-LR mode (result copied to results_lr64/).
JOBS: dict[str, tuple] = {
    "full":         (config.PIPELINE_MAIN, [], {}),                        # stitch + match + annotate (fixed)
    "full_3d":      (config.PIPELINE_MAIN, ["--run-3d"], {}),              # + 3D reconstruction
    "skip_stitch":  (config.PIPELINE_MAIN, ["--skip-stitch"], {}),         # reuse mosaic, fresh match
    "skip_match":   (config.PIPELINE_MAIN, ["--skip-match"], {}),          # reuse cached matches
    "match_only":   (config.MATCHER_MAIN, [], {}),                         # just the robust matcher
    "reconstruct3d": (config.SCRIPT_3D, [], {}),                           # OpenDroneMap 3D
    # 64×64 LR-to-LR mode: drone bhi 64 pe (seed-64 ↔ drone-64). Result results_lr64/ me alag copy.
    "match_lr64":   (config.PIPELINE_MAIN, ["--skip-stitch"], {"MATCH_LR": "64"}),
}


class PipelineRunner:
    def __init__(self, log_cb: Optional[LogCallback] = None):
        self._log_cb = log_cb
        self._proc: Optional[subprocess.Popen] = None
        self._thread: Optional[threading.Thread] = None
        self._lock = threading.Lock()
        self.current_job: Optional[str] = None
        self.last_exit: Optional[int] = None
        self.started_at: Optional[float] = None

    def set_log_callback(self, cb: LogCallback) -> None:
        self._log_cb = cb

    def is_running(self) -> bool:
        return self._proc is not None and self._proc.poll() is None

    def status(self) -> dict:
        return {
            "running": self.is_running(),
            "job": self.current_job,
            "last_exit": self.last_exit,
            "elapsed": (time.time() - self.started_at) if (self.started_at and self.is_running()) else 0,
        }

    def _log(self, level: str, msg: str) -> None:
        if self._log_cb:
            try:
                self._log_cb(level, msg)
            except Exception:
                pass

    def run(self, job: str) -> dict:
        with self._lock:
            if self.is_running():
                return {"ok": False, "error": f"pipeline already running ({self.current_job})"}
            if job not in JOBS:
                return {"ok": False, "error": f"unknown job '{job}'"}
            script, extra, extra_env = JOBS[job]
            if not script.exists():
                return {"ok": False, "error": f"script not found: {script.name}"}

            cmd = [config.PYTHON, str(script), *extra]
            proc_env = {**os.environ, **extra_env} if extra_env else None
            env_note = ("  [" + " ".join(f"{k}={v}" for k, v in extra_env.items()) + "]") if extra_env else ""
            self._log("info", f"$ {' '.join(cmd)}{env_note}  (cwd={config.BASE_DIR})")
            try:
                self._proc = subprocess.Popen(
                    cmd, cwd=str(config.BASE_DIR), env=proc_env,
                    stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                    text=True, bufsize=1)
            except Exception as exc:
                self._log("error", f"failed to launch pipeline: {exc}")
                return {"ok": False, "error": str(exc)}

            self.current_job = job
            self.started_at = time.time()
            self.last_exit = None
            self._thread = threading.Thread(target=self._pump, daemon=True,
                                            name="pipeline-pump")
            self._thread.start()
            return {"ok": True, "job": job}

    def _pump(self) -> None:
        proc = self._proc
        assert proc is not None and proc.stdout is not None
        for line in proc.stdout:
            self._log("pipe", line.rstrip("\n"))
        proc.wait()
        self.last_exit = proc.returncode
        dur = time.time() - (self.started_at or time.time())
        lvl = "info" if proc.returncode == 0 else "error"
        self._log(lvl, f"pipeline '{self.current_job}' finished "
                       f"(exit {proc.returncode}, {dur:.1f}s)")
        with self._lock:
            self._proc = None

    def stop(self) -> dict:
        with self._lock:
            if not self.is_running():
                return {"ok": False, "error": "no pipeline running"}
            self._log("warn", f"terminating pipeline '{self.current_job}' ...")
            try:
                self._proc.terminate()
                try:
                    self._proc.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    self._proc.kill()
            except Exception as exc:
                return {"ok": False, "error": str(exc)}
            return {"ok": True}
