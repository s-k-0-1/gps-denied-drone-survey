"""
server.py — FastAPI backend for the ASCEND Base Station dashboard.

Run (from ~/advanced_matcher):
    python3 -m base_station.server
or (from inside base_station/):
    python3 server.py

Then open http://localhost:8000  (or http://<wsl-ip>:8000 from Windows).
"""
from __future__ import annotations

import asyncio
import json
import threading
import time
from collections import deque
from contextlib import asynccontextmanager
from pathlib import Path
from typing import Optional

from fastapi import FastAPI, Request, WebSocket, WebSocketDisconnect
from fastapi.responses import FileResponse, JSONResponse, Response
from fastapi.staticfiles import StaticFiles

try:
    from . import config
    from .drone_link import LinkManager, MISSION_PHASES
    from .pipeline_runner import PipelineRunner, JOBS
    from . import results_store
except ImportError:                        # running as plain script
    import config
    from drone_link import LinkManager, MISSION_PHASES
    from pipeline_runner import PipelineRunner, JOBS
    import results_store


STATIC_DIR = Path(__file__).resolve().parent / "static"


# ──────────────────────────────────────────────────────────────────────────
# WebSocket hub (thread-safe publish from link / pipeline / watcher threads)
# ──────────────────────────────────────────────────────────────────────────
class Hub:
    def __init__(self, history: int = 400):
        self.clients: set[WebSocket] = set()
        self.loop: Optional[asyncio.AbstractEventLoop] = None
        self.log_history: deque = deque(maxlen=history)
        self.dock_history: deque = deque(maxlen=history)

    def set_loop(self, loop) -> None:
        self.loop = loop

    async def register(self, ws: WebSocket) -> None:
        await ws.accept()
        self.clients.add(ws)

    def unregister(self, ws: WebSocket) -> None:
        self.clients.discard(ws)

    async def _broadcast(self, msg: dict) -> None:
        data = json.dumps(msg, default=str)
        dead = []
        for ws in list(self.clients):
            try:
                await ws.send_text(data)
            except Exception:
                dead.append(ws)
        for ws in dead:
            self.clients.discard(ws)

    def publish(self, msg: dict) -> None:
        """Thread-safe: schedule a broadcast on the event loop."""
        if msg.get("type") == "log":
            self.log_history.append(msg)
        elif msg.get("type") == "dock":
            self.dock_history.append(msg)
        if self.loop is not None and not self.loop.is_closed():
            try:
                asyncio.run_coroutine_threadsafe(self._broadcast(msg), self.loop)
            except RuntimeError:
                pass


hub = Hub()


def emit_log(level: str, message: str) -> None:
    hub.publish({
        "type": "log",
        "data": {"t": time.time(), "level": level, "msg": message},
    })


def emit_dock(message: str, level: str = "esp") -> None:
    """Broadcast a docking/charging line to the SEPARATE dashboard panel."""
    hub.publish({
        "type": "dock",
        "data": {"t": time.time(), "level": level, "msg": message},
    })


# ── Docking (ESP32) state + control ──────────────────────────────────────
dock_state = {
    "esp_ip": (config.ESP_HOST or None),
    "docking_armed": False,       # landing received, waiting out the delay
    "last_landed": None,
}


def _esp_url(path: str) -> Optional[str]:
    host = dock_state["esp_ip"]
    if not host:
        return None
    if "://" not in host:
        host = "http://" + host
    return host.rstrip("/") + path


def _esp_call(path: str) -> bool:
    """Fire an HTTP GET at the ESP (blocking — call from a thread)."""
    import urllib.request
    url = _esp_url(path)
    if url is None:
        emit_dock(f"cannot reach ESP: no IP registered yet (wanted {path})", "warn")
        return False
    try:
        with urllib.request.urlopen(url, timeout=4) as r:
            r.read()
        emit_dock(f"→ ESP {path}  (ok)", "cmd")
        return True
    except Exception as exc:
        emit_dock(f"→ ESP {path} FAILED: {exc}", "warn")
        return False


def _trigger_docking() -> None:
    dock_state["docking_armed"] = False
    emit_dock(f"{config.DOCK_DELAY_S:.0f}s elapsed — commanding ESP to dock", "cmd")
    _esp_call("/landed")


def arm_docking_after_landing() -> None:
    """Called when the drone lands: wait DOCK_DELAY_S, then dock."""
    import threading
    dock_state["docking_armed"] = True
    dock_state["last_landed"] = time.time()
    emit_dock(f"Drone LANDED — docking starts in {config.DOCK_DELAY_S:.0f}s", "info")
    emit_log("info", "drone landed — docking sequence armed")
    threading.Timer(config.DOCK_DELAY_S, _trigger_docking).start()


def on_files_changed(kind: str) -> None:
    if kind in ("targets", "results"):
        hub.publish({"type": "refresh", "what": "targets",
                     "summary": results_store.read_summary()})
    else:
        hub.publish({"type": "refresh", "what": kind})


# ──────────────────────────────────────────────────────────────────────────
# Global app state
# ──────────────────────────────────────────────────────────────────────────
links = LinkManager(log_cb=emit_log, rate_hz=config.TELEMETRY_HZ)
pipeline = PipelineRunner(log_cb=emit_log)
watcher = results_store.ResultsWatcher(on_files_changed)

settings = {
    "run_pipeline_on_mission": config.RUN_PIPELINE_ON_MISSION,
    "pipeline_job": "full",
}


# ──────────────────────────────────────────────────────────────────────────
# Lifespan: start link, watcher, telemetry broadcaster
# ──────────────────────────────────────────────────────────────────────────
@asynccontextmanager
async def lifespan(app: FastAPI):
    hub.set_loop(asyncio.get_running_loop())
    # Link connect ko BACKGROUND thread me karo -> dashboard TURANT up ho jaye. MAVLink connect +
    # heartbeat-wait (6s tak) startup ko block na kare (warna "Waiting for application startup" pe
    # atak jaata). telemetry() link-start se pehle simulator defaults deta -> safe. Connect hone pe
    # real telemetry apne aap aane lagegi.
    def _start_link():
        try:
            mode = links.start(config.LINK_MODE)
            emit_log("info", f"drone link active: {mode}")
        except Exception as exc:
            emit_log("error", f"link start failed: {exc}")
    threading.Thread(target=_start_link, daemon=True, name="link-start").start()

    if watcher.start():
        emit_log("info", "file-watcher active on results/ + photos")
    else:
        emit_log("warn", "watchdog not available — auto-refresh via polling only")

    task = asyncio.create_task(_telemetry_loop())
    try:
        yield
    finally:
        task.cancel()
        watcher.stop()
        links.stop()


async def _telemetry_loop():
    period = 1.0 / max(1.0, config.TELEMETRY_HZ)
    while True:
        tel = links.telemetry()
        tel["link_mode"] = links.mode
        hub.publish({"type": "telemetry", "data": tel})
        await asyncio.sleep(period)


app = FastAPI(title="ASCEND Base Station", lifespan=lifespan)


def _check_auth(request) -> bool:
    import base64
    import secrets
    hdr = request.headers.get("Authorization", "")
    if not hdr.startswith("Basic "):
        return False
    try:
        user, _, pw = base64.b64decode(hdr[6:]).decode("utf-8").partition(":")
    except Exception:
        return False
    return (secrets.compare_digest(user, config.AUTH_USER)
            and secrets.compare_digest(pw, config.AUTH_PASS))


# Machine-to-machine endpoints: gated by ?token= instead of the browser password
# (the Jetson and ESP32 can't do an interactive login).
_MACHINE_PATHS = ("/api/landed", "/api/dock_log", "/api/dock_register", "/api/transfer_done")


@app.middleware("http")
async def gate(request, call_next):
    """Password gate (HTTP Basic for humans, token for machines) + no-cache."""
    path = request.url.path
    if path in _MACHINE_PATHS:
        if request.query_params.get("token") != config.MACHINE_TOKEN:
            return Response(status_code=401)
    elif config.AUTH_ENABLED and not _check_auth(request):
        return Response(
            status_code=401,
            headers={"WWW-Authenticate": 'Basic realm="ASCEND Base Station"'},
        )
    resp = await call_next(request)
    resp.headers["Cache-Control"] = "no-store, no-cache, must-revalidate, max-age=0"
    resp.headers["Pragma"] = "no-cache"
    return resp


# ──────────────────────────────────────────────────────────────────────────
# Pages / static
# ──────────────────────────────────────────────────────────────────────────
app.mount("/static", StaticFiles(directory=str(STATIC_DIR)), name="static")


@app.get("/")
async def index():
    return FileResponse(str(STATIC_DIR / "index.html"))


@app.get("/favicon.ico")
async def favicon():
    return Response(status_code=204)


# ──────────────────────────────────────────────────────────────────────────
# State / config APIs
# ──────────────────────────────────────────────────────────────────────────
@app.get("/api/state")
async def api_state():
    tel = links.telemetry()
    tel["link_mode"] = links.mode
    return {
        "telemetry": tel,
        "phases": MISSION_PHASES,
        "settings": settings,
        "pipeline": pipeline.status(),
        "jobs": list(JOBS.keys()),
        "summary": results_store.read_summary(),
        "logs": list(hub.log_history),
    }


@app.get("/api/targets")
async def api_targets():
    return results_store.read_summary()


@app.get("/api/config")
async def api_config():
    csv_path = config.active_coordinates_csv()
    sim_dir = config.simulator_data_dir()
    return {
        "team": config.TEAM_NAME,
        "drone": config.DRONE_NAME,
        "base_dir": str(config.BASE_DIR),
        "results_dir": str(config.RESULTS_DIR),
        "coordinates_csv": str(csv_path) if csv_path else None,
        "simulator_data_dir": str(sim_dir) if sim_dir else None,
        "model_glb": str(config.find_model_glb() or ""),
        "link_mode": links.mode,
        "result_set": config.ACTIVE_RESULT_SET,
        "result_sets": config.available_result_sets(),
        "scripts": {
            "iroc_pipeline": config.IROC_PIPELINE.exists(),
            "iroc_pipeline_fixed": config.IROC_PIPELINE_FIXED.exists(),
            "fused_search": config.FUSED_SEARCH.exists(),
            "stage3_robust": config.STAGE3_ROBUST.exists(),
            "3d": config.SCRIPT_3D.exists(),
            "pipeline_active": config.PIPELINE_MAIN.name,   # jo actually chalta hai
            "matcher_active": config.MATCHER_MAIN.name,
        },
    }


# ──────────────────────────────────────────────────────────────────────────
# Commands
# ──────────────────────────────────────────────────────────────────────────
_COMMANDS = {
    "start_mission": "start_mission",
    "takeoff": "takeoff",
    "survey": "survey",
    "return_land": "return_land",
    "data_transfer": "start_data_transfer",
    "charging": "start_charging",
    "capture": "capture_photo",
    "new_sortie": "new_sortie",
}


@app.post("/api/command/{name}")
async def api_command(name: str):
    if name not in _COMMANDS:
        return JSONResponse({"ok": False, "error": f"unknown command '{name}'"}, 400)
    link = links.link
    if link is None:
        return JSONResponse({"ok": False, "error": "no active link"}, 503)

    method = getattr(link, _COMMANDS[name])
    try:
        result = method()
    except Exception as exc:
        emit_log("error", f"command {name} failed: {exc}")
        return JSONResponse({"ok": False, "error": str(exc)}, 500)

    extra = {}
    # START MISSION may also kick off the pipeline (UI toggle).
    if name == "start_mission" and settings["run_pipeline_on_mission"]:
        job = settings["pipeline_job"]
        if config.PIPELINE_MAIN.exists() or config.MATCHER_MAIN.exists():
            r = pipeline.run(job)
            extra["pipeline"] = r
            if not r.get("ok"):
                emit_log("warn", f"pipeline not started: {r.get('error')}")
        else:
            emit_log("warn", "pipeline scripts not found — running drone only")

    return {"ok": True, "command": name, "result": result, **extra}


# ──────────────────────────────────────────────────────────────────────────
# Link switching + settings
# ──────────────────────────────────────────────────────────────────────────
@app.post("/api/link/{mode}")
async def api_link(mode: str):
    if mode not in ("mavlink", "simulator", "auto"):
        return JSONResponse({"ok": False, "error": "mode must be mavlink|simulator|auto"}, 400)
    effective = links.switch(mode)
    emit_log("info", f"link switched → requested '{mode}', active '{effective}'")
    return {"ok": True, "requested": mode, "active": effective}


@app.post("/api/settings")
async def api_settings(payload: dict):
    if "run_pipeline_on_mission" in payload:
        settings["run_pipeline_on_mission"] = bool(payload["run_pipeline_on_mission"])
    if "pipeline_job" in payload and payload["pipeline_job"] in JOBS:
        settings["pipeline_job"] = payload["pipeline_job"]
    emit_log("info", f"settings: {settings}")
    return {"ok": True, "settings": settings}


# ──────────────────────────────────────────────────────────────────────────
# Pipeline control
# ──────────────────────────────────────────────────────────────────────────
@app.post("/api/pipeline/{job}")
async def api_pipeline(job: str):
    r = pipeline.run(job)
    code = 200 if r.get("ok") else 400
    return JSONResponse(r, code)


@app.post("/api/pipeline_stop")
async def api_pipeline_stop():
    return JSONResponse(pipeline.stop())


@app.post("/api/result_set/{name}")
async def api_result_set(name: str):
    """Switch the dashboard's result view: 'default' -> results/, 'lr64' -> results_lr64/."""
    ok = config.set_result_set(name)
    emit_log("info", f"result view -> {config.ACTIVE_RESULT_SET}"
             + ("" if ok else f"  (results_{name}/ not found)"))
    return JSONResponse({"ok": ok, "result_set": config.ACTIVE_RESULT_SET})


# ──────────────────────────────────────────────────────────────────────────
# Docking / ESP32 (machine + dashboard control)
# ──────────────────────────────────────────────────────────────────────────
@app.post("/api/dock_register")
async def api_dock_register(request: Request):
    """ESP32 self-registers its current IP on boot / periodically."""
    ip = request.query_params.get("ip", "").strip()
    if not ip:
        return JSONResponse({"ok": False, "error": "missing ip"}, 400)
    dock_state["esp_ip"] = ip
    emit_dock(f"ESP32 registered at {ip}", "info")
    return {"ok": True, "esp_ip": ip}


@app.post("/api/landed")
async def api_landed(request: Request):
    """Called by the Jetson (MAVLink landed_state) when the drone touches down."""
    arm_docking_after_landing()
    return {"ok": True, "docking_in_s": config.DOCK_DELAY_S, "esp_ip": dock_state["esp_ip"]}


@app.post("/api/transfer_done")
async def api_transfer_done(request: Request):
    """Called by the Jetson AFTER the rsync completes → auto-run the pipeline.
    Streams the pipeline's live output to the dashboard Logs panel."""
    if not settings.get("run_pipeline_on_mission", True):
        emit_log("info", "photos received — auto-pipeline is OFF (toggle in the top bar)")
        return {"ok": True, "ran": False, "reason": "auto-pipeline disabled"}
    if pipeline.is_running():
        emit_log("warn", "photos received — pipeline already running, not restarting")
        return {"ok": True, "ran": False, "reason": "already running"}
    job = settings.get("pipeline_job", "full")
    emit_log("info", f"data transfer complete → auto-running pipeline ({job})")
    r = pipeline.run(job)
    if not r.get("ok"):
        emit_log("error", f"auto-pipeline failed to start: {r.get('error')}")
    return {"ok": r.get("ok", False), "ran": r.get("ok", False), **r}


@app.post("/api/dock_log")
async def api_dock_log(request: Request):
    """ESP32 posts its serial log lines here → shown in the Docking panel."""
    body = (await request.body()).decode("utf-8", "replace")
    for line in body.splitlines():
        line = line.rstrip()
        if line:
            emit_dock(line, "esp")
    return {"ok": True}


# Human (dashboard, password-gated) controls — proxy to the ESP.
@app.post("/api/dock/start")
async def api_dock_start():
    import asyncio as _a
    ok = await _a.to_thread(_esp_call, "/landed")
    return {"ok": ok}


@app.post("/api/dock/stop")
async def api_dock_stop():
    import asyncio as _a
    ok = await _a.to_thread(_esp_call, "/dockstop")
    return {"ok": ok}


@app.get("/api/dock_state")
async def api_dock_state():
    return {**dock_state, "logs": list(hub.dock_history)}


@app.get("/api/pipeline_status")
async def api_pipeline_status():
    return pipeline.status()


# ──────────────────────────────────────────────────────────────────────────
# Image / model serving
# ──────────────────────────────────────────────────────────────────────────
def _img(path: Path, media="image/jpeg"):
    if not path.exists():
        return JSONResponse({"error": "not found", "path": str(path)}, 404)
    return FileResponse(str(path), media_type=media,
                        headers={"Cache-Control": "no-cache"})


def _resolve_photo(name: str) -> Optional[Path]:
    name = Path(name).name                  # prevent path traversal
    cands = [config.DRONE_PHOTOS_DIR / name]
    sd = config.newest_survey_dir()
    if sd:
        cands.append(sd / name)
    for c in cands:
        if c.exists():
            return c
    for base in (config.DRONE_PHOTOS_DIR, config.SURVEY_DIR):
        if base.exists():
            for p in base.rglob(name):
                return p
    return None


def _latest_photo() -> Optional[Path]:
    dirs = []
    if config.DRONE_PHOTOS_DIR.exists():
        dirs.append(config.DRONE_PHOTOS_DIR)
    sd = config.newest_survey_dir()
    if sd:
        dirs.append(sd)
    newest, mt = None, -1.0
    for d in dirs:
        for p in d.iterdir():
            if p.is_file() and p.suffix in config.IMAGE_EXTS:
                m = p.stat().st_mtime
                if m > mt:
                    mt, newest = m, p
    return newest


@app.get("/api/image/annotated")
async def img_annotated():
    return _img(config.ANNOTATED_MAP)


@app.get("/api/image/ortho")
async def img_ortho():
    p = config.STITCH_DIR / "orthomosaic_preview.jpg"
    return _img(p if p.exists() else config.ORTHOMOSAIC)


@app.get("/api/image/rectified")
async def img_rectified():
    return _img(config.RECTIFIED_FIELD)


@app.get("/api/image/heightmap")
async def img_heightmap():
    return _img(config.RESULTS_DIR / "3d_map" / "color_heightmap.jpg")


@app.get("/api/stages")
async def api_stages():
    """Which pipeline-stage outputs currently exist (for the Stages tab)."""
    visuals = []
    if config.VISUALS_DIR.exists():
        visuals = sorted(p.name for p in config.VISUALS_DIR.glob("*.jpg"))
    ortho_preview = config.STITCH_DIR / "orthomosaic_preview.jpg"
    return {
        "ortho": config.ORTHOMOSAIC.exists() or ortho_preview.exists(),
        "rectified": config.RECTIFIED_FIELD.exists(),
        "annotated": config.ANNOTATED_MAP.exists(),
        "heightmap": (config.RESULTS_DIR / "3d_map" / "color_heightmap.jpg").exists(),
        "model": config.find_model_glb() is not None,
        "visuals": visuals,
    }


@app.get("/api/proof/{feature}")
async def img_proof(feature: str):
    name = Path(feature).name
    if not name.lower().endswith((".jpg", ".jpeg", ".png")):
        name += ".jpg"
    return _img(config.PROOF_HD_DIR / name)


@app.get("/api/visual/{feature}")
async def img_visual(feature: str):
    name = Path(feature).name
    if not name.lower().endswith((".jpg", ".jpeg", ".png")):
        name += ".jpg"
    return _img(config.VISUALS_DIR / name)


@app.get("/api/photo/{name}")
async def photo(name: str):
    p = _resolve_photo(name)
    if p is None:
        return JSONResponse({"error": "photo not found", "name": name}, 404)
    return _img(p)


@app.get("/api/photos")
async def api_photos():
    """List photos received from ASCEND (drone_photos/ or newest survey run)."""
    d = config.DRONE_PHOTOS_DIR if config.COORDINATES_CSV.exists() else config.newest_survey_dir()
    names = []
    if d and d.exists():
        names = sorted(p.name for p in d.iterdir()
                       if p.is_file() and p.suffix in config.IMAGE_EXTS)
    return {"dir": str(d) if d else None, "count": len(names), "photos": names}


# Internal ODM / reconstruction working dirs we never want to list.
_SKIP_DIRS = {
    "odm_datasets", "opensfm", "odm_filterpoints", "odm_meshing", "odm_texturing",
    "odm_georeferencing", "odm_dem", "odm_report", "odm_orthophoto", "odm_25dmeshing",
    "entwine_pointcloud", "mve", "depthmaps", "reports", "features", "matches",
}


@app.get("/api/datafiles")
async def api_datafiles():
    """Key result data files (csv/json/xlsx/txt) under results/ — skips ODM internals."""
    exts = {".csv", ".json", ".xlsx", ".xls", ".txt", ".tsv", ".geojson"}
    files = []
    if config.RESULTS_DIR.exists():
        for p in sorted(config.RESULTS_DIR.rglob("*")):
            if not p.is_file() or p.suffix.lower() not in exts:
                continue
            parts = p.relative_to(config.RESULTS_DIR).parts
            if any(seg in _SKIP_DIRS for seg in parts[:-1]):
                continue                       # skip ODM working files
            if len(parts) > 3:
                continue                       # skip deeply-nested junk
            files.append({
                "name": p.name,
                "rel": p.relative_to(config.RESULTS_DIR).as_posix(),
                "ext": p.suffix.lower().lstrip("."),
                "size": p.stat().st_size,
            })
    return {"dir": str(config.RESULTS_DIR), "files": files}


def _safe_result(relpath: str):
    base = config.RESULTS_DIR.resolve()
    target = (base / relpath).resolve()
    if base not in target.parents or not target.exists():
        return None
    return target


@app.get("/api/datafile/{relpath:path}")
async def api_datafile(relpath: str):
    target = _safe_result(relpath)
    if target is None:
        return JSONResponse({"error": "not found"}, 404)
    return FileResponse(str(target), filename=target.name)      # download


@app.get("/api/viewfile/{relpath:path}")
async def api_viewfile(relpath: str):
    target = _safe_result(relpath)
    if target is None:
        return JSONResponse({"error": "not found"}, 404)
    media = {
        ".json": "application/json", ".geojson": "application/json",
        ".csv": "text/plain; charset=utf-8", ".tsv": "text/plain; charset=utf-8",
        ".txt": "text/plain; charset=utf-8",
    }.get(target.suffix.lower(), "application/octet-stream")
    return FileResponse(str(target), media_type=media)          # inline (view in browser)


@app.post("/api/open_results")
async def api_open_results():
    """Open the results/ folder in the OS file explorer (Windows Explorer on WSL)."""
    import shutil
    import subprocess
    import sys
    path = str(config.RESULTS_DIR)
    try:
        if shutil.which("explorer.exe"):                 # WSL → Windows Explorer
            try:
                win = subprocess.check_output(["wslpath", "-w", path]).decode().strip()
            except Exception:
                win = path
            subprocess.Popen(["explorer.exe", win])
        elif sys.platform == "darwin":
            subprocess.Popen(["open", path])
        elif shutil.which("xdg-open"):
            subprocess.Popen(["xdg-open", path])
        else:
            return {"ok": False, "error": "no file-explorer opener found", "path": path}
        emit_log("info", f"opened results folder: {path}")
        return {"ok": True, "path": path}
    except Exception as exc:
        return {"ok": False, "error": str(exc), "path": path}


@app.get("/api/camera/latest")
async def camera_latest():
    # prefer the drone's last-captured photo from telemetry
    tel = links.telemetry()
    last = tel.get("last_photo")
    if last:
        p = _resolve_photo(last)
        if p is not None:
            return _img(p)
    p = _latest_photo()
    if p is None:
        return JSONResponse({"error": "no photos available"}, 404)
    return _img(p)


@app.get("/api/model")
async def model():
    p = config.find_model_glb()
    if p is None:
        return JSONResponse({"error": "no 3D model found"}, 404)
    return FileResponse(str(p), media_type="model/gltf-binary",
                        headers={"Cache-Control": "no-cache"})


# ──────────────────────────────────────────────────────────────────────────
# WebSocket
# ──────────────────────────────────────────────────────────────────────────
@app.websocket("/ws")
async def ws_endpoint(ws: WebSocket):
    await hub.register(ws)
    try:
        # send a hello snapshot
        tel = links.telemetry()
        tel["link_mode"] = links.mode
        await ws.send_text(json.dumps({
            "type": "hello",
            "telemetry": tel,
            "phases": MISSION_PHASES,
            "settings": settings,
            "summary": results_store.read_summary(),
            "pipeline": pipeline.status(),
            "logs": list(hub.log_history),
            "dock_logs": list(hub.dock_history),
        }, default=str))
        while True:
            # we don't expect client messages, but keep the socket alive
            await ws.receive_text()
    except WebSocketDisconnect:
        pass
    except Exception:
        pass
    finally:
        hub.unregister(ws)


# ──────────────────────────────────────────────────────────────────────────
def main():
    import uvicorn
    print(f"Base Station → http://localhost:{config.PORT}   "
          f"(base_dir={config.BASE_DIR})")
    uvicorn.run(app, host=config.HOST, port=config.PORT, log_level="info")


if __name__ == "__main__":
    main()
