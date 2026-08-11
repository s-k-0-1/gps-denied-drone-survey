# Drone Base Station Dashboard

A real-time desktop dashboard to command the drone and monitor the
survey + matching pipeline live. It's a **FastAPI web app**: the server
runs in WSL alongside your CUDA/Docker pipeline, and you open the dashboard in
your Windows browser. No X-server needed.

It **uses** the existing scripts (`iroc_pipeline_fixed.py` — the main pipeline —
plus `stage3_robust.py` and `3d.py`); it does not reimplement them. It watches
`results/` and updates the panels as files appear.

> Full project documentation is in [`../docs/`](../docs/). This file only covers
> the dashboard itself.

---

## Quick start (simulator — no drone needed)

```bash
# 1. from ~/advanced_matcher  (the pipeline root, where iroc_pipeline.py lives)
cd ~/advanced_matcher

# 2. install the (light) dashboard deps
pip install -r base_station/requirements.txt --break-system-packages

# 3. run it (simulator is the default link — no hardware needed)
python3 -m base_station.server
#    …or just:  bash base_station/run.sh
```

Open **http://localhost:8000** (from Windows: same URL usually works with WSL2;
if not, use the WSL IP from `hostname -I`, e.g. `http://172.x.x.x:8000`).

Click **START MISSION** → the simulator flies an autonomous sortie by replaying
your newest `survey/<run>/coordinates.csv`: Takeoff → Survey (capturing each
photo) → Returning → Landed → Data Transfer → Charging → Done, with live
battery, position, the camera feed, and the arena marker all moving.

> The simulator auto-detects flight data: it prefers `drone_photos/coordinates.csv`,
> otherwise the most recent `survey/<run>/` folder (which you already have), so
> the demo works out of the box even before `results/` exists.

---

## Running the real drone (MAVLink — default mode)

```bash
# default mode tries MAVLink first, falls back to simulator if no heartbeat
MAVLINK_CONN="udpin:0.0.0.0:14550" python3 -m base_station.server
```

`MAVLINK_CONN` examples: `udpin:0.0.0.0:14550` (listen — works with PX4/ArduPilot
SITL and most telemetry routers), `udp:127.0.0.1:14550`, `tcp:127.0.0.1:5760`,
`/dev/ttyUSB0,57600` (serial radio). You can also switch **Link** live from the
top-right dropdown.

START MISSION over MAVLink sends: arm → `NAV_TAKEOFF` (to `CRUISE_ALT_M`) →
set `AUTO` → `MISSION_START` (flies the survey mission you've uploaded to the
FC). Return/Land sends `RTL`. Camera capture sends `IMAGE_START_CAPTURE`.

---

## View from anywhere (public URL + password)

The dashboard is protected by a password (HTTP Basic). Default login:

| user | password |
|---|---|
| `luma` | `ascend2026` |

Change it with env vars before starting the server (recommended):

```bash
IROC_USER=luma IROC_PASS='your-strong-password' python3 -m base_station.server
```

To open it from **any network, anywhere**, run a Cloudflare quick tunnel in a
second WSL terminal (no account needed):

```bash
# 1. get cloudflared once
wget -q https://github.com/cloudflare/cloudflared/releases/latest/download/cloudflared-linux-amd64 -O cloudflared
chmod +x cloudflared

# 2. with the base station already running on :8000, expose it
./cloudflared tunnel --url http://localhost:8000
```

It prints a public `https://<random>.trycloudflare.com` URL. Open that from your
phone or any computer → the browser asks for the username/password → you're in.
Because the tunnel is HTTPS, your password is sent encrypted.

> Quick-tunnel URLs change every run. For a permanent address, create a free
> Cloudflare account and a *named* tunnel (`cloudflared tunnel create luma`),
> or use `ngrok http 8000` (free account + `ngrok config add-authtoken …`).

Disable the password entirely (e.g. on a trusted LAN) with `IROC_AUTH=0`.

---

## The two mission modes (toggle in the UI)

The **"Run pipeline on mission"** switch (top-right) decides what START MISSION
does with your pipeline:

* **ON** → commands the drone **and** launches the selected pipeline job
  (`iroc_pipeline.py …`) as a subprocess; its stdout streams into the Logs
  console and panels auto-refresh as `results/` updates.
* **OFF** → display-only: the app just commands the drone and watches
  `results/` while you run `iroc_pipeline.py` yourself in a terminal.

The job dropdown maps to the CLI: `full` = `iroc_pipeline_fixed.py`,
`skip_stitch`/`skip_match` = the matching flags, `match_only` = just the robust
matcher, `match 64×64 (LR)` = `MATCH_LR=64 … --skip-stitch` (result copied to
`results_lr64/`), `full_3d` = `--run-3d`, `reconstruct3d` = `3d.py`.

The **View** dropdown switches which result set the panels display —
`Default (128)` reads `results/`, `64×64` reads `results_lr64/`.

---

## Panels

1. **Mission Control** — START MISSION, the 8-phase state pipeline, manual
   buttons (Takeoff/Survey/Return/Transfer/Charge/Capture/+Sortie), mission
   timer, sortie counter, link status, link selector, pipeline toggle.
2. **Live Telemetry** — battery, altitude, x/y/z (relative to base), yaw,
   speed, mode, photo count, data-transfer progress.
3. **Live Camera** — latest captured photo (updates as the survey captures).
4. **Arena Map (2D)** — `stage4_annotated/annotated_field.jpg` with a live
   drone marker + trail; auto-refreshes when the pipeline rewrites it.
5. **3D Map** — `model.glb` via `<model-viewer>` (drag to rotate / zoom).
6. **Targets** — table (name, x, y, z, confidence) with HD-proof thumbnails;
   click a row → full-res proof with title `x=.. y=.. z=.. m`. "Found N / M".
7. **Logs** — live pipeline + drone event console.

---

## Architecture (modular — easy to extend)

```
base_station/
├── server.py            FastAPI app: REST + WebSocket + image serving
├── config.py            all paths/env knobs (auto-derives from ~/advanced_matcher)
├── pipeline_runner.py   runs iroc_pipeline.py / fused_search.py / 3d.py, streams logs
├── results_store.py     reads targets.json + fused_results.csv, watchdog auto-refresh
├── drone_link/
│   ├── base.py          DroneLink ABC + Telemetry + MissionState
│   ├── simulator.py     SimulatorLink — replays coordinates.csv as a flight
│   ├── mavlink_link.py  MavlinkLink — pymavlink (PX4/ArduPilot)
│   └── __init__.py      LinkManager — hot-swap links, sim fallback
├── static/              index.html · style.css · app.js (dark dashboard)
├── requirements.txt · run.sh · README.md
```

**Plugging in your own custom radio:** subclass `DroneLink` (implement
`_tick`, `start_mission`, `takeoff`, `survey`, `return_land`,
`start_data_transfer`, `start_charging`, `capture_photo`), then add it to the
factory in `drone_link/__init__.py`. The server and UI need no changes.

---

## Config / environment variables

| Var | Default | Meaning |
|---|---|---|
| `IROC_BASE_DIR` | parent of `base_station/` | pipeline root (`results/`, scripts) |
| `IROC_TEAM` | `LUMA` | team name shown in the header |
| `DRONE_LINK_MODE` | `simulator` | `mavlink` \| `simulator` \| `auto` (switch live in UI) |
| `MAVLINK_CONN` | `udpin:0.0.0.0:14550` | MAVLink endpoint |
| `BASE_STATION_PORT` | `8000` | HTTP port |
| `RUN_PIPELINE_ON_MISSION` | `1` | default state of the pipeline toggle |
| `CRUISE_ALT_M` | `3.0` | takeoff altitude |
| `IROC_PYTHON` | `python3` | interpreter used for the pipeline subprocess |

---

## Notes / troubleshooting

* **`model-viewer` / fonts** load from a CDN — fine on WSL2 with internet. For a
  fully offline demo, download `model-viewer.min.js` into `static/` and point
  the `<script>` tag at it.
* **No auto-refresh?** Install `watchdog` (in requirements). Without it, panels
  still refresh on WebSocket mission events but not on external file writes.
* **MAVLink shows "no heartbeat"** → it silently falls back to the simulator so
  the UI still works; check `MAVLINK_CONN` and that your FC/SITL is streaming.
* The app reads files only; it never edits your `results/`. Heavy GPU work
  happens in your pipeline subprocess exactly as before.
