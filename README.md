# ASCEND — Autonomous Survey & Feature Localization
### GPS-denied aerial mapping and object localization

**A drone flies over an arena and takes photos. This software tells you exactly where the
objects of interest are — in metres, without GPS.**

Give it the aerial photos and a small reference image of what to look for, and it will:

1. **Stitch** the overlapping photos into a single top-view map of the arena.
2. **Straighten** that map and scale it to real-world metres using the arena boundary.
3. **Find** each reference object in the survey using a deep-learning (semantic) matcher — one
   that still recognises the object under different rotation, lighting and scale.
4. **Report** each object's coordinates relative to the base station, with a low-resolution and a
   high-resolution proof image.

Position comes from the drone's camera and Pixhawk visual-inertial odometry, so the whole system
works **without GPS** — a hard requirement of the challenge, and a necessity anyway since GPS is
accurate to metres while this task needs ~0.1 m.

The repository also contains a **live web dashboard** (telemetry, maps, detected targets, logs,
one-click pipeline runs) and the **ESP32 firmware** for the auto-docking and battery-charging base
station.

---

<img src="docs/images/architecture.svg" width="900">

---

## What this system does

| # | Stage | What happens | Main file |
|---|---|---|---|
| 1 | **Stitching** | Overlapping HD photos → one top-view orthomosaic (LoFTR + bundle adjustment) | `iroc_pipeline.py` |
| 2 | **Field map** | Detect the yellow arena boundary → rectify to a straight, true-scale rectangle | `iroc_pipeline.py` |
| 3 | **Target matching** | Find each 64×64 seed image inside the drone's 128×128 LR photos (DINOv2 semantic) | `stage3_robust.py` |
| 4 | **Coordinates** | Target pixel → mosaic → rectified → metres, relative to the base station | `iroc_pipeline_fixed.py` |
| 5 | **3D map** *(optional)* | Photogrammetry → textured 3D model + elevation map | `3d.py` |

Plus:
- **On-drone autonomy** (`drone/`) — ROS 2 stack on the Jetson: optical-flow takeoff, **gated
  handover to RTAB-Map visual odometry**, autonomous lawnmower survey, yellow-boundary safety,
  marker precision landing.
- **Live dashboard** (`base_station/`) — telemetry, camera feed, arena map, targets, logs, pipeline control.
- **Docking + charging** (`esp32_firmware/`) — ESP32 drives the docking rods, detects contact
  and polarity, measures pack voltage, and runs the charger.

---

## Results

Validated on real flight data over a 35 × 25 ft arena:

- All target features detected and localized, with no false positives.
- Coordinates reported relative to the base station, in metres, **GPS-free** (VIO / optical flow).
- Accuracy ≈ **0.1 m** on distinct features.
- Per target, the pipeline automatically produces a low-resolution image, a high-resolution
  proof image and the coordinates.

### What the pipeline produces

| **Stage 1** — stitched orthomosaic | **Stage 2** — detected boundary → rectified field |
|:---:|:---:|
| <img src="docs/images/orthomosaic.jpg" width="420"> | <img src="docs/images/yellow_corners_debug.jpg" width="420"> |
| ~35 overlapping photos joined into one map | The 4 yellow corners found on the mosaic |

| **Stage 2** — rectified, true-scale arena | **Stage 4** — annotated result |
|:---:|:---:|
| <img src="docs/images/rectified_field.jpg" width="420"> | <img src="docs/images/annotated_field.jpg" width="420"> |
| Straightened and scaled to the real arena size | Each target circled with its coordinates |

**Stage 3 — target matching, shown live in the dashboard**
*(left: the 64×64 seed · right: where it was found in the survey, circled)*

<img src="docs/images/3.png" width="880">

More debug views: [`docs/images/`](docs/images/) · Expected output format: [`examples/`](examples/)

---

## Quick start

```bash
# 1. clone
git clone https://github.com/s-k-0-1/gps-denied-drone-survey.git
cd gps-denied-drone-survey

# 2. install (Python 3.10+)
pip install -r requirements.txt --break-system-packages

# 3. put your data in place
#    drone_photos/   -> HD survey photos + coordinates.csv
#    targets/        -> seed images (64x64)

# 4. run the full pipeline
python3 iroc_pipeline_fixed.py
```

Outputs appear in `results/` — see [docs/05_SETUP.md](docs/05_SETUP.md) for the detailed walkthrough.

Live dashboard:
```bash
pip install -r base_station/requirements.txt --break-system-packages
python3 -m base_station.server        # open http://localhost:8000
```

---

## Documentation

Read these in order — they are written for someone who has never seen this project before.

| Doc | What's inside |
|---|---|
| **[00 — Install Everything](docs/00_INSTALL_EVERYTHING.md)** | Every download/install from zero: WSL2, Python, PyTorch, Docker, Arduino IDE, QGroundControl, mavlink-router |
| **[01 — Hardware](docs/01_HARDWARE.md)** | Drone + base-station hardware structure, component list, full wiring tables, how to use each part |
| **[02 — Docking & Charging](docs/02_DOCKING_CHARGING.md)** | ESP32 firmware: how docking starts, contact & polarity detection, voltage measurement, the charging state machine, dashboard integration |
| **[03 — Data Transfer](docs/03_DATA_TRANSFER.md)** | How photos and telemetry get from the drone → Jetson → ground PC; MAVLink routing; file layout |
| **[04 — Pipeline / Feature Detection](docs/04_PIPELINE.md)** | Stage-by-stage explanation of how detection works, and what **every file** in the repo does |
| **[05 — Setup](docs/05_SETUP.md)** | Step-by-step installation on a fresh computer (Windows/WSL, Linux, Jetson) |
| **[06 — Git & GitHub](docs/06_GIT_GITHUB.md)** | Complete beginner's guide: install git, create the repo, push, update, clone |
| **[07 — Stage Guide](docs/07_STAGE_GUIDE.md)** | Practical per-stage reference: what runs, which file, the few parameters that matter, how to fix each stage |
| **[08 — Troubleshooting](docs/08_TROUBLESHOOTING.md)** | Every known symptom → fix, in one lookup table (install, stitching, matching, dashboard, MAVLink, ESP32, git) |
| **[09 — Drone Software](docs/09_DRONE_SOFTWARE.md)** | The ROS 2 stack on the Jetson: architecture, every node, the autonomous missions, yellow-boundary safety, data transfer |
| **[10 — VIO & Localization](docs/10_VIO_LOCALIZATION.md)** | How optical flow and RTAB-Map visual odometry are fused, the gated handover, and what happens when the camera fails |
| **[11 — Pixhawk & PX4](docs/11_PIXHAWK_PX4.md)** | Flight-controller wiring, MAVROS interface, the PX4 parameters for GPS-denied flight, tuning order |
| **[12 — End-to-End Automation](docs/12_END_TO_END_AUTOMATION.md)** | What happens automatically after touchdown: docking, data transfer, and the pipeline starting itself |
| **[13 — Operations Runbook](docs/13_OPERATIONS.md)** | Every command: new-Jetson setup, flight-day sequence, mission variants, calibration, build/deploy, checklists |
| [How It Works](HOW_IT_WORKS.md) | Full explanation of every stage and algorithm, plus the design decisions behind them |
| [Parameters Guide](PARAMETERS_GUIDE.md) | Every tunable parameter, when to change it and why |
| [Run Guide](RUN_GUIDE.md) | Run + validation checklist |
| [64×64 Mode](PARAMETERS_64x64.md) | Tuning for the alternate 64×64 matching mode |

---

## Repository structure

```
gps-denied-drone-survey/
├── iroc_pipeline_fixed.py     ← MAIN entry point (all fixes applied)
├── iroc_pipeline.py           ← base pipeline: stitching, field map, annotation
├── stage3_robust.py           ← target matcher (DINOv2 semantic, LR-to-LR)
├── fused_search.py            ← shared model loading + image helpers
├── 3d.py                      ← optional 3D reconstruction (OpenDroneMap)
├── make_lr.py                 ← build LR seed images from full-res references
├── make_test_dataset.py       ← synthetic dataset generator (for testing)
│
├── base_station/              ← live web dashboard (FastAPI + WebSocket)
│   ├── server.py              ← REST + WebSocket API, image serving
│   ├── config.py              ← all paths / environment settings
│   ├── pipeline_runner.py     ← runs the pipeline as a subprocess, streams logs
│   ├── results_store.py       ← reads results/, auto-refresh watcher
│   ├── drone_link/            ← MAVLink link + simulator (hot-swappable)
│   └── static/                ← dashboard UI (HTML/CSS/JS)
│
├── drone/                     ← ON-DRONE software (Jetson, ROS 2)
│   ├── viman_mission/         ← main ROS 2 package
│   │   ├── viman_mission/     ← nodes: mission_director, survey_mission, vio_gate,
│   │   │                         rs_pipeline, yellow_boundary_detector, boundary_guard,
│   │   │                         whycode_detector, precision_land, vision_bridge …
│   │   ├── launch/            ← bringup / survey_boundary / boundary_guard / hsv_calibrate
│   │   └── config/            ← mission_params.yaml  (ALL tuning lives here)
│   ├── whycode-ros2/          ← fiducial marker detection (C++) for precision landing
│   ├── whycode_interfaces/    ← marker message + service definitions
│   ├── scripts/               ← landing_transfer_node.py — auto data transfer on touchdown
│   ├── legacy/                ← earlier standalone scripts, kept for reference
│   ├── px4_params.params      ← exported PX4 parameters (EKF2, failsafe, PIDs)
│   ├── px4_config.yaml        ← MAVROS plugin configuration
│   ├── px4_pluginlists.yaml   ← which MAVROS plugins load
│   ├── main.conf              ← mavlink-router routing
│   ├── cyclonedds.xml         ← ROS 2 DDS configuration
│   └── landing-transfer.service ← systemd unit for the auto transfer
│
├── esp32_firmware/            ← docking + charging firmware (Arduino)
│   └── full_base_station_wifi.ino
│
├── docs/                      ← all documentation (start here)
│   └── images/                ← result screenshots + architecture diagram
├── examples/                  ← what a successful run produces (sample targets.json)
│
├── drone_photos/              ← INPUT: HD survey photos + coordinates.csv  (not in git)
├── targets/                   ← INPUT: seed images                          (not in git)
└── results/                   ← OUTPUT: mosaic, field map, targets, 3D      (not in git)
```

Input photos and result folders are **not** committed to git (they are large) — see
[docs/06_GIT_GITHUB.md](docs/06_GIT_GITHUB.md).

---

## Key design decisions

- **No GPS/GNSS.** Position comes from the camera + Pixhawk VIO / optical flow. GPS is accurate to
  metres, while this task needs ~0.1 m — and it is unavailable indoors anyway.
- **Semantic matching, not template matching.** DINOv2 embeddings match a feature even under
  different rotation, lighting and scale — where SIFT/template methods fail on low-texture ground.
- **Low-resolution matching.** The HD photos are down-sampled to 128×128 and matched against a
  64×64 reference. Matching in feature space means the two sizes do not have to agree, and small
  images keep the search fast.
- **Coordinates relative to the base station**, computed in the rectified (straightened) arena
  frame so they stay accurate despite VIO drift.

---

## Security note

The repository ships with **placeholders and defaults, not real credentials**. Before deploying,
set your own:

| What | Where | Default |
|---|---|---|
| WiFi SSID / password | `esp32_firmware/full_base_station_wifi.ino` | `YOUR_WIFI_SSID` / `YOUR_WIFI_PASSWORD` |
| Ground-PC address | same file, `BASE_URL` | `http://192.168.1.100:8000` |
| Machine token | ESP `DOCK_TOKEN` **and** `IROC_TOKEN` on the dashboard (must match) | `CHANGE_ME` |
| Dashboard login | env vars `IROC_USER` / `IROC_PASS` | `luma` / `ascend2026` |

```bash
IROC_USER=yourname IROC_PASS='a-strong-password' IROC_TOKEN='your-token' \
    python3 -m base_station.server
```

Never commit real WiFi passwords, tokens or private IP addresses.

---

## Credits

Built by **Team LUMA**, The LNM Institute of Information Technology, Jaipur — originally developed
for the ISRO Robotics Challenge (IRoC-U 2026).

## License

MIT — see [LICENSE](LICENSE).
