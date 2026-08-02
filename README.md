# ASCEND — Autonomous Survey & Feature Localization
### Team LUMA · IRoC-U 2026 (ISRO Robotics Challenge)

Ground-station software for an autonomous survey drone: it stitches the drone's aerial
photos into one map, finds the required target features in that map using a semantic
(deep-learning) matcher, and reports each target's real-world coordinates — **without GPS**.
It also includes a live web dashboard and the ESP32 firmware for the auto-docking /
battery-charging base station.

> **Scope note:** this repository covers **everything except the flight controller / autonomy
> code** (how the drone actually flies its mission). That part is maintained separately by
> another team member.

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
- **Live dashboard** (`base_station/`) — telemetry, camera feed, arena map, targets, logs, pipeline control.
- **Docking + charging** (`esp32_firmware/`) — ESP32 drives the docking rods, detects contact
  and polarity, measures pack voltage, and runs the charger.

---

## Results (final field round)

- All required features detected and localized.
- Coordinates reported relative to the base station, in metres, **GPS-free** (Pixhawk VIO / optical flow).
- Deliverables produced automatically per target: LR image, HD image, coordinates.

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
git clone https://github.com/<your-username>/ascend_iroc_2026_team_LUMA.git
cd ascend_iroc_2026_team_LUMA

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
| [How It Works (theory)](HOW_IT_WORKS_EN.md) | Deep theory of every algorithm — written for the viva |
| [Parameters Guide](PARAMETERS_GUIDE_EN.md) | Every tunable parameter, when to change it and why |
| [Run Guide](RUN_GUIDE.md) | Run + validation checklist |
| [64×64 Mode](PARAMETERS_64x64.md) | Tuning for the alternate 64×64 LR-to-LR matching mode |
| [Declarations](DECLARATIONS_11.6.md) | Competition declarations: coordinate scheme, no-GPS, survey pattern, processing location |

*(Hinglish versions of the theory and parameter guides are also included:
`HOW_IT_WORKS.md`, `PARAMETERS_GUIDE.md`.)*

---

## Repository structure

```
ascend_iroc_2026_team_LUMA/
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

- **No GPS/GNSS.** Position comes from the camera + Pixhawk VIO / optical flow, as required by the rulebook.
- **Semantic matching, not template matching.** DINOv2 embeddings match a feature even under
  different rotation, lighting and scale — where SIFT/template methods fail on low-texture ground.
- **LR-to-LR.** The drone's HD photos are down-sampled to 128×128 and compared against the
  64×64 seed, following the rulebook's low-resolution matching workflow.
- **Coordinates relative to the base station**, computed in the rectified (straightened) arena
  frame so they stay accurate despite VIO drift.

---

## Team

**Team LUMA** — The LNM Institute of Information Technology, Jaipur
IRoC-U 2026, ISRO Robotics Challenge — Unmanned Aerial Vehicle

## License

MIT — see [LICENSE](LICENSE).
