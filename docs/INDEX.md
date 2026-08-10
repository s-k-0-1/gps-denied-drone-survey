# Index — where to find anything

Look up a topic, a file, a parameter or an error message and go straight to the right place.

> **Tip:** press <kbd>t</kbd> on the GitHub repo page to search filenames, or <kbd>/</kbd> to
> search the code. Inside a long document, use <kbd>Ctrl</kbd>+<kbd>F</kbd>.

---

## A–Z topics

| Topic | Where to read it |
|---|---|
| **A4988 stepper driver** | [Hardware — wiring](01_HARDWARE.md) · [Docking & Charging — motion control](02_DOCKING_CHARGING.md) |
| **Accuracy (~0.1 m)** | [VIO & Localization](10_VIO_LOCALIZATION.md) · [How It Works](../HOW_IT_WORKS.md) |
| **Altitude / height source** | [Pixhawk & PX4 — `EKF2_HGT_REF`](11_PIXHAWK_PX4.md) |
| **Arena size (35 × 25 ft)** | [Stage Guide — Stage 2](07_STAGE_GUIDE.md) · `ARENA_LONG_FT` |
| **AUX OUT (why not MAIN)** | [Pixhawk & PX4 — connections](11_PIXHAWK_PX4.md) · [Hardware — propulsion](01_HARDWARE.md) |
| **Battery / BMS / 2-pin charging** | [Hardware — power distribution](01_HARDWARE.md) |
| **Boundary (yellow) — on the drone** | [Drone Software — boundary detection](09_DRONE_SOFTWARE.md) |
| **Boundary (yellow) — in the map** | [Stage Guide — Stage 2](07_STAGE_GUIDE.md) |
| **`boundary_start_corner` options** | [Operations — command reference](13_OPERATIONS.md) |
| **Buck converter (12 V for Jetson)** | [Hardware — power distribution](01_HARDWARE.md) |
| **Bundle adjustment** | [How It Works — Stage 1](../HOW_IT_WORKS.md) · [Stage Guide — Stage 1](07_STAGE_GUIDE.md) |
| **Camera fails — what happens** | [VIO & Localization — failure behaviour](10_VIO_LOCALIZATION.md) |
| **Camera mounting / orientation** | [Hardware — the built drone](01_HARDWARE.md) · [Drone Software](09_DRONE_SOFTWARE.md) |
| **Charging state machine** | [Docking & Charging](02_DOCKING_CHARGING.md) |
| **CLAHE** | [How It Works — Stage 3](../HOW_IT_WORKS.md) |
| **Contact + polarity detection** | [Docking & Charging](02_DOCKING_CHARGING.md) |
| **Coordinates — how they're computed** | [Stage Guide — Stage 4](07_STAGE_GUIDE.md) · [How It Works](../HOW_IT_WORKS.md) |
| **`coordinates.csv` format** | [Data Transfer](03_DATA_TRANSFER.md) |
| **DDS / CycloneDDS config** | [Operations — first-time setup](13_OPERATIONS.md) |
| **DINOv2 (why, how)** | [How It Works](../HOW_IT_WORKS.md) · [Pipeline — Stage 3](04_PIPELINE.md) |
| **Docking sequence** | [Docking & Charging](02_DOCKING_CHARGING.md) |
| **EKF2 parameters** | [Pixhawk & PX4](11_PIXHAWK_PX4.md) |
| **ESC calibration / DShot** | [Hardware — propulsion](01_HARDWARE.md) |
| **False positives — how avoided** | [How It Works — design decisions](../HOW_IT_WORKS.md) · [VIO — watchdogs](10_VIO_LOCALIZATION.md) |
| **Feature detection (the pipeline)** | [Pipeline](04_PIPELINE.md) · [Stage Guide — Stage 3](07_STAGE_GUIDE.md) |
| **Flight order (first flights)** | [Operations — first flights](13_OPERATIONS.md) |
| **GPS — why none** | [How It Works](../HOW_IT_WORKS.md) · [Pixhawk & PX4](11_PIXHAWK_PX4.md) |
| **HD proof image** | [Stage Guide — Stage 3](07_STAGE_GUIDE.md) |
| **HSV calibration (yellow)** | [Operations — calibration](13_OPERATIONS.md) · [Drone Software](09_DRONE_SOFTWARE.md) |
| **Initialization factor (IF = Q×A×S)** | [VIO & Localization — the trust decision](10_VIO_LOCALIZATION.md) |
| **Landing (why flow-only)** | [VIO & Localization](10_VIO_LOCALIZATION.md) · [Drone Software — mission flow](09_DRONE_SOFTWARE.md) |
| **Landing → transfer → pipeline chain** | [End-to-End Automation](12_END_TO_END_AUTOMATION.md) |
| **LoFTR** | [How It Works](../HOW_IT_WORKS.md) |
| **Loop closure** | [VIO & Localization — RTAB-Map](10_VIO_LOCALIZATION.md) *(with diagram)* |
| **LR image deliverable** | [Stage Guide — Stage 3](07_STAGE_GUIDE.md) |
| **MAVLink routing** | [Data Transfer](03_DATA_TRANSFER.md) · [Pixhawk & PX4](11_PIXHAWK_PX4.md) |
| **MAVROS config / plugins** | [Pixhawk & PX4 — MAVROS interface](11_PIXHAWK_PX4.md) |
| **Mission phases (survey flight)** | [Drone Software — mission flow](09_DRONE_SOFTWARE.md) · [main README](../README.md) |
| **Motor order / rotation** | [Operations — ground tests](13_OPERATIONS.md) · [Hardware — propulsion](01_HARDWARE.md) |
| **Mutual exclusion (two targets, one spot)** | [Stage Guide — Stage 4](07_STAGE_GUIDE.md) |
| **Optical flow ↔ vision switching** | [VIO & Localization — two position sources](10_VIO_LOCALIZATION.md) |
| **Precision landing (WhyCode marker)** | [Drone Software](09_DRONE_SOFTWARE.md) |
| **PX4 parameters (full list)** | [Pixhawk & PX4](11_PIXHAWK_PX4.md) · file `drone/px4_params.params` |
| **RealSense D455 settings** | [VIO & Localization — camera pipeline](10_VIO_LOCALIZATION.md) |
| **RTAB-Map — setup** | [VIO & Localization](10_VIO_LOCALIZATION.md) |
| **RTAB-Map — parameters** | [VIO & Localization](10_VIO_LOCALIZATION.md) · file `drone/viman_mission/viman_mission/rtabmap_config.py` |
| **RTAB-Map — inspecting the map** | [VIO & Localization](10_VIO_LOCALIZATION.md) |
| **Seeding (VIO frame alignment)** | [VIO & Localization](10_VIO_LOCALIZATION.md) |
| **Stitching** | [Stage Guide — Stage 1](07_STAGE_GUIDE.md) · [How It Works](../HOW_IT_WORKS.md) |
| **Survey grid parameters** | [Drone Software](09_DRONE_SOFTWARE.md) · `mission_params.yaml` → `survey_mission:` |
| **Tokens / passwords / credentials** | [main README — security note](../README.md) |
| **VIO gate (states 0–6)** | [VIO & Localization](10_VIO_LOCALIZATION.md) |
| **Voltage measurement (pads)** | [Docking & Charging](02_DOCKING_CHARGING.md) · [Hardware — voltage sensing](01_HARDWARE.md) |
| **Watchdogs (divergence, jump, spike)** | [VIO & Localization](10_VIO_LOCALIZATION.md) |
| **Wiring — every connection** | [Hardware — wiring tables](01_HARDWARE.md) |
| **2D photos → 3D model** | [main README](../README.md) · [Stage Guide — Stage 5](07_STAGE_GUIDE.md) |

---

## "I'm looking at this file — what does it do?"

### Ground pipeline

| File | Role | Explained in |
|---|---|---|
| `iroc_pipeline_fixed.py` | **Main entry point**, all fixes and config flags | [Pipeline](04_PIPELINE.md) |
| `iroc_pipeline.py` | Base pipeline: stitching, field map, annotation | [Pipeline](04_PIPELINE.md) |
| `stage3_robust.py` | Target matcher (DINOv2) | [Pipeline — Stage 3](04_PIPELINE.md) |
| `fused_search.py` | Model loading, image helpers, builds `drone_photos_lr/` | [Pipeline](04_PIPELINE.md) |
| `3d.py` | 3D reconstruction (OpenDroneMap) | [Stage Guide — Stage 5](07_STAGE_GUIDE.md) |
| `make_lr.py` | Full-res references → 64×64 seeds | [Stage Guide](07_STAGE_GUIDE.md) |
| `make_test_dataset.py` | Synthetic arena with known ground truth | [Install — verify](00_INSTALL_EVERYTHING.md) |

### Drone (`drone/viman_mission/viman_mission/`)

| File | Role | Explained in |
|---|---|---|
| `survey_boundary_director.py` | **The mission you fly** — survey + live boundary clamping | [Drone Software](09_DRONE_SOFTWARE.md) |
| `survey_mission.py` | Lawnmower survey with photo capture | [Drone Software](09_DRONE_SOFTWARE.md) |
| `mission_director.py` | Reference mission (hover) | [Drone Software](09_DRONE_SOFTWARE.md) |
| `vio_gate.py` | Scores VIO health, opens/closes the gate | [VIO & Localization](10_VIO_LOCALIZATION.md) |
| `vision_bridge.py` | RTAB-Map odom → `/mavros/vision_pose/pose` | [Drone Software](09_DRONE_SOFTWARE.md) |
| `rs_pipeline.py` | RealSense driver (hardware-stamped, aligned) | [VIO & Localization](10_VIO_LOCALIZATION.md) |
| `rtabmap_config.py` | All RTAB-Map tuning, in one place | [VIO & Localization](10_VIO_LOCALIZATION.md) |
| `yellow_boundary_detector.py` | Yellow tape → repulsion field | [Drone Software](09_DRONE_SOFTWARE.md) |
| `boundary_guard.py` | Stick clamp for manual flight | [Drone Software](09_DRONE_SOFTWARE.md) |
| `whycode_detector.py` | Marker detection for precision landing | [Drone Software](09_DRONE_SOFTWARE.md) |
| `boundary_test_auto.py` | Autonomous 4-corner finding | [Drone Software](09_DRONE_SOFTWARE.md) |
| `common.py` | Shared QoS, covariance checks, frame conventions | [Drone Software](09_DRONE_SOFTWARE.md) |

### Configuration files

| File | Contains | Explained in |
|---|---|---|
| `drone/viman_mission/config/mission_params.yaml` | **All mission tuning** (every node) | [Drone Software](09_DRONE_SOFTWARE.md) · [Operations](13_OPERATIONS.md) |
| `drone/px4_params.params` | PX4 parameters (EKF2, failsafe, PIDs) | [Pixhawk & PX4](11_PIXHAWK_PX4.md) |
| `drone/px4_config.yaml` | MAVROS plugin settings | [Pixhawk & PX4](11_PIXHAWK_PX4.md) |
| `drone/px4_pluginlists.yaml` | Which MAVROS plugins load | [Pixhawk & PX4](11_PIXHAWK_PX4.md) |
| `drone/main.conf` | mavlink-router endpoints | [Data Transfer](03_DATA_TRANSFER.md) |
| `drone/cyclonedds.xml` | ROS 2 DDS settings | [Operations](13_OPERATIONS.md) |
| `drone/landing-transfer.service` | Auto-transfer systemd unit | [End-to-End Automation](12_END_TO_END_AUTOMATION.md) |
| `esp32_firmware/*.ino` | Docking + charging firmware | [Docking & Charging](02_DOCKING_CHARGING.md) |

### Dashboard (`base_station/`)

| File | Role | Explained in |
|---|---|---|
| `server.py` | REST + WebSocket API, image serving | [Pipeline](04_PIPELINE.md) |
| `config.py` | Paths, environment settings, result-set switching | [Pipeline](04_PIPELINE.md) |
| `pipeline_runner.py` | Runs the pipeline, streams logs to the browser | [Pipeline](04_PIPELINE.md) |
| `drone_link/mavlink_link.py` | Live MAVLink telemetry | [Pipeline](04_PIPELINE.md) |

---

## Parameter lookup

| I want to change… | Parameter | In which file |
|---|---|---|
| Arena dimensions | `ARENA_LONG_FT`, `ARENA_SHORT_FT` | `iroc_pipeline_fixed.py` |
| Yellow mask (map side) | `YELLOW_S` | `iroc_pipeline_fixed.py` |
| Yellow mask (drone side) | `hsv_low`, `hsv_high` | `mission_params.yaml` |
| Detection strictness | `MIN_FOUND_PEAK`, `VERIFY_MIN` | `stage3_robust.py` |
| Stitching robustness | `GRID_RADIUS`, `MIN_INLIERS` | `iroc_pipeline.py` |
| Coordinate origin | `BASE_STATION_EXACT`, `HEADING_ROT_DEG` | `iroc_pipeline_fixed.py` |
| Survey grid / speed / altitude | `survey_*`, `target_alt` | `mission_params.yaml` |
| VIO gate strictness | `validate_if_min`, `cov_spike`, `divergence_max_m` | `mission_params.yaml` |
| RTAB-Map quality vs speed | `Vis/MaxFeatures`, `GFTT/MinDistance` | `rtabmap_config.py` |
| Boundary standoff distance | `boundary_keep_dist_m`, `stop_dist_m` | `mission_params.yaml` |
| Charging thresholds | `HIGH_THRESHOLD`, `DIVIDER_RATIO` | `esp32_firmware/*.ino` |

Full explanations → [Parameters Guide](../PARAMETERS_GUIDE.md) · [Stage Guide](07_STAGE_GUIDE.md)

---

## Error / log message lookup

| You see | What it means | Go to |
|---|---|---|
| `Stitched 15/35 photos` | Photos dropped during stitching | [Troubleshooting — stitching](08_TROUBLESHOOTING.md) |
| `[fix#13] spatial pairing…` | VIO-based pairing is active — good | [Stage Guide — Stage 1](07_STAGE_GUIDE.md) |
| `[fix#3] TRUE size …` | Metric scale applied — good | [Stage Guide — Stage 2](07_STAGE_GUIDE.md) |
| `[fix#1 base-origin] …` | Base-station origin applied — good | [Stage Guide — Stage 4](07_STAGE_GUIDE.md) |
| `peak=… V=…` | Per-target match scores | [Stage Guide — Stage 3](07_STAGE_GUIDE.md) |
| `NOT FOUND` | Target rejected by the thresholds | [Troubleshooting — matching](08_TROUBLESHOOTING.md) |
| `Odometry lost!` | RTAB-Map tracking failed | [VIO & Localization](10_VIO_LOCALIZATION.md) |
| `vio_state: 4 / 5 / 6` | A gate watchdog tripped | [VIO & Localization](10_VIO_LOCALIZATION.md) |
| `no heartbeat within Ns` | MAVLink not reaching the dashboard | [Troubleshooting — MAVLink](08_TROUBLESHOOTING.md) |
| `ParameterNotDeclaredException` | RTAB-Map parameter in the wrong channel | [VIO & Localization](10_VIO_LOCALIZATION.md) |
| `Waiting for drone to land...` | Auto-transfer service is healthy | [End-to-End Automation](12_END_TO_END_AUTOMATION.md) |
| `externally-managed-environment` | pip needs `--break-system-packages` | [Troubleshooting — install](08_TROUBLESHOOTING.md) |

---

**Back to:** [Documentation home](README.md) · [Repository README](../README.md)
