# 09 — Drone Software (Jetson / ROS 2)

Everything that runs **on the drone**: the ROS 2 package `viman_mission`, how the autonomous
missions work, and what each file does.

**Package:** `drone/viman_mission/` (ROS 2 Humble, `ament_python`)
**Runs on:** NVIDIA Jetson Orin Nano
**Talks to:** Pixhawk Cube Orange+ (PX4) over MAVROS, RealSense D455 over USB

---

## 1. The core idea

Flying indoors without GPS, the drone has two independent ways to know where it is:

| Source | Gives | Reliability |
|---|---|---|
| **Optical flow** (MTF-01) | velocity + height | Very robust, but **drifts** in position (it integrates velocity) |
| **Visual odometry** (RTAB-Map on the D455) | absolute position + yaw | Accurate, but **can fail** on low-texture ground, glare or motion blur |

Using VIO blindly is dangerous — if it diverges mid-flight, the drone flies away. So this stack
**never trusts VIO until it has proven itself**:

```
   take off on FLOW only  →  seed VIO on the ground truth  →  validate it for several seconds
        →  open the gate  →  fly the mission on fused VIO  →  close the gate  →  land on FLOW only
```

The component that decides "is VIO trustworthy right now?" is **`vio_gate`**, and its verdict is a
single number — the **initialization factor (IF)**.

---

## 2. Architecture (final: `bringup.launch.py`)

Everything is launched **on the ground**, as parallel processes. Nothing is spawned mid-flight.

```
                    ┌──────────────────────────────────────────────┐
   RealSense D455 ──►  rs_pipeline        (hardware-stamped RGB-D) │
                    └──────────────┬───────────────────────────────┘
                                   │ /camera/camera/color/image_raw
                                   │ /camera/camera/depth/image_rect_raw
                    ┌──────────────▼───────────────────────────────┐
                    │  RTAB-Map stack   (rgbd_odometry + rtabmap)  │
                    └──────────────┬───────────────────────────────┘
                                   │ /rtabmap/rtabmap/odom
                    ┌──────────────▼───────────────────────────────┐
                    │  vio_gate     IF = Q × A × S                 │
                    │  gate CLOSED ──validate──► gate OPEN         │
                    └──────────────┬───────────────────────────────┘
                                   │ /mavros/vision_pose/pose  (only when open)
                    ┌──────────────▼───────────────────────────────┐
                    │  PX4 EKF2  (fuses flow + vision + baro)      │
                    └──────────────▲───────────────────────────────┘
                                   │ setpoints (OFFBOARD)
                    ┌──────────────┴───────────────────────────────┐
                    │  mission node: survey / boundary / hover …   │
                    └──────────────────────────────────────────────┘
      optional: whycode_detector (marker landing) · yellow_boundary_detector (arena limits)
```

**Launch it:**

```bash
# Terminal 1 — MAVROS (as usual for your setup)
# Terminal 2
cd ~/drone_ws && source /opt/ros/humble/setup.bash && source install/setup.bash
ros2 launch viman_mission bringup.launch.py
```

Useful launch arguments:

| Argument | Default | Meaning |
|---|---|---|
| `mission_node` | `mission_director` | Which mission to fly (`survey_mission`, `boundary_test_auto`, …) |
| `start_camera` | `true` | Start `rs_pipeline`. **Never run two camera drivers at once** |
| `start_rtabmap` | `true` | Include the RTAB-Map stack |
| `start_whycode` | `true` | Marker detector for precision landing |
| `start_boundary` | `false` | Yellow-line detector (needed for boundary missions; costs CPU) |

> On the ground the downward camera sees nothing, so RTAB-Map odometry fail-resets in a loop.
> **This is by design** — it is harmless and stops once the mission seeds it at altitude.

---

## 3. Every file, and what it does

### Core infrastructure

| File | Role |
|---|---|
| `rs_pipeline.py` | RealSense D455 driver — hardware-stamped, depth aligned to colour, publishes TF. Handles USB hardware-reset on start and retries. |
| `rtabmap_config.py` | **Single source of truth** for the RTAB-Map launch configuration (robust flight preset + full-resolution storage). |
| `vio_gate.py` | The safety gate. Scores VIO health, seeds/validates it, and only then republishes pose to `/mavros/vision_pose/pose`. Runs 3 watchdogs. |
| `vision_bridge.py` | Legacy direct bridge: RTAB-Map odom → `/mavros/vision_pose/pose` with validation, remap and offset at 30 Hz. |
| `common.py` | Shared QoS profiles, covariance checks, yaw extraction, frame-convention notes. |

### Mission nodes

| File | Mission |
|---|---|
| `mission_director.py` | The reference mission: preflight gate → flow takeoff → seed → validate → hover on VIO → return → flow landing. |
| `survey_mission.py` | **Lawnmower survey** — flies a grid of checkpoints, hovers and captures a photo at each, then returns and lands (optionally on a WhyCode marker). |
| `survey_boundary_director.py` | Survey with live yellow-boundary clamping — can never lead within `boundary_keep_dist_m` of any line. |
| `boundary_test_auto.py` | **Autonomous corner finding** — lawnmower sweep that detects all four arena corners using the yellow lines. |
| `corner_survey_mission.py`, `corner1_test_auto.py` | Corner-approach variants used during development. |
| `precision_land.py` | AprilTag/board-based precision landing. |
| `whycode_mission.py` | Fly to a WhyCode marker, centre, probe forward/back, land on it. |
| `auto_mission.py` | Original standalone state machine (kept as reference). |

### Perception

| File | Role |
|---|---|
| `yellow_boundary_detector.py` | Finds the yellow arena tape in the camera image and publishes a **potential-field repulsion vector**, nearest-line distance, per-line vectors and corner detections. |
| `hsv_calibrate.py` | Interactive HSV tuning helper for the yellow detector. |
| `whycode_detector.py` / `whycon_detector.py` | Circular fiducial marker detection for precision landing. |
| `boundary_guard.py` | **Manual-flight safety net** — a stick filter. Near a line it takes OFFBOARD and clamps only the velocity component *toward* the line; every other direction still responds to the pilot. |

### Launch files

| File | Starts |
|---|---|
| `bringup.launch.py` | **The final architecture** (camera + RTAB-Map + gate + mission) |
| `mission.launch.py` | Legacy: `auto_mission` with the params file |
| `survey_boundary.launch.py` | Survey with boundary clamping |
| `boundary_guard.launch.py` | Manual-flight guard only |
| `boundary_corner.launch.py`, `corner1.launch.py` | Corner missions |
| `hsv_calibrate.launch.py` | Yellow tuning helper |

**All tuning lives in `config/mission_params.yaml`** — one file, per-node sections.

---

## 4. Mission flow (what actually happens in a flight)

Using `survey_mission` as the example:

| Phase | What the drone does | Position source |
|---|---|---|
| **PREFLIGHT** | Verifies MAVROS is alive and pose is arriving at ≥ `preflight_pose_hz_min` Hz. Blocks arming otherwise. | — |
| **WAIT RC** | Waits for CH5 PWM ≤ `rc_start_low` to start. CH5 ≥ `rc_interrupt_high` at any moment = pilot takeover. | — |
| **TAKEOFF** | Climbs to `target_alt` (3 m). | **Flow only** |
| **SETTLE** | Holds still `stable_of_secs` (4 s) so flow stabilises. | Flow |
| **YAW ALIGN** | Slews to the mission heading, then re-locks the grid frame to the drone's actual settled yaw so stripes fly straight. | Flow |
| **SEED** | Resets RTAB-Map odometry and captures the alignment between VIO frame and EKF frame. | Flow |
| **VALIDATE** | Flies a small square (`motion_amp_m`) while `vio_gate` scores IF. Needs `IF ≥ validate_if_min` held for `validate_hold_s`. | Flow |
| **HANDOVER** | Gate opens — vision pose starts feeding EKF2. | Flow **+ VIO** |
| **SURVEY** | Flies the lawnmower grid; at each checkpoint holds `waypoint_settle_s` and captures a photo + pose. | Fused |
| **RETURN** | Flies back to home at `return_speed_ms`. | Fused |
| **GATE OFF** | Camera contribution removed, EKF settles for `flow_settle_s` **before** descending. | Flow only |
| **DESCEND** | Precision descent with X/Y locked, handing over to `AUTO.LAND` below `descend_handoff_alt_m`. | Flow only |

**Landing is deliberately 100 % optical flow.** VIO is most likely to fail near the ground (motion
blur, prop wash, close-range depth), which is exactly when a failure would be unrecoverable.

If VIO faults mid-mission, the drone holds on flow, re-seeds and re-validates — up to
`max_revalidations` times — before giving up and returning home.

---

## 5. Yellow boundary detection (on the drone)

Different from the ground-station yellow detection (which finds the arena corners in the stitched
map). This one runs **live, at 15 Hz**, to keep the drone inside the arena.

```
image → HSV mask (hsv_low … hsv_high, CLAHE on V)
      → optional line filter (area, length, width, aspect ratio, rectangularity)
      → distance + direction of each line in body frame
      → potential field:  weight = (1 − r/R)^falloff_power
      → publishes:
           /viman/boundary/repulsion     push-away vector
           /viman/boundary/nearest_m     distance to nearest line
           /viman/boundary/lines         per-line (dist, nx, ny, strength)
           /viman/boundary/corner        corner detections (two arms at an angle)
```

Consumers:
- **`survey_boundary_director`** — clamps every survey setpoint so the drone can never lead within
  `boundary_keep_dist_m` (0.5 m) of any line.
- **`boundary_test_auto`** — uses the per-line vectors to approach and validate all four corners.
- **`boundary_guard`** — clamps *pilot* stick input in manual flight.

**Tuning:** `hsv_low` / `hsv_high` must be re-tuned per arena and lighting. Use `hsv_calibrate`, or
enable `mjpeg_port: 8080` and view the live mask at `http://<jetson-ip>:8080`.

---

## 6. Automatic data transfer

A systemd service on the Jetson, `landing-transfer.service`, watches for landing and pushes the
survey folder to the ground PC automatically.

```bash
systemctl status landing-transfer.service      # is it running?
journalctl -u landing-transfer.service -f      # live log
```

Survey output is written to `survey_dir` (default `/media/jetson/ROS2_SSD/survey`), which then
becomes the ground station's `drone_photos/` input — see
[03 — Data Transfer](03_DATA_TRANSFER.md).

Manual fallback: `manual_transfer.sh`.

---

## 7. Maps and storage

RTAB-Map writes a timestamped database per flight:

```
/media/jetson/ROS2_SSD/maps/flight_YYYYMMDD_HHMMSS.db
```

Storage is deliberately **max quality** (`Mem/ImagePreDecimation 1`, `NotLinkedNodesKept true`)
while live compute is conservative — the live job is stable odometry, not a pretty map. Build the
best map **offline, after landing**:

```bash
rtabmap-reprocess --Vis/MaxFeatures 2000 --Kp/MaxFeatures 1500 \
  --Rtabmap/DetectionRate 4 /media/jetson/ROS2_SSD/maps/flight_<ts>.db out.db
```

> An SSD is required. The full-resolution database grows quickly and an SD card cannot keep up.

---

## 8. Before the first flight (checklist)

1. **Verify the reset service name:** `ros2 service list | grep -i reset` — set `reset_service` in
   `mission_params.yaml` if it differs.
2. **Bench carry-test** (zero flight risk): launch bringup, hold the drone ~1 m over textured
   ground, call `ros2 service call /viman/seed std_srvs/srv/Trigger`, then carry it 1 m sideways.
   `/viman/init_factor` should stay high — this validates the frame correction.
3. **Set the PX4 parameters** — see [11 — Pixhawk & PX4](11_PIXHAWK_PX4.md), especially
   `EKF2_EV_DELAY` and `COM_OBL_RC_ACT`.
4. **Ground soak:** run the full stack on the pad for 30 minutes; watch RAM and `tegrastats`.
5. **Bench-verify stick signs** if using `boundary_guard` (push pitch forward → `STICK x` positive).

**Watch in flight:**

```bash
ros2 topic echo /viman/init_factor    # VIO health score
ros2 topic echo /viman/vio_state      # 0 unseeded · 1 seeding · 2 validating · 3 OPEN · 4/5/6 faults
```

---

## 9. Build

```bash
cd ~/drone_ws
colcon build --packages-select viman_mission --symlink-install
source install/setup.bash
```

`--symlink-install` means Python edits take effect without rebuilding — re-run `colcon` only when
adding files or changing `setup.py`.

---

**Next:** [10 — VIO & Localization](10_VIO_LOCALIZATION.md) · [11 — Pixhawk & PX4](11_PIXHAWK_PX4.md)
