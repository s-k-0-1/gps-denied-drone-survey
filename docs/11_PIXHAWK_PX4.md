# 11 — Pixhawk & PX4 Setup

Flight-controller wiring, MAVROS connection, and the PX4 parameters that make GPS-denied flight
work. The full exported parameter file is in `drone/px4_params.params`.

**Hardware:** Pixhawk Cube Orange+ · **Firmware:** PX4 · **Companion:** Jetson Orin Nano

---

## 1. Physical connections

| Device | Connects to | Notes |
|---|---|---|
| Jetson Orin Nano | Pixhawk **USB** (`/dev/ttyACM0` @ 921600) | Carries MAVLink; routed by `mavlink-router` |
| MTF-01 optical flow | Pixhawk serial (`TELEM`/`UART`) | Provides flow + range |
| RealSense D455 | Jetson USB 3 | Not connected to the Pixhawk at all |
| ESCs ×4 | **`AUX OUT 1–4`** (not MAIN) | Quadrotor X layout — see the note below |

> **Why AUX and not MAIN.** On the Cube Orange+ the MAIN outputs are driven by the separate IO
> co-processor, while the **AUX outputs come straight from the FMU**. DShot and the faster/cleaner
> timing paths are only available on the FMU outputs, so the ESCs are wired to AUX. Using AUX also
> keeps one less processor in the signal path.
>
> With ESCs on AUX, the motor assignment in PX4 is set on the **AUX** outputs — check this in
> QGroundControl → **Actuators**, not the MAIN tab, or the motor test will appear to do nothing.

The camera never talks to the flight controller directly — vision reaches PX4 only as
`/mavros/vision_pose/pose` through MAVROS.

---

## 2. MAVLink routing

`mavlink-router` on the Jetson takes the single USB stream and fans it out, so MAVROS, the ground
station and QGroundControl can all be connected at once.

```ini
[General]
TcpServerPort = 5760
MavlinkDialect = common

[UartEndpoint PX4]
Device = /dev/ttyACM0
Baud = 921600

[UdpEndpoint QGC]
Mode = Normal
Address = <ground-pc-ip>
Port = 14550

[UdpEndpoint Script]
Mode = Normal
Address = 127.0.0.1
Port = 14540
```

| Endpoint | Consumer |
|---|---|
| TCP `5760` | Ground-station dashboard |
| UDP `14550` | QGroundControl |
| UDP `14540` | MAVROS / onboard scripts |

Details: [03 — Data Transfer §4](03_DATA_TRANSFER.md#4-telemetry--mavlink-routing).

---

## 3. MAVROS interface

| Topic / service | Direction | Purpose |
|---|---|---|
| `/mavros/vision_pose/pose` | Jetson → PX4 | **The vision input**, published only when `vio_gate` is open |
| `/mavros/local_position/pose` | PX4 → Jetson | Fused EKF pose (used by missions and detectors) |
| `/mavros/setpoint_position/local` | Jetson → PX4 | OFFBOARD position setpoints (20 Hz) |
| `/mavros/rc/in` | PX4 → Jetson | RC channels — CH5 starts the mission / takes over |
| `/mavros/state` | PX4 → Jetson | Armed state, flight mode |
| `/mavros/cmd/arming`, `/mavros/set_mode` | services | Arm and switch to OFFBOARD |

> OFFBOARD requires a **continuous** setpoint stream. All missions publish at `sp_rate_hz` (20 Hz);
> if the stream stops, PX4 drops out of OFFBOARD and applies `COM_OBL_RC_ACT`.

### 3.1 Launching MAVROS

```bash
ros2 launch mavros px4.launch \
  fcu_url:=/dev/ttyACM0:921600 \
  config_yaml:=$HOME/drone_ws/px4_config.yaml
```

Two configuration files control it — both are in `drone/`:

| File | Purpose |
|---|---|
| `px4_config.yaml` | Per-plugin settings (frames, TF, rates, timesync) |
| `px4_pluginlists.yaml` | Which plugins load at all |

### 3.2 `px4_pluginlists.yaml` — plugin selection

```yaml
plugin_denylist:
  - image_pub
  - vibration
  - wheel_odometry
  - fake_gps          # vision goes in through vision_pose, not fake_gps
```

Two decisions worth understanding:

- **`fake_gps` is disabled on purpose.** A common GPS-denied approach is to convert vision into
  fake GPS messages. This stack does not — vision is fed to PX4 as a *vision pose*, which is the
  native path and lets EKF2 weight it with `EKF2_EVP_NOISE` instead of treating it as satellite
  data.
- **`distance_sensor` and `rangefinder` must NOT be denied.** They were on the denylist at one
  point, which silently disabled the rangefinder — and with `EKF2_HGT_REF = 2` (range as height
  reference) that breaks the height estimate. If height behaves strangely, check this list first.

### 3.3 `px4_config.yaml` — the settings that matter

| Section | Setting | Why |
|---|---|---|
| `sys_time` | `timesync_mode: MAVLINK`, `timesync_rate: 10.0` | Keeps the Jetson and Pixhawk clocks aligned. **Vision fusion depends on this** — a drifting clock makes `EKF2_EV_DELAY` meaningless |
| `local_position` | `tf.send: true`, `frame_id: map`, `child_frame_id: base_link` | Publishes the TF tree that RTAB-Map and the detectors rely on |
| `vision_pose` | `tf.listen: false` | The bridge publishes the pose **topic** directly; MAVROS must not try to derive it from TF as well |
| `global_position` | `tf.send: false` | No GPS, so no global TF |
| `imu` | stdev values | IMU noise model published alongside the data |
| `px4flow` | `ranger_min_range: 0.3`, `ranger_max_range: 5.0` | Rangefinder limits — below 0.3 m and above 5 m the reading is not trusted |
| `sys_status` | `conn_timeout: 10.0` | Connection-loss detection |

> `vision_pose.tf.listen: false` together with `local_position.tf.send: true` is the correct
> combination for this architecture: MAVROS *publishes* the fused pose as TF, and *receives*
> vision as a plain topic from `vio_gate`.

---

## 4. Key PX4 parameters

These are what make GPS-denied flight work. Set them in QGroundControl → **Vehicle Setup →
Parameters**, or load `drone/px4_params.params`.

### 4.1 Position sources

| Parameter | Value | Meaning |
|---|---|---|
| `EKF2_GPS_CTRL` | **0** | GPS fusion **completely disabled** |
| `EKF2_GPS_CHECK` | 0 | No GPS pre-arm checks (there is no GPS) |
| `EKF2_OF_CTRL` | **1** | Optical flow fusion **enabled** — the always-on fallback |
| `EKF2_EV_CTRL` | **9** | External vision: horizontal position + yaw |
| `EKF2_HGT_REF` | 2 | Height reference = range finder |
| `EKF2_BARO_CTRL` | 1 | Barometer fused for height |
| `EKF2_IMU_CTRL` | 7 | Full IMU fusion (bias + gyro + accel) |

Because flow and vision are both enabled, the EKF fuses whichever is arriving — which is why
"switching" is done simply by starting/stopping the vision stream (see
[10 — VIO & Localization](10_VIO_LOCALIZATION.md)).

### 4.2 Vision tuning

| Parameter | Value | Meaning |
|---|---|---|
| `EKF2_EV_DELAY` | **80 ms** | Vision measurement latency. **Critical** — a wrong value makes the EKF fight the vision input |
| `EKF2_EVP_NOISE` | 0.5 | Vision position noise (m) |
| `EKF2_EVV_NOISE` | 0.1 | Vision velocity noise |
| `EKF2_EVA_NOISE` | 0.1 | Vision angle noise |
| `EKF2_EVP_GATE` | 5.0 | Position innovation gate |
| `EKF2_EVV_GATE` | 3.0 | Velocity innovation gate |
| `EKF2_EV_POS_X/Y/Z` | 0 | Camera offset from the IMU — set if your camera is not at the centre |

### 4.3 Optical flow tuning

| Parameter | Value | Meaning |
|---|---|---|
| `EKF2_OF_DELAY` | 20 ms | Flow latency |
| `EKF2_OF_GATE` | 3.0 | Innovation gate |
| `EKF2_OF_N_MIN` / `EKF2_OF_N_MAX` | 0.15 / 0.5 | Flow noise range |
| `EKF2_OF_POS_X/Y/Z` | sensor offset | Position of the flow sensor relative to the IMU |

### 4.4 Failsafe

| Parameter | Purpose |
|---|---|
| `COM_OBL_RC_ACT` | What PX4 does if the OFFBOARD link is lost. Set to **AUTO.LAND** for this stack — the drone lands where it is rather than trying to navigate without a companion |
| `COM_RC_IN_MODE` | RC input mode — the pilot must always be able to take over |
| `NAV_RCL_ACT`, `NAV_DLL_ACT` | RC-loss and datalink-loss actions |

---

## 5. Tuning workflow

Get these right **in this order** — a later step will fight you if an earlier one is wrong.

1. **Sensor calibration** — accelerometer, gyro, level horizon, compass.
2. **Optical flow first.** Fly in Position mode on flow alone; it must hold position steadily
   before vision is introduced. If flow is unreliable, nothing built on top of it will work.
3. **`EKF2_EV_DELAY`.** Publish vision and watch the innovation in QGC; the wrong delay shows up as
   the estimate lagging or oscillating.
4. **Rate PIDs** (`MC_ROLLRATE_*`, `MC_PITCHRATE_*`, `MC_YAWRATE_*`) — with props on, in Stabilized
   mode, using PX4's standard tuning procedure.
5. **Position/velocity gains** (`MPC_XY_VEL_*`, `MPC_Z_VEL_*`) — only after the rate loop is clean.
6. **Only then** enable vision and fly the gated mission.

> The rate loop is the foundation. Tuning position gains on top of a poorly tuned rate loop
> produces oscillation that looks like a VIO problem but is not.

---

## 6. Pre-arm checklist

- [ ] Props **off** for every bench test
- [ ] Motor order + rotation verified in QGC **Actuators / Motor Test** (Quadrotor X)
- [ ] `EKF2_GPS_CTRL = 0`, `EKF2_OF_CTRL = 1`, `EKF2_EV_CTRL = 9`
- [ ] `EKF2_EV_DELAY` set for your pipeline
- [ ] `COM_OBL_RC_ACT` = AUTO.LAND
- [ ] RC CH5 mapped and tested (`rc_start_low` / `rc_interrupt_high`)
- [ ] MAVROS connected, `/mavros/local_position/pose` arriving at ≥ 15 Hz
- [ ] Buck converter output verified at 12 V **before** the Jetson is connected
- [ ] Flow sensor reading sensible height and velocity on the bench

---

## 7. Troubleshooting

| Symptom | Likely cause | Fix |
|---|---|---|
| Pre-arm rejected, "height estimate error" | No usable height source | Check flow/rangefinder; verify `EKF2_HGT_REF` |
| Drone drifts in Position mode | Flow unreliable (surface, light, height) | Fix flow before touching vision |
| Altitude overshoots after vision starts | Vision Z offset — seed problem | Confirm `vio_gate` seeding; check `EKF2_EV_DELAY` |
| Position estimate oscillates with vision on | `EKF2_EV_DELAY` wrong | Re-tune the delay |
| Drops out of OFFBOARD | Setpoint stream interrupted | Check the mission node is alive and publishing at 20 Hz |
| Vision pose never reaches PX4 | Gate never opened | `ros2 topic echo /viman/vio_state` — see [10 §2](10_VIO_LOCALIZATION.md) |
| Cannot arm | Preflight gate blocking | Check MAVROS is up and pose rate ≥ `preflight_pose_hz_min` |

---

**Back to:** [09 — Drone Software](09_DRONE_SOFTWARE.md) · [10 — VIO & Localization](10_VIO_LOCALIZATION.md)
