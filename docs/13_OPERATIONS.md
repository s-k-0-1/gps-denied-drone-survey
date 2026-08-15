# 13 — Operations Runbook (every command, with a test after every step)

The complete command reference. **Every step ends with a test — do not move on until it passes.**
Most steps are done with the **propellers removed**; the doc says clearly when props go on.

> Open this file when you have not touched the system for months, or when a teammate is setting it
> up for the first time.

---

## Contents

| Part | What | Props |
|---|---|---|
| **[1 — First-time setup](#part-1--first-time-setup-props-off)** | Install everything on a new Jetson | OFF |
| **[2 — Bench tests](#part-2--bench-tests-props-off-drone-on-the-table)** | Prove each subsystem alone: camera, yellow, WhyCode, VIO carry-test | OFF |
| **[3 — Ground tests](#part-3--ground-tests-with-the-flight-controller-props-still-off)** | MAVLink, motor order, vision reaching PX4, automation | OFF |
| **[4 — First flights](#part-4--first-flights-props-on-in-this-order)** | 6 flights in increasing risk order | **ON** |
| **[5 — Flight day](#part-5--normal-flight-day-sequence)** | The normal 3-terminal sequence | ON |
| **[6 — Command reference](#part-6--command-reference-what-every-command-does)** | Every command and what it does | — |
| **[7 — Edit → build → deploy](#part-7--edit--build--deploy)** | Changing parameters and code | — |
| **[8 — Network changes](#part-8--when-the-network-changes)** | New WiFi / new IPs | — |
| **[Pre-flight checklist](#pre-flight-checklist-print-this)** | Print this | — |

Each step has a **✅ TEST** box. If it fails, fix it there — a later step will not work anyway.

> 🔎 Looking for one specific command or parameter? → **[INDEX](INDEX.md)**

---

# PART 1 — First-time setup (props OFF)

## 1.1 System packages

```bash
sudo apt update
sudo apt install -y python3-pip git tmux nano rsync
source /opt/ros/humble/setup.bash
```

> **✅ TEST**
> ```bash
> ros2 --version          # must print a version
> ```

## 1.2 ROS 2 packages

```bash
sudo apt install -y \
  ros-humble-mavros ros-humble-mavros-extras \
  ros-humble-rtabmap-ros ros-humble-rtabmap-launch \
  ros-humble-cv-bridge ros-humble-image-transport \
  ros-humble-tf2-ros ros-humble-vision-msgs

sudo /opt/ros/humble/lib/mavros/install_geographiclib_datasets.sh
```

> **✅ TEST**
> ```bash
> ros2 pkg list | grep -E "mavros|rtabmap"     # both must appear
> ```

## 1.3 RealSense driver

```bash
sudo apt install -y librealsense2-utils librealsense2-dev python3-pyrealsense2
```

> **✅ TEST** — plug the D455 into a **USB 3 (blue)** port:
> ```bash
> rs-enumerate-devices | head -20      # must list the D455
> realsense-viewer                     # colour + depth streams must appear
> ```
> No device? Try a different USB 3 port and cable. USB 2 cannot carry 720p30 RGB-D.

## 1.4 mavlink-router

```bash
git clone https://github.com/mavlink-router/mavlink-router.git
cd mavlink-router && git submodule update --init --recursive
meson setup build . && ninja -C build && sudo ninja -C build install

sudo mkdir -p /etc/mavlink-router
sudo cp <repo>/drone/main.conf /etc/mavlink-router/main.conf
sudo nano /etc/mavlink-router/main.conf     # set [UdpEndpoint QGC] Address = your PC's IP
```

> **✅ TEST** — with the Pixhawk plugged in over USB:
> ```bash
> ls /dev/ttyACM0                                       # device must exist
> mavlink-routerd -c /etc/mavlink-router/main.conf
> ```
> Expected:
> ```
> Opened UART [4]PX4: /dev/ttyACM0
> UART [4]PX4: speed = 921600
> Opened TCP Server [9] [::]:5760
> ```

## 1.5 Build the workspace

```bash
mkdir -p ~/drone_ws/src && cd ~/drone_ws/src
cp -r <repo>/drone/viman_mission      .
cp -r <repo>/drone/whycode-ros2       .
cp -r <repo>/drone/whycode_interfaces .

cd ~/drone_ws
colcon build --symlink-install
source install/setup.bash
```

> **✅ TEST**
> ```bash
> ros2 pkg list | grep viman_mission            # package is found
> ros2 pkg executables viman_mission            # ~19 executables listed
> ```

## 1.6 Environment

```bash
cp <repo>/drone/cyclonedds.xml  ~/
cp <repo>/drone/px4_config.yaml <repo>/drone/px4_pluginlists.yaml ~/drone_ws/

cat >> ~/.bashrc <<'EOF'
source /opt/ros/humble/setup.bash
source ~/drone_ws/install/setup.bash
export ROS_DOMAIN_ID=0
export CYCLONEDDS_URI=file:///home/jetson/cyclonedds.xml
EOF
```

> **✅ TEST** — open a **new** terminal:
> ```bash
> ros2 pkg list | grep viman_mission     # works without sourcing anything manually
> ```

## 1.7 Storage

```bash
mkdir -p /media/jetson/ROS2_SSD/maps /media/jetson/ROS2_SSD/survey
```

> **✅ TEST**
> ```bash
> df -h /media/jetson/ROS2_SSD          # SSD mounted, plenty of free space
> touch /media/jetson/ROS2_SSD/maps/.w && rm /media/jetson/ROS2_SSD/maps/.w   # writable
> ```

## 1.8 Auto-transfer service

```bash
mkdir -p ~/scripts && cp <repo>/drone/scripts/* ~/scripts/ && chmod +x ~/scripts/*.sh

# passwordless SSH to the ground PC (otherwise the rsync hangs forever)
ssh-keygen -t ed25519
ssh-copy-id -p <pc-ssh-port> <pc-user>@<pc-ip>
```

Configure it — **prefer environment variables** over editing the file:

```bash
sudo tee /etc/systemd/system/landing-transfer.service >/dev/null <<'EOF'
[Unit]
Description=Auto transfer drone data on landing
After=network.target

[Service]
ExecStart=/home/jetson/scripts/start_landing_transfer.sh
Restart=always
RestartSec=5
User=jetson
Environment="ROS_DOMAIN_ID=0"
Environment="PC_USER=your_pc_user"
Environment="PC_IP=192.168.1.100"
Environment="PC_PORT=22"
Environment="PC_DEST_PATH=/home/your_pc_user/gps-denied-drone-survey/drone_photos/"
Environment="DOCK_TOKEN=your-shared-token"

[Install]
WantedBy=multi-user.target
EOF

sudo systemctl daemon-reload
sudo systemctl enable --now landing-transfer.service
```

> **✅ TEST**
> ```bash
> ssh -p <port> <pc-user>@<pc-ip> "echo ok"     # must print ok WITHOUT a password
> systemctl status landing-transfer.service      # active (running)
> journalctl -u landing-transfer.service -n 20   # "Waiting for drone to land..."
> ```

## 1.9 PX4 parameters

Load `drone/px4_params.params` in QGroundControl → **Vehicle Setup → Parameters → Tools → Load
from file**, then reboot the flight controller.

> **✅ TEST** — in QGC's parameter search, confirm:
> ```
> EKF2_GPS_CTRL = 0      EKF2_OF_CTRL = 1      EKF2_EV_CTRL = 9
> EKF2_HGT_REF  = 2      EKF2_EV_DELAY = 80    COM_OBL_RC_ACT = AUTO.LAND
> ```

---

# PART 2 — Bench tests (props OFF, drone on the table)

Prove each subsystem **alone** before combining them.

## 2.1 Camera node

```bash
ros2 run viman_mission rs_pipeline --ros-args \
  --params-file ~/drone_ws/src/viman_mission/config/mission_params.yaml
```

> **✅ TEST** — second terminal:
> ```bash
> ros2 topic hz /camera/camera/color/image_raw        # ~30 Hz
> ros2 topic hz /camera/camera/depth/image_rect_raw   # ~30 Hz
> ros2 topic echo /camera/camera/color/camera_info --once
> ```
> Rate much below 30 Hz → USB 2 port, or another camera driver is already running.

## 2.2 Yellow boundary detector

Point the camera at the yellow tape from ~1–2 m.

```bash
ros2 run viman_mission yellow_boundary_detector --ros-args \
  --params-file ~/drone_ws/src/viman_mission/config/mission_params.yaml
```

> **✅ TEST**
> ```bash
> ros2 topic echo /viman/boundary/nearest_m        # a sensible distance appears
> ros2 topic echo /viman/boundary/coverage_pct     # non-zero when tape is in view
> ```
> Nothing detected → the HSV range needs calibrating (§2.3).

## 2.3 HSV calibration (do this for every new arena / lighting)

```bash
ros2 launch viman_mission hsv_calibrate.launch.py
```

With the field-tested line-gate overrides:

```bash
ros2 launch viman_mission hsv_calibrate.launch.py \
    line_gate_max_width:=120 line_gate_aspect:=3.0 line_gate_min_len_frac:=0.20 \
    prior_v_min:=25 prior_s_min:=20 min_line_keep_pct:=90.0 edge_exclude_px:=21
```

Live view while tuning — set `mjpeg_port: 8080` in the YAML, then open `http://<jetson-ip>:8080`.

Copy the resulting values into the `yellow_boundary_detector:` block of `mission_params.yaml`
(`hsv_low`, `hsv_high`), then rebuild (§7).

> **✅ TEST** — re-run §2.2. Only the tape should be detected: walk a person in coloured clothing
> through the frame and confirm it is **not** picked up.

## 2.4 WhyCode marker detector

Place the marker under the camera.

```bash
ros2 run viman_mission whycode_detector --ros-args \
  --params-file ~/drone_ws/src/viman_mission/config/mission_params.yaml
```

> **✅ TEST**
> ```bash
> ros2 topic echo /whycode_node/markers        # marker pose appears; empty when covered
> ```

## 2.5 RTAB-Map odometry (hand-carried)

```bash
ros2 launch viman_mission bringup.launch.py
```

Hold the drone ~1 m above **textured** ground.

> **✅ TEST**
> ```bash
> ros2 topic hz /rtabmap/rtabmap/odom       # ~15–30 Hz
> ros2 service call /viman/seed std_srvs/srv/Trigger
> ros2 topic echo /viman/init_factor        # should be high (> 0.7)
> ```
> Now **carry the drone 1 m sideways** — `init_factor` must stay high and
> `/viman/vio_state` should reach **3 (OPEN)**. This validates the whole VIO chain with zero
> flight risk. Odometry below 15 Hz → see the tuning table in
> [10 — VIO](10_VIO_LOCALIZATION.md#39-tuning-checklist).

---

# PART 3 — Ground tests with the flight controller (props still OFF)

## 3.1 MAVLink chain

```bash
# Terminal 1
mavlink-routerd -c /etc/mavlink-router/main.conf

# Terminal 2
ros2 launch mavros px4.launch \
  fcu_url:=/dev/ttyACM0:921600 \
  config_yaml:=$HOME/drone_ws/px4_config.yaml
```

> **✅ TEST**
> ```bash
> ros2 topic echo /mavros/state --once            # connected: true
> ros2 topic hz /mavros/local_position/pose       # ≥ 15 Hz
> ros2 topic hz /mavros/imu/data                  # steady
> ros2 topic echo /mavros/rc/in --once            # move CH5 — the value must change
> ```
> QGroundControl should connect at the same time over UDP 14550 (both work together).

## 3.2 Motor order and direction — **props still OFF**

QGroundControl → **Vehicle Setup → Actuators → Motor Test**.

⚠️ ESCs are on **AUX OUT 1–4**, so use the **AUX** tab — the MAIN tab will appear to do nothing.

> **✅ TEST** — spin each motor one at a time and confirm the PX4 **Quadrotor X** layout:
>
> | Output | Position | Direction |
> |---|---|---|
> | AUX 1 | front-right | CCW |
> | AUX 2 | front-left | CW |
> | AUX 3 | rear-left | CCW |
> | AUX 4 | rear-right | CW |
>
> Wrong position → re-order the ESC signal wires. Wrong direction → swap **any two** of the three
> motor phase wires.

## 3.3 Vision reaching PX4

With MAVROS running and the bringup stack up (§2.5), seed and validate as before.

> **✅ TEST**
> ```bash
> ros2 topic echo /viman/vio_state          # 3 = gate OPEN
> ros2 topic hz /mavros/vision_pose/pose    # ~30 Hz ONLY while the gate is open
> ```
> In QGC, the local position estimate should now track when you carry the drone.

## 3.4 Automation endpoints

```bash
# from the Jetson
curl -i -X POST -H "X-Auth-Token: <TOKEN>" "http://<pc-ip>:8000/api/landed"
curl -i -X POST -H "X-Auth-Token: <TOKEN>" "http://<pc-ip>:8000/api/transfer_done"
```

> **✅ TEST** — both return **200**; the dashboard log shows them, docking starts on the first and
> the pipeline starts on the second.

---

# PART 4 — First flights (props ON, in this order)

Fit the propellers only now. Each flight adds one new capability.

| # | Flight | Command | What you are testing | Pass condition |
|---|---|---|---|---|
| 1 | **Manual hover** | (Position mode, no ROS mission) | Optical flow alone holds position | Stable hover, no drift, for 60 s |
| 2 | **Boundary guard** | `ros2 launch viman_mission boundary_guard.launch.py` | Stick clamping near the tape | Push toward the line — the drone stops ~0.5 m short; every other direction still responds |
| 3 | **Hover mission** | `ros2 launch viman_mission bringup.launch.py` | Takeoff → seed → validate → gate OPEN → hover → flow landing | `vio_state` reaches 3, hovers, returns, lands |
| 4 | **Square** | `ros2 launch viman_mission bringup.launch.py mission_node:=square_mission` | Flying on fused VIO | Square is square; no divergence faults |
| 5 | **Corner finding** | `ros2 launch viman_mission bringup.launch.py mission_node:=boundary_test_auto start_boundary:=true` | Boundary approach and corner logic | Finds 4/4 corners, never crosses a line |
| 6 | **Full survey** | `ros2 launch viman_mission survey_boundary.launch.py boundary_start_corner:=back_left` | The complete mission | Full grid flown, photos captured, returns, lands |

> **Do not skip ahead.** If flight 1 does not hold position on optical flow alone, nothing built on
> top of it will work — fix the flow sensor first.

**Every flight:** pull **RC CH5 low (≤ 1200 µs)** to start; **CH5 high (≥ 1700 µs) takes over
instantly** at any moment.

---

# PART 5 — Normal flight-day sequence

Three terminals on the Jetson, in this order.

```bash
# ── Terminal 1 — MAVLink router ──
mavlink-routerd -c /etc/mavlink-router/main.conf

# ── Terminal 2 — MAVROS ──
ros2 launch mavros px4.launch \
  fcu_url:=/dev/ttyACM0:921600 \
  config_yaml:=$HOME/drone_ws/px4_config.yaml

# ── Terminal 3 — mission ──
cd ~/drone_ws && source install/setup.bash
ros2 launch viman_mission survey_boundary.launch.py boundary_start_corner:=back_left
```

**Before arming, check:**

```bash
ros2 topic hz /mavros/local_position/pose      # ≥ 15 Hz
ros2 topic hz /camera/camera/color/image_raw   # ~30 Hz
ros2 topic echo /viman/boundary/nearest_m      # tape detected
```

**Then:** place the drone at the chosen corner **facing the survey direction** → arm → CH5 low.

**Watch during flight:**

```bash
ros2 topic echo /viman/vio_state       # 0 unseeded · 1 seeding · 2 validating · 3 OPEN · 4/5/6 fault
ros2 topic echo /viman/init_factor     # VIO health
```

---

# PART 6 — Command reference (what every command does)

### Core services

| Command | What it does |
|---|---|
| `mavlink-routerd -c /etc/mavlink-router/main.conf` | Routes the Pixhawk USB stream to TCP 5760 (dashboard), UDP 14550 (QGC), UDP 14540 (onboard) |
| `ros2 launch mavros px4.launch fcu_url:=/dev/ttyACM0:921600 config_yaml:=$HOME/drone_ws/px4_config.yaml` | Starts MAVROS with the plugin config — provides `/mavros/*` topics and accepts `vision_pose` |

### Missions

| Command | What it does |
|---|---|
| `ros2 launch viman_mission survey_boundary.launch.py` | **Main mission** — camera + RTAB-Map + gate + WhyCode + boundary detector + bounded survey |
| `… boundary_start_corner:=back_left` | Drone starts at the back-left corner → steps **right**, ends at the right line *(default)* |
| `… boundary_start_corner:=back_right` | Starts back-right → steps **left**, ends at the left line |
| `… boundary_start_corner:=center` | Starts in the middle → flies back, then left to find the back-left corner, re-yaws, then surveys as `back_left` |
| `… boundary_start_corner:=auto` | Works the corner out from what the camera sees |
| `ros2 launch viman_mission bringup.launch.py` | Reference stack — hover mission (`mission_director`) |
| `… mission_node:=square_mission` | 1 m square on fused VIO |
| `… mission_node:=boundary_test_auto start_boundary:=true` | Autonomous 4-corner finding |
| `ros2 launch viman_mission boundary_guard.launch.py` | **Manual flight** with the stick-clamp safety net |

### Launch overrides (all missions)

| Argument | Effect |
|---|---|
| `start_camera:=false` | You are running a camera driver yourself — **never run two** |
| `start_rtabmap:=false` | RTAB-Map started separately |
| `start_whycode:=false` | No marker landing |
| `start_yellow_boundary:=false` | No boundary detector (survey falls back to fixed extents) |
| `params_file:=/path/to.yaml` | Use a different tuning file |

### Single nodes (bench)

```bash
# built copy of the params (what a launch file uses)
PARAMS=~/drone_ws/install/viman_mission/share/viman_mission/config/mission_params.yaml
# or the source copy while tuning
PARAMS=~/drone_ws/src/viman_mission/config/mission_params.yaml

ros2 run viman_mission rs_pipeline              --ros-args --params-file $PARAMS   # camera
ros2 run viman_mission whycode_detector         --ros-args --params-file $PARAMS   # markers
ros2 run viman_mission yellow_boundary_detector --ros-args --params-file $PARAMS   # yellow tape
```

### Calibration

```bash
ros2 launch viman_mission hsv_calibrate.launch.py

ros2 launch viman_mission hsv_calibrate.launch.py \
    line_gate_max_width:=120 line_gate_aspect:=3.0 line_gate_min_len_frac:=0.20 \
    prior_v_min:=25 prior_s_min:=20 min_line_keep_pct:=90.0 edge_exclude_px:=21
```

| Override | Meaning |
|---|---|
| `line_gate_max_width` | Strokes thicker than this are blobs, not tape |
| `line_gate_aspect` | Minimum length:width — the strongest tape-vs-patch discriminator |
| `line_gate_min_len_frac` | Line must span this fraction of the frame |
| `prior_v_min`, `prior_s_min` | Lower bounds on brightness / saturation |
| `min_line_keep_pct` | How much of the detected line to keep |
| `edge_exclude_px` | Ignore this border margin (frame-edge artefacts) |

### After the flight

```bash
~/manual_transfer.sh                              # manual fallback for the auto transfer
ls -lh /media/jetson/ROS2_SSD/maps/               # flight databases
rtabmap-databaseViewer /media/jetson/ROS2_SSD/maps/flight_<ts>.db   # inspect graph + closures

rtabmap-reprocess --Vis/MaxFeatures 2000 --Kp/MaxFeatures 1500 \
  --Rtabmap/DetectionRate 4 flight_<ts>.db out.db                   # best map, offline
```

### Diagnostics

```bash
ros2 topic list | grep -E "viman|mavros|rtabmap|camera"
ros2 topic hz /camera/camera/color/image_raw     # ~30 Hz
ros2 topic hz /rtabmap/rtabmap/odom              # 15-30 Hz
ros2 topic hz /mavros/local_position/pose        # ≥ 15 Hz
ros2 topic hz /mavros/vision_pose/pose           # only when the gate is OPEN
ros2 topic echo /viman/vio_state
ros2 topic echo /viman/init_factor
ros2 topic echo /viman/boundary/nearest_m
ros2 service list | grep -i reset                # confirm reset_service in the YAML
tegrastats                                        # CPU / RAM / thermals
systemctl status landing-transfer.service
journalctl -u landing-transfer.service -f
```

---

# PART 7 — Edit → build → deploy

### Editing on the Jetson

```bash
nano ~/drone_ws/src/viman_mission/config/mission_params.yaml

cd ~/drone_ws && colcon build --packages-select viman_mission && source install/setup.bash
```

A single-value change without opening the editor:

```bash
sed -i 's/preflight_pose_hz_min: 15.0/preflight_pose_hz_min: 5.0/' \
    ~/drone_ws/src/viman_mission/config/mission_params.yaml
cd ~/drone_ws && colcon build --packages-select viman_mission && source install/setup.bash
```

> With `--symlink-install`, **Python edits need no rebuild**. YAML and launch files **are**
> installed, so those still require `colcon build`.

> **✅ TEST**
> ```bash
> grep preflight_pose_hz_min \
>   ~/drone_ws/install/viman_mission/share/viman_mission/config/mission_params.yaml
> ```
> The built copy must show your new value — if not, the build did not run.

### Deploying from a PC

```bash
# Windows
scp -r D:\drone\drone_ws\src\viman_mission\* jetson@<jetson-ip>:~/drone_ws/src/viman_mission/

# Linux / WSL
rsync -avh ~/drone_ws/src/viman_mission/ jetson@<jetson-ip>:~/drone_ws/src/viman_mission/

# then, on the Jetson
cd ~/drone_ws && colcon build --packages-select viman_mission && source install/setup.bash
```

### Clean rebuild (when something is inexplicably stale)

```bash
cd ~/drone_ws
rm -rf build install log
colcon build --symlink-install
source install/setup.bash
```

---

# PART 8 — When the network changes

Three places hold the ground-PC address:

| Where | What |
|---|---|
| `/etc/mavlink-router/main.conf` (Jetson) | `[UdpEndpoint QGC] Address` |
| `landing-transfer.service` (Jetson) | `PC_IP` / `BASE_URL` environment variables |
| `esp32_firmware/*.ino` (ESP32) | `BASE_URL` — needs a re-flash |

```bash
./change_ip.sh 192.168.1.100          # updates the ESP32 source + Jetson script
```

On the ground PC:

```bash
DRONE_LINK_MODE=mavlink MAVLINK_CONN="tcp:<jetson-ip>:5760" python3 -m base_station.server
```

> **✅ TEST**
> ```bash
> ping -c2 <jetson-ip>
> nc -vz <jetson-ip> 5760              # "succeeded"
> curl -i -X POST -H "X-Auth-Token: <TOKEN>" "http://<pc-ip>:8000/api/landed"   # 200
> ```

---

# Pre-flight checklist (print this)

- [ ] **Props off** for every bench and ground test
- [ ] SSD mounted at `/media/jetson/ROS2_SSD`, writable
- [ ] `mavlink-routerd` running, `/dev/ttyACM0` present
- [ ] MAVROS up, `/mavros/local_position/pose` ≥ 15 Hz
- [ ] Camera ~30 Hz on both colour and depth
- [ ] Yellow HSV calibrated **for today's lighting**
- [ ] VIO carry-test passed (`vio_state` reaches 3)
- [ ] Motor order + direction verified on the **AUX** tab
- [ ] `boundary_start_corner` matches where the drone actually stands
- [ ] Drone facing the intended survey direction **before arming**
- [ ] RC CH5 tested — low starts, high takes over
- [ ] Battery charged; buck output verified at 12 V
- [ ] `landing-transfer.service` active
- [ ] Ground PC reachable, dashboard open

---

**See also:** [09 — Drone Software](09_DRONE_SOFTWARE.md) · [10 — VIO](10_VIO_LOCALIZATION.md) · [11 — Pixhawk](11_PIXHAWK_PX4.md) · [12 — Automation](12_END_TO_END_AUTOMATION.md)
