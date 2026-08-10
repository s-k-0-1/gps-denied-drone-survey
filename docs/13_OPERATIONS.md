# 13 — Operations Runbook (every command, in order)

The complete command reference: setting up a new Jetson from scratch, the daily flight sequence,
every mission variant, calibration, and the edit → build → deploy loop.

> This is the file to open when you have not touched the system for months, or when a teammate is
> setting it up for the first time.

---

## Part A — First-time setup on a new Jetson

### A1. System dependencies

```bash
sudo apt update
sudo apt install -y python3-pip git tmux nano rsync

# ROS 2 Humble must already be installed. Verify:
source /opt/ros/humble/setup.bash
ros2 --version
```

### A2. ROS 2 packages

```bash
sudo apt install -y \
  ros-humble-mavros ros-humble-mavros-extras \
  ros-humble-rtabmap-ros ros-humble-rtabmap-launch \
  ros-humble-cv-bridge ros-humble-image-transport \
  ros-humble-tf2-ros ros-humble-vision-msgs

# MAVROS needs the GeographicLib datasets once:
sudo /opt/ros/humble/lib/mavros/install_geographiclib_datasets.sh
```

### A3. RealSense

```bash
sudo apt install -y librealsense2-utils librealsense2-dev python3-pyrealsense2
realsense-viewer          # confirm the D455 is detected before going further
```

### A4. mavlink-router

```bash
git clone https://github.com/mavlink-router/mavlink-router.git
cd mavlink-router && git submodule update --init --recursive
meson setup build . && ninja -C build && sudo ninja -C build install

sudo mkdir -p /etc/mavlink-router
sudo cp <repo>/drone/main.conf /etc/mavlink-router/main.conf
sudo nano /etc/mavlink-router/main.conf     # set the QGC Address to your PC's IP
```

### A5. The workspace

```bash
mkdir -p ~/drone_ws/src && cd ~/drone_ws/src

# from this repository:
cp -r <repo>/drone/viman_mission      .
cp -r <repo>/drone/whycode-ros2       .
cp -r <repo>/drone/whycode_interfaces .

cd ~/drone_ws
colcon build --symlink-install
source install/setup.bash
```

Add to `~/.bashrc` so every new terminal is ready:

```bash
echo 'source /opt/ros/humble/setup.bash' >> ~/.bashrc
echo 'source ~/drone_ws/install/setup.bash' >> ~/.bashrc
echo 'export ROS_DOMAIN_ID=0' >> ~/.bashrc
echo 'export CYCLONEDDS_URI=file:///home/jetson/cyclonedds.xml' >> ~/.bashrc
```

### A6. Supporting files

```bash
cp <repo>/drone/cyclonedds.xml   ~/
cp <repo>/drone/px4_config.yaml  ~/drone_ws/
mkdir -p ~/scripts && cp <repo>/drone/scripts/* ~/scripts/
chmod +x ~/scripts/*.sh
```

### A7. SSD for maps and surveys

```bash
# the SSD must mount at /media/jetson/ROS2_SSD
mkdir -p /media/jetson/ROS2_SSD/maps /media/jetson/ROS2_SSD/survey
```

> An SSD is mandatory. Full-resolution RTAB-Map databases and survey imagery will overwhelm an
> SD card.

### A8. Auto-transfer service

```bash
nano ~/scripts/landing_transfer_node.py     # set PC_USER / PC_IP / PC_PORT / BASE_URL / DOCK_TOKEN

# passwordless SSH to the ground PC (otherwise rsync hangs)
ssh-keygen -t ed25519
ssh-copy-id -p <pc-ssh-port> <pc-user>@<pc-ip>

sudo cp <repo>/drone/landing-transfer.service /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable --now landing-transfer.service
systemctl status landing-transfer.service
```

### A9. PX4 parameters

Load `drone/px4_params.params` in QGroundControl
(**Vehicle Setup → Parameters → Tools → Load from file**), then verify the critical ones —
see [11 — Pixhawk & PX4](11_PIXHAWK_PX4.md).

---

## Part B — Flight day sequence

Three terminals on the Jetson, in this order.

### Terminal 1 — MAVLink router

```bash
mavlink-routerd -c /etc/mavlink-router/main.conf
```

Expect: `Opened UART [4]PX4: /dev/ttyACM0`, `Opened TCP Server [9] [::]:5760`.

### Terminal 2 — MAVROS

```bash
ros2 launch mavros px4.launch \
  fcu_url:=/dev/ttyACM0:921600 \
  config_yaml:=$HOME/drone_ws/px4_config.yaml
```

Verify before continuing:

```bash
ros2 topic hz /mavros/local_position/pose     # must be ≥ 15 Hz
ros2 topic echo /mavros/state                 # connected: true
```

### Terminal 3 — the mission

```bash
cd ~/drone_ws && source install/setup.bash
ros2 launch viman_mission survey_boundary.launch.py boundary_start_corner:=back_left
```

### Then

1. Place the drone at the chosen start corner, **facing the survey direction** (the mission locks
   onto the arm-time heading).
2. Arm. The preflight gate blocks arming until MAVROS and pose are healthy.
3. Pull **RC CH5 low** (≤ 1200 µs) to start the mission.
4. Watch:
   ```bash
   ros2 topic echo /viman/vio_state       # 3 = gate OPEN
   ros2 topic echo /viman/init_factor     # VIO health
   ```
5. **CH5 high (≥ 1700 µs) at any moment = pilot takes over immediately.**

---

## Part C — Mission variants

### Bounded survey (the main mission)

```bash
ros2 launch viman_mission survey_boundary.launch.py boundary_start_corner:=<option>
```

| `boundary_start_corner` | Where you place the drone | Behaviour |
|---|---|---|
| `back_left` *(default)* | Back-left corner | Steps **right**, ends at the right line |
| `back_right` | Back-right corner | Steps **left**, ends at the left line |
| `center` | Arena middle | Flies **back**, then **left** to find the back-left corner, re-yaws, then surveys as `back_left` |
| `auto` | Anywhere | Works it out from what the camera sees |

Useful overrides:

```bash
start_camera:=false          # you are running a camera driver yourself
start_rtabmap:=false         # RTAB-Map started separately
start_whycode:=false         # no marker landing
start_yellow_boundary:=false # no boundary detector (survey falls back to fixed extents)
params_file:=/path/to.yaml   # different tuning file
```

### Other missions

```bash
# hover / reference mission
ros2 launch viman_mission bringup.launch.py

# a specific mission node through bringup
ros2 launch viman_mission bringup.launch.py mission_node:=survey_mission
ros2 launch viman_mission bringup.launch.py mission_node:=boundary_test_auto start_boundary:=true

# manual flight with the boundary stick-guard
ros2 launch viman_mission boundary_guard.launch.py
```

### Single nodes (bench testing, no flight)

```bash
PARAMS=~/drone_ws/install/viman_mission/share/viman_mission/config/mission_params.yaml

ros2 run viman_mission rs_pipeline              --ros-args --params-file $PARAMS
ros2 run viman_mission whycode_detector         --ros-args --params-file $PARAMS
ros2 run viman_mission yellow_boundary_detector --ros-args --params-file $PARAMS
```

> `install/.../config/mission_params.yaml` is the **built** copy. With `--symlink-install` it
> points at your source file, so editing the source is enough.

---

## Part D — Calibration

### Yellow boundary HSV

```bash
ros2 launch viman_mission hsv_calibrate.launch.py
```

With line-gate overrides (the values that worked in the field):

```bash
ros2 launch viman_mission hsv_calibrate.launch.py \
    line_gate_max_width:=120 line_gate_aspect:=3.0 line_gate_min_len_frac:=0.20 \
    prior_v_min:=25 prior_s_min:=20 min_line_keep_pct:=90.0 edge_exclude_px:=21
```

Then copy the resulting `hsv_low` / `hsv_high` into the `yellow_boundary_detector:` block of
`mission_params.yaml`.

**Re-calibrate for every new arena and lighting condition.** This is the single most
environment-sensitive part of the system.

Live view while tuning — set `mjpeg_port: 8080` in the YAML, then open
`http://<jetson-ip>:8080` from any browser on the network.

### VIO bench carry-test (no flight risk)

```bash
ros2 launch viman_mission bringup.launch.py
# hold the drone ~1 m above textured ground
ros2 service call /viman/seed std_srvs/srv/Trigger
# carry it 1 m sideways — /viman/init_factor should stay high
ros2 topic echo /viman/init_factor
```

---

## Part E — Edit → build → deploy

### Editing on the Jetson

```bash
nano ~/drone_ws/src/viman_mission/config/mission_params.yaml

cd ~/drone_ws && colcon build --packages-select viman_mission && source install/setup.bash
```

A quick single-value change:

```bash
sed -i 's/preflight_pose_hz_min: 15.0/preflight_pose_hz_min: 5.0/' \
    ~/drone_ws/src/viman_mission/config/mission_params.yaml
```

> With `--symlink-install`, **Python edits need no rebuild**. Rebuild only when you add a file,
> change `setup.py`, or edit a YAML/launch file (those are installed, not symlinked).

### Deploying from a PC

```bash
# Windows
scp -r D:\drone\drone_ws\src\viman_mission\* jetson@<jetson-ip>:~/drone_ws/src/viman_mission/

# Linux / WSL
rsync -avh ~/drone_ws/src/viman_mission/ jetson@<jetson-ip>:~/drone_ws/src/viman_mission/

# then on the Jetson
cd ~/drone_ws && colcon build --packages-select viman_mission && source install/setup.bash
```

---

## Part F — Verification & diagnostics

```bash
# topics and rates
ros2 topic list | grep -E "viman|mavros|rtabmap|camera"
ros2 topic hz /camera/camera/color/image_raw     # ~30 Hz
ros2 topic hz /rtabmap/rtabmap/odom              # ~15-30 Hz
ros2 topic hz /mavros/local_position/pose        # ≥ 15 Hz
ros2 topic hz /mavros/vision_pose/pose           # only when the gate is OPEN

# VIO state
ros2 topic echo /viman/vio_state                 # 0 unseeded 1 seeding 2 validating 3 OPEN 4/5/6 fault
ros2 topic echo /viman/init_factor

# boundary detector
ros2 topic echo /viman/boundary/nearest_m
ros2 topic echo /viman/boundary/coverage_pct

# services
ros2 service list | grep -i reset                # confirm reset_service in the YAML

# system load
tegrastats

# auto-transfer service
systemctl status landing-transfer.service
journalctl -u landing-transfer.service -f
```

---

## Part G — After the flight

```bash
# the transfer is automatic; manual fallback:
~/manual_transfer.sh

# maps are on the SSD
ls -lh /media/jetson/ROS2_SSD/maps/

# build the best map offline (never during flight)
rtabmap-reprocess --Vis/MaxFeatures 2000 --Kp/MaxFeatures 1500 \
  --Rtabmap/DetectionRate 4 \
  /media/jetson/ROS2_SSD/maps/flight_<ts>.db out.db
```

On the ground PC the pipeline starts by itself once the transfer completes
([12 — End-to-End Automation](12_END_TO_END_AUTOMATION.md)). To run it manually:

```bash
cd ~/gps-denied-drone-survey && python3 iroc_pipeline_fixed.py
```

---

## Part H — When the network changes (new WiFi / new IPs)

Three places hold the ground-PC address:

| Where | What |
|---|---|
| `/etc/mavlink-router/main.conf` (Jetson) | `[UdpEndpoint QGC] Address` |
| `~/scripts/landing_transfer_node.py` (Jetson) | `PC_IP` and `BASE_URL` |
| `esp32_firmware/*.ino` (ESP32) | `BASE_URL` — needs a re-flash |

`change_ip.sh` updates the last two in one command:

```bash
./change_ip.sh 192.168.1.100
```

Also update the dashboard's MAVLink target on the PC:

```bash
DRONE_LINK_MODE=mavlink MAVLINK_CONN="tcp:<jetson-ip>:5760" python3 -m base_station.server
```

---

## Part I — Pre-flight checklist (print this)

- [ ] Props off for all bench tests
- [ ] SSD mounted at `/media/jetson/ROS2_SSD`
- [ ] `mavlink-routerd` running
- [ ] MAVROS up, `/mavros/local_position/pose` ≥ 15 Hz
- [ ] Camera detected, `rs_pipeline` publishing ~30 Hz
- [ ] Yellow HSV calibrated **for today's lighting**
- [ ] `boundary_start_corner` matches where the drone is actually placed
- [ ] Drone facing the intended survey direction **before arming**
- [ ] RC CH5 tested (low = start, high = takeover)
- [ ] Battery charged, buck output verified at 12 V
- [ ] `landing-transfer.service` active
- [ ] Ground PC reachable, dashboard open

---

**See also:** [09 — Drone Software](09_DRONE_SOFTWARE.md) · [10 — VIO](10_VIO_LOCALIZATION.md) · [11 — Pixhawk](11_PIXHAWK_PX4.md) · [12 — Automation](12_END_TO_END_AUTOMATION.md)
