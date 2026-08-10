# 00 — Install Everything (from zero)

Every piece of software you need, on all three machines, in the order you actually use them:
**first get the drone flying and surveying, then set up the ground processing.**

---

## The three machines

| | Machine | What it does | Install part |
|---|---|---|---|
| 🚁 | **Jetson Orin Nano** (on the drone) | Flies the autonomous survey, captures photos + pose | **Part A** |
| 💻 | **Ground PC** (laptop) | Feature detection pipeline + live dashboard | **Part B** |
| 🔌 | **ESP32** (base station) | Auto-docking and battery charging | **Part C** |

You can install them independently, but this is the order that gets you a working system fastest:
**A → B → C**. Without Part A there is no survey data to process; Part C is optional convenience.

**Total time:** ~1 hour for A, ~40 min for B, ~15 min for C.
**Internet:** required for installation and the ground PC's first pipeline run (models download once).

---

## Software checklist

### 🚁 Part A — Drone (Jetson)

| # | Software | Needed for | Required? |
|---|---|---|---|
| 1 | Ubuntu 22.04 + **ROS 2 Humble** | Everything on the drone | ✅ |
| 2 | **MAVROS** + GeographicLib datasets | Talking to the Pixhawk | ✅ |
| 3 | **RTAB-Map ROS** | Visual odometry (VIO) | ✅ |
| 4 | **librealsense2** | RealSense D455 camera | ✅ |
| 5 | **mavlink-router** | Routing MAVLink to MAVROS + QGC + dashboard | ✅ |
| 6 | **`viman_mission`** workspace | The missions themselves | ✅ |
| 7 | **QGroundControl** (on a laptop) | PX4 setup, calibration, motor test | ✅ |

### 💻 Part B — Ground PC

| # | Software | Needed for | Required? |
|---|---|---|---|
| 8 | **WSL2 + Ubuntu 22.04** | Running everything on Windows | ✅ (Windows only) |
| 9 | **Python 3.10+**, pip, git | The pipeline | ✅ |
| 10 | **PyTorch** (CPU or CUDA) | LoFTR + DINOv2 models | ✅ |
| 11 | Pipeline packages (`requirements.txt`) | Stitching + feature detection | ✅ |
| 12 | Dashboard packages (`base_station/requirements.txt`) | Web dashboard | ✅ |
| 13 | **Docker** | 3D reconstruction (`3d.py`) | ⬜ optional |

### 🔌 Part C — Base station

| # | Software | Needed for | Required? |
|---|---|---|---|
| 14 | **Arduino IDE + ESP32 support** | Flashing the docking firmware | ⬜ only if you use the dock |

---

# 🚁 PART A — Drone (Jetson Orin Nano)

This gets you to a flying, surveying drone. Detailed per-step tests are in
[13 — Operations](13_OPERATIONS.md#part-1--first-time-setup-props-off) — do them as you go.

## A1. Base system

```bash
sudo apt update && sudo apt upgrade -y
sudo apt install -y python3-pip git tmux nano rsync curl wget
source /opt/ros/humble/setup.bash
ros2 --version                # must print a version
```

> ROS 2 Humble must already be installed (it ships with most JetPack images for Ubuntu 22.04).
> If not: <https://docs.ros.org/en/humble/Installation.html>

## A2. ROS 2 packages

```bash
sudo apt install -y \
  ros-humble-mavros ros-humble-mavros-extras \
  ros-humble-rtabmap-ros ros-humble-rtabmap-launch \
  ros-humble-cv-bridge ros-humble-image-transport \
  ros-humble-tf2-ros ros-humble-vision-msgs

# MAVROS needs this once, or it refuses to start
sudo /opt/ros/humble/lib/mavros/install_geographiclib_datasets.sh
```

✅ `ros2 pkg list | grep -E "mavros|rtabmap"` — both appear.

## A3. RealSense camera

```bash
sudo apt install -y librealsense2-utils librealsense2-dev python3-pyrealsense2
```

✅ Plug the D455 into a **USB 3 (blue)** port, then `rs-enumerate-devices` must list it.
USB 2 cannot carry 1280×720 @ 30 fps RGB-D.

## A4. mavlink-router

```bash
git clone https://github.com/mavlink-router/mavlink-router.git
cd mavlink-router && git submodule update --init --recursive
meson setup build . && ninja -C build && sudo ninja -C build install

sudo mkdir -p /etc/mavlink-router
sudo cp <repo>/drone/main.conf /etc/mavlink-router/main.conf
sudo nano /etc/mavlink-router/main.conf      # set [UdpEndpoint QGC] Address to your PC's IP
```

✅ `mavlink-routerd -c /etc/mavlink-router/main.conf` prints
`Opened UART [4]PX4: /dev/ttyACM0` and `Opened TCP Server [9] [::]:5760`.

## A5. The mission workspace

```bash
mkdir -p ~/drone_ws/src && cd ~/drone_ws/src
cp -r <repo>/drone/viman_mission      .
cp -r <repo>/drone/whycode-ros2       .
cp -r <repo>/drone/whycode_interfaces .

cd ~/drone_ws && colcon build --symlink-install && source install/setup.bash
```

✅ `ros2 pkg executables viman_mission` lists ~19 executables.

## A6. Environment + configs

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

✅ A **new** terminal finds `viman_mission` without sourcing anything by hand.

## A7. SSD storage

```bash
mkdir -p /media/jetson/ROS2_SSD/maps /media/jetson/ROS2_SSD/survey
df -h /media/jetson/ROS2_SSD
```

> An SSD is **mandatory** — full-resolution RTAB-Map databases and survey imagery will overwhelm
> an SD card, and a stalled write will stall odometry.

## A8. PX4 setup (QGroundControl)

Install QGC on a laptop: <https://qgroundcontrol.com/downloads/>

1. Flash **PX4** firmware to the Cube Orange+.
2. Calibrate sensors: accelerometer, gyro, level horizon, compass, RC.
3. Load the parameters: **Vehicle Setup → Parameters → Tools → Load from file** →
   `drone/px4_params.params`, then reboot.
4. **Motor test with props OFF** — ESCs are on **AUX OUT 1–4**, so use the **AUX** tab.

✅ Confirm in the parameter search:
`EKF2_GPS_CTRL = 0` · `EKF2_OF_CTRL = 1` · `EKF2_EV_CTRL = 9` · `EKF2_HGT_REF = 2` ·
`EKF2_EV_DELAY = 80` · `COM_OBL_RC_ACT = AUTO.LAND`

Details: [11 — Pixhawk & PX4](11_PIXHAWK_PX4.md)

## A9. Auto-transfer service (survey data → ground PC)

```bash
mkdir -p ~/scripts && cp <repo>/drone/scripts/* ~/scripts/ && chmod +x ~/scripts/*.sh

# passwordless SSH to the ground PC — without this the transfer hangs forever
ssh-keygen -t ed25519
ssh-copy-id -p <pc-ssh-port> <pc-user>@<pc-ip>
```

Install the service with your settings as environment variables (nothing sensitive in the code) —
the full unit file is in [13 — Operations §1.8](13_OPERATIONS.md).

✅ `systemctl status landing-transfer.service` → active, log says *"Waiting for drone to land..."*

## A10. First flights

Do **not** jump straight to the survey. Work through the flight order in
[13 — Operations Part 4](13_OPERATIONS.md#part-4--first-flights-props-on-in-this-order):

```
manual hover → boundary guard → hover mission → square → corner finding → full survey
```

The survey command itself:

```bash
cd ~/drone_ws && source install/setup.bash
ros2 launch viman_mission survey_boundary.launch.py boundary_start_corner:=back_left
```

**Part A is complete when a survey flight produces a folder in
`/media/jetson/ROS2_SSD/survey/` containing photos and `coordinates.csv`.**

---

# 💻 PART B — Ground PC (feature detection + dashboard)

This is what turns the survey photos into target coordinates.

## B1. Windows only — WSL2

PowerShell **as Administrator**:

```powershell
wsl --install -d Ubuntu-22.04
```

Restart, open **Ubuntu**, create your username and password.

> Everything below runs **inside Ubuntu (WSL)**, not PowerShell.
> Your Linux home is `~`; keep the project there (much faster than `/mnt/c`).

✅ `lsb_release -a` → Ubuntu 22.04

## B2. Base tools

```bash
sudo apt update && sudo apt install -y python3 python3-pip git curl nano rsync
python3 --version        # 3.10+
```

## B3. Get the code

```bash
cd ~
git clone https://github.com/s-k-0-1/gps-denied-drone-survey.git
cd gps-denied-drone-survey
```

New to git? → [06 — Git & GitHub](06_GIT_GITHUB.md)

## B4. Pipeline packages (feature detection)

```bash
pip install -r requirements.txt --break-system-packages
```

| Package | Why |
|---|---|
| `opencv-python` | All image processing |
| `numpy`, `scipy` | Maths, bundle adjustment |
| `torch`, `torchvision` | Runs the deep models |
| `kornia` | **LoFTR** — image stitching |
| `trimesh` | 3D model export (optional) |

**GPU (recommended, much faster)** — install the CUDA build for your version from
<https://pytorch.org/get-started/locally/>:

```bash
pip install torch torchvision --index-url https://download.pytorch.org/whl/cu121 --break-system-packages
python3 -c "import torch; print('CUDA:', torch.cuda.is_available())"
```

`False` is fine — everything still runs, just slower.

> **DINOv2 and LoFTR weights are not installed manually.** They download automatically on the first
> run (torch.hub / kornia) into `~/.cache/`. That first run needs internet; later runs are offline.

✅ `python3 -c "import cv2, numpy, scipy, torch, kornia; print('pipeline OK')"`

## B5. Verify feature detection with synthetic data

Prove the install before using real flight data:

```bash
python3 make_test_dataset.py
cp -r ~/advanced_matcher_testset/drone_photos ./
cp -r ~/advanced_matcher_testset/targets ./
python3 iroc_pipeline_fixed.py
```

✅ Expected finish:

```
Stitched 35/35 photos
[fix#3] TRUE size …
Found 3/3 targets
ALL STAGES DONE
```

Compare against `~/advanced_matcher_testset/ground_truth.txt` — coordinates should land within
~0.3–0.4 m.

## B6. Run on real survey data

```
gps-denied-drone-survey/
├── drone_photos/     ← HD photos + coordinates.csv   (from the Jetson, Part A)
└── targets/          ← 64×64 seed images             (what to look for)
```

Set your arena size at the top of `iroc_pipeline_fixed.py`:

```python
ARENA_LONG_FT  = 35
ARENA_SHORT_FT = 25
```

```bash
python3 iroc_pipeline_fixed.py
```

Results in `results/` — coordinates in `stage3_targets/targets.json`.
Stage-by-stage detail: [07 — Stage Guide](07_STAGE_GUIDE.md)

## B7. Dashboard (base station software)

```bash
pip install -r base_station/requirements.txt --break-system-packages
python3 -m base_station.server
```

Open **<http://localhost:8000>** — login `luma` / `ascend2026` (change it, see below).

Connected to the real drone:

```bash
DRONE_LINK_MODE=mavlink MAVLINK_CONN="tcp:<jetson-ip>:5760" python3 -m base_station.server
```

Without hardware it starts in simulator mode, so the UI always works.

| Variable | Default | Meaning |
|---|---|---|
| `BASE_STATION_PORT` | `8000` | HTTP port |
| `DRONE_LINK_MODE` | `simulator` | `mavlink` / `simulator` / `auto` |
| `MAVLINK_CONN` | `udpin:0.0.0.0:14550` | MAVLink endpoint (use `tcp:<jetson-ip>:5760`) |
| `IROC_USER` / `IROC_PASS` | `luma` / `ascend2026` | **Change these** |
| `IROC_TOKEN` | `lumadock` | Shared machine token — **must match** the Jetson and ESP32 |

✅ `python3 -c "import fastapi, uvicorn, pymavlink; print('dashboard OK')"`

## B8. Optional — 3D reconstruction

```bash
sudo apt install -y docker.io
sudo usermod -aG docker $USER      # log out and back in
sudo service docker start
docker run hello-world

python3 3d.py                      # or: python3 iroc_pipeline_fixed.py --run-3d
```

The OpenDroneMap image (~3 GB) downloads on first use. Outputs → `results/3d_map/`.

---

# 🔌 PART C — Base station (ESP32 docking + charging)

Optional — the system flies and processes without it; this only automates charging.

## C1. Arduino IDE

1. Download: <https://www.arduino.cc/en/software> (install on **Windows**, not WSL — USB is simpler)
2. **File → Preferences → Additional Board Manager URLs**:
   ```
   https://espressif.github.io/arduino-esp32/package_esp32_index.json
   ```
3. **Tools → Board → Boards Manager** → search `esp32` → install **esp32 by Espressif Systems**
4. Board not detected? Install the **CP210x** or **CH340** USB driver.

## C2. Configure and flash

Open `esp32_firmware/full_base_station_wifi.ino` and set:

```cpp
WIFI_SSID / WIFI_PASS     // your network
BASE_URL                  // http://<ground-pc-ip>:8000
DOCK_TOKEN                // must match IROC_TOKEN on the dashboard
DIVIDER_RATIO             // your actual resistor divider
```

Select **ESP32 Dev Module** → pick the port → **Upload** → Serial Monitor @ **115200**.

✅ It prints its IP once connected, and the dashboard's *Docking & Charging* panel starts showing
its log.

⚠️ **Before charging anything:** verify the divider ratio keeps the ADC below ~2.4 V at maximum
pack voltage — see [01 — Hardware §3.3](01_HARDWARE.md#33-voltage-sensing-contact--battery-voltage).

Wiring and full behaviour: [02 — Docking & Charging](02_DOCKING_CHARGING.md)

---

# Optional extras

| Tool | Why | Link |
|---|---|---|
| **VS Code** + WSL extension | Comfortable editing | <https://code.visualstudio.com/> |
| **MeshLab** | Open `.ply` point clouds | <https://www.meshlab.net/> |
| **CloudCompare** | Point-cloud inspection | <https://www.cloudcompare.org/> |
| **Windows 3D Viewer** | Opens `model.glb` | Microsoft Store |

---

# Install problems

| Problem | Fix |
|---|---|
| `wsl --install` not recognised | Update Windows; enable "Virtual Machine Platform" + "WSL" in Windows Features |
| `externally-managed-environment` | Add `--break-system-packages`, or use a venv |
| `pip: command not found` | `sudo apt install python3-pip` |
| torch install very slow | Normal — ~3 GB |
| `CUDA: False` with a GPU | Install the CUDA torch build (B4); on WSL also update the Windows NVIDIA driver |
| First pipeline run stuck at "Loading LoFTR/DINOv2" | Models downloading — needs internet |
| MAVROS won't start | Run the GeographicLib script (A2) |
| RealSense not detected | USB 3 port + cable; try another port |
| `colcon build` fails | Missing ROS package — re-check A2 |
| `docker: permission denied` | `sudo usermod -aG docker $USER`, log out/in |
| ESP32 not listed | CP210x / CH340 driver |
| Dashboard port in use | `BASE_STATION_PORT=8001 python3 -m base_station.server` |

---

# Minimum path

Just the ground pipeline and dashboard (no drone yet):

```bash
wsl --install -d Ubuntu-22.04            # Windows only, then restart

sudo apt update && sudo apt install -y python3 python3-pip git
git clone https://github.com/s-k-0-1/gps-denied-drone-survey.git
cd gps-denied-drone-survey
pip install -r requirements.txt --break-system-packages
pip install -r base_station/requirements.txt --break-system-packages

python3 make_test_dataset.py             # synthetic data to prove it works
python3 iroc_pipeline_fixed.py
python3 -m base_station.server           # http://localhost:8000
```

---

**Next:** [01 — Hardware](01_HARDWARE.md) · [13 — Operations Runbook](13_OPERATIONS.md)
