# 00 — Install Everything (from zero)

Everything you need to download and install to run this project on a completely fresh computer —
starting from a bare Windows machine.

Work through it top to bottom. Skip a section only if you know you don't need that part.

---

## 0. What you actually need (quick view)

| # | Software | Needed for | Required? |
|---|---|---|---|
| 1 | **WSL2 + Ubuntu 22.04** | Running everything on Windows | ✅ (Windows only) |
| 2 | **Python 3.10+ & pip** | The whole pipeline | ✅ |
| 3 | **git** | Downloading/updating the code | ✅ |
| 4 | **Python packages** (`requirements.txt`) | Stitching + matching | ✅ |
| 5 | **PyTorch** (CPU or CUDA) | LoFTR + DINOv2 models | ✅ |
| 6 | **Dashboard packages** (`base_station/requirements.txt`) | Web dashboard | ✅ (for the dashboard) |
| 7 | **Docker** | 3D reconstruction (`3d.py`) | ⬜ optional |
| 8 | **Arduino IDE + ESP32 support** | Flashing the docking firmware | ⬜ only if you touch the ESP32 |
| 9 | **QGroundControl** | PX4 setup, motor test, flight | ⬜ flight team |
| 10 | **mavlink-router** | Telemetry routing (runs on the **Jetson**) | ⬜ for live telemetry |
| 11 | **MeshLab / CloudCompare** | Viewing 3D point clouds | ⬜ optional |

**Disk space:** ~10 GB total (PyTorch alone is ~3 GB, plus ~2 GB for models and results).
**Internet:** required for installation and for the first pipeline run (models download once).

---

## 1. Windows → install WSL2 + Ubuntu

Skip this if you already use Linux.

Open **PowerShell as Administrator** (right-click Start → Terminal (Admin)):

```powershell
wsl --install -d Ubuntu-22.04
```

Restart the PC. Open **Ubuntu 22.04** from the Start menu — the first launch asks you to create a
username and password (this is your Linux account; the password is invisible while typing).

Check it worked:

```bash
lsb_release -a        # should say Ubuntu 22.04
```

> **From here on, every command goes into the Ubuntu (WSL) terminal**, not PowerShell.

Useful to know:
- Your Windows drives are visible at `/mnt/c/...`
- Your Linux home is `~` (i.e. `/home/<your-name>`) — keep the project here, it is much faster
- To open the Linux folder in Windows Explorer: `explorer.exe .`

---

## 2. Base tools (Ubuntu / WSL / Jetson)

```bash
sudo apt update && sudo apt upgrade -y
sudo apt install -y python3 python3-pip git curl wget nano build-essential
```

Verify:

```bash
python3 --version     # 3.10 or newer
pip3 --version
git --version
```

---

## 3. Get the code

```bash
cd ~
git clone https://github.com/<your-username>/gps-denied-drone-survey.git
cd gps-denied-drone-survey
```

Never used git before? → [06 — Git & GitHub](06_GIT_GITHUB.md)

---

## 4. Python packages — the pipeline

```bash
pip install -r requirements.txt --break-system-packages
```

This installs:

| Package | Why |
|---|---|
| `opencv-python` | All image processing |
| `numpy`, `scipy` | Maths, bundle adjustment |
| `torch`, `torchvision` | Runs the deep models |
| `kornia` | LoFTR matcher (stitching) |
| `trimesh` | Exports `model.glb` (3D, optional) |

> `--break-system-packages` is needed on Ubuntu 23+/Debian. Prefer isolation? Use a venv:
> ```bash
> python3 -m venv .venv && source .venv/bin/activate
> pip install -r requirements.txt
> ```
> (You must `source .venv/bin/activate` in every new terminal.)

### 4.1 PyTorch with NVIDIA GPU (optional, much faster)

The default `torch` may be CPU-only. With an NVIDIA GPU, install the matching CUDA build from
<https://pytorch.org/get-started/locally/> — for CUDA 12.1:

```bash
pip install torch torchvision --index-url https://download.pytorch.org/whl/cu121 --break-system-packages
```

Check:

```bash
python3 -c "import torch; print('CUDA:', torch.cuda.is_available())"
```

`False` is fine — everything still works, just slower.

> **DINOv2 and LoFTR weights are NOT installed manually.** They download automatically on the
> first run (torch.hub / kornia) and are cached in `~/.cache/`. That first run needs internet.

---

## 5. Python packages — the dashboard

```bash
pip install -r base_station/requirements.txt --break-system-packages
```

| Package | Why |
|---|---|
| `fastapi` | Web server |
| `uvicorn[standard]` | Runs the server + WebSockets |
| `watchdog` | Auto-refresh when `results/` changes |
| `pymavlink` | MAVLink telemetry link |

Start it:

```bash
python3 -m base_station.server
```

Open **<http://localhost:8000>** → login `luma` / `ascend2026`.

Connected to a real drone (mavlink-router running on the Jetson):

```bash
DRONE_LINK_MODE=mavlink MAVLINK_CONN="tcp:<JETSON-IP>:5760" python3 -m base_station.server
```

With no hardware it starts in simulator mode, so the UI always comes up.

---

## 6. Docker — only for 3D reconstruction

Skip unless you want `3d.py` / `--run-3d`.

```bash
sudo apt install -y docker.io
sudo usermod -aG docker $USER      # then LOG OUT and back in (or restart WSL)
sudo service docker start
docker run hello-world             # verify
```

The OpenDroneMap image (~3 GB) downloads automatically on the first 3D run.

For GPU-accelerated ODM you also need the NVIDIA Container Toolkit — CPU works fine, just slower.

---

## 7. Arduino IDE — only for the ESP32 firmware

Skip unless you are flashing the docking/charging board.

1. Download the Arduino IDE: <https://www.arduino.cc/en/software> (install on **Windows**, not WSL —
   USB access is simpler there).
2. **File → Preferences → Additional Board Manager URLs**, paste:
   ```
   https://espressif.github.io/arduino-esp32/package_esp32_index.json
   ```
3. **Tools → Board → Boards Manager** → search `esp32` → install **esp32 by Espressif Systems**.
4. Open `esp32_firmware/full_base_station_wifi.ino`.
5. **Tools → Board → ESP32 Dev Module**, select the COM port.
6. Edit the settings at the top (`WIFI_SSID`, `WIFI_PASS`, `BASE_URL`, `DOCK_TOKEN`, `DIVIDER_RATIO`).
7. **Upload**, then open Serial Monitor at **115200** baud.

If the board is not detected, install the USB driver: **CP210x** or **CH340** depending on your
ESP32 model.

---

## 8. QGroundControl — PX4 setup and flight

Skip if you are not doing flight setup.

- Download: <https://qgroundcontrol.com/downloads/> (Windows version is easiest)
- Used for: PX4 firmware flashing, sensor calibration, **Actuators / Motor Test**, mission planning
- Connect over USB, or over the network via mavlink-router UDP `14550`

---

## 9. mavlink-router — on the Jetson

This runs on the **drone's Jetson**, not on your laptop.

```bash
# on the Jetson
sudo apt install -y git meson ninja-build pkg-config gcc g++ systemd
git clone https://github.com/mavlink-router/mavlink-router.git
cd mavlink-router
git submodule update --init --recursive
meson setup build .
ninja -C build
sudo ninja -C build install
```

Then create `/etc/mavlink-router/main.conf` (full example in
[03 — Data Transfer §4.1](03_DATA_TRANSFER.md#41-jetson-config--etcmavlink-routermainconf)) and run:

```bash
mavlink-routerd -c /etc/mavlink-router/main.conf
```

---

## 10. Optional extras

| Tool | Why | Link |
|---|---|---|
| **VS Code** + WSL extension | Comfortable code editing | <https://code.visualstudio.com/> |
| **MeshLab** | Open `.ply` coloured point clouds | <https://www.meshlab.net/> |
| **CloudCompare** | Point-cloud inspection | <https://www.cloudcompare.org/> |
| **Windows 3D Viewer** | Opens `model.glb` directly | Microsoft Store |
| **rsync** | Fast resumable file copy from the Jetson | `sudo apt install rsync` |

---

## 11. Verify the whole install

```bash
cd ~/gps-denied-drone-survey

# 1. libraries import cleanly
python3 -c "import cv2, numpy, scipy, torch, kornia; print('pipeline OK')"
python3 -c "import fastapi, uvicorn, pymavlink; print('dashboard OK')"

# 2. end-to-end test on synthetic data (no drone needed)
python3 make_test_dataset.py
cp -r ~/advanced_matcher_testset/drone_photos ./
cp -r ~/advanced_matcher_testset/targets ./
python3 iroc_pipeline_fixed.py
```

Expected finish:

```
Stitched 35/35 photos
[fix#3] TRUE size …
Found 3/3 targets
ALL STAGES DONE
```

Then compare with `~/advanced_matcher_testset/ground_truth.txt`.

---

## 12. Install-time problems

| Problem | Fix |
|---|---|
| `wsl --install` not recognised | Update Windows, or enable "Virtual Machine Platform" + "WSL" in Windows Features |
| `externally-managed-environment` | Add `--break-system-packages`, or use a venv |
| `pip: command not found` | `sudo apt install python3-pip` |
| torch install very slow | Normal — it is ~3 GB; use a good connection |
| `CUDA: False` but you have a GPU | Install the CUDA-specific torch build (§4.1); on WSL also update your Windows NVIDIA driver |
| First run stuck at "Loading LoFTR/DINOv2" | Models are downloading — needs internet, wait it out |
| `docker: permission denied` | `sudo usermod -aG docker $USER`, then log out/in |
| ESP32 board not listed | Install CP210x / CH340 USB driver |
| Dashboard port already in use | `BASE_STATION_PORT=8001 python3 -m base_station.server` |

---

## 13. Summary — minimum path

Just want the pipeline and dashboard running?

```bash
# Windows only:
wsl --install -d Ubuntu-22.04            # then restart

# inside Ubuntu:
sudo apt update && sudo apt install -y python3 python3-pip git
git clone https://github.com/<your-username>/gps-denied-drone-survey.git
cd gps-denied-drone-survey
pip install -r requirements.txt --break-system-packages
pip install -r base_station/requirements.txt --break-system-packages

python3 iroc_pipeline_fixed.py           # pipeline
python3 -m base_station.server           # dashboard → http://localhost:8000
```

Everything else (Docker, Arduino, QGroundControl, mavlink-router) is only needed for the
extra subsystems.

---

**Next:** [01 — Hardware](01_HARDWARE.md) · [05 — Setup](05_SETUP.md)
