# 05 — Setup

How to get this running on a computer that has never seen the project. Follow the section that
matches your machine, then the common steps.

**Estimated time:** 20–40 minutes (mostly PyTorch downloading).

---

## 1. What you need

| Requirement | Minimum | Notes |
|---|---|---|
| OS | Ubuntu 20.04+ / Windows 10+ with WSL2 | Native Windows works but WSL2 is smoother |
| Python | 3.10+ | `python3 --version` |
| RAM | 8 GB | 16 GB comfortable |
| Disk | ~10 GB free | PyTorch + models + results |
| GPU | *optional* | NVIDIA GPU makes stitching/matching much faster; CPU works |
| Internet | Required on first run | Models download automatically |

---

## 2. Install Python and tools

### 2.1 Ubuntu / WSL2 / Jetson

```bash
sudo apt update
sudo apt install -y python3 python3-pip git
```

### 2.2 Windows (WSL2)

In PowerShell **as Administrator**:

```powershell
wsl --install -d Ubuntu-22.04
```

Restart, open **Ubuntu** from the Start menu, create your username/password, then follow the
Ubuntu instructions above from inside that terminal.

> Everything after this point happens **inside WSL**, not in PowerShell.

---

## 3. Get the code

```bash
cd ~
git clone https://github.com/<your-username>/ascend_iroc_2026_team_LUMA.git
cd ascend_iroc_2026_team_LUMA
```

New to git? See [06 — Git & GitHub](06_GIT_GITHUB.md).

---

## 4. Install Python packages

```bash
pip install -r requirements.txt --break-system-packages
```

> `--break-system-packages` is needed on Ubuntu 23+/Debian because the system Python is marked
> "externally managed". On older systems you can drop it. If you prefer isolation, use a
> virtual environment instead:
> ```bash
> python3 -m venv .venv && source .venv/bin/activate
> pip install -r requirements.txt
> ```

### 4.1 PyTorch with GPU (optional but recommended)

The generic `torch` from `requirements.txt` may be CPU-only. For an NVIDIA GPU, install the build
matching your CUDA version from <https://pytorch.org/get-started/locally/>, for example:

```bash
pip install torch torchvision --index-url https://download.pytorch.org/whl/cu121 --break-system-packages
```

Verify:

```bash
python3 -c "import torch; print('CUDA:', torch.cuda.is_available())"
```

`False` is fine — everything still runs, just slower.

### 4.2 Dashboard packages

```bash
pip install -r base_station/requirements.txt --break-system-packages
```

---

## 5. First run — verify with synthetic data

Before using real flight data, prove the install works:

```bash
python3 make_test_dataset.py        # creates ~/advanced_matcher_testset/
```

Copy the generated `drone_photos/` and `targets/` into the project folder, then:

```bash
python3 iroc_pipeline_fixed.py
```

On the **first** run the models download automatically (LoFTR via kornia, DINOv2 via torch.hub) —
this needs internet and takes a few minutes. Later runs are offline.

Success looks like:

```
Stitched 35/35 photos
[fix#3] TRUE size 10.67 x 7.62 m
Found 3/3 targets
ALL STAGES DONE
```

Then compare against `ground_truth.txt` from the generator.

---

## 6. Run with real data

### 6.1 Folder layout

```
ascend_iroc_2026_team_LUMA/
├── drone_photos/            ← HD photos + coordinates.csv   (from the Jetson)
└── targets/                 ← seed images, 64×64            (what to look for)
```

Copying from the drone is covered in [03 — Data Transfer](03_DATA_TRANSFER.md).

### 6.2 Set your arena size

Edit the top of `iroc_pipeline_fixed.py`:

```python
ARENA_LONG_FT  = 35     # your arena's long edge, in feet
ARENA_SHORT_FT = 25     # your arena's short edge
```

Which edge is which is detected automatically; only these two numbers must be correct.

### 6.3 Run

```bash
python3 iroc_pipeline_fixed.py
```

### 6.4 Read the results

| File | What it is |
|---|---|
| `results/stage3_targets/targets.json` | Final coordinates (`map_xyz`) per target |
| `results/stage3_targets/proof_hd/<t>.jpg` | HD image deliverable |
| `results/stage3_targets/lr_match/<t>.png` | LR image deliverable |
| `results/stage4_annotated/annotated_field.jpg` | Annotated arena map |
| `results/stage1_stitch/orthomosaic.jpg` | The stitched map |
| `results/stage2_field/yellow_mask_debug.jpg` | **Check this first if coordinates look wrong** |

---

## 7. Dashboard

```bash
python3 -m base_station.server
```

Open **<http://localhost:8000>** — login `luma` / `ascend2026` (change with `IROC_USER` /
`IROC_PASS`).

With a real drone connected through mavlink-router on the Jetson:

```bash
DRONE_LINK_MODE=mavlink MAVLINK_CONN="tcp:<JETSON-IP>:5760" python3 -m base_station.server
```

Without hardware it starts in simulator mode automatically, so the UI always works.

### Useful environment variables

| Variable | Default | Meaning |
|---|---|---|
| `BASE_STATION_PORT` | `8000` | HTTP port |
| `DRONE_LINK_MODE` | `simulator` | `mavlink` / `simulator` / `auto` |
| `MAVLINK_CONN` | `udpin:0.0.0.0:14550` | MAVLink endpoint |
| `IROC_USER` / `IROC_PASS` | `luma` / `ascend2026` | Dashboard login |
| `IROC_AUTH` | `1` | `0` disables the password (trusted LAN only) |
| `IROC_TOKEN` | `lumadock` | Shared token for ESP32 calls — must match the firmware |

---

## 8. Optional — 3D reconstruction

Needs Docker:

```bash
sudo apt install -y docker.io
sudo usermod -aG docker $USER      # then log out and back in
sudo service docker start
python3 3d.py                      # or: python3 iroc_pipeline_fixed.py --run-3d
```

Outputs land in `results/3d_map/` (`model.glb` opens in Windows 3D Viewer, Blender, MeshLab).

---

## 9. Optional — ESP32 firmware

1. Install the Arduino IDE, add the ESP32 board URL
   `https://espressif.github.io/arduino-esp32/package_esp32_index.json`.
2. Open `esp32_firmware/full_base_station_wifi.ino`.
3. Edit `WIFI_SSID`, `WIFI_PASS`, `BASE_URL` (your PC's IP), `DOCK_TOKEN`, `DIVIDER_RATIO`.
4. Select **ESP32 Dev Module**, pick the port, Upload.
5. Serial Monitor @ 115200 prints its IP.

Wiring details: [01 — Hardware](01_HARDWARE.md).

---

## 10. Troubleshooting

| Problem | Fix |
|---|---|
| `ModuleNotFoundError: cv2 / torch / kornia` | Re-run the pip install; if you used a venv, activate it first |
| `externally-managed-environment` error | Add `--break-system-packages`, or use a venv |
| First run hangs at "Loading LoFTR / DINOv2" | Models are downloading — needs internet, be patient |
| `CUDA out of memory` | Close other GPU apps, or force CPU: `CUDA_VISIBLE_DEVICES="" python3 …` |
| Photos dropped during stitching | Confirm `coordinates.csv` exists with `x_enu,y_enu`; try `--radius 2` |
| Coordinates look wrong | Open `yellow_mask_debug.jpg`; it must show only the tape frame |
| Target NOT FOUND | Lower `MIN_FOUND_PEAK` / `VERIFY_MIN` in `stage3_robust.py` |
| Dashboard won't open | Check the port is free; try `http://127.0.0.1:8000` |
| Dashboard shows SIMULATOR | No MAVLink heartbeat — verify `MAVLINK_CONN` and `nc -vz <JETSON-IP> 5760` |
| Docker permission denied (3D) | `sudo usermod -aG docker $USER`, then log out/in |

More: [RUN_GUIDE.md](../RUN_GUIDE.md) · [PARAMETERS_GUIDE.md](../PARAMETERS_GUIDE.md)

---

**Next:** [06 — Git & GitHub](06_GIT_GITHUB.md)
