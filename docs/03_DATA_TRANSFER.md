# 03 — Data Transfer

How images and telemetry move from the drone to the ground PC, how MAVLink is routed, and what
the ground station expects on disk.

---

## 1. The two data streams

| Stream | From → To | Transport | Used for |
|---|---|---|---|
| **Telemetry** (live) | Pixhawk → Jetson → PC | MAVLink over USB, then TCP/UDP over WiFi | Dashboard: battery, altitude, position, mode |
| **Survey data** (bulk) | Jetson → PC | File copy over WiFi (`scp` / `rsync`) | The pipeline: HD photos + `coordinates.csv` |

The dashboard runs on the PC and reads *both*.

---

## 2. Survey data — what the pipeline needs

After a sortie you need exactly two things on the ground PC:

```
drone_photos/
├── cp0000_c00s00.jpg      ← HD survey photos (1280×720)
├── cp0001_c00s01.jpg
├── …
└── coordinates.csv        ← one row per photo: pose at capture time
```

### 2.1 `coordinates.csv` format

```csv
checkpoint,row,col,timestamp_s,x_enu,y_enu,z_enu,yaw_deg,image_file
0,0,0,1784067291.300,-0.6392,0.4905,3.0527,-90.0,cp0000_c00s00.jpg
2,0,2,1784067310.470,-0.6710,-2.9349,2.9921,-90.0,cp0002_c00s02.jpg
```

| Column | Meaning | Used by |
|---|---|---|
| `checkpoint` | Sequential capture index | bookkeeping |
| `row`, `col` | Survey grid indices | Stage 1 (anchor / grid) |
| `timestamp_s` | Unix time of capture | bookkeeping |
| `x_enu`, `y_enu` | Position in metres, **relative to takeoff** (VIO) | Stage 1 pairing, Stage 4 base-station origin |
| `z_enu` | Height in metres (Pixhawk) | reported z |
| `yaw_deg` | Heading in degrees | arena axes |
| `image_file` | Photo filename this row belongs to | links pose ↔ photo |

**This file is the backbone of the whole system.** Stage 1 uses `x_enu, y_enu` to decide which
photos overlap, and Stage 4 uses VIO `(0,0)` — the takeoff point — as the base-station origin.

> The origin is the **takeoff point**, so the drone must take off from the base station for
> coordinates to be reported relative to it.

### 2.2 Seed images

```
targets/
├── feature_1.png      ← 64×64 seed images (one per object you want to find)
├── feature_2.png
└── feature_3.png
```

For practice you can generate seeds from full-resolution references:

```bash
python3 make_lr.py        # reference/ → targets/  (resizes to 64×64)
```

### 2.3 Automatically derived

```
drone_photos_lr/          ← 128×128 LR versions, created automatically by the matcher
```

You never create this by hand — `build_drone_lr()` down-samples every HD photo to 128×128 on the
first run. These are the images the matcher actually searches.

---

## 3. Transferring the survey folder (Jetson → PC)

### 3.1 With `scp` (simple)

```bash
# run on the PC
scp -r jetson@<JETSON-IP>:~/survey/drone_photos  ~/advanced_matcher/
```

### 3.2 With `rsync` (recommended — resumable, skips existing files)

```bash
rsync -avh --progress jetson@<JETSON-IP>:~/survey/drone_photos/  ~/advanced_matcher/drone_photos/
```

`rsync` is safer for a field day: if the link drops you just run it again and it continues.

### 3.3 Verify before processing

```bash
ls ~/advanced_matcher/drone_photos/*.jpg | wc -l      # how many photos
wc -l ~/advanced_matcher/drone_photos/coordinates.csv # rows (photos + 1 header)
```

The photo count and CSV row count should match. If `coordinates.csv` is missing or short, Stage 1
falls back to filename-based pairing and the mosaic quality drops.

---

## 4. Telemetry — MAVLink routing

The Pixhawk is connected to the **Jetson** by USB. `mavlink-router` on the Jetson takes that single
serial stream and re-broadcasts it to several consumers at once — so QGroundControl **and** the
dashboard can both be connected without fighting over the port.

### 4.1 Jetson config — `/etc/mavlink-router/main.conf`

```ini
[General]
TcpServerPort = 5760
ReportStats = false
MavlinkDialect = common          # PX4 uses the "common" dialect

[UartEndpoint PX4]
Device = /dev/ttyACM0
Baud = 921600

[UdpEndpoint QGC]
Mode = Normal
Address = <GROUND-PC-IP>
Port = 14550

[UdpEndpoint Script]
Mode = Normal
Address = 127.0.0.1
Port = 14540
```

Run it:

```bash
mavlink-routerd -c /etc/mavlink-router/main.conf
```

Expected output:

```
Opened UART [4]PX4: /dev/ttyACM0
UART [4]PX4: speed = 921600
Opened UDP Client [5]Script: 127.0.0.1:14540
Opened UDP Client [7]QGC: <PC-IP>:14550
Opened TCP Server [9] [::]:5760
```

### 4.2 Endpoints and who uses them

| Endpoint | Type | Consumer |
|---|---|---|
| `5760` | **TCP server** (accepts many clients) | **Ground dashboard** ← use this |
| `14550` | UDP → PC | QGroundControl |
| `14540` | UDP → localhost | Onboard scripts on the Jetson |

### 4.3 Connecting the dashboard

On the ground PC:

```bash
cd ~/advanced_matcher
DRONE_LINK_MODE=mavlink MAVLINK_CONN="tcp:<JETSON-IP>:5760" python3 -m base_station.server
```

Check reachability first:

```bash
ping -c2 <JETSON-IP>
nc -vz <JETSON-IP> 5760       # must say "succeeded"
```

Other connection-string forms the dashboard accepts:

| String | Use |
|---|---|
| `tcp:<ip>:5760` | mavlink-router TCP server (**recommended**) |
| `udpin:0.0.0.0:14560` | Listen for a UDP endpoint pointed at this PC |
| `udp:127.0.0.1:14550` | Connect to a local UDP endpoint |
| `/dev/ttyUSB0,57600` | Direct serial telemetry radio |

If no heartbeat arrives within the timeout, the dashboard automatically falls back to its built-in
simulator so the UI still works.

> **Note on QGroundControl:** if QGC is connected directly to the Pixhawk by USB (not through
> mavlink-router) then mavlink-router has no input and none of its endpoints will carry data.
> Route everything through mavlink-router.

---

## 5. Charging telemetry (ESP32 → dashboard)

Battery voltage during charging does **not** travel over MAVLink. The ESP32 sends its serial log
straight to the dashboard over WiFi:

```
ESP32 ──HTTP POST /api/dock_log  (X-Auth-Token header)──► dashboard "Docking & Charging" panel
ESP32 ──HTTP POST /api/dock_register?…&ip=…────► dashboard learns the ESP32's IP
```

Details in [02 — Docking & Charging §7](02_DOCKING_CHARGING.md#7-network-integration-with-the-dashboard).

---

## 6. Network setup for a field day

1. Start the WiFi network (router or phone hotspot, SSID `LUMA`).
2. Power the Jetson → it joins and starts `mavlink-router`.
3. Power the ESP32 → it joins and registers its IP with the dashboard.
4. Connect the ground PC to the same network; note its IP (`hostname -I`) — the ESP32's
   `BASE_URL` must point at it.
5. Start the dashboard with `MAVLINK_CONN="tcp:<JETSON-IP>:5760"`.

**Fixed addresses save time.** DHCP can hand out new IPs between sessions; giving the Jetson and
the PC static leases avoids re-flashing the ESP32 with a new `BASE_URL`.

---

## 7. After the flight — processing order

```bash
cd ~/advanced_matcher

# 1. copy the survey data across
rsync -avh --progress jetson@<JETSON-IP>:~/survey/drone_photos/ drone_photos/

# 2. put the seed images in targets/   (64×64 crops of what to find)

# 3. run everything
python3 iroc_pipeline_fixed.py

# 4. read the results
#    results/stage3_targets/targets.json          ← coordinates
#    results/stage3_targets/proof_hd/<t>.jpg      ← HD deliverable
#    results/stage3_targets/lr_match/<t>.png      ← LR deliverable
#    results/stage4_annotated/annotated_field.jpg ← annotated map
```

Or press **Run** in the dashboard, which launches the same pipeline and streams the log into the
browser.

---

**Next:** [04 — Pipeline / Feature Detection](04_PIPELINE.md)
