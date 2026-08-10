# 12 — End-to-End Automation (land → dock → transfer → process)

Once the drone touches down, **no human action is required**: the base station starts docking and
charging, the survey data copies itself to the ground PC, and the vision pipeline runs
automatically. This document explains that chain.

---

## 1. The full timeline

```
   ARM ──► autonomous survey flight ──► touchdown
                                            │
                    ┌───────────────────────┴────────────────────────┐
                    │  landing_transfer_node (Jetson) sees            │
                    │  /mavros/extended_state → landed                │
                    └───────────┬───────────────────┬────────────────┘
                                │                   │  (both happen at once)
              POST /api/landed  │                   │  rsync survey folder → PC
                                ▼                   ▼
                 ┌──────────────────────┐   ┌────────────────────────────┐
                 │ Dashboard waits 5 s  │   │ drone_photos/ fills up     │
                 │ → commands ESP32     │   └────────────┬───────────────┘
                 │   GET /landed        │                │ rsync exit 0
                 └──────────┬───────────┘                ▼
                            ▼                  POST /api/transfer_done
                 ┌──────────────────────┐                │
                 │ Rods drive in        │                ▼
                 │ contact + polarity   │   ┌────────────────────────────┐
                 │ → CHARGING           │   │ Dashboard runs the pipeline│
                 └──────────────────────┘   │ stitch → field → match →   │
                                            │ coordinates → annotate     │
                                            └────────────┬───────────────┘
                                                         ▼
                                              results/ + live dashboard panels
```

By the time the drone is sitting on the pad charging, the coordinates of every target are already
being computed.

---

## 2. The landing detector (Jetson)

**File:** `drone/scripts/landing_transfer_node.py`
**Service:** `landing-transfer.service` → `scripts/start_landing_transfer.sh`

```python
self.sub = self.create_subscription(
    ExtendedState, '/mavros/extended_state', self.cb, 10)

def cb(self, msg):
    if msg.landed_state in (4, 1):        # ON_GROUND / LANDED
        if not self.transferred_this_landing:
            self.notify_base_station()    # 1) fire docking
            self.run_transfer()           # 2) rsync the data
            self.transferred_this_landing = True
    elif msg.landed_state == 2:           # IN_AIR → re-arm for the next landing
        self.transferred_this_landing = False
```

Two details that matter:

- **The base-station notify is fire-and-forget**, on its own thread. A slow or unreachable
  dashboard can never delay the data transfer.
- **The flag resets on takeoff** (`landed_state == 2`), so a multi-sortie session transfers after
  every landing, but never twice for the same one.

### Configuration (top of the file)

| Constant | Purpose |
|---|---|
| `PC_USER`, `PC_IP`, `PC_PORT` | SSH target for the rsync |
| `PC_DEST_PATH` | Where photos land on the PC — must be the pipeline's `drone_photos/` |
| `SURVEY_ROOT` | Where the mission writes surveys on the SSD |
| `BASE_URL` | Dashboard address |
| `DOCK_TOKEN` | Must equal `IROC_TOKEN` on the dashboard |

> `PC_IP` and `BASE_URL` are DHCP-dependent. `change_ip.sh` updates them in one command.

### The transfer itself

```bash
rsync -avz --partial -e "ssh -p <port>" <latest_survey_folder>/ user@pc:<drone_photos>/
```

- **`--partial`** — a dropped WiFi link resumes instead of restarting.
- **No `--delete`** — the transfer only ever adds or updates files on the PC. Nothing is deleted,
  anywhere. This is deliberate: an automated process should never be able to destroy flight data.
- The **most recently modified** `*survey_*` folder is chosen, so it always sends the flight that
  just happened.
- 300 s timeout; on failure the error is logged and nothing downstream is triggered.

---

## 3. Base-station endpoints

| Endpoint | Called by | Effect |
|---|---|---|
| `POST /api/landed?token=…` | Jetson, on touchdown | Waits `DOCK_DELAY_S` (5 s), then calls the ESP32's `/landed` to start docking |
| `POST /api/transfer_done?token=…` | Jetson, after a successful rsync | Starts the vision pipeline automatically |
| `POST /api/dock_register?token=…&ip=…` | ESP32, on boot + every 30 s | Tells the dashboard the ESP32's current IP |
| `POST /api/dock_log?token=…` | ESP32, continuously | Streams the docking/charging log into the dashboard |

All four bypass the browser password but require the shared token, so the machines can talk to
each other without a login while the UI stays protected.

**Why the 5-second delay before docking:** the drone must be fully settled and disarmed before the
rods move. Starting immediately on the first touchdown sample risks pushing a drone that is still
bouncing.

---

## 4. What each machine is responsible for

| Machine | Responsibility |
|---|---|
| **Jetson** | Detect landing · notify · transfer data |
| **Ground PC** | Receive data · run the pipeline · command the ESP32 · display everything |
| **ESP32** | Dock the drone · detect contact and polarity · measure voltage · charge |

No machine depends on another being available:
- Dashboard down → transfer still happens (notify just fails and is logged)
- ESP32 down → transfer and pipeline still run
- Jetson transfer fails → nothing downstream triggers; data stays on the SSD for manual copy

---

## 5. Manual fallbacks

Everything automated has a manual equivalent:

| Automated | Manual |
|---|---|
| Landing transfer | `manual_transfer.sh` on the Jetson |
| Docking | **Start Docking** button in the dashboard, or the ESP32's own web page |
| Pipeline run | **Run** in the dashboard, or `python3 iroc_pipeline_fixed.py` |

---

## 6. Setting it up on a new system

1. **Passwordless SSH** from the Jetson to the PC (otherwise the rsync will hang waiting for a
   password):
   ```bash
   # on the Jetson
   ssh-keygen -t ed25519
   ssh-copy-id -p <port> <user>@<pc-ip>
   ```
2. **Edit the constants** at the top of `landing_transfer_node.py`.
3. **Install the service:**
   ```bash
   sudo cp landing-transfer.service /etc/systemd/system/
   sudo systemctl daemon-reload
   sudo systemctl enable --now landing-transfer.service
   ```
4. **Verify:**
   ```bash
   systemctl status landing-transfer.service
   journalctl -u landing-transfer.service -f     # should print "Waiting for drone to land..."
   ```
5. **Match the tokens** — `DOCK_TOKEN` (Jetson) = `DOCK_TOKEN` (ESP32) = `IROC_TOKEN` (dashboard).

### Test it without flying

```bash
# on the PC — should trigger the docking sequence
curl -i -X POST "http://<pc-ip>:8000/api/landed?token=<TOKEN>"

# should start the pipeline
curl -i -X POST "http://<pc-ip>:8000/api/transfer_done?token=<TOKEN>"
```

Both should return 200 and be visible in the dashboard log.

---

## 7. Troubleshooting

| Symptom | Cause | Fix |
|---|---|---|
| Nothing happens after landing | Service not running | `systemctl status landing-transfer.service` |
| "Waiting for drone to land..." forever | No `/mavros/extended_state` | Check MAVROS is up on the Jetson |
| rsync hangs | SSH asks for a password | Set up passwordless SSH (step 1 above) |
| rsync fails: no route to host | PC IP changed (DHCP) | Update `PC_IP` / `BASE_URL`, or run `change_ip.sh` |
| Docking never starts | Notify failed, or token mismatch | Check the log; verify all three tokens match |
| Pipeline doesn't auto-start | `transfer_done` never sent (rsync non-zero) | Read the rsync error in the journal |
| Transfers twice | — | Cannot happen: guarded by `transferred_this_landing` |
| Photos land in the wrong folder | `PC_DEST_PATH` | Must point at the pipeline's `drone_photos/` |

---

## 8. Legacy startup script

`drone/start_drone.sh` starts an **older** stack in tmux panes (ORB-SLAM3 + the standalone
`rs_pipeline_node.py` + MAVROS + `mavros_vision_bridge.py`). It is kept for reference.

**The current system uses `bringup.launch.py`** — see
[09 — Drone Software](09_DRONE_SOFTWARE.md). Do not run both: two camera drivers or two vision
bridges at once will corrupt the odometry.

---

**Back to:** [09 — Drone Software](09_DRONE_SOFTWARE.md) · [03 — Data Transfer](03_DATA_TRANSFER.md)
