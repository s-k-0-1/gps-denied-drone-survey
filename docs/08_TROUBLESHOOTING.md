# 08 — Troubleshooting (one place for everything)

Find your symptom, apply the fix. Each row links to the detailed section if you need more.

---

## Quick triage — where is the problem?

```
Does the pipeline finish without a Python error?
├─ No  → §1 Installation / crashes
└─ Yes → Is the mosaic (orthomosaic.jpg) complete and straight?
         ├─ No  → §2 Stitching
         └─ Yes → Is yellow_mask_debug.jpg showing only the tape?
                  ├─ No  → §3 Field map
                  └─ Yes → Are all targets FOUND?
                           ├─ No  → §4 Matching
                           └─ Yes → Are the coordinates sensible?
                                    ├─ No  → §5 Coordinates
                                    └─ Yes → 🎉
```

**Golden rule:** fix the **earliest** failing stage first. A bad mosaic makes the field map wrong,
which makes every coordinate wrong — tuning the matcher will not save you.

---

## 1. Installation & crashes

| Symptom | Cause | Fix |
|---|---|---|
| `ModuleNotFoundError: cv2 / torch / kornia` | Packages not installed | `pip install -r requirements.txt --break-system-packages` |
| `externally-managed-environment` | Ubuntu 23+ pip protection | Add `--break-system-packages`, or use a venv |
| `pip: command not found` | pip missing | `sudo apt install python3-pip` |
| Stuck at "Loading LoFTR / DINOv2" | Models downloading (first run only) | Needs internet — wait; ~1–2 GB |
| `CUDA out of memory` | GPU busy / too small | Close other GPU apps, or force CPU: `CUDA_VISIBLE_DEVICES="" python3 …` |
| `torch.cuda.is_available()` is `False` | CPU-only build | Install the CUDA build ([00 §4.1](00_INSTALL_EVERYTHING.md)) — CPU still works, just slower |
| `FileNotFoundError: drone_photos` | Data not in place | Copy photos + `coordinates.csv` into `drone_photos/` |
| `Need >=2 images` | Empty folder / wrong path | Check `ls drone_photos/*.jpg` |

---

## 2. Stage 1 — Stitching

| Symptom | Cause | Fix |
|---|---|---|
| `Stitched 15/35 photos` (many dropped) | Weak pairing or low overlap | Confirm `coordinates.csv` has `x_enu,y_enu`; run `--radius 2`; lower `MIN_INLIERS` to 20 |
| Half the arena missing | Match graph disconnected | Check the `[fix#13] spatial pairing` line appears in the log |
| Mosaic skewed / parallelogram | Drift not corrected | Confirm `[fix#12]` and `[fix#13]` lines; use `--radius 2` |
| Ghosting / doubled features | Bad matches accepted | Raise `MIN_INLIERS` (40), lower `RANSAC_THRESH` (3.0) |
| Stitching very slow | Large images / CPU only | Normal on CPU; reduce `MATCH_W/MATCH_H`, or use a GPU |
| `No pairs matched` | Photos don't overlap | Re-fly with more overlap; check the photos aren't blank/blurred |

Details: [07 §Stage 1](07_STAGE_GUIDE.md#stage-1--stitching)

---

## 3. Stage 2 — Field map / yellow boundary

**Always open `results/stage2_field/yellow_mask_debug.jpg` first.** It must show a thin tape frame
and nothing else.

| Symptom | Cause | Fix |
|---|---|---|
| Mask shows large white ground areas | Threshold too low | Raise `YELLOW_S` (75 → 85) in `detect_yellow_corners()` |
| Mask nearly empty / tape missing | Threshold too high | Lower `YELLOW_S` (45 → 30) |
| Base-station crate appears in the mask | Warm/brown colour passing the filter | Raise `YELLOW_S`; the solid-blob rejection usually removes it |
| Corners land inside the arena | Mask polluted → hull pulled in | Clean the mask first (`YELLOW_S`), then re-check `yellow_corners_debug.jpg` |
| `Field: 4.92 m × 3.81 m` (too small) | Size came from VIO, not the known arena | Set `ARENA_LONG_FT` / `ARENA_SHORT_FT`; confirm `[fix#3] TRUE size …` appears |
| Field size looks swapped | Long/short edge auto-choice | It is decided from pixel edges; verify the mosaic isn't cropped on one side |
| Rectified image is a thin sliver | Corner detection failed badly | Fix the mask; if the boundary is partly outside the mosaic, re-fly wider |

Details: [07 §Stage 2](07_STAGE_GUIDE.md#stage-2--field-map-straighten--scale)

---

## 4. Stage 3 — Target matching

Read the per-target line: `peak=0.31 V=0.72`. `peak` = presence strength, `V` = similarity to the seed.

| Symptom | Cause | Fix |
|---|---|---|
| Real target says **NOT FOUND** | Thresholds too strict | Lower `MIN_FOUND_PEAK` (0.10) and/or `VERIFY_MIN` (0.40) |
| Circle on the **wrong** object | Verification too loose | Raise `VERIFY_MIN` (0.55) |
| Target detected half-cut at a photo edge | Edge detection preferred | Raise `CENTER_PREF` (0.10) |
| Two targets matched to the same object | Duplicate assignment | Handled in Stage 4 — check `SEP_M`, look for `[fix#9 WARN]` |
| All targets NOT FOUND | Seeds don't match the arena, or wrong folder | Check `targets/` contains the correct seed images |
| Matching very slow | Many photos + CPU | Normal; lower `TOPK`, or use a GPU |
| Results look stale/unchanged | Old CSV reused | Fix #2 deletes it automatically — confirm `[fix#2]` in the log |

Details: [07 §Stage 3](07_STAGE_GUIDE.md#stage-3--target-matching-feature-detection-)

---

## 5. Stage 4 — Coordinates

| Symptom | Cause | Fix |
|---|---|---|
| Every coordinate shifted by the same amount | Base-station offset wrong | Check `[fix#1 base-origin] base station @ field (x,y)` — should be near takeoff |
| Coordinates in the wrong frame (axes rotated) | Assigned heading not applied | Set `HEADING_ROT_DEG` to 90/180/270 |
| Coordinates jitter between runs | Per-photo mapping error | Raise `AVG_R` (0.7) |
| Two targets reported at one spot | Separation too small | Adjust `SEP_M`; check `[fix#9 WARN]` |
| Coordinates don't match the visual map | Frame mismatch | Confirm `BASE_STATION_EXACT=True`; compare with `annotated_field.jpg` |
| `z` looks wrong | z comes straight from the Pixhawk | Intentional — it is not modified |

Details: [07 §Stage 4](07_STAGE_GUIDE.md#stage-4--coordinates)

---

## 6. Dashboard

| Symptom | Cause | Fix |
|---|---|---|
| Page won't open | Server not running / port busy | `python3 -m base_station.server`; try `BASE_STATION_PORT=8001` |
| Stuck on "Waiting for application startup" | Blocking link connect | Fixed — link now connects in a background thread; restart the server |
| Shows **SIMULATOR** instead of MAVLINK | No heartbeat | Check `MAVLINK_CONN`, then `nc -vz <JETSON-IP> 5760` |
| Battery stuck at 100 % | Simulator mode, or FC sends `-1 %` | Connect MAVLink; voltage-based estimate handles `-1` |
| UI looks broken after an update | Cached CSS/JS | Hard refresh: **Ctrl + Shift + R** |
| Docking panel empty | ESP32 not reaching the dashboard | Check `DOCK_TOKEN` == `IROC_TOKEN`, and `BASE_URL` points at the PC's IP |
| Pipeline "Run" does nothing | Script missing | Confirm `iroc_pipeline_fixed.py` is in the project root |
| 64×64 view is empty | That run hasn't been done | Run **match 64×64 (LR)** first — it creates `results_lr64/` |

---

## 7. MAVLink / telemetry

| Symptom | Cause | Fix |
|---|---|---|
| `nc -vz <ip> 5760` fails | mavlink-router not running | Start it on the Jetson: `mavlink-routerd -c /etc/mavlink-router/main.conf` |
| Router shows no UART | Pixhawk not detected | Check `ls /dev/ttyACM0`, cable, and that the FC is powered |
| QGC works but the dashboard doesn't | QGC connected directly by USB | Route everything through mavlink-router instead |
| Connection drops repeatedly | WiFi / DHCP | Use static IPs; check signal strength |
| Wrong dialect warnings | PX4 vs ArduPilot dialect | PX4 → `MavlinkDialect = common` |

Details: [03 §4](03_DATA_TRANSFER.md#4-telemetry--mavlink-routing)

---

## 8. ESP32 / docking / charging

| Symptom | Cause | Fix |
|---|---|---|
| Voltage reads low and stops rising | ADC clipping above ~2.4 V | Increase the divider ratio (47 k + 6.8 k) and update `DIVIDER_RATIO` |
| Voltage slightly off | Needs calibration | Set `CAL_SLOPE` / `CAL_OFFSET` from two multimeter readings |
| Contact never detected | Wiring / thresholds | Check pad continuity, **common ground**, `HIGH/LOW_THRESHOLD` |
| Motors buzz or stall | Current limit / WiFi sleep | Set A4988 VREF; ensure `WiFi.setSleep(false)`; lower `STEP_HZ` |
| Wires heat up after a pause | Both PWM low with EN high | Already fixed — `chargingEnable()` re-applies direction |
| Docking stops after 30 s | Timeout, no contact | Re-align the platform; check pad heights |
| ESP32 not found by the IDE | USB driver | Install CP210x or CH340 |
| Logs missing on the dashboard | Token / URL mismatch | `DOCK_TOKEN` == `IROC_TOKEN`; check `BASE_URL` |

Details: [02 §9](02_DOCKING_CHARGING.md#9-troubleshooting)

---

## 9. Git / GitHub

| Symptom | Cause | Fix |
|---|---|---|
| `Authentication failed` | Used account password | Use a Personal Access Token |
| `File … exceeds GitHub's 100 MB limit` | Data file committed | `git rm -r --cached <folder>`, add to `.gitignore`, re-commit |
| `Updates were rejected` | Remote is ahead | `git pull --rebase` then `git push` |
| `remote origin already exists` | Remote set twice | `git remote set-url origin <url>` |
| Data folders showing in `git status` | `.gitignore` not applied | `git rm -r --cached .`, fix `.gitignore`, `git add -A` |

Details: [06 §11](06_GIT_GITHUB.md#11-common-problems)

---

## 10. Still stuck? Collect this before asking

1. The **full terminal log** of the run (not just the error line).
2. These images:
   `orthomosaic.jpg`, `yellow_mask_debug.jpg`, `yellow_corners_debug.jpg`, `annotated_field.jpg`
3. `results/stage2_field/calibration.txt` and `results/stage3_targets/targets.json`
4. Your environment: OS, `python3 --version`, `torch.cuda.is_available()`

Those four items are enough to diagnose almost any pipeline problem.
