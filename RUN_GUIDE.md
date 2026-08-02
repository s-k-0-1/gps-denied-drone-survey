# RUN & VALIDATION GUIDE — ASCEND Pipeline

How to **run** the pipeline and how to **verify** it worked. Do a full run once and tick off the
checklist below.

---

## 1. Check your inputs first

| Needed | Where | Note |
|---|---|---|
| HD survey photos | `drone_photos/*.jpg` (1280×720) | Captured by ASCEND |
| Position log | `drone_photos/coordinates.csv` | Must contain `x_enu, y_enu, z_enu, yaw, row, col, image_file` — stitching fixes #12/#13 depend on these |
| Seed references | `targets/*.png` (64×64) | Provided by the organizers in the final round; for practice run `python3 make_lr.py` (from `reference/`) |

> **Battery voltage (11.3.8d)** comes from the ESP32 straight to the dashboard — nothing to do in
> the pipeline.

---

## 2. Run it

```bash
cd ~/advanced_matcher

# FULL run (new data): stitch + field + match + coordinates + annotate
python3 iroc_pipeline_fixed.py

# Matching / coordinates only (mosaic is already good)
python3 iroc_pipeline_fixed.py --skip-stitch

# + 3D map (needs Docker)
python3 iroc_pipeline_fixed.py --run-3d
```

---

## 3. Look for these log lines (✓ = healthy)

| Log line | Meaning |
|---|---|
| `[fix#12] stitch grid: N/N photos … (row,col)` | Grid read correctly from `coordinates.csv` ✓ |
| `[fix#13] spatial pairing: N/N photos, XX pairs (VIO …)` | VIO nearest-neighbour pairing active ✓ |
| `Stitched N/N photos` | **N should equal your photo count** (few drops) ✓ |
| `[fix#3] TRUE size 10.67 x 7.62 m` | Metric size = `ARENA_LONG_FT` × `ARENA_SHORT_FT` (35×25 ft) ✓ |
| `[fix#1 base-origin] base station @ field (x,y)` | Base-station origin applied ✓ |
| `[fix#14] coords rotated …°` | Only when `HEADING_ROT_DEG` is set |
| `peak=… V=…` per target + `Found N/M targets` | Matching result ✓ |

**If a Python error / traceback appears**, note the line — it identifies the failing stage
immediately.

---

## 4. Check the output files

| File | What to look for |
|---|---|
| `results/stage1_stitch/orthomosaic.jpg` | Whole arena, straight, no ghosting or gaps |
| `results/stage2_field/rectified_field.jpg` | Straight rectangle, boundary aligned |
| `results/stage2_field/yellow_mask_debug.jpg` | **Only the tape** — no ground, no crate |
| `results/stage3_targets/targets.json` | `map_xyz` coordinates per target |
| `results/stage3_targets/proof_hd/<t>.jpg` | Sharp HD crop, feature centred (≥720 px) |
| `results/stage3_targets/lr_match/<t>.png` | LR crop (128×128) matching the seed |
| `results/stage4_annotated/annotated_field.jpg` | Targets circled in the right places with coordinates |

---

## 5. Configuration flags

Top of `iroc_pipeline_fixed.py`:

| Flag | Default | Change when |
|---|---|---|
| `ARENA_LONG_FT` / `ARENA_SHORT_FT` | `35` / `25` | **Set to your real arena size** |
| `BASE_STATION_EXACT` | `True` | Final round → `True` (origin = base station). Yellow-corner origin → `False` |
| `HEADING_ROT_DEG` | `0.0` | Coordinates must follow an assigned heading (0/90/180/270) |
| `YELLOW_S` (in `detect_yellow_corners`) | `65` | Mask catching ground → raise; tape disappearing → lower |
| `MATCH_LR` (env var) | unset | `MATCH_LR=64 …` → 64×64 mode, result copied to `results_lr64/` |

---

## 6. Common issue → quick fix

| Symptom | Do this |
|---|---|
| Half / corner mosaic, photos dropped | Check the `[fix#13]` line; try `--radius 2` |
| Yellow boundary off | Inspect `yellow_mask_debug.jpg`; tune `YELLOW_S` |
| Coordinates shifted / wrong frame | Check `[fix#1 base-origin]`; verify `BASE_STATION_EXACT` |
| Coordinates need rotating | Set `HEADING_ROT_DEG` |
| Target NOT FOUND or wrong match | Adjust `MIN_FOUND_PEAK` / `VERIFY_MIN` in `stage3_robust.py` |
| Python crash | Read the traceback; see the troubleshooting doc |

> Detailed tuning: **[PARAMETERS_GUIDE.md](PARAMETERS_GUIDE.md)** ·
> Theory: **[HOW_IT_WORKS.md](HOW_IT_WORKS.md)** ·
> All symptoms: **[docs/08_TROUBLESHOOTING.md](docs/08_TROUBLESHOOTING.md)**

---

## 7. Running from the dashboard (optional)

```bash
pip install -r base_station/requirements.txt --break-system-packages   # once
python3 -m base_station.server
```

Open **<http://localhost:8000>** (login `luma` / `ascend2026`) → pick a job in the pipeline
dropdown → **Run**. The log streams into the browser and the panels refresh automatically.

---

### Do one full run, confirm the log lines in §3 and the outputs in §4 — then the pipeline is validated. ✅
