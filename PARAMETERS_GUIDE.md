# Parameters Guide (`iroc_pipeline_fixed.py`)

This guide lists **all tunable parameters** stage-wise: what they do, **when to change** them (which error), and **how** (increase/decrease). To run, always:
```bash
python3 iroc_pipeline_fixed.py            # full: stitch + field + match + annotate
```

Parameters live in **4 places**:
- **A. `iroc_pipeline_fixed.py`** (top) — coordinate frame, dedup, averaging, stitch-fix flags
- **B. `stage3_robust.py`** (top) — matching / detection thresholds
- **C. `iroc_pipeline.py`** (top, CONFIG) — stitching, field scale, camera
- **D. `make_lr.py`** — seed reference LR size

---

## 0. Run flags (command line)

| Flag | What it does | When to use |
|---|---|---|
| *(none)* | Full run: stitch + field + match + annotate | New data / first time |
| `--skip-stitch` | Reuse mosaic, **fresh matching** | Only testing matching/coords (stitch already good) |
| `--skip-match` | Reuse everything, only recompute coords | Only a coordinate/field tweak (fastest) |
| `--radius 2` | More neighbour pairs in stitch (8-NN → 12-NN) | Low overlap / weak stitch (slower) |
| `--run-3d` | 3D reconstruction (ODM, Docker) | When you want a 3D map |

> **Note:** to improve the stitch, do **NOT** use `--skip-stitch` (it skips Stage 1).

---

## 1. STAGE 1 — Stitching (`iroc_pipeline.py` CONFIG + fixed's fix#12/#13)

Joins photos into an orthomosaic (LoFTR + bundle adjustment). **Pairing now uses VIO position** (fix#13) — robust on center/corner/spiral/lawnmower patterns.

### 1a. Stitch fixes (`iroc_pipeline_fixed.py` — automatic, not a param)
| Fix | What it does | Requirement |
|---|---|---|
| **fix#12** | `discover()` now takes the correct `row,col` from **coordinates.csv** instead of the filename regex (for anchor/grid) | `row,col` columns in `coordinates.csv` |
| **fix#13** | `select_pairs()` now pairs by **VIO `x_enu,y_enu` K-nearest-neighbours + temporal** — the graph never disconnects | `x_enu,y_enu` in `coordinates.csv` |
| K neighbours | radius 1 → **8-NN**, radius 2 → **12-NN** | increased by `--radius 2` |

**Error → fix:** *Half/corner mosaic (on center-start)* or *photos DROP (Stitched 15/23)* → fixed automatically by fix#12/#13. The log should show `[fix#12]…` and `[fix#13] spatial pairing: N/N photos …`. If VIO is missing, it falls back to the old grid pairing.

### 1b. Stitch tuning (`iroc_pipeline.py` CONFIG)
| Parameter | Default | What it does | When / how to change |
|---|---|---|---|
| `GRID_RADIUS` | `1` | K-NN neighbours (fix#13): 1→8, 2→12 | **Gaps** in stitch or very low overlap → `2`. Or `--radius 2` |
| `MIN_INLIERS` | `30` | Min inliers to call two photos "matched" | Low-overlap photos **skipped** → **decrease** (20). Wrong pairs joining → **increase** (40) |
| `MIN_INLIER_RATIO` | `0.20` | Inlier/matches ratio | Too many pairs rejected → decrease (0.15) |
| `RANSAC_THRESH` | `5.0` | Homography RANSAC pixel tolerance | Distorted stitch → decrease (3.0); few matches → increase (7.0) |
| `SCALE_LO, SCALE_HI` | `0.40, 2.50` | Allowed scale between photos | If altitude varies a lot → widen the range |
| `MATCH_W, MATCH_H` | `960, 544` | LoFTR input size | More accurate (slower) → increase |

**Error → fix:** *low X in "BFS aligned X/N"* or *broken mosaic* → first check the `[fix#13]` log (is VIO available?); then `--radius 2` + decrease `MIN_INLIERS`.

---

## 2. STAGE 2 — Field map + Yellow boundary

### 2a. Arena size / scale
| Parameter | File | Default | What it does | When / how |
|---|---|---|---|---|
| `ARENA_LONG_FT` | fixed | `30.0` | Arena's **long** edge (ft) | Different arena size → set here |
| `ARENA_SHORT_FT` | fixed | `25.0` | Arena's **short** edge (ft) | ″ (longer pixel edge auto = LONG) |
| `PX_PER_M` | base | `150` | Rectified image resolution (px/m) | Sharper map (slower) → 200; lighter → 100 |

**Error → fix:** *wrong field size (e.g. 9.14×9.66)* → set `ARENA_LONG_FT/SHORT_FT` to your real arena. Which side is the long one is decided **automatically** (from mosaic pixel edges).

### 2b. Yellow detection (`iroc_pipeline_fixed.py` → `detect_yellow_corners`)
| Item | Default | What it does | When / how |
|---|---|---|---|
| HSV hue gate | `14–48` | Yellow hue range | Different tape shade → adjust range |
| `s >= 28` | 28 | Min saturation | Faded tape not found → decrease (20) |
| `diff (b*-a*) >= 15` | 15 | Yellow-ness (Lab) | Faded tape → decrease |
| **fill `>= 0.60` reject** | 0.60 | Reject **solid blobs** (features), keep thin tape | Yellow feature getting rejected → increase (0.7); tape rejected → increase |
| Auto (Otsu) fallback | — | Auto-threshold if the fixed one fails | Automatic (adapts to lighting) |

**Error → fix:** *wrong yellow line / corners off* → inspect `stage2_field/yellow_mask_debug.jpg`. Extra blobs → fill threshold; incomplete tape → decrease `s`/`diff`.

### 2c. Axis convention
| Parameter | File | Default | When / how |
|---|---|---|---|
| `SWAP_AXES` | fixed | `False` | If **x and y are swapped** in coords (vs ground-truth) → `True` |

---

## 3. STAGE 3 — Matching / Detection (`stage3_robust.py`) ⭐ *most tuning happens here*

### 3a. Found / Not-found thresholds
| Parameter | Default | What it does | When / how |
|---|---|---|---|
| `MIN_FOUND_PEAK` | `0.14` | Object-heatmap peak floor (found gate) | Real target **NOT FOUND** → decrease (0.10). Random match → increase |
| `VERIFY_MIN` | `0.45` | Verification (crop-sim) floor | Real target NOT FOUND (low verify) → decrease (0.40). Wrong match passing → increase (0.55) |
| `AUTO_CALIBRATE` | `True` | Auto-set peak threshold from background (lenient direction only) | Self-adjusts to new arena/lighting. Want it fixed → `False` |
| `PEAK_RATIO` | `1.5` | Auto-cal: peak ≥ background×ratio | Misses in a dark arena → decrease (1.2) |
| `PEAK_FLOOR` | `0.06` | Auto-cal absolute floor (noise reject) | — |

### 3b. Selection / localization
| Parameter | Default | What it does | When / how |
|---|---|---|---|
| `TOPK` | `8` | How many photos to shortlist (by peak) | Want more candidates → increase (slower) |
| `CENTER_PREF` | `0.06` | Prefer detections near the photo center | Edge/half-cut target appearing → increase (0.10) |
| `ROTATIONS` | `(0,1,2,3)` | 0/90/180/270 rotation in verification | Rotation-invariance (leave it) |
| `MIN_SEP_M` | `0.6` | Matcher-level min target separation | — (the pipeline-level `SEP_M` is the main one) |
| **Color weight** | `0.4 + 0.6×cs` | DINOv2 + colour ranking (distinct objects) | Colour causing errors → lower the weight; distinct-colour features → trust colour more |

### 3c. Camera (for gpos / mutual exclusion)
| Parameter | Default | When / how |
|---|---|---|
| `DRONE_HEIGHT` | `3.0` | Different flight altitude → set |
| `FOV_H_DEG, FOV_V_DEG` | `90, 65` | Different camera FOV → set |

**Common Stage-3 errors:**
- *Circle on the wrong object* → increase `VERIFY_MIN`, or increase colour weight.
- *Real target NOT FOUND* → decrease `MIN_FOUND_PEAK` / `VERIFY_MIN`.
- *Edge/half-cut* → increase `CENTER_PREF`.
- Each target's `peak=.. V..` is printed in the terminal — tune from that.

### 3d. Output images (per target)
| Item | Default | File | When / how |
|---|---|---|---|
| **LR image** | `128×128` | `stage3_targets/lr_match/<t>.png` | Tight feature crop like the seed. Change size → `(128,128)` in matcher/`_regen` |
| **HD proof** | native, **shorter side ≥720** | `stage3_targets/proof_hd/<t>.jpg` | SHARP native crop. Change `720` → in `sc/bh/bw` |
| HD tight crop size | `half = max(hr*2.2, hw*0.06)` | — | More/less background in HD → adjust `2.2` / `0.06` |

### 3e. Match resolution — LR-to-LR + `MATCH_LR` mode (`stage3_robust.py`)
**Seed 64×64 ↔ drone 128×128** (HD down-sampled). Matched via DINOv2 — different sizes are fine. **HD 720 is NOT in matching** (only stitch + proof).

**`MATCH_LR` env var — choose the drone's LR size (an option, no code change):**

| Command | Drone LR | Compare | When |
|---|---|---|---|
| `python3 iroc_pipeline_fixed.py --skip-stitch` | 128 | **seed-64 ↔ drone-128** | **DEFAULT — best, use this** ✅ |
| `MATCH_LR=64 python3 iroc_pipeline_fixed.py --skip-stitch` | 64 | **seed-64 ↔ drone-64** | If **64×64 matching** is required (organizer/round) 🔧 |
| `MATCH_LR=128 python3 iroc_pipeline_fixed.py --skip-stitch` | 128 | seed-64 ↔ drone-128 | = default |

**Chain (default):** seed `64→640→448`, drone `128→640→448` (DINOv2 32×32 patches).

> **The `MATCH_LR=64` option is KEPT** — 128 is best right now (perfect test), but if 64×64 is ever required just add `MATCH_LR=64` (no code change). Localization is a bit coarser then. Default = seed-64 ↔ drone-128.
>
> **Output folder:** when `MATCH_LR` is set, that run's result is also copied to **`results_lr<N>/`** (e.g. `results_lr64/` — `stage3_targets` + `stage4_annotated`), so the default (128) `results/` and the 64×64 both stay **side by side**. **To keep both, run in this order:** first `MATCH_LR=64 …` (→ `results_lr64/`), then default `…` (→ `results/`).

---

## 4. STAGE 4 — Coordinates, Frame, Dedup, Averaging (`iroc_pipeline_fixed.py`)

### 4a. Coordinate frame ⭐
| Parameter | Default | What it does | When / how |
|---|---|---|---|
| **`BASE_STATION_EXACT`** | `True` | Origin (0,0) = **actual base station** (VIO takeoff), coords relative to it | Keep `True` when the base station can be anywhere. Use `False` to place the origin at the yellow corner instead |

**Error → fix:** *coords shifted by a constant / wrong frame* → check the `[fix#1 base-origin] base station @ field (x,y)` log; the base station should be near takeoff. If it looks wrong → `BASE_STATION_EXACT=False` (yellow corner). If `A` (SIFT calib) fails it auto-falls-back to the yellow corner.

### 4b. Dedup / averaging
| Parameter | Default | What it does | When / how |
|---|---|---|---|
| `SEP_M` | `0.6` | Two targets closer than this → conflict (mutual exclusion) | Targets genuinely < 0.6m apart → decrease. Duplicate remaining → increase |
| `AVG_R` | `0.5` | **Average** same-object candidates within this radius (accuracy) | Turn off averaging → `0`. More aggressive → increase |
| `DUP_M` | `0.5` | **Warning** if targets are closer than this | Warning threshold only |
| `MAX_INSTANCES` | `1` | Max instances per feature-type | Final round targets are **unique** → `1`. Want multiple instances → `2/3` |
| `EXTRA_VERIFY_MIN` | `0.60` | Strict verify for an extra instance | Only applies when `MAX_INSTANCES>1`; false extras → increase |

**Error → fix:**
- *Two targets at one spot (overlap)* → check `[fix#9 WARN]`; fix `SEP_M`.
- *Slight coord jitter* → increase `AVG_R` (average more photos).
- *Wrong z* → z comes from the Pixhawk (untouched, per your request).

---

## 5. Seed references (`make_lr.py`)
| Parameter | Default | What it does | When / how |
|---|---|---|---|
| `LR_SIZE` | `(64, 64)` | Seed reference size | Drone LR is separately 128, so the comparison is seed-64 ↔ drone-128 |
| `SRC_DIR` | `reference/` | Input full-res references | — |
| `OUT_DIR` | `targets/` | Output LR seeds (matcher reads from here) | — |

---

## 6. Quick "error → parameter" table

| Error | Parameter | Direction |
|---|---|---|
| Half/corner mosaic, photos DROP | fix#12/#13 (auto), `--radius 2`, `MIN_INLIERS` | log check / 2 / decrease |
| Broken stitch / gaps | `GRID_RADIUS` / `--radius 2`, `MIN_INLIERS` | 2 / decrease |
| Wrong field size | `ARENA_LONG_FT`, `ARENA_SHORT_FT` | correct value |
| Yellow line off | `detect_yellow_corners`: fill, `s`, `diff` | adjust from debug |
| x/y swapped | `SWAP_AXES` | `True` |
| Coords not base-station / shifted | `BASE_STATION_EXACT` | `True` (Final) / `False` (corner) |
| Real target NOT FOUND | `MIN_FOUND_PEAK`, `VERIFY_MIN` | decrease |
| Wrong/random match | `VERIFY_MIN`, colour weight | increase |
| Edge/half-cut target | `CENTER_PREF` | increase |
| Two targets at one spot | `SEP_M` | adjust |
| Coord jitter | `AVG_R` | increase |
| Want multiple instances | `MAX_INSTANCES` | `2/3` |
| Too much background in HD | HD `half` (`2.2`/`0.06`) | decrease |
| New arena/lighting | `AUTO_CALIBRATE=True` (already) | — |

---

## 7. How to tune (workflow)
1. Full run: `python3 iroc_pipeline_fixed.py`
2. Read the terminal log: `[fix#12]/[fix#13]` (stitch), `[fix#1 base-origin]`, `[fix#3]` (field size), each target's `peak / V` (Stage 3). Plus `annotated_field.jpg`, `yellow_mask_debug.jpg`, `orthomosaic.jpg`.
3. Change the relevant parameter from the tables above, one at a time.
4. **Only coords/field tweak** → `--skip-match` (fastest). **Matching tweak** → `--skip-stitch`. **Stitch tweak** → full run (no skip-stitch).
5. Change **one** parameter at a time and verify.

> Locations: `iroc_pipeline_fixed.py` params are at the top (around STAGE3 / BASE_STATION_EXACT). `stage3_robust.py` top "CONFIG" block. `iroc_pipeline.py` top "CONFIG" (# Stitching / # Field map). `make_lr.py` top.
