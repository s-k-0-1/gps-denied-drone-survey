# 04 — Pipeline & Feature Detection

What every file does, and exactly how a target goes from "a 64×64 seed image" to "x = 3.79 m,
y = 1.92 m from the base station".

For the underlying theory (why DINOv2, what bundle adjustment is, etc.) see
[HOW_IT_WORKS.md](../HOW_IT_WORKS.md). For tuning values see
[PARAMETERS_GUIDE.md](../PARAMETERS_GUIDE.md).

---

## 1. File map — what each file is responsible for

### Pipeline

| File | Role | You run it? |
|---|---|---|
| **`iroc_pipeline_fixed.py`** | **Main entry point.** Imports the base pipeline and replaces buggy functions with fixed versions (coordinate frame, stitch pairing, deliverables). All configuration flags live at the top. | ✅ **Yes — always run this one** |
| `iroc_pipeline.py` | The base pipeline: Stage 1 stitching, Stage 2 field map, Stage 4 coordinates + annotation. Left untouched so fixes stay reviewable. | No (imported) |
| `stage3_robust.py` | Stage 3 matcher — DINOv2 semantic search that finds each seed in the drone photos. | No (launched by the pipeline) |
| `fused_search.py` | Shared helpers: model loading (LoFTR, DINOv2), image resizing, seed grouping, LR generation. Also contains the older matcher. | No (imported) |
| `3d.py` | Optional 3D reconstruction through OpenDroneMap (Docker). | Optional |
| `make_lr.py` | Converts full-resolution reference photos into LR seed images. | Optional |
| `make_test_dataset.py` | Builds a synthetic arena + flight with known ground truth, for testing without real data. | Optional |

### Dashboard (`base_station/`)

| File | Role |
|---|---|
| `server.py` | FastAPI app: REST endpoints, WebSocket telemetry, image serving, docking endpoints |
| `config.py` | Every path and environment setting; result-set switching (`results/` ↔ `results_lr64/`) |
| `pipeline_runner.py` | Runs the pipeline as a subprocess and streams its stdout to the browser |
| `results_store.py` | Reads `targets.json` + `fused_results.csv`, watches folders for changes |
| `drone_link/base.py` | Abstract link interface + telemetry/mission-state definitions |
| `drone_link/mavlink_link.py` | Real MAVLink link (pymavlink): telemetry parsing, commands |
| `drone_link/simulator.py` | Replays a recorded flight so the UI works with no hardware |
| `drone_link/__init__.py` | `LinkManager` — hot-swaps links, falls back to the simulator |
| `static/index.html`, `style.css`, `app.js` | The dashboard UI |

### Firmware

| File | Role |
|---|---|
| `esp32_firmware/full_base_station_wifi.ino` | Docking rods, contact/polarity detection, voltage measurement, charging state machine, WiFi log mirror |

---

## 2. Data flow at a glance

```
drone_photos/*.jpg + coordinates.csv          targets/*.png (64×64 seeds)
            │                                              │
            ▼                                              │
 ┌────────────────────────────┐                            │
 │ STAGE 1  run_stitching()   │  iroc_pipeline.py          │
 │ LoFTR → RANSAC → BFS →     │                            │
 │ bundle adjust → blend      │                            │
 └────────────┬───────────────┘                            │
   orthomosaic.jpg + photo_to_H                            │
            ▼                                              │
 ┌────────────────────────────┐                            │
 │ STAGE 2  setup_field_map() │  iroc_pipeline.py + fixed  │
 │ yellow detect → rectify →  │                            │
 │ true 35×25 ft scale        │                            │
 └────────────┬───────────────┘                            │
   rectified_field.jpg + M_persp                           │
            ▼                                              ▼
 ┌──────────────────────────────────────────────────────────┐
 │ STAGE 3  stage3_robust.py                                │
 │ CLAHE → DINOv2 prototypes → heatmap → verify → found/not │
 └────────────┬─────────────────────────────────────────────┘
   fused_results.csv + candidates.json + proof_hd/ + lr_match/
            ▼
 ┌────────────────────────────┐
 │ STAGE 4  compute_map_coords│  iroc_pipeline_fixed.py
 │ pixel → mosaic → rectified │
 │ → metres − base station    │
 └────────────┬───────────────┘
   targets.json + annotated_field.jpg
```

---

## 3. Stage 1 — Stitching

**File:** `iroc_pipeline.py` → `run_stitching()`, with two fixes in `iroc_pipeline_fixed.py`.

**Goal:** turn ~30 overlapping photos into one top-view map, and remember the transform
`photo_to_H[photo] = 3×3 matrix` that maps any photo pixel into mosaic pixels. Stage 4 needs
that transform later.

| Step | Function | What it does |
|---|---|---|
| 1 | `discover()` | Lists photos; reads each one's `row,col`. **Fix #12** takes these from `coordinates.csv` instead of parsing filenames. |
| 2 | `select_pairs()` | Chooses which photo pairs to match. **Fix #13** pairs by VIO **K-nearest-neighbours** (`x_enu,y_enu`) + consecutive frames, instead of a rigid grid. |
| 3 | `LoFTRMatcher.match()` | Deep feature matching between a pair → point correspondences → `estimateAffinePartial2D` with RANSAC → transform + inlier count. |
| 4 | `bfs_align()` | Spreads transforms from an anchor photo across the match graph. |
| 5 | `global_optimize()` | Bundle adjustment — minimises total reprojection error, removing accumulated drift. |
| 6 | `detect_loop_closures()` | Adds matches between non-consecutive photos that overlap, then re-optimises. |
| 7 | `compute_exposure_gains()` + `warp_and_blend()` | Brightness-matches and blends all photos into the final mosaic. |

**Why fix #13 matters:** the old grid pairing only worked for a perfect lawnmower survey. If the
drone started in the centre, or spiralled, many photos had no grid neighbour, the graph
disconnected, and half the arena went missing. VIO-based nearest-neighbour pairing works for
**any** flight pattern.

**Outputs**
```
results/stage1_stitch/orthomosaic.jpg      ← the map
results/stage1_stitch/photo_transforms.npz ← photo_to_H
results/stage1_stitch/stitch_log.txt       ← per-pair inlier counts
```

**Health check in the log:** `Stitched N/N photos` — N should equal the photo count. If many are
dropped, see the tuning table in the Parameters Guide (`GRID_RADIUS`, `MIN_INLIERS`).

---

## 4. Stage 2 — Field map (rectification & scale)

**Files:** `iroc_pipeline.py` → `setup_field_map()`, overridden in `iroc_pipeline_fixed.py`
(`detect_yellow_corners`, scale fix).

**Goal:** the mosaic is tilted and in arbitrary pixel units. Convert it into a straight rectangle
whose size in metres is known.

| Step | What happens |
|---|---|
| 1 | **Yellow mask** — convert to Lab; yellow tape has a large positive `b*−a*`. A saturation threshold (`YELLOW_S`) separates bright tape from dull ground. An Otsu-based adaptive path handles unusual lighting. |
| 2 | **Clean-up** — morphological open/close; connected components; blobs that are *solid* (high fill ratio) are rejected because real tape is thin. |
| 3 | **Corners** — convex hull of the remaining points → `approxPolyDP` → 4 corners (fallback: `minAreaRect`). |
| 4 | **Base-station roll** — the corner nearest the VIO origin becomes bottom-left, so the frame is consistent. |
| 5 | **Rectify** — `getPerspectiveTransform` maps those 4 corners onto a true rectangle → `M_persp`; the mosaic is warped straight. |
| 6 | **True scale (fix #3)** — the arena size is known (`ARENA_LONG_FT` × `ARENA_SHORT_FT`). The longer *pixel* edge is assigned the longer real edge, so VIO scale error is corrected. |

**Outputs**
```
results/stage2_field/rectified_field.jpg      ← straight, true-scale arena
results/stage2_field/yellow_mask_debug.jpg    ← what the detector saw   ← check this first
results/stage2_field/yellow_corners_debug.jpg ← detected corners on the mosaic
results/stage2_field/calibration.txt          ← field size, origin, corner pixels
```

**Debugging rule:** if coordinates look wrong, open `yellow_mask_debug.jpg`. It should show a thin
tape frame and nothing else. Ground or the base-station crate leaking into the mask drags the
corners and therefore every coordinate — raise `YELLOW_S`.

---

## 5. Stage 3 — Feature detection ⭐

**File:** `stage3_robust.py` (launched as a subprocess by `run_target_finding()`).

**Goal:** for each seed image, decide *whether* it is present, *in which photo*, and *where in
that photo*.

### 5.1 Inputs

| Input | Size | Note |
|---|---|---|
| Seed / reference | **64×64** | provided by organizers (`targets/`) |
| Drone LR photos | **128×128** | auto-generated into `drone_photos_lr/` |

Both are internally resized to 640×480, then to 448×448 for DINOv2 (a 32×32 patch grid). The
comparison happens in **feature space**, so the different source sizes are fine. HD (1280×720) is
**not** used for matching — only for stitching and the final HD proof crop.

### 5.2 The algorithm, step by step

| # | Function | What it does |
|---|---|---|
| 1 | `clahe_norm()` | CLAHE on the Lab L-channel — removes lighting/shadow differences between seed and photos. |
| 2 | `ref_prototypes()` | From the seed's DINOv2 patches: **background prototype** = average of border patches; **object prototype** = average of the patches least similar to the background. Averaging makes it rotation-robust. |
| 3 | `drone_heat()` | For every drone photo, each patch scores `(patch·object) − (patch·background)`. The result is a heatmap; its peak is the candidate location. |
| 4 | `peak_and_center()` | Extracts the peak value, a weighted centroid of high-response patches, and a radius. |
| 5 | *(selection)* | Photos are ranked by peak (plus a small centre preference) and the top `TOPK` are shortlisted. |
| 6 | `localize_verified()` | For each candidate, crop it, embed it with DINOv2, and take the **maximum cosine similarity over 4 rotations** against the seed crop → `vsim`. A colour histogram comparison is folded into ranking so, e.g., a white box is not confused with a grey rock. |
| 7 | *(decision)* | **FOUND** only if `peak ≥ MIN_FOUND_PEAK` **and** `vsim ≥ VERIFY_MIN`. Otherwise **NOT FOUND** — the system never guesses. |
| 8 | `AUTO_CALIBRATE` | The peak threshold is lowered (never raised) based on background response, so faint targets in a dark arena are still caught. |

### 5.3 Why this method

Template matching and geometric matchers (SIFT/LoFTR) need the same appearance and geometry. A
target seen from a different height, angle and lighting breaks them, and the arena floor is
low-texture. DINOv2 embeddings are **semantic** — they encode "what this looks like", so the same
object still matches after rotation, scale and lighting changes.

### 5.4 Outputs

```
results/stage3_targets/fused_results.csv   ← per target: photo, pixel, peak, vsim, confidence
results/stage3_targets/candidates.json     ← top-6 alternates per target (used by Stage 4)
results/stage3_targets/proof_hd/<t>.jpg    ← HD deliverable: sharp, feature-centred, ≥720 px
results/stage3_targets/lr_match/<t>.png    ← LR deliverable: 128×128 crop
results/stage3_targets/visuals/<t>.jpg     ← reference | detection, side by side
```

The terminal prints `peak=… V=…` per target — these two numbers are what you tune against.

---

## 6. Stage 4 — Coordinates

**File:** `iroc_pipeline_fixed.py` → `compute_map_coords()` and `_mutual_exclusion()`.

**Goal:** convert a detection's pixel position into metres relative to the base station.

### 6.1 The transform chain

```
drone photo pixel
   │  photo_to_H          (Stage 1)
   ▼
mosaic pixel
   │  M_persp             (Stage 2)
   ▼
rectified pixel
   │  ÷ PX_PER_M
   ▼
metres in the arena frame
   │  − base-station offset
   ▼
FINAL (x, y) relative to the base station
```

### 6.2 Base-station origin (fix #1)

VIO `(0,0)` is the takeoff point — the base station. The affine `A` (calibrated by SIFT between
photos and the mosaic) projects that point into the rectified frame, giving the base station's
field position. Subtracting it from every target makes all coordinates base-station-relative, as
the rulebook requires. If `A` is unavailable the code falls back to the yellow corner instead of
producing nonsense.

`HEADING_ROT_DEG` optionally rotates all coordinates so the axes align with an assigned initial
heading (0/90/180/270).

### 6.3 Cleaning up the results

| Mechanism | Purpose |
|---|---|
| **Mutual exclusion** | Two targets cannot occupy the same spot. Targets are assigned greedily by peak strength; a loser takes its next-best candidate from `candidates.json`, and its proof images are regenerated. |
| **Multi-photo averaging** (`AVG_R`) | If a target is visible in several photos, its positions are averaged — this removes per-photo mapping error. |
| **Duplicate warning** (`DUP_M`) | Logs a warning if two final targets are still suspiciously close. |

### 6.4 Outputs

```
results/stage3_targets/targets.json          ← final coordinates per target (map_xyz)
results/stage4_annotated/annotated_field.jpg ← circles + labels on the rectified arena
```

Terminal summary:

```
Target      Method          x(m)     y(m)     z(m)  Photo
1           stage3_robust   3.790    1.920    3.000  cp0011_r01c02.jpg
```

---

## 7. Stage 5 — 3D reconstruction (optional)

**File:** `3d.py` — wraps OpenDroneMap in Docker.

| Step | What happens |
|---|---|
| 1 | SIFT features on every HD photo (`--min-num-features 16000`) |
| 2 | Match features across all pairs (`--matcher-neighbors 0`) |
| 3 | **SfM** — camera poses + sparse point cloud (bundle adjustment) |
| 4 | **MVS** — dense point cloud (`--pc-quality high`) |
| 5 | Meshing + texturing → textured model |
| 6 | **DSM** (elevation grid) + **orthophoto** (true-scale colour map) |

`3d.py` then exports `model.glb`, fuses orthophoto colour with DSM height into a coloured point
cloud, and writes a side-by-side preview.

```bash
python3 iroc_pipeline_fixed.py --run-3d     # pipeline + 3D
python3 3d.py                               # 3D only (needs Docker)
python3 3d.py --skip-odm --view             # re-render from existing ODM output
```

Outputs → `results/3d_map/`.

---

## 8. Running it

```bash
python3 iroc_pipeline_fixed.py                 # full run
python3 iroc_pipeline_fixed.py --skip-stitch   # reuse mosaic, re-run matching (fast)
python3 iroc_pipeline_fixed.py --skip-match    # reuse matches, recompute coordinates (fastest)
python3 iroc_pipeline_fixed.py --radius 2      # more stitch pairs (low overlap)
python3 iroc_pipeline_fixed.py --run-3d        # + 3D reconstruction
```

Alternate matching resolution (see the Parameters Guide):

```bash
MATCH_LR=64 python3 iroc_pipeline_fixed.py --skip-stitch   # 64×64 mode → results_lr64/
```

### Log lines that confirm each stage worked

```
[fix#12] stitch grid: 35/35 photos … (row,col) mila     ← Stage 1 grid from CSV
[fix#13] spatial pairing: 35/35 photos, 140 pairs       ← Stage 1 VIO pairing
Stitched 35/35 photos                                    ← Stage 1 complete
[fix#8] fixed yellow OK (frac=0.043)                     ← Stage 2 yellow mask
[fix#3] TRUE size 10.67 x 7.62 m                         ← Stage 2 metric scale
peak=0.31 V=0.72  → FOUND                                ← Stage 3 per target
[fix#1 base-origin] base station @ field (0.72, 0.73) m  ← Stage 4 origin
Found 5/5 targets                                         ← done
```

---

## 9. Where to change things

| You want to… | Go to |
|---|---|
| Change arena size | `ARENA_LONG_FT` / `ARENA_SHORT_FT` in `iroc_pipeline_fixed.py` |
| Fix a bad yellow mask | `YELLOW_S` inside `detect_yellow_corners()` in `iroc_pipeline_fixed.py` |
| Make detection stricter/looser | `MIN_FOUND_PEAK`, `VERIFY_MIN` in `stage3_robust.py` |
| Improve a broken stitch | `GRID_RADIUS`, `MIN_INLIERS` in `iroc_pipeline.py`, or `--radius 2` |
| Switch coordinate origin | `BASE_STATION_EXACT`, `HEADING_ROT_DEG` in `iroc_pipeline_fixed.py` |
| Change seed size | `LR_SIZE` in `make_lr.py` |

Full explanations: [PARAMETERS_GUIDE.md](../PARAMETERS_GUIDE.md).

---

**Next:** [05 — Setup](05_SETUP.md)
