# How It Works

**GPS-denied aerial survey → feature localization**

A complete explanation of how the system works: what each stage does, how the algorithms work, and
which image is compared against which at what resolution. Read this if you are using or modifying
the code.

---

## 0. ⭐ First — SEED, DRONE-LR, HD (clearing the confusion)

Three different images, three jobs:

| Image | Size | Source | Job |
|---|---|---|---|
| **Seed / reference** | **64×64** | Supplied as input — a small crop of the object to find | matching reference |
| **Drone LR** | **128×128** | Down-sampled automatically from each HD `1280×720` photo | searched during matching |
| **HD photo** | **1280×720** | Onboard camera | stitching + final HD proof + 3D |

**Match (Stage 3) = seed-64 ↔ drone-128** — via DINOv2 features (different sizes are fine). **HD (720) is NOT used in matching.**

---

## 1. Problem (one line)

The drone autonomously surveys an arena, logging HD photos + position; from those photos we **find the target objects** and report their **coordinates** — **without GPS**. Position comes from **camera + Pixhawk VIO** (visual-inertial odometry / optical flow).

**Input:** `drone_photos/` (HD + `coordinates.csv`) + `targets/` (seed 64×64).
**Output (per target):** LR image, HD image, base-station-relative `(x, y, z)`. Plus an optional 3D map.

---

## 2. The low-resolution matching workflow

Matching happens between two small images, not between full-resolution ones:

1. **Reference / seed (prepared beforehand):** a small crop, 64×64, of the object to find. Either
   captured directly or down-sampled from a full-resolution photo (`make_lr.py` does this).
2. **After the survey:** each HD photo (`1280×720`) is **down-sampled to 128×128**, and the seed is
   searched for inside it.

> In code: `build_drone_lr()` creates `drone_photos_lr/` from the HD photos, then DINOv2 matches
> the 64×64 seed against those 128×128 images.

**Why work at low resolution:** the search runs over every photo, and the feature extractor has a
fixed input size anyway — so full HD adds cost without adding useful signal. Matching in feature
space also means the seed and the survey image do not need the same dimensions.

---

## 3. Full flow

```
 drone_photos (HD) + coordinates.csv (VIO)              targets/ (seed 64×64)
        │                                                        │
   [STAGE 1] STITCH  (LoFTR + bundle adjustment) → orthomosaic   │
        │                                                        │
   [STAGE 2] FIELD MAP  (yellow detect → perspective rectify)    │
        │                                                        │
   [STAGE 3] TARGET MATCH  (seed-64 ↔ drone-128, DINOv2) ◄───────┘
        │
   [STAGE 4] COORDINATES  (pixel → mosaic → rectified → metres, base-station origin)
        │
   [STAGE 5] 3D MAP  (3d.py: OpenDroneMap photogrammetry — optional)
```

---

## 4. Stage-by-stage FULL THEORY

### 🔹 STAGE 1 — Stitching (`iroc_pipeline.py`)

**Goal:** join the overlapping drone photos into a single **top-view orthomosaic**.

**Theory + steps:**

1. **Pair selection (VIO KNN):** each photo has its `x,y` (VIO) in `coordinates.csv`. We pair each photo with its **spatially-nearest** photos (K-nearest-neighbours). *Why:* nearby photos are the ones that **overlap**, so only those will match. (This works for center-start, corner-start, spiral — any pattern.)

2. **Feature matching — LoFTR:** LoFTR is a **detector-free deep matcher** (Transformer-based). Classic matchers (SIFT) first detect "keypoints" then match — they fail on low-texture (gravel, plain ground). LoFTR extracts **dense features over the whole image** and uses **attention** to directly output **corresponding points** between two photos — strong even on low-texture. Output: hundreds of matching point-pairs.

3. **Transform estimation (RANSAC + homography):** from the matches it computes a **homography/affine matrix** `H` telling how to place photo B onto photo A. **RANSAC** removes outliers (wrong matches) and fits only on **inliers** (hence the "inliers" count in the log).

4. **Graph + BFS align:** all photos form a **graph** (node = photo, edge = match). The **anchor** = the center photo. **BFS** brings every photo's transform into the anchor's frame.

5. **Bundle adjustment (global optimize):** small BFS errors **compound** (drift). Bundle adjustment optimizes all transforms **together**, minimizing the total **reprojection error** (mismatch of matched points). → eliminates drift/skew.

6. **Loop closure:** finds extra matches between far-apart photos (that meet again in the grid) and adds them to the graph → more constraints → a straighter mosaic.

7. **Warp + exposure blend:** warps each photo onto the canvas, matches brightness with **exposure gains**, and **blends** seams smoothly → clean mosaic. Cropped to coverage.

**Comparison here:** **drone-HD ↔ drone-HD** (grayscale 960×544), **GEOMETRIC** (points).

---

### 🔹 STAGE 2 — Field map + Yellow boundary (`iroc_pipeline.py` + fixed)

**Goal:** turn the skewed mosaic into a **straight rectangle** + set the **pixel→metre** scale.

**Theory + steps:**

1. **Yellow detection (Lab colour):** RGB is lighting-sensitive. In **Lab** colour space, `b*` = the blue↔yellow axis. Yellow tape has a **large positive** `b*−a*` → mask on that. If lighting changes, **Otsu** (auto-threshold) adapts. Keep thin tape, reject **solid yellow blobs** (features) via fill-ratio.

2. **Finding corners:** **connected components** of the mask → the largest = the tape frame. **Convex hull** → `approxPolyDP` (polygon approximation) → **4 corner points**.

3. **Perspective rectification (homography):** compute the **perspective transform `M_persp`** (`getPerspectiveTransform`) that maps the 4 detected corners onto the 4 corners of a **true rectangle** (the known arena size — ours is 35×25 ft). Warping the mosaic by this gives a top-down straight image (camera tilt/perspective removed).

4. **Metric scale:** the arena size is **known** and set in `ARENA_LONG_FT` / `ARENA_SHORT_FT` (ours: 35×25 ft). `PX_PER_M` (150 px/m) converts pixel↔metre. Which edge is the long one is decided automatically from the pixel edge lengths.

---

### 🔹 STAGE 3 — Target matching ⭐ (`stage3_robust.py`)

**Goal:** find each **seed (64)** in the drone LR (128) photos — which photo, and where.

**Method — DINOv2 semantic:** DINOv2 is Meta's **self-supervised Vision Transformer (ViT)**. It splits an image into small **patches** and gives each patch a **feature vector (embedding)** capturing "what this patch looks like" (colour + texture + shape), robust to **rotation/lighting/scale** changes. *Why a ViT:* attention gives global context — semantic matching (more robust than template/geometric).

**Steps + theory:**

1. **Illumination normalization (CLAHE):** apply **CLAHE** (adaptive histogram equalization) on the L-channel → reduces shadow/lighting differences.
2. **Prototypes from the seed:** using the seed's DINOv2 patches:
   - **Background prototype** = the average of **border** patches (the floor/surroundings).
   - **Object prototype** = the average of the patches **furthest** from the background (foreground) = "the target itself".
   - *Prototype = average → rotation-robust* (bag-of-features, order doesn't matter).
3. **Heatmap (per drone photo):** each patch's score = `(patch·object) − (patch·background)` (dot product = cosine similarity). High = target-like. → a **heatmap**; the peak = the target's location.
4. **Shortlist:** the top-K photos (by peak).
5. **Verification (appearance):** cosine similarity between the peak crop's DINOv2 embedding and the seed crop's embedding, taking the max over **4 rotations** (0/90/180/270) = `vsim`. Plus a **colour histogram** similarity (HSV hue-sat) → so distinct objects (white box vs grey rock) aren't confused.
6. **Found / Not-found:** `peak ≥ MIN_FOUND_PEAK` **AND** `vsim ≥ VERIFY_MIN` → FOUND, otherwise **NOT FOUND** (no random guess). **Auto-calibrate:** the threshold auto-adjusts from the background (lenient direction only) → catches subtle targets even in a dark arena.
7. **Mutual exclusion:** if two targets' field-positions are within `SEP_M` → conflict → **greedy assignment** (the strong-peak target claims its spot first, the other takes its next-best). No two targets at one spot.
8. **Multi-photo averaging:** if one target appears in several photos → slightly different position from each → **average** them → lower per-photo error, higher accuracy.

**Comparison here:** **seed-64 ↔ drone-128**, **LR-to-LR**, via **DINOv2 features (cosine)** — not pixel-to-pixel.

---

### 🔹 STAGE 4 — Coordinates + deliverables (`iroc_pipeline_fixed.py`)

**Goal:** convert a target's pixel location into a real-world coordinate.

**Theory + steps:**

1. **Homography chaining:** target pixel (drone photo) → **`photo_to_H`** (Stage 1) → **mosaic pixel** → **`M_persp`** (Stage 2) → **rectified pixel** → `÷ PX_PER_M` → **metres**.
2. **Base-station origin:** VIO `(0,0)` = drone **takeoff = base station**. We know each photo's VIO(x,y) and its rectified position → using `A` (VIO→mosaic affine, SIFT-calibrated) we project **VIO(0,0)** into the rectified frame → the base station's field-position. **Subtract this from every target** → base-station-relative coordinates.
3. **Save:** `lr_match/<t>.png` (LR), `proof_hd/<t>.jpg` (sharp native ≥720 HD), `targets.json` (coords), `annotated_field.jpg` (circle + label on the map).

---

### 🔹 STAGE 5 — 3D Map (`3d.py`) — OpenDroneMap photogrammetry (optional)

**Goal:** build a **3D model + elevation map** of the arena from the same overlapping HD photos.

**Theory — Photogrammetry (2D photos → 3D):** ODM (OpenDroneMap) runs in Docker (GPU). Pipeline:

1. **SIFT features:** thousands of distinctive points per photo (`--min-num-features 16000`).
2. **Feature matching:** match the same points across photos (`--matcher-neighbors 0` = all pairs → full connectivity).
3. **SfM (Structure from Motion):** from the matches + **bundle adjustment**, estimate the **camera positions/angles** + a **sparse 3D point cloud** — by minimizing reprojection error. (i.e. "where each photo was taken from" + "where the points are in 3D".)
4. **MVS (Multi-View Stereo):** turn the sparse into a **dense point cloud** — triangulate each pixel's depth from multiple overlapping views (`--pc-quality high`).
5. **Meshing + texturing:** dense cloud → surface **mesh** (2.5D) → map **colour/texture** from the original photos → **textured 3D model** (.obj + texture).
6. **DSM + Orthophoto:**
   - **DSM (Digital Surface Model)** = a top-down **elevation grid** (each cell's height `z`).
   - **Orthophoto** = a geometrically-corrected top-down **colour** image (perspective distortion removed, true scale — 2 cm/px).

**3d.py extras (after ODM):**
- **`export_glb`:** textured .obj → **`model.glb`** (universal — online viewer / Windows 3D Viewer / Blender, texture embedded). *This is the main 3D deliverable.*
- **`build_colored_cloud`:** fuses **orthophoto (colour) + DSM (elevation)** → each point gets `(x, y, z=height, rgb=real colour)` → a **colored point cloud** (`.ply`).
- **`make_2d_preview`:** side-by-side **orthophoto | DSM (TURBO colormap)** + an elevation scale-bar → `color_heightmap.jpg`.

**Outputs → `results/3d_map/`:** `model.glb` (textured 3D), `orthophoto.tif`, `dsm.tif`, `point_cloud.laz`, `color_heightmap.ply/.jpg`.

**Flow:**
```
HD photos → ODM: SIFT → match → SfM (bundle adj) → sparse cloud → MVS → dense cloud → mesh → texture
                  └→ orthophoto.tif + dsm.tif + point_cloud.laz + textured .obj
3d.py → export model.glb + fuse(ortho colour + DSM height) → colored point cloud + heightmap preview
```

**Run:** `python3 iroc_pipeline_fixed.py --run-3d` (full + 3D) or directly `python3 3d.py` (needs Docker). In the dashboard: "full + 3D" / "3D only".

---

## 4a. ⭐ Resolution chain — "at what size does the comparison happen?"

**Seed:**  `64×64` → *(to_work)* `640×480` → *(dino_patches)* `448×448` → DINOv2 features
**Drone:** `128×128` (drone_photos_lr) → *(to_work)* `640×480` → *(dino_patches)* `448×448` → DINOv2 features

| | Source size | to_work | DINOv2 input |
|---|---|---|---|
| **Seed** | **64×64** | 640×480 | 448×448 |
| **Drone (match)** | **128×128** | 640×480 | 448×448 |
| HD 1280×**720** | *not in matching* — stitch + HD proof + 3D | — | — |

➡️ **Comparison = seed-64 ↔ drone-128 (LR-to-LR).** Inside DINOv2 both go to 448 (32×32 patches) — this adds no new information, it's just the feature-extractor's fixed input size. The match is by **cosine** (not pixel-to-pixel), so different sizes are fine.

**Analogy:** face-recognition matches a person even from two differently-sized photos — because it compares **features**, not exact pixels.

---

## 5. Coordinate system (No GPS)

- **No GPS/GNSS** — Pixhawk VIO / optical-flow (`coordinates.csv`: `x_enu, y_enu, z_enu, yaw`).
- **VIO (Visual-Inertial Odometry):** fuses camera frames + IMU (accelerometer/gyro) to track the drone's position/heading — without satellites.
- **Origin (0,0) = base station** = takeoff (VIO 0,0). Coordinates are relative to it. The base station can be anywhere — controlled by the `BASE_STATION_EXACT` flag.
- **Accuracy:** the rectified-pixel frame (straightened by the yellow boundary) → distances between targets are exact (~0.1 m on distinct features).
- **z** — from the Pixhawk (untouched). **Max height ~6 m** (final field constraint).

---

## 6. Key algorithms — 1-line glossary

| Term | What it is |
|---|---|
| **LoFTR** | Detector-free deep image matcher (Transformer) — matches points even on low-texture. Stitching. |
| **RANSAC** | Outlier-robust fitting — removes wrong matches, keeps the correct transform. |
| **Homography** | A 3×3 matrix that maps one image onto another (or a rectified) plane. |
| **Bundle adjustment** | Jointly optimizes all camera/point estimates (minimizing reprojection error) — removes drift. |
| **DINOv2** | Self-supervised ViT — semantic embeddings of image patches (matching). |
| **CLAHE** | Adaptive histogram equalization — normalizes lighting/shadow. |
| **Cosine similarity** | The angle between two feature-vectors — how "similar". |
| **SfM** | Structure from Motion — camera poses + sparse 3D from 2D photos. |
| **MVS** | Multi-View Stereo — dense 3D point cloud. |
| **DSM** | Digital Surface Model — a top-down elevation grid. |
| **Orthophoto** | Distortion-free top-down colour map (true scale). |
| **VIO** | Visual-Inertial Odometry — GPS-free position from camera + IMU. |

---

## 7. What we improved (fixes)

| Fix | What it fixed |
|---|---|
| stage3_robust matcher | LR-to-LR DINOv2 semantic (seed-64 ↔ drone-128) |
| Stage-1 VIO pairing (#12/#13) | center/corner/spiral — clean full stitch for every pattern |
| Base-station origin (#1) | coordinates relative to the base station |
| Field scale (#3) | true metric size from the known arena dimensions (35×25 ft) |
| HD-720 + LR deliverables | one low-res and one high-res proof image per target |

---

## 8. Design decisions — why the system is built this way

Useful context if you are reading or modifying the code.

| Decision | Reason |
|---|---|
| **No GPS — VIO / optical flow instead** | Required by the rules, and GPS (metre-level) could not reach the ~0.1 m accuracy the task needs anyway. |
| **Coordinates measured in the rectified arena frame, not by integrating VIO** | VIO drifts over a survey, so integrated positions get progressively worse. The yellow boundary is a fixed physical reference, so the error does not grow with flight time. This is the most important accuracy decision in the project. |
| **Origin taken from VIO (0,0) = take-off point** | The base station's position is assigned at run time, so the origin cannot be hard-coded — it has to be measured. |
| **Scale from known arena dimensions** | Scale derived from VIO alone came out ≈ half the true size. Using the real 35×25 ft removes that error completely. |
| **DINOv2 (semantic) for target matching** | The target appears at a different height, angle, rotation and lighting than the seed. Template matching compares raw pixels and breaks; SIFT-style matchers need repeatable keypoints that low-texture paving does not provide. Semantic features match *what a thing is*, which survives all of those changes. |
| **Object prototype *minus* background prototype** | Without the subtraction, plain ground scores highly and the peak lands on empty floor. Scoring "object-like relative to its surroundings" is what actually localizes the feature. |
| **Two independent accept gates (`peak` + `vsim`)** | `peak` says something object-like is present; `vsim` says it really resembles the seed. Requiring both removes distractors that pass only one. If either fails, the target is reported NOT FOUND — a wrong coordinate is worse than an honest miss. |
| **Drone image at 128, not 64** | Testing showed 64 is visibly blurrier, with weaker feature responses and coarser localization; 128 keeps the search fast without that loss. |
| **LoFTR (detector-free) for stitching** | Plain paving and gravel give too few repeatable keypoints for classical detectors; LoFTR produces correspondences even on weak texture. |
| **Pairing photos by VIO distance instead of a grid** | Grid pairing assumed a clean lawnmower survey. On a centre-start flight many photos had no grid neighbour, the match graph disconnected, and half the arena was missing. Physical distance is what actually predicts overlap. |
| **Bundle adjustment + loop closure** | Pairwise errors compound along a chain and shear the mosaic. Optimising all transforms together keeps the arena rectangular. |
| **Multi-photo position averaging** | A target seen in several photos gives slightly different positions; averaging cancels per-photo mapping error. |
| **Mutual exclusion between targets** | Targets are unique, so two of them at one spot means a mistake — the weaker match is reassigned to its next-best candidate instead of silently reporting a duplicate. |
| **CLAHE + colour histogram** | Seed and survey photos are taken under different exposures; CLAHE equalizes local contrast, and the colour check separates objects that are structurally similar but differently coloured. |
| **OpenDroneMap for 3D instead of custom code** | Full SfM/MVS is a separate, heavy problem. Reusing a mature implementation is more reliable, and the step is optional so it never blocks the main result. |
| **Battery voltage read by the ESP32, not the pipeline** | The pipeline runs offline after the sortie; voltage is a live hardware measurement. Each subsystem reports only what it actually knows. |

### Known limitation

Everything downstream depends on the **yellow boundary being detected correctly**. If the mask
picks up ground or the base-station crate, the corners shift and every coordinate shifts with them.
Mitigations: a saturation threshold (`YELLOW_S`) that isolates bright tape, rejection of solid
blobs (real tape is thin), and `yellow_mask_debug.jpg` — the first file to check whenever results
look wrong.

---

*Files: `iroc_pipeline_fixed.py` (main), `stage3_robust.py` (matcher), `iroc_pipeline.py` (stitch/field), `3d.py` (ODM 3D), `make_lr.py` (seed 64), `base_station/` (dashboard). Tunable values: `PARAMETERS_GUIDE.md`.*
