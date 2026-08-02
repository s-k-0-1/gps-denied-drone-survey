# ASCEND Pipeline — How It Works (Full Theory · Viva Guide)

**Team LUMA · Drone ASCEND · IRoC-U 2026 · Rulebook V4.0 (Final Field Round)**

Poore system ki **theory** + **har part kaise kaam karta** + **kaunsi photo kis size/type se compare hoti**. Viva ke liye — har stage ka deep explanation.

---

## 0. ⭐ Sabse pehle — SEED, DRONE-LR, HD (confusion khatam)

Teen alag images, teen kaam:

| Image | Size | Kahan se | Kaam |
|---|---|---|---|
| **Seed / reference** | **64×64** | Final round me **organizers dete hain** (V4.0, 11.3.1) | matching ka reference |
| **Drone LR (ASCEND)** | **128×128** | ASCEND HD `1280×720` capture → down-sample (10.4) | matching me isme dhoondhte |
| **HD photo** | **1280×720** | ASCEND onboard camera | stitching + final HD proof + 3D |

**Match (Stage 3) = seed-64 ↔ drone-128** — DINOv2 features se (size alag chalta). **720 (HD) matching me use NAHI hota.**

---

## 1. Problem (ek line)

Drone khud arena survey karta hai, HD photos + position log karta hai; hum un photos se **targets** dhoondhte hain aur unke **coordinates** report karte hain — **bina GPS ke** (rulebook GPS/GNSS ban karta hai). Position **camera + Pixhawk VIO** (visual-inertial odometry / optical flow) se aati hai.

**Input:** `drone_photos/` (HD + `coordinates.csv`) + `targets/` (seed 64×64).
**Output (per target):** LR image, HD image, base-station-relative `(x, y, z)`. Plus optional 3D map.

---

## 2. Rulebook ka LR workflow (10.4)

Do phase LR-to-LR match:
1. **Reference/seed (pehle):** LR image 64×64. **Final me organizers dete** (elimination me teams).
2. **Sortie ke baad:** ASCEND **HD capture** → **usi HD ko down-sample** → **128×128 LR** (LR ASCEND image) → **seed se compare/match**.

> Humara pipeline: `build_drone_lr` HD → 128 LR (`drone_photos_lr/`), phir seed-64 se DINOv2 match. ✅

---

## 3. Poora flow

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

**Goal:** overlapping drone photos ko ek **top-view orthomosaic** me jodna.

**Theory + steps:**

1. **Pair selection (VIO KNN):** har photo ka `coordinates.csv` me `x,y` (VIO) hai. Har photo ko uske **spatially-nearest** photos se pair karte hain (K-nearest-neighbours). *Kyun:* jo photos paas hain unme **overlap** hota, wahi match honge. (Ye center/corner/spiral sab pattern pe kaam karta.)

2. **Feature matching — LoFTR:** LoFTR = **detector-free deep matcher** (Transformer-based). Normal matchers (SIFT) pehle "keypoints" detect karte, phir match — low-texture (gravel, plain ground) pe fail. LoFTR **poori image ke dense features** nikaal ke **attention** se do photos ke **corresponding points** seedhе deta — low-texture pe bhi strong. Output: sainkdon matching point-pairs.

3. **Transform estimation (RANSAC + homography):** matching points se ek **homography/affine matrix** `H` nikalta jo batata photo B ko photo A ke upar kahan/kaise rakhein. **RANSAC** outliers (galat matches) hata ke sirf **inliers** pe fit karta (isliye log me "inliers" count).

4. **Graph + BFS align:** saari photos ek **graph** (node=photo, edge=match). **Anchor** = center photo. **BFS** se har photo ka transform anchor ke frame me le aate.

5. **Bundle adjustment (global optimize):** BFS se chhote errors **compound** (drift) hote. Bundle adjustment saare transforms ko **ek saath** optimize karta — total **reprojection error** (matched points ka mismatch) minimize karke. → drift/skew khatam.

6. **Loop closure:** door ki photos (jo grid me wapas milti) ke extra matches dhoondh ke graph me daalte → aur constrain → mosaic aur straight.

7. **Warp + exposure blend:** har photo ko canvas pe warp, **exposure gains** se brightness match, aur seams pe smoothly **blend** → clean mosaic. Coverage ke bahar crop.

**Yahan compare:** **drone-HD ↔ drone-HD** (grayscale 960×544), **GEOMETRIC** (points).

---

### 🔹 STAGE 2 — Field map + Yellow boundary (`iroc_pipeline.py` + fixed)

**Goal:** tirchा mosaic ko **seedha rectangle** + **pixel→metre** scale.

**Theory + steps:**

1. **Yellow detect (Lab colour):** RGB lighting-sensitive hota. **Lab** colour space me `b*` = blue↔yellow axis. Yellow tape ka `b*−a*` **bada (+)** hota → isse mask. Lighting badle to **Otsu** (auto-threshold) se adapt. Patli tape rakho, **solid yellow blobs** (features) reject (fill-ratio se).

2. **Corners nikaalna:** mask ka **connected components** → sabse bada = tape frame. **Convex hull** → `approxPolyDP` (polygon approximate) → **4 corner points**.

3. **Perspective rectification (homography):** 4 detected corners ko ek **true rectangle** (known arena size — humara 35×25 ft) ke 4 corners pe map karne wali **perspective transform `M_persp`** nikaalte (`getPerspectiveTransform`). Isse mosaic ko warp → top-down straight image (camera tilt/perspective hata).

4. **Metric scale:** arena size **known** hai, `ARENA_LONG_FT` / `ARENA_SHORT_FT` me set (humara 35×25 ft). `PX_PER_M` (150 px/m) se pixel↔metre. Kaunsa edge lamba hai wo pixel lengths se **auto** decide hota.

---

### 🔹 STAGE 3 — Target matching ⭐ (`stage3_robust.py`)

**Goal:** har **seed (64)** ko drone LR (128) photos me dhoondhna — kis photo me, kahan.

**Method — DINOv2 semantic:** DINOv2 = Meta ka **self-supervised Vision Transformer (ViT)**. Image ko chhote **patches** me todta, har patch ka ek **feature vector (embedding)** deta jo "ye patch kis cheez jaisा dikhta" capture karta (colour+texture+shape), **rotation/lighting/scale** badle pe bhi. *Kyun ViT:* attention se global context — semantic match (template/geometric se robust).

**Steps + theory:**

1. **Illumination norm (CLAHE):** har image ke L-channel pe **CLAHE** (adaptive histogram equalization) → shadow/lighting differences kam.
2. **Seed se prototypes:** seed ke DINOv2 patches:
   - **Background prototype** = **border** patches ka average (floor/aas-paas).
   - **Object prototype** = jo patches background se sabse **door** (foreground) unka average = "target khud".
   - *Prototype = average → rotation-robust* (bag-of-features, order matter nahi karta).
3. **Heatmap (per drone photo):** har patch ka score = `(patch·object) − (patch·background)` (dot product = cosine similarity). High = target-jaisa. → **heatmap**; peak = target ki jagah.
4. **Shortlist:** top-K photos (peak ke hisaab se).
5. **Verification (appearance):** peak crop ka DINOv2 embedding ↔ seed crop embedding **cosine similarity**, **4 rotations** (0/90/180/270) me se max = `vsim`. + **colour histogram** similarity (HSV hue-sat) → distinct objects (white box vs grey rock) confuse na hon.
6. **Found / Not-found:** `peak ≥ MIN_FOUND_PEAK` **AND** `vsim ≥ VERIFY_MIN` → FOUND, warna **NOT FOUND** (koi random guess nahi). **Auto-calibrate:** threshold background se auto-adjust (sirf lenient direction) → dark arena me bhi subtle target catch.
7. **Mutual exclusion:** do targets ki field-position `SEP_M` se paas → conflict → **greedy assign** (strong-peak pehle apni jagah, doosra next-best). Ek jagah do target nahi.
8. **Multi-photo averaging:** ek target kai photos me dikhe → har photo se thodी alag position → un sabka **average** → per-photo error kam, accuracy up.

**Yahan compare:** **seed-64 ↔ drone-128**, **LR-to-LR**, **DINOv2 features (cosine)** — pixel-to-pixel nahi.

---

### 🔹 STAGE 4 — Coordinates + deliverables (`iroc_pipeline_fixed.py`)

**Goal:** target ki pixel-location ko real-world coordinate.

**Theory + steps:**

1. **Homography chaining:** target pixel (drone photo) → **`photo_to_H`** (Stage 1) → **mosaic pixel** → **`M_persp`** (Stage 2) → **rectified pixel** → `÷ PX_PER_M` → **metres**.
2. **Base-station origin:** VIO `(0,0)` = drone **takeoff = base station**. Har photo ka VIO(x,y) aur rectified-position dono pata → `A` (VIO→mosaic affine, SIFT-calibrated) se **VIO(0,0)** ko rectified frame me project → base station ki field-position. Ise **har target se subtract** → coords base-station-relative (rulebook 11.3.4).
3. **Save:** `lr_match/<t>.png` (LR), `proof_hd/<t>.jpg` (sharp native ≥720 HD), `targets.json` (coords), `annotated_field.jpg` (map pe circle + label).

---

### 🔹 STAGE 5 — 3D Map (`3d.py`) — OpenDroneMap photogrammetry (optional)

**Goal:** wahi overlapping HD photos se arena ka **3D model + elevation map** banana.

**Theory — Photogrammetry (2D photos → 3D):** ODM (OpenDroneMap) Docker me chalta (GPU). Pipeline:

1. **SIFT features:** har photo me hazaaron distinctive points (`--min-num-features 16000`).
2. **Feature matching:** photos ke beech same points match (`--matcher-neighbors 0` = saare pairs → full connectivity).
3. **SfM (Structure from Motion):** matched points + **bundle adjustment** se **camera positions/angles** + ek **sparse 3D point cloud** estimate — reprojection error minimize karke. (Yani "kaunsi photo kahan se li gayi" + "points 3D me kahan hain".)
4. **MVS (Multi-View Stereo):** sparse se **dense point cloud** — har pixel ka depth multiple overlapping views se triangulate (`--pc-quality high`).
5. **Meshing + texturing:** dense cloud → surface **mesh** (2.5D) → original photos se **colour/texture** map → **textured 3D model** (.obj + texture).
6. **DSM + Orthophoto:**
   - **DSM (Digital Surface Model)** = top-down **elevation grid** (har cell ki height `z`).
   - **Orthophoto** = geometrically-corrected top-down **colour** image (perspective distortion hata, true scale — 2 cm/px).

**3d.py extra (ODM ke baad):**
- **`export_glb`:** textured .obj → **`model.glb`** (universal — online viewer / Windows 3D Viewer / Blender, texture embedded). *Ye main 3D deliverable.*
- **`build_colored_cloud`:** **orthophoto (colour) + DSM (elevation)** ko fuse → har point ka `(x, y, z=height, rgb=real colour)` → **colored point cloud** (`.ply`).
- **`make_2d_preview`:** side-by-side **orthophoto | DSM (TURBO colormap)** + elevation scale-bar → `color_heightmap.jpg`.

**Outputs → `results/3d_map/`:** `model.glb` (textured 3D), `orthophoto.tif`, `dsm.tif`, `point_cloud.laz`, `color_heightmap.ply/.jpg`.

**Flow:**
```
HD photos → ODM: SIFT → match → SfM (bundle adj) → sparse cloud → MVS → dense cloud → mesh → texture
                  └→ orthophoto.tif + dsm.tif + point_cloud.laz + textured .obj
3d.py → export model.glb + fuse(ortho colour + DSM height) → colored point cloud + heightmap preview
```

**Run:** `python3 iroc_pipeline_fixed.py --run-3d` (full + 3D) ya seedhे `python3 3d.py` (Docker chahiye). Dashboard me "full + 3D" / "3D only".

---

## 4a. ⭐ Resolution chain — "kaunsi size pe compare?"

**Seed:**  `64×64` → *(to_work)* `640×480` → *(dino_patches)* `448×448` → DINOv2 features
**Drone:** `128×128` (drone_photos_lr) → *(to_work)* `640×480` → *(dino_patches)* `448×448` → DINOv2 features

| | Source size | to_work | DINOv2 input |
|---|---|---|---|
| **Seed** | **64×64** (V4.0) | 640×480 | 448×448 |
| **Drone (match)** | **128×128** (10.4) | 640×480 | 448×448 |
| HD 1280×**720** | *matching me NAHI* — stitch + HD proof + 3D | — | — |

➡️ **Compare = seed-64 ↔ drone-128 (LR-to-LR).** DINOv2 andar dono 448 pe (32×32 patches) — nayi info nahi, sirf feature-extractor ki fixed size. Match **cosine** se (pixel-to-pixel nahi), isliye size alag chalta.

**Analogy:** face-recognition alag-size photos se bhi banda match kar leta — **features** compare karta, exact pixels nahi.

---

## 5. Coordinate system (No GPS)

- **No GPS/GNSS** — Pixhawk VIO / optical-flow (`coordinates.csv`: `x_enu, y_enu, z_enu, yaw`).
- **VIO (Visual-Inertial Odometry):** camera ke frames + IMU (accelerometer/gyro) ko fuse karke drone ki position/heading track — bina satellite ke.
- **Origin (0,0) = base station** = takeoff (VIO 0,0). Coords iske relative (11.3.4). Base station kahin bhi ho — flag `BASE_STATION_EXACT`.
- **Accuracy:** rectified-pixel frame (yellow se straight) → targets ke aapas ke distances exact (~0.1 m distinct features pe).
- **z** — Pixhawk se (untouched). **Max height ~6 m** (final field constraint).

---

## 6. Key algorithms — 1-line glossary (viva me kaam aayega)

| Term | Kya hai |
|---|---|
| **LoFTR** | Detector-free deep image matcher (Transformer) — low-texture pe bhi points match. Stitching. |
| **RANSAC** | Outlier-robust fitting — galat matches hata ke sahi transform. |
| **Homography** | 3×3 matrix jo ek image ko doosri (ya rectified) plane pe map karta. |
| **Bundle adjustment** | Saare camera/point estimates ko ek saath optimize (reprojection error minimize) — drift hata. |
| **DINOv2** | Self-supervised ViT — image patches ke semantic embeddings (matching). |
| **CLAHE** | Adaptive histogram equalization — lighting/shadow normalize. |
| **Cosine similarity** | Do feature-vectors ka angle — kitne "similar". |
| **SfM** | Structure from Motion — 2D photos se camera poses + sparse 3D. |
| **MVS** | Multi-View Stereo — dense 3D point cloud. |
| **DSM** | Digital Surface Model — top-down elevation grid. |
| **Orthophoto** | Distortion-free top-down colour map (true scale). |
| **VIO** | Visual-Inertial Odometry — camera+IMU se GPS-free position. |

---

## 7. Humne kya kiya (fixes)

| Fix | Kya theek hua |
|---|---|
| stage3_robust matcher | LR-to-LR DINOv2 semantic (seed-64 ↔ drone-128) |
| Stage-1 VIO pairing (#12/#13) | center/corner/spiral — sab pe clean full stitch |
| Base-station origin (#1) | coords base station relative |
| Field scale (#3) | known arena dimensions se true metric size (35×25 ft) |
| HD-720 + LR deliverables | rulebook 11.3.8 output |

---

## 8. Viva Q&A

**Q: GPS use karte ho?** → Nahi. Pixhawk VIO / optical-flow.

**Q: Rulebook LR-to-LR maangta — karte ho?** → Haan. HD → 128 LR (10.4), seed 64 (V4) se DINOv2 compare.

**Q: Compare kaunsi size pe?** → **seed-64 ↔ drone-128**. Dono `640→448` pe process. **720 (HD) matching me NAHI.**

**Q: Size alag phir kaise match?** → DINOv2 feature vectors, **feature-space cosine** — pixel-to-pixel nahi.

**Q: 64×64 match karna pade to?** → `MATCH_LR=64 python3 iroc_pipeline_fixed.py --skip-stitch` — result alag `results_lr64/` me copy hota (dono compare kar sakte). Default (drone 128) best.

**Q: Stitching kaise?** → LoFTR se padosi drone-HD photos match (VIO se padosi chunte) → RANSAC homography → BFS → bundle adjustment → blend.

**Q: Low-texture ground pe match kaise?** → LoFTR (stitching) + DINOv2 (matching) dono **semantic/dense** — SIFT/geometric jahan fail wahan bhi chalte.

**Q: 3D kaise banta?** → `3d.py` → OpenDroneMap: SIFT → match → **SfM** (camera poses + sparse) → **MVS** (dense cloud) → mesh → texture → `model.glb` + orthophoto + **DSM** (elevation).

**Q: Coordinate origin?** → Base station (VIO takeoff), targets uske relative, metres.

**Q: False positive kaise rokte?** → `peak` + `vsim` threshold; kam ho to NOT FOUND. Object-vs-background prototype + colour + mutual-exclusion.

**Q: Accuracy?** → ~0.1 m distinct features (rectified-pixel + averaging).

---

*Files: `iroc_pipeline_fixed.py` (main), `stage3_robust.py` (matcher), `iroc_pipeline.py` (stitch/field), `3d.py` (ODM 3D), `make_lr.py` (seed 64), `base_station/` (dashboard). Tunable: `PARAMETERS_GUIDE.md`.*
