# IRoC Pipeline — Parameters Guide (`iroc_pipeline_fixed.py`)

Ye guide **saare tunable parameters** stage-wise deta hai: kya kaam, **kab badlo** (kaunsi error), aur **kaise** (badhao/ghatao). Chalane ke liye hamesha:
```bash
python3 iroc_pipeline_fixed.py            # full: stitch + field + match + annotate
```

Parameters **4 jagah** hain:
- **A. `iroc_pipeline_fixed.py`** (top) — coordinate frame, dedup, averaging, stitch-fix flags
- **B. `stage3_robust.py`** (top) — matching / detection thresholds
- **C. `iroc_pipeline.py`** (top, CONFIG) — stitching, field scale, camera
- **D. `make_lr.py`** — seed reference LR size

---

## 0. Run flags (command line)

| Flag | Kaam | Kab use |
|---|---|---|
| *(none)* | Full run: stitch + field + match + annotate | Naya data / pehli baar |
| `--skip-stitch` | Mosaic reuse, **matching fresh** | Sirf matching/coords test (stitch already achha) |
| `--skip-match` | Sab reuse, sirf coords recompute | Sirf coordinate/field tweak (fastest) |
| `--radius 2` | Stitch me zyada neighbour pairs (8-NN → 12-NN) | Overlap kam / stitch weak ho to (slower) |
| `--run-3d` | 3D reconstruction (ODM, Docker) | 3D map chahiye |

> **Note:** stitch improve karna hai to `--skip-stitch` **mat** lagao (wo Stage 1 skip kar deta hai).

---

## 1. STAGE 1 — Stitching (`iroc_pipeline.py` CONFIG + fixed ke fix#12/#13)

Photos ko orthomosaic me jodta hai (LoFTR + bundle adjustment). **Pairing ab VIO position se hoti hai** (fix#13) — center/corner/spiral/lawnmower sab patterns pe robust.

### 1a. Stitch fixes (`iroc_pipeline_fixed.py` — automatic, param nahi)
| Fix | Kya karta | Requirement |
|---|---|---|
| **fix#12** | `discover()` ab filename regex ki jagah **coordinates.csv se sahi `row,col`** leta (anchor/grid ke liye) | `coordinates.csv` me `row,col` columns |
| **fix#13** | `select_pairs()` ab **VIO `x_enu,y_enu` ke K-nearest-neighbours + temporal** se pair karta — graph kabhi disconnect nahi hota | `coordinates.csv` me `x_enu,y_enu` |
| K neighbours | radius 1 → **8-NN**, radius 2 → **12-NN** | `--radius 2` se badhta |

**Error → fix:** *Aadha/corner mosaic (center-start pe)* ya *photos DROP (Stitched 15/23)* → fix#12/#13 se auto theek. Log me `[fix#12]…` aur `[fix#13] spatial pairing: N/N photos …` dikhna chahiye. VIO na mile to purana grid pairing (fallback).

### 1b. Stitch tuning (`iroc_pipeline.py` CONFIG)
| Parameter | Default | Kya karta | Kab / Kaise badlo |
|---|---|---|---|
| `GRID_RADIUS` | `1` | K-NN neighbours (fix#13): 1→8, 2→12 | Stitch me **gaps** ya bahut kam overlap → `2`. Ya `--radius 2` |
| `MIN_INLIERS` | `30` | Do photos "matched" maanne ke min inliers | Kam-overlap photos **skip** → **ghatao** (20). Galat pairs jud rahe → **badhao** (40) |
| `MIN_INLIER_RATIO` | `0.20` | Inlier/matches ratio | Bahut pairs reject → ghatao (0.15) |
| `RANSAC_THRESH` | `5.0` | Homography RANSAC pixel tolerance | Distorted stitch → ghatao (3.0); kam matches → badhao (7.0) |
| `SCALE_LO, SCALE_HI` | `0.40, 2.50` | Photos ke beech allowed scale | Altitude bahut vary kare → range widen |
| `MATCH_W, MATCH_H` | `960, 544` | LoFTR input size | Zyada accurate (slow) → badhao |

**Error → fix:** *"BFS aligned X/N" me X kam* ya *mosaic tuta* → pehle `[fix#13]` log check (VIO mil raha?); phir `--radius 2` + `MIN_INLIERS` ghatao.

---

## 2. STAGE 2 — Field map + Yellow boundary

### 2a. Arena size / scale
| Parameter | File | Default | Kya karta | Kab / Kaise |
|---|---|---|---|---|
| `ARENA_LONG_FT` | fixed | `30.0` | Arena ka **lamba** edge (ft) | Aapka arena alag size → yahan set |
| `ARENA_SHORT_FT` | fixed | `25.0` | Arena ka **chhota** edge (ft) | ″ (longer pixel edge auto = LONG) |
| `PX_PER_M` | base | `150` | Rectified image resolution (px/m) | Sharper map (slow) → 200; halka → 100 |

**Error → fix:** *Field size galat (e.g. 9.14×9.66)* → `ARENA_LONG_FT/SHORT_FT` apne asli arena ke hisaab se set karo. Kaunsa side lamba hai wo **auto** decide hota (mosaic pixel edges se).

### 2b. Yellow detection (`iroc_pipeline_fixed.py` → `detect_yellow_corners`)
| Cheez | Default | Kya karta | Kab / Kaise |
|---|---|---|---|
| HSV hue gate | `14–48` | Yellow hue range | Tape ka shade alag → range adjust |
| `s >= 28` | 28 | Min saturation | Faded tape na mile → ghatao (20) |
| `diff (b*-a*) >= 15` | 15 | Yellow-ness (Lab) | Faded tape → ghatao |
| **fill `>= 0.60` reject** | 0.60 | **Solid blobs** (features) reject, thin tape rakho | Yellow feature reject → badhao (0.7); tape reject → badhao |
| Auto (Otsu) fallback | — | Fixed fail ho to auto-threshold | Automatic (lighting badle to khud) |

**Error → fix:** *Yellow line galat / corners off* → `stage2_field/yellow_mask_debug.jpg` dekho. Extra blobs → fill threshold; tape adhoora → `s`/`diff` ghatao.

### 2c. Axis convention
| Parameter | File | Default | Kab / Kaise |
|---|---|---|---|
| `SWAP_AXES` | fixed | `False` | Coords me **x aur y ULTE** aayen (ground-truth se) → `True` |

---

## 3. STAGE 3 — Matching / Detection (`stage3_robust.py`) ⭐ *sabse zyada tuning yahin*

### 3a. Found / Not-found thresholds
| Parameter | Default | Kya karta | Kab / Kaise |
|---|---|---|---|
| `MIN_FOUND_PEAK` | `0.14` | Object-heatmap peak floor (found gate) | Asli target **NOT FOUND** → ghatao (0.10). Random match → badhao |
| `VERIFY_MIN` | `0.45` | Verification (crop-sim) floor | Asli target NOT FOUND (verify kam) → ghatao (0.40). Galat match pass → badhao (0.55) |
| `AUTO_CALIBRATE` | `True` | Peak threshold background se auto (sirf lenient direction) | Naye arena/lighting pe khud adjust. Fixed chahiye → `False` |
| `PEAK_RATIO` | `1.5` | Auto-cal: peak ≥ background×ratio | Dark arena me miss → ghatao (1.2) |
| `PEAK_FLOOR` | `0.06` | Auto-cal absolute floor (noise reject) | — |

### 3b. Selection / localization
| Parameter | Default | Kya karta | Kab / Kaise |
|---|---|---|---|
| `TOPK` | `8` | Kitni photos shortlist (peak se) | Zyada candidates → badhao (slow) |
| `CENTER_PREF` | `0.06` | Photo-center wale detection prefer | Edge/aadha-cut target → badhao (0.10) |
| `ROTATIONS` | `(0,1,2,3)` | Verification me 0/90/180/270 rotation | Rotation-invariance (rehne do) |
| `MIN_SEP_M` | `0.6` | Matcher-level min target separation | — (pipeline-level `SEP_M` main hai) |
| **Color weight** | `0.4 + 0.6×cs` | DINOv2 + color rank (distinct objects) | Color se galat → weight ghatao; distinct-color features → color pe zyada bharosa |

### 3c. Camera (gpos / mutual exclusion ke liye)
| Parameter | Default | Kab / Kaise |
|---|---|---|
| `DRONE_HEIGHT` | `3.0` | Flight altitude alag → set |
| `FOV_H_DEG, FOV_V_DEG` | `90, 65` | Camera FOV alag → set |

**Common Stage-3 errors:**
- *Galat object pe circle* → `VERIFY_MIN` badhao, ya color weight badhao.
- *Asli target NOT FOUND* → `MIN_FOUND_PEAK` / `VERIFY_MIN` ghatao.
- *Edge/aadha-cut* → `CENTER_PREF` badhao.
- Har target ka `peak=.. V..` terminal me print — usse tune karo.

### 3d. Output deliverables (per target — rulebook 11.3.8)
| Cheez | Default | File | Kab / Kaise |
|---|---|---|---|
| **LR image** (11.3.8a) | `128×128` | `stage3_targets/lr_match/<t>.png` | Seed jaisा tight feature crop. Size badalna → matcher/`_regen` me `(128,128)` |
| **HD proof** (11.3.8b) | native, **shorter side ≥720** | `stage3_targets/proof_hd/<t>.jpg` | SHARP native crop. `720` badalna → `sc/bh/bw` me |
| HD tight crop size | `half = max(hr*2.2, hw*0.06)` | — | HD me zyada/kam background → `2.2` / `0.06` adjust |

### 3e. Match resolution — LR-to-LR + `MATCH_LR` mode (`stage3_robust.py`)
**Seed 64×64** (rulebook **V4.0**, organizers dete) **↔ drone 128×128** (rulebook **10.4**: HD→128 down-sample). Match DINOv2 se — size alag chalta. **HD 720 matching me NAHI** (sirf stitch + proof).

**`MATCH_LR` env var — drone ki LR size choose karo (option, bina code chhede):**

| Command | Drone LR | Compare | Kab |
|---|---|---|---|
| `python3 iroc_pipeline_fixed.py --skip-stitch` | 128 | **seed-64 ↔ drone-128** | **DEFAULT — best, abhi yehi** ✅ |
| `MATCH_LR=64 python3 iroc_pipeline_fixed.py --skip-stitch` | 64 | **seed-64 ↔ drone-64** | Agar **64×64 match** karna pade (organizer/round bole) 🔧 |
| `MATCH_LR=128 python3 iroc_pipeline_fixed.py --skip-stitch` | 128 | seed-64 ↔ drone-128 | = default |

**Chain (default):** seed `64→640→448`, drone `128→640→448` (DINOv2 32×32 patches).

> **`MATCH_LR=64` option RAKHA hua hai** — abhi 128 best hai (perfect test), par agar kabhi **64×64** karna pade to bas `MATCH_LR=64` laga do (koi code change nahi). Us me localization thodी coarse hogi. Default = seed-64 ↔ drone-128.
>
> **Output folder:** `MATCH_LR` set ho to us run ka result **`results_lr<N>/`** me bhi copy ho jaata (e.g. `results_lr64/` — `stage3_targets` + `stage4_annotated`), taaki default (128) `results/` aur 64×64 dono **ek saath** rahen. **Dono chahiye to order:** pehle `MATCH_LR=64 …` (→ `results_lr64/`), phir default `…` (→ `results/`).

---

## 4. STAGE 4 — Coordinates, Frame, Dedup, Averaging (`iroc_pipeline_fixed.py`)

### 4a. Coordinate frame ⭐
| Parameter | Default | Kya karta | Kab / Kaise |
|---|---|---|---|
| **`BASE_STATION_EXACT`** | `True` | Origin (0,0) = **actual base station** (VIO takeoff), coords uske relative (rulebook 11.2.2/11.3.4) | Final round → `True` (base station kahin bhi ho). Qualifier/practice me yellow-corner origin chahiye → `False` |

**Error → fix:** *Coords ek constant amount shift/galat frame* → log me `[fix#1 base-origin] base station @ field (x,y)` dekho; base station takeoff ke paas hona chahiye. Galat lage → `BASE_STATION_EXACT=False` (yellow corner). `A` (SIFT calib) fail ho to auto yellow-corner.

### 4b. Dedup / averaging
| Parameter | Default | Kya karta | Kab / Kaise |
|---|---|---|---|
| `SEP_M` | `0.6` | Do targets is se paas → conflict (mutual exclusion) | Targets sach me < 0.6m paas → ghatao. Duplicate reh raha → badhao |
| `AVG_R` | `0.5` | Is radius me same-object candidates **average** (accuracy) | Averaging off → `0`. Zyada aggressive → badhao |
| `DUP_M` | `0.5` | Is se paas targets pe **warning** | Sirf warning threshold |
| `MAX_INSTANCES` | `1` | Har feature-type ke max instances | Final round targets **unique** → `1`. Multiple instances chahiye → `2/3` |
| `EXTRA_VERIFY_MIN` | `0.60` | Extra instance ke liye strict verify | `MAX_INSTANCES>1` pe hi lagta; false extra → badhao |

**Error → fix:**
- *Do targets ek jagah (overlap)* → `[fix#9 WARN]` dekho; `SEP_M` sahi karo.
- *Coords thodi jitter* → `AVG_R` badhao (zyada photos average).
- *z galat* → z Pixhawk se aata (untouched, user request).

---

## 5. Seed references (`make_lr.py`)
| Parameter | Default | Kya karta | Kab / Kaise |
|---|---|---|---|
| `LR_SIZE` | `(64, 64)` | Seed reference LR size (**rulebook V4.0, 11.3.1**) | V4.0 Final = **64** (organizers dete). Drone LR alag se 128 (10.4). Compare = seed-64 ↔ drone-128 |
| `SRC_DIR` | `reference/` | Input full-res references | — |
| `OUT_DIR` | `targets/` | Output LR seeds (matcher yahin se) | — |

---

## 6. Quick "error → parameter" table

| Error | Parameter | Direction |
|---|---|---|
| Aadha/corner mosaic, photos DROP | fix#12/#13 (auto), `--radius 2`, `MIN_INLIERS` | log check / 2 / ghatao |
| Stitch tuta / gaps | `GRID_RADIUS` / `--radius 2`, `MIN_INLIERS` | 2 / ghatao |
| Field size galat | `ARENA_LONG_FT`, `ARENA_SHORT_FT` | sahi value |
| Yellow line off | `detect_yellow_corners`: fill, `s`, `diff` | dekh ke adjust |
| x/y ulte | `SWAP_AXES` | `True` |
| Coords base-station se nahi / shift | `BASE_STATION_EXACT` | `True` (Final) / `False` (corner) |
| Asli target NOT FOUND | `MIN_FOUND_PEAK`, `VERIFY_MIN` | ghatao |
| Galat/random match | `VERIFY_MIN`, color weight | badhao |
| Edge/aadha-cut target | `CENTER_PREF` | badhao |
| 2 target ek jagah | `SEP_M` | adjust |
| Coords jitter | `AVG_R` | badhao |
| Multiple instances chahiye | `MAX_INSTANCES` | `2/3` |
| HD me zyada background | HD `half` (`2.2`/`0.06`) | ghatao |
| Naya arena/lighting | `AUTO_CALIBRATE=True` (already) | — |

---

## 7. Kaise tune karein (workflow)
1. Poora run: `python3 iroc_pipeline_fixed.py`
2. Terminal log dekho: `[fix#12]/[fix#13]` (stitch), `[fix#1 base-origin]`, `[fix#3]` (field size), har target ka `peak / V` (Stage 3). Plus `annotated_field.jpg`, `yellow_mask_debug.jpg`, `orthomosaic.jpg`.
3. Upar table se relevant parameter ek-ek karke badlo.
4. **Sirf coords/field tweak** → `--skip-match` (fastest). **Matching tweak** → `--skip-stitch`. **Stitch tweak** → full run (skip-stitch nahi).
5. Ek baar me **ek** parameter badlo, verify karo.

> Locations: `iroc_pipeline_fixed.py` ke params top pe (STAGE3 / BASE_STATION_EXACT ke aas-paas). `stage3_robust.py` ke top "CONFIG" block. `iroc_pipeline.py` ke top "CONFIG" (# Stitching / # Field map). `make_lr.py` ke top.
