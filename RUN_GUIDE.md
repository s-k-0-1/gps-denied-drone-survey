# RUN & VALIDATION GUIDE — ASCEND Pipeline

Ye guide batata hai pipeline **kaise chalao** aur **kaise verify karo** ki sab theek chal raha hai (saare recent edits ke baad). Ek baar poora chala ke checklist tick karo.

---

## 1. Pehle ye check karo (inputs)

| Chahiye | Kahan | Note |
|---|---|---|
| HD survey photos | `drone_photos/*.jpg` (1280×720) | ASCEND ki photos |
| Position log | `drone_photos/coordinates.csv` | `x_enu,y_enu,z_enu,yaw,row,col,image_file` columns zaroori (fix#12/#13 inhi se) |
| Seed references | `targets/*.png` (64×64) | Final me organizers dete; practice me `python3 make_lr.py` (reference/ se) |

> **Voltage (11.3.8d):** ESP se **direct dashboard** pe aata hai — pipeline me kuch nahi karna.

---

## 2. Chalao

```bash
cd ~/advanced_matcher

# FULL run (pehli baar / naya data): stitch + field + match + coords + annotate
python3 iroc_pipeline_fixed.py

# sirf matching/coords dubara (mosaic achha hai): 
python3 iroc_pipeline_fixed.py --skip-stitch

# + 3D map (Docker chahiye)
python3 iroc_pipeline_fixed.py --run-3d
```

---

## 3. Log me ye lines dhoondho (✓ = sab sahi)

| Log line | Matlab |
|---|---|
| `[fix#12] stitch grid: N/N photos ... (row,col) mila` | CSV se sahi grid mila ✓ |
| `[fix#13] spatial pairing: N/N photos, XX pairs (VIO ...)` | VIO KNN pairing chala ✓ |
| `Stitched N/N photos` | **N ~ total** hona chahiye (drop kam) ✓ |
| `[fix#3] TRUE size 10.67 x 7.62 m` | field metric size = `ARENA_LONG_FT`×`ARENA_SHORT_FT` (35×25 ft) ✓ |
| `[fix#1 base-origin] base station @ field (x,y)` | base-station origin active ✓ |
| `[fix#14] coords rotated ...°` | *sirf tab jab* `HEADING_ROT_DEG` set ho |
| Har target: `peak=.. V..` + `Found N/M targets` | matching result ✓ |

**Agar koi Python error/traceback aaye** → wo line mujhe bhejo, turant fix kar dunga (maine bahut edits kiye, ek baar confirm zaroori).

---

## 4. Output files check karo

| File | Kya dekho |
|---|---|
| `results/stage1_stitch/orthomosaic.jpg` | arena poora, straight, kam skew/ghosting |
| `results/stage2_field/rectified_field.jpg` | seedha rectangle, yellow boundary aligned |
| `results/stage2_field/yellow_mask_debug.jpg` | sirf tape (solid features nahi) |
| `results/stage3_targets/targets.json` | `map_xyz` (coords) per target |
| `results/stage3_targets/proof_hd/<t>.jpg` | sharp HD, feature-focused (~720) |
| `results/stage3_targets/lr_match/<t>.png` | LR (128) — seed se match |
| `results/stage4_annotated/annotated_field.jpg` | targets pe circle + coords, sahi jagah |

---

## 5. Naye config flags (zaroorat pade to)

`iroc_pipeline_fixed.py` (top):

| Flag | Default | Kab badlo |
|---|---|---|
| `BASE_STATION_EXACT` | `True` | Final round True (base station origin). Qualifier corner chahiye → `False` |
| `HEADING_ROT_DEG` | `0.0` | Final round me coords **assigned heading** (0/90/180/270) ke frame me chahiye ho → set (e.g. `90`). Direction ulta lage → negative |
| `MATCH_LR` (env) | unset | `MATCH_LR=64 ...` → 64×64 match, result `results_lr64/` me alag |

---

## 6. Common issue → quick fix

| Dikhe | Karo |
|---|---|
| Aadha/corner mosaic, photos drop | `[fix#13]` log check; `--radius 2` |
| Yellow line off | `yellow_mask_debug.jpg` dekho; `PARAMETERS_GUIDE` 2b |
| Coords shift/galat frame | `[fix#1 base-origin]` log; `BASE_STATION_EXACT` |
| Coords rotated chahiye | `HEADING_ROT_DEG` set |
| Target NOT FOUND / galat | `MIN_FOUND_PEAK` / `VERIFY_MIN` (PARAMETERS 3a) |
| Python crash | error line bhejo |

> Detail tuning: **`PARAMETERS_GUIDE.md`** (ya `_EN`). Theory: **`HOW_IT_WORKS.md`** (ya `_EN`).

---

## 7. Dashboard se (optional)

```bash
pip install -r base_station/requirements.txt --break-system-packages   # ek baar
python3 -m base_station.server
```
Browser: **http://localhost:8000** (login `luma` / `ascend2026`) → **START MISSION** ya pipeline dropdown → **Run**. (Ab fixed pipeline chalata hai.)

---

### Bas ek baar poora chala ke Section 3 ki lines + Section 4 ke outputs confirm kar do — phir pipeline fully validated hai. ✅
