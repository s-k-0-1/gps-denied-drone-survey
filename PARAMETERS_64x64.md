# Parameters Guide — 64×64 Search Mode (`MATCH_LR=64`)

Ye guide **sirf 64×64 LR-to-LR search** ke liye hai (seed-64 ↔ **drone-64**). Default mode (seed-64 ↔ drone-128) ke params ke liye `PARAMETERS_GUIDE.md` dekho.

---

## 1. Kab / kyun 64×64

- Sirf jab **rulebook/organizer 64×64 match** maange (drone bhi 64 pe).
- Default (**drone 128**) **zyada accurate** hai — 64×64 sirf zaroorat pade to.
- 64×64 me drone image chhota → **blurry** → DINOv2 features **weaker** → `peak/vsim` **kam** → localization **thodी coarse**.

---

## 2. Kaise chalao

**Terminal (recommended — tuned):**
```bash
cd ~/advanced_matcher
MATCH_LR=64 MIN_FOUND_PEAK=0.11 VERIFY_MIN=0.40 CENTER_PREF=0.09 TOPK=10 \
    python3 iroc_pipeline_fixed.py --skip-stitch
```

**Plain (default thresholds, weaker):**
```bash
MATCH_LR=64 python3 iroc_pipeline_fixed.py --skip-stitch
```

**Dashboard:** pipeline dropdown → **"match 64×64 (LR)"** → **Run**. (env-tuning ke liye terminal behtar; dashboard job default thresholds use karta.)

Result **`results_lr64/`** me alag copy hota — dashboard **View → 64×64** se dekho.

---

## 3. 64×64 ke tuned parameters (`stage3_robust.py` — ab env se set kar sakte ho)

Features weaker hone ki wajah se thresholds **neeche** karo (warna asli target NOT FOUND):

| Parameter | Default (128) | **64×64 suggested** | Kyun |
|---|---|---|---|
| `MIN_FOUND_PEAK` | `0.14` | **`0.11`** (0.10–0.12) | 64 pe peak kamzor → warna miss |
| `VERIFY_MIN` | `0.45` | **`0.40`** (0.38–0.42) | crop-sim kamzor → warna NOT FOUND |
| `CENTER_PREF` | `0.06` | **`0.09`** | coarse localization → center-wale zyada prefer |
| `TOPK` | `8` | **`10`** | zyada candidates → best chunne ka mauka |
| `AUTO_CALIBRATE` | `True` | `True` (rehne do) | dark/low-signal pe khud lenient |
| `PEAK_RATIO` | `1.5` | `1.5` (ya `1.3`) | dark arena me aur lenient chahiye to ghatao |

> **Set kaise:** env-var se (command me aage laga do), ya `stage3_robust.py` ke top me values badal do. Env unset ho to default (0.14/0.45/…) chalta — default mode par asar nahi.

**Agar phir bhi problem:**
- *Asli target NOT FOUND* → `MIN_FOUND_PEAK`/`VERIFY_MIN` aur **ghatao** (0.09 / 0.36).
- *Galat/random match* → thoda **badhao** wapas (0.12 / 0.42).
- Terminal me har target ka `peak=.. V..` print hota — usse tune karo.

---

## 4. Coords / dedup (64×64 me thoda adjust — `iroc_pipeline_fixed.py`)

Localization coarse hone se positions me jitter zyada → averaging badhao:

| Parameter | Default | 64×64 suggested | Kyun |
|---|---|---|---|
| `AVG_R` | `0.5` | **`0.7`** | zyada photos average → coarse position smooth |
| `SEP_M` | `0.6` | `0.6` (ya `0.7`) | targets thodे door na dikhein to conflict avoid |

Baaki (`BASE_STATION_EXACT`, `HEADING_ROT_DEG`, `MAX_INSTANCES`) **default jaisा** — chhedना nahi.

---

## 5. Jo 64×64 me **NAHI** badalta (default jaisा hi)

- **Stage 1 stitch** (VIO pairing, GRID_RADIUS, MIN_INLIERS…) — same
- **Stage 2 yellow / field scale** (ARENA_LONG_FT, PX_PER_M, yellow detection) — same
- **Coordinate frame** (base-station origin, heading) — same
- Sirf **Stage 3 matching resolution + thresholds** 64×64 ke liye adjust hote.

---

## 6. Verify (dono compare karo)

```bash
# 64×64 (tuned) -> results_lr64/
MATCH_LR=64 MIN_FOUND_PEAK=0.11 VERIFY_MIN=0.40 CENTER_PREF=0.09 TOPK=10 \
    python3 iroc_pipeline_fixed.py --skip-stitch

# default 128 -> results/
python3 iroc_pipeline_fixed.py --skip-stitch
```

Check karo:
- Terminal: `Found N/M targets` + har target `peak / V` (64×64 me values kam honge — normal).
- `results_lr64/stage3_targets/targets.json` (coords) + `stage4_annotated/annotated_field.jpg`.
- Dashboard **View: Default ↔ 64×64** se side-by-side compare.

**Target: dono me same targets Found + coords ~paas.** 64×64 me thoda zyada position-error acceptable hai (coarse), par targets miss nahi hone chahiye.

---

## 7. Ek-line summary

> **64×64:** `MATCH_LR=64` + lower thresholds (`MIN_FOUND_PEAK≈0.11`, `VERIFY_MIN≈0.40`, `CENTER_PREF≈0.09`, `TOPK=10`) + `AVG_R≈0.7`. Result `results_lr64/` me. Stitch/field/coords sab default jaisा. Default (128) hamesha behtar — 64×64 sirf zaroorat par.
