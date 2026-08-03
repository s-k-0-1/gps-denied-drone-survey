# Parameters Guide — 64×64 Search Mode (`MATCH_LR=64`)

This guide covers **only the 64×64 LR-to-LR search** (seed-64 ↔ **drone-64**). For the default
mode (seed-64 ↔ drone-128) see [PARAMETERS_GUIDE.md](PARAMETERS_GUIDE.md).

---

## 1. When and why

- Use it only if you specifically need matching at 64×64 (the drone image is also
  down-sampled to 64).
- The default (**drone 128**) is **more accurate** — use 64×64 only when required.
- At 64×64 the drone image is smaller and therefore **blurrier**, so DINOv2 features are
  **weaker**, `peak` / `vsim` values come out **lower**, and localization is a little **coarser**.

---

## 2. How to run it

**Terminal (recommended — tuned thresholds):**

```bash
cd ~/advanced_matcher
MATCH_LR=64 MIN_FOUND_PEAK=0.11 VERIFY_MIN=0.40 CENTER_PREF=0.09 TOPK=10 \
    python3 iroc_pipeline_fixed.py --skip-stitch
```

**Plain (default thresholds — weaker at this resolution):**

```bash
MATCH_LR=64 python3 iroc_pipeline_fixed.py --skip-stitch
```

**Dashboard:** pipeline dropdown → **"match 64×64 (LR)"** → **Run**.
(The terminal is better for threshold tuning; the dashboard job uses the defaults.)

The result is copied to **`results_lr64/`**, so it does not overwrite your default run. View it in
the dashboard with **View → 64×64**.

---

## 3. Tuned parameters (`stage3_robust.py`, settable via environment variables)

Because the features are weaker, **lower** the thresholds — otherwise a real target is reported
NOT FOUND:

| Parameter | Default (128) | **64×64 suggested** | Why |
|---|---|---|---|
| `MIN_FOUND_PEAK` | `0.14` | **`0.11`** (0.10–0.12) | Peak response is weaker at 64 |
| `VERIFY_MIN` | `0.45` | **`0.40`** (0.38–0.42) | Crop similarity is weaker |
| `CENTER_PREF` | `0.06` | **`0.09`** | Coarser localization → prefer centred detections |
| `TOPK` | `8` | **`10`** | More candidates to choose the best from |
| `AUTO_CALIBRATE` | `True` | `True` (leave it) | Automatically lenient in dark/low-signal arenas |
| `PEAK_RATIO` | `1.5` | `1.5` (or `1.3`) | Lower it if a dark arena still needs more leniency |

> **How to set them:** append them as environment variables to the command (as shown above), or
> edit the values at the top of `stage3_robust.py`. If the variables are unset the defaults apply,
> so the normal mode is unaffected.

**If problems remain**
- *Real target NOT FOUND* → lower further (`0.09` / `0.36`).
- *Wrong / random match* → raise back up (`0.12` / `0.42`).
- Every target prints `peak=… V=…` in the terminal — tune against those numbers.

---

## 4. Coordinates / dedup (small adjustment, `iroc_pipeline_fixed.py`)

Coarser localization means more jitter, so average over more photos:

| Parameter | Default | 64×64 suggested | Why |
|---|---|---|---|
| `AVG_R` | `0.5` | **`0.7`** | Averages more photos → smooths the coarse position |
| `SEP_M` | `0.6` | `0.6` (or `0.7`) | Avoids false "same spot" conflicts |

Everything else (`BASE_STATION_EXACT`, `HEADING_ROT_DEG`, `MAX_INSTANCES`) stays at its default.

---

## 5. What does **not** change in 64×64 mode

- **Stage 1 stitching** (VIO pairing, `GRID_RADIUS`, `MIN_INLIERS`, …) — identical
- **Stage 2 yellow boundary / field scale** (`ARENA_LONG_FT`, `PX_PER_M`, detection) — identical
- **Coordinate frame** (base-station origin, heading rotation) — identical

Only the **Stage 3 matching resolution and its thresholds** are adjusted.

---

## 6. Verify (compare both modes)

```bash
# 64×64 (tuned) -> results_lr64/
MATCH_LR=64 MIN_FOUND_PEAK=0.11 VERIFY_MIN=0.40 CENTER_PREF=0.09 TOPK=10 \
    python3 iroc_pipeline_fixed.py --skip-stitch

# default 128 -> results/
python3 iroc_pipeline_fixed.py --skip-stitch
```

Check:
- Terminal: `Found N/M targets` and each target's `peak / V` (lower values at 64×64 are normal).
- `results_lr64/stage3_targets/targets.json` (coordinates) and `stage4_annotated/annotated_field.jpg`.
- Dashboard **View: Default ↔ 64×64** for a side-by-side comparison.

**Goal:** both modes find the same targets with coordinates close to each other. A slightly larger
position error at 64×64 is acceptable — missing a target is not.

---

## 7. One-line summary

> **64×64:** `MATCH_LR=64` plus lower thresholds (`MIN_FOUND_PEAK≈0.11`, `VERIFY_MIN≈0.40`,
> `CENTER_PREF≈0.09`, `TOPK=10`) and `AVG_R≈0.7`. Output goes to `results_lr64/`. Stitching, field
> map and coordinates are unchanged. The default (128) is always better — use 64×64 only when
> required.
