# Examples — what a successful run produces

This folder shows the **expected output** so you can check your own run against it, even before
you have your own flight data.

---

## Output layout

After `python3 iroc_pipeline_fixed.py` finishes:

```
results/
├── stage1_stitch/
│   ├── orthomosaic.jpg            ← all photos joined into one map
│   ├── photo_transforms.npz       ← photo → mosaic transforms
│   └── stitch_log.txt             ← per-pair inlier counts
├── stage2_field/
│   ├── rectified_field.jpg        ← straightened, true-scale arena
│   ├── yellow_mask_debug.jpg      ← what the boundary detector saw
│   ├── yellow_corners_debug.jpg   ← the 4 detected corners
│   └── calibration.txt            ← field size + origin
├── stage3_targets/
│   ├── targets.json               ← FINAL COORDINATES        ← main deliverable
│   ├── fused_results.csv          ← per-target match details
│   ├── proof_hd/<target>.jpg      ← HD image deliverable
│   ├── lr_match/<target>.png      ← LR image deliverable (128×128)
│   └── visuals/<target>.jpg       ← reference vs detection, side by side
└── stage4_annotated/
    └── annotated_field.jpg        ← targets circled + labelled on the arena
```

---

## `targets.json` — the main deliverable

See [`sample_targets.json`](sample_targets.json). Each entry:

| Field | Meaning |
|---|---|
| `target` | Seed name it matched |
| `found` | `true` = confidently located; `false` = NOT FOUND (no guess made) |
| `method` | Matcher used (`stage3_robust`) |
| `drone_photo` | Which survey photo it was found in |
| `drone_pixel` | Pixel location inside that photo |
| **`map_xyz`** | **Final coordinates in metres, relative to the base station** |
| `origin_enu` | The VIO origin used (bookkeeping) |

---

## What a healthy terminal log looks like

```
[fix#12] stitch grid: 35/35 photos ko coordinates.csv se (row,col) mila
[fix#13] spatial pairing: 35/35 photos, 140 pairs (VIO 8-NN + temporal)
Stitched 35/35 photos
Mosaic: 2535x1573 px

[fix#8] fixed yellow OK (frac=0.043)
[fix#3] TRUE size 10.67 x 7.62 m  (w_px=2100 h_px=1500 -> width=35ft)
Field: 10.67 m x 7.62 m

Features: 3 | Drone photos: 35
[1] peak=0.31 V=0.72 → FOUND
[fix#1 base-origin] base station @ field (0.720,0.730) m -> subtracted

Target      Method          x(m)     y(m)     z(m)  Photo
1           stage3_robust   3.790    1.920    3.000  cp0011_r01c02.jpg
Found 3/3 targets
ALL STAGES DONE
```

---

## Visual reference

Example images from a real run live in [`../docs/images/`](../docs/images/):

| File | Stage |
|---|---|
| `orthomosaic.jpg` | 1 — stitched map |
| `yellow_mask_debug.jpg`, `yellow_corners_debug.jpg` | 2 — boundary detection |
| `rectified_field.jpg` | 2 — rectified arena |
| `3.png` | 3 — target matches in the dashboard |
| `annotated_field.jpg` | 4 — final annotated result |

---

## Try it without a drone

`make_test_dataset.py` builds a synthetic arena, a fake flight and known ground truth, so you can
verify the whole pipeline end-to-end:

```bash
python3 make_test_dataset.py
cp -r ~/advanced_matcher_testset/drone_photos ./
cp -r ~/advanced_matcher_testset/targets ./
python3 iroc_pipeline_fixed.py
cat ~/advanced_matcher_testset/ground_truth.txt      # compare against your results
```

If the reported coordinates land within ~0.3–0.4 m of the ground truth, your installation is
working correctly.
