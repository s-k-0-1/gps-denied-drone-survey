#!/usr/bin/env python3
"""
make_test_dataset.py -- SYNTHETIC test dataset for iroc_pipeline (KNOWN ground-truth)
==============================================================================
Ek naya (aapke data jaisa) dataset banata hai taaki pipeline ko NAYE arena pe test kar sako
aur accuracy GROUND-TRUTH se verify kar sako:
  * fake arena: reddish-brown ground (texture) + yellow border + 5 DISTINCT features + base station
  * drone grid survey (5x7 = 35 HD photos, boustrophedon, rotation + lighting variation)
  * coordinates.csv (VIO-style x/y/z/yaw, base station ~0,0)
  * targets/<feature>.png (LR references)
  * ground_truth.txt (har feature ki ASLI x,y -- verify ke liye)

RUN:
  python3 make_test_dataset.py
OUTPUT -> ~/advanced_matcher_testset/

TEST (apne data ka backup lekar):
  cd ~/advanced_matcher
  mv drone_photos drone_photos_MYDATA ; mv targets targets_MYDATA        # backup
  cp -r ~/advanced_matcher_testset/drone_photos ~/advanced_matcher/
  cp -r ~/advanced_matcher_testset/targets      ~/advanced_matcher/
  python3 iroc_pipeline_fixed.py                                         # full run
  cat ~/advanced_matcher_testset/ground_truth.txt                        # asli x,y se compare
  # wapas apna data:
  rm -rf drone_photos targets ; mv drone_photos_MYDATA drone_photos ; mv targets_MYDATA targets

NOTE: synthetic texture LoFTR (real-image trained) ke liye perfect nahi -- agar stitch weak ho to
woh synthetic-data limitation hai. Asli check: coords ground_truth ke ~0.3-0.4m me aayein.
(Pipeline origin = yellow INNER corner, GT = arena corner -> ~0.12m yellow-band offset expected.)
"""
import os, csv, math, random
import cv2
import numpy as np

random.seed(7); np.random.seed(7)

OUT = os.path.expanduser("~/advanced_matcher_testset")
PXM = 200                              # px per meter (arena canvas)
ARENA_W, ARENA_H = 9.14, 7.62          # meters (30 x 25 ft)
MARGIN = 1.2                           # ground beyond yellow (m)
ROWS, COLS = 5, 7                      # survey grid
ALT = 3.0                              # drone height (m)
YAW = -16.0                            # arena heading (like real data)
HD_W, HD_H = 1280, 720
GND_W, GND_H = 4.6, 2.6                # camera footprint (m) -> overlap

CW = int((ARENA_W + 2 * MARGIN) * PXM)
CH = int((ARENA_H + 2 * MARGIN) * PXM)


def m2px(ax, ay):
    """arena meters (base-corner origin, x=east/right, y=north/up) -> canvas pixel."""
    return (int((MARGIN + ax) * PXM), int((MARGIN + ARENA_H - ay) * PXM))


# RULEBOOK V4.0 (Final round, 11.3): 3 UNIQUE targets (one per seed). Arena me uss jaisa aur kuch nahi.
FEAT_TYPES = {
    "rock_pile": [(2.0, 5.8)],                   # 3 unique, 1 instance each
    "red_patch": [(5.2, 4.3)],
    "ice_patch": [(7.3, 2.0)],
}
FEAT_POS = [(t, ax, ay) for t, insts in FEAT_TYPES.items() for (ax, ay) in insts]  # patch-avoid


# ---------------- 1. GROUND texture (feature-rich, LoFTR ke liye) ----------------
def make_ground():
    img = np.full((CH, CW, 3), (120, 140, 165), np.uint8)          # reddish-brown BGR
    noise = np.zeros((CH, CW, 3), np.float32)
    for k in (3, 9, 21):                                           # multi-scale variation
        noise += cv2.GaussianBlur(np.random.randint(-32, 32, (CH, CW, 3)).astype(np.float32), (0, 0), k)
    img = np.clip(img.astype(np.float32) + noise, 0, 255).astype(np.uint8)
    for _ in range(1400):                                          # scattered stones (kam -> features stand out)
        x, y = random.randint(0, CW - 1), random.randint(0, CH - 1)
        g = random.randint(90, 205)
        cv2.circle(img, (x, y), random.randint(2, 5), (g, g, g), -1)
    # NOTE: color-patch clutter HATA diye -- Final round me targets UNIQUE hain (arena me uss jaisa
    # aur kuch nahi). Sirf grey-stone texture (LoFTR ke liye), koi target-jaisा distractor nahi.
    return cv2.GaussianBlur(img, (0, 0), 1.0)


canvas = make_ground()

# ---------------- 2. YELLOW border ----------------
cv2.rectangle(canvas, m2px(0, ARENA_H), m2px(ARENA_W, 0), (40, 210, 235), int(0.12 * PXM))

# ---------------- 3. DISTINCT features at KNOWN positions ----------------
def rock_pile(cx, cy):
    cv2.circle(canvas, (cx, cy), int(0.55 * PXM), (60, 65, 78), -1)     # dark base (pit/shadow) -> distinct
    for _ in range(85):                                                # DENSE varied-color stones
        a = random.uniform(0, 2 * math.pi); r = random.uniform(0, 0.45) * PXM
        x = int(cx + r * math.cos(a)); y = int(cy + r * 0.7 * math.sin(a)); s = random.randint(10, 26)
        col = random.choice([(110, 120, 140), (150, 155, 165), (205, 208, 216), (90, 110, 150)])
        cv2.fillPoly(canvas, [np.array([[x + random.randint(-s, s), y + random.randint(-s, s)] for _ in range(5)])], col)

def red_patch(cx, cy):                                             # red-orange (instance variation)
    cv2.circle(canvas, (cx, cy), random.randint(46, 56),
               (random.randint(50, 65), random.randint(85, 95), random.randint(185, 205)), -1)

def ice_patch(cx, cy):
    cv2.circle(canvas, (cx, cy), random.randint(42, 50), (240, 200, 150), -1)   # bright BLUE
    for _ in range(35):                                            # texture (highlights) -> DINOv2 signal
        x = cx + random.randint(-38, 38); y = cy + random.randint(-38, 38)
        cv2.circle(canvas, (x, y), random.randint(2, 6), (255, 240, 205), -1)

DRAW = {"rock_pile": rock_pile, "red_patch": red_patch, "ice_patch": ice_patch}
for tname, insts in FEAT_TYPES.items():                            # har type ke saare instances draw
    for (ax, ay) in insts:
        px, py = m2px(ax, ay); DRAW[tname](px, py)

# base station marker (bottom-left corner ~ 0,0)
bpx, bpy = m2px(0.25, 0.25)
cv2.rectangle(canvas, (bpx - 40, bpy - 40), (bpx + 40, bpy + 40), (245, 245, 245), -1)
cv2.rectangle(canvas, (bpx - 40, bpy - 40), (bpx + 40, bpy + 40), (150, 120, 60), 3)
cv2.drawMarker(canvas, (bpx, bpy), (180, 120, 40), cv2.MARKER_STAR, 40, 4)

# ---------------- 4. DRONE grid survey ----------------
os.makedirs(os.path.join(OUT, "drone_photos"), exist_ok=True)
os.makedirs(os.path.join(OUT, "targets"), exist_ok=True)
cw_px, ch_px = int(GND_W * PXM), int(GND_H * PXM)
rows_csv = []; idx = 0
for r in range(ROWS):
    cols = range(COLS) if r % 2 == 0 else range(COLS - 1, -1, -1)  # boustrophedon
    for c in cols:
        ax = (c + 0.5) / COLS * ARENA_W
        ay = (r + 0.5) / ROWS * ARENA_H
        cx, cy = m2px(ax, ay)
        M = cv2.getRotationMatrix2D((cx, cy), YAW + random.uniform(-2, 2), 1.0)
        rot = cv2.warpAffine(canvas, M, (CW, CH), borderMode=cv2.BORDER_REFLECT)
        x0, y0 = max(0, cx - cw_px // 2), max(0, cy - ch_px // 2)
        crop = rot[y0:y0 + ch_px, x0:x0 + cw_px]
        if crop.shape[0] < 10 or crop.shape[1] < 10:
            continue
        photo = cv2.resize(crop, (HD_W, HD_H))
        photo = np.clip(photo.astype(np.float32) * random.uniform(0.82, 1.18) + random.uniform(-15, 15), 0, 255).astype(np.uint8)
        fname = f"cp{idx:04d}_r{r:02d}c{c:02d}.jpg"
        cv2.imwrite(os.path.join(OUT, "drone_photos", fname), photo)
        rows_csv.append([idx, r, c, 1783127000 + idx * 8, round(ax, 3), round(-ay, 3),
                         round(ALT + random.uniform(-0.1, 0.1), 3), round(YAW, 1), fname])
        idx += 1

with open(os.path.join(OUT, "drone_photos", "coordinates.csv"), "w", newline="") as f:
    w = csv.writer(f)
    w.writerow(["checkpoint", "row", "col", "timestamp_s", "x_enu", "y_enu", "z_enu", "yaw_deg", "image_file"])
    w.writerows(rows_csv)

# ---------------- 5. Target references (1 per TYPE, 64x64 -- Final round spec) ----------------
for tname, insts in FEAT_TYPES.items():
    ax, ay = insts[0]                                              # seed = instance-1 ka crop
    px, py = m2px(ax, ay); s = int(0.6 * PXM)
    crop = canvas[max(0, py - s):py + s, max(0, px - s):px + s]
    cv2.imwrite(os.path.join(OUT, "targets", f"{tname}.png"), cv2.resize(crop, (64, 64)))  # 64x64 seed

# ---------------- 6. Ground truth (saare instances) ----------------
with open(os.path.join(OUT, "ground_truth.txt"), "w") as f:
    f.write("GROUND TRUTH (base-corner frame; x=east/right, y=north/up; meters)\n")
    f.write("origin (0,0) = arena bottom-left corner (base station ke paas)\n")
    f.write("3 UNIQUE targets (Final round 11.3). Pipeline har seed ka 1 best match report kare.\n\n")
    f.write(f"{'target':12s} {'x':>6s} {'y':>6s}\n")
    for tname, insts in FEAT_TYPES.items():
        ax, ay = insts[0]
        f.write(f"{tname:12s} {ax:6.2f} {ay:6.2f}\n")

cv2.imwrite(os.path.join(OUT, "arena_overview.jpg"), cv2.resize(canvas, (CW // 2, CH // 2)))

print(f"Done. {idx} drone photos -> {OUT}/drone_photos/")
print(f"      3 UNIQUE references (64x64) -> {OUT}/targets/")
print(f"      ground_truth.txt + arena_overview.jpg -> {OUT}/")
print("Test karne ke instructions script ke top comment me hain.")
