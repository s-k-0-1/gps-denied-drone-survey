#!/usr/bin/env python3
"""
IROC Combined Pipeline — iroc_pipeline.py
==========================================
Stage 1  Stitch map/ photos → orthomosaic  (LoFTR + bundle adjustment)
         Saved → results/stage1_stitch/

Stage 2  ENU calibration from drone_photos/coordinates.csv + yellow-boundary.
         Rectified top-down field view. Origin = BL corner (0, 0, 0).
         Saved → results/stage2_field/

Stage 3  Matching via fused_search.py  -> results/stage3_targets/ (csv + visuals + HD proof).
         NOTE: matching ab fused_search.py karta hai (DINOv2 semantic LR->LR). Yeh pipeline
         sirf map/coordinates/annotation ka kaam karta hai. Pehle `python3 fused_search.py`
         chalao, phir yeh.

After stages 1-3 complete → 3D reconstruction callback (requires Docker):
         OpenDroneMap on drone_photos/ → results/stage0_3d/

Usage:
    python3 iroc_pipeline.py                   # COMPLETE run: stitch + coords + FRESH matching + annotate
                                               #   (purane results -> results_archive/<timestamp>/)
    python3 iroc_pipeline.py --run-3d          # + 3D reconstruction
    python3 iroc_pipeline.py --skip-stitch     # fast: reuse mosaic, fresh match
    python3 iroc_pipeline.py --skip-match      # fastest: reuse mosaic + cached matches
    python3 iroc_pipeline.py --radius 2        # wider stitch (more pairs, slower)
"""

from __future__ import annotations

import argparse, csv, json, math, os, re, shutil, subprocess, sys, time
from collections import defaultdict
from pathlib import Path
from typing import Optional

import cv2
import numpy as np
import torch

# ═══════════════════════════════════════════════════════════════════════════════
# CONFIG
# ═══════════════════════════════════════════════════════════════════════════════

BASE_DIR    = Path.home() / "advanced_matcher"
DRONE_HD_DIR= BASE_DIR / "drone_photos"        # HD survey photos + coordinates.csv  (Stage 1/2/4 input)
DRONE_DIR   = BASE_DIR / "drone_photos_lr"     # 128 LR (fused_search banata; fallbacks)
MAP_DIR     = DRONE_HD_DIR                      # stitch HD -> sharp mosaic (Stage 1)
TARGETS_DIR = BASE_DIR / "targets"             # seed references (3-5 per feature) -> matching
CSV_PATH    = DRONE_HD_DIR / "coordinates.csv" # primary: drone_photos/coordinates.csv
RESULTS_DIR = BASE_DIR / "results"
FUSED_CSV   = BASE_DIR / "results" / "stage3_targets" / "fused_results.csv"   # fused_search ka output (results/ me)
STAGE3_SCRIPT = "fused_search.py"   # Stage-3 matcher (default). iroc_pipeline2.py isse stage3_robust.py karta hai.

# Stage sub-folders
STAGE3D_DIR = RESULTS_DIR / "stage0_3d"
STITCH_DIR  = RESULTS_DIR / "stage1_stitch"
FIELD_DIR   = RESULTS_DIR / "stage2_field"
TARGET_DIR  = RESULTS_DIR / "stage3_targets"
ANNOT_DIR   = RESULTS_DIR / "stage4_annotated"
HD_DIR      = RESULTS_DIR / "hd_targets"

# Stage 0 — OpenDroneMap settings (same as 3d.py)
ODM_USE_GPU      = True
ODM_FRESH_RUN    = True   # wipe old ODM project so new photos reprocess cleanly
ODM_IMAGE_EXTS   = (".jpg", ".jpeg", ".png", ".tif", ".tiff", ".bmp", ".dng", ".nef")
ODM_EXTRA_OPTS   = [
    "--feature-type",         "sift",
    "--dsm",
    "--orthophoto-resolution","5",
    "--skip-report",
    "--max-concurrency",      "4",      # limit CPU threads → less peak RAM
    "--mesh-size",            "100000", # smaller mesh (default 200000)
    "--skip-3dmodel",                   # skip texturing → saves RAM + avoids PoissonRecon bug
    "--pc-quality",           "low",    # low point cloud quality → less RAM
]

# Stitching
MATCH_W, MATCH_H  = 960, 544
GRID_RADIUS       = 1   # 1 = 4-directional neighbors (~84 pairs for 42 imgs, ~5 min)
                        # 2 = extended neighbors (~327 pairs, ~25 min)
MIN_INLIERS       = 30           # lowered for drone close-up photos (less overlap)
MIN_INLIER_RATIO  = 0.20         # lowered: LoFTR produces many matches, fewer are inliers
RANSAC_THRESH     = 5.0          # slightly looser RANSAC for varying drone altitudes
SCALE_LO, SCALE_HI = 0.40, 2.50 # wider: drone altitude varies more than map surveys
BA_REG_WEIGHT     = 0.15
CANVAS_PAD        = 60
JPEG_QUALITY      = 97
PREVIEW_MAX       = 3000

# Field map
PX_PER_M          = 150
FALLBACK_W_FT     = 30.0
FALLBACK_H_FT     = 25.0
FEET_TO_M         = 0.3048

# Target finder
TOP_K             = 10
PROC_W, PROC_H    = 640, 480
SP_SCALES         = [1.0, 0.6, 0.4, 0.25, 1.5, 2.0]
SP_ROTATIONS      = [0, 90, 180, 270]
SP_MIN_INLIERS    = 25
LO_SCALES         = [1.0, 0.6, 0.4]
LO_MIN_INLIERS    = 15
MAX_KP            = 4096
DRONE_HEIGHT      = 3.0   # nominal camera height above ground (metres)

# Camera FOV
FOV_H_DEG, FOV_V_DEG = 90.0, 65.0
GND_W = 2.0 * DRONE_HEIGHT * math.tan(math.radians(FOV_H_DEG / 2))
GND_H = 2.0 * DRONE_HEIGHT * math.tan(math.radians(FOV_V_DEG / 2))

# HD crop: fraction of drone image side around detected point
HD_CROP_PX   = 720          # HD crop output size (square, pixels)

DEVICE = "cuda" if torch.cuda.is_available() else "cpu"
EXTS   = {".jpg", ".jpeg", ".png", ".tif", ".tiff", ".bmp",
          ".JPG", ".JPEG", ".PNG", ".TIF", ".TIFF", ".BMP"}

TARGET_COLORS = [
    (0, 220, 50), (0, 200, 255), (50, 100, 255), (255, 80, 80),
    (200, 0, 200), (0, 240, 220), (80, 80, 255), (0, 180, 120),
]


def log(msg=""):
    print(msg, flush=True)


def mkdir(*dirs):
    for d in dirs:
        d.mkdir(parents=True, exist_ok=True)


# ═══════════════════════════════════════════════════════════════════════════════
# STAGE 1 — STITCHING
# ═══════════════════════════════════════════════════════════════════════════════

_RC_RE = re.compile(r"r(\d+)c(\d+)")


def discover(folder: Path):
    skip = {"mosaic", "loftr", "ortho", "preview", "stitch", "annotated"}
    entries, fb = [], 0
    for f in sorted(folder.rglob("*")):   # recursive — handles subdirectories
        if not f.is_file() or f.suffix.lower() not in EXTS:
            continue
        if any(f.name.lower().startswith(s) for s in skip):
            continue
        m = _RC_RE.search(f.name)
        r, c = (int(m.group(1)), int(m.group(2))) if m else (fb, 0)
        if not m:
            fb += 1
        entries.append((r, c, f))
    return sorted(entries)


def select_pairs(entries, radius: int = GRID_RADIUS):
    rc2i = {(r, c): i for i, (r, c, _) in enumerate(entries)}
    pairs: set = set()
    for i, (r, c, _) in enumerate(entries):
        for dr in range(-radius, radius + 1):
            for dc in range(-radius, radius + 1):
                if dr == dc == 0:
                    continue
                j = rc2i.get((r + dr, c + dc))
                if j is not None and j > i:
                    pairs.add((i, j))
    return sorted(pairs)


class LoFTRMatcher:
    def __init__(self):
        import kornia.feature as KF
        self.dev = torch.device(DEVICE)
        self.model = KF.LoFTR(pretrained="outdoor").eval().to(self.dev)
        self._cache = {}

    def _load(self, path):
        key = str(path)
        if key not in self._cache:
            img = cv2.imread(key, cv2.IMREAD_GRAYSCALE)
            if img is None:
                raise IOError(f"Cannot read {path}")
            oh, ow = img.shape
            res = cv2.resize(img, (MATCH_W, MATCH_H), interpolation=cv2.INTER_AREA)
            t = torch.from_numpy(res).float()[None, None].to(self.dev) / 255.0
            self._cache[key] = (t, MATCH_W / ow, MATCH_H / oh)
            if len(self._cache) > 24:
                del self._cache[next(iter(self._cache))]
        return self._cache[key]

    @torch.no_grad()
    def match(self, pa, pb):
        ta, sxa, sya = self._load(pa)
        tb, sxb, syb = self._load(pb)
        try:
            out = self.model({"image0": ta, "image1": tb})
        except Exception:
            return None
        k0 = out["keypoints0"].cpu().numpy().astype(np.float64)
        k1 = out["keypoints1"].cpu().numpy().astype(np.float64)
        if len(k0) < 20:
            return None
        k0[:, 0] /= sxa; k0[:, 1] /= sya
        k1[:, 0] /= sxb; k1[:, 1] /= syb
        aff, mask = cv2.estimateAffinePartial2D(
            k0, k1, method=cv2.RANSAC,
            ransacReprojThreshold=RANSAC_THRESH, maxIters=5000, confidence=0.999)
        if aff is None or mask is None:
            return None
        inl = mask.ravel().astype(bool)
        n_in = int(inl.sum())
        if n_in < MIN_INLIERS or n_in / len(k0) < MIN_INLIER_RATIO:
            return None
        sc = math.hypot(float(aff[0, 0]), float(aff[1, 0]))
        if not (SCALE_LO < sc < SCALE_HI):
            return None
        H = np.eye(3, dtype=np.float64); H[:2] = aff
        pts = [(k0[inl][k].tolist(), k1[inl][k].tolist()) for k in range(n_in)]
        return H, n_in, pts

    def release(self):
        self._cache.clear()
        if self.dev.type == "cuda":
            torch.cuda.empty_cache()


def decompose(H):
    return (float(H[0,2]), float(H[1,2]),
            math.atan2(float(H[1,0]), float(H[0,0])),
            math.hypot(float(H[0,0]), float(H[1,0])))


def compose(tx, ty, ang, sc):
    c = math.cos(ang) * sc; s = math.sin(ang) * sc
    return np.array([[c,-s,tx],[s,c,ty],[0,0,1]], dtype=np.float64)


def bfs_align(n, graph, anchor):
    T = {anchor: np.eye(3, dtype=np.float64)}
    q = [anchor]
    while q:
        cur = q.pop(0)
        for nxt, H_cn, _ in sorted(graph[cur], key=lambda x: -x[2]):
            if nxt in T:
                continue
            T[nxt] = T[cur] @ np.linalg.inv(H_cn)
            q.append(nxt)
    return T


def global_optimize(aligned, inlier_data, initial, anchor):
    from scipy.optimize import least_squares
    idx_list = sorted(aligned); n = len(idx_list)
    i2s = {idx: s for s, idx in enumerate(idx_list)}
    anc_s = i2s[anchor]; anc_H = initial[anchor].copy()
    free = [s for s in range(n) if s != anc_s]
    if not free:
        return initial
    s2p = {s: p for p, s in enumerate(free)}
    x0 = np.zeros(len(free) * 4)
    for p, s in enumerate(free):
        tx, ty, a, sc = decompose(initial[idx_list[s]])
        x0[p*4:p*4+4] = [tx, ty, a, math.log(max(sc, 1e-9))]
    constraints = []
    for (i, j), pts in inlier_data.items():
        if i not in i2s or j not in i2s or not pts:
            continue
        pa = np.asarray([p[0] for p in pts]); pb = np.asarray([p[1] for p in pts])
        constraints.append((i2s[i], i2s[j], pa, pb))
    if not constraints:
        return initial
    log(f"  BA: {len(constraints)} constraints | {len(free)} free images")

    def get_H(x, slot):
        if slot == anc_s:
            return anc_H
        b = s2p[slot] * 4
        return compose(x[b], x[b+1], x[b+2], math.exp(x[b+3]))

    def residuals(x):
        blocks = []
        for si, sj, pa, pb in constraints:
            Hi = get_H(x, si); Hj = get_H(x, sj)
            ones = np.ones((len(pa), 1))
            wa = (Hi @ np.hstack([pa, ones]).T)[:2].T
            wb = (Hj @ np.hstack([pb, ones]).T)[:2].T
            blocks.append((wa - wb).ravel())
        w = BA_REG_WEIGHT
        for p, s in enumerate(free):
            tx0, ty0, a0, s0 = decompose(initial[idx_list[s]])
            b = p * 4
            blocks.append(np.array([(x[b]-tx0)*w, (x[b+1]-ty0)*w,
                                     (x[b+2]-a0)*w*10,
                                     (x[b+3]-math.log(max(s0,1e-9)))*w*10]))
        return np.concatenate(blocks)

    res = least_squares(residuals, x0, method="lm",
                        max_nfev=5000*len(free), ftol=1e-14, xtol=1e-14, gtol=1e-14)
    log(f"  BA residual={res.cost:.2f}  evals={res.nfev}")
    opt = dict(initial); opt[anchor] = anc_H
    for p, s in enumerate(free):
        opt[idx_list[s]] = get_H(res.x, s)
    return opt


def detect_loop_closures(entries, aligned, matcher, graph, inlier_data):
    n = len(entries); K = min(5, max(1, n // 8)); new = False
    for i in range(K):
        for j in range(n - K, n):
            if i >= j or (i, j) in inlier_data:
                continue
            ri,ci,_ = entries[i]; rj,cj,_ = entries[j]
            if abs(ri-rj)+abs(ci-cj) > 8:
                continue
            r = matcher.match(entries[i][2], entries[j][2])
            if r:
                H,nin,pts = r
                graph[i].append((j,H,nin)); graph[j].append((i,np.linalg.inv(H),nin))
                inlier_data[(i,j)]=pts; new=True
    rc2i = {(r,c):i for i,(r,c,_) in enumerate(entries)}
    for i in aligned:
        ri,ci,_ = entries[i]
        for dr,dc in [(0,1),(1,0),(0,-1),(-1,0),(1,1),(1,-1),(-1,1),(-1,-1)]:
            j = rc2i.get((ri+dr,ci+dc))
            if j is None or j<=i or (i,j) in inlier_data:
                continue
            r = matcher.match(entries[i][2], entries[j][2])
            if r:
                H,nin,pts = r
                graph[i].append((j,H,nin)); graph[j].append((i,np.linalg.inv(H),nin))
                inlier_data[(i,j)]=pts; new=True
                log(f"  Gap fill: {entries[i][2].name} <-> {entries[j][2].name}: {nin}")
    return new


def compute_canvas(entries, aligned, transforms):
    all_c = []
    for i in aligned:
        img = cv2.imread(str(entries[i][2]))
        if img is None:
            continue
        h,w = img.shape[:2]
        pts = np.array([[0,0],[w,0],[w,h],[0,h]], np.float64)
        proj = (transforms[i] @ np.hstack([pts, np.ones((4,1))]).T)[:2].T
        all_c.append(proj)
    all_c = np.vstack(all_c)
    mn, mx = all_c.min(0), all_c.max(0)
    shift = np.array([[1,0,-mn[0]+CANVAS_PAD],[0,1,-mn[1]+CANVAS_PAD],[0,0,1]], np.float64)
    return shift, int(mx[0]-mn[0])+2*CANVAS_PAD, int(mx[1]-mn[1])+2*CANVAS_PAD


def compute_exposure_gains(entries, aligned, inlier_data):
    idx_list = sorted(aligned); n = len(idx_list)
    i2s = {idx:s for s,idx in enumerate(idx_list)}
    ratios = []
    for (i,j),pts in inlier_data.items():
        if i not in i2s or j not in i2s or len(pts)<20:
            continue
        ii=cv2.imread(str(entries[i][2])); ji=cv2.imread(str(entries[j][2]))
        if ii is None or ji is None:
            continue
        hi,wi=ii.shape[:2]; hj,wj=ji.shape[:2]
        vi,vj=[],[]
        for (xa,ya),(xb,yb) in pts[::max(1,len(pts)//80)]:
            xi,yi=int(round(xa)),int(round(ya)); xj,yj=int(round(xb)),int(round(yb))
            if 0<=xi<wi and 0<=yi<hi and 0<=xj<wj and 0<=yj<hj:
                a,b=float(ii[yi,xi].mean()),float(ji[yj,xj].mean())
                if a>15 and b>15:
                    vi.append(a); vj.append(b)
        if len(vi)>=10:
            ratios.append((i2s[i],i2s[j],float(np.median(vi))/float(np.median(vj))))
    if not ratios:
        return {i:1.0 for i in aligned}
    A=np.zeros((len(ratios)+1,n)); b=np.zeros(len(ratios)+1)
    for k,(si,sj,r) in enumerate(ratios):
        A[k,si]=1; A[k,sj]=-1; b[k]=math.log(max(r,1e-9))
    A[-1,:]=1.0
    lg,_,_,_=np.linalg.lstsq(A,b,rcond=None)
    g=np.clip(np.exp(-lg),0.65,1.55)
    return {idx_list[s]:float(g[s]) for s in range(n)}


def _weight_map(h, w):
    mask=np.zeros((h,w),np.uint8); b=max(1,int(min(h,w)*0.05))
    mask[b:h-b,b:w-b]=255
    d=cv2.distanceTransform(mask,cv2.DIST_L2,5).astype(np.float32)
    mx=d.max()
    return (d/mx)**1.5 if mx>0 else d


def warp_and_blend(entries, aligned, transforms, shift, cw, ch, gains=None):
    if gains is None:
        gains={}
    canvas=np.zeros((ch,cw,3),np.uint8); best=np.full((ch,cw),-1.0,np.float32)
    for step,i in enumerate(aligned):
        img=cv2.imread(str(entries[i][2]))
        if img is None:
            continue
        h,w=img.shape[:2]
        g=gains.get(i,1.0)
        if abs(g-1.0)>0.01:
            img=np.clip(img.astype(np.float32)*g,0,255).astype(np.uint8)
        wt=_weight_map(h,w); H=shift@transforms[i]
        wr=cv2.warpPerspective(img,H,(cw,ch),flags=cv2.INTER_LANCZOS4,borderMode=cv2.BORDER_CONSTANT,borderValue=0)
        ww=cv2.warpPerspective(wt,H,(cw,ch),flags=cv2.INTER_LINEAR,borderMode=cv2.BORDER_CONSTANT,borderValue=0)
        better=ww>best; canvas[better]=wr[better]; best[better]=ww[better]
        if (step+1)%10==0 or step+1==len(aligned):
            log(f"    Warped {step+1}/{len(aligned)}")
    return canvas, best>0


def sharpen(img, r=1.5, a=0.60):
    blur=cv2.GaussianBlur(img.astype(np.float32),(0,0),r)
    return np.clip(img.astype(np.float32)+a*(img.astype(np.float32)-blur),0,255).astype(np.uint8)


def run_stitching(map_dir: Path, radius: int = GRID_RADIUS) -> tuple[Path, dict, np.ndarray]:
    """
    Stitch → orthomosaic. Returns (mosaic_path, photo_to_H, mosaic_bgr).
    photo_to_H[basename] = 3x3 matrix mapping photo pixel → saved mosaic pixel.
    All outputs saved to STITCH_DIR.
    """
    mkdir(STITCH_DIR)
    log("\n" + "="*62)
    log("  STAGE 1 -- Stitching  →  " + str(STITCH_DIR.relative_to(BASE_DIR)))
    log("="*62)

    entries = discover(map_dir)
    n = len(entries)
    if n < 2:
        raise RuntimeError(f"Need >=2 images in {map_dir}, found {n}")
    log(f"  {n} images")

    pairs = select_pairs(entries, radius)
    log(f"  {len(pairs)} pairs (radius {radius})")

    log("  Loading LoFTR ...")
    matcher = LoFTRMatcher()
    log(f"  Device: {DEVICE}")

    graph: dict = defaultdict(list)
    inlier_data: dict = {}
    ok = fail = 0
    log_lines = []
    t_pair_start = time.time()
    for p_idx, (i, j) in enumerate(pairs):
        ri,ci,pi = entries[i]; rj,cj,pj = entries[j]
        res = matcher.match(pi, pj)
        elapsed = time.time() - t_pair_start
        avg_s   = elapsed / (p_idx + 1)
        eta_s   = avg_s * (len(pairs) - p_idx - 1)
        eta_str = f"{int(eta_s//60)}m{int(eta_s%60):02d}s"
        if res:
            H,nin,pts = res
            graph[i].append((j,H,nin)); graph[j].append((i,np.linalg.inv(H),nin))
            inlier_data[(i,j)]=pts; ok+=1
            line = (f"    [{p_idx+1:3d}/{len(pairs)}] [{ri:02d},{ci:02d}]<->[{rj:02d},{cj:02d}]"
                    f"  {nin:4d} inliers  ETA {eta_str}")
            log(line); log_lines.append(line)
        else:
            fail+=1
            log(f"    [{p_idx+1:3d}/{len(pairs)}] [{ri:02d},{ci:02d}]<->[{rj:02d},{cj:02d}]"
                f"  SKIP  ETA {eta_str}")
    log(f"  Matched {ok}/{ok+fail} pairs")
    if ok==0:
        raise RuntimeError("No pairs matched.")

    cr = np.mean([r for r,c,_ in entries])
    cc = np.mean([c for r,c,_ in entries])
    anchor = min(range(n), key=lambda i: (entries[i][0]-cr)**2+(entries[i][1]-cc)**2)
    log(f"  Anchor: {entries[anchor][2].name}")

    transforms = bfs_align(n, graph, anchor)
    aligned = sorted(transforms)
    log(f"  BFS aligned {len(aligned)}/{n}")

    log("  Bundle adjustment ...")
    optimized = global_optimize(aligned, inlier_data, transforms, anchor)

    log("  Loop closure ...")
    if detect_loop_closures(entries, aligned, matcher, graph, inlier_data):
        t2 = bfs_align(n, graph, anchor); a2 = sorted(t2)
        if len(a2) >= len(aligned):
            aligned = a2
            optimized = global_optimize(aligned, inlier_data, t2, anchor)
    matcher.release()

    gains = compute_exposure_gains(entries, aligned, inlier_data)
    shift, cw, ch = compute_canvas(entries, aligned, optimized)
    log(f"  Canvas: {cw}x{ch} px")

    log(f"  Warping {len(aligned)} images ...")
    mosaic, coverage = warp_and_blend(entries, aligned, optimized, shift, cw, ch, gains=gains)

    ys, xs = np.where(coverage)
    pad = 10
    y0 = max(0, int(ys.min()) - pad); x0 = max(0, int(xs.min()) - pad)
    y1 = int(ys.max()) + pad + 1;     x1 = int(xs.max()) + pad + 1
    mosaic = sharpen(mosaic[y0:y1, x0:x1])

    mosaic_path = STITCH_DIR / "orthomosaic.jpg"
    cv2.imwrite(str(mosaic_path), mosaic, [cv2.IMWRITE_JPEG_QUALITY, JPEG_QUALITY])
    log(f"  Saved: {mosaic_path.name}  ({mosaic.shape[1]}x{mosaic.shape[0]} px)")

    if max(mosaic.shape[:2]) > PREVIEW_MAX:
        s = PREVIEW_MAX / max(mosaic.shape[:2])
        prev = cv2.resize(mosaic, None, fx=s, fy=s, interpolation=cv2.INTER_AREA)
        cv2.imwrite(str(STITCH_DIR/"orthomosaic_preview.jpg"), prev, [cv2.IMWRITE_JPEG_QUALITY,90])

    # Build photo_to_H: photo pixel → cropped mosaic pixel
    crop_shift = np.array([[1,0,-x0],[0,1,-y0],[0,0,1]], dtype=np.float64)
    photo_to_H: dict[str, np.ndarray] = {}
    for i in aligned:
        _, _, path = entries[i]
        photo_to_H[path.name] = crop_shift @ shift @ optimized[i]
    log(f"  photo_to_H: {len(photo_to_H)} photos")

    # Save stitch log
    with open(STITCH_DIR/"stitch_log.txt","w") as f:
        f.write(f"Stitched {len(aligned)}/{n} photos\n")
        f.write(f"Mosaic: {mosaic.shape[1]}x{mosaic.shape[0]} px\n")
        f.write(f"Anchor: {entries[anchor][2].name}\n\n")
        f.write("\n".join(log_lines))

    return mosaic_path, photo_to_H, mosaic


# ═══════════════════════════════════════════════════════════════════════════════
# STAGE 2 — FIELD MAP + ENU CALIBRATION
# Origin = bottom-left yellow corner (0, 0, ground_z)
# ═══════════════════════════════════════════════════════════════════════════════

def load_csv_locations(csv_path: Path) -> list[dict]:
    rows = []
    with csv_path.open(newline="", encoding="utf-8-sig") as f:
        for row in csv.DictReader(f):
            try:
                rows.append({
                    "image_file": row["image_file"],
                    "x": float(row["x_enu"]),
                    "y": float(row["y_enu"]),
                    "z": float(row.get("z_enu", DRONE_HEIGHT)),
                    "yaw": float(row.get("yaw_deg", 0.0)),
                })
            except (TypeError, ValueError, KeyError):
                continue
    return rows


def _sift_match_photo_to_mosaic(photo_bgr: np.ndarray,
                                 mosaic_bgr: np.ndarray,
                                 return_H: bool = False) -> Optional[tuple]:
    """
    SIFT-match a raw photo to the stitched mosaic.

    return_H=False  →  (mosaic_cx, mosaic_cy, n_inliers)
    return_H=True   →  (mosaic_cx, mosaic_cy, n_inliers, H)
                        where H maps photo_pixel → mosaic_pixel (full resolution).

    Returns None if matching fails.
    Ported from field_map.py:match_photo_center_to_mosaic().
    """
    # Downscale both images for faster descriptor extraction
    MAX_SIDE = 1400
    ph, pw = photo_bgr.shape[:2]
    mh, mw = mosaic_bgr.shape[:2]
    ps = min(1.0, MAX_SIDE / max(ph, pw))
    ms = min(1.0, MAX_SIDE / max(mh, mw))
    p_sm = cv2.resize(photo_bgr,  (int(pw*ps), int(ph*ps)))  if ps < 1 else photo_bgr
    m_sm = cv2.resize(mosaic_bgr, (int(mw*ms), int(mh*ms))) if ms < 1 else mosaic_bgr

    pg = cv2.cvtColor(p_sm, cv2.COLOR_BGR2GRAY)
    mg = cv2.cvtColor(m_sm, cv2.COLOR_BGR2GRAY)

    if hasattr(cv2, "SIFT_create"):
        det  = cv2.SIFT_create(nfeatures=4000)
        norm = cv2.NORM_L2
    else:
        det  = cv2.ORB_create(nfeatures=6000)
        norm = cv2.NORM_HAMMING

    kp_p, des_p = det.detectAndCompute(pg, None)
    kp_m, des_m = det.detectAndCompute(mg, None)
    if des_p is None or des_m is None or len(kp_p) < 12 or len(kp_m) < 12:
        return None

    raw = cv2.BFMatcher(norm).knnMatch(des_p, des_m, k=2)
    good = [m for pair in raw if len(pair) == 2
            for m, n in [pair] if m.distance < 0.75 * n.distance]
    if len(good) < 10:
        return None

    src = np.float32([kp_p[m.queryIdx].pt for m in good]).reshape(-1,1,2)
    dst = np.float32([kp_m[m.trainIdx].pt for m in good]).reshape(-1,1,2)
    H_sm, mask = cv2.findHomography(src, dst, cv2.RANSAC, 5.0)
    if H_sm is None or mask is None or int(mask.ravel().sum()) < 8:
        return None

    # Scale H back to full-resolution coordinates:
    #   photo_full_pt  → photo_small_pt  : multiply by ps
    #   mosaic_small_pt → mosaic_full_pt : divide by ms
    S_in  = np.diag([ps,    ps,    1.0])   # photo full → small
    S_out = np.diag([1/ms,  1/ms,  1.0])   # mosaic small → full
    H_full = S_out @ H_sm @ S_in

    ctr = cv2.perspectiveTransform(
        np.array([[[pw*0.5, ph*0.5]]], np.float32), H_full)[0, 0]
    if not (0 <= ctr[0] < mw and 0 <= ctr[1] < mh):
        return None

    n_inliers = int(mask.ravel().sum())
    if return_H:
        return float(ctr[0]), float(ctr[1]), n_inliers, H_full
    return float(ctr[0]), float(ctr[1]), n_inliers


def _find_target_in_mosaic_tmpl(
        target_bgr: np.ndarray,
        mosaic_bgr: np.ndarray,
        min_conf: float = 0.28,
) -> Optional[tuple]:
    """
    Multi-scale normalised cross-correlation (TM_CCOEFF_NORMED) to find a
    target template inside the mosaic.  Works even when the target occupies
    only ~20×20 px in the mosaic (SIFT fails at that scale).

    Returns (mosaic_cx, mosaic_cy, confidence) or None.
    """
    tg = cv2.cvtColor(target_bgr,  cv2.COLOR_BGR2GRAY)
    mg = cv2.cvtColor(mosaic_bgr, cv2.COLOR_BGR2GRAY)
    th, tw = tg.shape
    mh, mw = mg.shape

    # Scales cover the range from 'tiny in mosaic' to 'roughly same size'
    scales = [0.03, 0.05, 0.07, 0.10, 0.14, 0.20, 0.28, 0.40, 0.55]
    best_val, best_cx, best_cy = 0.0, 0.0, 0.0

    for s in scales:
        t_w = max(8, int(tw * s))
        t_h = max(8, int(th * s))
        if t_w >= mw or t_h >= mh:
            continue
        t_sc = cv2.resize(tg, (t_w, t_h), interpolation=cv2.INTER_AREA)
        res  = cv2.matchTemplate(mg, t_sc, cv2.TM_CCOEFF_NORMED)
        _, val, _, loc = cv2.minMaxLoc(res)
        if val > best_val:
            best_val = val
            best_cx  = float(loc[0] + t_w // 2)
            best_cy  = float(loc[1] + t_h // 2)

    if best_val >= min_conf:
        return best_cx, best_cy, best_val
    return None


def _find_target_in_map_photos(
        target_bgr: np.ndarray,
        map_dir: Path,
        photo_to_H: dict,
        min_conf: float = 0.30,
) -> Optional[tuple]:
    """
    Search for a target in every MAP photo that has a stitch transform.
    Map photos are at the same altitude as the survey, so the scale ratio
    between the target template and each map photo is much closer than
    mosaic-wide search.

    Returns (mosaic_cx, mosaic_cy, matched_photo_name, confidence) or None.
    """
    tg   = cv2.cvtColor(target_bgr, cv2.COLOR_BGR2GRAY)
    th, tw = tg.shape
    scales = [0.10, 0.18, 0.30, 0.45, 0.65, 0.90, 1.20, 1.60]

    best_val, best_cx, best_cy, best_name = 0.0, 0.0, 0.0, ""

    for bname, H in photo_to_H.items():
        photo_path = map_dir / bname
        if not photo_path.exists():
            continue
        photo_bgr = cv2.imread(str(photo_path))
        if photo_bgr is None:
            continue
        ph, pw = photo_bgr.shape[:2]
        pg = cv2.cvtColor(photo_bgr, cv2.COLOR_BGR2GRAY)

        for s in scales:
            t_w = max(8, int(tw * s))
            t_h = max(8, int(th * s))
            if t_w >= pw or t_h >= ph:
                continue
            t_sc = cv2.resize(tg, (t_w, t_h), interpolation=cv2.INTER_AREA)
            res  = cv2.matchTemplate(pg, t_sc, cv2.TM_CCOEFF_NORMED)
            _, val, _, loc = cv2.minMaxLoc(res)
            if val > best_val:
                px = float(loc[0] + t_w // 2)
                py = float(loc[1] + t_h // 2)
                # Project photo pixel → mosaic via stitch transform
                pt = H @ np.array([px, py, 1.0])
                best_val  = val
                best_cx   = float(pt[0])
                best_cy   = float(pt[1])
                best_name = bname

    if best_val >= min_conf:
        return best_cx, best_cy, best_name, best_val
    return None


def _find_target_in_mosaic_sift(
        target_bgr: np.ndarray,
        mosaic_bgr: np.ndarray,
        scales: Optional[list] = None,
        min_inliers: int = 8,
) -> Optional[tuple]:
    """
    Multi-scale SIFT search: find a target template directly inside the mosaic.
    Used when drone-photo-to-mosaic registration fails (large scale gap between
    close-up drone shot and wide-area mosaic).

    Returns (mosaic_cx, mosaic_cy, n_inliers) or None.
    """
    if scales is None:
        # Try shrinking the template to match expected apparent size in mosaic
        scales = [0.10, 0.18, 0.28, 0.40, 0.55, 0.75, 1.0]

    # Downscale mosaic once for speed
    MAX_M = 2000
    mh, mw = mosaic_bgr.shape[:2]
    ms = min(1.0, MAX_M / max(mh, mw))
    m_sm = cv2.resize(mosaic_bgr, (int(mw * ms), int(mh * ms))) if ms < 1 else mosaic_bgr
    mg = cv2.cvtColor(m_sm, cv2.COLOR_BGR2GRAY)

    if hasattr(cv2, "SIFT_create"):
        det  = cv2.SIFT_create(nfeatures=4000)
        norm = cv2.NORM_L2
    else:
        det  = cv2.ORB_create(nfeatures=4000)
        norm = cv2.NORM_HAMMING

    kp_m, des_m = det.detectAndCompute(mg, None)
    if des_m is None or len(kp_m) < 8:
        return None

    th, tw = target_bgr.shape[:2]
    best_inl, best_cx, best_cy = 0, 0.0, 0.0

    for s in scales:
        t_sc = cv2.resize(target_bgr, (max(32, int(tw * s)), max(32, int(th * s))))
        tg   = cv2.cvtColor(t_sc, cv2.COLOR_BGR2GRAY)
        kp_t, des_t = det.detectAndCompute(tg, None)
        if des_t is None or len(kp_t) < 4:
            continue

        raw  = cv2.BFMatcher(norm).knnMatch(des_t, des_m, k=2)
        good = [m for pair in raw if len(pair) == 2
                for m, n in [pair] if m.distance < 0.75 * n.distance]
        if len(good) < 6:
            continue

        src = np.float32([kp_t[m.queryIdx].pt for m in good]).reshape(-1, 1, 2)
        dst = np.float32([kp_m[m.trainIdx].pt for m in good]).reshape(-1, 1, 2)
        H, mask = cv2.findHomography(src, dst, cv2.RANSAC, 5.0)
        if H is None or mask is None:
            continue
        n_inl = int(mask.sum())
        if n_inl < min_inliers:
            continue

        tsh, tsw = t_sc.shape[:2]
        ctr = cv2.perspectiveTransform(
            np.array([[[tsw * 0.5, tsh * 0.5]]], np.float32), H)[0, 0]
        if not (0 <= ctr[0] < int(mw * ms) and 0 <= ctr[1] < int(mh * ms)):
            continue

        if n_inl > best_inl:
            best_inl = n_inl
            best_cx  = float(ctr[0]) / ms   # back to full-mosaic coords
            best_cy  = float(ctr[1]) / ms

    if best_inl >= min_inliers:
        return best_cx, best_cy, best_inl
    return None


def _select_spread_photos(csv_rows: list[dict], image_folder: Path,
                           max_pts: int = 12) -> list[dict]:
    """
    Pick a well-spread subset of photos for calibration:
    corners + centre + farthest-point sampling (same strategy as field_map.py).
    """
    rows = [r for r in csv_rows if (image_folder / r["image_file"]).exists()]
    if len(rows) <= max_pts:
        return rows
    coords = np.array([[r["x"], r["y"]] for r in rows], np.float64)
    seeds = np.array([
        [coords[:,0].min(), coords[:,1].min()],
        [coords[:,0].min(), coords[:,1].max()],
        [coords[:,0].max(), coords[:,1].min()],
        [coords[:,0].max(), coords[:,1].max()],
        [coords[:,0].mean(), coords[:,1].mean()],
    ])
    picked = []
    for t in seeds:
        idx = int(np.argmin(np.sum((coords - t)**2, axis=1)))
        if idx not in picked:
            picked.append(idx)
    while len(picked) < max_pts:
        pc = coords[picked]
        d2 = np.min(np.sum((coords[:,None,:] - pc[None,:,:])**2, axis=2), axis=1)
        for i in picked:
            d2[i] = -1.0
        picked.append(int(np.argmax(d2)))
    return [rows[i] for i in picked]


def calibrate_enu_sift(mosaic_bgr: np.ndarray,
                       csv_rows: list[dict],
                       image_folder: Path) -> Optional[np.ndarray]:
    """
    SIFT-based pixel→ENU calibration (method from field_map.py).

    For each control photo:
      1. SIFT-match the raw photo against the stitched mosaic.
      2. Map the photo's centre to a mosaic pixel via the fitted homography.
      3. Pair that mosaic pixel with the photo's GPS ENU (x, y).
    Fit affine  A: [mosaic_px, mosaic_py, 1] → [enu_x, enu_y].

    This is the same method used by survey_coordinates_pipeline.py and field_map.py.
    """
    control = _select_spread_photos(csv_rows, image_folder, max_pts=12)
    log(f"  SIFT ENU calibration: {len(control)} control photos from {image_folder.name}/")

    pixel_pts, enu_pts = [], []
    for row in control:
        photo = cv2.imread(str(image_folder / row["image_file"]))
        if photo is None:
            continue
        res = _sift_match_photo_to_mosaic(photo, mosaic_bgr)
        if res is None:
            log(f"    {row['image_file']}: no match")
            continue
        mx, my, inl = res
        pixel_pts.append([mx, my])
        enu_pts.append([row["x"], row["y"]])
        log(f"    {row['image_file']}: mosaic=({mx:.0f},{my:.0f})  "
            f"ENU=({row['x']:.3f},{row['y']:.3f})  [{inl} inliers]")

    log(f"  Matched {len(pixel_pts)}/{len(control)} control points")
    if len(pixel_pts) < 2:
        log("  [WARN] Too few SIFT matches — ENU calibration failed")
        return None

    px_arr = np.array(pixel_pts, np.float64)
    en_arr = np.array(enu_pts,   np.float64)

    if len(px_arr) >= 3:
        A, _ = cv2.estimateAffine2D(px_arr, en_arr, method=cv2.RANSAC,
                                    ransacReprojThreshold=0.40, confidence=0.99)
        if A is not None:
            log("  Affine fit (full, RANSAC)")
            return A.astype(np.float64)

    A, _ = cv2.estimateAffinePartial2D(px_arr, en_arr, method=cv2.RANSAC,
                                       ransacReprojThreshold=0.40, confidence=0.99)
    if A is not None:
        log("  Affine fit (similarity, RANSAC)")
        return A.astype(np.float64)

    return None


def order_corners(pts):
    pts = pts[np.argsort(pts[:, 1])]
    top = pts[:2][np.argsort(pts[:2, 0])]
    bot = pts[2:][np.argsort(pts[2:, 0])]
    return np.array([top[0], top[1], bot[1], bot[0]], np.float32)  # TL TR BR BL


def detect_yellow_corners(img_bgr):
    """Arena yellow boundary -> 4 ACTUAL corners. Mask Lab color-space se (b*=yellow HIGH, a*=red LOW)
    -> faded/muted tape bhi pakadta + reddish-brown terrain reject. Phir convex-hull + approxPolyDP se
    4 asli corner (arena mosaic me perspective se TRAPEZOID hota hai -> rectangle fit galat, quad sahi).
    Returns (corners TL/TR/BR/BL, tape_width_px)."""
    H, W = img_bgr.shape[:2]
    hsv = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2HSV)
    lab = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2Lab)
    h, s, v = cv2.split(hsv)
    L, A, B = cv2.split(lab)
    diff = B.astype(np.int16) - A.astype(np.int16)                 # b*-a*: yellow BADA (+), red-brown chhota
    # YELLOW = hue yellow-band + saturation + (b* clearly > a*). Lighting-robust; faded tape bhi.
    mask = (((h >= 14) & (h <= 48)) & (s >= 28) & (diff >= 15)).astype(np.uint8) * 255
    ko = max(3, int(min(H, W) * 0.003))                             # chhota OPEN -> terrain speckles hatao
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (ko, ko)))
    k = max(15, int(min(H, W) * 0.03))                              # bada CLOSE -> tape gaps bridge
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (k, k)))
    try:
        cv2.imwrite(str(FIELD_DIR / "yellow_mask_debug.jpg"), mask)  # DEBUG: mask verify (clean tape?)
    except Exception:
        pass
    n, lbl, stats, _ = cv2.connectedComponentsWithStats(mask)
    if n <= 1:
        return None
    # Tape TOOTI-PHOOTI (fragments) hai -> ek component chhota nikalta tha. Isliye SAARE decent yellow
    # fragments ke points JODO (sirf chhoti noise hatao). Ye fragments milke poora arena border banate
    # hain -> min-area-rect unpe fit karke L/W/angle recover karta (1 corner missing ho tab bhi kaam).
    areas = stats[1:, cv2.CC_STAT_AREA].astype(np.float64)
    amax = float(areas.max()) if len(areas) else 0.0
    keep = [i + 1 for i, a in enumerate(areas) if a >= max(40.0, 0.02 * amax)]  # noise specks EXCLUDE
    if not keep:
        return None
    ys, xs = np.where(np.isin(lbl, keep))
    if len(xs) < 4:
        return None
    pts = np.column_stack([xs, ys]).astype(np.int32)               # (x, y) saare arena-frame yellow points
    # ---- ACTUAL 4 corners (rectangle NAHI): arena mosaic me perspective/stitch se TRAPEZOID hota hai.
    #      Convex-hull -> approxPolyDP se 4 asli corner nikaalo -> perspective warp chaaron side yellow pe
    #      align karega (min-area-rect sirf 1-2 side fit karta, baaki bahar nikal jaate the). ----
    hull = cv2.convexHull(pts)
    peri_h = cv2.arcLength(hull, True)
    quad = None
    for ef in (0.02, 0.03, 0.04, 0.05, 0.06, 0.08, 0.10):
        ap = cv2.approxPolyDP(hull, ef * peri_h, True)
        if len(ap) == 4:
            quad = ap.reshape(4, 2).astype(np.float32)             # 4 asli outer corners (trapezoid ok)
            break
    if quad is None:                                               # fallback: rectangle fit
        quad = cv2.boxPoints(cv2.minAreaRect(pts.astype(np.float32))).astype(np.float32)
    box = order_corners(quad)                                      # TL, TR, BR, BL
    # TAPE WIDTH (px) = yellow band area / outer perimeter -> inner edge inset ke liye
    peri = sum(float(np.linalg.norm(box[i] - box[(i + 1) % 4])) for i in range(4))
    tape_w_px = float(len(xs)) / max(peri, 1.0)
    return box, tape_w_px


def compute_origin(corners: np.ndarray, A_px2enu: Optional[np.ndarray],
                   csv_rows: list[dict]) -> tuple[float, float, float]:
    """
    Return (origin_x, origin_y, origin_z) in ENU — the BL pixel corner.

    Matches field_map.py exactly:
        corner_enu = [A @ [pt[0], pt[1], 1] for pt in corners]  # corners = [TL,TR,BR,BL]
        shifted = corner_enu - corner_enu[3]                     # BL pixel = index 3 = origin

    So origin_x, origin_y = ENU of corners[3] (the bottom-left pixel corner of yellow tape).

    origin_z = median ground elevation = median(drone_z - DRONE_HEIGHT).
    z values reported are drone altitude above that ground reference (~3 m).

    Note: for annotation, x and y are now derived from M_persp (perspective warp) → rectified
    pixel / PX_PER_M, matching the field_map.py viewer click coordinates exactly.
    origin_x, origin_y are logged/saved but not used in that path.
    """
    # Ground z reference: median drone altitude minus camera height
    ground_zs = [r["z"] - DRONE_HEIGHT for r in csv_rows if r.get("z") is not None]
    origin_z = float(np.median(ground_zs)) if ground_zs else 0.0

    if A_px2enu is not None and corners is not None:
        # Convert all 4 corners [TL, TR, BR, BL] to ENU
        enu_corners = []
        for pt_px in corners:
            pt = A_px2enu @ np.array([pt_px[0], pt_px[1], 1.0])
            enu_corners.append((float(pt[0]), float(pt[1])))
        # BL pixel corner = index 3 (same as field_map.py: shifted = corner_enu - corner_enu[3])
        bl = enu_corners[3]
        log(f"  4 corners ENU [TL,TR,BR,BL]: {[f'({p[0]:.2f},{p[1]:.2f})' for p in enu_corners]}")
        log(f"  BL pixel corner (origin): ({bl[0]:.3f}, {bl[1]:.3f})")
        return float(bl[0]), float(bl[1]), origin_z

    # Fallback: use minimum ENU extents from CSV
    if csv_rows:
        xs = [r["x"] for r in csv_rows]
        ys = [r["y"] for r in csv_rows]
        return float(min(xs)), float(min(ys)), origin_z

    return 0.0, 0.0, origin_z


def setup_field_map(mosaic_bgr: np.ndarray, photo_to_H: dict,
                    csv_rows: list[dict]) -> tuple:
    """
    Returns (A_px2enu, corners, field_w_m, field_h_m, rect_bgr, M_persp,
             origin_x, origin_y, origin_z).
    All field-map outputs saved to FIELD_DIR.
    """
    mkdir(FIELD_DIR)
    log("\n" + "="*62)
    log("  STAGE 2 -- Field map  →  " + str(FIELD_DIR.relative_to(BASE_DIR)))
    log("="*62)

    # SIFT-based calibration (same method as survey_coordinates_pipeline / field_map.py)
    # Matches raw photos to the mosaic to get precise mosaic_pixel → ENU control points
    A = calibrate_enu_sift(mosaic_bgr, csv_rows, MAP_DIR)
    if A is None:
        log("  [WARN] SIFT calibration on map/ failed, trying drone_photos/ ...")
        A = calibrate_enu_sift(mosaic_bgr, csv_rows, DRONE_DIR)

    log("  Detecting yellow boundary ...")
    det = detect_yellow_corners(mosaic_bgr)
    corners, tape_w_px = det if det is not None else (None, 0.0)
    field_w_m = field_h_m = None

    # ── YELLOW-INNER rectification + BASE-STATION origin ─────────────────────────────
    #  * Base station = ENU (0,0) (drone home/takeoff; CSV ka pehla point ~0,0). A_inv se usko
    #    mosaic pixel me le jao -> jo yellow corner uske SABSE PAAS wahi ORIGIN (0,0) = BL.
    #  * Corners ko NON-MIRROR roll: base station -> BL (winding preserve, koi flip/mirror nahi).
    #  * Yellow line ke ANDAR ka part: outer corners ko tape-width jitna ANDAR inset -> inner edge.
    MIN_FIELD_M = 2.0
    MARGIN_M = 0.5                                          # sirf fallback (yellow fail) ke liye
    A_ok = A is not None
    src = None; origin_x = origin_y = 0.0
    if corners is not None and A_ok and len(corners) == 4:
        oc = order_corners(corners.astype(np.float32)).astype(np.float64)   # image TL,TR,BR,BL (CW winding)
        A_inv = cv2.invertAffineTransform(A.astype(np.float64))
        base_px = A_inv @ np.array([0.0, 0.0, 1.0])                         # ENU(0,0)=base station -> pixel
        bi = int(np.argmin([np.linalg.norm(oc[k] - base_px) for k in range(4)]))
        oc = np.roll(oc, (3 - bi) % 4, axis=0)                              # base station -> BL (idx3), winding same
        # yellow ke ANDAR: har corner ko apne DONO adjacent edges ke along tape-width andar khiscao
        # (per-corner -> trapezoid me bhi accurate).
        tw = float(tape_w_px)
        def _inset(cur, a, b):
            an = (a - cur) / (np.linalg.norm(a - cur) + 1e-6)
            bn = (b - cur) / (np.linalg.norm(b - cur) + 1e-6)
            return cur + an * tw + bn * tw
        px = np.array([_inset(oc[0], oc[1], oc[3]), _inset(oc[1], oc[0], oc[2]),   # inner TL, TR
                       _inset(oc[2], oc[1], oc[3]), _inset(oc[3], oc[2], oc[0])],  # inner BR, BL
                      np.float64)
        eu = (A @ np.hstack([px, np.ones((4, 1))]).T).T                    # inner corners -> ENU
        fw = 0.5 * (np.linalg.norm(eu[1]-eu[0]) + np.linalg.norm(eu[2]-eu[3]))
        fh = 0.5 * (np.linalg.norm(eu[3]-eu[0]) + np.linalg.norm(eu[2]-eu[1]))
        if fw >= MIN_FIELD_M and fh >= MIN_FIELD_M:
            src = px.astype(np.float32)
            field_w_m, field_h_m = fw, fh
            origin_x, origin_y = float(eu[3][0]), float(eu[3][1])          # inner BL = base station = (0,0)
    if src is None:                                        # yellow bad -> GPS axis-aligned bbox
        gx = [r["x"] for r in csv_rows]; gy = [r["y"] for r in csv_rows]
        if A_ok and gx and gy and (max(gx)-min(gx)) >= MIN_FIELD_M and (max(gy)-min(gy)) >= MIN_FIELD_M:
            e0, e1 = min(gx)-MARGIN_M, max(gx)+MARGIN_M; n0, n1 = min(gy)-MARGIN_M, max(gy)+MARGIN_M
            field_w_m, field_h_m = e1-e0, n1-n0
            A_inv = cv2.invertAffineTransform(A.astype(np.float64))
            eb = np.array([[e0, n1], [e1, n1], [e1, n0], [e0, n0]], np.float64)
            src = (A_inv @ np.hstack([eb, np.ones((4, 1))]).T).T.astype(np.float32)
            origin_x, origin_y = float(e0), float(n0)
            log("  [yellow weak] GPS axis-aligned field")
        else:
            field_w_m, field_h_m = FALLBACK_W_FT*FEET_TO_M, FALLBACK_H_FT*FEET_TO_M
            hm, wm = mosaic_bgr.shape[:2]
            src = np.float32([[0, 0], [wm, 0], [wm, hm], [0, hm]])
            log("  Fallback field")
    corners = src
    # DEBUG: rectification quad (red) + inner corners (green) + base station (magenta) mosaic pe
    try:
        dbg = mosaic_bgr.copy()
        cv2.polylines(dbg, [src.astype(np.int32)], True, (0, 0, 255), 5)
        for i, c in enumerate(src):
            cv2.circle(dbg, (int(c[0]), int(c[1])), 14, (0, 255, 0), -1)
            cv2.putText(dbg, ["TL", "TR", "BR", "BL"][i], (int(c[0]) + 18, int(c[1])),
                        cv2.FONT_HERSHEY_SIMPLEX, 1.6, (0, 255, 0), 4)
        if A_ok:                                            # base station (ENU 0,0) -> pixel = magenta 'ORIGIN'
            bp = cv2.invertAffineTransform(A.astype(np.float64)) @ np.array([0.0, 0.0, 1.0])
            cv2.circle(dbg, (int(bp[0]), int(bp[1])), 22, (255, 0, 255), 4)
            cv2.putText(dbg, "BASE(0,0)", (int(bp[0]) + 24, int(bp[1]) + 8),
                        cv2.FONT_HERSHEY_SIMPLEX, 1.6, (255, 0, 255), 4)
        cv2.imwrite(str(FIELD_DIR / "yellow_corners_debug.jpg"), dbg, [cv2.IMWRITE_JPEG_QUALITY, 82])
    except Exception:
        pass
    log(f"  Field: {field_w_m:.2f} m x {field_h_m:.2f} m  (yellow-inner, base=origin)")

    out_w = max(1, int(round(field_w_m * PX_PER_M)))
    out_h = max(1, int(round(field_h_m * PX_PER_M)))
    dst = np.float32([[0, 0], [out_w, 0], [out_w, out_h], [0, out_h]])
    M = cv2.getPerspectiveTransform(src.astype(np.float32), dst)
    rect = cv2.warpPerspective(mosaic_bgr, M, (out_w, out_h), flags=cv2.INTER_LANCZOS4)
    log(f"  Rectified: {out_w}x{out_h} px")

    ground_zs = [r["z"] - DRONE_HEIGHT for r in csv_rows if r.get("z") is not None]
    origin_z = float(np.median(ground_zs)) if ground_zs else 0.0
    log(f"  Origin (base station, BL): ({origin_x:.3f}, {origin_y:.3f}, {origin_z:.3f}) ENU")

    # Save field map + calibration info
    cv2.imwrite(str(FIELD_DIR/"rectified_field.jpg"), rect, [cv2.IMWRITE_JPEG_QUALITY,95])
    with open(FIELD_DIR/"calibration.txt","w") as f:
        f.write(f"Field size: {field_w_m:.3f} m x {field_h_m:.3f} m\n")
        f.write(f"Origin (yellow corner, BL) ENU: ({origin_x:.4f}, {origin_y:.4f}, {origin_z:.4f})\n")
        f.write(f"px_per_m: {PX_PER_M}\n")
        if corners is not None:
            for label, pt in zip(["TL","TR","BR","BL"], corners):
                f.write(f"  {label} mosaic px: ({pt[0]:.0f}, {pt[1]:.0f})\n")

    return A, corners, field_w_m, field_h_m, rect, M, origin_x, origin_y, origin_z


# ═══════════════════════════════════════════════════════════════════════════════
# STAGE 3 — TARGET FINDING
# ═══════════════════════════════════════════════════════════════════════════════

def target_xyz(cx, cy, iw, ih, loc, origin_x, origin_y, origin_z):
    """
    Drone photo pixel (cx, cy) → (x, y, z) metres relative to SW field corner.

    x, y = lateral ENU offset from origin (should both be positive inside the field).
    z    = drone altitude above ground reference  ≈ 3 m  (varies per photo, NOT fixed).
           Formula: loc["z"] - origin_z  where origin_z ≈ ground elevation.
    """
    if not loc or loc.get("x") is None:
        return None
    off_e = (cx - iw/2.0) * (GND_W / iw)
    off_n = -(cy - ih/2.0) * (GND_H / ih)
    yaw = loc.get("yaw")
    if yaw is not None:
        th = math.radians(yaw); c,s = math.cos(th),math.sin(th)
        re = off_e*c - off_n*s; rn = off_e*s + off_n*c
    else:
        re, rn = off_e, off_n

    abs_x = loc["x"] + re
    abs_y = loc["y"] + rn
    # z = drone altitude (from GPS) minus ground reference
    # This gives the flight altitude above ground ≈ 3 m, varying per photo
    abs_z = loc["z"] - origin_z

    return {
        "x": round(abs_x - origin_x, 3),
        "y": round(abs_y - origin_y, 3),
        "z": round(abs_z, 3),
    }


def run_target_finding(csv_rows: list[dict],
                       origin_x: float, origin_y: float, origin_z: float) -> list[dict]:
    """MATCHING fused_search.py karta hai (DINOv2 semantic LR->LR) -- yahin se chal jaata hai.
    Yeh fused_search.run() call karta hai (matching), phir uske results (fused_results.csv) ko iroc
    format me badalta hai (coordinates + Stage 4 ke liye). Photo names STEM se map (HD .jpg <-> LR .png);
    HD pixel coords use hote hain (Stage 1 HD mosaic ke saath consistent)."""
    mkdir(TARGET_DIR)
    log("\n" + "="*62)
    log("  STAGE 3 -- Matching via fused_search.py  →  " + str(TARGET_DIR.relative_to(BASE_DIR)))
    log("="*62)

    # 1) fused_search ka matching -> results/stage3_targets/ (FRESH har baar = complete run).
    #    Reuse/fast chahiye to:  python3 iroc_pipeline.py --skip-match
    fs_script = Path(__file__).resolve().parent / STAGE3_SCRIPT
    if fs_script.exists():
        log(f"  Running matching FRESH (subprocess): {fs_script.name} ...")
        r = subprocess.run([sys.executable, str(fs_script)], cwd=str(BASE_DIR))
        log(f"  fused_search.py finished (exit code {r.returncode})")
    else:
        log(f"  [ERROR] {fs_script} not found")

    if not FUSED_CSV.exists():
        log(f"  [WARN] {FUSED_CSV} missing -- matching nahi hua. drone_photos/ + targets/ check karo.")
        return []

    # csv GPS by STEM (ext-agnostic); HD photos by stem (Stage 1 HD mosaic ke saath consistent)
    loc_map    = {Path(r["image_file"]).stem: r for r in csv_rows}
    hd_by_stem = {p.stem: p for p in DRONE_HD_DIR.iterdir()
                  if p.is_file() and p.suffix.lower() in EXTS} if DRONE_HD_DIR.exists() else {}

    with open(FUSED_CSV, newline="") as f:
        rows = list(csv.DictReader(f))
    log(f"  {len(rows)} matches from {FUSED_CSV.name}  (visuals/proof_hd same folder me)")

    results = []; n_ok = 0
    for row in rows:
        tname  = (row.get("feature") or "").strip()
        mphoto = (row.get("matched_photo") or "").strip()
        if not tname or not mphoto:
            results.append({"target": tname, "found": False, "object_xyz": None}); continue
        stem = Path(mphoto).stem
        hd_path = hd_by_stem.get(stem)
        if hd_path is not None:
            himg = cv2.imread(str(hd_path))
            hw, hh = (himg.shape[1], himg.shape[0]) if himg is not None else (1280, 720)
            dphoto_name = hd_path.name                      # HD name -> photo_to_H (HD mosaic) match
        else:
            hw, hh = 1280, 720; dphoto_name = mphoto

        # HD pixel: CSV ke hx_hd/hy_hd (best) ya LR fraction se HD me convert
        try: hx = float(row.get("hx_hd", -1)); hy = float(row.get("hy_hd", -1))
        except (TypeError, ValueError): hx = hy = -1.0
        if hx < 0 or hy < 0:
            try:
                hx = float(row["cx_lr"]) / PROC_W * hw; hy = float(row["cy_lr"]) / PROC_H * hh
            except (TypeError, ValueError, KeyError):
                results.append({"target": tname, "found": False, "object_xyz": None}); continue

        loc = loc_map.get(stem, {})
        obj = target_xyz(hx, hy, hw, hh, loc, origin_x, origin_y, origin_z)
        conf = row.get("confidence", "")
        log(f"  {tname[:22]:22s} -> {dphoto_name:20s} px=({int(hx)},{int(hy)}) xyz={obj} [{conf}]")

        n_ok += 1
        results.append({"target": tname, "found": True, "method": "fused_search",
                        "drone_photo": dphoto_name,
                        "drone_pixel": [float(hx), float(hy)],
                        "drone_photo_wh": [int(hw), int(hh)],
                        "loftr_inliers": int(float(row.get("loftr") or 0)),
                        "sp_inliers": int(float(row.get("superpoint") or 0)),
                        "object_xyz": obj})

    log(f"\n  Loaded {n_ok}/{len(rows)} targets from fused_search")
    return results


# ═══════════════════════════════════════════════════════════════════════════════
# STAGE 4 — ANNOTATED MAP
# Direct stitch-transform mapping: H @ [cx, cy, 1] = mosaic pixel
# ═══════════════════════════════════════════════════════════════════════════════

def _text_box(img, text, org, fscale=0.50, thick=1, fg=(255,255,255), bg=(0,0,0)):
    font = cv2.FONT_HERSHEY_SIMPLEX
    (tw, th), bl = cv2.getTextSize(text, font, fscale, thick)
    x, y = int(org[0]), int(org[1])
    pad = 3
    cv2.rectangle(img, (x-pad, y-th-pad), (x+tw+pad, y+bl+pad), bg, -1)
    cv2.putText(img, text, (x,y), font, fscale, fg, thick, cv2.LINE_AA)
    return tw, th


def _place_label(cx, cy, r, tw, th, used, iw, ih):
    pad = 6
    candidates = [
        (cx+r+pad, cy-th//2),
        (cx-tw//2, cy-r-th-pad),
        (cx-tw//2, cy+r+pad),
        (cx-r-tw-pad, cy-th//2),
    ]
    for lx, ly in candidates:
        lx = int(max(2, min(lx, iw-tw-4)))
        ly = int(max(th+2, min(ly, ih-4)))
        if not any(lx<ux+uw+4 and lx+tw+4>ux and ly<uy+uh+4 and ly+th+4>uy
                   for ux,uy,uw,uh in used):
            return lx, ly
    if used:
        _, my, _, mh = max(used, key=lambda t: t[1]+t[3])
        return int(max(2, min(cx-tw//2, iw-tw-4))), my+mh+6
    return int(candidates[0][0]), int(candidates[0][1])


def annotate_target(img, cx, cy, radius, label, color, used_labels):
    cx, cy = int(round(cx)), int(round(cy))
    ih, iw = img.shape[:2]
    cv2.circle(img, (cx,cy), radius, color, 3, cv2.LINE_AA)
    cv2.circle(img, (cx,cy), 5, color, -1, cv2.LINE_AA)
    font = cv2.FONT_HERSHEY_SIMPLEX
    (tw, th), _ = cv2.getTextSize(label, font, 0.48, 1)
    lx, ly = _place_label(cx, cy, radius, tw, th, used_labels, iw, ih)
    ldx, ldy = lx+tw//2-cx, ly-th//2-cy
    dist = max(1, math.hypot(ldx, ldy))
    ex = int(cx + ldx/dist*radius); ey = int(cy + ldy/dist*radius)
    cv2.line(img, (ex,ey), (lx+tw//2, ly-th//2), (120,120,120), 1, cv2.LINE_AA)
    _text_box(img, label, (lx, ly), fscale=0.48, thick=1)
    used_labels.append((lx, ly-th, tw, th+4))


def draw_grid(img, field_w_m, field_h_m, px_per_m, ox=0.0, oy=None):
    """1m grid. Origin pixel (ox, oy) se — default BL corner (ox=0, oy=bottom). Base station
    origin ke liye (ox, oy) = base station rectified pixel; labels meters me (base ke aaspaas negative bhi)."""
    h, w = img.shape[:2]; gc = (55, 55, 55)
    if oy is None:
        oy = float(h)
    mx0 = int(math.ceil((0 - ox) / px_per_m)); mx1 = int(math.floor((w - ox) / px_per_m))
    for mm in range(mx0, mx1 + 1):
        px = int(round(ox + mm * px_per_m))
        cv2.line(img, (px, 0), (px, h), gc, 1)
        _text_box(img, f"{mm}m", (px + 2, 14), fscale=0.30, fg=(130, 130, 130))
    my0 = int(math.ceil((oy - h) / px_per_m)); my1 = int(math.floor(oy / px_per_m))
    for nn in range(my0, my1 + 1):
        py = int(round(oy - nn * px_per_m))
        cv2.line(img, (0, py), (w, py), gc, 1)
        _text_box(img, f"{nn}m", (3, py - 2), fscale=0.30, fg=(130, 130, 130))


def compute_arena_axes(csv_rows):
    """Arena ke x/y axes (ENU me) drone YAW se — no hardcode, kisi bhi arena pe chalta hai.
    Drone grid arena ke edges ke saath aligned udta hai; mean yaw = arena rotation. target_xyz
    bhi isi yaw se pixel-offset ko ENU me rotate karta -> consistent. Axes ko field ke andar
    (photo centroid ki taraf) orient karta hai taaki coords positive aayें (base station origin)."""
    yaws = [r.get("yaw") for r in csv_rows if r.get("yaw") is not None]
    if not yaws:
        return None, None
    th = math.radians(float(np.mean(yaws)))
    x_hat = np.array([math.cos(th), math.sin(th)])          # arena x-axis (photo local x) in ENU
    y_hat = np.array([-math.sin(th), math.cos(th)])         # arena y-axis (perpendicular) in ENU
    cx = float(np.mean([r["x"] for r in csv_rows]))
    cy = float(np.mean([r["y"] for r in csv_rows]))
    c = np.array([cx, cy])                                  # photo centroid (field ke andar) ENU, base=~0
    if np.dot(c, x_hat) < 0:
        x_hat = -x_hat
    if np.dot(c, y_hat) < 0:
        y_hat = -y_hat
    return x_hat, y_hat


def compute_map_coords(results, mosaic_bgr, photo_to_H, M_persp,
                       field_h_m, csv_rows, origin_z, A_px2enu=None,
                       origin_x=0.0, origin_y=0.0):
    """
    For each found target, project its drone pixel → mosaic pixel → rectified field pixel
    → (x, y, z) in metres from BL corner.

    Fallback chain (same as run_annotation):
      1. Stitch transform H (photo_to_H)
      2. On-the-fly SIFT drone→mosaic
      3. SIFT template search in mosaic
      4. cv2.matchTemplate in mosaic
      5. Template match in each MAP photo → project via stitch H

    Returns dict: target_name → {"x": float, "y": float, "z": float}
    """
    log("\n  [map-coords] Computing field x,y,z (VIO/odometry-anchored, arena axes) ...")
    loc_map = {os.path.basename(r["image_file"]): r for r in csv_rows}
    _H_cache: dict = {}
    coords: dict = {}
    x_hat, y_hat = compute_arena_axes(csv_rows)             # arena axes ENU (yaw se, no hardcode)
    if x_hat is not None:
        log(f"    arena axes: x_hat=({x_hat[0]:.3f},{x_hat[1]:.3f}) "
            f"y_hat=({y_hat[0]:.3f},{y_hat[1]:.3f})  (yaw-derived)")

    # BASE STATION = ENU (0,0) (drone home; CSV). Uska rectified-field coord nikaal ke har target se
    # SUBTRACT -> origin exactly base station (yellow inner corner nahi). Yahi ~50-60cm offset fix karta.
    bs_fx = bs_fy = 0.0
    if A_px2enu is not None:
        try:
            A_inv = cv2.invertAffineTransform(np.asarray(A_px2enu, np.float64))
            bpx = A_inv @ np.array([0.0, 0.0, 1.0])                  # ENU(0,0) -> mosaic pixel
            bf = cv2.perspectiveTransform(
                np.array([[[float(bpx[0]), float(bpx[1])]]], np.float32), M_persp)[0, 0]
            bs_fx = float(bf[0]) / PX_PER_M
            bs_fy = field_h_m - float(bf[1]) / PX_PER_M
            log(f"    base station field offset: ({bs_fx:.3f}, {bs_fy:.3f}) m  -> subtracted")
        except Exception:
            bs_fx = bs_fy = 0.0

    for res in results:
        if not res.get("found"):
            continue

        tname       = res["target"]
        drone_photo = res["drone_photo"]
        cx, cy      = res["drone_pixel"]

        # ── 1. Stitch transform ───────────────────────────────────────────────
        H = photo_to_H.get(drone_photo)
        if H is None:
            H = photo_to_H.get(os.path.basename(drone_photo))

        # ── 2. On-the-fly SIFT drone→mosaic ──────────────────────────────────
        if H is None:
            bname = os.path.basename(drone_photo)
            if bname not in _H_cache:
                dp = DRONE_HD_DIR / bname
                if not dp.exists():
                    dp = Path(drone_photo)
                dbgr = cv2.imread(str(dp)) if dp.exists() else None
                if dbgr is not None:
                    r2 = _sift_match_photo_to_mosaic(dbgr, mosaic_bgr, return_H=True)
                    _H_cache[bname] = r2[3] if r2 is not None else None
                else:
                    _H_cache[bname] = None
            H = _H_cache.get(bname)

        mpx = mpy = None

        if H is not None:
            pt = H @ np.array([cx, cy, 1.0])
            mpx, mpy = float(pt[0]), float(pt[1])
        else:
            # ── 3–5. Template-based fallbacks ─────────────────────────────────
            tpath = TARGETS_DIR / (tname + ".jpg")
            if not tpath.exists():
                for ext in (".png", ".jpeg", ".JPG", ".PNG"):
                    alt = TARGETS_DIR / (tname + ext)
                    if alt.exists():
                        tpath = alt; break

            if tpath.exists():
                timg = cv2.imread(str(tpath))
                if timg is not None:
                    # Method 3: SIFT template in mosaic
                    r3 = _find_target_in_mosaic_sift(timg, mosaic_bgr)
                    if r3 is not None:
                        mpx, mpy = r3[0], r3[1]
                    else:
                        # Method 4: cv2.matchTemplate in mosaic
                        r4 = _find_target_in_mosaic_tmpl(timg, mosaic_bgr)
                        if r4 is not None:
                            mpx, mpy = r4[0], r4[1]
                        else:
                            # Method 5: template in MAP photos
                            r5 = _find_target_in_map_photos(timg, MAP_DIR, photo_to_H)
                            if r5 is not None:
                                mpx, mpy = r5[0], r5[1]

        if mpx is None:
            log(f"    {tname}: all methods failed — using GPS estimate")
            # Fall back to Stage 3 GPS-based object_xyz if available
            gps_xyz = res.get("object_xyz")
            if gps_xyz:
                coords[tname] = gps_xyz
            continue

        # Project mosaic pixel → rectified field pixel → metres from BL corner
        fpt = cv2.perspectiveTransform(
            np.array([[[float(mpx), float(mpy)]]], np.float32), M_persp)[0, 0]
        rx, ry = float(fpt[0]), float(fpt[1])           # rectified pixel (VISUAL circle position)
        loc = loc_map.get(os.path.basename(drone_photo), {})

        # ── GPS-ANCHORED arena coords (metric, accurate) ──────────────────────────
        #  target ENU = source photo GPS + (pixel offset -> meters, yaw se rotate)  [mosaic bypass]
        #  phir arena axes (x_hat,y_hat) pe project -> base station (ENU 0,0) origin, arena-aligned.
        iw, ih = (res.get("drone_photo_wh") or [None, None])
        gx = None
        if x_hat is not None and loc and iw and ih:
            te = target_xyz(cx, cy, iw, ih, loc, 0.0, 0.0, origin_z)   # raw target ENU
            if te is not None:
                # origin = YELLOW LINE ka corner (origin_x, origin_y = yellow inner BL ENU)
                relv = np.array([te["x"] - origin_x, te["y"] - origin_y], float)
                gx = float(relv @ x_hat); gy = float(relv @ y_hat); gz = float(te["z"])

        if gx is not None:
            xyz = {"x": round(gx, 3), "y": round(gy, 3), "z": round(gz, 3),
                   "_rx": rx, "_ry": ry}
        else:                                            # fallback: purana mosaic method
            abs_z = loc.get("z", DRONE_HEIGHT + origin_z)
            xyz = {"x": round(rx/PX_PER_M - bs_fx, 3),
                   "y": round((field_h_m - ry/PX_PER_M) - bs_fy, 3),
                   "z": round(abs_z - origin_z, 3), "_rx": rx, "_ry": ry}
        coords[tname] = xyz
        log(f"    {tname}: mosaic=({int(mpx)},{int(mpy)})  "
            f"x={xyz['x']:.3f} y={xyz['y']:.3f} z={xyz['z']:.3f}")

    return coords


def run_annotation(mosaic_bgr, rect_bgr, photo_to_H, M_persp,
                   A_px2enu, field_w_m, field_h_m, results, csv_rows,
                   origin_x, origin_y, origin_z):
    """
    x, y: M_persp projects mosaic_px → rectified image pixel; divide by PX_PER_M.
          y flipped (image y↓, field y↑) to match field_map.py viewer convention.
          Equivalent to survey_coordinates_pipeline click coords with origin at BL corner.
    z:    loc["z"] - origin_z  (drone GPS altitude above ground reference, ~3 m).
    Coords always recomputed fresh — --skip-match safe.
    """
    mkdir(ANNOT_DIR)
    log("\n" + "="*62)
    log("  STAGE 4 -- Annotation  →  " + str(ANNOT_DIR.relative_to(BASE_DIR)))
    log("="*62)
    log(f"  x,y method: M_persp → rect_pixel / PX_PER_M  (matches field_map.py viewer)")
    log(f"  z   method: loc_z - origin_z  (drone GPS altitude above ground, ~3 m)")
    log(f"  Origin (BL corner): origin_z={origin_z:.3f} m ENU")

    loc_map = {os.path.basename(r["image_file"]): r for r in csv_rows}

    ann_m = mosaic_bgr.copy()
    ann_f = rect_bgr.copy()
    mh, mw = ann_m.shape[:2]
    fh, fw = ann_f.shape[:2]

    R_MOSAIC = max(20, int(min(mh,mw)*0.018))
    R_FIELD  = max(15, int(0.22*PX_PER_M))

    draw_grid(ann_f, field_w_m, field_h_m, float(PX_PER_M))
    cv2.rectangle(ann_f, (0,0),(fw-1,fh-1),(0,220,220),2)

    _text_box(ann_f, "(0,0)", (6, fh-8), fscale=0.40, fg=(0,255,200), bg=(0,0,0))
    cv2.circle(ann_f, (0, fh-1), 10, (0,255,200), 3)

    used_m, used_f = [], []
    found_count = 0
    computed_xyz = {}           # target_name → xyz dict (for summary)
    _drone_H_cache: dict = {}   # basename → H (photo→mosaic) from on-the-fly SIFT

    for idx, res in enumerate(results):
        if not res.get("found"):
            continue

        drone_photo = res["drone_photo"]
        cx, cy = res["drone_pixel"]
        color = TARGET_COLORS[idx % len(TARGET_COLORS)]

        # ── Step 1: get H mapping drone/map photo pixel → mosaic ──────────────
        # Priority: (a) stitch transform from Stage 1, (b) on-the-fly SIFT match
        H = photo_to_H.get(drone_photo)
        if H is None:
            H = photo_to_H.get(os.path.basename(drone_photo))

        if H is None:
            bname = os.path.basename(drone_photo)
            if bname not in _drone_H_cache:
                # Try to load the drone photo and SIFT-register it to the mosaic
                drone_path = DRONE_HD_DIR / bname
                if not drone_path.exists():
                    drone_path = Path(drone_photo)
                drone_bgr = cv2.imread(str(drone_path)) if drone_path.exists() else None
                if drone_bgr is not None:
                    log(f"  [SIFT-reg] Matching '{bname}' to mosaic ...")
                    result = _sift_match_photo_to_mosaic(
                        drone_bgr, mosaic_bgr, return_H=True)
                    _drone_H_cache[bname] = result[3] if result is not None else None
                    if result is not None:
                        log(f"    -> {result[2]} inliers  OK")
                    else:
                        log(f"    -> SIFT registration FAILED for '{bname}'")
                else:
                    log(f"  [WARN] {res['target']}: drone photo '{bname}' not found on disk")
                    _drone_H_cache[bname] = None
            H = _drone_H_cache.get(bname)

        if H is None:
            # ── Fallback 3: search the target template directly in the mosaic ──
            # Drone photos are close-ups; scale gap makes drone→mosaic SIFT
            # unreliable.  Searching the target reference image in the mosaic
            # avoids that problem entirely.
            tpath = TARGETS_DIR / (res["target"] + ".jpg")
            if not tpath.exists():
                # Try common extensions
                for ext in (".png", ".jpeg", ".JPG", ".PNG"):
                    alt = TARGETS_DIR / (res["target"] + ext)
                    if alt.exists():
                        tpath = alt
                        break
            if tpath.exists():
                timg = cv2.imread(str(tpath))
                if timg is not None:
                    log(f"  [tmpl-search] Searching '{res['target']}' template in mosaic ...")
                    tmpl_res = _find_target_in_mosaic_sift(timg, mosaic_bgr)
                    if tmpl_res is not None:
                        mpx_t, mpy_t, t_inl = tmpl_res
                        log(f"    -> found in mosaic at ({int(mpx_t)},{int(mpy_t)})  "
                            f"{t_inl} inliers")
                        # Project through M_persp directly (bypass H)
                        fpt_t = cv2.perspectiveTransform(
                            np.array([[[mpx_t, mpy_t]]], np.float32), M_persp)[0, 0]
                        rx_t, ry_t = float(fpt_t[0]), float(fpt_t[1])
                        field_x = rx_t / PX_PER_M
                        field_y = field_h_m - ry_t / PX_PER_M
                        loc    = loc_map.get(os.path.basename(drone_photo), {})
                        abs_z  = loc.get("z", DRONE_HEIGHT + origin_z)
                        xyz = {"x": round(field_x, 3),
                               "y": round(field_y, 3),
                               "z": round(abs_z - origin_z, 3)}
                        computed_xyz[res["target"]] = xyz
                        label = f"x={xyz['x']:.2f} y={xyz['y']:.2f} z={xyz['z']:.2f}m"
                        log(f"  {res['target']:28s}  mosaic=({int(mpx_t)},{int(mpy_t)})  {label}")
                        if 0 <= mpx_t < mw and 0 <= mpy_t < mh:
                            annotate_target(ann_m, mpx_t, mpy_t, R_MOSAIC, label, color, used_m)
                        if 0 <= rx_t < fw and 0 <= ry_t < fh:
                            annotate_target(ann_f, rx_t, ry_t, R_FIELD, label, color, used_f)
                        found_count += 1
                        continue   # done for this target
                    else:
                        log(f"    -> SIFT template: no match")
                        # ── Method 4: multi-scale template matching in mosaic ──
                        log(f"  [tmpl-match] cv2.matchTemplate in mosaic ...")
                        tm_res = _find_target_in_mosaic_tmpl(timg, mosaic_bgr)
                        if tm_res is not None:
                            mpx_t, mpy_t, conf = tm_res
                            log(f"    -> mosaic=({int(mpx_t)},{int(mpy_t)})  conf={conf:.3f}")
                        else:
                            # ── Method 5: search each MAP photo individually ──
                            log(f"  [map-search] Template matching in MAP photos ...")
                            mp_res = _find_target_in_map_photos(
                                timg, MAP_DIR, photo_to_H)
                            if mp_res is not None:
                                mpx_t, mpy_t, mp_name, conf = mp_res
                                log(f"    -> found in '{mp_name}'  "
                                    f"mosaic=({int(mpx_t)},{int(mpy_t)})  conf={conf:.3f}")
                            else:
                                mpx_t = None
                                log(f"    -> map-photo search: no match")

                        if mpx_t is not None:
                            fpt_t = cv2.perspectiveTransform(
                                np.array([[[mpx_t, mpy_t]]], np.float32), M_persp)[0, 0]
                            rx_t, ry_t = float(fpt_t[0]), float(fpt_t[1])
                            field_x = rx_t / PX_PER_M
                            field_y = field_h_m - ry_t / PX_PER_M
                            loc    = loc_map.get(os.path.basename(drone_photo), {})
                            abs_z  = loc.get("z", DRONE_HEIGHT + origin_z)
                            xyz = {"x": round(field_x, 3),
                                   "y": round(field_y, 3),
                                   "z": round(abs_z - origin_z, 3)}
                            computed_xyz[res["target"]] = xyz
                            label = f"x={xyz['x']:.2f} y={xyz['y']:.2f} z={xyz['z']:.2f}m"
                            log(f"  {res['target']:28s}  mosaic=({int(mpx_t)},{int(mpy_t)})  {label}")
                            if 0 <= mpx_t < mw and 0 <= mpy_t < mh:
                                annotate_target(ann_m, mpx_t, mpy_t, R_MOSAIC, label, color, used_m)
                            if 0 <= rx_t < fw and 0 <= ry_t < fh:
                                annotate_target(ann_f, rx_t, ry_t, R_FIELD, label, color, used_f)
                            found_count += 1
                            continue
            log(f"  [WARN] {res['target']}: all registration methods failed -- skip")
            continue

        pt_h = H @ np.array([cx, cy, 1.0])
        mpx, mpy = float(pt_h[0]), float(pt_h[1])

        # ── Step 2: x, y via M_persp → rectified image pixel → metres from BL corner ──
        # Identical to survey_coordinates_pipeline.py / field_map.py viewer click coords:
        #   x_m = rect_px / PX_PER_M
        #   y_m = field_h_m - rect_py / PX_PER_M   (flip: image y↓, field y↑)
        fpt = cv2.perspectiveTransform(np.array([[[mpx, mpy]]], np.float32), M_persp)[0, 0]
        rx, ry = float(fpt[0]), float(fpt[1])
        field_x = rx / PX_PER_M
        field_y = field_h_m - ry / PX_PER_M

        # ── Step 3: z from drone GPS altitude ──────────────────────────────
        loc = loc_map.get(os.path.basename(drone_photo), {})
        abs_z = loc.get("z", DRONE_HEIGHT + origin_z)

        xyz = {
            "x": round(field_x, 3),
            "y": round(field_y, 3),
            "z": round(abs_z - origin_z, 3),
        }

        computed_xyz[res["target"]] = xyz
        label = f"x={xyz['x']:.2f} y={xyz['y']:.2f} z={xyz['z']:.2f}m"
        log(f"  {res['target']:28s}  mosaic=({int(mpx)},{int(mpy)})  {label}")

        if 0 <= mpx < mw and 0 <= mpy < mh:
            annotate_target(ann_m, mpx, mpy, R_MOSAIC, label, color, used_m)

        # Draw circle on rectified field map (rx, ry already computed above)
        if 0 <= rx < fw and 0 <= ry < fh:
            annotate_target(ann_f, rx, ry, R_FIELD, label, color, used_f)

        found_count += 1

    # Legend
    _text_box(ann_f,
              f"IRoC -- {found_count} target(s)  |  origin BL corner (0,0,0)",
              (10, 20), fscale=0.50, fg=(0,255,200))
    _text_box(ann_f,
              f"Field: {field_w_m:.2f}m x {field_h_m:.2f}m  |  1m grid",
              (10, 38), fscale=0.42, fg=(170,170,170))

    # Build side-panel list directly from computed_xyz (already has all methods)
    fresh_results = [(r, computed_xyz[r["target"]])
                     for r in results
                     if r.get("found") and r["target"] in computed_xyz]

    if fresh_results:
        TW = 210
        panel = np.zeros((fh, fw+TW, 3), np.uint8)
        panel[:,:fw] = ann_f
        panel[:,fw:] = (18,18,18)
        ty = 28
        _text_box(panel, "Results (BL corner = 0,0,0)", (fw+4,ty), fscale=0.40, fg=(0,255,200)); ty+=18
        _text_box(panel, f"z = drone altitude above gnd", (fw+4,ty), fscale=0.32, fg=(110,110,110)); ty+=14
        for r, xyz in fresh_results:
            if ty > fh-20:
                break
            col = TARGET_COLORS[results.index(r) % len(TARGET_COLORS)]
            _text_box(panel, r["target"][:22], (fw+4,ty), fscale=0.38, fg=col); ty+=15
            _text_box(panel, f"  x={xyz['x']:+.3f}m", (fw+4,ty), fscale=0.34, fg=(220,220,220)); ty+=13
            _text_box(panel, f"  y={xyz['y']:+.3f}m", (fw+4,ty), fscale=0.34, fg=(220,220,220)); ty+=13
            _text_box(panel, f"  z={xyz['z']:+.3f}m", (fw+4,ty), fscale=0.34, fg=(220,220,220)); ty+=16
        ann_f = panel

    log(f"  Annotated {found_count} targets")
    return ann_m, ann_f, computed_xyz


# ═══════════════════════════════════════════════════════════════════════════════
# PERSIST STITCH TRANSFORMS
# ═══════════════════════════════════════════════════════════════════════════════

TRANSFORMS_FILE = STITCH_DIR / "photo_transforms.npz"


def save_transforms(photo_to_H: dict):
    mkdir(STITCH_DIR)
    keys = list(photo_to_H.keys())
    mats = np.stack([photo_to_H[k] for k in keys])
    np.savez(str(TRANSFORMS_FILE), keys=keys, matrices=mats)
    log(f"  Saved transforms -> {TRANSFORMS_FILE.name}")


def load_transforms() -> Optional[dict]:
    if not TRANSFORMS_FILE.exists():
        return None
    data = np.load(str(TRANSFORMS_FILE), allow_pickle=True)
    keys = data["keys"].tolist(); mats = data["matrices"]
    log(f"  Loaded {len(keys)} transforms from {TRANSFORMS_FILE.name}")
    return {k: mats[i] for i, k in enumerate(keys)}


# ═══════════════════════════════════════════════════════════════════════════════
# STAGE 0 — 3D RECONSTRUCTION (OpenDroneMap via Docker)
# ═══════════════════════════════════════════════════════════════════════════════

def _odm_check_docker():
    """Return True if Docker is installed and running."""
    if shutil.which("docker") is None:
        log("  [ERROR] Docker not found in PATH. Install Docker to use --run-3d.")
        return False
    r = subprocess.run(["docker", "info"],
                       stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    if r.returncode != 0:
        log("  [ERROR] Docker is installed but not running. Start with: sudo service docker start")
        return False
    return True


def _odm_gpu_available() -> bool:
    """Check whether Docker can see an NVIDIA GPU."""
    try:
        r = subprocess.run(
            ["docker", "run", "--rm", "--gpus", "all",
             "nvidia/cuda:12.3.1-base-ubuntu22.04", "nvidia-smi"],
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, timeout=60)
        return r.returncode == 0
    except Exception:
        return False


def run_3d_reconstruction(photos_dir: Path = DRONE_DIR) -> Optional[Path]:
    """
    Stage 0: Run OpenDroneMap (via Docker) on *photos_dir* to produce a
    full 3-D reconstruction.  Outputs are copied to results/stage0_3d/.

    Outputs
    -------
    odm_textured_model_geo.obj   3D textured mesh
    odm_orthophoto.tif           flat orthophoto (GeoTIFF)
    dsm.tif                      Digital Surface Model (elevation)
    odm_georeferenced_model.laz  georeferenced point cloud

    Returns the stage0_3d output directory, or None on failure.
    """
    mkdir(STAGE3D_DIR)
    log("\n" + "="*62)
    log("  STAGE 0 — 3-D Reconstruction  →  " + str(STAGE3D_DIR.relative_to(BASE_DIR)))
    log("="*62)

    if not _odm_check_docker():
        return None

    # Collect photos
    photos = [f for f in photos_dir.iterdir()
              if f.suffix.lower() in ODM_IMAGE_EXTS]
    if len(photos) < 5:
        log(f"  [ERROR] Only {len(photos)} photo(s) found in {photos_dir}. "
            f"Need ≥ 5 overlapping images.")
        return None
    log(f"  Photos   : {len(photos)} in {photos_dir}")

    # ODM project layout: results/stage0_3d/odm_datasets/project/images/
    datasets_dir = STAGE3D_DIR / "odm_datasets"
    project_dir  = datasets_dir / "project"
    images_dir   = project_dir / "images"

    if ODM_FRESH_RUN and project_dir.exists():
        log("  Clearing previous ODM project ...")
        try:
            shutil.rmtree(str(project_dir))
        except PermissionError:
            log(f"  [ERROR] Old ODM files owned by root. Run:\n"
                f"    sudo rm -rf {datasets_dir}")
            return None

    images_dir.mkdir(parents=True, exist_ok=True)
    log(f"  Copying {len(photos)} photos into ODM project ...")
    for p in photos:
        shutil.copy2(str(p), str(images_dir / p.name))

    # Choose GPU or CPU image
    use_gpu = ODM_USE_GPU
    if use_gpu:
        log("  Checking Docker GPU access ...")
        if _odm_gpu_available():
            log("  GPU detected — using GPU-accelerated ODM image.")
        else:
            log("  [WARN] GPU not visible to Docker. Falling back to CPU.")
            use_gpu = False

    odm_image  = "opendronemap/odm:gpu" if use_gpu else "opendronemap/odm"
    host_mount = str(datasets_dir).replace("\\", "/")

    # --user flag keeps output files owned by the current user (Linux/WSL only)
    uid_flag = []
    try:
        uid_flag = ["--user", f"{os.getuid()}:{os.getgid()}"]
    except AttributeError:
        pass  # Windows: no getuid — omit flag, ODM runs as container default

    cmd = ["docker", "run", "-ti", "--rm"]
    cmd += uid_flag
    cmd += ["-v", f"{host_mount}:/datasets"]
    if use_gpu:
        cmd += ["--gpus", "all"]
    cmd += [odm_image, "--project-path", "/datasets", "project"]
    cmd += ODM_EXTRA_OPTS

    log(f"  Running ODM ({'GPU' if use_gpu else 'CPU'}) ...")
    log("  " + " ".join(cmd))
    try:
        result = subprocess.run(cmd, check=True)
        log(f"  ODM finished (exit 0)")
    except subprocess.CalledProcessError as e:
        log(f"  [ERROR] ODM exited with code {e.returncode}")
        log(f"  Check Docker logs above for details.")
        log(f"  Common causes: insufficient memory, missing images, Docker not started.")
        return None

    # Verify at least one output exists before copying
    tex_src = project_dir / "odm_texturing"
    ortho_src = project_dir / "odm_orthophoto" / "odm_orthophoto.tif"
    if not tex_src.exists() and not ortho_src.exists():
        log("  [ERROR] ODM produced no outputs. Possible causes:")
        log("    - Not enough RAM (ODM needs ~4 GB per 100 photos)")
        log("    - Images have no GPS EXIF and ODM couldn't georeference")
        log("    - Docker container crashed silently")
        log(f"  Check: {project_dir}/opensfm/reconstruction.json")
        return None

    # Copy outputs to results/stage0_3d/
    # ── Textured 3D model: copy entire odm_texturing/ folder so that the
    #    .obj, .mtl and all texture images stay together (required for colour).
    tex_dst = STAGE3D_DIR / "odm_texturing"
    if tex_src.exists():
        if tex_dst.exists():
            shutil.rmtree(str(tex_dst))
        shutil.copytree(str(tex_src), str(tex_dst))
        log(f"  Saved → {tex_dst.relative_to(BASE_DIR)}/ ({len(list(tex_dst.iterdir()))} files)")
    else:
        log("  [WARN] odm_texturing/ not found — 3D model may be unavailable")

    # ── Other single-file outputs
    other_outputs = {
        project_dir / "odm_orthophoto" / "odm_orthophoto.tif":             "odm_orthophoto.tif",
        project_dir / "odm_dem"        / "dsm.tif":                         "dsm.tif",
        project_dir / "odm_georeferencing" / "odm_georeferenced_model.laz": "odm_georeferenced_model.laz",
    }
    for src, dst_name in other_outputs.items():
        if src.exists():
            dst = STAGE3D_DIR / dst_name
            shutil.copy2(str(src), str(dst))
            log(f"  Saved → {dst.relative_to(BASE_DIR)}")
        else:
            log(f"  [WARN] ODM output not found: {src.name}")

    log(f"\n  Stage 0 complete. Results in {STAGE3D_DIR.relative_to(BASE_DIR)}/")
    return STAGE3D_DIR


def view_3d_results(stage3d_dir: Path) -> None:
    """
    Automatically visualise Stage 0 outputs with colour:

    1. Orthophoto preview — save orthophoto_preview.jpg and open it.
    2. DSM elevation     — apply TURBO colormap, save dsm_preview.jpg and open it.
    3. Coloured 3D mesh  — open with Open3D interactive viewer (GPU-accelerated).
       Falls back to system default viewer if Open3D is not installed.
    """
    log("\n" + "="*62)
    log("  STAGE 0 — 3-D Viewer (colour)")
    log("="*62)

    previews_saved = []

    # ── 1. Orthophoto (flat coloured map) ─────────────────────────────────────
    ortho = stage3d_dir / "odm_orthophoto.tif"
    if ortho.exists():
        img = cv2.imread(str(ortho))           # cv2 reads GeoTIFF fine
        if img is not None:
            # Resize to at most 4K for preview
            max_side = 3840
            h, w = img.shape[:2]
            if max(h, w) > max_side:
                s = max_side / max(h, w)
                img = cv2.resize(img, (int(w*s), int(h*s)), interpolation=cv2.INTER_AREA)
            out = stage3d_dir / "orthophoto_preview.jpg"
            cv2.imwrite(str(out), img, [cv2.IMWRITE_JPEG_QUALITY, 92])
            previews_saved.append(out)
            log(f"  Orthophoto preview saved → {out.name}")
        else:
            log("  [WARN] Could not read odm_orthophoto.tif via cv2")

    # ── 2. DSM elevation with TURBO colourmap ─────────────────────────────────
    dsm_path = stage3d_dir / "dsm.tif"
    if dsm_path.exists():
        dsm = cv2.imread(str(dsm_path), cv2.IMREAD_ANYDEPTH | cv2.IMREAD_GRAYSCALE)
        if dsm is not None:
            dsm_f = dsm.astype(np.float32)
            # Ignore no-data values (very large negative numbers)
            valid_mask = dsm_f > (dsm_f.min() + 1.0)
            lo = float(dsm_f[valid_mask].min()) if valid_mask.any() else dsm_f.min()
            hi = float(dsm_f.max())
            norm = np.clip((dsm_f - lo) / max(hi - lo, 1e-6), 0.0, 1.0)
            colored = cv2.applyColorMap((norm * 255).astype(np.uint8), cv2.COLORMAP_TURBO)
            # Black out no-data pixels
            colored[~valid_mask] = 0
            # Add elevation legend bar (right edge)
            bar_w = max(20, int(colored.shape[1] * 0.015))
            bar = np.linspace(255, 0, colored.shape[0], dtype=np.uint8)
            bar_col = cv2.applyColorMap(
                np.tile(bar.reshape(-1, 1), (1, bar_w)), cv2.COLORMAP_TURBO)
            vis = np.hstack([colored, bar_col])
            cv2.putText(vis, f"{hi:.1f}m", (colored.shape[1]+2, 14),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255,255,255), 1, cv2.LINE_AA)
            cv2.putText(vis, f"{lo:.1f}m", (colored.shape[1]+2, vis.shape[0]-4),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255,255,255), 1, cv2.LINE_AA)
            out = stage3d_dir / "dsm_preview.jpg"
            cv2.imwrite(str(out), vis, [cv2.IMWRITE_JPEG_QUALITY, 92])
            previews_saved.append(out)
            log(f"  DSM elevation preview saved → {out.name}  "
                f"(range {lo:.1f} – {hi:.1f} m, TURBO colourmap)")
        else:
            log("  [WARN] Could not read dsm.tif via cv2")

    # ── 3. Open coloured 3D mesh with Open3D ──────────────────────────────────
    obj_path = stage3d_dir / "odm_texturing" / "odm_textured_model_geo.obj"
    if not obj_path.exists():
        # Fallback: check if it was copied directly to stage3d_dir
        obj_path = stage3d_dir / "odm_textured_model_geo.obj"

    if obj_path.exists():
        try:
            import open3d as o3d
            log("  Loading coloured 3D mesh with Open3D ...")
            mesh = o3d.io.read_triangle_mesh(str(obj_path), enable_post_processing=True)
            if len(mesh.triangles) == 0:
                log("  [WARN] Mesh loaded but contains no triangles.")
            else:
                mesh.compute_vertex_normals()
                n_tri = len(np.asarray(mesh.triangles))
                log(f"  Mesh: {n_tri:,} triangles — opening interactive viewer ...")
                log("  (Close the 3D window to continue / end pipeline)")
                o3d.visualization.draw_geometries(
                    [mesh],
                    window_name="IRoC 3D Model (coloured) — odm_textured_model_geo.obj",
                    width=1280, height=720,
                    mesh_show_back_face=True,
                )
        except ImportError:
            log("  [INFO] Open3D not installed. For the interactive 3D viewer run:")
            log("           pip install open3d")
            log(f"  [INFO] 3D file: {obj_path}")
            _open_with_system(obj_path)
        except Exception as exc:
            log(f"  [WARN] Open3D viewer error: {exc}")
            _open_with_system(obj_path)
    else:
        log("  [WARN] Textured OBJ not found — skipping 3D viewer")

    # ── Open preview images with system viewer ─────────────────────────────────
    for p in previews_saved:
        _open_with_system(p)


def _open_with_system(path: Path) -> None:
    """Open *path* with the OS default application (non-blocking)."""
    try:
        if sys.platform.startswith("linux"):
            subprocess.Popen(["xdg-open", str(path)],
                             stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        elif sys.platform == "darwin":
            subprocess.Popen(["open", str(path)])
        else:  # Windows
            os.startfile(str(path))
        log(f"  Opened {path.name} with system viewer")
    except Exception as exc:
        log(f"  [WARN] Could not open {path.name}: {exc}")


# ═══════════════════════════════════════════════════════════════════════════════
# STAGE 4 — ANNOTATED FIELD MAP  (circles + x,y,z labels, origin = BL corner)
# ═══════════════════════════════════════════════════════════════════════════════

def run_stage4_annotate(rect_bgr, field_w_m, field_h_m, results, map_coords,
                        origin_x, origin_y, origin_z, base_px=None):
    """
    Draw a filled circle + x,y,z label for every found target on the
    perspective-rectified field map.  Origin (0,0) = BASE STATION.
    base_px = base station ka rectified pixel (bx, by) -> grid/circle/label sab isi se reference
    (yellow corner nahi) -> coords base station se exact match, ~50-60cm offset gaya.

    Saves:
      results/stage4_annotated/annotated_field.jpg
    """
    mkdir(ANNOT_DIR)
    log("\n" + "="*62)
    log("  STAGE 4 -- Annotated field map  →  "
        + str(ANNOT_DIR.relative_to(BASE_DIR)))
    log("="*62)

    ann = rect_bgr.copy()
    fh, fw = ann.shape[:2]
    # base station rectified pixel = grid origin (0,0). None -> BL corner fallback.
    bx = float(base_px[0]) if base_px is not None else 0.0
    by = float(base_px[1]) if base_px is not None else float(fh)
    log(f"  Origin (yellow corner = 0,0,0) @ rect px ({bx:.0f},{by:.0f})   PX_PER_M={PX_PER_M}")

    # 1m grid (origin = base station)
    draw_grid(ann, field_w_m, field_h_m, float(PX_PER_M), ox=bx, oy=by)

    # Field border + origin marker (base station)
    cv2.rectangle(ann, (0, 0), (fw - 1, fh - 1), (0, 220, 220), 2)
    cv2.circle(ann, (int(bx), int(by)), 10, (0, 255, 200), 3)
    _text_box(ann, "(0,0)", (int(bx) + 8, int(by) - 6), fscale=0.40,
              fg=(0, 255, 200), bg=(0, 0, 0))

    R = max(15, int(0.22 * PX_PER_M))   # circle radius in rectified pixels
    used_labels = []
    annotated = 0

    for idx, res in enumerate(results):
        if not res.get("found"):
            continue
        tname = res["target"]
        xyz = map_coords.get(tname)
        if xyz is None:
            xyz = res.get("object_xyz")
        if xyz is None:
            log(f"  [SKIP] {tname}: no coordinates")
            continue

        # Circle VISUAL position = mosaic-mapped rectified pixel (target jahan actually dikhta hai).
        # Label = accurate GPS coords. (_rx/_ry na ho to coords se estimate.)
        rx = xyz.get("_rx"); ry = xyz.get("_ry")
        if rx is None or ry is None:
            rx = xyz["x"] * PX_PER_M + bx
            ry = by - xyz["y"] * PX_PER_M

        if not (0 <= rx < fw and 0 <= ry < fh):
            log(f"  [OUT-OF-BOUNDS] {tname}: ({rx:.0f},{ry:.0f}) outside field")
            continue

        color = TARGET_COLORS[idx % len(TARGET_COLORS)]
        label = (f"{tname}  "
                 f"x={xyz['x']:.2f} y={xyz['y']:.2f} z={xyz['z']:.2f}m")

        # Outer circle + filled centre dot
        cv2.circle(ann, (int(rx), int(ry)), R, color, 3, cv2.LINE_AA)
        cv2.circle(ann, (int(rx), int(ry)), 5, color, -1, cv2.LINE_AA)

        # Label with leader line (avoid overlaps)
        font = cv2.FONT_HERSHEY_SIMPLEX
        (tw, th), _ = cv2.getTextSize(label, font, 0.46, 1)
        lx, ly = _place_label(int(rx), int(ry), R, tw, th, used_labels, fw, fh)
        ex = int(rx + (lx + tw // 2 - rx) / max(1, abs(lx + tw // 2 - rx)) * R)
        ey = int(ry + (ly - th // 2 - ry) / max(1, abs(ly - th // 2 - ry)) * R)
        cv2.line(ann, (ex, ey), (lx + tw // 2, ly - th // 2),
                 (120, 120, 120), 1, cv2.LINE_AA)
        _text_box(ann, label, (lx, ly), fscale=0.46, thick=1,
                  fg=(255, 255, 255), bg=(0, 0, 0))
        used_labels.append((lx, ly - th, tw, th + 4))

        log(f"  {tname}: rect=({int(rx)},{int(ry)})  "
            f"x={xyz['x']:.3f} y={xyz['y']:.3f} z={xyz['z']:.3f}")
        annotated += 1

    # Legend header
    _text_box(ann,
              f"IRoC — {annotated} target(s)  |  origin = yellow corner (0,0,0)",
              (10, 20), fscale=0.50, fg=(0, 255, 200))
    _text_box(ann,
              f"Field: {field_w_m:.2f}m × {field_h_m:.2f}m  |  1m grid",
              (10, 38), fscale=0.42, fg=(170, 170, 170))

    out = ANNOT_DIR / "annotated_field.jpg"
    cv2.imwrite(str(out), ann, [cv2.IMWRITE_JPEG_QUALITY, 95])
    log(f"  Saved → {out}")
    log(f"  Annotated {annotated} targets")
    return ann


# ═══════════════════════════════════════════════════════════════════════════════
# MAIN
# ═══════════════════════════════════════════════════════════════════════════════

def archive_old_results():
    """Naya run se pehle purane results/ ko results_archive/<timestamp>/ me copy (history rakho)."""
    if not RESULTS_DIR.exists() or not any(RESULTS_DIR.iterdir()):
        return
    ts = time.strftime("%Y%m%d_%H%M%S")
    dest = BASE_DIR / "results_archive" / ts
    try:
        dest.parent.mkdir(parents=True, exist_ok=True)
        shutil.copytree(RESULTS_DIR, dest)
        log(f"  Purane results saved -> results_archive/{ts}/")
    except Exception as e:
        log(f"  [WARN] archive fail: {e}")


def main():
    parser = argparse.ArgumentParser(description="IROC Pipeline  (stages 1-3)")
    parser.add_argument("--run-3d",      action="store_true",
                        help="After stages 1-3 complete, run 3D reconstruction "
                             "via OpenDroneMap (requires Docker)")
    parser.add_argument("--skip-stitch", action="store_true",
                        help="Reuse existing orthomosaic + transforms")
    parser.add_argument("--skip-match",  action="store_true",
                        help="Also skip target finding (implies --skip-stitch)")
    parser.add_argument("--radius",      type=int, default=GRID_RADIUS,
                        help=f"Stitch neighbour radius (1=fast, 2=thorough; default {GRID_RADIUS})")
    args = parser.parse_args()
    if args.skip_match:
        args.skip_stitch = True

    archive_old_results()           # purane results -> results_archive/<timestamp>/ (overwrite se pehle)
    mkdir(RESULTS_DIR)
    t_start = time.time()

    # ── Stage 1: Stitching ────────────────────────────────────────────────────
    if not args.skip_stitch:
        mosaic_path, photo_to_H, mosaic_bgr = run_stitching(MAP_DIR, radius=args.radius)
        save_transforms(photo_to_H)
    else:
        mosaic_path = STITCH_DIR / "orthomosaic.jpg"
        if not mosaic_path.exists():
            mosaic_path = MAP_DIR / "orthomosaic.jpg"
        if not mosaic_path.exists():
            log("ERROR: orthomosaic.jpg not found. Run without --skip-stitch first.")
            sys.exit(1)
        mosaic_bgr = cv2.imread(str(mosaic_path))
        photo_to_H = load_transforms()
        if photo_to_H is None:
            log("ERROR: no saved transforms. Run without --skip-stitch first.")
            sys.exit(1)
        log(f"[Stage 1 skipped]  {mosaic_path.name}  "
            f"({mosaic_bgr.shape[1]}x{mosaic_bgr.shape[0]})")

    # ── Stage 2: Field map ────────────────────────────────────────────────────
    # CSV: drone_photos/coordinates.csv (primary), map/coordinates.csv (fallback)
    csv_used = CSV_PATH if CSV_PATH.exists() else DRONE_DIR / "coordinates.csv"
    if not csv_used.exists():
        log(f"  [WARN] No coordinates.csv found in {DRONE_DIR} or {MAP_DIR}")
    csv_rows = load_csv_locations(csv_used) if csv_used.exists() else []
    log(f"  CSV: {len(csv_rows)} entries from {csv_used}")

    A_px2enu, corners, field_w_m, field_h_m, rect_bgr, M_persp, \
        origin_x, origin_y, origin_z = \
        setup_field_map(mosaic_bgr, photo_to_H, csv_rows)

    # ── Stage 3: Target finding ───────────────────────────────────────────────
    json_path = TARGET_DIR / "targets.json"
    if args.skip_match and json_path.exists():
        log(f"\n[Stage 3 skipped]  Loading {json_path.name}")
        results = json.load(open(json_path)).get("targets", [])
    else:
        results = run_target_finding(csv_rows, origin_x, origin_y, origin_z)
        json.dump({"targets": results,
                   "origin_enu": {"x": origin_x, "y": origin_y, "z": origin_z}},
                  open(json_path, "w"), indent=2)
        log(f"  Saved -> {json_path}")

    # ── Map coordinates (x,y,z from mosaic perspective) ──────────────────────
    map_coords = compute_map_coords(
        results, mosaic_bgr, photo_to_H, M_persp,
        field_h_m, csv_rows, origin_z, A_px2enu, origin_x, origin_y)

    # Persist map_xyz into targets.json
    for r in results:
        tname = r["target"]
        if tname in map_coords:
            r["map_xyz"] = {k: v for k, v in map_coords[tname].items()
                            if not k.startswith("_")}      # internal _rx/_ry hatao
    json.dump({"targets": results,
               "origin_enu": {"x": origin_x, "y": origin_y, "z": origin_z}},
              open(json_path, "w"), indent=2)

    # ── Stage 4: Annotated field map ─────────────────────────────────────────
    # origin (0,0) = YELLOW LINE corner = rectified bottom-left (yellow inner BL yahin map hota).
    # base_px=None -> grid origin default BL (0, height) = yellow corner.
    run_stage4_annotate(rect_bgr, field_w_m, field_h_m, results, map_coords,
                        origin_x, origin_y, origin_z, None)

    # ── Summary ───────────────────────────────────────────────────────────────
    elapsed = time.time() - t_start
    log("\n" + "="*62)
    log(f"  ALL STAGES DONE  ({elapsed:.1f}s)")
    log("="*62)
    log(f"  results/stage1_stitch/     orthomosaic.jpg, transforms")
    log(f"  results/stage2_field/      rectified_field.jpg, calibration.txt")
    log(f"  results/stage3_targets/    targets.json + fused_results.csv + visuals/ + proof_hd/ (HD)")
    log(f"  results/stage4_annotated/  annotated_field.jpg")
    log(f"\n  CSV location: {csv_used}")
    log(f"  Origin (yellow corner, BL): ({origin_x:.3f}, {origin_y:.3f}, {origin_z:.3f}) ENU")
    log(f"  Field: {field_w_m:.2f} m × {field_h_m:.2f} m")
    log(f"\n  {'Target':<28} {'Method':<12} {'x(m)':>8} {'y(m)':>8} {'z(m)':>8}  Photo")
    log("  " + "-"*80)
    found_n = 0
    for r in results:
        if r.get("found"):
            xyz = map_coords.get(r["target"]) or r.get("object_xyz") or {}
            xv = f"{xyz['x']:>8.3f}" if xyz else f"{'?':>8}"
            yv = f"{xyz['y']:>8.3f}" if xyz else f"{'?':>8}"
            zv = f"{xyz['z']:>8.3f}" if xyz else f"{'?':>8}"
            log(f"  {r['target']:<28} {r['method']:<12} {xv} {yv} {zv}  {r['drone_photo']}")
            found_n += 1
        else:
            log(f"  {r['target']:<28} NOT FOUND")
    log(f"\n  Found {found_n}/{len(results)} targets")

    # ── 3D Reconstruction callback (after stages 1-3) ─────────────────────────
    if args.run_3d:
        log("\n" + "="*62)
        log("  3D RECONSTRUCTION CALLBACK  (stages 1-3 complete)")
        log("="*62)
        stage3d_out = run_3d_reconstruction(DRONE_DIR)
        if stage3d_out is not None:
            view_3d_results(stage3d_out)
            log(f"\n  results/stage0_3d/  odm_texturing/, odm_orthophoto.tif,")
            log(f"                      dsm.tif, point cloud,")
            log(f"                      orthophoto_preview.jpg, dsm_preview.jpg")
    log("")


if __name__ == "__main__":
    main()
