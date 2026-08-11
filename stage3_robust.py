#!/usr/bin/env python3
"""
IRoC-U 2026 -- STAGE 3 ROBUST MATCHER  (lighting + rotation + low-quality robust)
==============================================================================
Rulebook (Elimination V3.0) ke hisaab se:
  * 3-5 LR (128x128) seed images = feature TYPES (layered rock / red-oxide / ice-patch).
  * Arena me har type ke 2-3 INSTANCES dhoondhne hain (exact same object NAHI -> geometric
    LoFTR/SuperPoint yahan weak; SEMANTIC match chahiye).
  * Matching must be less sensitive to illumination conditions -> lighting-robust zaroori.
  * Matching 128x128 LR pe -> low quality inherent.

METHOD (RoMa/DINOv2 SOTA se adapt, CPU-friendly):
  1) ILLUMINATION NORM : har image pe CLAHE (Lab L-channel) -> lighting differences kam.
  2) OBJECT PROTOTYPE  : reference (fused seeds) ke DINOv2 patch features se
                         background(border) + object(foreground) prototype. Prototype = patches ka
                         AVERAGE -> rotation-robust (bag-of-features).
  3) SELECT + LOCALIZE : har DRONE photo ke DINOv2 patches pe heatmap = (patch.obj - patch.bg).
                         heatmap peak = target kitna PRESENT + KAHAN. Best (peak + center-pref) photo
                         choose + wahi peak = localization (ek hi step).
  4) VERIFY (rotation) : localized crop vs reference crop DINOv2 sim, 4 rotations me se max ->
                         false / survey-me-nahi target reject (NOT FOUND).
  5) THRESHOLD         : peak/verify kam -> NOT FOUND (koi random nahi).

Output: fused_search.py jaisa hi (results/stage3_targets/... + fused_results.csv) -> iroc_pipeline
me drop-in (iroc_pipeline2.py isko chalata hai).

RUN:  python3 stage3_robust.py
"""

import os, csv, time
import cv2
import numpy as np
import math
import torch
import fused_search as fs           # helpers reuse (models, dino, group_seeds, drone lr, draw)

# ================= CONFIG =================
BASE_DIR     = fs.BASE_DIR
TARGETS_DIR  = fs.TARGETS_DIR
OUT_DIR      = fs.OUT_DIR
DRONE_HD_DIR = fs.DRONE_HD_DIR
PROC_W, PROC_H = fs.PROC_W, fs.PROC_H

# MUTUAL EXCLUSION: do targets ki approx ground position (VIO + pixel offset) is se paas ->
# same object -> lower-priority target apni NEXT-BEST location leta hai (duplicate avoid).
MIN_SEP_M    = 0.6
DRONE_HEIGHT = 3.0                              # approx camera height (m) -- ground offset scale ke liye
FOV_H_DEG, FOV_V_DEG = 90.0, 65.0
GND_W = 2.0 * DRONE_HEIGHT * math.tan(math.radians(FOV_H_DEG / 2))
GND_H = 2.0 * DRONE_HEIGHT * math.tan(math.radians(FOV_V_DEG / 2))

# ─────────────────────────── TUNABLE THRESHOLDS (#7) ───────────────────────────
# Ye values IS dataset pe tune ki hain. Naye data/lighting pe adjust kar sakte ho --
# terminal me har target ka `peak=.. V..` print hota hai, unhe dekh ke set karo:
#   * asli target NOT FOUND aa raha  -> MIN_FOUND_PEAK / VERIFY_MIN ghatao
#   * galat/random match aa raha     -> MIN_FOUND_PEAK / VERIFY_MIN badhao
def _envf(key, default, cast=float):        # env-override (64x64 mode ke liye handy; unset -> default)
    v = os.environ.get(key)
    try:
        return cast(v) if v not in (None, "") else default
    except Exception:
        return default
MIN_FOUND_PEAK = _envf("MIN_FOUND_PEAK", 0.14)  # selection: object-heatmap peak is se KAM -> NOT FOUND. Badhao=strict.
VERIFY_MIN     = _envf("VERIFY_MIN", 0.45)      # verification crop-sim (rotation-max) is se KAM -> NOT FOUND. Badhao=strict.

# AUTO-CALIBRATION (#3): peak threshold ko background se auto-set, par sirf MORE-LENIENT direction me
# (fixed se kabhi zyada strict nahi) -> current behaviour preserve + DARK/low-signal arena pe threshold
# neeche ho ke subtle targets bhi catch. pmin = min(MIN_FOUND_PEAK, max(PEAK_FLOOR, bg_median*RATIO)).
# False-positive rejection verification (color+dino) sambhalta hai. AUTO_CALIBRATE=False -> fixed.
AUTO_CALIBRATE = True
PEAK_FLOOR     = 0.06      # auto-cal ka absolute floor (pure noise reject)
PEAK_RATIO     = 1.5       # background_median * RATIO (par fixed se strict nahi hoga)
CENTER_PREF    = _envf("CENTER_PREF", 0.06)  # peak center ke paas ho to prefer (edge/aadha-cut demote). sim units.
TOPK           = _envf("TOPK", 8, int)       # peak se top-K photos shortlist, phir APPEARANCE-verify se best chuno
                           # (metallic screw jaisa distractor high-peak deta -> verify se reject).
ROTATIONS      = (0, 1, 2, 3)   # np.rot90 k: 0/90/180/270 (verification me rotation-invariance)

# ── LR-MATCH experiment (rulebook LR-to-LR try) ──────────────────────────────────
#   Drone photo ko is LR size pe DOWN-scale karke phir match (seed 64 ke saath). Result compare
#   karne ke liye -- bina code chhede env-var se:
#       MATCH_LR=64  python3 iroc_pipeline_fixed.py --skip-stitch    # pure 64-to-64
#       MATCH_LR=128 python3 iroc_pipeline_fixed.py --skip-stitch    # 64-to-128
#       python3 iroc_pipeline_fixed.py --skip-stitch                 # (unset) accurate default
#   None/unset -> current behaviour (128 LR source -> 640 process). NOTE: chhoti LR pe localization
#   COARSE hoti (kam patches) -> accuracy giregi; ye sirf comparison/rulebook-demo ke liye hai.
MATCH_LR = os.environ.get("MATCH_LR")
MATCH_LR = int(MATCH_LR) if (MATCH_LR and MATCH_LR.isdigit()) else None
# ────────────────────────────────────────────────────────────────────────────────


# ================= ILLUMINATION NORMALIZATION =================
def clahe_norm(bgr):
    """CLAHE on Lab L-channel -> lighting/shadow differences kam (illumination-robust)."""
    lab = cv2.cvtColor(bgr, cv2.COLOR_BGR2LAB)
    l, a, b = cv2.split(lab)
    l = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8)).apply(l)
    return cv2.cvtColor(cv2.merge([l, a, b]), cv2.COLOR_LAB2BGR)


def rot90(img, k):
    return np.ascontiguousarray(np.rot90(img, k))


# ================= DINOv2 PROTOTYPES + HEATMAP =================
def ref_prototypes(M, ref):
    """Reference se OBJECT + BACKGROUND prototype (DINOv2 patches). Border=bg, foreground=object.
    Prototype = patches ka average -> rotation-robust. Returns (obj, bg, p) torch tensors."""
    Fr = fs.dino_patches(M, ref)                       # (p,p,D) L2-normalized, on DEVICE
    p = Fr.shape[0]; D = Fr.shape[-1]
    fr = Fr.reshape(-1, D)
    idx = torch.arange(p, device=fr.device)
    yy, xx = torch.meshgrid(idx, idx, indexing='ij')
    border = ((xx < 2) | (xx >= p - 2) | (yy < 2) | (yy >= p - 2)).reshape(-1)
    bg = torch.nn.functional.normalize(fr[border].mean(0), dim=0)
    fgness = 1.0 - (fr @ bg)                            # har patch ka foreground-ness
    k = max(6, int(0.15 * p * p))
    obj = torch.nn.functional.normalize(fr[torch.topk(fgness, k).indices].mean(0), dim=0)
    return obj, bg, p


def drone_heat(Fd, obj, bg, p):
    """Drone patches (p,p,D) pe object-heatmap = (patch.obj - patch.bg). numpy (p,p)."""
    fd = Fd.reshape(-1, Fd.shape[-1])
    heat = ((fd @ obj) - (fd @ bg)).reshape(p, p).float().cpu().numpy()
    return cv2.GaussianBlur(np.maximum(heat, 0.0), (0, 0), 1.0)


def peak_and_center(heat, p):
    """Heatmap ka peak value + high-response patches ka weighted centroid (patch coords) +
    drone-pixel center + radius."""
    mx = float(heat.max())
    if mx < 1e-6:
        return 0.0, None
    hn = heat / mx
    ys, xs = np.where(hn >= 0.55)
    if len(xs) < 1:
        my, mxi = np.unravel_index(int(hn.argmax()), hn.shape); ys, xs = np.array([my]), np.array([mxi])
    wts = hn[ys, xs]
    cxp = float((xs * wts).sum() / wts.sum()); cyp = float((ys * wts).sum() / wts.sum())
    radp = float(np.sqrt(max(len(xs), 1) / np.pi))
    cx = (cxp + 0.5) / p * PROC_W; cy = (cyp + 0.5) / p * PROC_H
    rad = radp / p * (PROC_W + PROC_H) / 2.0 * 1.15
    rad = float(min(max(rad, PROC_W * 0.05), PROC_W * 0.22))
    return mx, (cx, cy, rad, cxp, cyp)


def _ref_crop_embs(M, ref):
    """Reference ke CENTER crop (target) ke DINOv2 embeddings, 4 rotations (rotation-robust)."""
    embs = []
    for k in ROTATIONS:
        r = rot90(ref, k)
        h, w = r.shape[:2]; rr = int(min(h, w) * 0.35)
        rc = r[max(0, h // 2 - rr):h // 2 + rr, max(0, w // 2 - rr):w // 2 + rr]
        if rc.size and min(rc.shape[:2]) >= 6:
            embs.append(fs.dino_global(M, rc))
    return embs


def _crop_bgr(photo, cx, cy, rad):
    H, W = photo.shape[:2]; r = int(max(rad, PROC_W * 0.08))
    c = photo[max(0, int(cy) - r):min(H, int(cy) + r), max(0, int(cx) - r):min(W, int(cx) + r)]
    if c.size == 0 or min(c.shape[:2]) < 6:
        return None
    return c


def color_hist(bgr):
    """HSV hue-saturation 2D histogram (normalized). Distinct objects (white box vs grey rock)
    ka color alag -> discrimination. CLAHE-normed crops pe -> lighting-robust."""
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    h = cv2.calcHist([hsv], [0, 1], None, [30, 32], [0, 180, 0, 256])
    cv2.normalize(h, h, 0, 1, cv2.NORM_MINMAX)
    return h


def localize_verified(M, ref_embs, ref_hist, photo, heat, p):
    """Heatmap ke TOP candidate regions me se woh chuno jiska crop reference target se sabse
    zyada MILTA hai (rotation-max DINOv2 sim + COLOR similarity). Distinct object (color alag)
    penalize hota -> unique targets confuse nahi hote (rock-pile ref white-box pe nahi jaata).
    ref_embs/ref_hist = reference center crop ke embeddings + color hist. Returns (vsim,cx,cy,rad)|None."""
    if not ref_embs:
        return None
    hn = heat / (heat.max() + 1e-9)
    n, lbl, stats, _ = cv2.connectedComponentsWithStats((hn >= 0.40).astype(np.uint8))
    cands = []
    if n <= 1:
        my, mx = np.unravel_index(int(hn.argmax()), hn.shape)
        cands = [(float(mx), float(my), 1)]
    else:
        blobs = []
        for i in range(1, n):
            m = (lbl == i); area = int(stats[i, cv2.CC_STAT_AREA])
            ys, xs = np.where(m); w = hn[ys, xs]
            blobs.append((float(heat[m].mean()) * (area ** 0.5),
                          float((xs * w).sum() / w.sum()), float((ys * w).sum() / w.sum()), area))
        blobs.sort(reverse=True)
        cands = [(b[1], b[2], b[3]) for b in blobs[:4]]     # top-4 candidate regions
    best = None   # (rank, dsim, cx, cy, rad)
    for cxp, cyp, area in cands:
        radp = float(np.sqrt(max(area, 1) / np.pi))
        cx = (cxp + 0.5) / p * PROC_W; cy = (cyp + 0.5) / p * PROC_H
        rad = float(min(max(radp / p * (PROC_W + PROC_H) / 2.0 * 1.15, PROC_W * 0.05), PROC_W * 0.25))
        crop = _crop_bgr(photo, cx, cy, rad)
        if crop is None:
            continue
        dsim = max(float(np.dot(re, fs.dino_global(M, crop))) for re in ref_embs)   # rotation-max DINOv2 sim
        rank = dsim
        if ref_hist is not None:                             # COLOR: sirf candidate RANKING ke liye
            cs = float(cv2.compareHist(color_hist(crop), ref_hist, cv2.HISTCMP_CORREL))
            rank = dsim * (0.4 + 0.6 * max(0.0, cs))         # color match -> upar; distinct object -> neeche
        if best is None or rank > best[0]:
            best = (rank, dsim, cx, cy, rad)
    if best is None:
        return None
    return (best[1], best[2], best[3], best[4])              # vsim = PURE dino (threshold unchanged -> no false NOT FOUND)


def load_locs():
    """coordinates.csv -> {stem: {x,y,yaw}} (VIO/Pixhawk positions). Mutual-exclusion ke liye."""
    import csv as _csv, os as _os
    path = _os.path.join(DRONE_HD_DIR, "coordinates.csv")
    locs = {}
    if _os.path.exists(path):
        with open(path) as f:
            for r in _csv.DictReader(f):
                try:
                    stem = _os.path.splitext(r["image_file"])[0]
                    locs[stem] = {"x": float(r["x_enu"]), "y": float(r["y_enu"]),
                                  "yaw": float(r.get("yaw_deg", 0.0))}
                except (KeyError, TypeError, ValueError):
                    continue
    return locs


def gpos(loc, cx, cy):
    """Approx ground ENU position: photo VIO position + pixel-offset (yaw se rotate). Rough,
    par do targets same object pe hain ya nahi -- yeh detect karne ko kaafi."""
    if not loc or loc.get("x") is None:
        return None
    oe = (cx - PROC_W / 2.0) * (GND_W / PROC_W)
    on = -(cy - PROC_H / 2.0) * (GND_H / PROC_H)
    yaw = loc.get("yaw")
    if yaw is not None:
        th = math.radians(yaw); c, s = math.cos(th), math.sin(th)
        re = oe * c - on * s; rn = oe * s + on * c
    else:
        re, rn = oe, on
    return (loc["x"] + re, loc["y"] + rn)


# ================= RUN =================
def run():
    groups = fs.group_seeds(TARGETS_DIR)
    if not groups:
        print(f"ERROR: {TARGETS_DIR} me koi feature folder/image nahi."); return
    drone_lr, hd_map = fs.build_drone_lr()
    if not drone_lr:
        print("ERROR: drone photos nahi."); return
    print(f"Features: {len(groups)} | Drone photos: {len(drone_lr)}" +
          (f" | HD proof: ON" if hd_map else " | HD proof: OFF") + "\n")

    M = fs.load_models()

    # DRONE: CLAHE-norm work images + DINOv2 patches (once). Selection+localize dono isi se.
    print("Indexing drone photos (CLAHE + DINOv2 patches) ...")
    if MATCH_LR:
        print(f"[LR-MATCH] drone photos ko {MATCH_LR}x{MATCH_LR} LR pe le ja rahe "
              f"(seed 64 ke saath LR-to-LR) -- localization coarse hogi.")
    drones = []                                        # (dname, clahe_work_bgr)
    for p in drone_lr:
        im = cv2.imread(p)
        if im is None:
            continue
        if MATCH_LR:                                   # LR-to-LR experiment: drone ko chosen LR pe downscale
            im = cv2.resize(im, (MATCH_LR, MATCH_LR), interpolation=cv2.INTER_AREA)
        drones.append((os.path.basename(p), fs.to_work(clahe_norm(im))))
    drone_patches = [fs.dino_patches(M, ph) for _, ph in drones]

    vis_dir = os.path.join(OUT_DIR, "visuals"); fus_dir = os.path.join(OUT_DIR, "fused")
    proof_dir = os.path.join(OUT_DIR, "proof_hd")
    lr_dir = os.path.join(OUT_DIR, "lr_match")     # (11.3.8a) LR image corresponding to seed
    for d in (vis_dir, fus_dir, proof_dir, lr_dir):
        os.makedirs(d, exist_ok=True)
    rows = []
    locs = load_locs()                                 # VIO positions (mutual exclusion ke liye)

    # ── PASS 1: har target ke CANDIDATES (topk photos, best-verify per photo) + ground position ──
    tdata = []   # (name, ref, n_seeds, cand_list); cand=(vsim,cx,cy,rad,peak,dname,photo,gpos) vsim-desc
    for name, seeds in groups.items():
        imgs = [clahe_norm(cv2.imread(s)) for s in seeds if cv2.imread(s) is not None]
        if not imgs:
            print(f"  {name:16s} -> reference read FAIL"); continue
        ref = fs.to_work(imgs[0])
        if len(imgs) > 1:                              # multi-seed -> per-pixel median (denoise)
            stack = np.stack([fs.to_work(x).astype(np.float32) for x in imgs], 0)
            ref = np.median(stack, 0).astype(np.uint8)
        cv2.imwrite(os.path.join(fus_dir, f"{name}.png"), ref)
        obj, bg, p = ref_prototypes(M, ref)
        ref_embs = _ref_crop_embs(M, ref)
        rh_, rw_ = ref.shape[:2]; rr_ = int(min(rh_, rw_) * 0.35)   # reference center crop -> color hist
        _rc = ref[max(0, rh_ // 2 - rr_):rh_ // 2 + rr_, max(0, rw_ // 2 - rr_):rw_ // 2 + rr_]
        ref_hist = color_hist(_rc) if _rc.size and min(_rc.shape[:2]) >= 6 else None
        # topk photos by object-heatmap peak (center-preference)
        cands = []
        for i, (dname, photo) in enumerate(drones):
            heat = drone_heat(drone_patches[i], obj, bg, p)
            peak, loc = peak_and_center(heat, p)
            if loc is None:
                continue
            _, _, _, cxp, cyp = loc
            cdn = np.hypot(cxp - p / 2.0, cyp - p / 2.0) / (0.5 * np.hypot(p, p))
            cands.append((peak - CENTER_PREF * float(cdn), peak, i, dname, photo))
        cands.sort(reverse=True)
        # AUTO-CALIBRATE (#3): peak threshold background se, par sirf lenient direction (fixed se strict nahi)
        if AUTO_CALIBRATE and cands:
            bg_peak = float(np.median([c[1] for c in cands]))      # NOTE: 'bg' = DINO prototype, alag rakho
            pmin = min(MIN_FOUND_PEAK, max(PEAK_FLOOR, bg_peak * PEAK_RATIO))
        else:
            pmin = MIN_FOUND_PEAK
        topk = [c for c in cands if c[1] >= pmin][:TOPK]
        # har topk photo ka best-verify location -> candidate + ground position
        cand_list = []
        for _, peak_i, i, dname_i, photo_i in topk:
            heat = drone_heat(drone_patches[i], obj, bg, p)
            vr = localize_verified(M, ref_embs, ref_hist, photo_i, heat, p)
            if vr is None:
                continue
            vsim, cx, cy, rad = vr
            gp = gpos(locs.get(os.path.splitext(dname_i)[0]), cx, cy)
            cand_list.append((vsim, cx, cy, rad, peak_i, dname_i, photo_i, gp))
        cand_list.sort(key=lambda c: c[0], reverse=True)
        tdata.append((name, ref, len(imgs), cand_list))

    # ── PASS 2: GLOBAL greedy assignment (mutual exclusion by ground position) ──
    #   Order: jiska best candidate ka PEAK zyada (object-match strong) wo pehle claim kare.
    #   Har target apni highest-vsim (>=VERIFY_MIN) candidate le jo claimed spot ke MIN_SEP me na ho.
    order = sorted(range(len(tdata)),
                   key=lambda k: max((c[4] for c in tdata[k][3]), default=0.0), reverse=True)
    assign = {}; claimed = []
    for k in order:
        name, ref, ns, cand_list = tdata[k]
        pick = None
        for cand in cand_list:
            if cand[0] < VERIFY_MIN:
                break                                  # vsim-desc: aage sab neeche
            gp = cand[7]
            if gp is None or all(math.hypot(gp[0]-q[0], gp[1]-q[1]) >= MIN_SEP_M for q in claimed):
                pick = cand; break                     # non-conflicting best-verify
        assign[name] = pick
        if pick is not None and pick[7] is not None:
            claimed.append(pick[7])

    # ── PASS 3: draw + CSV ──
    for name, ref, ns, cand_list in tdata:
        pick = assign.get(name)
        if pick is None:
            vv = cand_list[0][0] if cand_list else 0.0
            print(f"  {name:16s} | NOT FOUND (verify/dup {vv:.2f})")
            rows.append({"feature": name, "identity": "", "seeds_fused": ns,
                         "matched_photo": "", "confidence": "NOT_FOUND", "loftr": 0, "superpoint": 0,
                         "dino_sim": round(vv, 3), "cx_lr": -1, "cy_lr": -1, "radius_lr": -1,
                         "localizer": "none", "hd_proof": "", "hx_hd": -1, "hy_hd": -1, "hr_hd": -1})
            continue
        vsim, cx, cy, rad, peak, dname, photo, gp = pick
        conf = "HIGH" if vsim >= 0.60 else "MED" if vsim >= 0.50 else "LOW"
        circ = fs.draw_circle(photo, cx, cy, rad)
        tt = ref.copy()
        cv2.putText(tt, "REFERENCE", (10, 26), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        cv2.putText(circ, f"{conf} {dname} peak{peak:.2f} V{vsim:.2f}", (10, 26),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 0), 2)
        cv2.imwrite(os.path.join(vis_dir, f"{name}.jpg"), np.hstack([tt, circ]))
        hx = hy = hr = -1; proof_name = ""
        hd_path = hd_map.get(dname)
        if hd_path and os.path.exists(hd_path):
            hd = cv2.imread(hd_path)
            if hd is not None:
                hh, hw = hd.shape[:2]
                hx = cx / PROC_W * hw; hy = cy / PROC_H * hh; hr = rad / PROC_W * hw
                # HD PROOF = feature ka TIGHT crop (seed jaisा framing, minimal background)
                # -> rulebook 11.4.4: "excessive surrounding background" pe marks katte, tight = max marks.
                half = int(max(hr * 2.2, hw * 0.06))
                x0 = max(0, int(hx - half)); y0 = max(0, int(hy - half))
                x1 = min(hw, int(hx + half)); y1 = min(hh, int(hy + half))
                crop = hd[y0:y1, x0:x1]                          # TIGHT crop -> LR seed (11.3.8a)
                # HD PROOF (11.3.8b/11.4.4): NATIVE pixels, feature-centered, shorter side ~720 -> SHARP.
                bw = min(max(2 * half, 720), hw); bh = min(max(2 * half, 720), hh)
                hx0 = max(0, min(int(hx) - bw // 2, hw - bw)); hy0 = max(0, min(int(hy) - bh // 2, hh - bh))
                hd_crop = hd[hy0:hy0 + bh, hx0:hx0 + bw]         # no upscale -> full native sharpness
                if hd_crop.size:
                    if min(hd_crop.shape[:2]) < 720:            # tiny-photo fallback only
                        s = 720.0 / max(1, min(hd_crop.shape[:2]))
                        hd_crop = cv2.resize(hd_crop, (round(hd_crop.shape[1] * s), round(hd_crop.shape[0] * s)),
                                             interpolation=cv2.INTER_CUBIC)
                    cv2.imwrite(os.path.join(proof_dir, f"{name}.jpg"), hd_crop)
                    proof_name = f"{name}.jpg"
                if crop.size:
                    # LR DELIVERABLE (11.3.8a): TIGHT feature crop -> 128 LR (seed se match hua)
                    cv2.imwrite(os.path.join(lr_dir, f"{name}.png"),
                                cv2.resize(crop, (128, 128), interpolation=cv2.INTER_AREA))
        print(f"  {name:16s} | peak={peak:.3f} V{vsim:.2f} -> {dname:16s} [{conf}]")
        rows.append({"feature": name, "identity": "", "seeds_fused": ns,
                     "matched_photo": dname, "confidence": conf, "loftr": 0, "superpoint": 0,
                     "dino_sim": round(peak, 3), "cx_lr": int(cx), "cy_lr": int(cy),
                     "radius_lr": int(rad), "localizer": "robust_dino",
                     "hd_proof": proof_name, "hx_hd": int(hx), "hy_hd": int(hy), "hr_hd": int(hr)})

    # ── candidates.json: har target ke top DISTINCT candidates (pipeline mutual-exclusion ke liye).
    #    Pipeline inke ACCURATE field positions (stitch mapping) se overlapping targets ko alag deta. ──
    import json as _json
    cand_out = {}
    for name, ref, ns, cand_list in tdata:
        picks = []                                         # top candidates (NO dedup) -- multi-photo
        for (vsim, cx, cy, rad, peak, dname, photo, gp) in cand_list:  # vsim-desc
            if vsim < VERIFY_MIN:
                break
            hx = hy = -1
            hd_path = hd_map.get(dname)
            if hd_path and os.path.exists(hd_path):
                hd = cv2.imread(hd_path)
                if hd is not None:
                    hh, hw = hd.shape[:2]
                    hx = cx / PROC_W * hw; hy = cy / PROC_H * hh
            picks.append({"stem": os.path.splitext(dname)[0], "hx_hd": int(hx), "hy_hd": int(hy),
                          "cx_lr": int(cx), "cy_lr": int(cy), "radius_lr": int(rad),
                          "vsim": round(float(vsim), 3), "peak": round(float(peak), 3)})
            if len(picks) >= 6:                            # multiple photos ka same object bhi -> averaging
                break
        cand_out[name] = picks
    with open(os.path.join(OUT_DIR, "candidates.json"), "w") as f:
        _json.dump(cand_out, f, indent=1)

    if rows:
        with open(os.path.join(OUT_DIR, "fused_results.csv"), "w", newline="") as f:
            wr = csv.DictWriter(f, fieldnames=list(rows[0].keys())); wr.writeheader(); wr.writerows(rows)
    print(f"\nDone. Visuals -> {vis_dir}\n      CSV -> {os.path.join(OUT_DIR, 'fused_results.csv')}"
          f"\n      Candidates -> {os.path.join(OUT_DIR, 'candidates.json')}")


if __name__ == "__main__":
    run()
