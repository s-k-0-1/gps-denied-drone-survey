#!/usr/bin/env python3
"""
IRoC-U 2026 -- FUSED SEED SEARCH  (LR -> LR, multi-seed fusion)
==============================================================================
Har feature ke 1-4 SEED LR images (target CENTER me hote hain):

  1) FUSE     : seeds ko align karke ek CLEAN reference (denoised + sharp). 1 seed -> seedha denoise.
  2) SEARCH   : fused reference ko har DRONE LR photo me dhoondo -- LoFTR + SuperPoint inliers
                (geometric) + DINOv2 global (semantic, low-texture pe bhi strong).
                score = loftr + superpoint + 200*max(0, dino_sim-0.55). Best score = woh photo.
  3) THRESHOLD: best score < MIN_FOUND_SCORE -> NOT FOUND (koi random match nahi aata).
  4) CIRCLE   : target reference me CENTER me hota hai -> DINOv2 semantic localization se drone
                photo me target center -> circle (segment/crop ki zaroorat NAHI).

FOLDER:
  ~/advanced_matcher/targets/<feature>/seed1.png seed2.png ...   (har subfolder = ek feature)
  ~/advanced_matcher/drone_photos/    (HD drone photos; LR auto banti hai)

RUN:
  python3 fused_search.py
OUTPUT:
  ~/advanced_matcher/results/stage3_targets/visuals/<feature>.jpg   (target | LR drone+circle)
  ~/advanced_matcher/results/stage3_targets/proof_hd/<feature>.jpg  (HD drone + circle -- rulebook 10.5.7 proof)
  ~/advanced_matcher/results/stage3_targets/fused/<feature>.png     (clean fused reference -- debug)
  ~/advanced_matcher/results/stage3_targets/fused_results.csv       (LR + HD coords; NOT FOUND = empty matched_photo)

NOTE (rulebook): matching 128x128 LR se hota hai (HD -> to_lr -> 128). HD sirf PROOF + coordinate
calc ke liye. HD proof tab milega jab drone_photos/ me HD originals hon.
"""

import os, glob, csv, time
import cv2
import numpy as np
import torch

# ================= CONFIG =================
BASE_DIR     = os.path.expanduser("~/advanced_matcher")
DRONE_HD_DIR = os.path.join(BASE_DIR, "drone_photos")
DRONE_LR_DIR = os.path.join(BASE_DIR, "drone_photos_lr")
TARGETS_DIR  = os.path.join(BASE_DIR, "targets")
OUT_DIR      = os.path.join(BASE_DIR, "results", "stage3_targets")   # sab results -> results/ ke andar

LR_SIZE      = 128
PROC_W, PROC_H = 640, 480
EXTS   = ('*.jpg','*.jpeg','*.png','*.JPG','*.JPEG','*.PNG')
DEVICE = "cuda" if torch.cuda.is_available() else "cpu"

# DETECTION THRESHOLD: best match ka score is se KAM -> NOT FOUND (random match nahi).
# score = loftr_inliers + superpoint_inliers + 200*max(0, dino_sim-0.55) - edge_penalty.
# Zyada strict chahiye to badhao, zyada lenient chahiye to ghatao.
MIN_FOUND_SCORE = 24.0

# SELECTION center-preference: jis drone photo me target CENTER ke paas ho usko prefer karo
# (edge/aadha-cut photo -> galat position). edge_penalty = CENTER_PENALTY * (center_dist_norm^2),
# center_dist_norm: 0 = photo center, ~1 = corner. Badhao = center zyada zaroori.
CENTER_PENALTY = 30.0

# VERIFICATION: localize hone ke baad, drone photo me circle wale crop ko reference target se
# DINOv2 se compare karo. True match -> crop target jaisa (high sim). FALSE match (target survey
# me hai hi nahi) -> crop sirf ground -> low sim -> NOT FOUND. Yeh threshold se alag, appearance check.
# sim is se KAM -> NOT FOUND. Badhao = strict (zyada reject), ghatao = lenient.
VERIFY_MIN = 0.50


# ================= MODELS =================
def load_models():
    print(f"Device: {DEVICE}" + (f"  GPU: {torch.cuda.get_device_name(0)}" if DEVICE == "cuda" else ""))
    from lightglue import SuperPoint, LightGlue
    sp_ext = SuperPoint(max_num_keypoints=4096).eval().to(DEVICE)
    sp_mat = LightGlue(features="superpoint").eval().to(DEVICE)
    import kornia.feature as KF
    print("Loading LoFTR...")
    loftr = KF.LoFTR(pretrained="outdoor").eval().to(DEVICE)
    print("Loading DINOv2 (semantic localization)...")
    dino = torch.hub.load('facebookresearch/dinov2', 'dinov2_vits14').eval().to(DEVICE)
    print("Models ready.\n")
    return {"sp_ext": sp_ext, "sp_mat": sp_mat, "loftr": loftr, "dino": dino}


# ================= HELPERS =================
def to_work(im):  return cv2.resize(im, (PROC_W, PROC_H), interpolation=cv2.INTER_CUBIC)
def to_lr(img):   return cv2.resize(img, (LR_SIZE, LR_SIZE), interpolation=cv2.INTER_AREA)

def gray_t(img_bgr, W, H):
    g = cv2.resize(cv2.cvtColor(img_bgr, cv2.COLOR_BGR2GRAY), (W, H))
    return (torch.from_numpy(g)[None, None].float() / 255.0).to(DEVICE)

def list_imgs_flat(root):
    f = []
    for e in EXTS: f += glob.glob(os.path.join(root, e))
    return sorted(set(f))

def group_seeds(targets_dir):
    """Har immediate SUBFOLDER = ek feature (uske andar ke saare images = seeds).
    Agar seedha targets/ me loose images hain to har image apna alag feature."""
    groups = {}
    if not os.path.isdir(targets_dir): return groups
    subs = [d for d in sorted(os.listdir(targets_dir)) if os.path.isdir(os.path.join(targets_dir, d))]
    for d in subs:
        seeds = list_imgs_flat(os.path.join(targets_dir, d))
        if seeds: groups[d] = seeds
    for p in list_imgs_flat(targets_dir):          # loose images -> single-seed features
        groups[os.path.splitext(os.path.basename(p))[0]] = [p]
    return groups


# ================= 1) FUSE SEEDS -> CLEAN REFERENCE =================
def fuse_seeds(M, seed_paths):
    """3-4 LR seeds ko LoFTR-HOMOGRAPHY se PIXEL-LEVEL align karke average -> ek CLEAN reference.
    LoFTR dense matches -> homography (perspective/scale/rotation/translation sab handle) -> warp ->
    object exactly overlap (ghosting nahi). Per-pixel VALID averaging (border artifacts nahi).
    Sabse sharp seed = base. Align fail/kam-inliers seed skip. 1 seed -> seedha denoise."""
    imgs = [cv2.imread(p) for p in seed_paths]
    imgs = [im for im in imgs if im is not None]
    if not imgs: return None, None, 0
    ups = [to_work(im) for im in imgs]
    if len(ups) == 1:
        return cv2.bilateralFilter(ups[0], 7, 50, 50), ups[0], 1
    sharp = [cv2.Laplacian(cv2.cvtColor(u, cv2.COLOR_BGR2GRAY), cv2.CV_64F).var() for u in ups]
    bi = int(np.argmax(sharp)); base = ups[bi]
    bt = gray_t(base, PROC_W, PROC_H)
    acc = base.astype(np.float32)
    cntmap = np.ones((PROC_H, PROC_W), np.float32)          # har pixel kitni baar average hua
    ones = np.ones((PROC_H, PROC_W), np.float32)
    cnt = 1
    for i, u in enumerate(ups):
        if i == bi: continue
        try:
            with torch.no_grad():                           # seed -> base dense matches
                c = M["loftr"]({"image0": gray_t(u, PROC_W, PROC_H), "image1": bt})
            k0 = c["keypoints0"].cpu().numpy(); k1 = c["keypoints1"].cpu().numpy()
            if len(k0) < 20: continue
            Hh, mask = cv2.findHomography(k0, k1, cv2.RANSAC, 3.0)   # seed -> base
            if Hh is None or int(mask.sum()) < 15: continue
            al = cv2.warpPerspective(u, Hh, (PROC_W, PROC_H), flags=cv2.INTER_CUBIC, borderValue=(0, 0, 0))
            valid = cv2.warpPerspective(ones, Hh, (PROC_W, PROC_H)) > 0.5   # warp-in pixels
            acc[valid] += al[valid].astype(np.float32)
            cntmap[valid] += 1.0; cnt += 1
        except Exception:
            continue
    fused = (acc / cntmap[..., None]).astype(np.uint8)
    fused = cv2.bilateralFilter(fused, 7, 50, 50)
    blur = cv2.GaussianBlur(fused, (0, 0), 2.0)
    fused = cv2.addWeighted(fused, 1.5, blur, -0.5, 0)      # halka unsharp (edges sharp)
    return fused, base, cnt                                 # base = sharpest seed (matching ke liye, texture intact)


# ================= SEGMENT OBJECT (central distinct blob) =================
def segment_object(work):
    """Object = ground/tape se sabse DISTINCT compact blob (jahan bhi ho -- central nahi maana).
    Lab anomaly -> Otsu -> blob jiska mean-anomaly (distinctness) sabse zyada. Tape (yellow)
    excluded. NO central bias (object reference me central ho ya na ho). None agar fail."""
    H, W = work.shape[:2]
    hsv = cv2.cvtColor(work, cv2.COLOR_BGR2HSV)
    yellow = cv2.inRange(hsv, (18, 50, 50), (45, 255, 255))
    lab = cv2.cvtColor(work, cv2.COLOR_BGR2LAB).astype(np.float32)
    bg = np.median(lab.reshape(-1, 3), axis=0)
    amap = np.linalg.norm(lab - bg, axis=2)                       # raw anomaly (distinctness)
    gA = float(np.median(amap)) + 1e-6
    dn = cv2.normalize(amap, None, 0, 255, cv2.NORM_MINMAX).astype(np.uint8)
    _, m = cv2.threshold(dn, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
    m[yellow > 0] = 0
    m = cv2.morphologyEx(m, cv2.MORPH_OPEN,  np.ones((3, 3), np.uint8))
    m = cv2.morphologyEx(m, cv2.MORPH_CLOSE, np.ones((21, 21), np.uint8))   # scattered (screws) connect
    n, lbl, stats, cent = cv2.connectedComponentsWithStats(m)
    if n <= 1: return None
    best_i, best = None, -1
    for i in range(1, n):
        x, y, w, h, area = stats[i]
        if area < 0.005 * H * W or area > 0.85 * H * W: continue
        compact = area / float(w * h + 1)
        ar = min(w, h) / float(max(w, h))
        manom = float(amap[lbl == i].mean()) / gA                # blob distinctness (object = high)
        sc = (manom ** 2) * (area ** 0.5) * (0.5 + 0.5 * compact) * (0.6 + 0.4 * ar)
        if sc > best: best, best_i = sc, i
    if best_i is None: return None
    return cv2.dilate((lbl == best_i).astype(np.uint8) * 255, np.ones((9, 9), np.uint8))


# ================= 2) IDENTIFY TARGET =================
def identify_target(fused):
    """Fused reference se batao 'target kya h': color + shape + size."""
    m = segment_object(fused)
    if m is None: return "object (segmentation unclear)", None
    hsv = cv2.cvtColor(fused, cv2.COLOR_BGR2HSV)
    mh, ms, mv = cv2.mean(hsv, mask=m)[:3]
    if ms < 45:            color = "grey/metallic"
    elif mv < 70:          color = "dark"
    elif mh < 18 or mh > 165: color = "red/brown"
    elif mh < 33:          color = "tan/brown"
    elif mh < 90:          color = "green"
    else:                  color = "blue/grey"
    ys, xs = np.where(m > 0)
    w_ = xs.max() - xs.min() + 1; h_ = ys.max() - ys.min() + 1
    ar = min(w_, h_) / float(max(w_, h_))
    area_frac = float((m > 0).sum()) / (m.shape[0] * m.shape[1])
    solidity = float((m > 0).sum()) / float(w_ * h_ + 1)
    shape = "round/compact" if (ar > 0.7 and solidity > 0.6) else \
            "irregular/scattered" if solidity < 0.5 else "elongated"
    size = "large" if area_frac > 0.18 else "medium" if area_frac > 0.06 else "small"
    return f"{color}, {shape}, {size}", m


# ================= 3) MATCH (selection) =================
def loftr_inliers(M, ref, photo):
    with torch.no_grad():
        c = M["loftr"]({"image0": gray_t(ref, PROC_W, PROC_H), "image1": gray_t(photo, PROC_W, PROC_H)})
    k0 = c["keypoints0"].cpu().numpy(); k1 = c["keypoints1"].cpu().numpy()
    if len(k0) < 8: return 0, k0, k1, None
    Hh, mask = cv2.findHomography(k0, k1, cv2.RANSAC, 4.0)
    if mask is None: return 0, k0, k1, None
    return int(mask.sum()), k0, k1, mask.ravel().astype(bool)

def sp_inliers(M, ref, photo):
    """Proven format (lr_pipeline): SuperPoint.extract() + rbd. Errors -> 0 (LoFTR fallback)."""
    from lightglue.utils import rbd
    def ext(img):
        rgb = cv2.cvtColor(cv2.resize(img, (PROC_W, PROC_H)), cv2.COLOR_BGR2RGB)
        t = (torch.from_numpy(rgb).float().permute(2, 0, 1)[None] / 255.0).to(DEVICE)
        with torch.no_grad(): f = rbd(M["sp_ext"].extract(t))
        return f, t.shape[-1], t.shape[-2]
    try:
        f0, w0, h0 = ext(ref); f1, w1, h1 = ext(photo)
        d0 = {"keypoints": f0["keypoints"].unsqueeze(0), "descriptors": f0["descriptors"].unsqueeze(0),
              "image_size": torch.tensor([[w0, h0]]).float().to(DEVICE)}
        d1 = {"keypoints": f1["keypoints"].unsqueeze(0), "descriptors": f1["descriptors"].unsqueeze(0),
              "image_size": torch.tensor([[w1, h1]]).float().to(DEVICE)}
        with torch.no_grad(): out = rbd(M["sp_mat"]({"image0": d0, "image1": d1}))
        m = out["matches"].cpu().numpy()
        if len(m) < 6: return 0
        p0 = f0["keypoints"].cpu().numpy()[m[:, 0]]; p1 = f1["keypoints"].cpu().numpy()[m[:, 1]]
        Hh, mask = cv2.findHomography(p0, p1, cv2.RANSAC, 4.0)
        return int(mask.sum()) if mask is not None else len(m)
    except Exception:
        return 0


# ================= 4) LOCALIZE =================
def ref_object_hist(ref):
    """Reference me OBJECT ka color = top-anomaly (sabse distinct) pixels (tape excluded) ka HSV hist.
    Segmentation CENTROID pe depend nahi -- sirf object ka rang chahiye (robust)."""
    hsv = cv2.cvtColor(ref, cv2.COLOR_BGR2HSV)
    yellow = cv2.inRange(hsv, (18, 50, 50), (45, 255, 255))
    lab = cv2.cvtColor(ref, cv2.COLOR_BGR2LAB).astype(np.float32)
    bg = np.median(lab.reshape(-1, 3), axis=0)
    amap = np.linalg.norm(lab - bg, axis=2); amap[yellow > 0] = 0
    thr = float(np.percentile(amap, 88))                       # top ~12% anomalous = object
    mask = (amap >= thr).astype(np.uint8) * 255
    if int((mask > 0).sum()) < 40: return None
    h = cv2.calcHist([hsv], [0, 1], mask, [40, 48], [0, 180, 0, 256]); cv2.normalize(h, h, 0, 1, cv2.NORM_MINMAX)
    return h

def density_peak(k1):
    """Match-DENSITY peak (drone coords): textured object (screws/gravel) pe matches concentrate
    karte hain -> peak object pe. Smooth object ke aas-paas. None agar kam matches."""
    if k1 is None or len(k1) < 8: return None
    gh, gw = PROC_H // 8 + 1, PROC_W // 8 + 1
    hist = np.zeros((gh, gw), np.float32)
    for x, y in k1:
        gx, gy = int(x) // 8, int(y) // 8
        if 0 <= gy < gh and 0 <= gx < gw: hist[gy, gx] += 1.0
    hist = cv2.GaussianBlur(hist, (0, 0), 1.5)
    _, _, _, ml = cv2.minMaxLoc(hist)
    return float(ml[0] * 8 + 4), float(ml[1] * 8 + 4)

def object_blob(photo, rh, anchors, radius):
    """Drone me distinct object-COLOR blob jo kisi bhi ANCHOR ke radius me ho -> globally best
    (anomaly * size * color). Anchors = [seg-weighted-center, match-density-peak]. Tape excluded.
    None agar koi nahi. (closeness gate, par score me nahi -> sahi object color/distinctness se jeete)."""
    H, W = photo.shape[:2]
    hsv = cv2.cvtColor(photo, cv2.COLOR_BGR2HSV)
    yellow = cv2.inRange(hsv, (18, 50, 50), (45, 255, 255))
    lab = cv2.cvtColor(photo, cv2.COLOR_BGR2LAB).astype(np.float32)
    bg = np.median(lab.reshape(-1, 3), axis=0)
    amap = cv2.GaussianBlur(np.linalg.norm(lab - bg, axis=2), (0, 0), 2.0); gA = float(np.median(amap)) + 1e-6
    an = cv2.normalize(amap, None, 0, 255, cv2.NORM_MINMAX).astype(np.uint8); an[yellow > 0] = 0
    _, th = cv2.threshold(an, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU); th[yellow > 0] = 0
    th = cv2.morphologyEx(th, cv2.MORPH_OPEN,  np.ones((3, 3), np.uint8))
    th = cv2.morphologyEx(th, cv2.MORPH_CLOSE, np.ones((19, 19), np.uint8))   # scattered (screws) connect
    n, lbl, stats, cent = cv2.connectedComponentsWithStats(th)
    best = None
    for i in range(1, n):
        area = stats[i, cv2.CC_STAT_AREA]
        if area < 0.004 * H * W or area > 0.5 * H * W: continue
        ccx, ccy = float(cent[i][0]), float(cent[i][1])
        if min(np.hypot(ccx - ax, ccy - ay) for ax, ay in anchors) > radius: continue   # kisi anchor ke paas nahi
        bm = (lbl == i)
        manom = float(amap[bm].mean()) / gA
        if manom < 1.1: continue
        csim = 0.5
        if rh is not None:
            bh = cv2.calcHist([hsv], [0, 1], (bm.astype(np.uint8) * 255), [40, 48], [0, 180, 0, 256]); cv2.normalize(bh, bh, 0, 1, cv2.NORM_MINMAX)
            csim = float(cv2.compareHist(rh, bh, cv2.HISTCMP_CORREL))
            if csim < 0.08: continue
        sc = (manom ** 1.5) * np.sqrt(area) * (0.3 + 0.7 * max(csim, 0.0))
        if best is None or sc > best[0]:
            ww, hh = stats[i, cv2.CC_STAT_WIDTH], stats[i, cv2.CC_STAT_HEIGHT]
            best = (sc, ccx, ccy, 0.5 * max(ww, hh))
    if best is None: return None
    return best[1], best[2], float(min(max(best[3], PROC_W * 0.05), PROC_W * 0.2))

def weighted_center(k0, k1, ocx, ocy, orad):
    """Fallback: object-center ke aaspaas matches ka Gaussian-weighted avg (MAD outlier reject)."""
    sigma = max(orad, PROC_W * 0.12)
    w = np.exp(-((k0[:, 0] - ocx) ** 2 + (k0[:, 1] - ocy) ** 2) / (2.0 * sigma * sigma))
    sel = w > 0.25
    if int(sel.sum()) < 6: sel = w > 0.10
    if int(sel.sum()) < 6: return None
    p1, ww = k1[sel], w[sel]
    mx, my = float(np.median(p1[:, 0])), float(np.median(p1[:, 1]))
    dd = np.hypot(p1[:, 0] - mx, p1[:, 1] - my); mad = float(np.median(dd)) + 1e-6
    keep = dd < 2.5 * mad
    if int(keep.sum()) >= 4: p1, ww = p1[keep], ww[keep]
    sw = float(ww.sum())
    cx = float((p1[:, 0] * ww).sum() / sw); cy = float((p1[:, 1] * ww).sum() / sw)
    rad = float(np.percentile(np.hypot(p1[:, 0] - cx, p1[:, 1] - cy), 80))
    return cx, cy, float(min(max(rad, PROC_W * 0.05), PROC_W * 0.18))

_DINO_MEAN = torch.tensor([0.485, 0.456, 0.406]).view(3, 1, 1)
_DINO_STD  = torch.tensor([0.229, 0.224, 0.225]).view(3, 1, 1)

def dino_patches(M, img_bgr, size=448):
    """DINOv2 patch features (p x p x D), L2-normalized. p = size/14."""
    rgb = cv2.cvtColor(cv2.resize(img_bgr, (size, size)), cv2.COLOR_BGR2RGB)
    t = torch.from_numpy(rgb).float().permute(2, 0, 1) / 255.0
    t = ((t - _DINO_MEAN) / _DINO_STD)[None].to(DEVICE)
    with torch.no_grad():
        f = M["dino"].forward_features(t)["x_norm_patchtokens"][0]   # (N, D)
    p = size // 14
    return torch.nn.functional.normalize(f, dim=-1).reshape(p, p, -1)


def dino_global(M, img_bgr, size=224):
    """DINOv2 GLOBAL descriptor (CLS token), L2-normalized. Semantic -> low-texture (gravel) pe bhi
    strong (jahan LoFTR/SuperPoint geometric weak). Selection score me add hota hai (#3)."""
    rgb = cv2.cvtColor(cv2.resize(img_bgr, (size, size)), cv2.COLOR_BGR2RGB)
    t = torch.from_numpy(rgb).float().permute(2, 0, 1) / 255.0
    t = ((t - _DINO_MEAN) / _DINO_STD)[None].to(DEVICE)
    with torch.no_grad():
        f = M["dino"].forward_features(t)["x_norm_clstoken"][0]      # (D,)
    return torch.nn.functional.normalize(f, dim=0).float().cpu().numpy()

def dino_crop_sim(M, ref, photo, cx, cy, rad):
    """VERIFICATION: drone photo me localized crop (circle wali jagah) vs reference ka CENTER crop
    (target centered hota hai). DINOv2 global cosine sim. True match -> dono target -> high.
    FALSE match -> drone crop sirf ground -> low. Isse survey me na-hone wale target reject hote hain."""
    H, W = photo.shape[:2]
    r = int(max(rad, PROC_W * 0.08))
    dcrop = photo[max(0, int(cy) - r):min(H, int(cy) + r), max(0, int(cx) - r):min(W, int(cx) + r)]
    rh, rw = ref.shape[:2]
    rr = int(min(rh, rw) * 0.35)                                   # reference center (~target) crop
    rcrop = ref[max(0, rh // 2 - rr):rh // 2 + rr, max(0, rw // 2 - rr):rw // 2 + rr]
    if dcrop.size == 0 or rcrop.size == 0 or min(dcrop.shape[:2]) < 6 or min(rcrop.shape[:2]) < 6:
        return 0.0
    try:
        return float(np.dot(dino_global(M, dcrop), dino_global(M, rcrop)))
    except Exception:
        return 0.0


def localize_dino(M, ref, photo):
    """SOTA SEMANTIC LOCALIZATION (segmentation/color/blob ki zaroorat NAHI):
      1. Reference ke DINOv2 patches -> border patches = BACKGROUND prototype (ground).
      2. Object patches = background se sabse alag (foreground) -> OBJECT prototype.
      3. Drone ke har patch ki similarity: (object_proto) - (background_proto) = heatmap.
      4. Heatmap ka peak/weighted-centroid = drone me OBJECT. -> circle.
    DINOv2 features SEMANTIC hain -> low-contrast screws / weak-texture gravel dono pe kaam karta hai."""
    try:
        Fr = dino_patches(M, ref); Fd = dino_patches(M, photo)      # (p,p,D)
    except Exception as e:
        return None
    p = Fr.shape[0]; D = Fr.shape[-1]
    fr = Fr.reshape(-1, D); fd = Fd.reshape(-1, D)
    idx = torch.arange(p, device=fr.device)
    yy, xx = torch.meshgrid(idx, idx, indexing='ij')
    border = ((xx < 2) | (xx >= p - 2) | (yy < 2) | (yy >= p - 2)).reshape(-1)
    bg = torch.nn.functional.normalize(fr[border].mean(0), dim=0)   # background (ground) prototype
    ref_fg = 1.0 - (fr @ bg)                                        # har ref-patch ka foreground-ness
    k = max(6, int(0.15 * p * p))
    obj_idx = torch.topk(ref_fg, k).indices                        # sabse foreground patches = object
    obj = torch.nn.functional.normalize(fr[obj_idx].mean(0), dim=0) # OBJECT prototype
    heat = ((fd @ obj) - (fd @ bg)).reshape(p, p).float().cpu().numpy()   # drone object-heatmap
    heat = cv2.GaussianBlur(np.maximum(heat, 0.0), (0, 0), 1.0)
    if float(heat.max()) < 1e-6: return None
    hn = heat / float(heat.max())
    ys, xs = np.where(hn >= 0.55)                                   # high-similarity patches = object
    if len(xs) < 1:
        my, mx = np.unravel_index(int(hn.argmax()), hn.shape); ys, xs = np.array([my]), np.array([mx])
    wts = hn[ys, xs]
    cx_p = float((xs * wts).sum() / wts.sum()); cy_p = float((ys * wts).sum() / wts.sum())
    rad_p = float(np.sqrt(max(len(xs), 1) / np.pi))
    cx = (cx_p + 0.5) / p * PROC_W; cy = (cy_p + 0.5) / p * PROC_H  # patch -> pixel
    rad = rad_p / p * (PROC_W + PROC_H) / 2.0 * 1.15
    return cx, cy, float(min(max(rad, PROC_W * 0.05), PROC_W * 0.22))

def localize(ref, k0, k1, obj_mask, photo):
    """HYBRID: (1) geometric ANCHOR = object-center (segmentation) ke paas ke matches ka weighted avg
    (drone coords, sahi vicinity). (2) Us anchor ke PAAS distinct object-color blob pe snap (exact
    object center). Blob anchor-constrained hai -> door distractor nahi pakdega, par object thoda
    door bhi ho (seg error) to nearest distinct blob mil jaata. Blob na mile -> anchor."""
    if obj_mask is not None:
        ys, xs = np.where(obj_mask > 0)
        ocx, ocy = float(xs.mean()), float(ys.mean())
        orad = 0.5 * max(int(xs.max() - xs.min()), int(ys.max() - ys.min()))
    else:
        ocx, ocy, orad = PROC_W / 2.0, PROC_H / 2.0, PROC_W * 0.14
    if k0 is None or len(k0) < 8:                              # matches nahi -> center
        return ocx, ocy, float(min(max(orad, PROC_W * 0.05), PROC_W * 0.18)), "center"
    anchors = []
    wc = weighted_center(k0, k1, ocx, ocy, orad)               # A1: seg-center ke paas matches (drone)
    if wc is not None: anchors.append((wc[0], wc[1]))
    dp = density_peak(k1)                                      # A2: match-density peak (textured object)
    if dp is not None: anchors.append(dp)
    if not anchors:
        return ocx, ocy, float(min(max(orad, PROC_W * 0.05), PROC_W * 0.18)), "center"
    blob = object_blob(photo, ref_object_hist(ref), anchors, 0.28 * PROC_W)   # dono anchors ke paas, best object
    if blob is not None:
        return blob[0], blob[1], blob[2], "blob"
    if wc is not None: return wc[0], wc[1], wc[2], "match"
    return dp[0], dp[1], float(orad), "peak"


def draw_circle(img, cx, cy, r):
    out = img.copy()
    th = max(3, int(img.shape[1] / 280))           # HD pe mota, LR pe patla
    cv2.circle(out, (int(cx), int(cy)), int(max(r, 8)), (0, 0, 255), th)
    cv2.circle(out, (int(cx), int(cy)), max(3, th), (0, 0, 255), -1)
    return out


# ================= DRONE INDEX =================
def build_drone_lr():
    """HD drone photos -> 128 LR. Returns (lr_paths, hd_map) jahan hd_map[lr_basename]=HD source
    path (HD proof image ke liye)."""
    os.makedirs(DRONE_LR_DIR, exist_ok=True)
    src = list_imgs_flat(DRONE_HD_DIR)
    if not src:                                    # HD nahi to LR folder hi use karo (HD proof nahi milega)
        return list_imgs_flat(DRONE_LR_DIR), {}
    out, hd_map = [], {}
    for p in src:
        im = cv2.imread(p)
        if im is None: continue
        base = os.path.splitext(os.path.basename(p))[0] + ".png"
        o = os.path.join(DRONE_LR_DIR, base)
        cv2.imwrite(o, to_lr(im)); out.append(o)
        hd_map[base] = p                           # LR basename -> HD original
    return out, hd_map


# ================= RUN =================
def run():
    groups = group_seeds(TARGETS_DIR)
    if not groups:
        print(f"ERROR: {TARGETS_DIR} me koi feature folder/image nahi."); return
    drone_lr, hd_map = build_drone_lr()
    if not drone_lr:
        print(f"ERROR: drone photos nahi ({DRONE_HD_DIR} ya {DRONE_LR_DIR})."); return
    print(f"Features: {len(groups)} | Drone photos: {len(drone_lr)}" +
          (f" | HD proof: ON ({len(hd_map)})" if hd_map else " | HD proof: OFF (drone_photos/ me HD daalo)") + "\n")

    M = load_models()
    drones = [(os.path.basename(p), to_work(cv2.imread(p))) for p in drone_lr if cv2.imread(p) is not None]
    print("Indexing drone photos (DINOv2 global) ...")
    drone_embs = [dino_global(M, ph) for _, ph in drones]   # #3: semantic descriptors (low-texture robust)

    vis_dir = os.path.join(OUT_DIR, "visuals"); fus_dir = os.path.join(OUT_DIR, "fused")
    proof_dir = os.path.join(OUT_DIR, "proof_hd")
    os.makedirs(vis_dir, exist_ok=True); os.makedirs(fus_dir, exist_ok=True); os.makedirs(proof_dir, exist_ok=True)
    rows = []

    for name, seeds in groups.items():
        t0 = time.time()
        fused, base_seed, ncnt = fuse_seeds(M, seeds)      # fused=display/seg, base_seed=sharp (matching)
        if fused is None:
            print(f"  {name:16s} -> fuse FAIL (no images)"); continue
        cv2.imwrite(os.path.join(fus_dir, f"{name}.png"), fused)
        desc, omask = identify_target(fused)               # identity + object mask (fused = clean)

        # SEARCH: SHARPEST SEED se LoFTR+SuperPoint (geometric) + DINOv2 global (semantic).
        ref_emb = dino_global(M, base_seed)
        best = None  # (score, dname, photo, k0, k1, inlmask, lo, sp, dsim)
        for i, (dname, photo) in enumerate(drones):
            lo, k0, k1, im = loftr_inliers(M, base_seed, photo)
            sp = sp_inliers(M, base_seed, photo)
            dsim = float(np.dot(ref_emb, drone_embs[i]))            # DINOv2 semantic similarity (0-1)
            # target is photo me KAHAN? matched inlier keypoints ka centroid ~ target location.
            # Center ke paas -> accurate; edge/aadha-cut -> penalty (aisi photo demote).
            edge_pen = 0.0
            if im is not None and k1 is not None and int(im.sum()) >= 6:
                pts = k1[im]
                cdx = float(pts[:, 0].mean()) - PROC_W / 2.0
                cdy = float(pts[:, 1].mean()) - PROC_H / 2.0
                cdn = np.hypot(cdx, cdy) / (0.5 * np.hypot(PROC_W, PROC_H))   # 0=center .. ~1=corner
                edge_pen = CENTER_PENALTY * float(cdn) ** 2
            score = lo + sp + 200.0 * max(0.0, dsim - 0.55) - edge_pen        # semantic bonus - edge penalty
            if best is None or score > best[0]:
                best = (score, dname, photo, k0, k1, im, lo, sp, dsim)
        score, dname, photo, k0, k1, im, lo, sp, dsim = best

        # ---- (#3) THRESHOLD: best score kam -> NOT FOUND (koi random match nahi) ----
        if score < MIN_FOUND_SCORE:
            dt = time.time() - t0
            print(f"  {name:16s} | NOT FOUND (best score {score:.1f} < {MIN_FOUND_SCORE})  "
                  f"[best={dname} L{lo} S{sp} D{dsim:.2f}]  {dt:.1f}s")
            rows.append({"feature": name, "identity": desc, "seeds_fused": ncnt,
                         "matched_photo": "", "confidence": "NOT_FOUND", "loftr": lo, "superpoint": sp,
                         "dino_sim": round(dsim, 3),
                         "cx_lr": -1, "cy_lr": -1, "radius_lr": -1, "localizer": "none",
                         "hd_proof": "", "hx_hd": -1, "hy_hd": -1, "hr_hd": -1})
            continue

        conf = "HIGH" if score >= 60 else "MED" if score >= 25 else "LOW"

        # LOCALIZE: DINOv2 semantic (PRIMARY; target center me -> seg/crop ki zaroorat NAHI) -> geom fallback
        if im is not None and k0 is not None:
            k0i, k1i = k0[im], k1[im]
        else:
            k0i, k1i = k0, k1
        dl = localize_dino(M, fused, photo)
        if dl is not None:
            cx, cy, rad, loc_m = dl[0], dl[1], dl[2], "dino"
        else:
            cx, cy, rad, loc_m = localize(fused, k0i, k1i, omask, photo)

        # ---- VERIFICATION: localized crop reference target jaisa hai? (false/survey-me-nahi reject) ----
        vsim = dino_crop_sim(M, fused, photo, cx, cy, rad)
        if vsim < VERIFY_MIN:
            dt = time.time() - t0
            print(f"  {name:16s} | NOT FOUND (verify {vsim:.2f} < {VERIFY_MIN})  "
                  f"[best={dname} L{lo} S{sp} D{dsim:.2f} score{score:.0f}]  {dt:.1f}s")
            rows.append({"feature": name, "identity": desc, "seeds_fused": ncnt,
                         "matched_photo": "", "confidence": "NOT_FOUND", "loftr": lo, "superpoint": sp,
                         "dino_sim": round(dsim, 3),
                         "cx_lr": -1, "cy_lr": -1, "radius_lr": -1, "localizer": "none",
                         "hd_proof": "", "hx_hd": -1, "hy_hd": -1, "hr_hd": -1})
            continue

        circ = draw_circle(photo, cx, cy, rad)
        tt = fused.copy()
        if omask is not None:
            yy, xx = np.where(omask > 0)
            cv2.circle(tt, (int(xx.mean()), int(yy.mean())), 8, (0, 255, 0), 2)
        cv2.putText(tt, "FUSED TARGET", (10, 26), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        cv2.putText(circ, f"{conf} {dname} L{lo} S{sp} loc={loc_m}", (10, 26), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 0), 2)
        vis = np.hstack([tt, circ])
        cv2.imwrite(os.path.join(vis_dir, f"{name}.jpg"), vis)

        # ---- HD PROOF IMAGE (rulebook 10.5.7): LR circle ko HD pe map karke save ----
        # framing same hai (HD->128->640 sirf resolution badalti), to fraction preserve hota hai.
        hx = hy = hr = -1; proof_name = ""
        hd_path = hd_map.get(dname)
        if hd_path and os.path.exists(hd_path):
            hd = cv2.imread(hd_path)
            if hd is not None:
                hh, hw = hd.shape[:2]
                hx = cx / PROC_W * hw; hy = cy / PROC_H * hh; hr = rad / PROC_W * hw
                cv2.imwrite(os.path.join(proof_dir, f"{name}.jpg"), draw_circle(hd, hx, hy, hr))
                proof_name = f"{name}.jpg"

        dt = time.time() - t0
        print(f"  {name:16s} | {desc:32s} | seeds={ncnt} -> {dname:16s} [{conf} L{lo} S{sp} D{dsim:.2f} V{vsim:.2f}] loc={loc_m}  {dt:.1f}s")
        rows.append({"feature": name, "identity": desc, "seeds_fused": ncnt,
                     "matched_photo": dname, "confidence": conf, "loftr": lo, "superpoint": sp,
                     "dino_sim": round(dsim, 3),
                     "cx_lr": int(cx), "cy_lr": int(cy), "radius_lr": int(rad), "localizer": loc_m,
                     "hd_proof": proof_name, "hx_hd": int(hx), "hy_hd": int(hy), "hr_hd": int(hr)})

    if rows:
        with open(os.path.join(OUT_DIR, "fused_results.csv"), "w", newline="") as f:
            wr = csv.DictWriter(f, fieldnames=list(rows[0].keys())); wr.writeheader(); wr.writerows(rows)
    print(f"\nDone. Visuals -> {vis_dir}\n      HD proof -> {proof_dir}\n      Fused refs -> {fus_dir}\n      CSV -> {os.path.join(OUT_DIR,'fused_results.csv')}")


if __name__ == "__main__":
    run()
