#!/usr/bin/env python3
"""
iroc_pipeline_fixed.py  --  iroc_pipeline.py ka SAFE fixed version
==============================================================================
Original iroc_pipeline.py / fused_search.py / stage3_robust.py ko CHHEDA NAHI gaya.
Yeh unhe import karke sirf BUGGY functions ko FIXED versions se replace karta hai --
har fix alag block me + clearly commented. Errors ek-ek karke yahin add ho rahe hain.

RUN:
  python3 iroc_pipeline_fixed.py                 # full run (fixed)
  python3 iroc_pipeline_fixed.py --skip-stitch   # saare flags original jaise kaam karte hain
  python3 iroc_pipeline_fixed.py --skip-match

Stage 3 method niche STAGE3 se choose karo.

NOTE: z coordinate ko touch NAHI kiya (user ne bola z waise hi rahe).

FIXES applied (audit se):
  #2  stale CSV: matcher subprocess fail ho to purana fused_results.csv use na ho.
  (#3..#11 -- ek-ek karke add ho rahe hain.)
"""
import iroc_pipeline as base

# ── Stage 3 method choose: "fused_search.py" (purana fusion) ya "stage3_robust.py" (naya robust) ──
#    ROBUST default: top-K + verification se target 1 (rock pile) bhi milta hai (fused_search miss karta).
STAGE3 = "stage3_robust.py"
base.STAGE3_SCRIPT = STAGE3


# ═══════════════════════════════════════════════════════════════════════════════
# FIX #1 -- BASE-STATION-EXACT origin (rulebook 11.2.2 / 11.3.4: coords base station se)
#   Problem: abhi origin = yellow-inner corner. Final round me base station LOTTERY se kahin bhi
#            rakha jaata hai -> corner pe ho zaroori nahi -> coords galat frame me report honge.
#   Fix:     base station = VIO/ENU (0,0) = drone takeoff point. A_inv se usko mosaic pixel ->
#            M_persp se rectified pixel -> field offset (bs_fx, bs_fy). Har target se ye SUBTRACT ->
#            origin exactly base station. Target positions abhi bhi accurate rectified-pixel se aate
#            (sirf ek constant shift). A (SIFT calib) na mile to gracefully yellow-corner pe gir jaata.
#
#   BASE_STATION_EXACT = True  -> origin = ACTUAL base station (Final round ke liye SAHI).
#   BASE_STATION_EXACT = False -> origin = yellow corner (Qualifier/practice, purana behaviour bilkul same).
# ═══════════════════════════════════════════════════════════════════════════════
BASE_STATION_EXACT = True
_BS_RECT_PX = None       # base station ka rectified pixel (annotation grid origin) -- runtime pe set hota

# FIX #14 -- ASSIGNED INITIAL-HEADING frame (Final round)
#   Coords base-station origin pe hain, par axes yellow-ARENA edges ke along. Final round me coords
#   ASSIGNED heading (0/90/180/270) ke frame me chahiye ho sakte -> HEADING_ROT_DEG se saare (x,y) ko
#   origin ke around rotate. 0 = current (arena-edge axes, koi change nahi). Rulebook heading-aligned
#   axes maange to yahan set karo (e.g. 90). Direction galat lage to negative (e.g. -90) karo.
HEADING_ROT_DEG = 0.0


# ═══════════════════════════════════════════════════════════════════════════════
# FIX #2 -- STALE CSV on matcher failure
#   Problem: run_target_finding Stage-3 matcher ko subprocess se chalata hai. Agar woh crash
#            kare (missing lib/error) par pichhle run ka fused_results.csv maujood ho, to
#            pipeline PURANE (stale) matches chup-chaap use kar leta hai.
#   Fix:     matcher chalane se PEHLE stale CSV delete. Fail hone par CSV nahi banegi ->
#            run_target_finding [] return karega (galat purana result nahi).
# ═══════════════════════════════════════════════════════════════════════════════
_orig_run_target_finding = base.run_target_finding
def run_target_finding(csv_rows, origin_x, origin_y, origin_z):
    try:
        base.FUSED_CSV.unlink()               # stale hatao (matcher fresh likhega)
        base.log(f"  [fix#2] purana {base.FUSED_CSV.name} delete kiya (stale avoid)")
    except FileNotFoundError:
        pass
    res = _orig_run_target_finding(csv_rows, origin_x, origin_y, origin_z)
    # method label sahi karo: base pipeline "fused_search" HARDCODE karta hai chahe koi bhi matcher
    # chale. Actual matcher STAGE3 se dikhao (summary/targets.json me confusion na ho).
    tag = str(STAGE3).replace(".py", "")
    for r in (res or []):
        if r.get("method") == "fused_search":
            r["method"] = tag
    return res
base.run_target_finding = run_target_finding


# ═══════════════════════════════════════════════════════════════════════════════
# FIX #4 -- ARENA x/y AXIS CONVENTION verify/swap
#   Problem: compute_arena_axes yaw se x/y banata hai. Ground-truth me x aur y ULTE ho
#            sakte hain (convention mismatch). Verify karne ka clean tareeka chahiye.
#   Fix:     SWAP_AXES flag. Agar aapke known points se x/y interchanged aayen to True kar do.
# ═══════════════════════════════════════════════════════════════════════════════
SWAP_AXES = False        # x/y ulte aayen (ground truth se) -> True

def compute_arena_axes(csv_rows):
    np, math = base.np, base.math
    yaws = [r.get("yaw") for r in csv_rows if r.get("yaw") is not None]
    if not yaws:
        return None, None
    th = math.radians(float(np.mean(yaws)))
    x_hat = np.array([math.cos(th), math.sin(th)])
    y_hat = np.array([-math.sin(th), math.cos(th)])
    c = np.array([float(np.mean([r["x"] for r in csv_rows])),
                  float(np.mean([r["y"] for r in csv_rows]))])
    if np.dot(c, x_hat) < 0: x_hat = -x_hat
    if np.dot(c, y_hat) < 0: y_hat = -y_hat
    if SWAP_AXES:
        x_hat, y_hat = y_hat, x_hat            # FIX: convention match karne ke liye swap
    return x_hat, y_hat
base.compute_arena_axes = compute_arena_axes


# ═══════════════════════════════════════════════════════════════════════════════
# FIX #5 -- COORDS visual se MATCH nahi karte the (label alag frame me)
#   Problem: circle rectified image me sahi jagah (yellow-aligned frame), par label GPS-arena
#            (yaw-axes) se aata tha -> dono ~90° rotated frames -> target 2 top-left par x=7,
#            target 1 bottom-right par x=2.5 (ulta). Coords galat dikhte the.
#   Fix:     x,y ko RECTIFIED pixel (_rx,_ry) se compute -> label = grid position = jahan circle
#            actually hai. Origin = yellow corner (rect bottom-left). z unchanged (user ki request).
#
# FIX #9 -- DUPLICATE / OVERLAPPING targets warn (do features same jagah -> ek mislocalized).
# ═══════════════════════════════════════════════════════════════════════════════
DUP_M = 0.5              # is se paas do targets -> possible duplicate/mislocalization
SEP_M = 0.6              # mutual-exclusion: do targets ki field position is se paas -> conflict

# IMPROVEMENT #1 -- MULTIPLE INSTANCES per feature type (rulebook: har type ke 2-3 instances).
#   MAX_INSTANCES = 1  -> DEFAULT: kuch nahi badlta (abhi jaisa, har type ka 1 best match).
#   MAX_INSTANCES = 2/3 -> real arena pe: har type ke aur STRONG (>= EXTRA_VERIFY_MIN) + DISTINCT
#                          (>= SEP_M door) instances bhi report honge (name#2, name#3).
#   RULEBOOK V4.0 (Final round, 11.3): 3 UNIQUE seed targets (har seed ka 1 best match; extra HD
#   ki lower weightage). Isliye MAX_INSTANCES = 1 (unique targets). Multiple-instance interpretation
#   ke liye 2/3 kar sakte ho (opt-in), par Final round unique hai.
MAX_INSTANCES    = 1
EXTRA_VERIFY_MIN = 0.60  # extra instance ke liye strict verify (false extra avoid)

# IMPROVEMENT #2 (safe) -- MULTI-PHOTO position AVERAGING.
#   Ek target agar kai photos me dikhta (adjacent), har photo se thodi alag field-pos aati (mapping
#   error). Un candidates jo primary spot ke AVG_R me hain (same object) -> field-pos AVERAGE ->
#   per-photo error kam, position zyada accurate + robust. AVG_R = 0 -> feature off.
AVG_R = 0.5              # (m) is radius me ke same-object candidates average honge


def _mutual_exclusion(coords, results, photo_to_H, M_persp, field_h_m, bs_fx=0.0, bs_fy=0.0):
    """PIPELINE-level dedup (ACCURATE stitch positions se, matcher ki rough gpos se nahi).
    candidates.json (matcher) se har target ke alternates lo, unki EXACT field position stitch
    mapping (H -> mosaic -> M_persp -> rect) se nikaalo, phir greedy assign: strong-peak target
    pehle apni jagah claim kare; overlapping (SEP_M) lower target apni NEXT-BEST location le."""
    np, cv2, os, json = base.np, base.cv2, base.os, base.json
    cpath = base.TARGET_DIR / "candidates.json"
    if not cpath.exists() or field_h_m is None or M_persp is None or not photo_to_H:
        return
    cand_map = json.load(open(cpath))
    H_by_stem = {os.path.splitext(os.path.basename(k))[0]: v for k, v in photo_to_H.items()}

    def field_of(c):
        H = H_by_stem.get(c["stem"])
        if H is None or c.get("hx_hd", -1) < 0:
            return None
        pt = np.asarray(H, float) @ np.array([c["hx_hd"], c["hy_hd"], 1.0])
        mpx, mpy = float(pt[0]), float(pt[1])
        fpt = cv2.perspectiveTransform(np.array([[[mpx, mpy]]], np.float32), M_persp)[0, 0]
        rx, ry = float(fpt[0]), float(fpt[1])
        return (rx / base.PX_PER_M, field_h_m - ry / base.PX_PER_M, rx, ry)

    tc = {}                                    # name -> [(vsim, peak, fx, fy, rx, ry, cand), ...]
    for name, cl in cand_map.items():
        lst = []
        for c in cl:
            fp = field_of(c)
            if fp:
                lst.append((c["vsim"], c["peak"], fp[0], fp[1], fp[2], fp[3], c))
        if lst:
            tc[name] = lst

    def _regen(label, base_name, c):           # HD proof + per-target visual (correct location)
        try:
            hd_p = base.DRONE_HD_DIR / (c["stem"] + ".jpg")
            hd = cv2.imread(str(hd_p)) if hd_p.exists() else None
            if hd is None:
                return
            hh2, hw2 = hd.shape[:2]
            hr = c["radius_lr"] / 640.0 * hw2
            half = int(max(hr * 2.2, hw2 * 0.06))    # HD PROOF = feature ka TIGHT crop (seed jaisा, min bg)
            x0 = max(0, int(c["hx_hd"] - half)); y0 = max(0, int(c["hy_hd"] - half))
            x1 = min(hw2, int(c["hx_hd"] + half)); y1 = min(hh2, int(c["hy_hd"] + half))
            crop = hd[y0:y1, x0:x1]                                 # TIGHT crop -> LR seed (11.3.8a)
            (base.TARGET_DIR / "proof_hd").mkdir(exist_ok=True)
            (base.TARGET_DIR / "lr_match").mkdir(exist_ok=True)
            # HD PROOF (11.3.8b): NATIVE pixels, feature-centered, shorter side ~720 -> SHARP.
            bw = min(max(2 * half, 720), hw2); bh = min(max(2 * half, 720), hh2)
            hx0 = max(0, min(int(c["hx_hd"]) - bw // 2, hw2 - bw))
            hy0 = max(0, min(int(c["hy_hd"]) - bh // 2, hh2 - bh))
            hd_crop = hd[hy0:hy0 + bh, hx0:hx0 + bw]               # no upscale -> full native sharpness
            if hd_crop.size:
                if min(hd_crop.shape[:2]) < 720:
                    s = 720.0 / max(1, min(hd_crop.shape[:2]))
                    hd_crop = cv2.resize(hd_crop, (round(hd_crop.shape[1] * s), round(hd_crop.shape[0] * s)),
                                         interpolation=cv2.INTER_CUBIC)
                cv2.imwrite(str(base.TARGET_DIR / "proof_hd" / (label + ".jpg")), hd_crop)
            if crop.size:
                # LR DELIVERABLE (11.3.8a): TIGHT feature crop -> 128 LR
                cv2.imwrite(str(base.TARGET_DIR / "lr_match" / (label + ".png")),
                            cv2.resize(crop, (128, 128), interpolation=cv2.INTER_AREA))
            work = cv2.resize(hd, (640, 480))
            cv2.circle(work, (int(c["cx_lr"]), int(c["cy_lr"])), max(int(c["radius_lr"]), 12), (0, 0, 255), 3)
            cv2.circle(work, (int(c["cx_lr"]), int(c["cy_lr"])), 5, (0, 0, 255), -1)
            cv2.putText(work, label, (10, 26), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 0), 2)
            ref_p = base.TARGET_DIR / "fused" / (base_name + ".png")
            refimg = cv2.imread(str(ref_p)) if ref_p.exists() else None
            if refimg is not None:
                tt = cv2.resize(refimg, (640, 480))
                cv2.putText(tt, "REFERENCE", (10, 26), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                (base.TARGET_DIR / "visuals").mkdir(exist_ok=True)
                cv2.imwrite(str(base.TARGET_DIR / "visuals" / (label + ".jpg")), np.hstack([tt, work]))
        except Exception:
            pass

    order = sorted(tc, key=lambda n: max(x[1] for x in tc[n]), reverse=True)   # best-peak pehle
    claimed = []
    # ── PASS A: primary instance per target (dedup -- yeh behaviour unchanged) ──
    for name in order:
        pick = None
        for c in sorted(tc[name], key=lambda z: z[0], reverse=True):           # vsim desc
            fx, fy = c[2], c[3]
            if all((fx - qx) ** 2 + (fy - qy) ** 2 >= SEP_M * SEP_M for qx, qy in claimed):
                pick = c; break
        if pick is None:
            continue
        vsim, peak, fx, fy, rx, ry, cd = pick
        # MULTI-PHOTO AVERAGING (#2): same target ke candidates jo is spot ke AVG_R me hain (same
        # object, doosri photos se) -> unki field-pos average -> per-photo mapping error kam.
        if AVG_R > 0 and field_h_m is not None:
            near = [(c[2], c[3]) for c in tc[name]
                    if (c[2] - fx) ** 2 + (c[3] - fy) ** 2 <= AVG_R * AVG_R]
            if len(near) >= 2:
                fx = sum(nx for nx, _ in near) / len(near)
                fy = sum(ny for _, ny in near) / len(near)
                rx = fx * base.PX_PER_M; ry = (field_h_m - fy) * base.PX_PER_M
                base.log(f"  [avg] '{name}' {len(near)} photos se averaged -> ({fx:.2f},{fy:.2f})")
        claimed.append((fx, fy))
        moved = False
        if name in coords and coords[name]:
            nx, ny = fx - bs_fx, fy - bs_fy            # base-station-relative (bs=0 -> yellow corner)
            ox, oy = coords[name].get("x"), coords[name].get("y")
            moved = ox is not None and (abs(ox - nx) > 0.15 or abs(oy - ny) > 0.15)
            coords[name]["x"] = round(nx, 3); coords[name]["y"] = round(ny, 3)
            coords[name]["_rx"] = rx; coords[name]["_ry"] = ry
        for r in results:
            if r.get("target") == name:
                r["drone_photo"] = cd["stem"] + ".jpg"
                r["drone_pixel"] = [cd["hx_hd"], cd["hy_hd"]]
        if moved:
            base.log(f"  [dedup] '{name}' -> ({fx:.2f},{fy:.2f}) vsim={vsim:.2f}  (reassigned to {cd['stem']})")
            _regen(name, name, cd)

    # ── PASS B: EXTRA instances (#1, opt-in). MAX_INSTANCES>1 -> har type ke aur strong distinct spots ──
    if MAX_INSTANCES > 1:
        for name in order:
            got = 1
            for c in sorted(tc[name], key=lambda z: z[0], reverse=True):
                if got >= MAX_INSTANCES or c[0] < EXTRA_VERIFY_MIN:
                    break
                fx, fy = c[2], c[3]
                if all((fx - qx) ** 2 + (fy - qy) ** 2 >= SEP_M * SEP_M for qx, qy in claimed):
                    got += 1; claimed.append((fx, fy))
                    label = f"{name}#{got}"; cd = c[6]
                    coords[label] = {"x": round(fx - bs_fx, 3), "y": round(fy - bs_fy, 3),
                                     "z": (coords.get(name) or {}).get("z", 0.0),
                                     "_rx": c[4], "_ry": c[5]}
                    results.append({"target": label, "found": True, "method": "robust_dino",
                                    "drone_photo": cd["stem"] + ".jpg",
                                    "drone_pixel": [cd["hx_hd"], cd["hy_hd"]], "object_xyz": None})
                    base.log(f"  [instance] '{label}' -> ({fx:.2f},{fy:.2f}) vsim={c[0]:.2f}")
                    _regen(label, name, cd)


_orig_compute_map_coords = base.compute_map_coords
def compute_map_coords(*a, **k):
    global _BS_RECT_PX
    coords = _orig_compute_map_coords(*a, **k)
    results   = a[0] if len(a) > 0 else k.get("results")
    photo_toH = a[2] if len(a) > 2 else k.get("photo_to_H")
    M_persp   = a[3] if len(a) > 3 else k.get("M_persp")
    field_h_m = a[4] if len(a) > 4 else k.get("field_h_m")
    A_px2enu  = a[7] if len(a) > 7 else k.get("A_px2enu")

    # --- FIX #1: BASE-STATION-EXACT origin -> base station (VIO 0,0) ka field offset nikaalo ---
    bs_fx = bs_fy = 0.0; _BS_RECT_PX = None
    if BASE_STATION_EXACT and A_px2enu is not None and M_persp is not None and field_h_m is not None:
        try:
            cv2, np = base.cv2, base.np
            A_inv = cv2.invertAffineTransform(np.asarray(A_px2enu, np.float64))
            bpx = A_inv @ np.array([0.0, 0.0, 1.0])                       # VIO(0,0) -> mosaic pixel
            bf = cv2.perspectiveTransform(np.array([[[float(bpx[0]), float(bpx[1])]]],
                                                   np.float32), M_persp)[0, 0]
            _BS_RECT_PX = (float(bf[0]), float(bf[1]))                    # rect pixel (annotation grid origin)
            bs_fx = float(bf[0]) / base.PX_PER_M
            bs_fy = field_h_m - float(bf[1]) / base.PX_PER_M
            base.log(f"  [fix#1 base-origin] base station @ field ({bs_fx:.3f},{bs_fy:.3f}) m -> subtracted")
        except Exception as e:
            base.log(f"  [fix#1 base-origin] skip ({e}) -> yellow-corner origin")

    # --- FIX #5: rectified-pixel se x,y (visual ke saath consistent), minus base-station offset ---
    if field_h_m is not None:
        for n, v in coords.items():
            if v and v.get("_rx") is not None and v.get("_ry") is not None:
                v["x"] = round(v["_rx"] / base.PX_PER_M - bs_fx, 3)
                v["y"] = round(field_h_m - v["_ry"] / base.PX_PER_M - bs_fy, 3)

    # --- MUTUAL EXCLUSION: overlapping targets ko alag spots (accurate positions + matcher alternates) ---
    try:
        _mutual_exclusion(coords, results, photo_toH, M_persp, field_h_m, bs_fx, bs_fy)
    except Exception as e:
        base.log(f"  [dedup] skip ({e})")

    # --- FIX #9: bache-khuche paas-paas targets pe warning ---
    items = [(n, v) for n, v in coords.items() if v and v.get("x") is not None]
    for i in range(len(items)):
        for j in range(i + 1, len(items)):
            (n1, v1), (n2, v2) = items[i], items[j]
            d = ((v1["x"] - v2["x"]) ** 2 + (v1["y"] - v2["y"]) ** 2) ** 0.5
            if d < DUP_M:
                base.log(f"  [fix#9 WARN] '{n1}' & '{n2}' abhi bhi paas ({d:.2f}m) -- verify karo")

    # --- FIX #14: assigned initial-heading frame (Final round) ---
    #   base-station origin ke around saare (x,y) ko HEADING_ROT_DEG se rotate -> axes assigned
    #   heading ke align. 0 = koi rotation (arena-edge axes, current behaviour).
    if HEADING_ROT_DEG:
        import math as _mh
        _th = _mh.radians(float(HEADING_ROT_DEG)); _ct, _st = _mh.cos(_th), _mh.sin(_th)
        for _n, _v in coords.items():
            if _v and _v.get("x") is not None and _v.get("y") is not None:
                _x, _y = _v["x"], _v["y"]
                _v["x"] = round(_x * _ct - _y * _st, 3)
                _v["y"] = round(_x * _st + _y * _ct, 3)
        base.log(f"  [fix#14] coords rotated {HEADING_ROT_DEG}° -> assigned-heading frame")

    return coords
base.compute_map_coords = compute_map_coords


# ═══════════════════════════════════════════════════════════════════════════════
# FIX #3 -- FIELD SCALE (true metric) + kaunsa side 30ft / kaunsa 25ft (auto)
#   Problem: field size drone VIO/odometry se aata (9.14x9.66) -> VIO scale error/drift ki wajah
#            se galat + aspect distorted (real arena 30x25ft = 9.144x7.62m).
#   Fix:     arena size KNOWN hai (30x25 ft). Kaunsa detected edge 30 aur kaunsa 25 -->
#            MOSAIC PIXEL edge lengths se decide (aspect image-based, VIO distortion se bacha
#            -- VIO ne to aspect ulta kar diya tha). LONGER pixel edge = 30ft. Phir yellow quad ko
#            true 30x25 rectangle pe warp -> scale + aspect + coords sab sahi.
# ═══════════════════════════════════════════════════════════════════════════════
ARENA_LONG_FT  = 35    # arena ka LAMBA edge (ft)
ARENA_SHORT_FT = 25   # arena ka CHHOTA edge (ft)

_orig_setup_field_map = base.setup_field_map
def setup_field_map(mosaic_bgr, photo_to_H, csv_rows):
    cv2, np = base.cv2, base.np
    A, corners, fw, fh, rect, M, ox, oy, oz = _orig_setup_field_map(mosaic_bgr, photo_to_H, csv_rows)
    try:
        # corners (yellow ON = yellow quad; yellow OFF = VIO/ENU de-rotated bbox) ko KNOWN arena size
        # (ARENA_LONG x ARENA_SHORT) pe re-warp -> VIO scale error correct + de-rotated + sahi metric size.
        if corners is None or len(corners) != 4:
            return A, corners, fw, fh, rect, M, ox, oy, oz
        c = np.asarray(corners, np.float64)            # TL, TR, BR, BL (src pixel quad)
        dist = lambda p, q: float(np.hypot(*(c[p] - c[q])))
        w_px = 0.5 * (dist(0, 1) + dist(3, 2))         # top + bottom  = WIDTH edges (pixels)
        h_px = 0.5 * (dist(0, 3) + dist(1, 2))         # left + right  = HEIGHT edges (pixels)
        long_m  = ARENA_LONG_FT  * base.FEET_TO_M      # 30 ft = 9.144 m
        short_m = ARENA_SHORT_FT * base.FEET_TO_M      # 25 ft = 7.620 m
        # LONGER pixel edge = LAMBA side (30ft). Aspect mosaic-pixels se (VIO se nahi).
        if w_px >= h_px:
            new_fw, new_fh, which = long_m, short_m, "width=30ft"
        else:
            new_fw, new_fh, which = short_m, long_m, "width=25ft"
        out_w = max(1, int(round(new_fw * base.PX_PER_M)))
        out_h = max(1, int(round(new_fh * base.PX_PER_M)))
        src = c.astype(np.float32)
        dst = np.float32([[0, 0], [out_w, 0], [out_w, out_h], [0, out_h]])
        M2 = cv2.getPerspectiveTransform(src, dst)
        rect2 = cv2.warpPerspective(mosaic_bgr, M2, (out_w, out_h), flags=cv2.INTER_LANCZOS4)
        cv2.imwrite(str(base.FIELD_DIR / "rectified_field.jpg"), rect2, [cv2.IMWRITE_JPEG_QUALITY, 95])
        base.log(f"  [fix#3] TRUE size {new_fw:.2f} x {new_fh:.2f} m  "
                 f"(w_px={w_px:.0f} h_px={h_px:.0f} -> {which})")
        return A, corners, new_fw, new_fh, rect2, M2, ox, oy, oz
    except Exception as e:
        base.log(f"  [fix#3] skip ({e})")
        return A, corners, fw, fh, rect, M, ox, oy, oz
base.setup_field_map = setup_field_map


# ═══════════════════════════════════════════════════════════════════════════════
# FIX #8 -- YELLOW detection lighting-ADAPTIVE
#   Problem: detect_yellow_corners fixed Lab thresholds (s>=28, b*-a*>=15) use karta hai --
#            is arena ki lighting ke liye tuned. Alag/brighter/darker lighting me yellow ka
#            saturation/b* badal jaata -> fixed threshold fail (mask khali ya noisy).
#   Fix:     PEHLE fixed threshold try karo (current lighting me bilkul waisa hi chalega). Agar
#            mask ka fraction galat aaye (khali/bahut zyada) TABHI ADAPTIVE: Otsu on (b*-a*) over
#            hue-gated pixels -> lighting-independent split. Baaki logic (morphology, hull, corners)
#            same. -> current case preserve + doosri lighting me robust.
# ═══════════════════════════════════════════════════════════════════════════════
def detect_yellow_corners(img_bgr):
    cv2, np = base.cv2, base.np
    H, W = img_bgr.shape[:2]
    hsv = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2HSV)
    lab = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2Lab)
    h, s, v = cv2.split(hsv)
    L, A, B = cv2.split(lab)
    diff = (B.astype(np.int16) - A.astype(np.int16))               # b*-a*: yellow BADA (+)
    diff_u = np.clip(diff, 0, 255).astype(np.uint8)

    # 1) FIXED mask. YELLOW_S = saturation threshold -- YEHI TUNE KARO:
    #    mask me ground aa raha (corner galat) -> BADHAO (60/70/80).  Tape gayab -> ghatao (40/28).
    YELLOW_S = 65
    fixed = (((h >= 14) & (h <= 48)) & (s >= YELLOW_S) & (diff >= 15)).astype(np.uint8) * 255
    frac = float((fixed > 0).sum()) / (H * W)
    if 0.002 <= frac <= 0.35:
        mask = fixed
        base.log(f"  [fix#8] fixed yellow OK (frac={frac:.3f})")
    else:
        # 2) ADAPTIVE: Otsu on hue-gated (b*-a*) -> lighting-independent
        hue_gate = (h >= 12) & (h <= 52)
        vals = diff_u[hue_gate & (s >= 18)]
        thr = 15
        if vals.size >= 200:
            hist = np.bincount(vals, minlength=256).astype(np.float64)
            pn = hist / hist.sum(); w0 = np.cumsum(pn); mu = np.cumsum(pn * np.arange(256))
            mut = mu[-1]; den = w0 * (1 - w0); den[den == 0] = 1e-9
            thr = max(int(np.argmax((mut * w0 - mu) ** 2 / den)), 10)
        mask = (hue_gate & (s >= 18) & (diff_u >= thr)).astype(np.uint8) * 255
        base.log(f"  [fix#8] fixed frac={frac:.3f} off -> ADAPTIVE (b*-a* thr={thr})")

    ko = max(3, int(min(H, W) * 0.003))
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (ko, ko)))
    k = max(15, int(min(H, W) * 0.03))
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (k, k)))
    try:
        cv2.imwrite(str(base.FIELD_DIR / "yellow_mask_debug.jpg"), mask)
    except Exception:
        pass
    n, lbl, stats, _ = cv2.connectedComponentsWithStats(mask)
    if n <= 1:
        return None
    areas = stats[1:, cv2.CC_STAT_AREA].astype(np.float64)
    amax = float(areas.max()) if len(areas) else 0.0
    # yellow TAPE/border THIN hota hai (bbox me fill kam). SOLID blobs (yellowish features) fill zyada
    # -> unhe REJECT. Tape fragments (thin) keep.
    keep = []
    for i in range(1, n):
        a = stats[i, cv2.CC_STAT_AREA]
        if a < max(40.0, 0.02 * amax):
            continue
        wcc = int(stats[i, cv2.CC_STAT_WIDTH]); hcc = int(stats[i, cv2.CC_STAT_HEIGHT])
        fill = float(a) / float(max(wcc * hcc, 1))
        if fill >= 0.60:
            continue
        keep.append(i)
    if not keep:
        return None
    ys, xs = np.where(np.isin(lbl, keep))
    if len(xs) < 4:
        return None
    pts = np.column_stack([xs, ys]).astype(np.int32)
    hull = cv2.convexHull(pts)
    peri_h = cv2.arcLength(hull, True)
    quad = None
    for ef in (0.02, 0.03, 0.04, 0.05, 0.06, 0.08, 0.10):
        ap = cv2.approxPolyDP(hull, ef * peri_h, True)
        if len(ap) == 4:
            quad = ap.reshape(4, 2).astype(np.float32); break
    if quad is None:
        quad = cv2.boxPoints(cv2.minAreaRect(pts.astype(np.float32))).astype(np.float32)
    box = base.order_corners(quad)
    peri = sum(float(np.linalg.norm(box[i] - box[(i + 1) % 4])) for i in range(4))
    tape_w_px = float(len(xs)) / max(peri, 1.0)
    return box, tape_w_px
base.detect_yellow_corners = detect_yellow_corners


# ═══════════════════════════════════════════════════════════════════════════════
# FIX #1 (cont.) -- ANNOTATION grid origin = base station (jab BASE_STATION_EXACT)
#   compute_map_coords _BS_RECT_PX (base station ka rect pixel) set karta hai. base.main() Stage-4
#   ko base_px=None deta hai (grid origin = yellow corner). Yahan None ko base station rect pixel se
#   replace -> grid + circle + label sab base station se reference (coords ke consistent).
#   NOTE: annotation ka legend text abhi bhi "yellow corner" likhta hai (base fn ke andar hardcoded,
#   cosmetic) -- circles/grid/labels sahi base-station frame me hain.
# ═══════════════════════════════════════════════════════════════════════════════
_orig_stage4_annotate = base.run_stage4_annotate
def run_stage4_annotate(rect_bgr, field_w_m, field_h_m, results, map_coords,
                        origin_x, origin_y, origin_z, base_px=None):
    if BASE_STATION_EXACT and base_px is None and _BS_RECT_PX is not None:
        base_px = _BS_RECT_PX                 # grid/label origin = actual base station
    return _orig_stage4_annotate(rect_bgr, field_w_m, field_h_m, results, map_coords,
                                 origin_x, origin_y, origin_z, base_px)
base.run_stage4_annotate = run_stage4_annotate


# ═══════════════════════════════════════════════════════════════════════════════
# FIX #12 -- STAGE 1 stitch: photos DROP + drift/skew (mosaic "not good")
#   Problem: discover() filename se "r##c##" regex (_RC_RE) se (row,col) nikaalta hai. Lekin map/
#            photos ke naam "cp0012_c03s02.jpg" (c##s## format) us regex se MATCH nahi karte ->
#            har photo ko (fb, 0) = column-0 sequential mil jaata -> poori 2D grid ek 1D CHAIN.
#            Nateeja: (a) sirf consecutive pairs -> error compound -> DRIFT/SKEW (parallelogram),
#            (b) ek pair fail hote hi chain toot ke aage ke photos DROP (log: "Stitched 15/23").
#   Fix:     coordinates.csv me authoritative integer row,col (+ x_enu,y_enu) hai. Usse har photo ko
#            SAHI (row,col) do -> select_pairs ko asli 8-neighbour 2D grid milta -> cross-row loop
#            closures -> bundle adjustment DRIFT kaat deta + broken pair se graph disconnect nahi hota
#            (alternate paths) -> zyada photos aligned + clean mosaic. CSV na mile/photo missing ->
#            purana regex/fallback (kuch bhe break nahi).
# ═══════════════════════════════════════════════════════════════════════════════
import csv as _csv_mod
from pathlib import Path as _Path

def _stitch_rowcol_map(folder):
    """coordinates.csv se {image_file basename: (row, col)} -- authoritative survey grid."""
    cands = [_Path(folder) / "coordinates.csv"]
    for attr in ("DRONE_HD_DIR", "DRONE_DIR", "MAP_DIR", "BASE_DIR"):
        d = getattr(base, attr, None)
        if d is not None:
            cands.append(_Path(d) / "coordinates.csv")
    pos = {}
    for csvp in cands:
        try:
            if csvp.exists():
                with open(csvp, newline="", encoding="utf-8-sig") as f:
                    for row in _csv_mod.DictReader(f):
                        name = (row.get("image_file") or "").strip()
                        r, c = row.get("row"), row.get("col")
                        if name and r not in (None, "") and c not in (None, ""):
                            pos.setdefault(name, (int(float(r)), int(float(c))))
                if pos:
                    break
        except Exception:
            pass
    return pos

_orig_discover = base.discover
def discover(folder):
    entries = _orig_discover(folder)                 # (row, col, path) -- row/col filename se (buggy)
    try:
        pos = _stitch_rowcol_map(folder)
        if not pos:
            return entries                           # CSV nahi -> original behaviour
        fixed, hit = [], 0
        for (r, c, p) in entries:
            rc = pos.get(p.name)
            if rc is not None:
                fixed.append((rc[0], rc[1], p)); hit += 1
            else:
                fixed.append((r, c, p))
        base.log(f"  [fix#12] stitch grid: {hit}/{len(entries)} photos ko coordinates.csv se "
                 f"(row,col) mila -> asli 2D neighbours (drift/drop fix)")
        return sorted(fixed)
    except Exception as e:
        base.log(f"  [fix#12] skip ({e}) -> filename grid")
        return entries
base.discover = discover


# ═══════════════════════════════════════════════════════════════════════════════
# FIX #13 -- STAGE 1 stitch: CENTER-start (ya spiral) pe aadha mosaic
#   Problem: select_pairs integer (row,col) GRID neighbours (r±1,c±1) dhoondhta hai. Ye tabhi
#            chalta jab survey ek clean rectangular lawnmower ho. Drone CENTER se start kare / spiral
#            kare to row,col ek dense rectangular grid nahi banti -> kai photos ko koi grid-neighbour
#            nahi milta -> graph DISCONNECT -> sirf ek hissa stitch hota (aadha/corner mosaic).
#   Fix:     photos ko integer grid ki jagah ACTUAL VIO position (x_enu,y_enu) ke K-NEAREST
#            neighbours (+ temporal consecutive) se pair karo. Ye pattern-AGNOSTIC hai -> center,
#            corner, spiral, lawnmower -- sab pe nearest photos (max overlap) connect hote -> poora
#            mosaic. VIO na mile to purana grid pairing (fix#12 + fallback).
# ═══════════════════════════════════════════════════════════════════════════════
def _stitch_xy_map(folder=None):
    """coordinates.csv se {image_file basename: (x_enu, y_enu)} -- true VIO positions."""
    cands = []
    if folder is not None:
        cands.append(_Path(folder) / "coordinates.csv")
    for attr in ("DRONE_HD_DIR", "DRONE_DIR", "MAP_DIR", "BASE_DIR"):
        d = getattr(base, attr, None)
        if d is not None:
            cands.append(_Path(d) / "coordinates.csv")
    pos = {}
    for csvp in cands:
        try:
            if csvp.exists():
                with open(csvp, newline="", encoding="utf-8-sig") as f:
                    for row in _csv_mod.DictReader(f):
                        name = (row.get("image_file") or "").strip()
                        x, y = row.get("x_enu"), row.get("y_enu")
                        if name and x not in (None, "") and y not in (None, ""):
                            pos.setdefault(name, (float(x), float(y)))
                if pos:
                    break
        except Exception:
            pass
    return pos

_orig_select_pairs = base.select_pairs
def select_pairs(entries, radius=None):
    if radius is None:
        radius = base.GRID_RADIUS
    try:
        np = base.np
        pos = _stitch_xy_map()
        names = [p.name for (_, _, p) in entries]
        idxs = [i for i in range(len(entries)) if names[i] in pos]
        if len(idxs) < 3:
            return _orig_select_pairs(entries, radius)          # VIO nahi -> grid pairing
        P = np.array([pos[names[i]] for i in idxs], float)
        K = min((8 if radius <= 1 else 12), len(idxs) - 1)      # radius1 -> 8-NN, radius2 -> 12-NN
        pairs = set()
        for a in range(len(idxs)):
            d = np.sum((P - P[a]) ** 2, axis=1)
            for b in np.argsort(d)[1:K + 1]:                    # nearest K (self skip)
                i, j = idxs[a], idxs[int(b)]
                pairs.add((i, j) if i < j else (j, i))
        for i in range(len(entries) - 1):                       # temporal overlap safety net
            pairs.add((i, i + 1))
        base.log(f"  [fix#13] spatial pairing: {len(idxs)}/{len(entries)} photos, {len(pairs)} pairs "
                 f"(VIO {K}-NN + temporal) -- center/corner dono robust")
        return sorted(pairs)
    except Exception as e:
        base.log(f"  [fix#13] skip ({e}) -> grid pairing")
        return _orig_select_pairs(entries, radius)
base.select_pairs = select_pairs


if __name__ == "__main__":
    base.main()
    # MATCH_LR mode: is run ka result ALAG folder (results_lr<N>/) me copy -> default (128) aur
    # 64x64 dono results ek saath rakho / compare karo (results/ overwrite ho jaata, ye copy bachi rehti).
    import os as _os, shutil as _shutil
    _mlr = _os.environ.get("MATCH_LR")
    if _mlr and str(_mlr).isdigit():
        try:
            root = base.TARGET_DIR.parent                 # .../results
            dst = root.parent / f"results_lr{_mlr}"        # .../results_lr64
            dst.mkdir(parents=True, exist_ok=True)
            for sub in (base.TARGET_DIR, base.ANNOT_DIR):  # stage3_targets + stage4_annotated
                if sub.exists():
                    d = dst / sub.name
                    if d.exists():
                        _shutil.rmtree(d)
                    _shutil.copytree(sub, d)
            base.log(f"  [MATCH_LR={_mlr}] result ALAG folder me copy -> {dst}")
            print(f"\n  >> {_mlr}x{_mlr} result: {dst}\n  >> default result: {root}")
        except Exception as e:
            base.log(f"  [MATCH_LR copy] skip ({e})")
