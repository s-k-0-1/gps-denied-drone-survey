#!/usr/bin/env python3
"""
3d.py - Turn a folder of photos into a 3D map, then show a full-color
        height map (orthophoto color + DSM elevation).

Wraps OpenDroneMap (via Docker) for 3D reconstruction, then automatically:
  1. Builds a colored point cloud (real RGB + elevation Z)
  2. Saves a side-by-side 2D preview (orthophoto | DSM TURBO colormap)
  3. Opens an interactive Open3D viewer

USAGE
    python3 3d.py                    # drone_photos/ se 3D banao (ODM + colored point cloud)
    python3 3d.py /path/to/photos    # custom folder
    python3 3d.py --skip-odm         # ODM skip, sirf colored heightmap regenerate
    python3 3d.py --view             # banane ke baad 3D viewer bhi kholo (GUI/WSLg chahiye)

OUTPUT -> results/3d_map/   (SAB ek jagah, results/ ke andar)
    model.glb              TEXTURED 3D model -- online viewer (trellis3d.co) + Windows '3D Viewer'
                           + MeshLab/Blender sab me khulta (texture embedded). <-- MAIN 3D map
    orthophoto.tif         flat orthophoto (real color)
    dsm.tif                elevation map (DSM)
    point_cloud.laz        georeferenced point cloud
    color_heightmap.ply    colored point cloud (color + elevation; MeshLab/CloudCompare)
    color_heightmap.jpg    2D preview (orthophoto | DSM TURBO)

NOTE: model.glb (textured) sahi ban-na ke liye ODM ko theek se chalne do (--feature-quality high,
      --min-num-features 16000). GLB ke liye trimesh chahiye: pip install trimesh --break-system-packages
"""

import os
import sys
import shutil
import subprocess
from pathlib import Path

import cv2
import numpy as np

# ── Settings ──────────────────────────────────────────────────────────────────
USE_GPU       = True
FRESH_RUN     = True
DEFAULT_PHOTOS = "drone_photos"
EXTRA_OPTIONS = [
    "--feature-type",          "sift",
    "--feature-quality",       "high",      # behtar/zyada features (pehle default -> reconstruction fail)
    "--min-num-features",      "16000",     # har image me zyada SIFT features -> behtar SfM matching
    "--matcher-neighbors",     "0",         # 0 = SAARE pairs match (35 photos -> full connectivity)
    "--pc-quality",            "high",      # zyada points (pehle 'low' -> sirf 574 points the)
    "--dsm",
    "--orthophoto-resolution", "2",         # finer (2 cm/px) -> bada, detailed orthophoto
    "--skip-report",
    "--skip-3dmodel",                       # texturing skip (PoissonRecon bug se bachne ko; point cloud kaafi)
    "--max-concurrency",       "4",         # threads limit -> kam peak RAM
]
IMAGE_EXTS = (".jpg", ".jpeg", ".png", ".tif", ".tiff", ".bmp", ".dng", ".nef")

# Color heightmap: 1=full res, 2=half, 4=quarter (use 2 for speed)
HEIGHTMAP_DOWNSAMPLE = 2
# ──────────────────────────────────────────────────────────────────────────────

HERE      = Path(os.path.dirname(os.path.abspath(__file__)))
DATASETS  = HERE / "datasets"
PROJECT   = DATASETS / "project"
IMAGES    = PROJECT / "images"
HM_DIR    = HERE / "results" / "3d_map"            # SAB 3D outputs yahan (results/ ke andar)


# ═══════════════════════════════════════════════════════════════════════════════
# ODM helpers
# ═══════════════════════════════════════════════════════════════════════════════

def fail(msg):
    print("\n[ERROR] " + msg)
    sys.exit(1)


def check_docker():
    if shutil.which("docker") is None:
        fail("Docker is not installed or not in PATH.")
    try:
        subprocess.run(["docker", "info"], check=True,
                       stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    except Exception:
        fail("Docker is installed but not running.\n"
             "Start it with:  sudo service docker start")


def gpu_available():
    try:
        r = subprocess.run(
            ["docker", "run", "--rm", "--gpus", "all",
             "nvidia/cuda:12.3.1-base-ubuntu22.04", "nvidia-smi"],
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, timeout=180)
        return r.returncode == 0
    except Exception:
        return False


def clear_old_project():
    if PROJECT.exists():
        print("[INFO] Clearing previous ODM project ...")
        try:
            shutil.rmtree(str(PROJECT))
        except PermissionError:
            fail("Old files are owned by root.\n"
                 "Run:  sudo rm -rf %s" % DATASETS)


def stage_images(src_folder: str) -> int:
    if not os.path.isdir(src_folder):
        fail("Folder does not exist: " + src_folder)
    photos = [f for f in os.listdir(src_folder)
              if f.lower().endswith(IMAGE_EXTS)]
    if len(photos) < 5:
        fail("Only %d image(s) found. Need ≥5 overlapping photos." % len(photos))
    IMAGES.mkdir(parents=True, exist_ok=True)
    print("[INFO] Copying %d photos into ODM project ..." % len(photos))
    for f in photos:
        shutil.copy2(os.path.join(src_folder, f), str(IMAGES / f))
    return len(photos)


def run_odm(use_gpu: bool):
    odm_image  = "opendronemap/odm:gpu" if use_gpu else "opendronemap/odm"
    host_mount = str(DATASETS).replace("\\", "/")

    cmd = ["docker", "run", "-ti", "--rm"]
    try:
        cmd += ["--user", f"{os.getuid()}:{os.getgid()}"]
    except AttributeError:
        pass   # Windows: no getuid
    cmd += ["-v", host_mount + ":/datasets"]
    if use_gpu:
        cmd += ["--gpus", "all"]
    cmd += [odm_image, "--project-path", "/datasets", "project"]
    cmd += EXTRA_OPTIONS

    print("\n[INFO] Running ODM (%s) ..." % ("GPU" if use_gpu else "CPU"))
    print("       " + " ".join(cmd) + "\n")
    subprocess.run(cmd, check=True)


# ═══════════════════════════════════════════════════════════════════════════════
# Color heightmap
# ═══════════════════════════════════════════════════════════════════════════════

def _load_tif_color(path: Path) -> np.ndarray:
    img = cv2.imread(str(path))
    if img is None:
        try:
            import rasterio
            with rasterio.open(str(path)) as src:
                r = src.read(1); g = src.read(2); b = src.read(3)
                img = cv2.merge([b, g, r]).astype(np.uint8)
        except Exception:
            raise IOError(f"Cannot read {path}")
    return img


def _load_tif_float(path: Path) -> np.ndarray:
    arr = cv2.imread(str(path), cv2.IMREAD_ANYDEPTH | cv2.IMREAD_GRAYSCALE)
    if arr is None:
        try:
            import rasterio
            with rasterio.open(str(path)) as src:
                arr = src.read(1).astype(np.float32)
        except Exception:
            raise IOError(f"Cannot read {path}")
    return arr.astype(np.float32)


def build_colored_cloud(ortho_bgr: np.ndarray, dsm: np.ndarray,
                        downsample: int = 2):
    """
    Fuse orthophoto (color) + DSM (elevation) into a colored point cloud.
    Returns (xyz [N,3], rgb [N,3] float32 0-1).
    """
    h_o, w_o = ortho_bgr.shape[:2]

    # Resize DSM to match orthophoto
    if dsm.shape != (h_o, w_o):
        dsm = cv2.resize(dsm, (w_o, h_o), interpolation=cv2.INTER_LINEAR)

    # Downsample for performance
    if downsample > 1:
        nw = max(1, w_o // downsample)
        nh = max(1, h_o // downsample)
        ortho_bgr = cv2.resize(ortho_bgr, (nw, nh), interpolation=cv2.INTER_AREA)
        dsm       = cv2.resize(dsm,       (nw, nh), interpolation=cv2.INTER_LINEAR)
        h_o, w_o  = nh, nw

    # Mask no-data pixels (very low values = missing elevation)
    valid = dsm > (dsm.min() + 1.0)

    xs = np.tile(np.arange(w_o, dtype=np.float32), (h_o, 1))
    ys = np.tile(np.arange(h_o, dtype=np.float32).reshape(-1, 1), (1, w_o))

    xyz = np.stack([xs[valid], ys[valid], dsm[valid]], axis=1)

    ortho_rgb = cv2.cvtColor(ortho_bgr, cv2.COLOR_BGR2RGB).astype(np.float32) / 255.0
    rgb = ortho_rgb.reshape(-1, 3)[valid.ravel()]

    return xyz, rgb


def save_ply(path: Path, xyz: np.ndarray, rgb: np.ndarray):
    """Save binary PLY colored point cloud."""
    n = len(xyz)
    rgb_u8 = (rgb * 255).clip(0, 255).astype(np.uint8)
    header = (
        "ply\nformat binary_little_endian 1.0\n"
        f"element vertex {n}\n"
        "property float x\nproperty float y\nproperty float z\n"
        "property uchar red\nproperty uchar green\nproperty uchar blue\n"
        "end_header\n"
    )
    data = np.zeros(n, dtype=[('x','f4'),('y','f4'),('z','f4'),
                               ('r','u1'),('g','u1'),('b','u1')])
    data['x'] = xyz[:,0]; data['y'] = xyz[:,1]; data['z'] = xyz[:,2]
    data['r'] = rgb_u8[:,0]; data['g'] = rgb_u8[:,1]; data['b'] = rgb_u8[:,2]
    with open(str(path), "wb") as f:
        f.write(header.encode())
        f.write(data.tobytes())
    print(f"  [SAVED] {path.name}  ({n:,} points)")


def make_2d_preview(ortho_bgr: np.ndarray, dsm: np.ndarray, out_path: Path):
    """Side-by-side: orthophoto (left) | DSM TURBO colormap (right)."""
    h, w = ortho_bgr.shape[:2]
    dsm_r = cv2.resize(dsm, (w, h), interpolation=cv2.INTER_LINEAR)

    valid = dsm_r > (dsm_r.min() + 1.0)
    lo = float(dsm_r[valid].min()) if valid.any() else float(dsm_r.min())
    hi = float(dsm_r.max())

    norm      = np.clip((dsm_r - lo) / max(hi - lo, 1e-6), 0, 1)
    dsm_color = cv2.applyColorMap((norm * 255).astype(np.uint8), cv2.COLORMAP_TURBO)
    dsm_color[~valid] = 0

    # Elevation scale bar
    bar_w   = max(24, int(w * 0.02))
    bar     = np.linspace(255, 0, h, dtype=np.uint8)
    bar_col = cv2.applyColorMap(
        np.tile(bar.reshape(-1, 1), (1, bar_w)), cv2.COLORMAP_TURBO)
    cv2.putText(bar_col, f"{hi:.1f}m", (2, 14),
                cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255,255,255), 1, cv2.LINE_AA)
    cv2.putText(bar_col, f"{lo:.1f}m", (2, h - 4),
                cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255,255,255), 1, cv2.LINE_AA)

    preview = np.hstack([ortho_bgr, dsm_color, bar_col])

    # Labels
    cv2.putText(preview, "Orthophoto (real color)", (8, 24),
                cv2.FONT_HERSHEY_SIMPLEX, 0.65, (255,255,255), 2, cv2.LINE_AA)
    cv2.putText(preview, "DSM elevation (TURBO)", (w + 8, 24),
                cv2.FONT_HERSHEY_SIMPLEX, 0.65, (255,255,255), 2, cv2.LINE_AA)

    # Cap at 4K
    ph, pw = preview.shape[:2]
    if pw > 3840:
        s = 3840 / pw
        preview = cv2.resize(preview, (int(pw*s), int(ph*s)), interpolation=cv2.INTER_AREA)

    cv2.imwrite(str(out_path), preview, [cv2.IMWRITE_JPEG_QUALITY, 93])
    print(f"  [SAVED] {out_path.name}")


def open_viewer(ply_path: Path):
    """Open3D viewer try karo. WSL pe OpenGL aksar fail hota hai -> Windows me kholne ki guidance."""
    try:
        import open3d as o3d
        print("\n  Opening Open3D viewer  (Q or Esc to close) ...")
        pcd = o3d.io.read_point_cloud(str(ply_path))
        print(f"  {len(pcd.points):,} points loaded")
        pcd.estimate_normals(
            search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=2.0, max_nn=30))
        o3d.visualization.draw_geometries(
            [pcd], window_name="3D Color Height Map", width=1280, height=720)
    except ImportError:
        print("  [WARN] open3d not installed:  pip install open3d --break-system-packages")
    except Exception as e:
        print(f"  [WARN] viewer error: {e}")
    # A 3D GUI often fails under WSL (OpenGL/Wayland) -> open the file from Windows instead.
    home = os.path.expanduser("~")
    distro = os.environ.get("WSL_DISTRO_NAME", "Ubuntu")
    win = str(ply_path).replace(home, rf"\\wsl.localhost\{distro}{home}").replace("/", "\\")
    print("\n  If the 3D window does not open under WSL, open the PLY from Windows "
          "(MeshLab / CloudCompare, both free):")
    print(f"    {win}")


def copy_odm_outputs():
    """ODM ke asli outputs (orthophoto, DSM, point cloud) ko results/3d_map/ me copy."""
    pairs = [
        (PROJECT / "odm_orthophoto"   / "odm_orthophoto.tif",            "orthophoto.tif"),
        (PROJECT / "odm_dem"          / "dsm.tif",                       "dsm.tif"),
        (PROJECT / "odm_dem"          / "dtm.tif",                       "dtm.tif"),
        (PROJECT / "odm_georeferencing" / "odm_georeferenced_model.laz", "point_cloud.laz"),
    ]
    n = 0
    for src, name in pairs:
        if src.exists():
            try:
                shutil.copy2(str(src), str(HM_DIR / name)); n += 1
            except Exception as e:
                print(f"  [WARN] copy {name}: {e}")
    if n:
        print(f"  [SAVED] {n} ODM output(s) -> results/3d_map/")


def export_glb():
    """ODM ka TEXTURED 3D model (.obj + texture) -> model.glb (universal format: online viewer
    + Windows '3D Viewer' + MeshLab/Blender sab me khulta, texture embedded)."""
    obj = None
    for sub in ("odm_texturing_25d", "odm_texturing"):                 # 2.5D pehle (--skip-3dmodel ke saath yahi banta)
        p = PROJECT / sub / "odm_textured_model_geo.obj"
        if p.exists():
            obj = p; break
    if obj is None:
        print("  [WARN] textured model (.obj) nahi mila -> GLB skip"); return
    glb = HM_DIR / "model.glb"
    try:
        import trimesh
        scene = trimesh.load(str(obj))                                 # OBJ + MTL + texture (same folder se)
        scene.export(str(glb))
        print(f"  [SAVED] model.glb -> results/3d_map/  (online viewer + PC me kholo)")
    except ImportError:
        print("  [WARN] trimesh nahi hai -> GLB ke liye:  pip install trimesh --break-system-packages")
        # fallback: textured OBJ + MTL + texture copy karo (GLB khud bana sakte/online convert)
        shutil.copy2(str(obj), str(HM_DIR / "model.obj"))
        for f in list(obj.parent.glob("*.mtl")) + list(obj.parent.glob("*.png")):
            shutil.copy2(str(f), str(HM_DIR / f.name))
        print(f"  [SAVED] model.obj + texture -> results/3d_map/")
    except Exception as e:
        print(f"  [WARN] GLB export fail: {e}")


def run_color_heightmap():
    """Build and show the full-color 3D height map from ODM outputs."""
    ortho_path = PROJECT / "odm_orthophoto" / "odm_orthophoto.tif"
    dsm_path   = PROJECT / "odm_dem"        / "dsm.tif"

    if not ortho_path.exists():
        print(f"[WARN] Orthophoto not found: {ortho_path}")
        return
    if not dsm_path.exists():
        print(f"[WARN] DSM not found: {dsm_path}")
        print("  Make sure --dsm is in EXTRA_OPTIONS (it is by default).")
        return

    HM_DIR.mkdir(parents=True, exist_ok=True)
    copy_odm_outputs()                                 # ODM outputs (ortho/dsm/point cloud) -> results/3d_map/
    export_glb()                                       # textured 3D model -> results/3d_map/model.glb

    print("\n" + "="*60)
    print("  COLOR HEIGHT MAP")
    print("="*60)
    print(f"  Orthophoto : {ortho_path}")
    print(f"  DSM        : {dsm_path}")
    print(f"  Downsample : {HEIGHTMAP_DOWNSAMPLE}x")

    print("\n  Loading orthophoto ...")
    ortho_bgr = _load_tif_color(ortho_path)
    print(f"  {ortho_bgr.shape[1]}×{ortho_bgr.shape[0]} px")

    print("  Loading DSM ...")
    dsm = _load_tif_float(dsm_path)
    valid = dsm > (dsm.min() + 1.0)
    lo = float(dsm[valid].min()) if valid.any() else float(dsm.min())
    hi = float(dsm.max())
    print(f"  {dsm.shape[1]}×{dsm.shape[0]} px   elevation: {lo:.2f} – {hi:.2f} m")

    print("\n  Generating 2D preview ...")
    make_2d_preview(ortho_bgr, dsm, HM_DIR / "color_heightmap.jpg")

    print(f"  Building colored point cloud (downsample={HEIGHTMAP_DOWNSAMPLE}) ...")
    xyz, rgb = build_colored_cloud(ortho_bgr, dsm, HEIGHTMAP_DOWNSAMPLE)
    print(f"  {len(xyz):,} points")

    ply_path = HM_DIR / "color_heightmap.ply"
    save_ply(ply_path, xyz, rgb)

    print(f"\n  Results saved to: {HM_DIR}/")
    if "--view" in sys.argv:
        open_viewer(ply_path)                          # GUI sirf --view ke saath (warna headless pe hang)
    else:
        print("  3D dekhne ke liye:  python3 3d.py --skip-odm --view")


# ═══════════════════════════════════════════════════════════════════════════════
# Main
# ═══════════════════════════════════════════════════════════════════════════════

def main():
    skip_odm = "--skip-odm" in sys.argv
    src = next((a for a in sys.argv[1:] if not a.startswith("--")), DEFAULT_PHOTOS)

    if not skip_odm:
        check_docker()
        if FRESH_RUN:
            clear_old_project()
        n = stage_images(src)

        use_gpu = USE_GPU
        if use_gpu:
            print("[INFO] Checking Docker GPU access ...")
            if gpu_available():
                print("[INFO] GPU detected — using GPU-accelerated ODM.")
            else:
                print("[WARN] GPU not visible to Docker. Falling back to CPU.")
                use_gpu = False

        run_odm(use_gpu)

        print(f"\n[DONE] Processed {n} photos.  ODM raw outputs -> {PROJECT}/")
        print(f"  (orthophoto, DSM, point cloud results/3d_map/ me copy ho jayenge)")
    else:
        print("[INFO] Skipping ODM — generating color heightmap from existing outputs.")

    # Always run color heightmap after ODM
    run_color_heightmap()


if __name__ == "__main__":
    main()
