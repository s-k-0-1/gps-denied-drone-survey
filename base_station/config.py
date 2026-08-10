"""
config.py — Central configuration & path resolution for the ASCEND
Base Station desktop dashboard.

All paths are derived from BASE_DIR, which defaults to the parent of this
package (i.e. ~/advanced_matcher) so the app finds your existing pipeline
without any hard-coded username. Override anything with environment variables.
"""
from __future__ import annotations

import os
from pathlib import Path

# Team / branding
TEAM_NAME = os.environ.get("IROC_TEAM", "LUMA")
DRONE_NAME = os.environ.get("IROC_DRONE", "ASCEND")

# ──────────────────────────────────────────────────────────────────────────
# Core directories (match iroc_pipeline.py layout)
# ──────────────────────────────────────────────────────────────────────────
# base_station/ lives inside the pipeline repo, so BASE_DIR = its parent.
_PKG_DIR = Path(__file__).resolve().parent
BASE_DIR = Path(os.environ.get("IROC_BASE_DIR", _PKG_DIR.parent)).resolve()

DRONE_PHOTOS_DIR = BASE_DIR / "drone_photos"          # HD survey photos + coordinates.csv
DRONE_LR_DIR     = BASE_DIR / "drone_photos_lr"
SURVEY_DIR       = BASE_DIR / "survey"                # timestamped sim/real survey runs
TARGETS_DIR      = BASE_DIR / "targets"               # reference features
RESULTS_DIR      = BASE_DIR / "results"

# results/ sub-folders produced by the pipeline
STITCH_DIR  = RESULTS_DIR / "stage1_stitch"
FIELD_DIR   = RESULTS_DIR / "stage2_field"
TARGET_DIR  = RESULTS_DIR / "stage3_targets"
ANNOT_DIR   = RESULTS_DIR / "stage4_annotated"
STAGE3D_DIR = RESULTS_DIR / "stage0_3d"

# Key files
ANNOTATED_MAP   = ANNOT_DIR / "annotated_field.jpg"
ORTHOMOSAIC     = STITCH_DIR / "orthomosaic.jpg"
RECTIFIED_FIELD = FIELD_DIR / "rectified_field.jpg"
TARGETS_JSON    = TARGET_DIR / "targets.json"
FUSED_CSV       = TARGET_DIR / "fused_results.csv"
PROOF_HD_DIR    = TARGET_DIR / "proof_hd"
VISUALS_DIR     = TARGET_DIR / "visuals"

COORDINATES_CSV = DRONE_PHOTOS_DIR / "coordinates.csv"

# Pipeline scripts
IROC_PIPELINE       = BASE_DIR / "iroc_pipeline.py"          # original
IROC_PIPELINE_FIXED = BASE_DIR / "iroc_pipeline_fixed.py"    # ALL fixes: base-station origin, LR,
                                                            # HD-720 sharp, stage3_robust matcher, #2-#11
FUSED_SEARCH  = BASE_DIR / "fused_search.py"                 # old matcher
STAGE3_ROBUST = BASE_DIR / "stage3_robust.py"               # robust DINOv2 semantic matcher
SCRIPT_3D     = BASE_DIR / "3d.py"

# Prefer the FIXED pipeline / ROBUST matcher when present (graceful fallback to originals).
# -> START MISSION aur pipeline buttons ab automatically saare fixes ke saath chalte hain.
PIPELINE_MAIN = IROC_PIPELINE_FIXED if IROC_PIPELINE_FIXED.exists() else IROC_PIPELINE
MATCHER_MAIN  = STAGE3_ROBUST if STAGE3_ROBUST.exists() else FUSED_SEARCH

# Image extensions we treat as photos
IMAGE_EXTS = {".jpg", ".jpeg", ".png", ".tif", ".tiff", ".bmp",
              ".JPG", ".JPEG", ".PNG", ".TIF", ".TIFF", ".BMP"}

# ──────────────────────────────────────────────────────────────────────────
# Server
# ──────────────────────────────────────────────────────────────────────────
HOST = os.environ.get("BASE_STATION_HOST", "0.0.0.0")
PORT = int(os.environ.get("BASE_STATION_PORT", "8000"))
PYTHON = os.environ.get("IROC_PYTHON", "python3")   # interpreter used for pipeline subprocess

# Password gate (HTTP Basic) — protects the dashboard when exposed over a tunnel.
# Change these with env vars: IROC_USER, IROC_PASS.  Disable with IROC_AUTH=0.
AUTH_USER = os.environ.get("IROC_USER", "luma")
AUTH_PASS = os.environ.get("IROC_PASS", "ascend2026")
AUTH_ENABLED = os.environ.get("IROC_AUTH", "1") not in ("0", "false", "False", "no")

# ──────────────────────────────────────────────────────────────────────────
# Docking / ESP32 integration
# ──────────────────────────────────────────────────────────────────────────
# Shared token used by machine-to-machine calls (Jetson → /api/landed,
# ESP32 → /api/dock_log & /api/dock_register). These bypass the dashboard
# password but must present this token. Keep it matching DOCK_TOKEN in the ESP.
MACHINE_TOKEN = os.environ.get("IROC_TOKEN", "lumadock")

# Seconds to wait AFTER the drone lands before commanding the ESP to dock.
DOCK_DELAY_S = float(os.environ.get("IROC_DOCK_DELAY", "5"))

# Optional fallback ESP address (host or host:port). Normally the ESP
# self-registers its IP on boot, so this can be left blank.
ESP_HOST = os.environ.get("IROC_ESP_HOST", "")

# ──────────────────────────────────────────────────────────────────────────
# Drone link
# ──────────────────────────────────────────────────────────────────────────
#   "mavlink"   → use pymavlink, fall back to simulator if no connection
#   "simulator" → always simulator (no hardware needed)
#   "auto"      → same as "mavlink" (try real, fall back to sim)
# Default is "simulator" so the dashboard + START MISSION work instantly with
# no hardware. Switch to MAVLink live in the UI, or set DRONE_LINK_MODE=mavlink.
LINK_MODE = os.environ.get("DRONE_LINK_MODE", "simulator")
MAVLINK_CONN = os.environ.get("MAVLINK_CONN", "udpin:0.0.0.0:14550")
MAVLINK_HEARTBEAT_TIMEOUT = float(os.environ.get("MAVLINK_HEARTBEAT_TIMEOUT", "6"))

# Mission defaults
CRUISE_ALT_M       = float(os.environ.get("CRUISE_ALT_M", "3.0"))
SIM_GROUND_SPEED   = float(os.environ.get("SIM_GROUND_SPEED", "1.6"))   # m/s during survey
SIM_BATTERY_DRAIN  = float(os.environ.get("SIM_BATTERY_DRAIN", "0.45")) # %/s while flying
SIM_BATTERY_CHARGE = float(os.environ.get("SIM_BATTERY_CHARGE", "4.0")) # %/s while charging
TELEMETRY_HZ       = float(os.environ.get("TELEMETRY_HZ", "5"))

# Whether START MISSION also launches iroc_pipeline.py (toggle in UI).
RUN_PIPELINE_ON_MISSION = os.environ.get("RUN_PIPELINE_ON_MISSION", "1") not in ("0", "false", "False")


# ──────────────────────────────────────────────────────────────────────────
# Helpers
# ──────────────────────────────────────────────────────────────────────────
def _csv_data_rows(csv_path: Path) -> int:
    """Number of non-header lines in a coordinates.csv (0 if header-only/missing)."""
    try:
        with csv_path.open("r", encoding="utf-8-sig") as f:
            return max(0, sum(1 for _ in f) - 1)
    except Exception:
        return 0


def _has_photo(d: Path) -> bool:
    try:
        return any(p.is_file() and p.suffix in IMAGE_EXTS for p in d.iterdir())
    except Exception:
        return False


def newest_survey_dir() -> Path | None:
    """
    Most recent survey/<run>/ folder usable for a flight replay.

    Prefers runs that actually have waypoints AND photos (so the simulator +
    camera feed work out of the box), then falls back to any run with
    waypoints, then any run with a coordinates.csv at all.
    """
    if not SURVEY_DIR.exists():
        return None
    runs = [d for d in SURVEY_DIR.iterdir()
            if d.is_dir() and (d / "coordinates.csv").exists()]
    if not runs:
        return None

    full   = [d for d in runs if _csv_data_rows(d / "coordinates.csv") > 0 and _has_photo(d)]
    with_wp = [d for d in runs if _csv_data_rows(d / "coordinates.csv") > 0]
    pool = full or with_wp or runs
    return max(pool, key=lambda d: d.stat().st_mtime)


def simulator_data_dir() -> Path | None:
    """
    Folder the simulator replays as a 'flight'.
    Prefer real drone_photos/, else newest survey/<run>/ (so the demo works
    out-of-the-box with the survey data already on disk).
    """
    if COORDINATES_CSV.exists():
        return DRONE_PHOTOS_DIR
    return newest_survey_dir()


def active_coordinates_csv() -> Path | None:
    """coordinates.csv the app should read for waypoints / live position."""
    if COORDINATES_CSV.exists():
        return COORDINATES_CSV
    sd = newest_survey_dir()
    if sd is not None:
        return sd / "coordinates.csv"
    return None


def find_model_glb() -> Path | None:
    """Locate a textured 3D model (.glb) from several known locations."""
    candidates = [
        RESULTS_DIR / "3d_map" / "model.glb",
        STAGE3D_DIR / "odm_texturing" / "odm_textured_model_geo.glb",
        STAGE3D_DIR / "odm_texturing" / "odm_textured_model.glb",
        BASE_DIR / "model.glb",
    ]
    existing = [c for c in candidates if c.exists()]
    # fall back to any *.glb under results/ if none of the known names exist
    if not existing and RESULTS_DIR.exists():
        existing = list(RESULTS_DIR.rglob("*.glb"))
    if not existing:
        return None
    # return the NEWEST one, so a fresh 3D run always wins over a stale file
    return max(existing, key=lambda p: p.stat().st_mtime)


def watch_dirs() -> list[Path]:
    """Directories the file-watcher should monitor (only existing ones)."""
    dirs = [RESULTS_DIR, DRONE_PHOTOS_DIR, SURVEY_DIR]
    return [d for d in dirs if d.exists()]


# ──────────────────────────────────────────────────────────────────────────
# Result-set switching (dashboard view): "default" -> results/, "lr64" -> results_lr64/
# (created by a MATCH_LR=64 run). Only the target + annotated paths switch; stitch/field/3d
# stay on the shared default results/. Endpoints read config.<PATH> at call time, so reassigning
# these module globals switches the whole dashboard view instantly.
# ──────────────────────────────────────────────────────────────────────────
ACTIVE_RESULT_SET = "default"
_DEF_TARGETS_JSON = TARGETS_JSON
_DEF_FUSED_CSV    = FUSED_CSV
_DEF_PROOF_HD_DIR = PROOF_HD_DIR
_DEF_VISUALS_DIR  = VISUALS_DIR
_DEF_ANNOTATED    = ANNOTATED_MAP


def available_result_sets() -> list[str]:
    """['default'] + any results_<name>/ variant folders present (e.g. 'lr64')."""
    sets = ["default"]
    try:
        for d in BASE_DIR.glob("results_*"):
            if d.is_dir():
                sets.append(d.name[len("results_"):])
    except Exception:
        pass
    return sets


def set_result_set(name: str) -> bool:
    """Point the target/annotated result paths at a variant folder ('lr64' -> results_lr64/),
    or back to the default ('default'). Returns False if the folder doesn't exist."""
    global ACTIVE_RESULT_SET, TARGETS_JSON, FUSED_CSV, PROOF_HD_DIR, VISUALS_DIR, ANNOTATED_MAP
    if name in ("default", "", None):
        TARGETS_JSON, FUSED_CSV = _DEF_TARGETS_JSON, _DEF_FUSED_CSV
        PROOF_HD_DIR, VISUALS_DIR, ANNOTATED_MAP = _DEF_PROOF_HD_DIR, _DEF_VISUALS_DIR, _DEF_ANNOTATED
        ACTIVE_RESULT_SET = "default"
        return True
    root = BASE_DIR / f"results_{name}"          # e.g. results_lr64
    if not root.exists():
        return False
    tdir, adir = root / "stage3_targets", root / "stage4_annotated"
    TARGETS_JSON  = tdir / "targets.json"
    FUSED_CSV     = tdir / "fused_results.csv"
    PROOF_HD_DIR  = tdir / "proof_hd"
    VISUALS_DIR   = tdir / "visuals"
    ANNOTATED_MAP = adir / "annotated_field.jpg"
    ACTIVE_RESULT_SET = name
    return True
