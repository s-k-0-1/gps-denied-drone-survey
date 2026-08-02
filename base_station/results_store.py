"""
results_store.py — reads the pipeline's outputs and watches the results/
folder for changes.

* read_targets()  → merged target list from targets.json + fused_results.csv
* read_summary()  → "Found N/M targets", field size, origin
* ResultsWatcher  → watchdog observer that fires a debounced callback whenever
                    results/, drone_photos/ or survey/ change, so the UI can
                    auto-refresh.
"""
from __future__ import annotations

import csv
import json
import threading
import time
from pathlib import Path
from typing import Callable, Optional

try:
    from . import config
except ImportError:
    import config


# ──────────────────────────────────────────────────────────────────────────
# Reading results
# ──────────────────────────────────────────────────────────────────────────
def _load_fused_csv() -> dict[str, dict]:
    """feature → row dict from fused_results.csv (for confidence + proof)."""
    out: dict[str, dict] = {}
    if not config.FUSED_CSV.exists():
        return out
    try:
        with config.FUSED_CSV.open(newline="", encoding="utf-8-sig") as f:
            for row in csv.DictReader(f):
                feat = (row.get("feature") or "").strip()
                if feat:
                    out[feat] = row
    except Exception:
        pass
    return out


def read_targets() -> list[dict]:
    """
    Merged, UI-ready target list. Prefers map_xyz (mosaic-perspective coords),
    falls back to object_xyz. Joins fused_results.csv for confidence + proof.
    """
    fused = _load_fused_csv()
    targets: list[dict] = []

    data = {}
    if config.TARGETS_JSON.exists():
        try:
            data = json.loads(config.TARGETS_JSON.read_text())
        except Exception:
            data = {}

    json_targets = data.get("targets", [])

    # If targets.json missing but CSV exists, synthesize from CSV.
    if not json_targets and fused:
        for feat, row in fused.items():
            json_targets.append({
                "target": feat,
                "found": bool(row.get("matched_photo")),
                "drone_photo": row.get("matched_photo", ""),
                "object_xyz": None,
            })

    for t in json_targets:
        name = t.get("target", "")
        row = fused.get(name, {})
        xyz = t.get("map_xyz") or t.get("object_xyz") or {}
        proof = (row.get("hd_proof") or "").strip()
        proof_exists = bool(proof) and (config.PROOF_HD_DIR / proof).exists()
        targets.append({
            "name": name,
            "identity": row.get("identity", ""),
            "found": bool(t.get("found")),
            "x": xyz.get("x"),
            "y": xyz.get("y"),
            "z": xyz.get("z"),
            "confidence": row.get("confidence", ""),
            "matched_photo": t.get("drone_photo") or row.get("matched_photo", ""),
            "loftr": row.get("loftr", t.get("loftr_inliers", "")),
            "superpoint": row.get("superpoint", t.get("sp_inliers", "")),
            "proof": proof if proof_exists else None,
            "has_proof": proof_exists,
        })
    return targets


def read_summary() -> dict:
    targets = read_targets()
    found = sum(1 for t in targets if t.get("found"))
    total = len(targets)
    origin = {}
    field = {}
    if config.TARGETS_JSON.exists():
        try:
            data = json.loads(config.TARGETS_JSON.read_text())
            origin = data.get("origin_enu", {})
        except Exception:
            pass
    # field size from calibration.txt if present
    calib = config.FIELD_DIR / "calibration.txt"
    if calib.exists():
        try:
            for line in calib.read_text().splitlines():
                if line.lower().startswith("field size"):
                    field["raw"] = line.split(":", 1)[1].strip()
        except Exception:
            pass
    return {
        "found": found,
        "total": total,
        "targets": targets,
        "origin_enu": origin,
        "field": field,
        "has_annotated": config.ANNOTATED_MAP.exists(),
        "has_ortho": config.ORTHOMOSAIC.exists(),
        "has_model": config.find_model_glb() is not None,
        "updated": _latest_mtime(),
    }


def _latest_mtime() -> float:
    paths = [config.TARGETS_JSON, config.ANNOTATED_MAP, config.FUSED_CSV]
    glb = config.find_model_glb()          # so re-running 3D triggers a reload
    if glb is not None:
        paths.append(glb)
    mt = 0.0
    for p in paths:
        try:
            if p.exists():
                mt = max(mt, p.stat().st_mtime)
        except Exception:
            pass
    return mt


# ──────────────────────────────────────────────────────────────────────────
# File watching
# ──────────────────────────────────────────────────────────────────────────
class ResultsWatcher:
    """
    Debounced recursive watcher over results/, drone_photos/, survey/.
    Calls ``on_change(kind)`` where kind is a coarse hint ('results',
    'photos', 'map', 'targets', '3d').
    """
    def __init__(self, on_change: Callable[[str], None], debounce_s: float = 0.6):
        self._on_change = on_change
        self._debounce = debounce_s
        self._observer = None
        self._timer: Optional[threading.Timer] = None
        self._pending: set[str] = set()
        self._lock = threading.Lock()

    def start(self) -> bool:
        try:
            from watchdog.observers import Observer
            from watchdog.events import FileSystemEventHandler
        except Exception:
            return False

        store = self

        class _Handler(FileSystemEventHandler):
            def on_any_event(self, event):
                if event.is_directory:
                    return
                store._note(str(event.src_path))

        self._observer = Observer()
        watched = 0
        for d in config.watch_dirs():
            try:
                self._observer.schedule(_Handler(), str(d), recursive=True)
                watched += 1
            except Exception:
                continue
        if watched == 0:
            return False
        self._observer.daemon = True
        self._observer.start()
        return True

    def _note(self, path: str) -> None:
        kind = "results"
        p = path.replace("\\", "/").lower()
        if "stage4_annotated" in p or "annotated_field" in p or "orthomosaic" in p:
            kind = "map"
        elif "stage3_targets" in p or "targets.json" in p or "fused_results" in p or "proof_hd" in p:
            kind = "targets"
        elif "3d" in p or p.endswith(".glb"):
            kind = "3d"
        elif "drone_photos" in p or "survey" in p:
            kind = "photos"
        with self._lock:
            self._pending.add(kind)
            if self._timer is not None:
                self._timer.cancel()
            self._timer = threading.Timer(self._debounce, self._flush)
            self._timer.daemon = True
            self._timer.start()

    def _flush(self) -> None:
        with self._lock:
            kinds = list(self._pending)
            self._pending.clear()
        for k in kinds:
            try:
                self._on_change(k)
            except Exception:
                pass

    def stop(self) -> None:
        if self._observer is not None:
            try:
                self._observer.stop()
                self._observer.join(timeout=2)
            except Exception:
                pass
