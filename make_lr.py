#!/usr/bin/env python3
"""
Folder ki saari photos ko LR (64x64) me convert karo -- RULEBOOK V4.0 Final round SEED spec (11.3.1).
(Final round me organizers khud 64x64 seed dete hain; ye script elimination/practice ke liye.)
Originals ko haath nahi lagata -- naye folder me LR copies banata hai.

Run:
    python3 make_lr.py
"""

import os, glob
import cv2

# ---- CONFIG ---- (apne hisaab se change kar lo)
SRC_DIR = os.path.expanduser("~/advanced_matcher/reference")   # input folder
OUT_DIR = os.path.expanduser("~/advanced_matcher/targets") # output folder
LR_SIZE = (64, 64)          # RULEBOOK V4.0 Final round SEED = 64x64 (11.3.1). Drone LR alag se 128
                            # (build_drone_lr, rulebook 10.4). Match = seed-64 <-> drone-128 (DINOv2).
EXTS    = ('*.jpg','*.jpeg','*.png','*.JPG','*.JPEG','*.PNG')

def list_photos(root):
    files = []
    for e in EXTS:
        files += glob.glob(os.path.join(root, e))          # flat
        files += glob.glob(os.path.join(root, "*", e))     # subfolders bhi
    return sorted(set(files))

def main():
    photos = list_photos(SRC_DIR)
    if not photos:
        print(f"ERROR: koi photo nahi mili {SRC_DIR} me")
        return
    print(f"{len(photos)} photos -> LR {LR_SIZE} convert kar rahe hain")
    print(f"  output: {OUT_DIR}\n")

    done = 0
    for p in photos:
        img = cv2.imread(p)
        if img is None:
            print(f"  skip (load fail): {p}")
            continue
        # subfolder structure preserve karo
        rel = os.path.relpath(p, SRC_DIR)
        out_path = os.path.join(OUT_DIR, rel)
        os.makedirs(os.path.dirname(out_path), exist_ok=True)
        lr = cv2.resize(img, LR_SIZE, interpolation=cv2.INTER_LANCZOS4)
        cv2.imwrite(out_path, lr)
        done += 1
        if done % 10 == 0:
            print(f"  [{done}/{len(photos)}] done")

    print(f"\nDone: {done} LR photos saved in {OUT_DIR}")

if __name__ == "__main__":
    main()
