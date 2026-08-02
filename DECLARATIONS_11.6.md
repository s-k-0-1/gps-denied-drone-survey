# IRoC-U 2026 — System Declarations (Rulebook §11.6)

**Team:** _[fill team name / ID]_
**Sub-system:** ASCEND — Autonomous Survey & Cartographic Enumeration of Novel Distinctives
**Date:** _[fill]_

> This document declares how our target-localization pipeline works, for the Final Field
> Round. Cross-check the exact wording of §11.6 in the rulebook and adjust labels if needed.

---

## 1. Localization source — No-GPS / No-GNSS compliance

We **do not use GPS or any GNSS** for positioning at any stage. All position data comes from:

- **Onboard camera** (downward-facing HD) + **Pixhawk (PX4) Visual-Inertial Odometry / optical-flow**, giving per-frame pose `(x, y, z, yaw)`.
- Telemetry is logged per captured image in `coordinates.csv`
  (`x_enu, y_enu, z_enu, yaw_deg, image_file`).

No satellite navigation, no external positioning beacon, no pre-surveyed GPS control points are used. This satisfies the rulebook no-GPS mandate.

## 2. Coordinate system

| Item | Declaration |
|---|---|
| **Origin (0,0)** | Base-station reference point (see §5). Coordinates of every target are reported **relative to the base station**. |
| **X axis** | Along the longer arena edge (30 ft). |
| **Y axis** | Along the shorter arena edge (25 ft), perpendicular to X. |
| **Z** | Height from Pixhawk barometer/VIO (reported as provided, unmodified). |
| **Units** | Metres. |
| **Handedness / heading** | Axes aligned to the initial heading assigned at run-time (0°/90°/180°/270°). |

**Metric scale:** the known arena dimensions (30 ft × 25 ft) are used to scale the stitched
map to true metres; the longer pixel edge of the rectified field is set to 30 ft, the shorter to 25 ft.

> **Implementation note (internal):** the pipeline currently anchors the map origin at the
> **yellow-boundary inner corner** and reports positions in the rectified-arena (visual) frame.
> When the base station is placed at that corner this equals the required frame. If the base
> station is assigned to a **different** position, enable base-station-origin mode so the reported
> origin coincides with the actual base-station location.

## 3. Survey / flight pattern

- **Lawnmower (boustrophedon) grid** over the full arena — parallel sweeps with alternating
  direction so consecutive rows connect.
- Nominal altitude ≈ 3 m; forward/side overlap maintained so adjacent HD frames share features
  (required for image stitching).
- Each capture stores the corresponding Pixhawk pose in `coordinates.csv`.

## 4. Data-processing location

- **Ground-based (off-board) post-processing.** Captured HD photos and the telemetry CSV are
  transferred to the ground station after the survey.
- On the ground station we run: image stitching (feature matching → mosaic), yellow-boundary
  detection and perspective rectification, and DINOv2 semantic target localization.
- No real-time on-board GPU inference is required to produce the target coordinates; the drone's
  onboard job is capture + pose logging.

## 5. Reference-frame handling & drift

- The VIO/odometry frame is initialised at **take-off from the base station**; yaw is measured
  relative to the assigned initial heading.
- Final target coordinates are computed in the **visual (rectified-arena) frame** derived from the
  yellow boundary — so the reported positions are **independent of VIO drift or mid-flight frame
  resets**. The odometry is used to associate each photo with its arena location; the yellow
  boundary provides the absolute, drift-free reference.

## 6. Target deliverables (per detected feature)

For each of the (max 3) unique targets we submit:

- **(a) Low-resolution image** corresponding to the seed — `lr_match/<target>.png` (128 px).
- **(b) High-resolution image** — `proof_hd/<target>.jpg`, native-resolution, feature-centred,
  ≥ 720 px shorter side.
- **(c) Coordinates** — base-station-relative `(x, y)` in metres (with Z as logged).
- **(d) Battery voltage** — read from the flight-controller/hardware telemetry.

## 7. Assumptions & notes

- Arena is a known **30 ft × 25 ft** rectangle bounded by a yellow line; this is used for metric
  scaling and rectification.
- There are up to **3 unique targets**, one per seed image; no other object in the arena resembles
  a seed (so semantic single-best matching is valid).
- Seed images are provided at **64 × 64** (Final round spec).
- A confidence threshold is applied: if no location clears it, the target is reported **NOT FOUND**
  rather than a random guess.

---

_Prepared for IRoC-U 2026 Final Field Round submission. Values in italics/underscore are to be
filled by the team._
