# docs/images — what each picture is

Every image used in the documentation lives here. Nothing is generated at build time.

## Flight path / 3D map (RTAB-Map)

| File | What it shows |
|---|---|
| `rtabmap_3d_map_top.png` | Top-down RTAB-Map 3D view of a real flight — point cloud + trajectory |
| `rtabmap_3d_map_oblique.png` | Same flight, tilted, so the constant survey height is visible |
| `rtabmap_3d_map_gui.jpg` | The RTAB-Map GUI (`rtabmapviz`) 3D Map panel during a live flight |
| `rtabmap_loop_closure.svg` | Diagram: how a loop closure corrects accumulated drift |

**Trajectory colour key** (same in all three screenshots):

| Colour | Meaning |
|---|---|
| Magenta / purple | Survey — the lawnmower sweep, one marker per keyframe |
| Yellow | Return to home — the straight leg back to the base station |
| Cyan | Takeoff / landing at the base-station pad |
| Red | Loop closure link — a drift correction |
| White speckle | The point cloud, not a path |

Full explanation: [10 — VIO & Localization §3.8](../10_VIO_LOCALIZATION.md#38-reading-the-3d-map-real-flight-screenshots)

## Hardware

| File | What it shows |
|---|---|
| `drone_build_top.JPG` | The assembled drone from above — frame, arms, props, payload plate |
| `drone_build_angle.JPG` | Angled view — Pixhawk, Jetson, RealSense, battery, docking pads |

## Ground pipeline results

| File | Stage | What it shows |
|---|---|---|
| `orthomosaic.jpg` | 1 | Stitched top-down mosaic of the arena |
| `yellow_mask_debug.jpg` | 2 | The HSV yellow mask used to find the boundary |
| `yellow_corners_debug.jpg` | 2 | The four detected boundary corners |
| `rectified_field.jpg` | 2 | The arena warped to a true-scale rectangle |
| `3.png` | 3 | Target matching — 64×64 seed vs. where it was found |
| `annotated_field.jpg` | 4 | Final map with every located target labelled |

## 3D reconstruction (photogrammetry, from the same 2D photos)

| File | What it shows |
|---|---|
| `3d_model_top.png` | Reconstructed arena, top-down |
| `3d_model_angle.png` | Angled — mesh edges visible on the objects |
| `3d_model_oblique.png` | Low angle — features standing up from the surface (elevation) |

## Diagrams

| File | What it shows |
|---|---|
| `architecture.svg` | Whole system: drone → ground PC → base station, and the 5 pipeline stages |
