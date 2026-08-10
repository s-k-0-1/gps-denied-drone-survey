"""Single source of truth for the RTAB-Map launch command.

Previously this exact configuration was duplicated verbatim in
auto_mission.py and rtabmap_trigger.py — any tuning change had to be
made twice. Both nodes now build their command here.

CLI override strategy (rtabmap.launch.py):
  `args`         → fed to BOTH rgbd_odometry AND rtabmap nodes
  `odom_args`    → fed ONLY to rgbd_odometry
  `rtabmap_args` → fed ONLY to rtabmap (merged with args)

CRITICAL: Odom/* and OdomF2M/* must go in odom_args ONLY.
rtabmap does NOT declare these params and crashes with
ParameterNotDeclaredException if they appear in args.
OdomF2M/BundleAdjustment MUST be in odom_args — the := form is
overridden by the param flood back to 1, causing
"Too low inliers after bundle adjustment" quality drops.
"""

from typing import List

# ── MAX-QUALITY TUNING ─────────────────────────────────────────────
# Odometry node is flight-critical → tuned up moderately; mapping node
# runs async → tuned up aggressively. Previous flight-tested values
# shown as (was X) — revert if odometry rate drops below ~15 Hz.

# Params valid for BOTH nodes (Vis/*, GFTT/*, Kp/*):
_SHARED_CLI = (
    "--Vis/MinInliers 8 "
    "--Vis/InlierDistance 0.1 "
    "--Vis/CorNNDRRatio 0.80 "
    "--GFTT/MinDistance 5 "       # (was 7)  denser feature grid
    "--GFTT/QualityLevel 0.005 "  # (was 0.01) more features accepted
    "--Kp/MaxFeatures 750 "       # (was 500) richer loop-closure vocabulary
    "--Kp/MaxDepth 8.0 "
    "--Kp/MinDepth 0.3"
)

# Params for rgbd_odometry ONLY — crashes rtabmap if in args:
_ODOM_ONLY_CLI = (
    "--Odom/ResetCountdown 5 "
    "--Odom/Strategy 0 "
    "--Odom/FilteringStrategy 1 "
    "--Odom/GuessMotion true "
    "--Odom/KeyFrameThr 0.5 "
    "--OdomF2M/BundleAdjustment 0 "  # KEEP 0 — BA over-filters inliers (flight-tested fix)
    "--OdomF2M/MaxSize 4000 "        # (was 3000) bigger local map → steadier poses
    "--OdomF2M/MaxNewFeatures 400 "  # (was 300)
    "--OdomF2M/ValidDepthRatio 0.3"
)

# Params for rtabmap ONLY (loop closure / SLAM node):
_RTABMAP_ONLY_CLI = (
    "--Vis/MinInliers 8 "
    "--Rtabmap/DetectionRate 2.0 "   # (was 1.0) denser map graph
    "--RGBD/OptimizeMaxError 1.5 "
    "--RGBD/LinearUpdate 0.05 "      # add nodes after 5 cm motion (default 0.1 m)
    "--RGBD/AngularUpdate 0.05 "     # ...or ~3° rotation
    "--Mem/ImagePreDecimation 1 "    # full-resolution images stored in .db
    "--Mem/ImagePostDecimation 1 "   # → best offline reprocessing/export
    "--Mem/NotLinkedNodesKept true"  # keep every captured frame
)


def build_rtabmap_cmd(db_path: str, init_pose: str) -> List[str]:
    """Full `ros2 launch` argv for the RTAB-Map stack.

    Args:
        db_path:   output .db file path for the map.
        init_pose: "x y z roll pitch yaw" — true starting pose in the world
                   frame (ENU, origin = home/arm point). Without this,
                   RTAB-Map sets (0,0,0) at the drone's current position,
                   causing a Z offset in all published vision poses → PX4
                   EKF shifts → drone overshoots altitude → crash on land.
    """
    return [
        "ros2", "launch", "rtabmap_launch", "rtabmap.launch.py",
        f"database_path:={db_path}",
        f"initial_pose:={init_pose}",
        "rgb_topic:=/camera/camera/color/image_raw",
        "depth_topic:=/camera/camera/depth/image_rect_raw",
        "camera_info_topic:=/camera/camera/color/camera_info",
        "frame_id:=camera_link",
        "approx_sync:=false",
        "odom_topic:=rtabmap/odom",
        "visual_odometry:=true",
        "publish_tf:=true",
        "tf_delay:=0.05",
        "tf_tolerance:=0.2",
        "rviz:=false",
        "rtabmap_viz:=false",
        "odom_frame_id:=rtabmap/odom",
        # `:=` form for params not subject to flood override:
        "Odom/ImageDecimation:=1",      # full-resolution odometry
        "Vis/FeatureType:=9",
        "Vis/MaxFeatures:=1200",        # (was 800) more odom features — watch CPU
        "Vis/PnPFlags:=0",
        "Vis/DepthAsMask:=false",
        "Kp/NNStrategy:=1",
        "Kp/BadSignRatio:=0.3",
        "LccBow/MinInliers:=6",
        "LccBow/InlierDistance:=0.15",
        "Reg/VarianceFromInliersCount:=false",
        "RGBD/ProximityBySpace:=true",
        "RGBD/ProximityMaxGraphDepth:=0",
        "RGBD/ProximityPathMaxNeighbors:=20",
        "Mem/STMSize:=50",
        "Mem/RehearsalSimilarity:=0.5",
        "Grid/FromDepth:=false",
        "RGBD/OptimizeFromGraphEnd:=false",
        "Optimizer/Slam2D:=false",
        # CLI strings — applied by RTAB-Map's own parser, highest priority:
        f"args:={_SHARED_CLI}",
        f"odom_args:={_ODOM_ONLY_CLI}",
        f"rtabmap_args:={_RTABMAP_ONLY_CLI}",
    ]


# ════════════════════════════════════════════════════════════════
#  ROBUST FLIGHT PRESET — for the final architecture (bringup.launch.py)
# ════════════════════════════════════════════════════════════════
# Live RTAB-Map has ONE flight-critical job: stable odometry. Map
# quality comes from offline reprocessing of the stored full-res .db
# (rtabmap-reprocess). So compute params are flight-tested-conservative;
# only the STORAGE params are max-quality (they cost SSD, not CPU).
# No initial_pose: vio_gate's seed-relative output handles alignment.

_ROBUST_SHARED_CLI = (
    "--Vis/MinInliers 6 "           # (was 8) — 7/8 failures seen in flight; 6 survives low-texture runs
    "--Vis/InlierDistance 0.1 "
    "--Vis/CorNNDRRatio 0.80 "
    "--GFTT/MinDistance 6 "         # (was 4) — 4 caused CPU overload: odom running 3-5Hz instead of 30Hz;
                                    # 6 is denser than original 7, still extracts more features in sparse
                                    # texture without saturating Jetson processing budget
    "--GFTT/QualityLevel 0.003 "    # (was 0.01) — accept weaker corners; more features in dim lighting
    "--Kp/MaxFeatures 500 "         # (was 750) — 750 + MinDistance=4 = too many keypoints, Jetson CPU cap
    "--Kp/MaxDepth 8.0 "
    "--Kp/MinDepth 0.3 "
    # ── rtabmap-node params moved here from rtabmap_args ──────────
    # rtabmap_args is NOT a recognised launch arg in rtabmap.launch.py
    # and is silently ignored — confirmed by flight logs showing
    # DetectionRate=1.0 and OptimizeMaxError=3.0 despite being set below.
    # Passing via 'args' (both nodes) works; rgbd_odometry ignores unknown keys.
    "--Rtabmap/DetectionRate 1.0 "  # (was 0.5) — bump to 1 Hz: at 0.25 m/s survey
                                    # speed, 0.5 Hz means only one closure check per
                                    # 0.5m of travel; adjacent stripes (1.5m apart)
                                    # were being missed, causing accumulated drift.
                                    # 1 Hz gives a check every 0.25m — enough to
                                    # catch proximity closures with the previous stripe.
    "--RGBD/OptimizeMaxError 5.0 "  # (was 3.0 default) — edge 58→73 kept blocking
                                    # closures at ratio 3.02-3.12; 5.0 accepts them
                                    # and lets the optimizer stitch map segments
    "--RGBD/ProximityPathMaxNeighbors 40 " # (was 20) — search more spatial neighbours
                                    # per proximity check; catches loop closures from
                                    # one stripe over to the adjacent stripe
    "--Mem/NotLinkedNodesKept true" # keep every frame for offline reprocessing
)
_ROBUST_ODOM_CLI = (
    "--Odom/ResetCountdown 5 "
    "--Odom/Strategy 0 "
    "--Odom/FilteringStrategy 1 "
    "--Odom/GuessMotion true "      # re-enabled — false caused 3-5 Hz odometry (0.2-0.5s/frame)
                                    # because RANSAC starts cold and needs 10× more iterations
                                    # to find consensus at quality=400-600. The Z=-0.297m bad
                                    # guess was a one-off during a quality collapse (EKF bad state),
                                    # not systematic. At survey speed 0.4 m/s, frame-to-frame
                                    # motion is 0.013m — IMU guess is accurate and fast.
    "--Odom/KeyFrameThr 0.5 "
    "--OdomF2M/BundleAdjustment 0 "
    "--OdomF2M/MaxSize 1500 "       # (was 3000) — NN search scales with map size; 3000 points
                                    # caused update_time to grow from 0.116s → 0.313s over a
                                    # long flight as the map filled. 1500 caps this degradation.
    "--OdomF2M/MaxNewFeatures 300 " # (was 500) — fewer new points added per frame keeps map
                                    # from hitting the cap too quickly
    "--OdomF2M/ValidDepthRatio 0.3"
)
_ROBUST_RTABMAP_CLI = (
    # rtabmap_args is silently ignored by rtabmap.launch.py — critical params
    # (DetectionRate, OptimizeMaxError, NotLinkedNodesKept) have been moved to
    # _ROBUST_SHARED_CLI where they are passed via the working 'args' key.
    # This string is kept for any future param that is genuinely rtabmap-only
    # and confirmed to apply via rtabmap_args.
    "--Mem/ImagePreDecimation 1 "    # full-res images in .db for offline reprocessing
    "--Mem/ImagePostDecimation 1"
)


def robust_flight_launch_args(db_path: str) -> List:
    """launch_arguments for IncludeLaunchDescription(rtabmap.launch.py):
    robust compute preset + full-res storage, no initial_pose."""
    return [
        ("database_path", db_path),
        ("rgb_topic", "/camera/camera/color/image_raw"),
        ("depth_topic", "/camera/camera/depth/image_rect_raw"),
        ("camera_info_topic", "/camera/camera/color/camera_info"),
        ("frame_id", "camera_link"),
        ("approx_sync", "false"),
        ("odom_topic", "rtabmap/odom"),
        ("visual_odometry", "true"),
        ("publish_tf", "true"),
        ("tf_delay", "0.05"),
        ("tf_tolerance", "0.2"),
        ("rviz", "false"),
        ("rtabmap_viz", "false"),
        # Quiet the per-frame INFO spam ("Odom: quality=…", "rtabmap (N): Rate=…").
        # WARN/ERROR still print, so "Odometry lost!" and real faults stay visible.
        ("log_level", "warn"),
        ("odom_frame_id", "rtabmap/odom"),
        ("Odom/ImageDecimation", "1"),
        ("Vis/FeatureType", "9"),
        ("Vis/MaxFeatures", "600"),     # (was 800) — reduced with GFTT/MinDistance=6; CPU headroom
        ("Vis/PnPFlags", "0"),
        ("Vis/DepthAsMask", "false"),
        ("Kp/NNStrategy", "1"),
        ("Kp/BadSignRatio", "0.3"),
        ("LccBow/MinInliers", "6"),
        ("LccBow/InlierDistance", "0.15"),
        ("Reg/VarianceFromInliersCount", "false"),
        ("RGBD/ProximityBySpace", "true"),
        ("RGBD/ProximityMaxGraphDepth", "0"),
        ("RGBD/ProximityPathMaxNeighbors", "20"),
        ("Mem/STMSize", "50"),
        ("Mem/RehearsalSimilarity", "0.5"),
        ("Grid/FromDepth", "false"),
        ("RGBD/OptimizeFromGraphEnd", "false"),
        ("Optimizer/Slam2D", "false"),
        ("args", _ROBUST_SHARED_CLI),
        ("odom_args", _ROBUST_ODOM_CLI),
        ("rtabmap_args", _ROBUST_RTABMAP_CLI),
    ]


def build_vision_bridge_cmd(offset_x: float, offset_y: float,
                            offset_z: float) -> List[str]:
    """argv to start vision_bridge via ros2 run with the ENU offset
    captured at RTAB-Map init time (corrects the odom Z-shift)."""
    return [
        "ros2", "run", "viman_mission", "vision_bridge",
        "--ros-args",
        "-p", f"offset_x:={offset_x:.4f}",
        "-p", f"offset_y:={offset_y:.4f}",
        "-p", f"offset_z:={offset_z:.4f}",
    ]
