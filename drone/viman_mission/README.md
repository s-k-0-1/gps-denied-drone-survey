# viman_mission

## ★ FINAL ARCHITECTURE (recommended) — bringup.launch.py

Implements FINAL_ARCHITECTURE.md: everything pre-launched on the ground as
parallel processes, flow-only takeoff, validated gated handover to VIO,
flow-only landing. No Python subprocess wraps RTAB-Map — the launch system
owns it (clean SIGINT → .db survives).

```bash
# Terminal 1: MAVROS (as usual)
# Terminal 2:
cd ~/drone_ws && source /opt/ros/humble/setup.bash && source install/setup.bash
ros2 launch viman_mission bringup.launch.py
```

Nodes (each its own process): `rs_pipeline` (hardware-stamped, aligned
RealSense driver — STOP the old root rs_pipeline_node.py, never run both),
RTAB-Map stack (robust preset + full-res storage), `vio_gate`
(initialization factor IF=Q×A×S, seed/gate services, 3 watchdogs),
`mission_director` (preflight gate → mission phases).
Running your own camera driver instead? `start_camera:=false`.

Watch in flight: `ros2 topic echo /viman/init_factor` and `/viman/vio_state`
(0 unseeded, 1 seeding, 2 validating, 3 OPEN, 4/5/6 faults).

Before first flight:
1. Verify the reset service name: `ros2 service list | grep -i reset` —
   set `reset_service` in config/mission_params.yaml if it differs.
2. Bench carry-test: launch bringup with the drone in hand at ~1 m over
   texture, call `ros2 service call /viman/seed std_srvs/srv/Trigger`,
   carry it 1 m East — `/viman/init_factor` should stay high. This
   validates the self-calibrating frame correction with zero flight risk.
3. Set EKF2 params in QGC (see FINAL_ARCHITECTURE.md table), especially
   EKF2_EV_DELAY and COM_OBL_RC_ACT = AUTO.LAND.
4. Ground soak: 30 min running on the pad, watch RAM + `tegrastats`.

After landing — max-quality map offline:
```bash
rtabmap-reprocess --Vis/MaxFeatures 2000 --Kp/MaxFeatures 1500 \
  --Rtabmap/DetectionRate 4 /media/jetson/ROS2_SSD/maps/flight_<ts>.db out.db
```

---

## Legacy implementations (kept for reference)

Autonomous RTAB-Map VIO mission stack (Viman Rakshak / IRoC-U 2026), packaged as a proper ROS2 ament_python package. Flight behavior is identical to the original standalone scripts — only the structure changed.

## Build

```bash
cd ~/drone_ws
colcon build --packages-select viman_mission --symlink-install
source install/setup.bash
```

`--symlink-install` means edits to the Python files take effect without rebuilding (only re-run colcon if you add files or change setup.py).

## Run

```bash
# Full mission (recommended):
ros2 launch viman_mission mission.launch.py

# With overridden params:
ros2 launch viman_mission mission.launch.py params_file:=/path/to/overrides.yaml

# Individual nodes:
ros2 run viman_mission auto_mission
ros2 run viman_mission rtabmap_trigger
ros2 run viman_mission vision_bridge --ros-args -p offset_x:=0.0 -p offset_y:=0.0 -p offset_z:=1.2
```

## Structure

| File | Role |
|---|---|
| `viman_mission/auto_mission.py` | OFFBOARD mission state machine (phase dispatch table) |
| `viman_mission/vision_bridge.py` | RTAB-Map odom → `/mavros/vision_pose/pose` (validate, remap, offset, 30 Hz) |
| `viman_mission/rtabmap_trigger.py` | Standalone altitude-triggered RTAB-Map launcher |
| `viman_mission/rtabmap_config.py` | **Single source of truth** for the RTAB-Map launch command (was duplicated in two files) |
| `viman_mission/common.py` | Shared QoS profiles, covariance check, yaw extraction, frame-convention docs |
| `config/mission_params.yaml` | All tunables (previously hard-coded constants; defaults identical) |
| `launch/mission.launch.py` | Launches auto_mission with the params file |

## Notes

- auto_mission now starts vision_bridge via `ros2 run viman_mission vision_bridge`, so the package **must be built and sourced** before flight (the old hard-coded `/home/jetson/drone_ws/vision_bridge.py` path is gone).
- The original scripts in the workspace root are untouched — keep them as backup until the package is flight-tested.
- Tune RTAB-Map in `rtabmap_config.py`; one change now applies to both auto_mission and rtabmap_trigger.
