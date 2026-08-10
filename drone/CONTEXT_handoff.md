# Drone Project — Handoff Context File
**Last updated:** May 31, 2026
**For:** Next Claude session to continue setup

---

## Hardware
- **Compute:** Jetson Orin Nano 8GB — Ubuntu 22.04 (JetPack 6.x)
- **Camera:** Intel RealSense D455 — USB 3.0 — Downward facing
- **Flight Controller:** Pixhawk CubeOrange+ — USB (/dev/ttyACM1) — PX4 v1.15
- **Goal:** GPS-denied stable hover using D455 + ORB-SLAM3

---

## ✅ Phase 1 — D455 Complete
- librealsense installed (RSUSB backend, source build)
- RGB 640x480 @ 30fps ✅
- Depth 640x480 @ 30fps ✅
- IMU Accel+Gyro @ 390Hz ✅

**D455 dual pipeline fix:**
```python
# Pipeline 1 — Video
pipe_video = rs.pipeline()
cfg_video.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
cfg_video.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
# Pipeline 2 — IMU
pipe_imu = rs.pipeline()
cfg_imu.enable_stream(rs.stream.accel, rs.format.motion_xyz32f, 200)
cfg_imu.enable_stream(rs.stream.gyro, rs.format.motion_xyz32f, 200)
```

---

## ✅ Phase 2 — Pixhawk MAVLink Complete
- `/dev/ttyACM1` (can change on reboot — use auto-detect)
- mavsdk + pymavlink installed ✅
- Heartbeat OK ✅
- LOCAL_POSITION_NED publishing @ 0.2Hz ✅

**PX4 Parameters set:**
---

## ✅ Phase 3 — ROS2 D455 Pipeline Complete
**File:** `~/ros2_ws/src/rs_pipeline_node.py`

Topics:
- `/camera/camera/imu` @ 390Hz ✅
- `/camera/camera/color/image_raw` @ 30Hz ✅
- `/camera/camera/depth/image_rect_raw` @ 30Hz ✅

**CRITICAL:** Use `msg._data = buf.flatten()` for color and
`msg._data = buf.view(np.uint8).flatten()` for depth — tobytes() is too slow.
Both IMU and image timestamps use `self.get_clock().now().to_msg()`.

Run:
```bash
source /opt/ros/humble/setup.bash
python3 ~/ros2_ws/src/rs_pipeline_node.py
```

---

## ✅ Phase 4 — ORB-SLAM3 ROS2 Wrapper Complete
**Binary:** `~/ros2_ws/install/orbslam3/lib/orbslam3/rgbd_inertial`
**Node:** `~/ros2_ws/src/orbslam3_ros2/src/rgbd-inertial/`
**Config:** `~/ORB_SLAM3/Examples/RGB-D-Inertial/RealSense_D455.yaml`
**Vocab:** `~/ORB_SLAM3/Vocabulary/ORBvoc.txt` (must be extracted, not .tar.gz)

Publishes pose to `/orb_slam3/odom` @ 30Hz when tracking ✅

Run:
```bash
export LD_LIBRARY_PATH=/home/jetson/ORB_SLAM3/lib:$LD_LIBRARY_PATH
export PANGOLIN_WINDOW_URI="headless://"
unset DISPLAY
VOC=/home/jetson/ORB_SLAM3/Vocabulary/ORBvoc.txt
CFG=/home/jetson/ORB_SLAM3/Examples/RGB-D-Inertial/RealSense_D455.yaml
~/ros2_ws/install/orbslam3/lib/orbslam3/rgbd_inertial $VOC $CFG
```

**KNOWN ISSUE:** Map resets every 1-2 seconds when camera moves.
Root cause: ORB-SLAM3 loses tracking between frames due to depth inconsistency.
**NEXT FIX NEEDED:** Tune ORB-SLAM3 to be more tolerant of tracking loss.
Specifically: increase `Tracking.maxFrames` and lower `ORBextractor.nFeatures`.

**Frame convention (downward camera):**
- SLAM X = right
- SLAM Y = forward  
- SLAM Z = up
- Pixhawk NED: N=Y, E=X, D=-Z

---

## ✅ Phase 5 — Vision Bridge Complete
**File:** `~/pixhawk_vision_bridge.py`

- Subscribes to `/orb_slam3/odom`
- Converts SLAM→NED frame
- Sends `vision_position_estimate` to Pixhawk @ 20Hz
- Skips zero poses (SLAM lost)
- Auto-detects Pixhawk port

Run:
```bash
source /opt/ros/humble/setup.bash
sudo chmod 666 /dev/ttyACM1
python3 ~/pixhawk_vision_bridge.py
```

---

## 🔄 CURRENT ISSUE — Map Resets
SLAM initializes fine (600-800 point maps) but loses tracking every 1-2 seconds.
When tracking lost → pose = 0,0,0 → Pixhawk EKF confused.

**Things to try next session:**
1. Increase `Tracking.maxFrames` in ORB-SLAM3 source
2. Reduce `ORBextractor.nFeatures` from 1000 to 500
3. Try running without IMU (pure RGB-D mode) to isolate issue
4. Check if `EKF2_EV_NOISE` parameters need tuning in QGC

---

## ⏳ Phase 6 — PENDING: Stable Hover Test
1. Fix map reset issue first
2. Props OFF tethered test
3. MAVLink offboard mode
4. Altitude hold via depth
5. Position hold via SLAM pose
6. 30cm hover first

---

## Important Paths
## Full Startup Sequence
```bash
# Terminal 1 — Pipeline
source /opt/ros/humble/setup.bash
python3 ~/ros2_ws/src/rs_pipeline_node.py

# Terminal 2 — SLAM
export LD_LIBRARY_PATH=/home/jetson/ORB_SLAM3/lib:$LD_LIBRARY_PATH
export PANGOLIN_WINDOW_URI="headless://"
unset DISPLAY
VOC=/home/jetson/ORB_SLAM3/Vocabulary/ORBvoc.txt
CFG=/home/jetson/ORB_SLAM3/Examples/RGB-D-Inertial/RealSense_D455.yaml
~/ros2_ws/install/orbslam3/lib/orbslam3/rgbd_inertial $VOC $CFG 2>&1 | grep 'State=2'

# Terminal 3 — Vision Bridge
source /opt/ros/humble/setup.bash
sudo chmod 666 /dev/ttyACM1
python3 ~/pixhawk_vision_bridge.py
```
