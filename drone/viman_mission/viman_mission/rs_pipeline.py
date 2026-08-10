#!/usr/bin/env python3
"""
rs_pipeline — RealSense color+depth driver tuned for RTAB-Map VIO.
Team Viman Rakshak — IRoC-U 2026

Fixes over the original rs_pipeline_node.py (each was a real VIO bug):

  1. HARDWARE TIMESTAMPS. Frames are stamped with the camera's own
     global-time timestamp (host-synced by librealsense), not the wall
     clock at publish time. The old way baked 15–40 ms of variable
     processing lag into every stamp — which EKF2_EV_DELAY cannot
     compensate because it isn't constant. Fallback: host time captured
     immediately after wait_for_frames(), before align/filter.
  2. CORRECT ALIGNED-DEPTH IDENTITY. After rs.align(color) the depth
     image lives in the COLOR optical frame with COLOR intrinsics.
     It is now published that way (frame_id = color optical frame,
     camera_info = color intrinsics). The old node published the raw
     depth sensor's intrinsics under a depth frame_id — wrong for any
     consumer that reprojects.
  3. STATIC TF PUBLISHED. camera_link → camera_color_optical_frame
     (the standard optical rotation). RTAB-Map is launched with
     frame_id:=camera_link and silently needs this transform.
  4. HOLE-FILLING OFF by default. It fabricates depth at object edges —
     exactly where GFTT places features — corrupting odometry inliers
     AND permanently baking invented geometry into the .db map.
     Spatial filter stays on (smooths real data, doesn't invent it).
  5. QoS is RELIABLE by default — RTAB-Map's subscribers request
     RELIABLE, and a best-effort publisher cannot connect to them.
     (Set reliable_qos:=false only if rtabmap is launched with qos:=2.)
  6. Identical stamp on color + depth from the same frameset —
     required by approx_sync:=false in the RTAB-Map launch.
  7. Clean shutdown: thread joined, pipeline stopped exactly once.

Both color and depth run 848x480 @ 30 fps (matching rates are required
for exact sync; the D435 depth sweet spot is 848x480).
"""

import array
import atexit
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                       ReliabilityPolicy)

from builtin_interfaces.msg import Time as TimeMsg
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import StaticTransformBroadcaster

import pyrealsense2 as rs

COLOR_OPTICAL_FRAME = "camera_color_optical_frame"
CAMERA_LINK_FRAME   = "camera_link"

# Standard ROS optical rotation: camera_link (x fwd, y left, z up) →
# optical (z fwd, x right, y down). Same values realsense2_camera uses.
OPTICAL_ROTATION_Q = (-0.5, 0.5, -0.5, 0.5)   # x, y, z, w


def make_qos(reliable: bool, depth: int = 5) -> QoSProfile:
    """RELIABLE by default: a reliable PUBLISHER is compatible with both
    reliable and best-effort subscribers, while a best-effort publisher
    is rejected by reliable subscribers — and RTAB-Map subscribes
    reliable by default (qos_image=0). Best-effort publishing only works
    if RTAB-Map is launched with qos:=2."""
    return QoSProfile(
        reliability=(ReliabilityPolicy.RELIABLE if reliable
                     else ReliabilityPolicy.BEST_EFFORT),
        history=HistoryPolicy.KEEP_LAST,
        depth=depth,
        durability=DurabilityPolicy.VOLATILE,
    )


class RsPipeline(Node):

    def __init__(self):
        super().__init__("rs_pipeline")

        # ── Parameters ───────────────────────────────────────────
        p = self.declare_parameters("", [
            ("width",            848),
            ("height",           480),
            ("fps",              30),
            ("laser_power",      360.0),   # max on D435 — best depth
            ("rgb_ae_limit_us",  200.0),   # auto-exposure cap (motion blur);
                                           # 0 = uncapped. VERIFY image isn't
                                           # too dark in arena lighting!
            ("spatial_filter",   True),
            ("hole_filling",     False),   # OFF for VIO — see header
            ("publish_tf",       True),
            ("reliable_qos",     True),    # matches RTAB-Map defaults
            ("hw_reset_on_start", True),   # auto "replug" — heals stuck USB
            ("start_retries",    3),
            ("stats_period_s",   10.0),
        ])
        (self._w, self._h, self._fps, self._laser, self._ae_limit,
         self._use_spatial, self._use_holes, self._publish_tf,
         self._reliable_qos, self._hw_reset, self._start_retries,
         self._stats_period) = (x.value for x in p)

        # ── Publishers ───────────────────────────────────────────
        qos = make_qos(self._reliable_qos)
        self._color_pub = self.create_publisher(
            Image, "/camera/camera/color/image_raw", qos)
        self._depth_pub = self.create_publisher(
            Image, "/camera/camera/depth/image_rect_raw", qos)
        self._color_info_pub = self.create_publisher(
            CameraInfo, "/camera/camera/color/camera_info", qos)
        self._depth_info_pub = self.create_publisher(
            CameraInfo, "/camera/camera/depth/camera_info", qos)

        # ── RealSense pipeline (self-healing startup) ────────────
        # A previous run that died uncleanly leaves the camera in a
        # half-open USB state ("failed to set power state"). The fix a
        # human does is replug the cable; hardware_reset() is the same
        # thing over USB — do it automatically, then retry the open.
        self._stopped = False
        atexit.register(self._shutdown)   # stop pipeline on ANY exit path

        if self._hw_reset:
            self._hardware_reset_and_wait()

        self._pipe = rs.pipeline()
        cfg = rs.config()
        cfg.enable_stream(rs.stream.color, self._w, self._h,
                          rs.format.rgb8, self._fps)
        cfg.enable_stream(rs.stream.depth, self._w, self._h,
                          rs.format.z16, self._fps)
        profile = self._start_with_retry(cfg)

        self._configure_sensors(profile)

        # ── Filters ──────────────────────────────────────────────
        self._spatial = None
        if self._use_spatial:
            self._spatial = rs.spatial_filter()
            self._spatial.set_option(rs.option.filter_magnitude, 2)
            self._spatial.set_option(rs.option.filter_smooth_alpha, 0.5)
            self._spatial.set_option(rs.option.filter_smooth_delta, 20)
        self._holes = None
        if self._use_holes:
            self._holes = rs.hole_filling_filter()
            self._holes.set_option(rs.option.holes_fill, 1)
            self.get_logger().warn(
                "Hole-filling ENABLED — do not fly VIO with this on; "
                "it fabricates depth and contaminates the .db map.")
        self.get_logger().info(
            f"Filters: spatial={'ON' if self._spatial else 'OFF'}  "
            f"hole-filling={'ON' if self._holes else 'OFF'}  temporal=OFF")

        # ── Align depth → color ──────────────────────────────────
        self._align = rs.align(rs.stream.color)

        # ── Camera info: BOTH topics carry COLOR intrinsics, because
        #    the published depth is aligned into the color frame. ──
        color_sp = profile.get_stream(rs.stream.color) \
                          .as_video_stream_profile()
        self._color_info = self._build_info(color_sp, COLOR_OPTICAL_FRAME)
        self._depth_info = self._build_info(color_sp, COLOR_OPTICAL_FRAME)

        # ── Static TF: camera_link → color optical frame ─────────
        if self._publish_tf:
            self._tf_pub = StaticTransformBroadcaster(self)
            t = TransformStamped()
            t.header.stamp = self.get_clock().now().to_msg()
            t.header.frame_id = CAMERA_LINK_FRAME
            t.child_frame_id = COLOR_OPTICAL_FRAME
            (t.transform.rotation.x, t.transform.rotation.y,
             t.transform.rotation.z, t.transform.rotation.w) = \
                OPTICAL_ROTATION_Q
            self._tf_pub.sendTransform(t)
            self.get_logger().info(
                f"Static TF published: {CAMERA_LINK_FRAME} → "
                f"{COLOR_OPTICAL_FRAME}")

        # ── Timestamp mode (decided on first frame) ──────────────
        self._ts_mode = None     # "hw" | "host"
        self._last_stamp = (-1, -1)   # duplicate-stamp guard

        # ── Stats ────────────────────────────────────────────────
        self._frames = 0
        self._drops  = 0
        self._stats_t0 = time.monotonic()

        # ── Capture thread ───────────────────────────────────────
        self._running = True
        self._thread = threading.Thread(target=self._frame_loop,
                                        daemon=True)
        self._thread.start()
        self.get_logger().info(
            f"Pipeline up: {self._w}x{self._h}@{self._fps} color+depth, "
            "aligned, hardware-stamped")

    # ════════════════════════════════════════════════════════════
    #  SELF-HEALING STARTUP / SHUTDOWN
    # ════════════════════════════════════════════════════════════

    def _hardware_reset_and_wait(self, timeout_s: float = 12.0):
        """USB-level reset (same effect as physically replugging),
        then wait for the camera to re-enumerate."""
        devs = rs.context().query_devices()
        if devs.size() == 0:
            self.get_logger().warn(
                "No RealSense device found pre-reset — continuing; "
                "start will retry anyway")
            return
        self.get_logger().info("Hardware-resetting camera (auto-replug)…")
        try:
            devs[0].hardware_reset()
        except Exception as e:
            self.get_logger().warn(f"hardware_reset failed: {e}")
            return
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            time.sleep(1.0)
            try:
                if rs.context().query_devices().size() > 0:
                    time.sleep(1.0)   # settle after re-enumeration
                    self.get_logger().info("Camera re-enumerated ✓")
                    return
            except Exception:
                pass
        self.get_logger().warn(
            f"Camera did not re-enumerate within {timeout_s:.0f}s — "
            "trying to start anyway")

    def _start_with_retry(self, cfg):
        last_err = None
        for attempt in range(1, int(self._start_retries) + 1):
            try:
                profile = self._pipe.start(cfg)
                if attempt > 1:
                    self.get_logger().info(
                        f"Pipeline started on attempt {attempt}")
                return profile
            except RuntimeError as e:
                last_err = e
                self.get_logger().warn(
                    f"pipe.start failed (attempt {attempt}/"
                    f"{int(self._start_retries)}): {e} — retrying in 3 s")
                time.sleep(3.0)
        self.get_logger().fatal(
            f"Camera failed to start after {int(self._start_retries)} "
            f"attempts: {last_err}. If this persists, update firmware "
            "(rs-fw-update) — 5.13.0.50 is known-bad for stuck states.")
        raise last_err

    def _shutdown(self):
        """Idempotent: called from destroy_node AND atexit, so the
        pipeline is stopped on every exit path — a camera left open is
        exactly what forces the physical replug next run."""
        if self._stopped:
            return
        self._stopped = True
        self._running = False
        try:
            if hasattr(self, "_thread") and self._thread.is_alive():
                self._thread.join(timeout=2.0)
        except Exception:
            pass
        try:
            self._pipe.stop()
        except Exception:
            pass

    # ════════════════════════════════════════════════════════════

    def _configure_sensors(self, profile):
        for sensor in profile.get_device().query_sensors():
            name = sensor.get_info(rs.camera_info.name)
            if name == "RGB Camera":
                try:
                    sensor.set_option(rs.option.global_time_enabled, 1)
                except Exception:
                    pass
                try:
                    sensor.set_option(rs.option.enable_auto_exposure, 1)
                    if self._ae_limit > 0:
                        sensor.set_option(
                            rs.option.auto_exposure_limit_toggle, 1)
                        sensor.set_option(
                            rs.option.auto_exposure_limit, self._ae_limit)
                        self.get_logger().info(
                            f"RGB: auto-exposure ON, "
                            f"limit={self._ae_limit:.0f} µs")
                    else:
                        self.get_logger().info(
                            "RGB: auto-exposure ON, uncapped")
                except Exception as e:
                    self.get_logger().warn(f"RGB exposure config: {e} — "
                                           "auto-exposure default")
            elif name == "Stereo Module":
                try:
                    sensor.set_option(rs.option.global_time_enabled, 1)
                except Exception:
                    pass
                try:
                    sensor.set_option(rs.option.emitter_enabled, 1)
                    sensor.set_option(rs.option.laser_power, self._laser)
                    self.get_logger().info(
                        f"Stereo: IR emitter ON, laser={self._laser:.0f}")
                except Exception as e:
                    self.get_logger().warn(f"Stereo option: {e}")

    @staticmethod
    def _build_info(stream_profile, frame_id) -> CameraInfo:
        i = stream_profile.get_intrinsics()
        m = CameraInfo()
        m.header.frame_id = frame_id
        m.width, m.height = i.width, i.height
        m.distortion_model = "plumb_bob"
        m.d = list(i.coeffs)
        m.k = [i.fx, 0.0, i.ppx, 0.0, i.fy, i.ppy, 0.0, 0.0, 1.0]
        m.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        m.p = [i.fx, 0.0, i.ppx, 0.0,
               0.0, i.fy, i.ppy, 0.0,
               0.0, 0.0, 1.0, 0.0]
        return m

    # ════════════════════════════════════════════════════════════

    def _stamp_for(self, frames) -> TimeMsg:
        """Hardware (global-time) stamp if available, else host time
        captured before processing. Decided once, logged once."""
        if self._ts_mode is None:
            domain = frames.get_frame_timestamp_domain()
            self._ts_mode = (
                "hw" if domain in (
                    rs.timestamp_domain.global_time,
                    rs.timestamp_domain.system_time) else "host")
            self.get_logger().info(
                f"Timestamp mode: {self._ts_mode} (domain={domain})")

        t = TimeMsg()
        if self._ts_mode == "hw":
            ms = frames.get_timestamp()          # epoch ms, host-synced
            t.sec = int(ms // 1000.0)
            t.nanosec = int((ms - t.sec * 1000.0) * 1e6)
        else:
            now = self.get_clock().now().nanoseconds
            t.sec = now // 1_000_000_000
            t.nanosec = now % 1_000_000_000
        return t

    def _frame_loop(self):
        while self._running:
            try:
                frames = self._pipe.wait_for_frames(timeout_ms=1000)
            except RuntimeError:
                self._drops += 1
                continue

            # Stamp FIRST — before align/filter add processing lag
            stamp = self._stamp_for(frames)

            # librealsense occasionally repeats a hardware timestamp
            # (esp. at startup); RTAB rejects non-increasing stamps, so
            # drop the duplicate frameset here instead.
            if (stamp.sec, stamp.nanosec) == self._last_stamp:
                self._drops += 1
                continue
            self._last_stamp = (stamp.sec, stamp.nanosec)

            aligned = self._align.process(frames)
            color = aligned.get_color_frame()
            depth = aligned.get_depth_frame()
            if not color or not depth:
                self._drops += 1
                continue

            if self._spatial is not None:
                depth = self._spatial.process(depth).as_depth_frame()
            if self._holes is not None:
                depth = self._holes.process(depth).as_depth_frame()

            try:
                self._publish(color, depth, stamp)
            except Exception as e:
                if self._running:
                    self.get_logger().error(f"Publish error: {e}")
                continue

            self._frames += 1
            now = time.monotonic()
            if now - self._stats_t0 >= self._stats_period:
                hz = self._frames / (now - self._stats_t0)
                self.get_logger().info(
                    f"{hz:.1f} fps  (drops: {self._drops})")
                self._frames = 0
                self._drops = 0
                self._stats_t0 = now

    @staticmethod
    def _img_bytes(frame) -> array.array:
        """Frame data as array('B') — rclpy's Image.data setter has a
        zero-copy fast path for array('B'). Passing bytes instead would
        trigger a per-BYTE pure-Python validation loop (~340 ms/frame
        pair on Jetson → the 3 fps bug)."""
        a = array.array("B")
        a.frombytes(bytes(frame.get_data()))
        return a

    def _publish(self, color, depth, stamp):
        w, h = color.get_width(), color.get_height()

        cm = Image()
        cm.header.stamp = stamp
        cm.header.frame_id = COLOR_OPTICAL_FRAME
        cm.height, cm.width = h, w
        cm.encoding = "rgb8"
        cm.is_bigendian = 0
        cm.step = w * 3
        cm.data = self._img_bytes(color)
        self._color_pub.publish(cm)

        dw, dh = depth.get_width(), depth.get_height()
        dm = Image()
        dm.header.stamp = stamp                    # identical stamp —
        dm.header.frame_id = COLOR_OPTICAL_FRAME   # aligned = color frame
        dm.height, dm.width = dh, dw
        dm.encoding = "16UC1"
        dm.is_bigendian = 0
        dm.step = dw * 2
        dm.data = self._img_bytes(depth)
        self._depth_pub.publish(dm)

        self._color_info.header.stamp = stamp
        self._color_info_pub.publish(self._color_info)
        self._depth_info.header.stamp = stamp
        self._depth_info_pub.publish(self._depth_info)

    # ════════════════════════════════════════════════════════════

    def destroy_node(self):
        self._shutdown()
        super().destroy_node()


def main():
    rclpy.init()
    node = None
    try:
        node = RsPipeline()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            if node is not None:
                node.destroy_node()
        except Exception:
            pass
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()
