#!/usr/bin/env python3
"""
vio_gate — validated, gated bridge between RTAB-Map odometry and PX4.

Runs as its own process from ground bringup (parallel to mission_director).
Publishes NOTHING to /mavros/vision_pose/pose until the mission explicitly
opens the gate — and the gate only opens after the initialization factor
confirms RTAB-Map agrees with the optical-flow EKF.

State machine (published on /viman/vio_state, std_msgs/UInt8):
  0 UNSEEDED         no origin alignment yet
  1 SEEDING          RTAB odom reset requested, waiting for first good frame
  2 VALIDATING       seeded; initialization factor being computed; gate CLOSED
  3 OPEN             gate open; vision poses streaming to PX4
  4 FAULT_QUALITY    covariance ≥ 100 while open      → gate auto-closed
  5 FAULT_RESET      RTAB pose jump (odometry reset)  → gate auto-closed
  6 FAULT_DIVERGENCE vision vs EKF drifted apart      → gate auto-closed

Initialization factor (published on /viman/init_factor, std_msgs/Float32):
  IF = Q × A × S, each ∈ [0, 1]
  Q  quality   : clamp(1 − cov/cov_norm); 0 if lost/silent
  A  agreement : clamp(1 − ‖Δp_rtab_corrected − Δp_ekf‖ / agree_tol)
                 over a sliding window — DELTAS, not absolutes, so the
                 flow-EKF's slow X/Y drift can't fail a good camera
  S  stability : clamp(1 − σ_horizontal_velocity / vel_tol)

Frame correction (self-calibrating — replaces the old hand-tuned remap):
  At seed time:  q_corr = q_ekf ⊗ q_rtab⁻¹
  Always after:  p_out = p_ekf_anchor + R(q_corr)·(p_rtab − p_rtab_anchor)
                 q_out = q_corr ⊗ q_rtab
  The SAME rotation acts on positions and orientations (geometrically
  consistent), and anchors are re-captured at gate-open so the first pose
  PX4 fuses is exactly its own current estimate — zero innovation at t₀.

Services:
  /viman/seed (std_srvs/Trigger) : reset RTAB odom + capture alignment
  /viman/gate (std_srvs/SetBool) : open (true) / close (false) the gate
"""

from collections import deque
from enum import IntEnum

import numpy as np
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped, TwistStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32, UInt8
from std_srvs.srv import Empty, SetBool, Trigger

from viman_mission.common import (qos_best_effort, qos_reliable,
                                  quat_conj, quat_from_msg, quat_mult,
                                  quat_normalize, quat_rotate)


def _quat_from_matrix(R):
    """3x3 rotation matrix → quaternion (x, y, z, w)."""
    t = np.trace(R)
    if t > 0:
        s = np.sqrt(t + 1.0) * 2
        return ((R[2, 1] - R[1, 2]) / s, (R[0, 2] - R[2, 0]) / s,
                (R[1, 0] - R[0, 1]) / s, 0.25 * s)
    i = int(np.argmax([R[0, 0], R[1, 1], R[2, 2]]))
    if i == 0:
        s = np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2
        return (0.25 * s, (R[0, 1] + R[1, 0]) / s,
                (R[0, 2] + R[2, 0]) / s, (R[2, 1] - R[1, 2]) / s)
    if i == 1:
        s = np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2
        return ((R[0, 1] + R[1, 0]) / s, 0.25 * s,
                (R[1, 2] + R[2, 1]) / s, (R[0, 2] - R[2, 0]) / s)
    s = np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2
    return ((R[0, 2] + R[2, 0]) / s, (R[1, 2] + R[2, 1]) / s,
            0.25 * s, (R[1, 0] - R[0, 1]) / s)


class GateState(IntEnum):
    UNSEEDED         = 0
    SEEDING          = 1
    VALIDATING       = 2
    OPEN             = 3
    FAULT_QUALITY    = 4
    FAULT_RESET      = 5
    FAULT_DIVERGENCE = 6


class VioGate(Node):

    def __init__(self):
        super().__init__("vio_gate")

        # ── Parameters ───────────────────────────────────────────
        self.declare_parameter("rtab_odom_topic", "/rtabmap/rtabmap/odom")
        self.declare_parameter("reset_service", "/rtabmap/reset_odom")
        self.declare_parameter("vision_rate_hz", 30.0)
        self.declare_parameter("cov_bad", 100.0)     # ≥ this → lost (complete loss)
        self.declare_parameter("cov_norm", 50.0)     # Q normalization
        self.declare_parameter("window_secs", 2.0)   # agreement window
        self.declare_parameter("agree_tol_m", 0.3)   # A normalization
        self.declare_parameter("vel_tol_ms", 0.3)    # S normalization
        self.declare_parameter("divergence_max_m", 0.5)
        self.declare_parameter("jump_max_m", 1.0)    # reset detector
        self.declare_parameter("seed_settle_secs", 0.5)
        # Fast-quality watchdog (in OPEN state): if pose covariance exceeds
        # cov_spike for cov_spike_frames consecutive frames, close the gate
        # immediately — before PX4 EKF fuses the bad pose.
        # At quality≈12, σ_position≈0.15m → cov[0]≈0.022 >> cov_spike=0.01.
        # At quality≈200, σ_position≈0.019m → cov[0]≈0.0004 << cov_spike.
        self.declare_parameter("cov_spike", 0.01)         # σ > 0.1m threshold
        self.declare_parameter("cov_spike_frames", 1)     # 1 = close on first spike
        g = self.get_parameter
        self._cov_bad   = g("cov_bad").value
        self._cov_norm  = g("cov_norm").value
        self._window_ns = int(g("window_secs").value * 1e9)
        self._agree_tol = g("agree_tol_m").value
        self._vel_tol   = g("vel_tol_ms").value
        self._div_max   = g("divergence_max_m").value
        self._jump_max  = g("jump_max_m").value
        self._seed_settle_ns = int(g("seed_settle_secs").value * 1e9)
        self._vision_interval_ns = int(1e9 / g("vision_rate_hz").value)
        self._cov_spike        = g("cov_spike").value
        self._cov_spike_frames = int(g("cov_spike_frames").value)

        # ── State ────────────────────────────────────────────────
        self._state = GateState.UNSEEDED
        self._cov   = float("inf")

        # Latest EKF data
        self._ekf_p = None            # (x, y, z)
        self._ekf_q = (0.0, 0.0, 0.0, 1.0)

        # Frame correction + anchors (set at seed / gate-open)
        self._q_corr        = None
        self._q_active      = None                  # rotation used when OPEN
        self._q_align       = (0.0, 0.0, 0.0, 1.0)  # orientation alignment
        self._p_rtab_anchor = None
        self._p_ekf_anchor  = None

        # Seed handshake
        self._seed_req_ns = 0

        # Sliding buffers for the initialization factor
        self._pair_buf = deque()      # (t_ns, p_rtab_raw, p_ekf)
        self._vel_buf  = deque()      # (t_ns, vx, vy)

        # Rotation auto-calibration (Kabsch fit over motion tracks).
        # The camera is mounted at an arbitrary rotation (downward+yaw);
        # a seed-attitude-only correction CANNOT map RTAB deltas to ENU.
        # Instead we fit the rotation directly from matched motion and
        # use the fit residual as the agreement score.
        self._fit_buf = deque(maxlen=300)   # (p_rtab, p_ekf) @ 5 Hz
        self._last_fit_sample_ns = 0
        self._q_fit = None                  # fitted RTAB→ENU rotation
        self._last_rtab_q = None

        # Reset detector / throttles
        self._last_rtab_p   = None
        self._last_vision_ns = 0
        self._pub_count = 0

        # Fast-quality watchdog counter (in OPEN state)
        self._cov_spike_count = 0

        # ── ROS interfaces ───────────────────────────────────────
        qos_rel, qos_be = qos_reliable(), qos_best_effort()

        self.create_subscription(
            Odometry, g("rtab_odom_topic").value, self._rtab_cb, qos_rel)
        self.create_subscription(
            PoseStamped, "/mavros/local_position/pose", self._ekf_cb, qos_be)
        self.create_subscription(
            TwistStamped, "/mavros/local_position/velocity_local",
            self._vel_cb, qos_be)

        self._vision_pub = self.create_publisher(
            PoseStamped, "/mavros/vision_pose/pose", qos_rel)
        self._if_pub    = self.create_publisher(Float32, "/viman/init_factor", 10)
        self._state_pub = self.create_publisher(UInt8, "/viman/vio_state", 10)

        self._reset_cli = self.create_client(Empty, g("reset_service").value)
        self.create_service(Trigger, "/viman/seed", self._seed_srv)
        self.create_service(SetBool,  "/viman/gate", self._gate_srv)

        # 10 Hz status/IF publisher
        self.create_timer(0.1, self._status_tick)

        self.get_logger().info(
            "VIO gate up — CLOSED. PX4 receives no vision data until the "
            "mission seeds and opens the gate.")

    # ════════════════════════════════════════════════════════════
    #  SERVICES
    # ════════════════════════════════════════════════════════════

    def _seed_srv(self, req, res):
        """Reset RTAB odometry, then capture frame alignment on the next
        good frame. Non-blocking: completion is signalled by state 2."""
        if not self._reset_cli.service_is_ready():
            res.success = False
            res.message = "RTAB reset service not ready"
            return res
        if self._ekf_p is None:
            res.success = False
            res.message = "No EKF pose yet"
            return res
        self._reset_cli.call_async(Empty.Request())
        self._seed_req_ns = self.get_clock().now().nanoseconds
        self._set_state(GateState.SEEDING)
        self._pair_buf.clear()
        self._fit_buf.clear()
        self._q_fit = None
        self._q_corr = None
        res.success = True
        res.message = "Seeding — watch /viman/vio_state for 2 (VALIDATING)"
        return res

    def _gate_srv(self, req, res):
        if req.data:   # open
            if self._state != GateState.VALIDATING or self._q_corr is None:
                res.success = False
                res.message = f"Cannot open from state {self._state.name}"
                return res
            if self._last_rtab_p is None:
                res.success = False
                res.message = "No RTAB pose available"
                return res
            # Prefer the motion-fitted rotation (correct for ANY camera
            # mount); seed-attitude correction only as fallback.
            self._q_active = self._q_fit if self._q_fit is not None \
                else self._q_corr
            if self._q_fit is not None:
                self.get_logger().info("Using motion-calibrated rotation ✓")
            else:
                self.get_logger().warn(
                    "No motion fit available — falling back to seed "
                    "attitude correction (mount errors NOT compensated)")
            # Orientation alignment so first published attitude ≡ EKF's:
            if self._last_rtab_q is not None:
                self._q_align = quat_normalize(quat_mult(
                    quat_conj(quat_mult(self._q_active, self._last_rtab_q)),
                    self._ekf_q))
            else:
                self._q_align = (0.0, 0.0, 0.0, 1.0)
            # Re-anchor at THIS instant: first published pose ≡ current EKF
            # estimate → zero innovation at handover, no transient.
            self._p_rtab_anchor = self._last_rtab_p
            self._p_ekf_anchor  = self._ekf_p
            self._set_state(GateState.OPEN)
            res.success = True
            res.message = "Gate OPEN — vision streaming to PX4"
        else:          # close
            self._set_state(
                GateState.VALIDATING if self._q_corr is not None
                else GateState.UNSEEDED)
            res.success = True
            res.message = "Gate closed"
        return res

    # ════════════════════════════════════════════════════════════
    #  CALLBACKS
    # ════════════════════════════════════════════════════════════

    def _ekf_cb(self, msg: PoseStamped):
        p = msg.pose.position
        self._ekf_p = (p.x, p.y, p.z)
        self._ekf_q = quat_from_msg(msg.pose.orientation)

    def _vel_cb(self, msg: TwistStamped):
        now_ns = self.get_clock().now().nanoseconds
        self._vel_buf.append((now_ns, msg.twist.linear.x, msg.twist.linear.y))
        cutoff = now_ns - self._window_ns
        while self._vel_buf and self._vel_buf[0][0] < cutoff:
            self._vel_buf.popleft()

    def _rtab_cb(self, msg: Odometry):
        self._cov = msg.pose.covariance[0]
        now_ns = self.get_clock().now().nanoseconds

        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        # NaN/Inf + quaternion-norm sanity
        norm_sq = q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w
        if not (0.9801 <= norm_sq <= 1.0201) or p.x != p.x:
            return
        rtab_p = (p.x, p.y, p.z)
        rtab_q = (q.x, q.y, q.z, q.w)
        self._last_rtab_q = rtab_q

        good = 0.0 < self._cov < self._cov_bad

        # ── SEEDING: capture alignment on first good post-reset frame ──
        if self._state == GateState.SEEDING:
            if good and (now_ns - self._seed_req_ns) > self._seed_settle_ns \
                    and self._ekf_p is not None:
                # Self-calibrating frame correction: one rotation maps
                # RTAB's odom frame into ENU, derived from the live
                # attitudes — absorbs camera mounting automatically.
                self._q_corr = quat_normalize(
                    quat_mult(self._ekf_q, quat_conj(rtab_q)))
                self._p_rtab_anchor = rtab_p
                self._p_ekf_anchor  = self._ekf_p
                self._last_rtab_p   = rtab_p
                self._set_state(GateState.VALIDATING)
                self.get_logger().info(
                    f"Seeded: anchors locked, q_corr captured "
                    f"(cov={self._cov:.2f})")
            return

        # ── Reset detector (VALIDATING and OPEN) ─────────────────
        if self._last_rtab_p is not None and self._state in (
                GateState.VALIDATING, GateState.OPEN):
            jump = self._dist(rtab_p, self._last_rtab_p)
            if jump > self._jump_max:
                self.get_logger().error(
                    f"RTAB pose jump {jump:.2f} m — odometry reset detected. "
                    "Gate closed; re-seed required.")
                self._set_state(GateState.FAULT_RESET)
                self._last_rtab_p = rtab_p
                self._fit_buf.clear()
                self._q_fit = None
                return
        self._last_rtab_p = rtab_p

        if self._state == GateState.VALIDATING:
            if good and self._ekf_p is not None:
                self._pair_buf.append((now_ns, rtab_p, self._ekf_p))
                cutoff = now_ns - self._window_ns
                while self._pair_buf and self._pair_buf[0][0] < cutoff:
                    self._pair_buf.popleft()
                # Rotation-fit track, decimated to 5 Hz:
                if now_ns - self._last_fit_sample_ns > 200_000_000:
                    self._last_fit_sample_ns = now_ns
                    self._fit_buf.append((rtab_p, self._ekf_p))
            return

        # ── OPEN: watchdogs + publish ────────────────────────────
        if self._state != GateState.OPEN:
            return

        if not good:
            self.get_logger().error(
                f"RTAB quality lost while open (cov={self._cov:.1f}) — "
                "gate closed.")
            self._cov_spike_count = 0
            self._set_state(GateState.FAULT_QUALITY)
            return

        # Fast-quality spike watchdog: close gate on cov_spike_frames
        # consecutive frames with σ_position > √cov_spike (default σ > 0.1m).
        # This fires before PX4 fuses the bad pose — faster than waiting for
        # position divergence to accumulate to divergence_max_m.
        if self._cov > self._cov_spike:
            self._cov_spike_count += 1
            if self._cov_spike_count >= self._cov_spike_frames:
                self.get_logger().error(
                    f"Pose covariance spike {self._cov:.4f} m² "
                    f"(σ={self._cov**0.5:.3f}m ≥ {self._cov_spike**0.5:.3f}m) "
                    f"for {self._cov_spike_count} frame(s) — gate closed.")
                self._cov_spike_count = 0
                self._set_state(GateState.FAULT_QUALITY)
                return
        else:
            self._cov_spike_count = 0

        # Corrected output pose (motion-calibrated rotation)
        d = (rtab_p[0] - self._p_rtab_anchor[0],
             rtab_p[1] - self._p_rtab_anchor[1],
             rtab_p[2] - self._p_rtab_anchor[2])
        rd = quat_rotate(self._q_active, d)
        out_p = (self._p_ekf_anchor[0] + rd[0],
                 self._p_ekf_anchor[1] + rd[1],
                 self._p_ekf_anchor[2] + rd[2])

        # Divergence watchdog: vision vs EKF inconsistency the EKF is
        # silently absorbing — catch it before it becomes a crash.
        if self._ekf_p is not None and \
                self._dist(out_p, self._ekf_p) > self._div_max:
            self.get_logger().error(
                f"Vision–EKF divergence "
                f"{self._dist(out_p, self._ekf_p):.2f} m — gate closed.")
            self._set_state(GateState.FAULT_DIVERGENCE)
            return

        # 30 Hz throttle
        if now_ns - self._last_vision_ns < self._vision_interval_ns:
            return
        self._last_vision_ns = now_ns

        oq = quat_normalize(quat_mult(quat_mult(self._q_active, rtab_q),
                                      self._q_align))
        out = PoseStamped()
        out.header.stamp    = msg.header.stamp   # RTAB stamp → EKF2_EV_DELAY
        out.header.frame_id = "map"
        out.pose.position.x, out.pose.position.y, out.pose.position.z = out_p
        (out.pose.orientation.x, out.pose.orientation.y,
         out.pose.orientation.z, out.pose.orientation.w) = oq
        self._vision_pub.publish(out)

        self._pub_count += 1
        if self._pub_count % 100 == 1:
            self.get_logger().info(
                f"Vision #{self._pub_count}: x={out_p[0]:.3f} "
                f"y={out_p[1]:.3f} z={out_p[2]:.3f} cov={self._cov:.2f}")

    # ════════════════════════════════════════════════════════════
    #  INITIALIZATION FACTOR  (10 Hz)
    # ════════════════════════════════════════════════════════════

    def _status_tick(self):
        m = UInt8()
        m.data = int(self._state)
        self._state_pub.publish(m)

        f = Float32()
        f.data = self._init_factor()
        self._if_pub.publish(f)

        # Per-component diagnostics — DEBUG so the terminal stays clean (RTAB
        # quality now lives in the detector's table). State CHANGES still log at
        # INFO below. Enable with --log-level vio_gate:=debug if you need this.
        if self._state in (GateState.VALIDATING, GateState.OPEN):
            q, a, s = self._last_qas
            self.get_logger().debug(
                f"IF={f.data:.2f}  [Q={q:.2f} A={a:.2f} S={s:.2f}]  "
                f"cov={self._cov:.4f}",
                throttle_duration_sec=2.0)

    _last_qas = (0.0, 0.0, 0.0)

    def _init_factor(self) -> float:
        self._last_qas = (0.0, 0.0, 0.0)
        if self._state not in (GateState.VALIDATING, GateState.OPEN) \
                or self._q_corr is None:
            return 0.0

        # Q — quality
        if not (0.0 < self._cov < self._cov_bad):
            return 0.0
        q_score = max(0.0, min(1.0, 1.0 - self._cov / self._cov_norm))

        # A — agreement via rotation auto-calibration (Kabsch).
        # With enough motion, fit the single rigid rotation that maps the
        # RTAB track onto the EKF track; the fit residual IS the score.
        # This self-calibrates ANY camera mount (downward, yawed, ...) —
        # a fixed seed-attitude correction cannot. With too little motion
        # (drone still), fall back to rotation-free magnitude comparison.
        a_score = 0.0
        fitted = self._try_fit_rotation()
        if fitted is not None:
            rms = fitted
            a_score = max(0.0, min(1.0, 1.0 - rms / self._agree_tol))
        else:
            if len(self._pair_buf) < 2:
                return 0.0
            t0, r0, e0 = self._pair_buf[0]
            t1, r1, e1 = self._pair_buf[-1]
            if t1 - t0 < self._window_ns * 0.5:
                return 0.0
            dr = self._dist(r1, r0)      # magnitudes are rotation-free
            de = self._dist(e1, e0)
            a_score = max(0.0, min(1.0, 1.0 - abs(dr - de) / self._agree_tol))

        # S — flow-hold stability (is the reference trustworthy?)
        s_score = 1.0
        n = len(self._vel_buf)
        if n >= 5:
            mx = sum(v[1] for v in self._vel_buf) / n
            my = sum(v[2] for v in self._vel_buf) / n
            var = sum((v[1] - mx) ** 2 + (v[2] - my) ** 2
                      for v in self._vel_buf) / n
            s_score = max(0.0, min(1.0, 1.0 - (var ** 0.5) / self._vel_tol))

        self._last_qas = (q_score, a_score, s_score)
        return q_score * a_score * s_score

    def _try_fit_rotation(self):
        """Kabsch: fit RTAB→ENU rotation from matched motion tracks.
        Returns residual RMS (m) and stores the rotation in _q_fit,
        or None if there isn't enough motion excitation to solve it."""
        if len(self._fit_buf) < 15:           # ≥3 s of track
            return None
        A = np.array([p[0] for p in self._fit_buf])   # RTAB
        B = np.array([p[1] for p in self._fit_buf])   # EKF
        Ac = A - A.mean(axis=0)
        Bc = B - B.mean(axis=0)
        # Excitation check: need motion in ≥2 directions, else the
        # rotation is unobservable and the fit would be garbage.
        s = np.linalg.svd(Ac, compute_uv=False)
        if s[1] < 0.05:                       # 2nd axis < 5 cm spread
            return None
        H = Ac.T @ Bc
        U, _, Vt = np.linalg.svd(H)
        d = np.sign(np.linalg.det(Vt.T @ U.T))
        D = np.diag([1.0, 1.0, d])            # enforce proper rotation
        R = Vt.T @ D @ U.T
        rms = float(np.sqrt(np.mean(
            np.sum((Ac @ R.T - Bc) ** 2, axis=1))))
        self._q_fit = _quat_from_matrix(R)
        return rms

    # ════════════════════════════════════════════════════════════

    def _set_state(self, s: GateState):
        if s != self._state:
            self.get_logger().info(f"Gate state: {self._state.name} → {s.name}")
            self._state = s

    @staticmethod
    def _dist(a, b) -> float:
        return ((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2
                + (a[2] - b[2]) ** 2) ** 0.5


def main():
    rclpy.init()
    node = VioGate()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Under `ros2 launch`, rclpy may already be shut down by the
        # launch SIGINT — a second shutdown raises RCLError. Tolerate it.
        try:
            node.destroy_node()
        except Exception:
            pass
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()
