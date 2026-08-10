#!/usr/bin/env python3
"""
survey_boundary_director — boundary-bounded lawnmower survey.
Team Viman Rakshak / IRoC-U 2026.

WHAT THIS IS (and what it is NOT)
─────────────────────────────────
This is a THIN COORDINATOR that runs IN PARALLEL with two existing,
UNCHANGED nodes and glues them together over ROS 2 topics:

  1. yellow_boundary_detector   (its own process — UNCHANGED)
        Publishes the yellow-line geometry every frame:
          /viman/boundary/nearest_m   std_msgs/Float32
          /viman/boundary/repulsion   geometry_msgs/Vector3Stamped (body-FLU)
          /viman/boundary/lines       std_msgs/Float32MultiArray
                                      [n, (dist,nx,ny,strength) x n], body-FLU,
                                      normals point AWAY from each line.

  2. survey_mission.SurveyMission (imported as a LIBRARY — UNCHANGED)
        The full, field-proven mission state machine
        (ARM / TAKEOFF / SEED / VALIDATE / HANDOVER / GOTO_HOME /
         SURVEY / RETURN / MARKER_LAND / LAND / DISARM).

This file does NOT copy either of those into one big script. It
SUBCLASSES SurveyMission so every pre-survey and post-survey phase is
reused EXACTLY as-is, and it OVERRIDES ONLY the SURVEY phase so the
lawnmower turns are driven by the yellow boundary instead of (only) the
hardcoded grid extents. It SUBSCRIBES to the detector's topics — the two
nodes run as separate processes and communicate over ROS.

HOW THE SURVEY IS DRIVEN NOW
────────────────────────────
Nominal motion is unchanged: the drone still crawls forward/backward
along the arm-time heading and steps sideways between stripes, exactly
like survey_mission. The ONLY additions are yellow-line overrides:

  • Crawling FORWARD  → the moment the FRONT line is seen inside
    [front_stop_lo_m, front_stop_hi_m] (default 0.5-1.0 m) the drone
    STOPS (never crosses the tape) and steps RIGHT to the next stripe.
  • Crawling BACKWARD → same, but triggered by the BACK line.
  • While stepping/traversing, if the RIGHT line appears inside
    [right_detect_lo_m, right_detect_hi_m] (default 0.5-1.5 m) the
    current column is flagged as the LAST one. The drone parks a safe
    stand-off from the right tape, does ONE full traversal alongside it
    (forward OR backward, whichever the lawnmower dictates), then ends
    the survey → normal RETURN → home → land.
  • If NO line is ever seen, each leg still terminates at the hardcoded
    grid extent (max_leg_m, default = survey_height_m), so behaviour
    degrades gracefully to the original survey.

SAFETY
──────
  • The drone stops as soon as a leading line reaches the UPPER edge of
    the stop band, so it never gets closer than ~stop_hi and never
    crosses the tape.
  • If the detector goes SILENT (no messages for boundary_stale_s) the
    drone HOLDS instead of crawling blindly toward a line it can no
    longer see.
  • A hard no-cross backstop (min_cross_m) freezes forward motion if any
    line is read closer than that.
  • RC CH5 interrupt, VIO-fault FLOW_HOLD recovery and Ctrl+C AUTO.LAND
    are all inherited untouched from SurveyMission.

RUN (two parallel processes — e.g. from your bringup launch)
────────────────────────────────────────────────────────────
  # terminal / launch node A — the detector (unchanged):
  ros2 run viman_mission yellow_boundary_detector \
      --ros-args --params-file <mission_params.yaml>

  # terminal / launch node B — THIS coordinator (reuses survey params):
  ros2 run viman_mission survey_boundary_director \
      --ros-args --params-file <mission_params.yaml>

NODE NAME / PARAMETERS
──────────────────────
Because it subclasses SurveyMission, this node is still named
"survey_mission", so it reads the EXISTING `survey_mission:` block in
mission_params.yaml and reuses all your survey tuning verbatim. The
extra boundary knobs below are declared with safe defaults; to tune
them, just add the `boundary_*` keys INTO the same `survey_mission:`
block (a ready-to-paste snippet ships alongside this file).

FIELD CHECKLIST (do these before flying)
────────────────────────────────────────
  [ ] Detector is up and the http/rqt debug view shows the tape masked
      cleanly at cruise altitude (retune HSV per arena if not).
  [ ] Bench-verify the detector's body-FLU sign: hold the drone over the
      tape, tape AHEAD must show as FRONT (push-away −x). If the red
      arrow points the wrong way, fix cam_yaw_offset_deg in the DETECTOR
      params (not here).
  [ ] Confirm which physical side is "right" for your arena and set
      `boundary_step_side` accordingly.
  [ ] Keep leg approach speed low (boundary_approach_speed_ms) — at 3 m
      altitude the camera only sees ~1.1-1.2 m ahead, so the drone must
      crawl to have reaction time before the stop band.
"""

import csv
import math
import os
from datetime import datetime

import cv2
import rclpy
from rclpy.executors import MultiThreadedExecutor

from geometry_msgs.msg import Vector3Stamped
from std_msgs.msg import Float32, Float32MultiArray
from std_srvs.srv import SetBool

from viman_mission.common import yaw_deg_from_quaternion

# Reuse the ENTIRE mission machine and its Phase enum, unchanged.
from viman_mission.survey_mission import SurveyMission, Phase, GS_FAULT_MIN


class BoundedSurveyMission(SurveyMission):
    """SurveyMission + yellow-boundary-driven lawnmower turns.

    Everything except the SURVEY phase is inherited untouched. We only:
      • declare a handful of boundary_* parameters,
      • subscribe to the yellow_boundary_detector topics,
      • override _begin_survey() to set up our reactive-leg state,
      • override _do_survey() with the boundary-bounded lawnmower.
    The Phase.SURVEY -> handler mapping in SurveyMission.__init__ stores
    `self._do_survey`, which on THIS instance resolves to our override —
    so no change to survey_mission.py is needed.
    """

    def __init__(self):
        # Builds the full node: params, subscriptions, publishers, the
        # 20 Hz loop timer and the phase-handler table (which will call
        # OUR _do_survey for Phase.SURVEY). Node name stays "survey_mission".
        super().__init__()

        # ── Boundary parameters (add these keys under `survey_mission:` in
        #    mission_params.yaml to tune; safe defaults otherwise) ────────
        bp = self.declare_parameters("", [
            # topics published by yellow_boundary_detector
            ("boundary_nearest_topic",   "/viman/boundary/nearest_m"),
            ("boundary_lines_topic",     "/viman/boundary/lines"),
            ("boundary_repulsion_topic", "/viman/boundary/repulsion"),
            # front/back STOP band — enter it and the leg ends (turn right)
            ("front_stop_lo_m",          0.5),
            ("front_stop_hi_m",          0.8),   # settle 0.5-0.8 m off the front
            ("back_stop_lo_m",           0.5),
            ("back_stop_hi_m",           0.8),   # settle 0.5-0.8 m off the back
            # right-line DETECT band — seeing the right line in here flags
            # the current column as the last one
            ("right_detect_lo_m",        0.5),
            ("right_detect_hi_m",        1.5),
            # when stepping toward the end boundary, park at least this far
            # from that tape (also: a leg running this close to the end line
            # is treated as already alongside it → final stripe)
            ("right_step_stop_m",        0.6),
            # WHICH CORNER the drone is placed at before arming. This decides
            # the lawnmower step direction and which side line ends the run:
            #   "back_left"  → near back+LEFT  line → step RIGHT, end at RIGHT
            #   "back_right" → near back+RIGHT line → step LEFT,  end at LEFT
            #   "auto"       → pick from whichever side line is nearer at the
            #                  survey start (needs that line in the camera FOV;
            #                  prefer an explicit corner — it's the safe choice)
            ("boundary_start_corner",    "back_left"),
            # low-level override (normally derived from boundary_start_corner):
            # "right" (body −Y) or "left" (body +Y)
            ("boundary_step_side",       "right"),
            # anti-thrash: a leg that ends on a line after moving less than
            # this counts as "no room"; that many in a row → RETURN (keeps a
            # degenerate/very-thin arena from ping-ponging forever)
            ("boundary_min_leg_m",       0.4),
            ("boundary_stuck_max",       3),
            # sideways distance per stripe (default: reuse survey col spacing)
            ("boundary_col_step_m",      -1.0),   # <0 → use col_spacing_m
            # fallback leg length if NO line is ever seen (default: survey_height)
            ("boundary_max_leg_m",       -1.0),   # <0 → use survey_height_m
            # capture a photo every this many metres along a leg
            ("boundary_capture_spacing_m", -1.0), # <0 → use stripe_spacing_m
            # gentle approach speed during legs/steps (reaction time!)
            ("boundary_approach_speed_ms", 0.20),
            # a line's unit normal must be at least this aligned with an axis
            # to count as front/back/left/right (rejects ~diagonal ambiguity)
            ("boundary_dir_dot",         0.5),
            # hard no-cross backstop: any line closer than this → freeze fwd
            ("boundary_min_cross_m",     0.35),
            # detector silent longer than this → HOLD (no blind motion)
            ("boundary_stale_s",         0.8),
            # safety cap on number of stripes before forcing RETURN
            ("boundary_max_columns",     30),
            # ── ALL-FOUR-SIDES no-cross standoff clamp (applied to EVERY
            #    survey setpoint). The drone may never lead within keep_dist of
            #    ANY visible yellow line — front, back, left OR right — and is
            #    eased back to keep_dist if it ends up inside. This is the
            #    "maintain 0.5-1.0 m, never cross" safety net; it does NOT
            #    replace the leg/step turn logic, it runs underneath it. ──
            ("boundary_keep_dist_m",     0.5),   # hard never-cross standoff [m]
            ("boundary_keep_band_hi_m",  1.0),   # far edge of the "in range" band [m]
            ("boundary_veto_deadband_m", 0.15),  # buffer/deadband — kills chatter [m]
            ("boundary_ease_speed_ms",   0.15),  # cap when easing back out [m/s]
            # ── END-line (final-stripe) CONFIDENCE GATE. A side-line reading
            #    only ends the survey if enough yellow is actually in view AND
            #    the reading holds for end_confirm_s. This rejects tiny stray-
            #    pixel blobs (e.g. 0.03 % / ~70 px) that used to fire a false
            #    "final stripe" and send the drone home after one column. ──
            ("boundary_coverage_topic",       "/viman/boundary/coverage_pct"),
            ("boundary_end_min_coverage_pct", 2.0),  # need >= this % yellow in view
            ("boundary_end_confirm_s",        0.6),  # held this long before it counts
            # ── Tapered FRONT/BACK end approach (reference: corner1
            #    _line_perp_approach_step). Decelerate from slow_dist down to a
            #    creep so the drone eases into the 0.5-0.8 m stop band, and
            #    confirm the in-band reading so a jumpy near-edge reading can't
            #    end the leg 2-2.5 m early. ──
            ("boundary_slow_dist_m",     1.5),   # start decelerating here [m]
            ("boundary_creep_speed_ms",  0.06),  # min approach speed at the band [m/s]
            ("boundary_stop_confirm_s",  0.4),   # leading line must hold in-band [s]
        ])
        (self._nearest_topic, self._lines_topic, self._rep_topic,
         self._front_lo, self._front_hi, self._back_lo, self._back_hi,
         self._right_lo, self._right_hi, self._right_step_stop,
         self._start_corner, self._step_side, self._min_leg, self._stuck_max,
         self._col_step_p, self._max_leg_p,
         self._cap_spacing_p, self._appr_speed, self._dir_dot,
         self._min_cross, self._bnd_stale_s,
         self._max_columns,
         self._keep_dist, self._keep_band_hi,
         self._veto_deadband, self._ease_speed,
         self._cov_topic, self._end_min_cov,
         self._end_confirm_s,
         self._slow_dist, self._creep_speed,
         self._stop_confirm_s) = (x.value for x in bp)

        # Resolve "<0 → inherit from survey params" defaults
        self._col_step = (self._col_spacing if self._col_step_p < 0
                          else self._col_step_p)
        self._max_leg = (self._survey_h if self._max_leg_p < 0
                         else self._max_leg_p)
        self._cap_spacing = (self._stripe_spacing if self._cap_spacing_p < 0
                             else self._cap_spacing_p)

        # Resolve the step direction from the start corner. The corner is the
        # authoritative control; boundary_step_side is only a manual fallback.
        corner = str(self._start_corner).lower()
        # Centre start: the drone is placed at the MIDDLE of the arena, not a
        # corner. Before surveying it must first FIND the back-left corner —
        # fly BACK to the back line, then slide LEFT along it until the LEFT
        # line is also in range (both 0.5-1.2 m = the corner), re-yaw on the
        # L, then survey exactly like a normal back_left start (step RIGHT,
        # end at the RIGHT line). See _do_acq_back / _do_acq_left / _do_acq_reyaw.
        self._find_first_corner = False
        self._corner_found      = False
        if corner == "back_right":
            self._step_side, self._corner_auto = "left", False
        elif corner == "back_left":
            self._step_side, self._corner_auto = "right", False
        elif corner == "center":
            self._step_side, self._corner_auto = "right", False
            self._find_first_corner = True      # run ACQ_BACK → ACQ_LEFT first
        else:                                   # "auto" — decided at survey start
            self._corner_auto = True
            # keep boundary_step_side as the provisional guess until detected

        # ── Live boundary data (updated by the callbacks below) ─────────
        self._bnd_nearest = -1.0          # metres, -1 = no yellow
        self._bnd_lines   = []            # list of {dist,nx,ny,strength}
        self._bnd_rep     = (0.0, 0.0)    # repulsion (body-FLU x,y)
        self._bnd_last_ns = 0             # last time ANY boundary msg arrived

        # ── Reactive-leg state (set in _begin_survey / used in _do_survey)
        self._bsub        = "LEG"         # "LEG" or "STEP"
        self._leg_dir     = 1             # +1 forward, -1 backward (body fwd axis)
        self._leg_anchor  = None          # (x,y) where the current leg started
        self._leg_target  = None          # far fallback target for this leg
        self._step_anchor = None          # (x,y) where the current step started
        self._step_target = None          # right-step target
        self._col_idx     = 0             # current stripe index
        self._last_col    = False         # end boundary reached → final leg
        self._next_cap_d  = 0.0           # next along-leg distance to capture at
        self._cp_count    = 0             # photos captured (own counter)
        self._stuck       = 0             # consecutive "no room" legs
        self._bnd_coverage   = 0.0        # % of frame that is yellow (0-100)
        self._end_line_since = None       # debounce timer for the END-line gate
        self._stop_since     = None       # confirm timer for the front/back stop
        self._ret_best       = None       # best (smallest) RETURN distance so far
        self._ret_stall_ts   = None       # time RETURN progress last improved
        # L-corner yaw averaging (fills every detection; used at HOVER_HOME lock)
        self._yaw_est_buf        = []      # [(t_ns, yaw_rad), ...]
        self._yaw_avg_window_s   = 2.5     # average estimates over this window [s]
        self._yaw_min_samples    = 6       # need >= this many AGREEING (inlier) samples
        self._yaw_max_spread_deg = 15.0    # an estimate within this of the mean = inlier
        # median filter on the leading front/back distance (rejects near-edge spikes)
        self._lead_buf       = []          # recent leading-line distances [m]
        self._lead_med_n     = 5           # median window size
        # FIRST/LAST sweep: follow the side boundary line holding a 0.5-1.2 m band
        # (corner1 _line_follow_step + _band_correction), so the arena edges are
        # fully covered. Only the first (start-side) and last (end-side) legs use it.
        self._follow_lo   = 0.5            # never closer than this to the side line [m]
        self._follow_hi   = 1.2            # ease back toward the line beyond this [m]
        self._follow_gain = 0.8            # perpendicular ease gain (dist outside band)
        self._follow_cap  = 0.15           # perpendicular ease speed cap [m/s]
        # A follow leg (first/last sweep) may NOT end on a front/back line until it
        # has traversed at least this far — so it can't bail out early when it just
        # drifts close to the side line; it completes the whole edge. The band
        # controller + no-cross clamp keep it safe (0.5-1.2 m) meanwhile.
        self._follow_min_traverse = 2.5    # [m]

        # ── Centre-start "find first corner" state (ACQ_BACK/LEFT/REYAW) ──
        # Only used when boundary_start_corner=="center". Fly BACK to the back
        # line, then LEFT along it to the back-left corner (both lines 0.5-1.2 m),
        # re-yaw on the L, then hand off to the normal back_left survey.
        self._acq_start        = None      # (x,y) where the current ACQ leg began
        self._acq_since        = None      # dwell timer for the in-band transition
        self._acq_lost_since   = None      # back-line-lost safety timer (ACQ_LEFT)
        self._acq_reyaw_target = None      # locked L-corner yaw for ACQ_REYAW [rad]
        self._acq_reyaw_start  = None      # ACQ_REYAW start time (refine timeout)
        self._acq_max_travel   = 8.0       # bail after this far with no back line [m]
        # Survey-frame ORIGIN in the real MAVROS frame. For a centre start this
        # is RE-ZEROED to the back-left corner at ACQ_REYAW lock, so the survey
        # reasons corner-relative (0,0 = corner). RETURN still targets the CENTRE
        # (=_home_x/_home_y, the arm point), NOT the corner. None = origin is home.
        self._survey_org       = None
        # Register the three centre-start handlers (base built _handlers already).
        self._handlers[Phase.ACQ_BACK]  = self._do_acq_back
        self._handlers[Phase.ACQ_LEFT]  = self._do_acq_left
        self._handlers[Phase.ACQ_REYAW] = self._do_acq_reyaw

        # ── Subscribe to the detector (runs as a separate parallel node) ─
        from viman_mission.common import qos_best_effort
        qos_be = qos_best_effort()
        self.create_subscription(Float32, self._nearest_topic,
                                 self._bnd_nearest_cb, qos_be)
        self.create_subscription(Float32MultiArray, self._lines_topic,
                                 self._bnd_lines_cb, qos_be)
        self.create_subscription(Vector3Stamped, self._rep_topic,
                                 self._bnd_rep_cb, qos_be)
        self.create_subscription(Float32, self._cov_topic,
                                 self._bnd_cov_cb, qos_be)

        self.get_logger().info(
            "BoundedSurveyMission active — lawnmower turns driven by yellow "
            f"boundary. start_corner='{corner}' → step '{self._step_side}', "
            f"end at the '{self._side_dir()}' line. "
            f"FRONT/BACK stop [{self._front_lo:.1f}-{self._front_hi:.1f}] m, "
            f"END-line detect ≤{self._right_hi:.1f} m, step {self._col_step:.2f} m, "
            f"fallback leg {self._max_leg:.1f} m."
            + ("  [auto: side decided at survey start]" if self._corner_auto else ""))

    # ─────────────────────────────────────────────────────────────
    # Boundary topic callbacks
    # ─────────────────────────────────────────────────────────────

    def _bnd_nearest_cb(self, m: Float32):
        self._bnd_nearest = float(m.data)
        self._bnd_last_ns = self.get_clock().now().nanoseconds

    def _bnd_rep_cb(self, m: Vector3Stamped):
        self._bnd_rep = (float(m.vector.x), float(m.vector.y))
        self._bnd_last_ns = self.get_clock().now().nanoseconds

    def _bnd_cov_cb(self, m: Float32):
        self._bnd_coverage = float(m.data)
        self._bnd_last_ns = self.get_clock().now().nanoseconds

    def _bnd_lines_cb(self, m: Float32MultiArray):
        d = list(m.data)
        lines = []
        if d:
            n = int(d[0])
            for i in range(n):
                b = 1 + 4 * i
                if b + 3 < len(d):
                    lines.append({'dist': d[b], 'nx': d[b + 1],
                                  'ny': d[b + 2], 'strength': d[b + 3]})
        self._bnd_lines = lines
        self._bnd_last_ns = self.get_clock().now().nanoseconds
        # Continuously buffer the per-frame L-corner yaw estimate so HOVER_HOME can
        # average it — before the arm yaw is locked AND during the yellow-refine
        # window (self._yaw_refining). Prune to a little over the averaging window.
        if self._hold_heading_q is None or getattr(self, "_yaw_refining", False):
            y = self._corner_yaw_estimate()
            if y is not None:
                now = self.get_clock().now().nanoseconds
                self._yaw_est_buf.append((now, y))
                cutoff = now - int((self._yaw_avg_window_s + 1.0) * 1e9)
                self._yaw_est_buf = [(t, v) for (t, v) in self._yaw_est_buf
                                     if t >= cutoff]

    # ─────────────────────────────────────────────────────────────
    # Boundary helpers
    # ─────────────────────────────────────────────────────────────

    def _bnd_stale(self) -> bool:
        if self._bnd_last_ns == 0:
            return True
        age = (self.get_clock().now().nanoseconds - self._bnd_last_ns) / 1e9
        return age > self._bnd_stale_s

    def _lines_or_fallback(self):
        """Per-line list, or a single synthesized line from repulsion+nearest.

        The detector publishes ≥1 line whenever yellow is in view (it falls
        back to the nearest-pixel line for thin/curved tape). If for any
        reason the lines array is empty but nearest≥0, rebuild one pseudo
        line from the repulsion vector (which points AWAY from the line)."""
        if self._bnd_lines:
            return self._bnd_lines
        if self._bnd_nearest is not None and self._bnd_nearest >= 0.0:
            rx, ry = self._bnd_rep
            n = math.hypot(rx, ry)
            if n > 1e-6:
                return [{'dist': self._bnd_nearest,
                         'nx': rx / n, 'ny': ry / n, 'strength': 0.0}]
        return []

    def _dist_to(self, direction: str):
        """Nearest perpendicular distance (m) to a line on the given side,
        or None. Side is decided by the line's push-away normal (body-FLU,
        x=forward y=left): a line AHEAD pushes back (nx<0); a line on the
        RIGHT pushes left (ny>0); BACK → nx>0; LEFT → ny<0."""
        best = None
        for L in self._lines_or_fallback():
            nx, ny, dist = L['nx'], L['ny'], L['dist']
            if direction == 'front':
                hit = nx < -self._dir_dot
            elif direction == 'back':
                hit = nx > self._dir_dot
            elif direction == 'right':
                hit = ny > self._dir_dot
            elif direction == 'left':
                hit = ny < -self._dir_dot
            else:
                hit = False
            if hit and (best is None or dist < best):
                best = dist
        return best

    def _end_line_dist(self):
        """CONFIDENT distance (m) to the END (step-side) boundary line, or None.

        A side-line reading only counts as the real end boundary once (a) enough
        yellow is actually in view (coverage >= end_min_cov) AND (b) it has held
        for end_confirm_s. This rejects tiny stray-pixel blobs (e.g. 0.03 % /
        ~70 px) that used to fire a false 'final stripe' and send the drone home
        after a single column. Only the final-stripe decision uses this gate;
        the hard no-cross clamp and the front/back stop still see every line."""
        end_d = self._dist_to(self._side_dir())
        if end_d is not None and self._bnd_coverage >= self._end_min_cov:
            if self._end_line_since is None:
                self._end_line_since = self.get_clock().now()
            if self._secs(self._end_line_since) >= self._end_confirm_s:
                return end_d
            return None
        self._end_line_since = None
        return None

    def _grid_axes(self):
        """Forward and step (right/left) unit vectors in ENU, from the
        arm-time survey heading held constant throughout the survey.

        ENU + CCW yaw:  forward = (cos, sin).
          body RIGHT  = 90° CW  from forward = ( sin yaw, -cos yaw)
          body LEFT   = 90° CCW from forward = (-sin yaw,  cos yaw)
        (body RIGHT is the same direction survey_mission._build_waypoints
        sweeps its columns — its vector is just named 'left_x/left_y' there.)
        This matches the yellow_boundary_detector body-FLU convention, so
        stepping 'right' here moves toward the detector's body-right line.
        """
        yaw = self._cmd_yaw_rad
        fwd   = (math.cos(yaw), math.sin(yaw))
        right = (math.sin(yaw), -math.cos(yaw))     # 90° CW  = body right
        left  = (-math.sin(yaw), math.cos(yaw))     # 90° CCW = body left
        step = right if self._step_side == "right" else left
        return fwd, step

    def _side_dir(self):
        return 'right' if self._step_side == "right" else 'left'

    def _first_side(self):
        """The boundary the FIRST leg runs along = the START side (opposite the
        step direction). For start_corner=back_right (step left) that's 'right'."""
        return 'left' if self._step_side == "right" else 'right'

    def _side_line_enu(self, side: str):
        """Nearest classified `side` ('left'/'right') boundary line as
        {ex, ey, dist} in ENU (push-away normal), or None if not seen."""
        yaw = self._cmd_yaw_rad
        if yaw is None:
            return None
        c, s = math.cos(yaw), math.sin(yaw)
        best = None
        for L in self._lines_or_fallback():
            nx, ny, dist = L['nx'], L['ny'], L['dist']
            if dist is None or dist < 0.0:
                continue
            # body-FLU classification: RIGHT line pushes +y? detector convention
            # (see _dist_to): right → ny > dir_dot, left → ny < -dir_dot.
            if side == 'right' and not (ny > self._dir_dot):
                continue
            if side == 'left' and not (ny < -self._dir_dot):
                continue
            n = math.hypot(nx, ny)
            if n < 1e-6:
                continue
            nx, ny = nx / n, ny / n
            enu = {'ex': nx * c - ny * s, 'ey': nx * s + ny * c, 'dist': dist}
            if best is None or dist < best['dist']:
                best = enu
        return best

    def _dir_line_enu(self, direction: str):
        """Nearest classified boundary line on `direction`
        ('front'/'back'/'left'/'right') as {ex, ey, dist} in ENU (push-away
        normal, unit), or None if not seen. Same classification as _dist_to,
        same ENU rotation as _side_line_enu — generalised to all four sides so
        the centre-start acquisition can target the BACK line and the LEFT line
        independently."""
        yaw = self._cmd_yaw_rad
        if yaw is None:
            return None
        c, s = math.cos(yaw), math.sin(yaw)
        best = None
        for L in self._lines_or_fallback():
            nx, ny, dist = L['nx'], L['ny'], L['dist']
            if dist is None or dist < 0.0:
                continue
            if direction == 'front':
                hit = nx < -self._dir_dot
            elif direction == 'back':
                hit = nx > self._dir_dot
            elif direction == 'right':
                hit = ny > self._dir_dot
            elif direction == 'left':
                hit = ny < -self._dir_dot
            else:
                hit = False
            if not hit:
                continue
            n = math.hypot(nx, ny)
            if n < 1e-6:
                continue
            nx, ny = nx / n, ny / n
            enu = {'ex': nx * c - ny * s, 'ey': nx * s + ny * c, 'dist': dist}
            if best is None or dist < best['dist']:
                best = enu
        return best

    def _band_perp(self, L):
        """Two-sided dead-band PERPENDICULAR setpoint delta (ENU, this tick) that
        eases the drone to hold line `L` inside [follow_lo, follow_hi] = 0.5-1.2 m
        (corner1 _band_correction). Beyond follow_hi → step TOWARD the line;
        closer than follow_lo → step AWAY; inside the band → hold (0,0)."""
        if L is None:
            return 0.0, 0.0
        ex, ey, dist = L['ex'], L['ey'], L['dist']
        dt = 1.0 / self._sp_rate_hz
        if dist > self._follow_hi:                       # too far → toward line
            v = min(self._follow_cap, self._follow_gain * (dist - self._follow_hi))
            return -ex * v * dt, -ey * v * dt
        if dist < self._follow_lo:                       # too close → away
            v = min(self._follow_cap, self._follow_gain * (self._follow_lo - dist))
            return ex * v * dt, ey * v * dt
        return 0.0, 0.0                                  # in band → hold

    def _line_in_band(self, L):
        """True iff line L is seen and inside the 0.5-1.2 m follow band."""
        return L is not None and self._follow_lo <= L['dist'] <= self._follow_hi

    def _line_in_range(self, L):
        """True iff line L is seen and within the follow band's OUTER edge
        (<= follow_hi). This is the SINGLE 'acquired' test for the centre-start
        corner: the all-four-sides no-cross clamp already stops the drone from
        getting closer than the keep distance (~0.5 m), so we only need the
        upper bound. Using the full 0.5-1.2 m band here made the corner latch
        flicker — at the clamp standoff the reading dips just under 0.5 m and
        the transition kept resetting, so the drone froze at the corner instead
        of re-yawing. One upper-bound check = no lower-edge jitter."""
        return L is not None and L['dist'] <= self._follow_hi

    def _follow_leg(self, ref_side: str, desired, speed: float) -> bool:
        """FOLLOW the `ref_side` boundary line: cruise TANGENT along it in the
        `desired` ENU direction at `speed`, while a two-sided DEAD-BAND holds the
        perpendicular distance inside [follow_lo, follow_hi] = 0.5-1.2 m (corner1
        _line_follow_step + _band_correction). The hard no-cross clamp still runs.
        Returns True if the side line was found and followed; False → caller falls
        back to the straight crawl."""
        L = self._side_line_enu(ref_side)
        if L is None:
            return False
        if self._sp is None:
            self._sp = [self._pose.pose.position.x, self._pose.pose.position.y]
        ex, ey, dist = L['ex'], L['ey'], L['dist']
        # tangent to the line, flipped to point the way we want to sweep
        tx, ty = -ey, ex
        if tx * desired[0] + ty * desired[1] < 0.0:
            tx, ty = -tx, -ty
        # two-sided dead-band perpendicular correction (quiet inside the band)
        if dist > self._follow_hi:                       # too far -> toward line
            pv = min(self._follow_cap, self._follow_gain * (dist - self._follow_hi))
            px, py = -ex * pv, -ey * pv
        elif dist < self._follow_lo:                     # too close -> away
            pv = min(self._follow_cap, self._follow_gain * (self._follow_lo - dist))
            px, py = ex * pv, ey * pv
        else:
            px, py = 0.0, 0.0                            # in band -> hold
        dt = 1.0 / self._sp_rate_hz
        self._sp[0] += (tx * speed + px) * dt
        self._sp[1] += (ty * speed + py) * dt
        self._apply_boundary()                           # never cross any line
        self._pub_sp(self._sp[0], self._sp[1], self._target_alt)
        return True

    def _boundary_lines_enu(self):
        """_lines_or_fallback() with each push-away normal rotated from the
        detector's body-FLU frame (x=forward, y=left) into ENU using the
        locked survey yaw, so the no-cross clamp works in the same world frame
        as the position setpoints. Returns [{ex, ey, dist}, ...] where (ex,ey)
        is the unit vector pointing AWAY from the line (toward the drone)."""
        yaw = self._cmd_yaw_rad
        if yaw is None:
            return []
        c, s = math.cos(yaw), math.sin(yaw)
        out = []
        for L in self._lines_or_fallback():
            nx, ny, dist = L['nx'], L['ny'], L['dist']
            if dist is None or dist < 0.0:
                continue
            n = math.hypot(nx, ny)
            if n < 1e-6:
                continue
            nx, ny = nx / n, ny / n
            # ENU = nx * body_forward + ny * body_left
            #   body_forward = (cos, sin), body_left = (-sin, cos)
            out.append({'ex': nx * c - ny * s,
                        'ey': nx * s + ny * c,
                        'dist': dist})
        return out

    def _apply_boundary(self):
        """ALL-FOUR-SIDES hard no-cross clamp on the advancing setpoint.

        For every visible yellow line (front/back/left/right) the setpoint is
        clamped so it can never lead within keep_dist of that line — the drone
        NEVER crosses a line, from any direction. If the drone is already inside
        keep_dist (tape often isn't seen until close), the setpoint is eased
        back OUT to keep_dist, capped at ease_speed and with a deadband so
        jumpy tape readings don't cause chatter. Sideways / backward motion away
        from a line always stays free, so the lawnmower can still turn. This is
        the survey analog of boundary_test_auto._apply_boundary()."""
        if self._sp is None:
            return
        px, py = self._pose.pose.position.x, self._pose.pose.position.y
        for L in self._boundary_lines_enu():
            ex, ey, dist = L['ex'], L['ey'], L['dist']
            lead_x = self._sp[0] - px
            lead_y = self._sp[1] - py
            toward = -(lead_x * ex + lead_y * ey)   # setpoint lead TOWARD line
            allow = dist - self._keep_dist
            if allow >= 0.0:
                # Outside the standoff: block any lead past it (never cross).
                if toward > allow:
                    excess = toward - allow
                    self._sp[0] += ex * excess
                    self._sp[1] += ey * excess
            else:
                # Inside the standoff: ease back out to keep_dist. Deadband +
                # speed cap keep it smooth on noisy readings.
                deficit = -allow
                if deficit > self._veto_deadband:
                    excess = max(toward, 0.0) + min(
                        deficit - self._veto_deadband,
                        self._ease_speed / self._sp_rate_hz)
                    self._sp[0] += ex * excess
                    self._sp[1] += ey * excess

    def _crawl_toward(self, tx, ty, speed):
        """Ramp the internal setpoint toward (tx,ty) at `speed`. Mirrors
        SurveyMission._crawl_to but with a caller-chosen speed. Returns the
        drone-to-target distance."""
        if self._sp is None:
            self._sp = [self._pose.pose.position.x,
                        self._pose.pose.position.y]
        stepd = speed / self._sp_rate_hz
        dx, dy = tx - self._sp[0], ty - self._sp[1]
        d = math.hypot(dx, dy)
        if d > 1e-6:
            f = min(1.0, stepd / d)
            self._sp[0] += dx * f
            self._sp[1] += dy * f
        # ALL-FOUR-SIDES no-cross clamp on the advancing setpoint (0.5-1.0 m
        # keep band); never lead within keep_dist of any line, from any side.
        self._apply_boundary()
        self._pub_sp(self._sp[0], self._sp[1], self._target_alt)
        return math.hypot(self._pose.pose.position.x - tx,
                          self._pose.pose.position.y - ty)

    def _hold_here(self):
        """Publish a hold at the current position (stop advancing)."""
        self._pub_sp(self._pose.pose.position.x,
                     self._pose.pose.position.y,
                     self._target_alt)

    def _rel_xy(self):
        """Current position in the SURVEY frame (corner-relative for a centre
        start, else absolute). Telemetry only — 0,0 = the survey origin."""
        ox, oy = self._survey_org if self._survey_org is not None else (0.0, 0.0)
        return (self._pose.pose.position.x - ox,
                self._pose.pose.position.y - oy)

    # ─────────────────────────────────────────────────────────────
    # Photo capture (own counter; matches the survey CSV schema)
    # ─────────────────────────────────────────────────────────────

    def _capture_point(self, col: int, seq: int):
        idx = self._cp_count
        img_name = f"cp{idx:04d}_c{col:02d}s{seq:02d}.jpg"
        img_path = os.path.join(self._survey_run_dir, img_name)
        saved = False
        if self._latest_image is not None:
            try:
                bgr = cv2.cvtColor(self._latest_image, cv2.COLOR_RGB2BGR)
                cv2.imwrite(img_path, bgr, [cv2.IMWRITE_JPEG_QUALITY, 95])
                saved = True
            except Exception as e:
                self.get_logger().error(f"imwrite cp{idx}: {e}")
                img_name = "WRITE_FAILED"
        else:
            img_name = "NO_FRAME"

        p = self._pose.pose.position
        yaw = yaw_deg_from_quaternion(self._pose.pose.orientation)
        ts = self.get_clock().now().nanoseconds / 1e9
        try:
            with open(self._log_path, 'a', newline='') as f:
                csv.writer(f).writerow([
                    idx, col, seq, f"{ts:.3f}",
                    f"{p.x:.4f}", f"{p.y:.4f}", f"{p.z:.4f}",
                    f"{yaw:.1f}", img_name])
        except Exception as e:
            self.get_logger().error(f"csv cp{idx}: {e}")

        self._cp_count += 1
        self._wpt_idx = self._cp_count       # keeps the DISARM summary sane
        self.get_logger().info(
            f"[cp {idx:04d}] col={col} seq={seq} "
            f"pos=({p.x:.2f},{p.y:.2f})  {'✓' if saved else '✗ no image'}")

    # ─────────────────────────────────────────────────────────────
    # OVERRIDE: survey initialisation (reactive legs instead of a grid)
    # ─────────────────────────────────────────────────────────────

    def _begin_survey(self):
        """Called by GOTO_HOME/HANDOVER. Reuses survey storage but sets up
        the reactive-leg state instead of a fixed waypoint grid. Idempotent
        across VIO-fault recovery (state is preserved on re-entry).

        CENTRE START: if boundary_start_corner=="center" and the corner has not
        been located yet, divert to the acquisition phases FIRST (ACQ_BACK →
        ACQ_LEFT → ACQ_REYAW). Those find the back-left corner and re-yaw on it,
        then call _begin_survey() again with _corner_found=True, which falls
        through here to the normal survey setup (step RIGHT, end at RIGHT line)."""
        if self._find_first_corner and not self._corner_found:
            self._sp            = None
            self._acq_start     = None
            self._acq_since     = None
            self._acq_lost_since = None
            self._phase         = Phase.ACQ_BACK
            self.get_logger().info(
                "CENTRE start — locating the back-left corner first: flying BACK "
                "to the back line, then LEFT along it until both lines are "
                f"{self._follow_lo:.1f}-{self._follow_hi:.1f} m (the corner).")
            return
        if not self._survey_initialized:
            self._waypoints = []             # not grid-driven anymore
            self._wpt_idx   = 0
            self._init_survey_storage()
            self._survey_initialized = True
            # first leg starts forward from the current (home) position
            self._col_idx    = 0
            self._leg_dir    = 1
            self._last_col   = False
            self._cp_count   = 0
            self._stuck      = 0
            # If start_corner=="auto", decide the step direction now from
            # whichever side line the camera can see nearer at the origin.
            # (Needs that line in the FOV — with an explicit corner this is
            # skipped and the configured side is used.)
            if self._corner_auto:
                dl = self._dist_to('left')
                dr = self._dist_to('right')
                if dr is not None and (dl is None or dr < dl):
                    self._step_side = "left"      # near RIGHT line → back_right
                    self.get_logger().info(
                        f"AUTO start corner: right line nearer ({dr:.2f} m) → "
                        "treating as BACK-RIGHT, stepping LEFT.")
                elif dl is not None:
                    self._step_side = "right"     # near LEFT line → back_left
                    self.get_logger().info(
                        f"AUTO start corner: left line nearer ({dl:.2f} m) → "
                        "treating as BACK-LEFT, stepping RIGHT.")
                else:
                    self._step_side = "right"
                    self.get_logger().warn(
                        "AUTO start corner: no side line visible at origin — "
                        "defaulting to BACK-LEFT (step RIGHT). Set "
                        "boundary_start_corner explicitly to be safe.")
            self._start_leg()
            fwd, step = self._grid_axes()
            self.get_logger().info(
                "Bounded survey initialised — first leg FORWARD from home, "
                f"stepping '{self._step_side}', end at '{self._side_dir()}' line.")
            self.get_logger().info(
                f"DIRECTION CHECK  yaw={math.degrees(self._cmd_yaw_rad):.1f}°  "
                f"forward=({fwd[0]:+.2f},{fwd[1]:+.2f}) ENU  "
                f"step '{self._step_side}'=({step[0]:+.2f},{step[1]:+.2f}) ENU. "
                "Watch the FIRST step: it must move the drone toward the OPEN "
                "side of the arena. If it moves toward the near boundary, the "
                "start corner is set wrong — flip boundary_start_corner.")
        else:
            self.get_logger().info(
                f"Bounded survey resuming — col {self._col_idx}, "
                f"{'FWD' if self._leg_dir > 0 else 'BCK'}, "
                f"sub={self._bsub}, captured {self._cp_count}.")
        self._sp            = None
        self._arrived_since = None
        self._settle_start  = None
        self._phase         = Phase.SURVEY

    def _start_leg(self):
        """Anchor a new forward/backward leg at the current position."""
        fwd, _ = self._grid_axes()
        ax = self._pose.pose.position.x
        ay = self._pose.pose.position.y
        self._leg_anchor = (ax, ay)
        self._leg_target = (ax + self._leg_dir * self._max_leg * fwd[0],
                            ay + self._leg_dir * self._max_leg * fwd[1])
        self._next_cap_d = 0.0               # capture at the leg start too
        self._stop_since = None              # fresh confirm timer for this leg
        self._lead_buf   = []                # fresh median filter for this leg
        self._sp = None
        self._bsub = "LEG"

    def _start_step(self):
        """Anchor a sideways step toward the next stripe."""
        _, step = self._grid_axes()
        ax = self._pose.pose.position.x
        ay = self._pose.pose.position.y
        self._step_anchor = (ax, ay)
        self._step_target = (ax + self._col_step * step[0],
                             ay + self._col_step * step[1])
        self._sp = None
        self._bsub = "STEP"

    # ─────────────────────────────────────────────────────────────
    # OVERRIDE: yaw at HOVER_HOME aligns to the yellow L-corner
    # ─────────────────────────────────────────────────────────────

    def _corner_yaw_estimate(self):
        """SINGLE-FRAME L-corner yaw estimate [rad], or None. Builds ENU normals
        from the current pose yaw (the frame the detector normals are in), finds a
        perpendicular pair, and picks the BACK line using the ARM-TIME heading as
        'into the arena' — using the arm heading (not the drifting live yaw) keeps
        the back/side choice correct well past 45° of drift. FORWARD = back normal
        = parallel to the side line."""
        if self._bnd_stale():
            return None
        raw = self._lines_or_fallback()          # body-FLU normals {dist,nx,ny}
        if len(raw) < 2:
            return None
        cur = math.radians(yaw_deg_from_quaternion(self._pose.pose.orientation))
        c, s = math.cos(cur), math.sin(cur)
        enu = []
        for L in raw:
            nx, ny = L['nx'], L['ny']
            n = math.hypot(nx, ny)
            if n < 1e-6:
                continue
            nx, ny = nx / n, ny / n
            enu.append((nx * c - ny * s, nx * s + ny * c))   # push-away, ENU
        if len(enu) < 2:
            return None
        pair = None
        for i in range(len(enu)):
            for j in range(i + 1, len(enu)):
                if abs(enu[i][0] * enu[j][0] + enu[i][1] * enu[j][1]) < 0.5:
                    pair = (enu[i], enu[j])
                    break
            if pair:
                break
        if pair is None:
            return None
        a, b = pair
        if self._arm_heading_q is not None:
            ref = math.radians(yaw_deg_from_quaternion(self._arm_heading_q))
        else:
            ref = cur
        rfx, rfy = math.cos(ref), math.sin(ref)
        back = a if (a[0] * rfx + a[1] * rfy) >= (b[0] * rfx + b[1] * rfy) else b
        return math.atan2(back[1], back[0])

    def _yaw_align_override(self):
        """L-corner yaw for HOVER_HOME: AVERAGE the per-frame estimates collected
        over the last yaw_avg_window_s (filled every detection in _bnd_lines_cb)
        and return the circular mean — but only if enough samples agree closely.
        Returns None if too few samples or they're too scattered (ambiguous/noisy),
        so the base cleanly falls back to the arm-time auto-yaw. Averaging in place
        removes the single-frame noise that caused the occasional crooked lock, and
        needs no movement, so there's no flow drift before the seed."""
        now = self.get_clock().now().nanoseconds
        win = self._yaw_avg_window_s * 1e9
        ys = [y for (t, y) in self._yaw_est_buf if now - t <= win]
        if len(ys) < self._yaw_min_samples:
            return None
        # Circular mean, then keep only the INLIERS within tol of it. Rejecting a
        # few jittery frames (instead of failing the whole window on one bad
        # sample) lets a stable-enough L-corner actually lock — so the first
        # survey sweep flies straight along the arena instead of the residual
        # arm-heading skew. A genuinely scattered / bimodal set still yields too
        # few inliers → None → clean fall back to the arm-time yaw.
        mean = math.atan2(sum(math.sin(y) for y in ys),
                          sum(math.cos(y) for y in ys))
        tol = math.radians(self._yaw_max_spread_deg)
        inl = [y for y in ys
               if abs((y - mean + math.pi) % (2 * math.pi) - math.pi) <= tol]
        if len(inl) < self._yaw_min_samples:
            self.get_logger().warn(
                f"Yaw L-corner too scattered ({len(inl)}/{len(ys)} within "
                f"{self._yaw_max_spread_deg:.0f}°) — using arm-time heading instead.")
            return None
        m2 = math.atan2(sum(math.sin(y) for y in inl),
                        sum(math.cos(y) for y in inl))
        self.get_logger().info(
            f"Yaw: yellow L-corner locked from {len(inl)}/{len(ys)} inlier samples "
            f"— FORWARD parallel to the side line at {math.degrees(m2):.1f} deg.")
        return m2

    def _yaw_refine_enabled(self):
        """Yes — after the arm-time yaw lock, run the yellow-line refine window."""
        return True

    # ─────────────────────────────────────────────────────────────
    # OVERRIDE: RETURN home WITH the all-four-sides no-cross clamp
    # ─────────────────────────────────────────────────────────────

    def _do_return(self):
        """Return home, but never cross a yellow line doing it: motion goes
        through _crawl_toward, which applies the 0.5 m no-cross clamp on every
        side. If the clamp parks the drone at the standoff (progress stalls),
        land there rather than hovering out the whole timeout."""
        if self._vio_state >= GS_FAULT_MIN:
            self.get_logger().warn("VIO fault during return — landing here on flow")
            self._phase = Phase.LAND
            return
        if self._goto_start is None:
            self._goto_start   = self.get_clock().now()
            self._ret_best     = None
            self._ret_stall_ts = None
        if self._secs(self._goto_start) > self._goto_timeout:
            self.get_logger().warn("Return timeout — landing on flow")
            self._phase = Phase.LAND
            return

        self._crawl_toward(self._home_x, self._home_y, self._appr_speed)
        dist = math.hypot(self._pose.pose.position.x - self._home_x,
                          self._pose.pose.position.y - self._home_y)

        # The clamp may hold us at a standoff short of home. Track progress;
        # if it stalls for a few seconds, treat it as arrived and land there.
        stalled = False
        if self._ret_best is None or dist < self._ret_best - 0.05:
            self._ret_best     = dist
            self._ret_stall_ts = self.get_clock().now()
        elif (self._ret_stall_ts is not None
              and self._secs(self._ret_stall_ts) >= 3.0):
            stalled = True

        self._tele(f"RETURN dist={dist:.2f} m -> home"
                   + ("  [clamped standoff - landing here]" if stalled else ""))

        if dist <= self._goto_radius or stalled:
            if self._arrived_since is None:
                self._arrived_since = self.get_clock().now()
            elif self._secs(self._arrived_since) >= 1.0:
                if self._gate_cli.service_is_ready():
                    req = SetBool.Request()
                    req.data = False
                    self._gate_cli.call_async(req)
                self._gate_close_sent = True
                self._flow_settle_ts  = self.get_clock().now()
                self.get_logger().info("Home reached — gate closed, flow settle")
                self._arrived_since = None
                self._goto_start    = None
                self._phase         = Phase.FLOW_SETTLE
        else:
            self._arrived_since = None

    # ─────────────────────────────────────────────────────────────
    # CENTRE START: acquire the back-left corner before surveying
    #   ACQ_BACK  → fly BACK to the back line, ease into the 0.5-1.2 m band
    #   ACQ_LEFT  → slide LEFT along the back line until the LEFT line is
    #               also in band (both in band = the back-left corner)
    #   ACQ_REYAW → re-yaw on the L-corner, re-lock the grid, start survey
    # (ported from corner1_test_auto ACQ_BACK / ACQ_LEFT, reusing the
    #  director's own ENU line helpers + no-cross clamp.)
    # ─────────────────────────────────────────────────────────────

    def _do_acq_back(self):
        """Phase 1 — fly BACKWARD from the arena centre until the BACK yellow
        line is seen, then perpendicular-approach until it sits in the
        0.5-1.2 m band. Only the BACK line is a target here (front/left/right
        are ignored); the all-four-sides no-cross clamp still keeps every line
        safe."""
        if self._vio_state >= GS_FAULT_MIN:
            self.get_logger().warn("VIO fault during corner acquisition — flow hold.")
            self._flow_hold_start = None
            self._phase = Phase.FLOW_HOLD
            return
        if self._sp is None:
            self._sp = [self._pose.pose.position.x, self._pose.pose.position.y]
        if self._acq_start is None:
            self._acq_start = (self._pose.pose.position.x,
                               self._pose.pose.position.y)

        fwd, _ = self._grid_axes()
        dt = 1.0 / self._sp_rate_hz
        back = None if self._bnd_stale() else self._dir_line_enu('back')

        if back is None:
            # OPEN cruise: advance the setpoint straight BACKWARD (locked −fwd)
            self._sp[0] -= fwd[0] * self._appr_speed * dt
            self._sp[1] -= fwd[1] * self._appr_speed * dt
            mode, bstr = "SEARCH-BACK", "back=--"
        else:
            # FOLLOW-PERP: ease straight onto the back line into the band
            dx, dy = self._band_perp(back)
            self._sp[0] += dx
            self._sp[1] += dy
            mode, bstr = "APPROACH-BACK", f"back={back['dist']:.2f} m"

        self._apply_boundary()                       # never cross any line
        self._pub_sp(self._sp[0], self._sp[1], self._target_alt)

        travelled = -((self._pose.pose.position.x - self._acq_start[0]) * fwd[0]
                      + (self._pose.pose.position.y - self._acq_start[1]) * fwd[1])
        self._tele(f"ACQ_BACK[{mode}] trav={travelled:.2f}/"
                   f"{self._acq_max_travel:.1f} m  {bstr}")

        # transition: back line WITHIN RANGE (<= 1.2 m) → start sliding LEFT.
        # Single upper-bound test (no lower-edge jitter) + a short 0.4 s debounce.
        if self._line_in_range(back):
            if self._acq_since is None:
                self._acq_since = self.get_clock().now()
            elif self._secs(self._acq_since) >= 0.4:
                self.get_logger().info(
                    f"Back line acquired ({back['dist']:.2f} m) — sliding LEFT "
                    "along it to find the back-left corner.")
                self._sp             = None
                self._acq_start      = None
                self._acq_since      = None
                self._acq_lost_since = None
                self._phase          = Phase.ACQ_LEFT
        else:
            self._acq_since = None

        # bail-out: flew the whole way with no back line → survey from here
        if back is None and travelled > self._acq_max_travel:
            self.get_logger().warn(
                f"No back line within {self._acq_max_travel:.1f} m — starting "
                "survey from here as back_left (centre acquisition gave up).")
            self._corner_found = True
            self._sp           = None
            self._acq_start    = None
            self._begin_survey()

    def _do_acq_left(self):
        """Phase 2 — slide LEFT along the back line, holding it in the
        0.5-1.2 m band, until the LEFT line is ALSO in that band. Both lines
        in band = the back-left corner → re-yaw. Motion = tangent-left along
        the back line + dead-band perpendicular corrections on whichever line(s)
        are out of band (corner1 _do_acq_left)."""
        if self._vio_state >= GS_FAULT_MIN:
            self.get_logger().warn("VIO fault during corner acquisition — flow hold.")
            self._flow_hold_start = None
            self._phase = Phase.FLOW_HOLD
            return
        if self._sp is None:
            self._sp = [self._pose.pose.position.x, self._pose.pose.position.y]

        dt = 1.0 / self._sp_rate_hz
        yaw = self._cmd_yaw_rad
        lx, ly = -math.sin(yaw), math.cos(yaw)       # locked body-LEFT in ENU
        stale = self._bnd_stale()
        back = None if stale else self._dir_line_enu('back')
        left = None if stale else self._dir_line_enu('left')
        # 'In range' = seen and <= 1.2 m. This is the ONLY corner test (the
        # no-cross clamp handles the near side), so it can't jitter at the
        # 0.5 m edge the way the full band did.
        back_in = self._line_in_range(back)
        left_in = self._line_in_range(left)

        if back is None:
            # Lost the back line while sliding — HOLD and wait for it to return;
            # if it stays lost, fall back to acquiring it again.
            self._pub_sp(self._sp[0], self._sp[1], self._target_alt)
            if self._acq_lost_since is None:
                self._acq_lost_since = self.get_clock().now()
            self._tele("ACQ_LEFT[HOLD] back line lost — waiting")
            if self._secs(self._acq_lost_since) >= 3.0:
                self.get_logger().warn(
                    "Back line lost > 3 s — re-acquiring it (ACQ_BACK).")
                self._sp = None
                self._acq_start = None
                self._acq_since = None
                self._acq_lost_since = None
                self._phase = Phase.ACQ_BACK
            return
        self._acq_lost_since = None

        corner = back_in and left_in
        if corner:
            mode = "CORNER"
            dxb, dyb = self._band_perp(back)
            dxl, dyl = self._band_perp(left)
            self._sp[0] += dxb + dxl
            self._sp[1] += dyb + dyl
        elif left is not None and not left_in:
            # LEFT line seen but out of band → ease onto it while holding back
            mode = "SEEK-LEFT-BAND"
            dxb, dyb = self._band_perp(back)
            dxl, dyl = self._band_perp(left)
            # still creep left a little so we keep closing on the corner
            self._sp[0] += lx * self._appr_speed * 0.4 * dt + dxb + dxl
            self._sp[1] += ly * self._appr_speed * 0.4 * dt + dyb + dyl
        else:
            # SLIDE-LEFT: cruise tangent-left along the back line, hold its band
            mode = "SLIDE-LEFT"
            ex, ey = back['ex'], back['ey']
            tx, ty = -ey, ex                         # tangent to the back line
            if tx * lx + ty * ly < 0.0:              # point it body-LEFT
                tx, ty = -tx, -ty
            dxb, dyb = self._band_perp(back)
            self._sp[0] += tx * self._appr_speed * dt + dxb
            self._sp[1] += ty * self._appr_speed * dt + dyb

        self._apply_boundary()
        self._pub_sp(self._sp[0], self._sp[1], self._target_alt)
        bd = f"{back['dist']:.2f}" if back else "--"
        ld = f"{left['dist']:.2f}" if left else "--"
        self._tele(f"ACQ_LEFT[{mode}] back={bd} m left={ld} m")

        # transition: BOTH lines within range (<= 1.2 m) → the corner → re-yaw.
        # Single range test + a short 0.4 s debounce, so the drone doesn't stall
        # at the corner waiting on a strict band that flickers.
        if corner:
            if self._acq_since is None:
                self._acq_since = self.get_clock().now()
            elif self._secs(self._acq_since) >= 0.4:
                self.get_logger().info(
                    f"Back-left CORNER reached (back {back['dist']:.2f} m, "
                    f"left {left['dist']:.2f} m) — re-yawing on the L.")
                self._acq_since        = None
                self._acq_reyaw_target = None
                self._acq_reyaw_start  = self.get_clock().now()
                self._phase            = Phase.ACQ_REYAW
        else:
            self._acq_since = None

    def _do_acq_reyaw(self):
        """Phase 3 — at the corner, re-run the L-corner yaw estimate, slew onto
        it and RE-LOCK the grid (same align+relock scheme as HOVER_HOME), then
        hand off to the normal back_left survey. Holds position at the corner
        throughout. If no stable L appears in the refine window, keeps the
        current yaw."""
        if self._vio_state >= GS_FAULT_MIN:
            self.get_logger().warn("VIO fault during corner re-yaw — flow hold.")
            self._flow_hold_start = None
            self._phase = Phase.FLOW_HOLD
            return
        if self._sp is None:
            self._sp = [self._pose.pose.position.x, self._pose.pose.position.y]
        # keep collecting L-corner samples while we hold at the corner
        self._yaw_refining = True

        # ── Phase A: pick the target yaw (stable L-corner, else current) ──
        if self._acq_reyaw_target is None:
            ovr = self._yaw_align_override()
            if ovr is not None:
                self._acq_reyaw_target  = ovr
                self._hold_heading_q    = self._yaw_quat(ovr)
                self._yaw_align_start   = self.get_clock().now()
                self._yaw_aligned_since = None
                self.get_logger().info(
                    f"Corner L-yaw found ({math.degrees(ovr):.1f}°) — slewing "
                    "onto it and re-locking the grid.")
            elif self._secs(self._acq_reyaw_start) > self._yaw_refine_window_s:
                cur = math.radians(
                    yaw_deg_from_quaternion(self._pose.pose.orientation))
                self._acq_reyaw_target  = cur
                self._hold_heading_q    = self._yaw_quat(cur)
                self._yaw_align_start   = self.get_clock().now()
                self._yaw_aligned_since = None
                self.get_logger().info(
                    f"No stable L-corner in {self._yaw_refine_window_s:.0f} s — "
                    f"keeping current yaw {math.degrees(cur):.1f}°.")
            else:
                self._pub_sp(self._sp[0], self._sp[1], self._target_alt)
                self._tele("ACQ_REYAW  watching for L-corner "
                           f"({self._secs(self._acq_reyaw_start):.1f}/"
                           f"{self._yaw_refine_window_s:.0f} s)")
            return

        # ── Phase B: slew onto the target, wait aligned, RE-LOCK, survey ──
        self._pub_sp(self._sp[0], self._sp[1], self._target_alt)   # slews cmd_yaw
        cur = yaw_deg_from_quaternion(self._pose.pose.orientation)
        tgt = yaw_deg_from_quaternion(self._hold_heading_q)
        err = abs((cur - tgt + 180.0) % 360.0 - 180.0)
        self._tele(f"ACQ_REYAW  yaw={cur:.1f} → {tgt:.1f}° (err={err:.1f}°)")

        if err <= self._yaw_align_tol_deg:
            if self._yaw_aligned_since is None:
                self._yaw_aligned_since = self.get_clock().now()
        else:
            self._yaw_aligned_since = None
        aligned = (self._yaw_aligned_since is not None and
                   self._secs(self._yaw_aligned_since) >= self._yaw_align_hold_s)
        timeout = self._secs(self._yaw_align_start) > self._yaw_align_timeout_s
        if aligned or timeout:
            actual = math.radians(cur)
            self._cmd_yaw_rad    = actual
            self._hold_heading_q = self._yaw_quat(actual)   # target==cmd → no drift
            self._yaw_refining   = False
            self._corner_found = True
            # RE-ZERO the survey frame here: the corner is the survey origin
            # (0,0). RETURN still goes to the CENTRE (home), computed via this
            # offset, so the drone comes back to the middle — not the corner.
            self._survey_org = (self._pose.pose.position.x,
                                self._pose.pose.position.y)
            self.get_logger().info(
                f"Grid re-locked at the corner ({cur:.1f}°, "
                f"{'aligned' if aligned else 'timeout'}). Survey origin re-zeroed "
                f"to the corner ({self._survey_org[0]:.2f}, {self._survey_org[1]:.2f}) "
                f"— corner-relative survey; RETURN targets CENTRE/home "
                f"({self._home_x:.2f}, {self._home_y:.2f}). Step RIGHT, end at the "
                "RIGHT line.")
            self._sp           = None
            self._begin_survey()

    # ─────────────────────────────────────────────────────────────
    # OVERRIDE: the boundary-bounded lawnmower
    # ─────────────────────────────────────────────────────────────

    def _do_survey(self):
        # VIO fault handling — identical policy to the base survey: bank the
        # leg state and drop to FLOW_HOLD; we resume on the next HANDOVER.
        if self._vio_state >= GS_FAULT_MIN:
            self.get_logger().error(
                f"VIO fault during bounded survey (col {self._col_idx}, "
                f"{'FWD' if self._leg_dir > 0 else 'BCK'}) — flow hold. "
                "Will resume after revalidation.")
            self._flow_hold_start = None
            self._phase = Phase.FLOW_HOLD
            return

        stale = self._bnd_stale()

        # NOTE: the "this is the final stripe" decision is made inside the
        # sub-states — during a STEP (we're approaching the end boundary) or
        # when a LEG is already running right alongside it. Detecting it only
        # there (not on every tick) avoids ending early if a leg merely
        # glimpses the far line, and it fires even when the line reads < the
        # detect-band floor (e.g. 0.0 m right over the tape).
        if self._bsub == "LEG":
            self._do_leg(stale)
        else:
            self._do_step(stale)

    # ── Sub-state: crawl a stripe forward/backward ───────────────
    def _do_leg(self, stale: bool):
        fwd, _ = self._grid_axes()
        leading = 'front' if self._leg_dir > 0 else 'back'
        lo = self._front_lo if self._leg_dir > 0 else self._back_lo
        hi = self._front_hi if self._leg_dir > 0 else self._back_hi

        # If this leg is already running right alongside the END boundary
        # (the side we step toward), mark it the final stripe. This catches the
        # case where a prior step parked us next to the end line. NOT on col 0:
        # the FIRST stripe runs along the START-side line (the opposite edge),
        # so at the start corner one of the nearby lines (start/back tape the
        # drone is sitting on) can be misread as the far 'end' boundary and end
        # the survey after a single column. The end boundary is only reached by
        # STEPPING across the arena, so allow this detection from col 1 onward
        # (a genuine one-column arena still terminates via the step logic).
        if not stale and not self._last_col and self._col_idx > 0:
            end_d = self._end_line_dist()   # confidence-gated (coverage + hold)
            if end_d is not None and end_d <= self._right_step_stop:
                self._last_col = True
                self.get_logger().info(
                    f"Running alongside the '{self._side_dir()}' boundary "
                    f"({end_d:.2f} m, {self._bnd_coverage:.1f}% yellow) — "
                    "this is the FINAL stripe.")

        # Leading front/back distance, MEDIAN-FILTERED over the last few frames so
        # a jumpy near-edge reading (the tape's near edge reads closer than its
        # centre) can't taper/stop the leg early. None clears the filter.
        raw_lead = self._dist_to(leading)
        if raw_lead is None:
            self._lead_buf = []
            near_lead = None
        else:
            self._lead_buf.append(raw_lead)
            if len(self._lead_buf) > self._lead_med_n:
                self._lead_buf.pop(0)
            near_lead = sorted(self._lead_buf)[len(self._lead_buf) // 2]

        # Distance travelled on this leg (from its anchor).
        ax, ay = self._leg_anchor
        travelled = math.hypot(self._pose.pose.position.x - ax,
                               self._pose.pose.position.y - ay)
        # A FOLLOW leg (first / last sweep) may NOT end on a front/back line until
        # it has covered the edge (follow_min_traverse). Before that, the band
        # controller + no-cross clamp keep it safe, so drifting close to the side
        # line it's following can't bail the sweep out early — it completes the
        # whole edge and only stops at the FAR corner.
        is_follow = (self._col_idx == 0 or self._last_col)
        allow_end = (not is_follow) or (travelled >= self._follow_min_traverse)

        # Hard no-cross backstop on the leading line.
        if (allow_end and not stale and near_lead is not None
                and near_lead < self._min_cross):
            self._hold_here()
            self._tele(f"LEG col{self._col_idx} "
                       f"{'FWD' if self._leg_dir > 0 else 'BCK'} — "
                       f"HARD STOP, {leading} line {near_lead:.2f} m")
            self._end_leg(via_line=True)
            return

        # Primary stop: leading line CONFIRMED inside the stop band → turn.
        # We do NOT end on the first sub-hi reading — a jumpy near-edge reading
        # at cruise altitude used to turn the drone 2-2.5 m early. Require the
        # in-band reading to hold for stop_confirm_s; meanwhile the tapered
        # creep below keeps easing the drone into the 0.5-0.8 m band.
        if allow_end and not stale and near_lead is not None and near_lead <= hi:
            if self._stop_since is None:
                self._stop_since = self.get_clock().now()
            if self._secs(self._stop_since) >= self._stop_confirm_s:
                self._hold_here()
                self.get_logger().info(
                    f"{leading.upper()} line at {near_lead:.2f} m "
                    f"(band {lo:.1f}-{hi:.1f}, held {self._stop_confirm_s:.1f}s) "
                    f"— stopping, will step {self._step_side}.")
                self._end_leg(via_line=True)
                return
        else:
            self._stop_since = None

        # If the detector is blind, HOLD rather than crawl toward a line we
        # can't see. (Never advance across a boundary on stale data.)
        if stale:
            self._hold_here()
            self._tele(f"LEG col{self._col_idx} — detector STALE, holding")
            return

        # Fallback termination: reached the hardcoded grid extent with no
        # line in sight (keeps the mission finite even without tape).
        if travelled >= self._max_leg:
            self._hold_here()
            self.get_logger().info(
                f"Leg reached fallback extent {self._max_leg:.1f} m with no "
                f"{leading} line — stepping {self._step_side}.")
            self._end_leg(via_line=False)
            return

        # Tangent cruise speed, TAPERED once the leading line is within slow_dist
        # so the drone decelerates from ~1.5 m and eases into the stop band.
        speed = self._appr_speed
        if near_lead is not None and near_lead <= self._slow_dist:
            span = max(1e-3, self._slow_dist - lo)
            frac = max(0.0, min(1.0, (near_lead - lo) / span))
            speed = self._creep_speed + (self._appr_speed - self._creep_speed) * frac

        # FIRST sweep (col 0 → hug the START-side line) and LAST sweep (final
        # stripe → hug the END-side line) FOLLOW that boundary holding the
        # 0.5-1.2 m band, so the arena edges are fully covered. Middle stripes,
        # or if the side line isn't visible, use the straight fallback crawl.
        followed = False
        if self._col_idx == 0 or self._last_col:
            fwd, _ = self._grid_axes()
            ref_side = self._first_side() if self._col_idx == 0 else self._side_dir()
            desired  = (self._leg_dir * fwd[0], self._leg_dir * fwd[1])
            followed = self._follow_leg(ref_side, desired, speed)
        if not followed:
            self._crawl_toward(self._leg_target[0], self._leg_target[1], speed)

        if travelled >= self._next_cap_d:
            self._capture_point(self._col_idx, int(travelled / max(
                self._cap_spacing, 1e-3)))
            self._next_cap_d += self._cap_spacing

        ld = f"{near_lead:.2f}" if near_lead is not None else "—"
        if followed:
            fs = self._first_side() if self._col_idx == 0 else self._side_dir()
            follow_tag = f"  [FOLLOW {fs} 0.5-1.2m]"
        else:
            follow_tag = ""
        rx, ry = self._rel_xy()
        self._tele(
            f"LEG col{self._col_idx} {'FWD' if self._leg_dir > 0 else 'BCK'} "
            f"trav={travelled:.2f}/{self._max_leg:.1f} m  {leading}={ld} m "
            f"v={speed:.2f}  pos=({rx:+.2f},{ry:+.2f}){follow_tag}"
            + ("  [FINAL col]" if self._last_col else ""))

    def _end_leg(self, via_line: bool = True):
        """A leg finished (line stop or fallback). Either return home (if this
        was the final stripe) or step to the next stripe.

        Anti-thrash: a leg that ended on a line after almost no travel means
        there is no room to survey here (degenerate/very thin strip). Count
        those; too many in a row → RETURN instead of ping-ponging forever."""
        if self._last_col:
            self.get_logger().info(
                f"Final stripe traversed (col {self._col_idx}) — "
                f"{self._cp_count} photos. Returning home.")
            self._arrived_since = None
            self._goto_start    = None
            self._phase         = Phase.RETURN
            return

        ax, ay = self._leg_anchor
        travelled = math.hypot(self._pose.pose.position.x - ax,
                               self._pose.pose.position.y - ay)
        if via_line and travelled < self._min_leg:
            self._stuck += 1
        else:
            self._stuck = 0
        if self._stuck >= int(self._stuck_max):
            self.get_logger().warn(
                f"No room to survey ({self._stuck} near-zero legs in a row) "
                "— returning home.")
            self._arrived_since = None
            self._goto_start    = None
            self._phase         = Phase.RETURN
            return

        if self._col_idx + 1 >= int(self._max_columns):
            self.get_logger().warn(
                f"Column cap ({int(self._max_columns)}) reached — returning.")
            self._arrived_since = None
            self._goto_start    = None
            self._phase         = Phase.RETURN
            return
        self._start_step()

    # ── Sub-state: step sideways to the next stripe ──────────────
    def _do_step(self, stale: bool):
        side = self._side_dir()

        # On stale data, HOLD (don't step blindly toward the right boundary).
        if stale:
            self._hold_here()
            self._tele(f"STEP → {self._step_side} — detector STALE, holding")
            return

        # Never cross the end boundary while stepping: park a stand-off from
        # it. Stop the step when we've either shifted a full column or come
        # within right_step_stop of the side line.
        side_d = self._dist_to(side)
        ax, ay = self._step_anchor
        shifted = math.hypot(self._pose.pose.position.x - ax,
                             self._pose.pose.position.y - ay)

        # We are stepping toward the END boundary — the moment it comes into
        # view (any reading up to the detect range, INCLUDING < the band floor
        # or 0.0 right over the tape) this is the final column. We still park a
        # stand-off and do one full traversal alongside it before returning.
        end_conf = self._end_line_dist()   # confidence-gated (coverage + hold)
        if not self._last_col and end_conf is not None and end_conf <= self._right_hi:
            self._last_col = True
            self.get_logger().info(
                f"'{side}' boundary in view ({end_conf:.2f} m, "
                f"{self._bnd_coverage:.1f}% yellow) — final column. "
                "Parking a stand-off, then one traversal alongside it.")

        near_side = (side_d is not None and side_d <= self._right_step_stop)
        full_step = shifted >= self._col_step

        # If we stopped because the SIDE line is within the stand-off distance,
        # the drone has PHYSICALLY reached the end boundary — this is the last
        # stripe even when coverage is too low for the confidence gate above.
        # Marking it here makes the next leg RETURN instead of stepping again
        # into the tape, which kills the end-of-arena thrash (repeated zero-
        # length columns that eventually drift across the line).
        if near_side and not self._last_col:
            self._last_col = True
            self.get_logger().info(
                f"'{side}' boundary reached ({side_d:.2f} m ≤ "
                f"{self._right_step_stop:.2f} m) — FINAL stripe, RETURN after it.")

        if near_side or full_step:
            self._hold_here()
            reason = (f"side line {side_d:.2f} m ≤ {self._right_step_stop:.2f}"
                      if near_side else f"stepped {shifted:.2f} m")
            self.get_logger().info(
                f"Step complete ({reason}) — starting "
                f"{'FINAL ' if self._last_col else ''}"
                f"{'BCK' if self._leg_dir > 0 else 'FWD'} leg.")
            self._col_idx += 1
            self._leg_dir *= -1              # boustrophedon: flip direction
            self._start_leg()
            return

        self._crawl_toward(self._step_target[0], self._step_target[1],
                           self._appr_speed)
        sd = f"{side_d:.2f}" if side_d is not None else "—"
        self._tele(f"STEP → {self._step_side}  shifted={shifted:.2f}/"
                   f"{self._col_step:.2f} m  {side}={sd} m"
                   + ("  [FINAL col next]" if self._last_col else ""))


# ─────────────────────────────────────────────────────────────────
# Entry point
# ─────────────────────────────────────────────────────────────────

def main():
    rclpy.init()
    node = BoundedSurveyMission()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
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
