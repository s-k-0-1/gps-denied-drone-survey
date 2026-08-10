# Corner1 Test Auto — Full Mission Reference

**Package:** `viman_mission`
**Launch file:** `launch/corner1.launch.py`
**Mission node:** `viman_mission/corner1_test_auto.py`
**Node name (in ROS graph):** `boundary_test_auto` (deliberately reuses the tuned YAML block of that name)
**Team:** Viman Rakshak / IRoC-U 2026

---

## Command

```bash
ros2 launch viman_mission corner1.launch.py
```

Optional overrides:

```bash
ros2 launch viman_mission corner1.launch.py start_detector:=false   # use your own yellow detector
ros2 launch viman_mission corner1.launch.py rtabmap_log:=info       # full RTAB detail
ros2 launch viman_mission corner1.launch.py start_camera:=false     # camera driver runs separately
ros2 launch viman_mission corner1.launch.py start_rtabmap:=false    # RTAB-Map runs separately
```

MAVROS is **not** launched here — start it the way you already do.

---

## What the launch file spawns

Five parallel processes, each in its own OS process (the launch system owns them, so Ctrl+C delivers SIGINT cleanly):

1. **`rs_pipeline`** — RealSense colour + depth driver
   Hardware-stamped frames, aligned depth in colour frame, correct camera_info, static TF `camera_link` → `camera_color_optical_frame`, auto USB hardware-reset + retry on start.

2. **RTAB-Map stack** — `rgbd_odometry` + `rtabmap`
   Uses the "robust flight preset" from `rtabmap_config.robust_flight_launch_args`. Saves the map to `/media/jetson/ROS2_SSD/maps/corner1_<timestamp>.db`. On the ground the downward camera sees nothing, so odometry fail-resets in a loop — that is by design; the mission seeds it at altitude.

3. **`vio_gate`** — validated, gated RTAB → PX4 bridge
   Refuses to publish `/mavros/vision_pose/pose` until the mission calls `/viman/seed` and then `/viman/gate true`. Computes the Initialization Factor `IF = Q × A × S`. Watchdogs auto-close the gate on covariance spikes, RTAB pose jumps, or vision-EKF divergence.

4. **`yellow_boundary_detector`** — LINE-ONLY yellow detector
   Filters kept: area ≥ 150 px, length ≥ 30 % frame (floor 100 px), width ≤ 40 px, aspect ratio ≥ 3.0 : 1, rectangularity ≥ 0.40. Publishes `/viman/boundary/{repulsion, nearest_m, coverage_pct, lines, corner}`.

5. **`corner1_test_auto`** — the mission state machine (this file describes it end to end)

## Launch arguments

| argument         | default | meaning |
|------------------|---------|---------|
| `start_rtabmap`  | `true`  | include RTAB-Map stack |
| `start_camera`   | `true`  | run this package's `rs_pipeline` (NEVER run two camera drivers) |
| `start_detector` | `true`  | run yellow_boundary_detector |
| `params_file`    | mission_params.yaml | ROS parameter YAML |
| `rtabmap_log`    | `warn`  | RTAB verbosity: `warn` (default), `info` (VIO debug), `error` (near-silent) |

---

## One-sentence mission

Flow takeoff → return to arm point + lock yaw to the drone's actual settled heading → seed and validate RTAB VIO → climb from the 2 m validation altitude to **3 m** on fused VIO → fly backward until the back yellow line is seen, then switch to perpendicular line-frame approach and settle inside the 0.5–1.2 m band off it → follow the back line leftward (holding the back band) until the left line appears → keep sliding left until **BOTH** the back and left lines are simultaneously inside the 0.5–1.2 m band → hold 5 s = **Corner 1** (terminal shows `1/4 corner detected`) → enter a dedicated left-line-follow phase (the survey has NOT started), fly FORWARD tangent to the left line while holding its band, then bring **BOTH** the front and left lines into the 0.5–1.2 m band → hold 5 s = **Corner 2** (`2/4 corner detected`) → start the lawnmower with a 2 m RIGHT step while holding the front-line band, run BACKWARD, step 2 m RIGHT while holding the back-line band, run FORWARD, and repeat → at the 3 m cruise altitude a RIGHT line may be visible from far away, but it is ignored by Corner 3 logic until its classified distance remains inside the **0.5–1.8 m right-acquisition gate** → once that gate latches, use the same six-mode range-band controller as Corner 2 to bring the current front/back end + right pair into 0.5–1.2 m → hold 5 s = **Corner 3** → if Corner 3 is at the front, use the six-mode controller to follow the right-line band BACKWARD to the back; if it is at the back, follow the right-line band FORWARD to the front → distant opposite-end detections are ignored until they enter the partner-reach gate → bring right + opposite end into 0.5–1.2 m and hold 5 s = **Corner 4** → once all 4 corners are found → return home → close gate → flow settle → precision descend on locked X/Y → AUTO.LAND touchdown + disarm.

> **Corner rule (applies to every corner):** a corner is only validated once the drone is inside the **0.5–1.2 m band from BOTH meeting lines at the same time**, held for 5 s. If one line is in-band but the other is not, the drone first manoeuvres (slides / eases) to bring **both** lines into the band *before* the 5 s hold starts. Never validate on one line alone.

## Two motion modes (the core design)

**OPEN mode — no yellow line visible**
Motion is along the LOCKED body axes (`_cruise_yaw`). "Forward" and "left" mean the same thing throughout the mission even if the live pose yaw wobbles.

**LINE-FOLLOW mode — yellow line visible**
Motion is defined RELATIVE TO THE LINE (perpendicular = distance control; tangent = slide along it). Yaw is irrelevant; the line's own detected geometry dictates direction. Four sub-flavours:

- **FOLLOW-PERP** — perpendicular approach toward the line until it sits inside the standoff band (used by ACQ_BACK).
- **FOLLOW-LEFT** — tangent slide along a single line while keeping perpendicular distance inside the `[band_lo, band_hi]` = 0.5–1.2 m band (used by ACQ_LEFT to move left along the back line for Corner 1, and by FOLLOW_LEFT_FWD to move forward along the left line for Corner 2).
- **FOLLOW-RIGHT** — after Corner 3, tangent travel along the right boundary while holding its band: BACKWARD if Corner 3 was at the front, FORWARD if Corner 3 was at the back. The opposite end line then supplies Corner 4.
- **CORNER-BAND** — the corner-validation rule: the drone is only "at a corner" when BOTH meeting lines are simultaneously inside the 0.5–1.2 m band. If only one line is in-band, the drone keeps its tangent slide alive (or eases perpendicular) until the second line also enters the band — it does **not** freeze the moment the second line becomes visible. This replaces the old exact-point "equalize to 0.5 m each" scheme, which could stall at a false equilibrium where the two distance gradients cancelled before either line reached target (the "drone sits at the corner and never moves" bug).

**RIGHT-GATE CLOSED — right line visible but farther than 1.8 m**
The detector may publish the right boundary long before the drone is physically near it. SURVEY_STEP and SURVEY_STRIPE may show that distance in telemetry, but they do not brake toward it, align to it, or create a Corner 3 candidate. The lawnmower continues exactly as if that far right line were absent.

**RIGHT-GATE OPEN — classified right line held inside 0.5–1.8 m**
Only after `right_gate_lo_m ≤ right.dist ≤ right_gate_hi_m` is held for `right_gate_confirm_s` does `_right_gate_latched` become true. This latch permits the same six-mode controller used for Corner 2. The **1.8 m value is acquisition only**: Corner 3 / 4 counting still requires both meeting lines in the tighter 0.5–1.2 m band plus the full 5 s hold.

## Key safety layers (always live)

- **CH5 latch** — CH5 must be flipped HIGH once, then LOW, to start the mission.
- **CH5 kill** — CH5 HIGH at any point → STABILIZED, pilot has full control (SAFE_MANUAL phase).
- **Ctrl+C** — first press → emergency AUTO.LAND; second press → force exit.
- **Preflight gate** — no arming until FCU / pose rate / RC / RTAB / vio_gate / boundary detector / MAVROS are all healthy.
- **VIO fault** — any fault anywhere in the mission → FLOW_HOLD → re-seed / re-validate → resume the banked phase (up to `max_revalidations = 6`).
- **Boundary detector silent** — motion blocked; silent too long → return + land.
- **Hard wall veto (`_apply_boundary`)** — every published setpoint is per-line-clamped so the drone can never lead past `stop_dist` toward any yellow line. THE DRONE NEVER CROSSES A LINE.
- **Breach recovery (`_breach_recover`)** — if the drone ends up inside standoff, ease AWAY at proportional speed, hold at `stop_dist + recover_margin`, then release.
- **Back-lost hold (ACQ_LEFT)** — if back line vanishes > `acq_left_back_lost_s` (1.5 s) → HOLD the strafe (safety against blind cruise into an unmapped area).
- **Left-lost hold (FOLLOW_LEFT_FWD)** — a brief LEFT-line dropout freezes the advancing setpoint for `line_bridge_s`; if the line does not return, `left_lost_hold` keeps the drone stationary. Corner 2 search never continues forward without the left boundary that defines its tangent path.
- **Far-right rejection (survey)** — a correctly classified right line with `dist > right_gate_hi_m` is `RIGHT-FAR-IGNORED`: it cannot slow the step, stop a stripe, or start Corner 3. “Ignored” applies only to mission/corner decisions; `_apply_boundary()` and `_breach_recover()` still see every valid line and remain authoritative.
- **Survey progress watchdog** — during SURVEY_STEP / SURVEY_STRIPE / FOLLOW_RIGHT_END, remember the last commanded direction and compare it with actual projected pose progress. If progress stays below `survey_stall_min_progress_m` for `survey_stall_timeout_s` (10 s), reset the setpoint to the live pose, perform a zero-velocity settle for `recover_settle_s`, health-check all guards, and retry that same direction at `corner_speed_ms`. Retries are bounded by `survey_stall_max_retries`; it never drives indefinitely into a physical obstruction.

---

## The 24 phases in flight order

| # | Phase | One-line role |
|---|-------|---------------|
| 1  | IDLE          | preflight gate + CH5 latch, waiting to start |
| 2  | ARM           | OFFBOARD + arm; capture HOME + arm-time heading |
| 3  | TAKEOFF       | flow-only climb to `takeoff_alt` (2.0 m) |
| 4  | STABLE_OF     | zero-velocity flow settle (4 s) |
| 5  | HOVER_HOME    | back to arm point + slew to arm-time heading + LOCK cruise_yaw to actual settled yaw |
| 6  | SEED          | trigger `/viman/seed` → RTAB odom reset + frame alignment |
| 7  | VALIDATE      | small motion square; IF ≥ 0.7 held for 5 s (dip grace 1 s) |
| 8  | HANDOVER      | call `/viman/gate true`; wait for GS_OPEN + settle |
| 9  | CLIMB         | fused-VIO climb from `takeoff_alt` (2.0 m) to `cruise_alt` (3.0 m) |
| 10 | ACQ_BACK      | fly backward (OPEN) → switch to FOLLOW-PERP once yellow line visible → settle at stop_dist |
| 11 | ACQ_LEFT      | slide LEFT along back line (holding its band) → keep sliding until BOTH back + left lines are in the 0.5–1.2 m band |
| 12 | CORNER1_HOLD  | actively hold back + left in-band for 5 s → count Corner 1 (`1/4 corner detected`) |
| 13 | FOLLOW_LEFT_FWD | six-mode Corner-1-equivalent controller: FORWARD tangent along left, independently bring front + left in-band, then hand Corner 2 to CORNER_HOLD; survey is not active yet |
| 14 | CORNER_HOLD   | reusable active 5 s in-band hold for Corner 2, Corner 3, or Corner 4; branch by `_pending_corner_number` |
| 15 | SURVEY_STEP   | shift 2 m RIGHT holding the current end line; ignore far-right detections until the 0.5–1.8 m right gate latches, then run the six-mode end + right Corner 3 approach |
| 16 | SURVEY_STRIPE | alternate BACK/FORWARD; ignore far right while the gate is closed; if the gate opens here, six-mode-follow right toward the active front/back end for Corner 3 |
| 17 | FOLLOW_RIGHT_END | after Corner 3, run the complete six-mode right-reference controller toward the opposite end for Corner 4; direction depends on whether Corner 3 was FRONT or BACK |
| 18 | RETURN        | crawl to HOME on fused VIO |
| 19 | FLOW_SETTLE   | camera off; EKF transitions to flow at altitude (2.5 s) |
| 20 | DESCEND       | OFFBOARD precision descent, X/Y locked on HOME |
| 21 | FLOW_HOLD     | VIO-fault recovery: flow hold → re-seed → resume banked phase |
| 22 | LAND          | close gate; request AUTO.LAND |
| 23 | DISARM        | wait for PX4 self-disarm after touchdown |
| 24 | SAFE_MANUAL   | CH5 HIGH → STABILIZED; pilot has control |

(**DONE** = terminal no-op state after DISARM completes.)

---

### 1. IDLE

Publishes a low-altitude idle setpoint `(0, 0, 0.3)` at 20 Hz to pre-stream OFFBOARD.

Runs `_preflight_failures()`. Every check that fails goes in a list; if the list isn't empty, throttle-logs `PREFLIGHT BLOCKED: <reasons>` every 5 s.

Preflight checks:

- FCU connected (MAVROS `/mavros/state.connected`)
- Pose rate ≥ 15 Hz (from a 2 s ring buffer of pose stamps)
- RC frames arriving (channels length > `rc_ch5_index`)
- RTAB odom fresh (last message < 2 s ago)
- vio_gate reporting (`vio_state != 255`)
- Boundary detector fresh (last message < 2 s ago)
- `/viman/seed` service ready
- MAVROS arming + set_mode services ready

Once preflight passes, requires **CH5 latch**: CH5 must be seen HIGH (≥ 1300) at least once, then LOW (≤ `rc_start_low` = 1200) — protects against accidental starts.

On CH5 LOW after latch → transition to **ARM**.

### 2. ARM

Keeps pre-streaming idle setpoints so PX4 accepts OFFBOARD.

Sequence:

1. Request `SetMode("OFFBOARD")` once.
2. Wait for `state.mode == "OFFBOARD"`. Keep re-requesting if not.
3. Request `CommandBool(True)` once.
4. Wait for `state.armed == True`.

Once armed:

- **Capture HOME** = current ENU `(x, y)` from `/mavros/local_position/pose`.
- **Capture arm-time heading** into `_arm_heading_q`.
- **Leave `_hold_heading_q` and `_cmd_yaw_rad` as None** so yaw floats during the whole climb — an EKF2 mag-yaw reset cannot snap the airframe. The heading is applied later, at altitude, in HOVER_HOME.
- Log: `Armed. HOME=(x, y) ARM-TIME heading = X deg → will hold ARM-TIME heading at home`.

→ **TAKEOFF**.

### 3. TAKEOFF

Publishes `(home_x, home_y, takeoff_alt)` at 20 Hz. Flow-only climb (VIO isn't seeded yet, camera on the ground has no features).

Confirms at altitude: `|alt − takeoff_alt| ≤ alt_tolerance` (0.12 m) held for `at_alt_confirm_s` (1.5 s).

→ **STABLE_OF**.

### 4. STABLE_OF

Publishes zero-velocity `TwistStamped` (no position setpoint). Optical flow directly counteracts drift instead of fighting a drifting EKF position estimate.

Waits `stable_of_secs` (4 s) → **HOVER_HOME**.

### 5. HOVER_HOME — yaw lock happens here

The most delicate phase. Two stages.

**Stage 1: get home + settle**

Publishes `(home_x, home_y, takeoff_alt)`. Watches distance to home. When distance ≤ `goto_radius` (0.2 m) for 2 s:

- Decide the target heading:
  - If `yaw_use_arm_heading = true` (recommended, AUTO style) → target = `_arm_heading_q`
  - Else → target = `_yaw_quat(mission_yaw_deg)` (fixed compass angle)
- Set `_hold_heading_q` = target, initialize `_cruise_yaw` = target (temporarily), `_cmd_yaw_rad` = current live yaw.
- Start the alignment timer.
- Log: `At home, settled. Slewing onto heading X deg, then holding until aligned before seeding.`

**Stage 2: slew + wait for alignment**

`_pub_sp()` gently slews `_cmd_yaw_rad` toward the target at `yaw_slew_dps` (15°/s), publishing the slewed quaternion in every setpoint. PX4 tracks it.

The loop measures the ACTUAL pose yaw against target every tick. When `err ≤ yaw_align_tol_deg` (3°) is held for `yaw_align_hold_s` (1 s), OR `yaw_align_timeout_s` (25 s) elapses:

**RE-LOCK `_cruise_yaw`, `_cmd_yaw_rad`, `_hold_heading_q` to the drone's ACTUAL settled yaw** (not the intended target). Reason: PX4 will stop within tol_deg of target — usually a couple of degrees off. Anchoring the mission frame to the actual heading makes forward/back/left/right fly straight along the drone's real body axes.

Log: `Yaw locked at ACTUAL X deg (target was Y, aligned/timeout). Mission frame anchored here — forward / back / left / right now fly straight along the drone's real body axes. Seeding.`

→ **SEED**.

### 6. SEED

Publishes zero-velocity hold.

Calls `/viman/seed` (Trigger) once. This causes vio_gate to:

- Call `/rtabmap/reset_odom`
- Capture `q_corr = q_ekf ⊗ q_rtab⁻¹` and pose anchors
- Transition to `GS_SEEDING` → `GS_VALIDATING`

Waits for `vio_state == GS_VALIDATING`. On timeout (`seed_timeout_s` = 10 s) → **FLOW_HOLD**.

On success: capture `_validate_anchor` = current pose (x, y), reset IF timers. → **VALIDATE**.

### 7. VALIDATE

Runs a tiny **motion square** around `_validate_anchor` (only if `motion_test = true`, default):

Corners: `(0,0), (amp,0), (amp,amp), (0,amp)` where `amp = motion_amp_m` (0.2 m). Each leg lasts `motion_leg_s` (4 s). This forces non-trivial motion so vio_gate's Kabsch fit has real data to work with.

Every tick:

- Publish `(anchor_x + ox, anchor_y + oy, takeoff_alt)` — offset by current motion square corner.
- Check `vio_state`. If ≥ `GS_FAULT_MIN` → **FLOW_HOLD**.
- Read `init_factor` (published by vio_gate at 10 Hz).
  - If IF ≥ `validate_if_min` (0.7): start / continue `_if_good_since` timer. Held ≥ `validate_hold_s` (5 s) → **HANDOVER**.
  - If IF < threshold with prior good history: start `_if_low_since` grace timer. If low-time > `validate_dip_grace_s` (1 s) → reset the good timer.

Timeout: `validate_timeout_s` (60 s) → **FLOW_HOLD**.

### 8. HANDOVER

Publishes hold setpoint at `_validate_anchor`, altitude = `takeoff_alt` (first time) or `cruise_alt` (re-entry after fault recovery).

Calls `/viman/gate true` (SetBool) once. vio_gate then:

- Prefers the motion-fitted rotation `_q_fit` (Kabsch); falls back to seed-attitude `_q_corr`.
- Re-captures anchors at THIS instant → first published vision pose ≡ current EKF estimate (zero innovation).
- Transitions to `GS_OPEN`, starts publishing `/mavros/vision_pose/pose` at 30 Hz.

Waits for `vio_state == GS_OPEN`, then a `handover_settle_s` (2 s) settle so PX4's EKF absorbs the first fused samples.

Branch:

- First entry (`_mission_started == False`) → **CLIMB**.
- Re-entry after FLOW_HOLD → resume the phase saved in `_resume_phase`.

On timeout (5 s without gate open) → **FLOW_HOLD**.

### 9. CLIMB

Uses fused VIO now.

Ramps altitude up: `_sp_z += climb_speed_ms / sp_rate_hz` per tick, capped at `cruise_alt` (3.0 m). Holds X/Y at `_validate_anchor`.

Confirms at altitude: `|alt − cruise_alt| ≤ alt_tol` for `at_alt_confirm_s` (1.5 s).

On confirm:

- Set `_acq_start` = current pose.
- Set `_mission_started = True`.
- Log: `At X m — flying BACKWARD from centre to find the back line (first boundary).`
- → **ACQ_BACK**.

### 10. ACQ_BACK — first line-frame switch

Guard against VIO fault and boundary-dead conditions.

Initializes `_sp` from current pose. Runs `_breach_recover` (if we already ended up inside standoff, ease back out first before any forward drive — the phase handler returns while recovering).

Get `_dir_vectors()` → forward/left ENU unit vectors from LOCKED `_cruise_yaw`.

Classify visible lines → back = closest line whose normal aligns with body back. Fall back to nearest line if classification failed.

Register the target with `_approach()` — this latches `_appr_acquired = True` the first time a line is seen within `slow_dist_m` (1.2 m).

**Two sub-modes:**

- **OPEN** (no line visible):
  ```
  _sp[0] -= fx × survey_speed × dt   # backward = -forward
  _sp[1] -= fy × survey_speed × dt
  ```
  This is the fixed 4-direction backward cruise along locked body -X.

- **FOLLOW-PERP** (line visible): call `_line_perp_approach_step(target, stop_dist, dt, speed_cap=survey_speed)`:
  ```
  err = line.dist - stop_dist        # + = too far, need to close
  speed = clamp(0, corner_gain × err, speed_cap)
  _sp[0] -= line.ex × speed × dt     # toward line = -push_away
  _sp[1] -= line.ey × speed × dt
  ```
  Straight-perpendicular approach to the wall regardless of yaw offset. Tapers as we approach — no overshoot, no diagonal drift.

Apply wall veto (`_apply_boundary`), publish setpoint.

Tele: `ACQ_BACK[OPEN|FOLLOW-PERP] trav=X/6.0 near=Y m ...`

Transition on ACQUIRE + settle: once `_appr_acquired == True` for `acq_settle_s` (1 s):

- Log: `Back line acquired (~Y m) — entering YELLOW-LINE-AREA, following back line LEFT to find Corner 1.`
- Reset `_acq_start`, `_corner_latched`, `_corner_at_target`, `_sp`.
- → **ACQ_LEFT**.

Bailout: if travelled > `max_forward_m` (6 m) without acquiring → return home.

### 11. ACQ_LEFT — slide left until BOTH lines are in-band

The goal of this phase is NOT to park on an exact point. It is to slide LEFT along the back line, keeping the back line inside the `[band_lo, band_hi]` = **0.5–1.2 m** band, and keep sliding until the **left line also enters that same band** — at which point both lines are in-band together and we hand off to the corner hold. This range-band approach is what fixes the old freeze: the drone no longer stops the instant the left line appears, and there is no single-point gradient that can stall.

Guards. Initialize `_sp` and corner flags. `_breach_recover` first.

Get `_dir_vectors()`. Classify lines.

Identify:

- `back` = classified back line, or nearest fallback.
- `partner` = the LEFT line = any other line whose normal is roughly perpendicular to back's normal (`|n1·n2| < corner_perp_dot` = 0.5, i.e. angle between 60° and 120°).

Define the **in-band test** for a line: `band_lo (0.5) ≤ line.dist ≤ band_hi (1.2)`.

Track `_acq_left_back_lost_since`. When back is invisible:

- Start / continue the loss timer.
- If `back_lost_dur > acq_left_back_lost_s` (1.5 s) → `back_lost_hold = True`.

**Motion modes** (in priority order):

1. **HOLD** (`back_lost_hold = True`): don't move. Safety against blind cruise.

2. **SLIDE-LEFT + hold back band** (back visible, left NOT yet in-band — this is the normal case while approaching the corner):
   ```
   Tangent leftward along back: t1 = (-ey, ex), t2 = (ey, -ex)
   Pick the tangent whose ENU projection on (lx, ly) is positive → (tx, ty)
   # two-sided band controller on the BACK line (dead zone inside the band):
   if   back.dist > band_hi:  perp_v = -corner_gain × (back.dist - band_hi)   # too far → ease TOWARD
   elif back.dist < band_lo:  perp_v = +corner_gain × (band_lo - back.dist)   # too close → ease AWAY
   else:                       perp_v = 0                                       # in band → hold
   perp_v = clamp(-corner_speed, perp_v, corner_speed)
   _sp[0] += (tx × strafe_speed - back.ex × perp_v) × dt
   _sp[1] += (ty × strafe_speed - back.ey × perp_v) × dt
   ```
   Keep this tangent slide **alive even after the left line becomes visible** — do not switch away just because a second line is now in frame. Only stop sliding once the left line is actually *in-band*.

3. **APPROACH-LEFT-INTO-BAND** (back in-band, left visible but OUT of band): the left line exists but is still too far (> 1.2 m) or too close (< 0.5 m). Ease perpendicular to the LEFT line to bring it into the band, while a light band controller keeps the back line inside its band too:
   ```
   left band term (toward/away from the LEFT line, same two-sided rule as above)
   back band term  (toward/away from the BACK line)
   sum the two band velocities, cap magnitude at corner_speed
   _sp += v × dt
   ```
   This is the "back range me hai par left nahi — pehle dono ko range me lao" step. There is no single fixed target point, so it cannot stall at a false equilibrium; each line only pushes when it is *outside* the band, and goes quiet once inside it.

4. **APPROACH-BACK-INTO-BAND** (left in-band, back visible but OUT of band): the back line exists but is still too far (> 1.2 m) or too close (< 0.5 m). Ease perpendicular to the BACK line to bring it into the band, while a light band controller keeps the left line inside its band too:
   ```
   back band term  (toward/away from the BACK line, same two-sided rule as above)
   left band term  (toward/away from the LEFT line)
   sum the two band velocities, cap magnitude at corner_speed
   _sp += v × dt
   ```
   This is the "left range me hai par back nahi — pehle dono ko range me lao" step. There is no single fixed target point, so it cannot stall at a false equilibrium; each line only pushes when it is *outside* the band and goes quiet once inside it.

5. **BOTH-IN-BAND** (back in-band AND left in-band at the same time): stop the tangent slide. Mark `_corner_at_target = True` and hold position with gentle two-sided band control on both lines (so a jumpy reading that drifts a line out of band nudges it back in). This is the CORNER-BAND condition.

6. **back-flicker** (back briefly out < grace): no setpoint change, just log the flicker duration.

Apply wall veto, publish setpoint.

Tele: `ACQ_LEFT[MODE] strafe=X/6.0 back=Y m left=Z m band=0.5-1.2 m`.

Corner 1 candidate confirmed: `_corner_at_target` (both lines in-band) held for `acq_settle_s` (1 s):

- Log: `Corner 1 (back-left): both lines in band (back Y m, left Z m, band 0.5-1.2 m) - holding 5 s to confirm.`
- Capture `_hold_xy` = current pose, clear `_sp`.
- → **CORNER1_HOLD** (the 5 s dwell; Corner 1 is *declared* at the end of that hold).

Bailout: if strafed > `max_strafe_m` (6 m) without ever getting both lines in-band → return home.

### 12. CORNER1_HOLD — active 5 s hold, then dedicated left-line forward follow

The drone actively keeps BOTH back + left inside the 0.5–1.2 m band throughout the dwell. This is not a frozen position: `_band_correction()` is zero while a line is in-band and gently returns it only after it leaves the band, so noisy tape readings do not create exact-point hunting.

Guards. Initialize `_sp` from `_hold_xy`. Identify `back` + perpendicular partner `left` using the same classification and `corner_perp_dot` test as ACQ_LEFT. Run `_breach_recover()` before normal hold control.

- If both lines are visible → apply two-sided band control to both; cap the combined correction at `corner_speed_ms`.
- If either line is outside `[band_lo, band_hi]` → correct it and reset `_hover_start`; the 5 s validation time cannot accumulate while only one line is valid.
- If a line flickers → bridge its last valid observation for at most `line_bridge_s`; after that, hold position and reset `_hover_start`.
- If nothing is visible → leave the setpoint at the live hold position; boundary-stale handling and the wall veto remain active.

Apply `_apply_boundary()`, publish at `sp_rate_hz`.

Tele: `CORNER1 hold N/5s back=Ym left=Zm band=0.5-1.2m`.

**Declaration happens at the END of the uninterrupted in-band hold.** After `corner1_hold_s` (`corner_hold_s`) = **5 s**:

- `_count_corner("back + left lines in band")` de-dupes using `corner_dedup_m` and increments `_corners_found`.
- On a new corner, log:
  ```
  ★ CORNER 1/4 found (back + left lines in band) at (x, y).
  1/4 corner detected
  ```
- Initialize:
  ```
  _follow_start           = current pose
  _corner_candidate_since = None
  _left_line_lost_since   = None
  _sp                     = current pose
  ```
- Log: `Corner 1 hold complete (5 s) — following the LEFT line FORWARD to find Corner 2. Survey has NOT started.`
- → **FOLLOW_LEFT_FWD**.

> Corner 1 never transitions directly to SURVEY_STRIPE. Corner 2 has its own line-follow and validation path.

### 13. FOLLOW_LEFT_FWD — hold the left-line band and fly forward to Corner 2

This is a dedicated line-frame phase, not a lawnmower stripe. It flies tangent to the LEFT boundary in the locked forward direction while continuously holding that line inside `[band_lo, band_hi]`.

Guards. Early-exit to RETURN if the target is not found within `max_stripe_m`. A VIO fault banks FOLLOW_LEFT_FWD in `_resume_phase` and enters FLOW_HOLD. Boundary silence blocks motion at `boundary_stale_s` and begins return after `stale_land_s`. Run `_breach_recover()` before any tangent command.

Get `_dir_vectors()` → locked forward `(fx, fy)` and left `(lx, ly)`. Classify:

- `left` = line whose push-away normal aligns with locked left using `left_thresh`.
- `front` = line whose normal aligns with locked forward using `ahead_thresh`.
- A valid front + left corner pair must also satisfy `|n_front·n_left| < corner_perp_dot`.

Track `_follow_left_lost_since` exactly as ACQ_LEFT tracks `_acq_left_back_lost_since`:

```
if left visible:
    _follow_left_lost_since = None
    left_lost_hold = False
else:
    start / continue _follow_left_lost_since
    left_lost_hold = left_lost_duration > line_bridge_s
```

Define once and reuse for both lines:

```
def band_term(line):
    if line.dist > band_hi:
        # Too far: move TOWARD the line (opposite its push-away normal).
        v = -(line.ex, line.ey) × corner_gain × (line.dist - band_hi)
    elif line.dist < band_lo:
        # Too close: move AWAY along the push-away normal.
        v = +(line.ex, line.ey) × corner_gain × (band_lo - line.dist)
    else:
        v = (0, 0)  # dead zone: this line is already acceptable
    return magnitude_cap(v, corner_speed)
```

The Corner 2 controller has the same six modes as the Corner 1 approach, with the roles rotated: `back → left reference`, `left partner → front partner`, and `slide left → slide forward`. A raw second contour does not trigger alignment: `front` must pass the front classification, the perpendicular-pair test, and be within `corner_side_reach_m` before the specific two-line alignment modes override normal tangent travel.

1. **HOLD** (`left_lost_hold = True`): do not move. Keep `_sp` at the live hold pose. This is the direct Corner 2 equivalent of ACQ_LEFT's `back_lost_hold`; it prevents blind forward cruise after losing the LEFT boundary that defines the track.

2. **SLIDE-FORWARD + hold left band** (`left` visible, `front` not yet a usable in-band partner): this is the normal motion while travelling from Corner 1 toward Corner 2.
   ```
   # Tangents to the LEFT line:
   t1 = (-left.ey,  left.ex)
   t2 = ( left.ey, -left.ex)

   # Select the tangent pointing along locked mission FORWARD:
   (tx, ty) = tangent with max(dot(tangent, (fx, fy)))

   left_v = band_term(left)
   _sp[0] += (tx × survey_speed + left_v.x) × dt
   _sp[1] += (ty × survey_speed + left_v.y) × dt
   ```
   The tangent component uses `survey_speed_ms` (0.2 m/s); `left_v` uses `corner_gain` (0.8) and is capped at `corner_speed_ms` (0.15 m/s). Keep the tangent motion alive merely because a second yellow contour becomes visible. Switch away only when it is classified as the perpendicular FRONT line within `corner_side_reach_m`, or when the front line is actually in-band. This prevents stopping on an irrelevant or distant contour.

3. **APPROACH-FRONT-INTO-BAND** (`left` in-band, valid `front` visible within `corner_side_reach_m`, but `front` OUT of band): stop the forward tangent and ease perpendicular to the FRONT line while continuing to protect the left-line band.
   ```
   front_v = band_term(front)
   left_v  = band_term(left)   # zero while left remains in-band
   v       = magnitude_cap(front_v + left_v, corner_speed)
   _sp    += v × dt
   ```
   This is: **"left range me hai, front nahi — pehle front ko bhi range me lao."** A front reading at 1.2–1.5 m is only the alignment trigger. It cannot start the hold until it reaches 0.5–1.2 m.

4. **APPROACH-LEFT-INTO-BAND** (`front` in-band, `left` visible but OUT of band): stop tangent travel and correct the LEFT reference line while a light front-line term keeps the front inside its band.
   ```
   left_v  = band_term(left)
   front_v = band_term(front)  # zero while front remains in-band
   v       = magnitude_cap(left_v + front_v, corner_speed)
   _sp    += v × dt
   ```
   This is: **"front range me hai, left nahi — pehle left ko bhi range me lao."** There is no exact target point and no opposing fixed-point gradient: each line pushes only while outside the band and becomes quiet inside it.

5. **BOTH-IN-BAND** (`left` in-band AND `front` in-band at the same time): stop the forward tangent. Set `_corner_at_target = True` and apply gentle two-sided band control to both lines so a reading that leaves the band is nudged back. This is the Corner 2 CORNER-BAND condition.
   ```
   v    = magnitude_cap(band_term(left) + band_term(front), corner_speed)
   _sp += v × dt
   ```

6. **LEFT-FLICKER** (`left` missing briefly, duration ≤ `line_bridge_s`): do not advance `_sp`; log the flicker duration and preserve the current setpoint. If the line returns inside the grace window, resume the appropriate mode. If it does not, mode 1 HOLD becomes active.

Mode selection is mutually exclusive in this order:

```
if left_lost_hold:
    HOLD
elif left missing briefly:
    LEFT-FLICKER
elif left_in_band and front_in_band:
    BOTH-IN-BAND
elif valid_front_within_reach and left_in_band and not front_in_band:
    APPROACH-FRONT-INTO-BAND
elif valid_front_within_reach and front_in_band and not left_in_band:
    APPROACH-LEFT-INTO-BAND
else:
    SLIDE-FORWARD + hold left band
```

If a valid front partner is within reach while BOTH lines are outside the band, remain in SLIDE-FORWARD while correcting the left reference line; once either line enters the band, the corresponding explicit alignment mode takes control. `_breach_recover()` and `_apply_boundary()` remain higher-priority safety layers in every mode.

Apply `_apply_boundary()` after all motion terms, then publish the setpoint.

Start `_corner_candidate_since` only while `_corner_at_target` remains true — BOTH front + left simultaneously in-band. Reset `_corner_at_target` and the timer immediately if either leaves. After `corner_confirm_s` (1 s, the same candidate-settle duration used for Corner 1):

```
_pending_corner_number = 2
_pending_line_a         = "front"
_pending_line_b         = "left"
_pending_corner_reason  = "front + left lines in band"
_corner_source_phase    = FOLLOW_LEFT_FWD
_after_corner           = START_SURVEY
_corner_hold_since      = None
_sp                     = current pose
```

→ **CORNER_HOLD**.

Tele:

```
FOLLOW_LEFT_FWD[SLIDE-FORWARD] trav=X/12.0 left=Ym front=--- band=0.5-1.2m
FOLLOW_LEFT_FWD[APPROACH-FRONT-INTO-BAND] trav=X/12.0 left=Ym front=Zm band=0.5-1.2m
FOLLOW_LEFT_FWD[APPROACH-LEFT-INTO-BAND] trav=X/12.0 left=Ym front=Zm band=0.5-1.2m
FOLLOW_LEFT_FWD[BOTH-IN-BAND] trav=X/12.0 left=Ym front=Zm confirm=N/1.0s
FOLLOW_LEFT_FWD[LEFT-FLICKER] lost=Ts/1.0s hold-position
FOLLOW_LEFT_FWD[HOLD] left-lost - no blind forward motion
```

Corner 2 candidate confirmed after `_corner_at_target` remains true for `corner_confirm_s`:

```
Corner 2 (front-left): both lines in band (front Y m, left Z m, band 0.5-1.2 m) - holding 5 s to confirm.
```

Capture the live pose in `_sp`, clear the candidate timer, and enter CORNER_HOLD. Corner 2 is declared only after that phase completes the full `corner_hold_s` (5 s) dwell; the 1 s candidate confirmation does not count as part of the 5 s.

Bailout: travelled > `max_stripe_m` (12 m) without a valid front-line corner candidate → return home.

### 14. CORNER_HOLD — one identical validator for Corners 2, 3 and 4

This reusable phase makes the Corner 2 / 3 / 4 rule identical instead of duplicating slightly different timers inside survey handlers. `_pending_corner_number`, `_pending_line_a`, and `_pending_line_b` define the expected pair; `_after_corner` defines the next mission phase.

Every tick:

1. Reclassify the two expected lines and verify `|n1·n2| < corner_perp_dot`.
2. Run `_breach_recover()` first if the drone is inside standoff.
3. Apply `_band_correction()` to both lines, cap the combined vector at `corner_speed_ms`, then apply `_apply_boundary()`.
4. Accumulate `_corner_hold_since` only while both lines remain in `[band_lo, band_hi]`.
5. Reset the full hold timer if either line leaves the band, disappears beyond `line_bridge_s`, or stops passing the perpendicular-pair test.

Tele: `CORNER<N> hold T/5s line_a=Ym line_b=Zm band=0.5-1.2m`.

After `corner_hold_s` (`corner1_hold_s`) = **5 s**:

```
new_corner = _count_corner(_pending_corner_reason)
```

`_count_corner()` enforces `corner_dedup_m` (1.2 m) and `target_corners` (4). If the candidate is de-duped, do not advance; clear its timers and resume `_corner_source_phase`.

For a new corner, log:

```
★ CORNER N/4 found (<line_a> + <line_b> lines in band) at (x, y).
N/4 corner detected
```

If VIO recovery interrupts CORNER_HOLD, preserve the pending line roles and next-phase branch but reset `_corner_hold_since`; pre-fault hold time never counts after revalidation.

**After Corner 2:**

```
_stripe_dir       = +1       # just arrived at the front
_stripe_count     = 0
_step_start       = current pose
_step_end_role    = "front"
_corner3_end_role = None
_right_gate_latched = False
_right_gate_since   = None
_corner_at_target   = False
_corner_candidate_since = None
_survey_stall_retries = 0
_sp               = current pose
```

Log: `Corner 2 hold complete (5 s) — starting sweep: 2.00 m RIGHT holding FRONT, then BACKWARD.`

→ **SURVEY_STEP**. The first survey action is always the 2 m right shift.

**After Corner 3:**

```
if _corner3_end_role == "front":
    _right_follow_dir    = -1
    _right_follow_target = "back"
else:  # Corner 3 was at the back
    _right_follow_dir    = +1
    _right_follow_target = "front"

_right_follow_start = current pose
_right_reference_lost_since = None
_corner_at_target  = False
_sp                 = current pose
```

→ **FOLLOW_RIGHT_END**.

**After Corner 4:** `_corners_found == target_corners` → `_begin_return()` → **RETURN**.

### Shared right-boundary acquisition gate for Corners 3 and 4

At higher operating altitude (for example 3 m), the front, back, and right tape can all be visible while still physically far away. Detection therefore cannot be treated as acquisition.

Classify `right` only when its push-away normal aligns with locked mission RIGHT `(rx, ry) = (-lx, -ly)`. Reuse the magnitude of `left_thresh` for this opposite-side classification; an arbitrary nearest line is never allowed to open the gate.

Before Corner 3, evaluate:

```
right_gate_sample = (
    right is correctly classified
    and right_gate_lo_m <= right.dist <= right_gate_hi_m
)

if right_gate_sample:
    start / continue _right_gate_since
    if held >= right_gate_confirm_s:
        _right_gate_latched = True
else if not _right_gate_latched:
    _right_gate_since = None
```

Interpretation:

- `right.dist > right_gate_hi_m` (1.8 m) → **RIGHT-FAR-IGNORED**. Continue the current 2 m step or forward/back stripe. Do not taper, turn, align, or start a corner timer because of this reading.
- `right_gate_lo_m ≤ right.dist ≤ right_gate_hi_m` (0.5–1.8 m) held for `right_gate_confirm_s` → latch the right boundary and enable the Corner 3 six-mode controller.
- `right.dist < right_gate_lo_m` (0.5 m) → this is not a valid corner sample. `_breach_recover()` moves away first toward `stop_dist_m + recover_margin_m`; only a later in-window reading may open the gate.
- Once latched, a single noisy sample above 1.8 m does not close the gate. Use the normal reference-line flicker / lost HOLD logic. Keep the latch through Corner 3 because the right boundary becomes Corner 4's reference. Reset it after Corner 4 / mission restart, or during a pre-Corner-3 VIO recovery that invalidates the acquisition; a post-Corner-3 fault preserves the known branch but still discards cached line geometry.

> **Safety meaning of “ignore”:** a far right line is ignored only by the state transition and corner-counting logic. It remains available to telemetry, `_apply_boundary()`, and `_breach_recover()`. The drone never gains permission to cross a yellow line.

After the right gate opens, Corners 3 and 4 call the same six-mode range-band primitive documented in FOLLOW_LEFT_FWD:

```
_six_mode_corner_approach(reference, partner, tangent_direction, tangent_speed)
```

It always provides:

1. `HOLD` when the reference line is lost beyond `line_bridge_s`.
2. `SLIDE-<direction> + hold reference band` while the partner is not yet a usable in-band line.
3. `APPROACH-PARTNER-INTO-BAND` when reference is in-band but partner is outside.
4. `APPROACH-REFERENCE-INTO-BAND` when partner is in-band but reference is outside.
5. `BOTH-IN-BAND` when both simultaneously satisfy 0.5–1.2 m.
6. `REFERENCE-FLICKER` for a brief reference dropout; freeze the advancing setpoint during the grace window.

The shared mutually-exclusive selection is:

```
if reference_lost_hold:
    HOLD
elif reference missing briefly:
    REFERENCE-FLICKER
elif reference_in_band and partner_in_band:
    BOTH-IN-BAND
elif partner_usable and reference_in_band and not partner_in_band:
    APPROACH-PARTNER-INTO-BAND
elif partner_usable and partner_in_band and not reference_in_band:
    APPROACH-REFERENCE-INTO-BAND
else:
    SLIDE-<direction> + hold reference band
```

`partner_usable` never means “visible.” For Corner 3 during SURVEY_STEP it means the confirmed right gate is open. For Corner 3 during SURVEY_STRIPE and for Corner 4, it means the correct front/back role is perpendicular to right and inside `corner_side_reach_m`. Every non-BOTH mode resets the candidate timer.

The exact role mapping is:

| Candidate | Entry situation | Reference line | Tangent direction | Partner line |
|---|---|---|---|---|
| Corner 2 | dedicated left follow | left | locked FORWARD | front |
| Corner 3 | gate opens during SURVEY_STEP | current end (`front` or `back`) | locked RIGHT | right |
| Corner 3 | gate opens during SURVEY_STRIPE | right | current stripe direction (FORWARD or BACKWARD) | active end (`front` or `back`) |
| Corner 4 | Corner 3 was FRONT | right | locked BACKWARD | back |
| Corner 4 | Corner 3 was BACK | right | locked FORWARD | front |

This mapping is what makes Corner 3 direction-independent: the mission never assumes Corner 3 is at the front. `_corner3_end_role` comes from `_step_end_role` when acquired during a step, or from `_stripe_dir` when acquired during a stripe.

### 15. SURVEY_STEP — move 2 m right while holding the current end line

This is the first phase after Corner 2 and the transition between later stripes. Get locked right `(rx, ry) = (-lx, -ly)`. `_step_end_role` identifies the front/back line just reached. The current end line is the reference; the right boundary is only a possible partner after the right gate latches.

Run `_breach_recover()` before all normal motion. Classify `end` from `_step_end_role` and classify `right` against locked RIGHT. Update `_right_gate_since` using the shared gate, but do not let a far right line alter the normal step.

**RIGHT-GATE-CLOSED / normal 2 m step:**

```
stepped = max(0, dot(current_xy - _step_start, (rx, ry)))
end_v   = _hold_offset(end)  # keep current front/back end in 0.5-1.2 m

_sp[0] += (rx × strafe_speed + end_v.x) × dt
_sp[1] += (ry × strafe_speed + end_v.y) × dt
```

There is deliberately no right-distance brake while `_right_gate_latched == False`. If a valid right line reads 4 m, 3 m, 2.2 m, or any value above `right_gate_hi_m` = 1.8 m, telemetry reports it as `RIGHT-FAR-IGNORED` and the full `strafe_speed_ms` step continues. `_apply_boundary()` remains the final safety veto.

**RIGHT-GATE-OPEN / Corner 3 approach from an end:** map the Corner 2 six-mode controller as:

```
reference         = end (front or back)
partner           = right
tangent_direction = locked RIGHT
tangent_speed     = strafe_speed_ms
```

The modes are:

1. **HOLD-END-LOST** — the current front/back reference is missing beyond `line_bridge_s`; do not move right.
2. **SLIDE-RIGHT + hold end band** — choose the tangent to `end` with the largest dot against `(rx, ry)`, move at `strafe_speed_ms`, and add `band_term(end)`. This remains active while both lines are outside the band or until a specific alignment case applies.
   ```
   t1 = (-end.ey,  end.ex)
   t2 = ( end.ey, -end.ex)
   (tx, ty) = tangent with max(dot(tangent, (rx, ry)))

   end_v  = band_term(end)
   _sp[0] += (tx × strafe_speed + end_v.x) × dt
   _sp[1] += (ty × strafe_speed + end_v.y) × dt
   ```
3. **APPROACH-RIGHT-INTO-BAND** — `end` is in-band, `right` is gate-latched but outside 0.5–1.2 m. Stop tangent travel and apply `band_term(right) + band_term(end)`, capped at `corner_speed_ms`.
4. **APPROACH-END-INTO-BAND** — `right` is in-band but `end` is outside. Stop tangent travel and apply `band_term(end) + band_term(right)`, capped at `corner_speed_ms`.
5. **BOTH-IN-BAND** — end + right are simultaneously inside 0.5–1.2 m. Stop tangent travel, set `_corner_at_target = True`, and keep both under gentle band control.
6. **END-FLICKER** — the end reference disappears for ≤ `line_bridge_s`; freeze `_sp`, log the duration, and then enter HOLD-END-LOST if it does not return.

Additional partner-loss guard: after the right gate has latched, a missing right line also freezes `_sp`; the drone never continues a rightward tangent toward a boundary it can no longer measure.

Every mode except BOTH-IN-BAND sets `_corner_at_target = False` and clears `_corner_candidate_since`.

After `_corner_at_target` remains true for `corner_confirm_s`:

```
_pending_corner_number = 3
_pending_line_a         = _step_end_role
_pending_line_b         = "right"
_pending_corner_reason  = f"{_step_end_role} + right lines in band"
_corner3_end_role       = _step_end_role  # FRONT or BACK — never assumed
_corner_source_phase    = SURVEY_STEP
_after_corner           = FOLLOW_RIGHT_END
```

→ **CORNER_HOLD**. It performs the separate full 5 s validation.

If the hard wall veto blocks the right step but the end line is unavailable beyond `line_bridge_s`, HOLD and begin RETURN rather than reversing blindly or falsely declaring a corner.

**Normal step completion:** only while the right gate is still closed, when `stepped >= stripe_step_m` (2 m):

```
_stripe_dir *= -1
_stripe_count += 1
_acq_start = current pose
_sp        = current pose
```

Because Corner 2 seeds `_stripe_dir = +1`, the first completed step changes it to `-1`, so stripe 1 runs BACKWARD. If `_stripe_count > max_stripes` (12), return home; otherwise → **SURVEY_STRIPE**.

Tele:

```
STEP RIGHT[RIGHT-FAR-IGNORED] X/2.0 end=FRONT/BACK end=Ym right=Zm gate=0.5-1.8m
STEP RIGHT[GATE-CONFIRM] X/2.0 end=Ym right=Zm gate=N/0.5s
STEP RIGHT[SLIDE-RIGHT] end=Ym right=Zm band=0.5-1.2m
STEP RIGHT[APPROACH-RIGHT-INTO-BAND] end=Ym right=Zm
STEP RIGHT[APPROACH-END-INTO-BAND] end=Ym right=Zm
STEP RIGHT[BOTH-IN-BAND] end=Ym right=Zm confirm=N/1.0s
STEP RIGHT[HOLD-END-LOST] no blind right motion
```

The survey progress watchdog is active only during genuine SLIDE-RIGHT / normal step commands, not while gate-confirming, holding, recovering a breach, or band-aligning a corner.

### 16. SURVEY_STRIPE — alternate back / forward until Corner 3

Guards. Early-exit if `_corners_found >= target_corners`. Initialize `_sp` and `_acq_start` from the live pose. Expected end:

```
d = _stripe_dir
end = front if d > 0 else back
```

Only a classified RIGHT line may participate in Corner 3. Never accept the old LEFT boundary or an arbitrary nearest line. Continuously update the right gate, but separate the handler into gate-closed survey and gate-open six-mode follow.

#### RIGHT-GATE-CLOSED — normal survey; far right is ignored

While the expected end line is not in approach range:

```
_sp[0] += d × fx × survey_speed × dt
_sp[1] += d × fy × survey_speed × dt
```

A right line with `dist > right_gate_hi_m` may appear in telemetry, but it contributes zero corner motion and zero corner state. The stripe does not turn diagonally toward it.

When the expected end appears, taper only because of the END line using `_approach(end)` from `slow_dist_m`; use its detected normal for final perpendicular closure. A far right line still has no effect. `stop_dist_m` remains the hard floor.

If the end remains in-band for `reach_confirm_s` while the right gate is closed, it is a plain mid-edge:

```
_step_start    = current pose
_step_end_role = "front" if d > 0 else "back"
_sp            = current pose
```

→ **SURVEY_STEP**. The pattern remains `2 m RIGHT → BACK → 2 m RIGHT → FORWARD → 2 m RIGHT → ...`.

If an in-window right sample begins while already at the end, hold the end long enough to finish `right_gate_confirm_s`; do not race into the next phase during that short confirmation. If confirmation fails, proceed to SURVEY_STEP.

#### RIGHT-GATE-OPEN — six-mode right follow to whichever end is active

If the 0.5–1.8 m gate latches during a stripe (including recovery from a detector miss during the preceding step), map the Corner 2 controller as:

```
reference         = right
partner           = end = front if d > 0 else back
tangent_direction = d × locked FORWARD
tangent_speed     = survey_speed_ms
```

This mapping automatically handles Corner 3 being ahead or behind:

- `_stripe_dir > 0` → follow right FORWARD; Corner 3 candidate is FRONT + RIGHT.
- `_stripe_dir < 0` → follow right BACKWARD; Corner 3 candidate is BACK + RIGHT.

The six modes are:

1. **HOLD-RIGHT-LOST** — right reference missing beyond `line_bridge_s`; do not continue the stripe blind.
2. **SLIDE-FWD/BACK + hold right band** — choose the right-line tangent aligned with `d × (fx, fy)`, move at `survey_speed_ms`, and add `band_term(right)`. A visible front/back line farther than `corner_side_reach_m` is logged but ignored as a corner partner.
   ```
   t1 = (-right.ey,  right.ex)
   t2 = ( right.ey, -right.ex)
   desired = d × (fx, fy)
   (tx, ty) = tangent with max(dot(tangent, desired))

   right_v = band_term(right)
   _sp[0] += (tx × survey_speed + right_v.x) × dt
   _sp[1] += (ty × survey_speed + right_v.y) × dt
   ```
3. **APPROACH-END-INTO-BAND** — right is in-band and the correct perpendicular end is within `corner_side_reach_m` but outside 0.5–1.2 m. Stop tangent travel and sum `band_term(end) + band_term(right)`.
4. **APPROACH-RIGHT-INTO-BAND** — end is in-band but right is outside. Stop tangent travel and sum `band_term(right) + band_term(end)`.
5. **BOTH-IN-BAND** — active end + right are simultaneously in-band. Stop tangent travel, set `_corner_at_target = True`, and maintain both with gentle band terms.
6. **RIGHT-FLICKER** — right disappears for ≤ `line_bridge_s`; freeze `_sp`, then enter HOLD-RIGHT-LOST if it does not return.

Every mode except BOTH-IN-BAND sets `_corner_at_target = False` and clears `_corner_candidate_since`.

The partner end is usable only when all are true:

```
end role matches d                  # FRONT for +1, BACK for -1
abs(dot(n_end, n_right)) < corner_perp_dot
end.dist <= corner_side_reach_m
```

Therefore a distant visible front/back line cannot pull the drone toward a corner. Until it enters partner reach, mode 2 continues tangent along the right boundary.

After BOTH-IN-BAND holds for `corner_confirm_s`:

```
_pending_corner_number = 3
_pending_line_a         = "front" if d > 0 else "back"
_pending_line_b         = "right"
_pending_corner_reason  = f"{_pending_line_a} + right lines in band"
_corner3_end_role       = _pending_line_a  # derives from live stripe direction
_corner_source_phase    = SURVEY_STRIPE
_after_corner           = FOLLOW_RIGHT_END
```

→ **CORNER_HOLD** for the independent 5 s validation.

Apply `_breach_recover()` first, `_apply_boundary()` last, and publish at `sp_rate_hz`. Run the survey progress watchdog only during actual OPEN or SLIDE-FWD/BACK commands, not during gate confirmation, flicker hold, or band alignment.

Tele:

```
STRIPE#N FWD/BACK[RIGHT-FAR-IGNORED] trav=X/12.0 end=Ym right=Zm gate=0.5-1.8m
STRIPE#N FWD/BACK[GATE-CONFIRM] right=Zm gate=N/0.5s
STRIPE#N FWD/BACK[SLIDE-RIGHT-LINE] right=Ym end=Zm
STRIPE#N[APPROACH-END-INTO-BAND] right=Ym end=Zm
STRIPE#N[APPROACH-RIGHT-INTO-BAND] right=Ym end=Zm
STRIPE#N[BOTH-IN-BAND] right=Ym end=Zm confirm=N/1.0s corner3=FRONT|BACK
STRIPE#N[HOLD-RIGHT-LOST] no blind stripe motion
```

Bailouts: travelled > `max_stripe_m` (12 m) without the expected end line, or `_stripe_count > max_stripes` (12) → return home.

### 17. FOLLOW_RIGHT_END — after Corner 3, follow right to the opposite end for Corner 4

This phase does not resume the lawnmower. `_right_gate_latched` is already true because Corner 3 could not have been validated otherwise. Corner 3 fixes the final travel direction:

- Corner 3 at **FRONT** → `_right_follow_dir = -1`; follow the RIGHT boundary BACKWARD; target = back line.
- Corner 3 at **BACK** → `_right_follow_dir = +1`; follow the RIGHT boundary FORWARD; target = front line.

Map the exact Corner 2 six-mode controller as:

```
reference         = right
partner           = _right_follow_target
tangent_direction = _right_follow_dir × locked FORWARD
tangent_speed     = survey_speed_ms
```

The six modes are:

1. **HOLD-RIGHT-LOST** — right reference missing beyond `line_bridge_s`; hold the live pose. Corner 4 search never continues without the boundary that defines its track.
2. **SLIDE-BACK/FORWARD + hold right band** — select the right-line tangent aligned with `_right_follow_dir × (fx, fy)`, move at `survey_speed_ms`, and add `band_term(right)`.
   ```
   t1 = (-right.ey,  right.ex)
   t2 = ( right.ey, -right.ex)
   desired = _right_follow_dir × (fx, fy)
   (tx, ty) = tangent with max(dot(tangent, desired))

   right_v = band_term(right)
   _sp[0] += (tx × survey_speed + right_v.x) × dt
   _sp[1] += (ty × survey_speed + right_v.y) × dt
   ```
   A target end may be visible from far away at higher altitude. If `target_end.dist > corner_side_reach_m`, report `END-FAR-IGNORED` and keep following right tangent; do not aim diagonally at the distant front/back line.
3. **APPROACH-END-INTO-BAND** — right is in-band and the correct opposite end is within `corner_side_reach_m` but outside 0.5–1.2 m. Stop tangent travel; sum `band_term(target_end) + band_term(right)`, capped at `corner_speed_ms`.
4. **APPROACH-RIGHT-INTO-BAND** — target end is in-band but right has moved outside the band. Stop tangent travel; sum `band_term(right) + band_term(target_end)`.
5. **BOTH-IN-BAND** — right + opposite end simultaneously satisfy 0.5–1.2 m. Stop tangent travel, set `_corner_at_target = True`, and actively hold both under the same dead-zone band controller.
6. **RIGHT-FLICKER** — right disappears for ≤ `line_bridge_s`; freeze `_sp`, log the duration, and enter HOLD-RIGHT-LOST if it does not return.

Every mode except BOTH-IN-BAND sets `_corner_at_target = False` and clears `_corner_candidate_since`.

The opposite end is a usable partner only if it has the correct role, is perpendicular to right under `corner_perp_dot`, and is within `corner_side_reach_m`. A visible line outside that reach cannot change the flight direction or start the candidate timer.

Start `_corner_candidate_since` only while BOTH-IN-BAND remains true; reset immediately if either line leaves the band. After `corner_confirm_s`:

```
_pending_corner_number = 4
_pending_line_a         = "right"
_pending_line_b         = _right_follow_target
_pending_corner_reason  = f"right + {_right_follow_target} lines in band"
_corner_source_phase    = FOLLOW_RIGHT_END
_after_corner           = RETURN
```

→ **CORNER_HOLD**. After its complete 5 s validation, Corner 4 triggers RETURN.

Apply `_breach_recover()` first, `_apply_boundary()` last, and publish at `sp_rate_hz`. The progress watchdog runs only during genuine SLIDE-BACK/FORWARD motion; flicker, HOLD, partner-reach gating, and band alignment are intentional non-progress states.

Tele:

```
FOLLOW_RIGHT_END[BACK][END-FAR-IGNORED] trav=X/12.0 right=Ym back=Zm
FOLLOW_RIGHT_END[FWD][END-FAR-IGNORED] trav=X/12.0 right=Ym front=Zm
FOLLOW_RIGHT_END[SLIDE-RIGHT-LINE] dir=BACK|FWD right=Ym end=Zm band=0.5-1.2m
FOLLOW_RIGHT_END[APPROACH-END-INTO-BAND] right=Ym end=Zm
FOLLOW_RIGHT_END[APPROACH-RIGHT-INTO-BAND] right=Ym end=Zm
FOLLOW_RIGHT_END[BOTH-IN-BAND] right=Ym end=Zm confirm=N/1.0s
FOLLOW_RIGHT_END[RIGHT-FLICKER] lost=Ts/1.0s hold-position
FOLLOW_RIGHT_END[HOLD-RIGHT-LOST] no blind motion
```

Bailout: travelled > `max_stripe_m` (12 m) without finding the opposite end → return home.

### Survey progress watchdog — finite recovery from a stuck sweep

The watchdog runs only in the genuine-motion portions of SURVEY_STEP, SURVEY_STRIPE, and FOLLOW_RIGHT_END. It is disabled during right-gate confirmation, reference/partner flicker, HOLD, corner alignment/hold, `_breach_recover()`, detector-stale hold, normal approach taper, or any command intentionally clamped by `_apply_boundary()`. `RIGHT-FAR-IGNORED` does not disable it because the survey is still intentionally moving at normal speed.

On every new non-retry movement segment or direction change, reset `_survey_stall_retries = 0` and capture:

```
_last_motion_kind = "right" | "forward" | "backward"
_last_motion_vec  = corresponding locked ENU unit vector
_stall_anchor_xy  = current actual pose
_stall_since      = now
```

Measure actual progress, not detector response:

```
progress = dot(current_pose_xy - _stall_anchor_xy, _last_motion_vec)
```

No yellow line in open space is normal and is NOT a stall. If `progress >= survey_stall_min_progress_m`, advance the anchor and reset the timer. A stall is declared only when a non-zero motion command has remained active, no intentional hold/taper/veto applies, progress stays below threshold, and elapsed time reaches `survey_stall_timeout_s` (10 s).

Recovery sequence:

1. Preserve the current phase plus `_last_motion_kind` / `_last_motion_vec`.
2. Reset `_sp` to the live pose so an unreachable accumulated setpoint cannot cause a later lunge.
3. Publish zero velocity for `recover_settle_s` (1 s).
4. Re-run guards in priority: CH5 kill → SAFE_MANUAL; VIO fault → FLOW_HOLD; stale boundary → existing stale hold/return; active wall veto or a newly visible end line → normal line handling, not a retry.
5. If healthy, increment `_survey_stall_retries` and reissue the remembered direction from the live pose at `min(normal_phase_speed, corner_speed_ms)`.
6. Once projected progress reaches `survey_stall_min_progress_m`, restore normal speed, reset `_survey_stall_retries = 0`, and reset the watchdog window. A retry itself does not clear the count before progress is proven.
7. The retry continues only until normal line acquisition or the existing `max_stripe_m` / `max_stripes` bailout. It never runs indefinitely "until yellow".
8. If `_survey_stall_retries > survey_stall_max_retries`, log the unresolved stall and begin RETURN. If VIO is unhealthy, use FLOW_HOLD / LAND instead of navigating home on bad pose.

Logs:

```
SURVEY STALL: BACKWARD commanded, progress=0.08m/0.20m in 10.0s — hold + health check.
SURVEY STALL retry 1/2: resuming BACKWARD from live pose at 0.15m/s.
SURVEY progress restored — normal stripe speed 0.20m/s.
SURVEY STALL unresolved after 2 retries — returning home.
```

### 18. RETURN

Entered via `_begin_return()` from many places — the "give up and go home" fallback.

Setup:

```
_ret_sp = current pose
_ret_start = now
_ret_arrived_since = None
```

Every tick, crawl `_ret_sp` toward HOME:

```
step = forward_speed / sp_rate_hz
f    = min(1.0, step / distance_to_home)
_ret_sp += (home - _ret_sp) × f
```

Publish `_ret_sp` at `cruise_alt`. PX4 tracks the moving target smoothly instead of lunging.

Bailouts:

- VIO fault → LAND (flow handles the descent from here).
- Return timeout `goto_timeout_s` (60 s) → LAND.

Arrival: `err ≤ goto_radius` (0.2 m) held for 1 s:

- Call `/viman/gate false` → camera stops feeding PX4 AT ALTITUDE, so the EKF's flow transition happens above ground where drift is survivable.
- Start `_settle_start` timer.
- Log: `Home reached — camera off, flow settle.`
- → **FLOW_SETTLE**.

### 19. FLOW_SETTLE

Publish HOME setpoint at `cruise_alt`. Wait `flow_settle_s` (2.5 s) for the EKF to finish its VIO → flow transition at altitude.

Capture current z as `_desc_z`. → **DESCEND**.

### 20. DESCEND — OFFBOARD precision descent

Ramps `_desc_z` down at `descend_speed_ms` (0.25 m/s):

```
_desc_z = max(0, _desc_z - descend_speed / sp_rate_hz)
publish (home_x, home_y, _desc_z)
```

X/Y are actively locked on HOME the whole way down — AUTO.LAND alone drifts laterally, this doesn't.

Handoff to AUTO.LAND when `alt ≤ descend_handoff_alt_m` (0.3 m) OR descent timeout `descend_timeout_s` (15 s).

→ **LAND**.

### 21. FLOW_HOLD — VIO fault recovery

Entered from many places when `_vio_fault()` fires. `_resume_phase` was saved at entry.

Invalidate all cached line geometry. If the banked phase is SURVEY_STEP or SURVEY_STRIPE before Corner 3, clear `_right_gate_latched` and `_right_gate_since`; after HANDOVER the live right line must again pass the complete 0.5–1.8 m gate. If Corner 3 was already counted and the banked phase is FOLLOW_RIGHT_END, preserve `_corner3_end_role` / travel direction but clear the right-line cache and HOLD until a fresh classified right reference returns. Never resume from a stale high-altitude line measurement.

Publishes zero velocity for `stable_of_secs` (4 s) — pure flow settle.

Then:

- Increment `_revalidations`.
- If `> max_revalidations` (6) → LAND.
- Otherwise:
  - Log: `Re-validation N/6 — will resume PHASE.`
  - Reset `_seed_sent`.
  - → **SEED**.

The mission then loops through SEED → VALIDATE → HANDOVER, and HANDOVER's re-entry branch jumps to `_resume_phase`.

### 22. LAND

If gate not closed yet → call `/viman/gate false` (belt and braces — RETURN already did this, but if we got here directly from a fault we need to do it now).

Request `SetMode("AUTO.LAND")` once. → **DISARM**.

### 23. DISARM

Waits for PX4 to self-disarm after touchdown (`state.armed == False`).

On disarm:

- Log: `Disarmed — corner mission complete.`
- → **DONE** (terminal, no-op).

### 24. SAFE_MANUAL

Entered from any airborne phase when CH5 ≥ `rc_interrupt_high` (1700). Also entered if PX4 changed modes externally during boundary_guard interactions.

Sets `SetMode("STABILIZED")` — pilot has full manual control.

Logs `SAFE MANUAL — pilot has control.` every 5 s. State machine does nothing else; pilot flies the drone home manually. Mission is over.

---

## Parameter reference (from `mission_params.yaml` → `boundary_test_auto:`)

### Flight profile

| param | value | meaning |
|---|---|---|
| `takeoff_alt` | 2.0 | flow-only camera-init altitude (m) |
| `cruise_alt`  | 3.0 | fused-VIO corner-search and lawnmower altitude (m); distant tape visibility is handled by the explicit right/partner acquisition gates |
| `climb_speed_ms` | 0.2 | gentle VIO climb speed |
| `alt_tolerance` | 0.12 | ±m band for "at altitude" |
| `at_alt_confirm_s` | 1.5 | must hold altitude this long |
| `stable_of_secs` | 4.0 | flow settle before seeding |

### Yaw

| param | value | meaning |
|---|---|---|
| `yaw_use_arm_heading` | `true` | AUTO style — hold arm-time heading. `false` = fixed compass. |
| `mission_yaw_deg` | 90.0 | fixed target if arm heading disabled (ENU pose-yaw) |
| `yaw_slew_dps` | 15.0 | max yaw rotation rate |
| `yaw_align_tol_deg` | 3.0 | tighten before locking `_cruise_yaw` to actual settled yaw |
| `yaw_align_hold_s` | 1.0 | hold aligned this long before seed |
| `yaw_align_timeout_s` | 25.0 | give up aligning, proceed anyway |

### VIO seed / validate / handover

| param | value | meaning |
|---|---|---|
| `seed_timeout_s` | 10.0 | seed → validating deadline |
| `validate_if_min` | 0.7 | IF ≥ this required |
| `validate_hold_s` | 5.0 | IF must hold this long |
| `validate_dip_grace_s` | 1.0 | brief dips tolerated |
| `validate_timeout_s` | 60.0 | validation deadline |
| `motion_test` | `true` | run motion square during VALIDATE |
| `motion_amp_m` | 0.2 | square side length |
| `motion_leg_s` | 4.0 | seconds per leg |
| `handover_settle_s` | 2.0 | gate-open settle time |

### Boundary approach

| param | value | meaning |
|---|---|---|
| `forward_speed_ms` | 0.15 | ACQ_BACK forward search speed |
| `strafe_speed_ms` | 0.2 | ACQ_LEFT / STEP strafe speed |
| `corner_speed_ms` | 0.15 | corner settle speed |
| `slow_dist_m` | 1.2 | begin braking here |
| `stop_dist_m` | 0.5 | SOFT standoff — never closer than this |
| `push_gain` | 0.6 | breach-recover retreat gain |
| `push_speed_max_ms` | 0.2 | breach-recover cap |
| `recover_margin_m` | 0.25 | safe distance = stop + this |
| `recover_settle_s` | 1.0 | hold at safe distance this long |
| `breach_floor_m` | 0.20 | ignore ultra-close phantom readings below this |
| `ahead_thresh` | 0.3 | line counts as "front" if fwd alignment ≥ this |
| `left_thresh` | 0.3 | same for "left" |
| `corner_gain` | 0.8 | band-controller gain (ease speed ∝ distance outside band) |
| `corner_confirm_s` | 1.0 | both lines must read in-band this long before the hold starts |
| `veto_deadband_m` | 0.15 | wall-hold slack (kills chatter) |
| `max_forward_m` | 6.0 | ACQ_BACK bailout |
| `max_strafe_m` | 6.0 | ACQ_LEFT bailout |
| `band_lo_m` (`acq_left_lo_m`) | 0.5 | **corner band LOW** — never closer than this to a line |
| `band_hi_m` (`acq_left_hi_m`) | 1.2 | **corner band HIGH** — a line counts as "reached / in-band" at ≤ this |
| `acq_left_back_lost_s` | 1.5 | HOLD if back lost longer than this |

> **The 0.5–1.2 m band is the heart of the corner logic.** Every corner (1–4) is validated only when both meeting lines sit inside `[band_lo_m, band_hi_m]` at the same time. `stop_dist_m` (0.5 m) is still the hard floor the wall veto enforces; `band_hi_m` (1.2 m) is the far edge of the acceptable window and equals `slow_dist_m`.

### Corner detection & counting

| param | value | meaning |
|---|---|---|
| `corner_side_reach_m` | 1.5 | after the reference boundary is acquired, the perpendicular front/back partner must be within this reach before its alignment mode may override tangent travel; counting still requires 0.5–1.2 m |
| `corner_dedup_m` | 1.2 | new corner must be > this from every counted one |
| `target_corners` | 4 | mission ends after this many |
| `corner_perp_dot` | 0.5 | `|n1·n2|` below this = perpendicular (the two lines meet) |
| `right_gate_lo_m` | 0.5 | low edge of the Corner 3 RIGHT-boundary acquisition window; below this invokes breach recovery, not corner acquisition |
| `right_gate_hi_m` | 1.8 | far edge of RIGHT acquisition; a visible right line beyond this is ignored by sweep/corner decisions |
| `right_gate_confirm_s` | 0.5 | correctly classified right line must remain inside 0.5–1.8 m this long before `_right_gate_latched = True` |

> `right_gate_hi_m` is deliberately wider than `band_hi_m`: 1.8 m permits safe acquisition at higher altitude, while the actual Corner 3 / 4 validation band remains 0.5–1.2 m. The three `right_gate_*` entries are new mission parameters and must be declared in `corner1_test_auto.py` plus added under `boundary_test_auto:` in `mission_params.yaml` when implemented.

### Lawnmower survey

| param | value | meaning |
|---|---|---|
| `survey_speed_ms` | 0.2 | FOLLOW_LEFT_FWD, normal forward/back stripe, right-gate-open stripe follow, and FOLLOW_RIGHT_END tangent speed |
| `stripe_step_m` | 2.0 | right shift that starts the sweep after Corner 2 and separates later stripes |
| `max_stripe_m` | 12.0 | stripe bailout (must be > arena depth) |
| `max_stripes` | 12 | stripe count safety cap |
| `acq_settle_s` | 1.0 | settle at reached line/corner |
| `reach_confirm_s` | 0.5 | confirm transition this long |
| `line_bridge_s` | 1.0 | detector-dropout grace for left/right/end reference lines; advancing setpoints freeze during Corner 2 / 3 / 4 reference flicker |
| `survey_stall_timeout_s` | 10.0 | active sweep command may show insufficient projected pose progress for this long before recovery |
| `survey_stall_min_progress_m` | 0.20 | minimum actual progress along the remembered motion direction required to reset the 10 s watchdog |
| `survey_stall_max_retries` | 2 | bounded same-direction retries before abandoning the sweep and returning home |

> These three `survey_stall_*` entries are new mission parameters. They must be declared by `corner1_test_auto.py` and added under `boundary_test_auto:` in `mission_params.yaml` when the documented behavior is implemented; otherwise the watchdog cannot be tuned from YAML.

### Corner hold

| param | value | meaning |
|---|---|---|
| `corner_hold_s` (`corner1_hold_s`) | 5.0 | in-band hold at EVERY corner (1–4) before moving on |
| `hover_duration` | 5.0 | legacy alias |

### Return / land

| param | value | meaning |
|---|---|---|
| `goto_radius_m` | 0.2 | arrival radius |
| `goto_timeout_s` | 60.0 | return timeout |
| `flow_settle_s` | 2.5 | camera-off EKF settle |
| `descend_speed_ms` | 0.25 | precision descent rate |
| `descend_handoff_alt_m` | 0.3 | handoff to AUTO.LAND altitude |
| `descend_timeout_s` | 15.0 | descent timeout |

### Recovery + fail-safe

| param | value | meaning |
|---|---|---|
| `max_revalidations` | 6 | flow-hold retries before landing |
| `boundary_stale_s` | 1.0 | detector silent → hold |
| `stale_land_s` | 5.0 | detector silent too long → return + land |

### Infra

| param | value | meaning |
|---|---|---|
| `sp_rate_hz` | 20.0 | setpoint publish rate |
| `rc_ch5_index` | 4 | RC channel 5 (0-based) |
| `rc_start_low` | 1200 | CH5 PWM ≤ this → start |
| `rc_interrupt_high` | 1700 | CH5 PWM ≥ this → pilot takeover |
| `preflight_pose_hz_min` | 15.0 | minimum pose rate |
| `yellow_log_period_s` | 1.0 | terminal yellow readout period |

---

## Topics used

**Subscribed:**

- `/mavros/state` (MAVROS State)
- `/mavros/local_position/pose` (PoseStamped, ENU)
- `/mavros/rc/in` (RCIn)
- `/rtabmap/rtabmap/odom` (Odometry) — aliveness only
- `/viman/vio_state` (UInt8) — gate state 0-6
- `/viman/init_factor` (Float32) — Q × A × S
- `/viman/boundary/repulsion` (Vector3Stamped) — blended push-away, body FLU
- `/viman/boundary/nearest_m` (Float32) — nearest yellow distance
- `/viman/boundary/coverage_pct` (Float32) — % of frame that is yellow
- `/viman/boundary/lines` (Float32MultiArray) — `[n, (dist, nx, ny, strength) × n]`, body FLU
- `/viman/boundary/corner` (Vector3Stamped) — L-corner vertex, body FLU, z=1.0 if visible

**Published:**

- `/mavros/setpoint_position/local` (PoseStamped) — position + yaw
- `/mavros/setpoint_velocity/cmd_vel` (TwistStamped) — zero-velocity holds

**Services called:**

- `/mavros/cmd/arming` (CommandBool)
- `/mavros/set_mode` (SetMode)
- `/viman/seed` (Trigger)
- `/viman/gate` (SetBool)

---

## Log lines you'll see (in flight order)

```
== Phase: IDLE ==
PREFLIGHT BLOCKED: FCU, pose rate, no RC, RTAB silent, vio_gate silent, boundary detector silent
Preflight OK - flip CH5 HIGH once then LOW to start
== Phase: ARM ==
Armed. HOME=(0.02,0.01) ARM-TIME heading = 70.0 deg -> will hold ARM-TIME heading at home (like the survey).
== Phase: TAKEOFF ==
TAKEOFF(flow) 1.94/2.0 m
== Phase: STABLE_OF ==
== Phase: HOVER_HOME ==
HOVER_HOME dist=0.18 m -> home
At home, settled. Slewing onto heading 70.0 deg, then holding until aligned before seeding.
HOVER_HOME yaw=70.8 -> 70.0 deg (err=0.8 deg)
Yaw locked at ACTUAL 70.0 deg (target was 70.0, aligned). Mission frame anchored here — forward / back / left / right now fly straight along the drone's real body axes. Seeding.
== Phase: SEED ==
Gate state: UNSEEDED → SEEDING
Gate state: SEEDING → VALIDATING
== Phase: VALIDATE ==
VALIDATE IF=0.95 (>=0.70)  t=1s
== Phase: HANDOVER ==
Using motion-calibrated rotation ✓
Gate state: VALIDATING → OPEN
Gate OPEN - climbing to 3.0 m.
== Phase: CLIMB ==
CLIMB(VIO) 2.00 -> 3.0 m
At 3.00 m - flying BACKWARD from centre to find the back line (first boundary).
== Phase: ACQ_BACK ==
ACQ_BACK[OPEN] trav=0.36/6.0 ...
ACQ_BACK[FOLLOW-PERP] trav=0.78/6.0 ...
Back line acquired (~0.55 m) — entering YELLOW-LINE-AREA, following back line LEFT to find Corner 1.
== Phase: ACQ_LEFT ==
ACQ_LEFT[SLIDE-LEFT] strafe=0.46/6.0 back=0.85m left=--- band=0.5-1.2m
ACQ_LEFT[APPROACH-LEFT-INTO-BAND] strafe=1.24/6.0 back=0.90m left=1.55m band=0.5-1.2m
ACQ_LEFT[BOTH-IN-BAND] strafe=1.40/6.0 back=0.80m left=0.95m band=0.5-1.2m
Corner 1 (back-left) confirmed: back 0.80 m, left 0.95 m (both in 0.5-1.2 m band).
== Phase: CORNER1_HOLD ==
CORNER1 hold 3/5s back=0.82m left=0.93m band=0.5-1.2m
★ CORNER 1/4 found (back + left lines in band) at (X, Y).
1/4 corner detected
Corner 1 hold complete (5 s) - following the LEFT line FORWARD to find Corner 2. Survey has NOT started.
== Phase: FOLLOW_LEFT_FWD ==
FOLLOW_LEFT_FWD[SLIDE-FORWARD] trav=1.20/12.0 left=0.90m front=--- band=0.5-1.2m
FOLLOW_LEFT_FWD[APPROACH-FRONT-INTO-BAND] trav=2.40/12.0 left=0.92m front=1.42m band=0.5-1.2m
FOLLOW_LEFT_FWD[BOTH-IN-BAND] trav=2.62/12.0 left=0.90m front=1.05m confirm=1.0/1.0s
Corner 2 (front-left): both lines in band (front 1.05 m, left 0.90 m, band 0.5-1.2 m) - holding 5 s to confirm.
== Phase: CORNER_HOLD ==
CORNER2 hold 3/5s front=1.02m left=0.91m band=0.5-1.2m
★ CORNER 2/4 found (front + left lines in band) at (X, Y).
2/4 corner detected
Corner 2 hold complete (5 s) - starting sweep: 2.00 m RIGHT holding FRONT, then BACKWARD.
== Phase: SURVEY_STEP ==
STEP RIGHT[RIGHT-FAR-IGNORED] 1.70/2.00 end=FRONT end=0.95m right=4.60m gate=0.5-1.8m
Stepped 2.00 m RIGHT - stripe #1 BACKWARD.
== Phase: SURVEY_STRIPE ==
STRIPE#1 BACK[RIGHT-FAR-IGNORED] trav=1.20/12.0 end=--- right=4.58m gate=0.5-1.8m
STRIPE#1 BACK[RIGHT-FAR-IGNORED] trav=3.10/12.0 end=0.95m right=4.55m mode=END-APPROACH
Back edge reached; RIGHT is outside acquisition gate - stepping 2.00 m RIGHT, then FORWARD.
== Phase: SURVEY_STEP ==
STEP RIGHT[RIGHT-FAR-IGNORED] 2.00/2.00 end=BACK end=0.92m right=2.55m gate=0.5-1.8m
== Phase: SURVEY_STRIPE ==
STRIPE#2 FWD[RIGHT-FAR-IGNORED] trav=2.80/12.0 end=1.00m right=2.52m gate=0.5-1.8m
Front edge reached; RIGHT is still outside gate - stepping RIGHT again.
== Phase: SURVEY_STEP ==
STEP RIGHT[GATE-CONFIRM] end=0.98m right=1.74m gate=0.5/0.5s
RIGHT gate OPEN at 1.74m - applying six-mode FRONT-reference + RIGHT-partner approach.
STEP RIGHT[APPROACH-RIGHT-INTO-BAND] end=0.98m right=1.42m
STEP RIGHT[BOTH-IN-BAND] end=0.98m right=1.10m confirm=1.0/1.0s
== Phase: CORNER_HOLD ==
CORNER3 hold 4/5s front=0.99m right=1.08m band=0.5-1.2m
★ CORNER 3/4 found (front + right lines in band) at (X, Y).
3/4 corner detected
Corner 3 found at FRONT - following RIGHT boundary BACKWARD to find Corner 4.
== Phase: FOLLOW_RIGHT_END ==
FOLLOW_RIGHT_END[BACK][END-FAR-IGNORED] trav=1.60/12.0 right=1.02m back=3.80m
FOLLOW_RIGHT_END[SLIDE-RIGHT-LINE] dir=BACK right=1.01m back=1.62m band=0.5-1.2m
FOLLOW_RIGHT_END[APPROACH-END-INTO-BAND] right=1.01m back=1.38m
FOLLOW_RIGHT_END[BOTH-IN-BAND] right=1.02m back=1.08m confirm=1.0/1.0s
== Phase: CORNER_HOLD ==
CORNER4 hold 5/5s right=1.02m back=1.08m band=0.5-1.2m
★ CORNER 4/4 found (right + back lines in band) at (X, Y).
4/4 corner detected
All 4 corners found - returning home.
== Phase: RETURN ==
RETURN dist=1.24 m -> home
Home reached - camera off, flow settle
== Phase: FLOW_SETTLE ==
== Phase: DESCEND ==
DESCEND alt=0.42 -> 0.30 m
== Phase: LAND ==
== Phase: DISARM ==
Disarmed - corner mission complete.
```

The example above shows Corner 3 found at the FRONT. If it is found at the BACK instead, the branch log is:

```
RIGHT gate OPEN during BACKWARD survey - Corner 3 target role = BACK.
STRIPE#N BACK[SLIDE-RIGHT-LINE] right=Ym back=Zm
STRIPE#N BACK[BOTH-IN-BAND] right=Ym back=Zm confirm=1.0/1.0s
Corner 3 found at BACK - following RIGHT boundary FORWARD to find Corner 4.
FOLLOW_RIGHT_END[FWD][END-FAR-IGNORED] trav=X/12.0 right=Ym front=Zm
FOLLOW_RIGHT_END[SLIDE-RIGHT-LINE] dir=FWD right=Ym front=Zm band=0.5-1.2m
FOLLOW_RIGHT_END[BOTH-IN-BAND] right=Ym front=Zm confirm=1.0/1.0s
★ CORNER 4/4 found (right + front lines in band) at (X, Y).
4/4 corner detected
```

Conditional survey-stall logs (shown only if actual projected motion fails the watchdog) are:

```
SURVEY STALL: BACKWARD commanded, progress=0.08m/0.20m in 10.0s - hold + health check.
SURVEY STALL retry 1/2: resuming BACKWARD from live pose at 0.15m/s.
SURVEY progress restored - normal stripe speed 0.20m/s.
SURVEY STALL unresolved after 2 retries - returning home.
```

---

## Fault paths you might hit

- **Preflight blocked forever** → check FCU / MAVROS / camera / RTAB / vio_gate / detector are all up before flipping CH5.
- **Seed timeout** → `/viman/seed` might not be routed to `/rtabmap/reset_odom`. Check `ros2 service list | grep reset`.
- **Validation timeout** → tape / floor too low-texture; move validate anchor to richer scene, or lower `validate_if_min`.
- **Gate did not open** → RTAB not converging; check `IF` breakdown `Q A S` in vio_gate log.
- **VIO fault during ACQ / FOLLOW / STEP / STRIPE / CORNER_HOLD** → mission banks the active phase and auto-recovers via FLOW_HOLD. A pending corner keeps its expected line roles but restarts the full 5 s hold after revalidation. If it exhausts `max_revalidations`, it lands where it is.
- **Back line lost during ACQ_LEFT** → HOLD kicks in at 1.5 s; check yellow detector output on the browser MJPEG feed.
- **Left line lost while travelling to Corner 2** → LEFT-FLICKER immediately freezes the advancing setpoint while the loss duration is ≤ `line_bridge_s`; if the line does not return, `left_lost_hold` keeps HOLD active. It never continues forward without the line that defines its track.
- **Far right line is visible and the drone turns toward it early** → it must not. Verify Corner 3 transitions are gated by `right_gate_lo_m <= right.dist <= right_gate_hi_m` held for `right_gate_confirm_s`. A right reading above 1.8 m must log `RIGHT-FAR-IGNORED` and contribute no motion or corner timer.
- **Right gate never opens** → confirm the line is classified against locked RIGHT (not merely selected as nearest), distance actually enters 0.5–1.8 m, and the reading survives `right_gate_confirm_s`. Do not increase `right_gate_hi_m` merely because tape is visible from altitude; first verify depth scale and line normal.
- **Right distance flickers around 1.8 m** → the pre-latch confirmation resets on an out-of-window sample. Once properly latched it does not flap closed; reference-line flicker / HOLD rules take over. If false samples latch it, increase `right_gate_confirm_s` rather than widening the acquisition window.
- **Right line lost after the gate opens or after Corner 3** → freeze the advancing setpoint during `line_bridge_s`, then HOLD. SURVEY_STEP never continues rightward toward an unmeasured latched partner, and FOLLOW_RIGHT_END never continues blind toward Corner 4.
- **Front/back line is visible far away after right acquisition and the drone turns diagonally toward it** → it must remain in SLIDE-FWD/BACK along the right reference. The target end becomes a usable partner only inside `corner_side_reach_m`, with the correct FRONT/BACK role and `corner_perp_dot` test.
- **Never sees a corner arm** → increase `corner_perp_dot` tolerance (up to 0.7 for ~45–135°), or lower detector aspect ratio.
- **Drone freezes at a corner and never moves (the old bug)** → this was caused by exact-point equalization stalling where two gradients cancelled. Every corner path now uses independent range-band terms: a line contributes correction only while outside 0.5–1.2 m and becomes quiet inside it. If it still stops early, confirm the second line is actually published/classified before changing `band_hi_m` or `band_lo_m`.
- **Validates a corner on one line only** → it shouldn't: the hold timer only starts once BOTH meeting lines read in-band. If it does, raise `corner_confirm_s` so a transient single-line reading can't trip it, and verify the second line is being classified (not filtered out by aspect ratio).
- **Corner hold feels too short / too long** → tune `corner_hold_s` (default 5 s, applies to all 4 corners).
- **Survey starts by going backward instead of stepping right** → Corner 2 must transition to SURVEY_STEP with `_stripe_dir = +1` and `_stripe_count = 0`; only after the 2 m step does the direction reverse to `-1` for stripe 1 BACKWARD.
- **Sweep step slows merely because a far right line is visible** → remove any pre-gate right-distance brake. While `right.dist > right_gate_hi_m`, normal SURVEY_STEP uses full `strafe_speed_ms` plus end-line band hold; only wall veto / breach recovery may override it.
- **Sweep step wrong** → `stripe_step_m` is the right shift between stripes (default 2 m). SURVEY_STEP holds `_step_end_role` in-band; it leaves the normal 2 m pattern only after the confirmed right gate enables the six-mode Corner 3 approach.
- **Corner 3 found but drone continues lawnmower stepping** → it should not. Corner 3 at FRONT must transition to FOLLOW_RIGHT_END BACKWARD; Corner 3 at BACK must transition to FOLLOW_RIGHT_END FORWARD. No further 2 m right step is commanded after Corner 3.
- **VIO fault occurs while the right gate is latched before Corner 3** → FLOW_HOLD clears the latch and cached geometry. After revalidation the live right line must pass the complete 0.5–1.8 m acquisition gate again; never resume from the stale pre-fault distance.
- **Survey commands movement but actual pose does not progress** → the watchdog waits `survey_stall_timeout_s` (10 s), resets the setpoint to the live pose, holds for `recover_settle_s`, health-checks VIO / detector / wall veto, and retries the remembered direction at `corner_speed_ms`. After `survey_stall_max_retries` (2), it returns instead of retrying forever. If a wall veto or approach taper is active, this is intentional and must not be diagnosed as a stall.
- **Boundary detector silent** → holds; too long → returns and lands. Check the detector's own status table on the terminal or the MJPEG stream.

---

## Build + fly

```bash
cd ~/drone_ws
colcon build --packages-select viman_mission --symlink-install
source install/setup.bash
```

Terminal 1 — MAVROS (however you normally start it).

Terminal 2:

```bash
ros2 launch viman_mission corner1.launch.py
```

Face the drone the way you want "forward" to mean, then flip CH5 HIGH once and back LOW to arm. The mission handles the rest.
