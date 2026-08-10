#!/usr/bin/python3
"""
Autonomous Hover & Land v2 — IRoC-U 2026, Team Viman Rakshak
============================================================
Clean rewrite with correct OFFBOARD sequencing and minimal drift.

Uses the FULL navigation stack for position hold:
  D455 camera → RTAB-Map → vision_bridge → /mavros/vision_pose/pose → EKF2
  MTF-01 optical flow → EKF2 (velocity)
This script reads the vision+flow-fused LOCAL_POSITION_NED from PX4 and
commands position setpoints. The better the vision fusion, the less drift.

Mission:
  1. Wait for CH5 pos 3 trigger (3 s abort window)
  2. EKF health check
  3. Wait for vision_pose to be active (so hold uses camera, not just flow)
  4. Stream setpoints → OFFBOARD → ARM   (correct order — motors spin)
  5. Re-capture HOME after arm (EKF origin resets on arm)
  6. Stepped climb to TARGET_ALT_M
  7. Hold for HOVER_SECS (logs pos + vel)
  8. Stepped descent + force-disarm

Safety:
  - CH5 off pos 3      → release control, drone stays in RC-selected mode, exit
  - Ctrl+C             → controlled descent then exit
  - EKF diverged       → controlled descent
  - Vision lost mid-air→ warn, continue on flow (hold still works, more drift)

NED frame: z negative when airborne.  alt = HOME_Z - ned_z
"""

import time
import sys
import subprocess
import shutil
import os
from pymavlink import mavutil

# ─────────────────────────────────────────────
#  CONFIG
# ─────────────────────────────────────────────
MAVLINK_PORT  = "udpin:0.0.0.0:14540"
BAUD          = 921600

TARGET_ALT_M  = 2.5     # hover altitude (m)
HOVER_SECS    = 20      # hold duration (s)

TAKEOFF_TIMEOUT_S = 60
LAND_TIMEOUT_S    = 60
ARM_TIMEOUT_S     = 10

# Stepped motion
STEP_M          = 0.5   # m per step
STEP_HZ         = 20    # setpoint rate (Hz) — high rate keeps OFFBOARD alive
FOLLOW_TOL      = 0.15  # advance step once within this of current setpoint (m)
ARRIVE_DEADBAND = 0.15  # arrival tolerance (m)
GROUND_M        = 0.15  # alt below this = on ground

# RC CH5 thresholds (3-pos switch): 1=Stabilize 2=POSCTL 3=Script
CH5_LOW_MAX = 1300
CH5_MID_MAX = 1700

# Vision fusion wait
VISION_WAIT_S      = 50    # how long to wait for vision_pose before arming
REQUIRE_VISION     = True  # if False, fly on optical flow only

# EKF
MAX_EKF_ERROR = 0.8

# PX4 custom mode IDs
PX4_OFFBOARD_MAIN, PX4_OFFBOARD_SUB = 6, 0
PX4_LAND_MAIN,     PX4_LAND_SUB     = 3, 6
PX4_AUTO_MAIN = 3
PX4_TAKEOFF_SUB = 2
PX4_LOITER_SUB  = 3
PX4_LAND_AUTO_SUB = 6

# Global home (NED, captured after arm)
HOME_X = HOME_Y = HOME_Z = 0.0


# ─────────────────────────────────────────────
#  CONNECTION
# ─────────────────────────────────────────────
def connect():
    print(f"[CONNECT] {MAVLINK_PORT}")
    mav = mavutil.mavlink_connection(MAVLINK_PORT, baud=BAUD)
    mav.wait_heartbeat(timeout=15)
    mav.target_system = 1          # force PX4 sysid
    print(f"[CONNECT] Heartbeat — sysid={mav.target_system}")
    mav.mav.request_data_stream_send(
        mav.target_system, mav.target_component,
        mavutil.mavlink.MAV_DATA_STREAM_ALL, 20, 1)
    return mav


# ─────────────────────────────────────────────
#  POSITION HELPERS (always read freshest)
# ─────────────────────────────────────────────
def get_latest_position(mav, wait=True):
    """Drain buffer → newest LOCAL_POSITION_NED. Avoids stale readings."""
    latest = None
    while True:
        m = mav.recv_match(type="LOCAL_POSITION_NED", blocking=False)
        if m is None:
            break
        latest = m
    if latest is None and wait:
        latest = mav.recv_match(type="LOCAL_POSITION_NED", blocking=True, timeout=1)
    return latest


def alt_of(ned_z):
    return HOME_Z - ned_z


def capture_home(mav, n=5):
    """Average n samples into HOME_X/Y/Z. Call AFTER arm (EKF resets on arm)."""
    global HOME_X, HOME_Y, HOME_Z
    print("[HOME] Capturing origin ...")
    xs, ys, zs = [], [], []
    deadline = time.time() + 3.0
    while time.time() < deadline and len(zs) < n:
        m = get_latest_position(mav)
        if m:
            xs.append(m.x); ys.append(m.y); zs.append(m.z)
        time.sleep(0.1)
    if zs:
        HOME_X = sum(xs)/len(xs)
        HOME_Y = sum(ys)/len(ys)
        HOME_Z = sum(zs)/len(zs)
    print(f"[HOME] X={HOME_X:.3f} Y={HOME_Y:.3f} Z={HOME_Z:.3f}")


# ─────────────────────────────────────────────
#  SETPOINT
# ─────────────────────────────────────────────
def send_sp(mav, alt_m):
    """Position setpoint, X/Y locked to home, Z = home - alt."""
    mav.mav.set_position_target_local_ned_send(
        int(time.time()*1000) & 0xFFFFFFFF,
        mav.target_system, mav.target_component,
        mavutil.mavlink.MAV_FRAME_LOCAL_NED,
        0b0000111111111000,                 # position only
        HOME_X, HOME_Y, HOME_Z - alt_m,
        0,0,0, 0,0,0, 0,0)


def stream_sp(mav, alt_m, secs):
    """Stream a setpoint at STEP_HZ for `secs` seconds."""
    end = time.time() + secs
    dt  = 1.0 / STEP_HZ
    while time.time() < end:
        send_sp(mav, alt_m)
        time.sleep(dt)


# ─────────────────────────────────────────────
#  RC
# ─────────────────────────────────────────────
def ch5_pwm(mav):
    m = mav.recv_match(type="RC_CHANNELS", blocking=True, timeout=1)
    return m.chan5_raw if m else None


def ch5_pos(mav):
    p = ch5_pwm(mav)
    if p is None:
        return None
    if p <= CH5_LOW_MAX:
        return 1
    if p <= CH5_MID_MAX:
        return 2
    return 3


def pilot_took_over(mav):
    return ch5_pos(mav) not in (3, None)


# ─────────────────────────────────────────────
#  MODE / ARM
# ─────────────────────────────────────────────
def set_mode(mav, main, sub, label):
    mav.mav.command_long_send(
        mav.target_system, mav.target_component,
        mavutil.mavlink.MAV_CMD_DO_SET_MODE,
        0, 209, main, sub, 0, 0, 0, 0)
    print(f"[MODE] → {label}")
    time.sleep(0.3)


def arm(mav):
    print("[ARM] ...")
    mav.mav.command_long_send(
        mav.target_system, mav.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0, 1, 0, 0, 0, 0, 0, 0)
    deadline = time.time() + ARM_TIMEOUT_S
    while time.time() < deadline:
        hb = mav.recv_match(type="HEARTBEAT", blocking=True, timeout=2)
        if hb and (hb.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED):
            print("[ARM] Armed")
            return True
        time.sleep(0.2)
    print("[ARM] FAILED")
    return False


def disarm(mav):
    mav.mav.command_long_send(
        mav.target_system, mav.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0, 0, 21196, 0, 0, 0, 0, 0)
    print("[DISARM] Force-disarm sent")


def land_mode_fallback(mav):
    mav.mav.command_long_send(
        mav.target_system, mav.target_component,
        mavutil.mavlink.MAV_CMD_NAV_LAND,
        0, 0, 0, 0, float('nan'), float('nan'), float('nan'), 0)
    set_mode(mav, PX4_LAND_MAIN, PX4_LAND_SUB, "LAND fallback")


def ekf_ok(mav):
    m = mav.recv_match(type="EKF_STATUS_REPORT", blocking=False)
    if m:
        return m.pos_horiz_variance < MAX_EKF_ERROR
    return True


# ─────────────────────────────────────────────
#  VISION CHECK (camera-based hold)
# ─────────────────────────────────────────────
def wait_for_vision(timeout):
    """Block until /mavros/vision_pose/pose publishes, or timeout."""
    if not shutil.which("ros2"):
        print("[VISION] ros2 CLI not found — skipping vision check")
        return False
    print(f"[VISION] Waiting up to {timeout}s for vision_pose ...")
    deadline = time.time() + timeout
    env = {**os.environ, "ROS_DOMAIN_ID": os.environ.get("ROS_DOMAIN_ID", "0")}
    while time.time() < deadline:
        try:
            r = subprocess.run(
                ["ros2", "topic", "hz", "/mavros/vision_pose/pose", "--window", "5"],
                capture_output=True, text=True, timeout=6, env=env)
            if "average rate" in r.stdout:
                print("[VISION] vision_pose ACTIVE — camera hold enabled")
                return True
        except Exception:
            pass
        print(f"  [VISION] not active ... {deadline-time.time():.0f}s left")
    print("[VISION] vision_pose NOT active")
    return False


# ─────────────────────────────────────────────
#  TAKEOFF — stepped ascent
# ─────────────────────────────────────────────
def takeoff(mav, target_alt):
    print(f"\n[TAKEOFF] → {target_alt} m")
    deadline = time.time() + TAKEOFF_TIMEOUT_S
    dt       = 1.0 / STEP_HZ
    sp       = 0.0

    print(f"\n  {'Alt':>7} {'Setpt':>7} {'NEDz':>8} {'dX':>7} {'dY':>7}")
    while time.time() < deadline:
        if pilot_took_over(mav):
            print("\n[TAKEOFF] Pilot override — releasing control")
            return False

        m = get_latest_position(mav)
        if not m:
            send_sp(mav, sp); time.sleep(dt); continue

        alt = alt_of(m.z)
        dx, dy = m.x - HOME_X, m.y - HOME_Y

        # advance step only once drone reaches current setpoint
        if alt >= (sp - FOLLOW_TOL) and sp < target_alt:
            sp = min(target_alt, sp + STEP_M)

        send_sp(mav, sp)
        print(f"  {alt:>7.3f} {sp:>7.3f} {m.z:>8.3f} {dx:>7.3f} {dy:>7.3f}")

        if abs(alt - target_alt) < ARRIVE_DEADBAND:
            print(f"\n[TAKEOFF] Reached {alt:.2f} m")
            return True
        time.sleep(dt)

    print("[TAKEOFF] Timeout")
    return False


# ─────────────────────────────────────────────
#  HOVER
# ─────────────────────────────────────────────
def hover(mav, alt, secs):
    print(f"\n[HOLD] {alt} m for {secs}s")
    print(f"  {'t-left':>7} {'Alt':>7} {'NEDz':>8} {'dX':>7} {'dY':>7} "
          f"{'vx':>6} {'vy':>6} {'vz':>6}")
    start = time.time()
    last  = 0.0
    dt    = 1.0 / STEP_HZ

    while True:
        elapsed = time.time() - start
        if pilot_took_over(mav):
            print("\n[OVERRIDE] Pilot took control — exiting")
            return "override"
        if elapsed >= secs:
            return "done"
        if not ekf_ok(mav):
            print("\n[ABORT] EKF diverged")
            return "ekf"

        send_sp(mav, alt)

        now = time.time()
        if now - last >= 1.0:
            m = get_latest_position(mav)
            if m:
                print(f"  {secs-elapsed:>6.1f}s {alt_of(m.z):>7.3f} {m.z:>8.3f} "
                      f"{m.x-HOME_X:>7.3f} {m.y-HOME_Y:>7.3f} "
                      f"{m.vx:>6.2f} {m.vy:>6.2f} {m.vz:>6.2f}")
            last = now
        time.sleep(dt)


# ─────────────────────────────────────────────
#  LANDING — stepped descent
# ─────────────────────────────────────────────
def land(mav):
    print("\n[LAND] Stepped descent")
    deadline = time.time() + LAND_TIMEOUT_S
    dt       = 1.0 / STEP_HZ

    m  = get_latest_position(mav)
    sp = alt_of(m.z) if m else TARGET_ALT_M

    while time.time() < deadline:
        m = get_latest_position(mav)
        if not m:
            send_sp(mav, sp); time.sleep(dt); continue

        alt = alt_of(m.z)
        if alt <= GROUND_M:
            print(f"[LAND] Touchdown {alt:.2f} m")
            disarm(mav)
            return True

        if alt <= (sp + FOLLOW_TOL) and sp > 0.0:
            sp = max(0.0, sp - STEP_M)

        send_sp(mav, sp)
        print(f"  alt={alt:>6.3f}  sp={sp:>5.2f}  NEDz={m.z:>7.3f}")
        time.sleep(dt)

    print("[LAND] Timeout — LAND mode fallback")
    land_mode_fallback(mav)
    return False


# ─────────────────────────────────────────────
#  MAIN
# ─────────────────────────────────────────────
def main():
    mav = connect()

    # 1 — Wait for CH5 pos 3
    print("\n[STANDBY] Waiting CH5 pos 3 (1=Stab 2=POSCTL 3=START)")
    while ch5_pos(mav) != 3:
        time.sleep(0.3)
    print("[TRIGGER] CH5 pos 3")

    # 2 — Abort window
    print("[TRIGGER] Starting in 3s — flip CH5 to cancel")
    time.sleep(3)
    if ch5_pos(mav) != 3:
        print("[CANCEL] Cancelled"); sys.exit(0)

    # 3 — EKF check
    if not ekf_ok(mav):
        print("[ABORT] EKF unhealthy"); sys.exit(1)
    print("[CHECK] EKF OK")

    # 4 — Note: vision will activate at ~1m during climb
    #     rtabmap_trigger fires at 0.8m, vision_bridge starts after quality confirms
    print("[VISION] Vision will activate during climb — waiting until 1m altitude")

    # 5 — Stream → OFFBOARD → ARM  (correct order!)
    print("[OFFBOARD] Pre-streaming setpoints (2s)")
    stream_sp(mav, 0.0, 2.0)
    set_mode(mav, PX4_OFFBOARD_MAIN, PX4_OFFBOARD_SUB, "OFFBOARD")
    stream_sp(mav, 0.0, 0.5)

    if not arm(mav):
        print("[ABORT] Arm failed"); sys.exit(1)

    # 6 — Keep streaming while EKF settles, then capture home
    print("[HOME] Streaming while EKF settles (3s)")
    stream_sp(mav, 0.0, 3.0)
    capture_home(mav)
    stream_sp(mav, 0.0, 1.0)   # keep alive after home capture

    # 7 — Climb to 1m on optical flow only
    print("[TAKEOFF] Phase 1 — climbing to 2.8m on optical flow")
    if not takeoff(mav, 2.8):
        print("[EXIT] Phase 1 takeoff aborted")
        land(mav)
        sys.exit(0)

    # 8 — Wait for vision to fuse at 1m
    print(f"[VISION] Holding at 2.8m — waiting up to {VISION_WAIT_S}s for vision_pose")
    vision_deadline = time.time() + VISION_WAIT_S
    vision_ok = False
    env = {**os.environ, "ROS_DOMAIN_ID": os.environ.get("ROS_DOMAIN_ID", "0")}

    # Stream setpoints in background thread so drone holds while we check vision
    import threading
    _keep_streaming = [True]
    def _stream_hold():
        while _keep_streaming[0]:
            send_sp(mav, 2.8)
            time.sleep(0.05)   # 20Hz — keeps OFFBOARD alive
    stream_thread = threading.Thread(target=_stream_hold, daemon=True)
    stream_thread.start()

    while time.time() < vision_deadline:
        if pilot_took_over(mav):
            _keep_streaming[0] = False
            print("[OVERRIDE] Pilot took control")
            sys.exit(0)
        # Check ESTIMATOR_STATUS — pos_horiz_ratio < 0.5 means vision is fused
        es = mav.recv_match(type="ESTIMATOR_STATUS", blocking=False)
        if es and hasattr(es, "pos_horiz_ratio"):
            ratio = es.pos_horiz_ratio
            if ratio > 0.0 and ratio < 0.8:
                vision_ok = True
                print(f"[VISION] Vision fused — pos_horiz_ratio={ratio:.3f}")
                break
        remaining = vision_deadline - time.time()
        if int(remaining) % 5 == 0:
            ratio_str = f"{ratio:.3f}" if es else "?"
            print(f"  [VISION] waiting ... {remaining:.0f}s left  pos_horiz_ratio={ratio_str}")
        time.sleep(0.5)

    _keep_streaming[0] = False   # stop background stream

    if not vision_ok:
        if REQUIRE_VISION:
            print("[ABORT] Vision not active — landing for safety")
            land(mav)
            sys.exit(1)
        else:
            print("[WARN] Vision not active — continuing on optical flow only")

    # Re-capture HOME after vision fuses — EKF origin shifts when vision kicks in
    _keep_streaming[0] = False
    time.sleep(0.3)
    print("[HOME] Re-capturing origin after vision fusion...")
    capture_home(mav)
    # Resume streaming at new home altitude
    _keep_streaming[0] = True
    stream_thread2 = threading.Thread(target=_stream_hold, daemon=True)
    stream_thread2.start()
    time.sleep(2.0)  # let drone settle at corrected position
    _keep_streaming[0] = False

    # 9 — Climb to mission altitude with vision active
    print(f"[TAKEOFF] Phase 2 — climbing to {TARGET_ALT_M}m with vision")
    if not takeoff(mav, TARGET_ALT_M):
        print("[EXIT] Phase 2 takeoff aborted")
        land(mav)
        sys.exit(0)

    # 10 — Hover
    result = hover(mav, TARGET_ALT_M, HOVER_SECS)
    if result == "override":
        sys.exit(0)            # pilot has control, don't interfere
    if result == "ekf":
        land(mav); sys.exit(1)

    # 9 — Land
    land(mav)

    print("[LAND] Waiting disarm ...")
    for _ in range(60):
        hb = mav.recv_match(type="HEARTBEAT", blocking=True, timeout=2)
        if hb and not (hb.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED):
            print("[DONE] Disarmed — mission complete")
            break
        time.sleep(0.5)
    sys.exit(0)


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\n[ABORT] Ctrl+C — emergency descent")
        try:
            m = mavutil.mavlink_connection(MAVLINK_PORT, baud=BAUD)
            m.wait_heartbeat(timeout=5)
            m.target_system = 1
            land(m)
        except Exception:
            pass
        sys.exit(1)
