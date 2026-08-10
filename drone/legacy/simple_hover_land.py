#!/usr/bin/python3
"""
Simple Autonomous Hover & Land — IRoC-U 2026, Team Viman Rakshak
=================================================================
Mission:
  1. Wait for CH5 position 3 to start (3-second abort window)
  2. EKF check
  3. Capture home position on ground
  4. Arm
  5. Offboard stepped climb to 1.5 m
  6. Hold position for HOVER_SECS (logs local position from vision+optical flow)
  7. Offboard stepped descent + disarm

RC Override behaviour:
  - CH5 flipped to pos 1 (Stabilize) → script releases control, drone stays in Stabilize
  - CH5 flipped to pos 2 (POSCTL)    → script releases control, drone stays in POSCTL
  - CH5 stays pos 3                  → script owns the drone
  Pilot always wins — script just stops sending setpoints and exits cleanly.

Ctrl+C → emergency controlled_land then exit.

NED frame:
  Z negative when airborne. alt_above_home = (-z) - (-HOME_Z).
"""

import time
import sys
from pymavlink import mavutil

# ─────────────────────────────────────────────
#  USER CONFIGURATION
# ─────────────────────────────────────────────
SERIAL_PORT       = "udpin:0.0.0.0:14540"
BAUD_RATE         = 921600

TARGET_ALT_M      = 2     # hover altitude above ground (metres)
HOVER_SECS        = 20      # seconds to hold at altitude
TAKEOFF_TIMEOUT_S = 60
ARM_TIMEOUT_S     = 10

# RC channel 5 thresholds (3-position switch)
# Pos 1 = Stabilize  Pos 2 = POSCTL  Pos 3 = Script START
CH5_LOW_MAX  = 1300
CH5_MID_MAX  = 1700

# EKF check
MAX_EKF_ERROR = 0.8

# Stepped ascent/descent tuning
STEP_M          = 0.5    # metres per step
STEP_HZ         = 5      # setpoint send rate (Hz)
FOLLOW_TOL      = 0.15   # step only once drone catches up within this (m)
ARRIVE_DEADBAND = 0.15   # arrival tolerance (m)
LAND_TIMEOUT_S  = 60
GROUND_M        = 0.15   # alt threshold → "on ground"

# PX4 custom mode IDs
PX4_OFFBOARD_MAIN = 6
PX4_OFFBOARD_SUB  = 0
PX4_LAND_MAIN     = 3
PX4_LAND_SUB      = 6

# ─────────────────────────────────────────────
#  GLOBAL HOME (captured before arming)
# ─────────────────────────────────────────────
HOME_X = 0.0
HOME_Y = 0.0
HOME_Z = 0.0


# ─────────────────────────────────────────────
#  CONNECTION
# ─────────────────────────────────────────────

def connect(port, baud):
    print(f"[CONNECT] Opening {port} ...")
    mav = mavutil.mavlink_connection(port, baud=baud)
    print("[CONNECT] Waiting for heartbeat ...")
    mav.wait_heartbeat(timeout=15)
    print(f"[CONNECT] Heartbeat from sysid={mav.target_system}")
    # Request RC + local position streams
    mav.mav.request_data_stream_send(
        mav.target_system, mav.target_component,
        mavutil.mavlink.MAV_DATA_STREAM_RC_CHANNELS, 10, 1)
    mav.mav.request_data_stream_send(
        mav.target_system, mav.target_component,
        mavutil.mavlink.MAV_DATA_STREAM_POSITION, 10, 1)
    return mav


# ─────────────────────────────────────────────
#  RC HELPERS
# ─────────────────────────────────────────────

def get_ch5_pwm(mav):
    msg = mav.recv_match(type='RC_CHANNELS', blocking=True, timeout=1)
    return msg.chan5_raw if msg else None


def get_ch5_position(pwm):
    if pwm is None:
        return None
    if pwm <= CH5_LOW_MAX:
        return 1   # Stabilize
    elif pwm <= CH5_MID_MAX:
        return 2   # POSCTL
    else:
        return 3   # Script / Offboard


def pilot_override(mav):
    """
    Returns (True, position) if pilot has flipped CH5 away from pos 3.
    pos 1 = Stabilize, pos 2 = POSCTL.
    Script does NOT command a mode change — PX4 already switched via RC.
    """
    pos = get_ch5_position(get_ch5_pwm(mav))
    if pos != 3:
        return True, pos
    return False, 3


# ─────────────────────────────────────────────
#  MODE / ARM HELPERS
# ─────────────────────────────────────────────

def set_flight_mode(mav, main_mode, sub_mode, label):
    mav.mav.command_long_send(
        mav.target_system, mav.target_component,
        mavutil.mavlink.MAV_CMD_DO_SET_MODE,
        0, 209, main_mode, sub_mode, 0, 0, 0, 0)
    print(f"[MODE] → {label}")
    time.sleep(0.5)


def arm(mav):
    print("[ARM] Arming ...")
    mav.mav.command_long_send(
        mav.target_system, mav.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0, 1, 0, 0, 0, 0, 0, 0)
    deadline = time.time() + ARM_TIMEOUT_S
    while time.time() < deadline:
        hb = mav.recv_match(type='HEARTBEAT', blocking=True, timeout=2)
        if hb and (hb.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED):
            print("[ARM] Armed ✓")
            return True
        time.sleep(0.2)
    print("[ARM] FAILED")
    return False


def disarm(mav):
    mav.mav.command_long_send(
        mav.target_system, mav.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0, 0, 21196, 0, 0, 0, 0, 0)
    print("[DISARM] Force-disarm sent.")


def force_land_mode(mav):
    """Fallback only — if Offboard descent times out."""
    mav.mav.command_long_send(
        mav.target_system, mav.target_component,
        mavutil.mavlink.MAV_CMD_NAV_LAND,
        0, 0, 0, 0, float('nan'), float('nan'), float('nan'), 0)
    set_flight_mode(mav, PX4_LAND_MAIN, PX4_LAND_SUB, "LAND fallback")


# ─────────────────────────────────────────────
#  POSITION / EKF HELPERS
# ─────────────────────────────────────────────

def ned_z_to_alt(ned_z):
    return -ned_z + HOME_Z

def flush_position(mav, count=20):
    for _ in range(count):
        mav.recv_match(type="LOCAL_POSITION_NED", blocking=False)


def get_home_position(mav):
    global HOME_X, HOME_Y, HOME_Z
    print("[HOME] Capturing takeoff origin (5 samples) ...")
    samples = []
    deadline = time.time() + 3.0
    while time.time() < deadline and len(samples) < 5:
        # Drain buffer to get freshest position
        msg = None
        while True:
            m = mav.recv_match(type='LOCAL_POSITION_NED', blocking=False)
            if m is None:
                break
            msg = m
        if msg is None:
            msg = mav.recv_match(type='LOCAL_POSITION_NED', blocking=True, timeout=0.5)
        if msg:
            samples.append((msg.x, msg.y, msg.z))
        time.sleep(0.2)
    if not samples:
        print("[HOME] WARNING: No data — using 0,0,0")
        return
    HOME_X = sum(s[0] for s in samples) / len(samples)
    HOME_Y = sum(s[1] for s in samples) / len(samples)
    HOME_Z = sum(s[2] for s in samples) / len(samples)
    print(f"[HOME] X={HOME_X:.4f}  Y={HOME_Y:.4f}  Z={HOME_Z:.4f} (NED)")
    print(f"[HOME] Cruise NED z target = {HOME_Z - TARGET_ALT_M:.4f} m")


def send_position_target(mav, alt_m):
    """Publish SET_POSITION_TARGET_LOCAL_NED — X/Y locked to home."""
    target_z = HOME_Z - alt_m
    mav.mav.set_position_target_local_ned_send(
        int(time.time() * 1000) & 0xFFFFFFFF,
        mav.target_system, mav.target_component,
        mavutil.mavlink.MAV_FRAME_LOCAL_NED,
        0b0000111111111000,
        HOME_X, HOME_Y, target_z,
        0, 0, 0,
        0, 0, 0,
        0, 0)


def get_ekf_ok(mav):
    msg = mav.recv_match(type='EKF_STATUS_REPORT', blocking=False)
    if msg:
        return msg.pos_horiz_variance < MAX_EKF_ERROR
    return True


def enter_offboard(mav, seed_alt_m):
    print(f"[OFFBOARD] Pre-streaming at {seed_alt_m:.2f} m ...")
    for _ in range(10):
        send_position_target(mav, seed_alt_m)
        time.sleep(0.1)
    set_flight_mode(mav, PX4_OFFBOARD_MAIN, PX4_OFFBOARD_SUB, "OFFBOARD")
    time.sleep(0.5)


# ─────────────────────────────────────────────
#  TAKEOFF — stepped Offboard ascent
# ─────────────────────────────────────────────

def controlled_takeoff(mav, target_alt, timeout):
    print(f"\n[TAKEOFF] Climbing to {target_alt} m ...")
    deadline     = time.time() + timeout
    interval     = 1.0 / STEP_HZ
    setpoint_alt = 0.0

    # OFFBOARD already active — just start streaming
    print(f"\n  {'Alt(m)':>8}  {'Setpt(m)':>10}  {'NED z':>8}  {'ΔX':>8}  {'ΔY':>8}")
    print(f"  {'-'*8}  {'-'*10}  {'-'*8}  {'-'*8}  {'-'*8}")

    while time.time() < deadline:
        # Check pilot override during takeoff
        override, ch5_pos = pilot_override(mav)
        if override:
            print(f"\n[TAKEOFF] Pilot override — CH5 pos {ch5_pos} "
                  f"({'Stabilize' if ch5_pos == 1 else 'POSCTL'}) — releasing control")
            return False

        # Drain buffer to get freshest position
        msg = None
        while True:
            m = mav.recv_match(type='LOCAL_POSITION_NED', blocking=False)
            if m is None:
                break
            msg = m
        if msg is None:
            msg = mav.recv_match(type='LOCAL_POSITION_NED', blocking=True, timeout=0.5)
        if not msg:
            send_position_target(mav, setpoint_alt)
            time.sleep(interval)
            continue

        alt = ned_z_to_alt(msg.z)
        dx  = msg.x - HOME_X
        dy  = msg.y - HOME_Y

        # Only step up if drone has actually reached current setpoint
        if alt >= (setpoint_alt - FOLLOW_TOL) and setpoint_alt < target_alt:
            setpoint_alt = min(target_alt, setpoint_alt + STEP_M)

        send_position_target(mav, setpoint_alt)
        print(f"  {alt:>8.3f}  {setpoint_alt:>10.3f}  {msg.z:>8.3f}  {dx:>8.3f}  {dy:>8.3f}")

        if abs(alt - target_alt) < ARRIVE_DEADBAND:
            print(f"\n[TAKEOFF] Reached {alt:.3f} m ✓  ΔX={dx:.3f}  ΔY={dy:.3f}")
            return True

        time.sleep(interval)

    print(f"\n[TAKEOFF] Timeout after {timeout}s")
    return False


# ─────────────────────────────────────────────
#  LANDING — stepped Offboard descent
# ─────────────────────────────────────────────

def controlled_land(mav):
    print("\n[LAND] Controlled descent ...")
    deadline = time.time() + LAND_TIMEOUT_S
    interval = 1.0 / STEP_HZ

    pos_msg      = mav.recv_match(type='LOCAL_POSITION_NED', blocking=True, timeout=2)
    setpoint_alt = ned_z_to_alt(pos_msg.z) if pos_msg else TARGET_ALT_M
    print(f"  Starting from {setpoint_alt:.3f} m")

    print(f"\n  {'Alt(m)':>8}  {'Setpt(m)':>10}  {'NED z':>8}  {'ΔX':>8}  {'ΔY':>8}")
    print(f"  {'-'*8}  {'-'*10}  {'-'*8}  {'-'*8}  {'-'*8}")

    while time.time() < deadline:
        pos_msg = mav.recv_match(type='LOCAL_POSITION_NED', blocking=True, timeout=1)
        if not pos_msg:
            send_position_target(mav, setpoint_alt)
            time.sleep(interval)
            continue

        alt = ned_z_to_alt(pos_msg.z)
        dx  = pos_msg.x - HOME_X
        dy  = pos_msg.y - HOME_Y

        if alt <= GROUND_M:
            print(f"\n[LAND] Touchdown at {alt:.3f} m — disarming")
            disarm(mav)
            return True

        # Only step down if drone has actually descended to current setpoint
        if alt <= (setpoint_alt + FOLLOW_TOL) and setpoint_alt > 0.0:
            setpoint_alt = max(0.0, setpoint_alt - STEP_M)

        send_position_target(mav, setpoint_alt)
        print(f"  {alt:>8.3f}  {setpoint_alt:>10.3f}  {pos_msg.z:>8.3f}  {dx:>8.3f}  {dy:>8.3f}")

        time.sleep(interval)

    print(f"\n[LAND] Timeout — LAND mode fallback")
    force_land_mode(mav)
    return False


# ─────────────────────────────────────────────
#  MAIN
# ─────────────────────────────────────────────

def main():
    mav = connect(SERIAL_PORT, BAUD_RATE)

    # ── STEP 1: Wait for CH5 pos 3 ───────────────────────────────────────────
    print("\n[STANDBY] Waiting for CH5 position 3 ...")
    print("  Pos 1 = Stabilize  |  Pos 2 = POSCTL  |  Pos 3 = START\n")
    while True:
        pwm = get_ch5_pwm(mav)
        pos = get_ch5_position(pwm)
        if pos == 3:
            print(f"\n[TRIGGER] CH5 pos 3 detected (PWM={pwm})")
            break
        print(f"  [STANDBY] CH5 pos={pos}  PWM={pwm}", end="\r")
        time.sleep(0.5)

    # ── STEP 2: 3-second abort window ────────────────────────────────────────
    print("[TRIGGER] Starting in 3s — flip CH5 back NOW to cancel ...")
    time.sleep(3)
    if get_ch5_position(get_ch5_pwm(mav)) != 3:
        print("[CANCEL] CH5 flipped back — mission cancelled. Exiting.")
        sys.exit(0)

    # ── STEP 3: EKF check ────────────────────────────────────────────────────
    print("\n[CHECK] EKF health ...")
    if not get_ekf_ok(mav):
        print("[ABORT] EKF not healthy. Exiting.")
        sys.exit(1)
    print("[CHECK] EKF OK ✓")

    # ── STEP 5: Enter OFFBOARD and arm ───────────────────────────────────────
    # Must stream setpoints BEFORE switching to OFFBOARD and arming
    print("[OFFBOARD] Pre-streaming setpoints before arm...")
    for _ in range(40):  # 2 seconds at 20Hz
        send_position_target(mav, 0.0)
        time.sleep(0.05)

    set_flight_mode(mav, PX4_OFFBOARD_MAIN, PX4_OFFBOARD_SUB, "OFFBOARD")
    time.sleep(0.5)

    if not arm(mav):
        print("[ABORT] Arming failed. Exiting.")
        sys.exit(1)

    # Keep streaming after arm while EKF settles
    print("[HOME] Streaming setpoints while EKF settles (3s)...")
    deadline_settle = time.time() + 3.0
    while time.time() < deadline_settle:
        send_position_target(mav, 0.0)
        time.sleep(0.05)

    flush_position(mav, count=50)
    get_home_position(mav)

    # Keep streaming after home capture
    deadline_post = time.time() + 1.0
    while time.time() < deadline_post:
        send_position_target(mav, 0.0)
        time.sleep(0.05)

    # ── STEP 6: Takeoff ───────────────────────────────────────────────────────
    if not controlled_takeoff(mav, TARGET_ALT_M, TAKEOFF_TIMEOUT_S):
        # Either timeout or pilot override — don't command anything, just exit
        print("[EXIT] Takeoff aborted — pilot has control.")
        sys.exit(0)

    # ── STEP 7: Hover ─────────────────────────────────────────────────────────
    print(f"\n[HOLD] Hovering at {TARGET_ALT_M} m for {HOVER_SECS}s ...")
    print("  Flip CH5 to Stabilize (pos 1) or POSCTL (pos 2) to take control\n")
    print(f"  {'Time left':>10}  {'Alt(m)':>8}  {'NED z':>8}  "
          f"{'ΔX(m)':>8}  {'ΔY(m)':>8}  {'vx':>6}  {'vy':>6}  {'vz':>6}")
    print(f"  {'-'*10}  {'-'*8}  {'-'*8}  {'-'*8}  {'-'*8}  {'-'*6}  {'-'*6}  {'-'*6}")

    hold_start = time.time()
    last_print = 0.0

    while True:
        elapsed = time.time() - hold_start

        # ── Pilot override check ──────────────────────────────────────────────
        override, ch5_pos = pilot_override(mav)
        if override:
            mode_name = "Stabilize" if ch5_pos == 1 else "POSCTL"
            print(f"\n[OVERRIDE] CH5 → pos {ch5_pos} ({mode_name})")
            print(f"[OVERRIDE] Releasing control — drone is in {mode_name}")
            print(f"[OVERRIDE] Script exiting cleanly. Pilot has full control.")
            sys.exit(0)

        # ── Hover complete ────────────────────────────────────────────────────
        if elapsed >= HOVER_SECS:
            print(f"\n[HOLD] {HOVER_SECS}s complete — descending")
            break

        # ── EKF watchdog ──────────────────────────────────────────────────────
        if not get_ekf_ok(mav):
            print("\n[ABORT] EKF diverged during hover — landing")
            controlled_land(mav)
            sys.exit(1)

        # ── Keep Offboard alive ───────────────────────────────────────────────
        send_position_target(mav, TARGET_ALT_M)

        # ── Print telemetry every 1s ──────────────────────────────────────────
        now = time.time()
        if now - last_print >= 1.0:
            pos_msg = mav.recv_match(type='LOCAL_POSITION_NED', blocking=False)
            if pos_msg:
                alt = ned_z_to_alt(pos_msg.z)
                dx  = pos_msg.x - HOME_X
                dy  = pos_msg.y - HOME_Y
                # vx/vy/vz from vision+optical flow EKF estimate
                print(f"  {HOVER_SECS-elapsed:>9.1f}s  {alt:>8.3f}  {pos_msg.z:>8.3f}  "
                      f"{dx:>8.3f}  {dy:>8.3f}  "
                      f"{pos_msg.vx:>6.2f}  {pos_msg.vy:>6.2f}  {pos_msg.vz:>6.2f}")
            last_print = now

        time.sleep(0.2)

    # ── STEP 8: Land ──────────────────────────────────────────────────────────
    controlled_land(mav)

    print("\n[LAND] Waiting for disarm ...")
    for _ in range(60):
        hb = mav.recv_match(type='HEARTBEAT', blocking=True, timeout=2)
        if hb and not (hb.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED):
            print("[LAND] Disarmed ✓ — mission complete!")
            break
        time.sleep(0.5)

    print("[DONE] Re-run script to fly again.")
    sys.exit(0)


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\n[ABORT] Ctrl+C — emergency land ...")
        try:
            mav = mavutil.mavlink_connection(SERIAL_PORT, baud=BAUD_RATE)
            mav.wait_heartbeat(timeout=5)
            controlled_land(mav)
        except Exception:
            pass
        sys.exit(1)
