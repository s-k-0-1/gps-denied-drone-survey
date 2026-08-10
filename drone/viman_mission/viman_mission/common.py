"""Shared helpers for the viman_mission package.

Coordinate frame convention (whole package):
  MAVROS publishes /mavros/local_position/pose in ENU (East-North-Up).
  PX4 internally uses NED, but MAVROS auto-converts — we never touch NED.
    ENU: X=East  Y=North  Z=Up(+altitude)   ← what this package uses
    NED: X=North Y=East   Z=Down(-altitude) ← PX4 internal / QGC display
  Origin is the same for both: home/arming point.
"""

import math

from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                       ReliabilityPolicy)

# RTAB-Map covariance semantics (pose.covariance[0], the x-x diagonal):
#   0            → not yet publishing
#   0 < cov < 100 → tracking good
#   ≥ 100        → lost (99999 when RTAB-Map reports tracking failure)
RTAB_BAD_COV = 100.0


def qos_best_effort(depth: int = 10) -> QoSProfile:
    """Sensor-style QoS — matches MAVROS pose/velocity/RC publishers."""
    return QoSProfile(
        reliability=ReliabilityPolicy.BEST_EFFORT,
        history=HistoryPolicy.KEEP_LAST,
        depth=depth,
        durability=DurabilityPolicy.VOLATILE,
    )


def qos_reliable(depth: int = 10) -> QoSProfile:
    """Reliable QoS — state, odometry, vision pose."""
    return QoSProfile(
        reliability=ReliabilityPolicy.RELIABLE,
        history=HistoryPolicy.KEEP_LAST,
        depth=depth,
        durability=DurabilityPolicy.VOLATILE,
    )


def rtab_cov_good(cov: float) -> bool:
    """True if RTAB-Map covariance indicates healthy tracking."""
    return 0.0 < cov < RTAB_BAD_COV


def yaw_deg_from_quaternion(q) -> float:
    """Extract yaw (degrees) from a geometry_msgs Quaternion."""
    return math.degrees(
        math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                   1.0 - 2.0 * (q.y * q.y + q.z * q.z)))


# ── Quaternion math (tuples in ROS order: x, y, z, w) ──────────────
# Used by vio_gate for the self-calibrating frame correction:
# q_corr = q_ekf_seed ⊗ q_rtab_seed⁻¹  maps RTAB's odom frame into ENU,
# and the SAME rotation is applied to positions and orientations —
# replacing the old hand-tuned remap that applied a reflection to
# positions but a rotation to quaternions (geometrically inconsistent,
# prime suspect for the drift-after-handover crashes).

def quat_mult(a, b):
    """Hamilton product a ⊗ b. Args/result: (x, y, z, w) tuples."""
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw,
            aw * bw - ax * bx - ay * by - az * bz)


def quat_conj(q):
    """Conjugate (= inverse for unit quaternions)."""
    return (-q[0], -q[1], -q[2], q[3])


def quat_rotate(q, v):
    """Rotate vector v=(x,y,z) by unit quaternion q."""
    r = quat_mult(quat_mult(q, (v[0], v[1], v[2], 0.0)), quat_conj(q))
    return (r[0], r[1], r[2])


def quat_normalize(q):
    n = math.sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3])
    if n < 1e-9:
        return (0.0, 0.0, 0.0, 1.0)
    return (q[0] / n, q[1] / n, q[2] / n, q[3] / n)


def quat_from_msg(q):
    """geometry_msgs Quaternion → tuple."""
    return (q.x, q.y, q.z, q.w)
