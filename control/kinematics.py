"""
Wireless Robot Arm: kinematics (forward, inverse, motor mixing).

Conventions (identical to the 3D viewer and the arm_v0 / wrist_v0 CAD):
  - Base frame: origin on the shoulder axis, which crosses the yaw axis.
    z points up; at rest (q2 = 0) the arm is vertical and it leans towards +y.
    The desk is at z = -SHOULDER_H.
  - Angles in degrees.
      q1  base (yaw)            Rz(q1)
      q2  shoulder              Rx(-q2)      positive = forwards
      q3  elbow (flexion)       Rx(-q3)      about the axis at z = LU
      q4  pronation             Rz(q4)       about the forearm axis
      q5  wrist pitch           Rx(q5)       about the wrist centre
      q6  wrist roll            Rz(q6)       endless
  - Chain:  T = Rz(q1) Rx(-q2) T(0,0,LU) Rx(-q3) Rz(q4) T(0,0,LF) Rx(q5) Rz(q6) T(0,0,D6)
  - The q4, q5, q6 axes meet at the wrist centre (spherical wrist):
    the wrist centre position depends only on q1, q2, q3.
"""
import numpy as np

# --- dimensions from the CAD (arm v0.46 / wrist v1.38 / hand v2.6) ---------
LU = 229.0          # shoulder -> elbow (v0.45: 700 mm reach with the 266 mm forearm)
LF = 266.0          # elbow -> wrist centre (v0.45)
D6 = 205.4          # wrist centre -> grip centre (fingertips at ~228)
SHOULDER_H = 216.0  # shoulder axis above the desk (gearboxes on the turntable, 20T pinions, arm v0.44)

# --- transmissions ----------------------------------------------------------
R_YAW = 146 / 15 * 5    # base: 5:1 planetary on the motor, then 146-tooth ring / 15-tooth pinion
R_SH = 10 * 56 / 20     # shoulder: two NEMA23 + 10:1 planetaries, 20T steel pinions on 56 teeth cut in the steel bones (v0.44)
K_ES = -1.0             # elbow motors per degree of shoulder: their 16T pulleys sit on the shoulder axis, fixed to the turret,
                        # so turning the shoulder turns the elbow pulleys relative to the arm (see arm_v0 elbow drive, v0.40)
R_EL = 110 / 20 * 160 / 20  # elbow: two belt stages, 20T -> 110T outside the bone, 20T -> 160T inside (v0.44: 44:1)
KP_EL = 0.5             # elbow: 30/15 bevels -> half a side-gear turn per satellite turn
R_WR = 100 / 20         # wrist: 100/20 belts (v1.34: 20T motor pulleys for steel-cord belts), 1:1 bevels
STEPS_REV = 200 * 16    # steps per turn with 16 microsteps

LIMITS = {  # degrees (q6 endless)
    'q1': (-170, 170), 'q2': (-90, 90), 'q3': (-60, 105),
    'q4': (-180, 180), 'q5': (-90, 90), 'q6': (-np.inf, np.inf),
}


# --- elementary matrices ----------------------------------------------------
def Rx(d):
    t = np.radians(d); c, s = np.cos(t), np.sin(t)
    return np.array([[1, 0, 0, 0], [0, c, -s, 0], [0, s, c, 0], [0, 0, 0, 1.0]])


def Rz(d):
    t = np.radians(d); c, s = np.cos(t), np.sin(t)
    return np.array([[c, -s, 0, 0], [s, c, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1.0]])


def Tz(L):
    m = np.eye(4); m[2, 3] = L
    return m


# --- forward kinematics -----------------------------------------------------
def fk_frames(q, d6=D6):
    """Frame of each joint (4x4) in the base frame: shoulder, elbow, forearm, wrist, hand, grip."""
    q1, q2, q3, q4, q5, q6 = q
    F_sh = Rz(q1) @ Rx(-q2)
    F_el = F_sh @ Tz(LU) @ Rx(-q3)
    F_fo = F_el @ Rz(q4)
    F_wr = F_fo @ Tz(LF)
    F_ha = F_wr @ Rx(q5) @ Rz(q6)
    F_tcp = F_ha @ Tz(d6)
    return {'shoulder': F_sh, 'elbow': F_el, 'forearm': F_fo, 'wrist': F_wr, 'hand': F_ha, 'tcp': F_tcp}


def fk(q, d6=D6):
    """Grip pose (4x4) in the base frame."""
    return fk_frames(q, d6)['tcp']


def wrist_center(q1, q2, q3):
    """Closed form: the wrist centre depends only on q1, q2, q3."""
    a2, a23 = np.radians(q2), np.radians(q2 + q3)
    r = LU * np.sin(a2) + LF * np.sin(a23)
    z = LU * np.cos(a2) + LF * np.cos(a23)
    t1 = np.radians(q1)
    return np.array([-r * np.sin(t1), r * np.cos(t1), z])


# --- a convenient way to give the hand orientation -------------------------
def hand_frame(p, tilt=0.0, spin=0.0):
    """3x3 hand rotation for a grip at p.
    tilt: 0 = fingers pointing down, 90 = horizontal, pointing outwards.
    spin: rotation of the gripper about its own axis."""
    az = np.arctan2(-p[0], p[1])
    ur = np.array([-np.sin(az), np.cos(az), 0.0])     # radial, outwards
    t = np.array([np.cos(az), np.sin(az), 0.0])       # horizontal tangent
    T = np.radians(tilt)
    z = np.sin(T) * ur - np.cos(T) * np.array([0, 0, 1.0])
    s = np.radians(spin)
    x = np.cos(s) * t + np.sin(s) * np.cross(z, t)
    y = np.cross(z, x)
    return np.column_stack([x, y, z])


# --- inverse kinematics ------------------------------------------------------
def _wrap(a):
    return (a + 180.0) % 360.0 - 180.0


def ik_all(p, R, d6=D6):
    """All solutions (up to 8) for the grip at p (mm, base frame) with rotation R (3x3)."""
    p = np.asarray(p, float); R = np.asarray(R, float)
    pw = p - d6 * R[:, 2]                       # 1) wrist centre
    x, y, z = pw
    sols = []
    for flip in (+1, -1):                       # arm in front / behind
        q1 = np.degrees(np.arctan2(-x, y)) if flip > 0 else _wrap(np.degrees(np.arctan2(-x, y)) + 180)
        r = flip * np.hypot(x, y)               # signed distance in the arm plane
        D = (r * r + z * z - LU * LU - LF * LF) / (2 * LU * LF)
        if abs(D) > 1 + 1e-9:
            continue                            # out of reach
        D = np.clip(D, -1, 1)
        for elbow in (+1, -1):                  # elbow up / down
            q3 = elbow * np.degrees(np.arccos(D))
            t3 = np.radians(q3)
            q2 = np.degrees(np.arctan2(r, z) - np.arctan2(LF * np.sin(t3), LU + LF * np.cos(t3)))
            q2 = _wrap(q2)
            R03 = (Rz(q1) @ Rx(-(q2 + q3)))[:3, :3]
            W = R03.T @ R                       # = Rz(q4) Rx(q5) Rz(q6)  (ZXZ Euler)
            s5 = np.hypot(W[0, 2], W[1, 2])
            for wflip in (+1, -1):              # wrist up / down
                q5 = np.degrees(np.arctan2(wflip * s5, W[2, 2]))
                if s5 < 1e-9:                   # singular: q4 and q6 on the same axis
                    q4 = 0.0
                    q6 = np.degrees(np.arctan2(W[1, 0], W[0, 0]))
                else:
                    q4 = np.degrees(np.arctan2(wflip * W[0, 2], -wflip * W[1, 2]))
                    q6 = np.degrees(np.arctan2(wflip * W[2, 0], wflip * W[2, 1]))
                sols.append(np.array([q1, q2, q3, q4, q5, q6]))
    return sols


def within_limits(q):
    return all(lo - 1e-6 <= v <= hi + 1e-6 for v, (lo, hi) in zip(q, LIMITS.values()))


def ik(p, R, q_prev=None, d6=D6):
    """Best solution: inside the limits and closest to q_prev (the endless roll is unwrapped)."""
    q_prev = np.zeros(6) if q_prev is None else np.asarray(q_prev, float)
    best, cost = None, np.inf
    for q in ik_all(p, R, d6):
        q = q.copy()
        if abs(q[4]) < 1e-6:                    # singular: keep q4, put everything on q6
            tot = q[3] + q[5]
            q[3] = q_prev[3]; q[5] = tot - q[3]
        q[5] = q_prev[5] + _wrap(q[5] - q_prev[5])   # endless roll: nearest turn
        if not within_limits(q):
            continue
        c = np.sum((q - q_prev) ** 2)
        if c < cost:
            best, cost = q, c
    return best


# --- joints to motors (and back) --------------------------------------------
def joints_to_motors(q):
    """Rotor angles (degrees), in the same direction as the parts they move.
    The real direction of each motor is set in the firmware (DIR pin) at homing."""
    q1, q2, q3, q4, q5, q6 = q
    return np.array([
        R_YAW * q1,                         # base
        R_SH * q2,                          # shoulder
        R_EL * (-q3 + KP_EL * q4) + K_ES * q2,   # elbow A  (side gear A = -q3 + q4/2), plus the shoulder coupling
        R_EL * (-q3 - KP_EL * q4) + K_ES * q2,   # elbow B
        R_WR * (q5 + q6),                   # wrist A  (side gear A = q5 + q6)
        R_WR * (q5 - q6),                   # wrist B
    ])


def motors_to_joints(m):
    m1, m2, eA, eB, wA, wB = m
    eA, eB = eA - K_ES * m2 / R_SH, eB - K_ES * m2 / R_SH            # remove the shoulder coupling first
    return np.array([
        m1 / R_YAW,
        m2 / R_SH,
        -(eA + eB) / (2 * R_EL),
        (eA - eB) / (2 * KP_EL * R_EL),
        (wA + wB) / (2 * R_WR),
        (wA - wB) / (2 * R_WR),
    ])


def joints_to_steps(q):
    return np.round(joints_to_motors(q) * STEPS_REV / 360.0).astype(int)


def jacobian(q, d6=D6, h=1e-4):
    """Numerical 6x6 Jacobian (linear velocity mm/degree, angular rad/degree)."""
    T0 = fk(q, d6); J = np.zeros((6, 6))
    for i in range(6):
        dq = np.array(q, float); dq[i] += h
        T1 = fk(dq, d6)
        J[:3, i] = (T1[:3, 3] - T0[:3, 3]) / h
        dR = T1[:3, :3] @ T0[:3, :3].T
        J[3:, i] = np.array([dR[2, 1] - dR[1, 2], dR[0, 2] - dR[2, 0], dR[1, 0] - dR[0, 1]]) / (2 * h)
    return J


if __name__ == '__main__':
    q = [-50, 37.3, 79.24, 0, -63.46, 0]
    T = fk(q)
    print('grip', T[:3, 3].round(2), 'above the desk', round(T[2, 3] + SHOULDER_H, 1))
    print('IK  ', ik(T[:3, 3], T[:3, :3]).round(3))
    print('steps', joints_to_steps(q))
