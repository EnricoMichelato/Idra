"""
Wireless Robot Arm: rigid-body dynamics and motor sizing check.

    python3 control/dynamics.py            # prints the report and writes control/dynamics_report.md

Model
  - 6 links with mass, centre of mass and inertia from the CAD (control/mass_model.json, from tools/mass_model.py),
    plus a payload at the grip centre.
  - Joint torques:  tau = M(q) qdd + c(q, qd) + g(q)   (M from the link Jacobians, c and g by differentiation).
  - Motor torques through the transmissions of kinematics.joints_to_motors (the two differentials mix two joints each),
    with an efficiency per stage and the rotor inertia of each motor.
  - A stepper gives its rated torque up to a corner speed, then torque falls as 1/speed.
Everything is an estimate until the real parts are weighed and the motors are measured.
"""
import json, os
import numpy as np
import kinematics as K

HERE = os.path.dirname(os.path.abspath(__file__))
G = 9.81
JOINTS = ('base', 'shoulder', 'elbow', 'pronation', 'wrist pitch', 'wrist roll')
MOTORS = ('base', 'shoulder', 'elbow A', 'elbow B', 'wrist A', 'wrist B')

# --- motors (edit after measuring) -------------------------------------------
# torque: usable torque at low speed with a TMC2209 (Nm); corner: speed where torque starts to fall (rev/s, 24 V);
# rotor: rotor inertia (kg m2); eta: efficiency of the transmission after the motor
MOTOR = {
    'base':     dict(torque=0.70, corner=10.0, rotor=1.1e-5, eta=0.85),   # v0.47: TWO salvaged NEMA17 (0.35 Nm each) + 5:1 planetaries + printed 146T ring gear
    'shoulder': dict(torque=1.70, corner=6.0,  rotor=6.0e-5, eta=0.855),  # TWO 56 mm NEMA23 (0.85 Nm usable each) + 10:1 planetary (90%) + 56/20 steel spur (95%)
    'elbow A':  dict(torque=0.50, corner=8.0, rotor=8.2e-6, eta=0.80),    # v0.47: new 48 mm NEMA17 (0.59 Nm holding, 0.50 usable, closed loop) + two belt stages + bevels
    'elbow B':  dict(torque=0.50, corner=8.0, rotor=8.2e-6, eta=0.80),
    'wrist A':  dict(torque=0.40, corner=10.0, rotor=5.4e-6, eta=0.85),   # 40 mm NEMA17 (salvaged ones: measure them)
    'wrist B':  dict(torque=0.40, corner=10.0, rotor=5.4e-6, eta=0.85),
}
BALANCER = 0.0      # Nm: shoulder gravity balancer. Its torque is BALANCER * sin(q2), against gravity (v0.49: one spring per side, outside the cheek supports)
FLANGE = 0.058      # m: wrist centre -> hand mounting face (where the spec payload hangs, 'like a hook')
SAFE = 0.6          # use at most this fraction of the available torque: a stepper that stalls loses its position

# m = JM q  (motor angles from joint angles)
JM = np.array([K.joints_to_motors(e) for e in np.eye(6)]).T


def load_model(payload=0.2, path=None, at=None):
    d = json.load(open(path or os.path.join(HERE, 'mass_model.json')))['links']
    links = [dict(m=d[str(k)]['mass'], c=np.array(d[str(k)]['com']), I=np.array(d[str(k)]['inertia'])) for k in range(1, 7)]
    if payload:                                           # point mass at the grip centre, on the last link
        l = links[5]; cp = np.array([0, 0, (K.D6/1000.0) if at is None else at]); m2 = l['m'] + payload
        c2 = (l['m']*l['c'] + payload*cp)/m2
        for mm, cc in ((l['m'], l['c']), (payload, cp)):
            d_ = cc - c2; l['I'] = l['I'] + mm*((d_ @ d_)*np.eye(3) - np.outer(d_, d_))
        l['m'], l['c'] = m2, c2
    return links


def link_frames(q):
    """world frame (4x4, metres) of each of the 6 links"""
    q1, q2, q3, q4, q5, q6 = q
    F1 = K.Rz(q1); F2 = F1 @ K.Rx(-q2)
    F3 = F2 @ K.Tz(K.LU/1000) @ K.Rx(-q3); F4 = F3 @ K.Rz(q4)
    F5 = F4 @ K.Tz(K.LF/1000) @ K.Rx(q5); F6 = F5 @ K.Rz(q6)
    return [F1, F2, F3, F4, F5, F6]


def _state(q, links):
    Fs = link_frames(q)
    return [(F[:3, :3], F[:3, 3] + F[:3, :3] @ l['c']) for F, l in zip(Fs, links)]


def potential(q, links):
    return sum(l['m']*G*p[2] for (R, p), l in zip(_state(q, links), links))


def mass_matrix(q, links, h=1e-3):
    """6x6 joint-space inertia (q in degrees, result per radian)"""
    S0 = _state(q, links); M = np.zeros((6, 6)); Jv = []; Jw = []
    for (R0, p0), l in zip(S0, links): Jv.append(np.zeros((3, 6))); Jw.append(np.zeros((3, 6)))
    for i in range(6):
        dq = np.array(q, float); dq[i] += h
        for k, ((R1, p1), (R0, p0)) in enumerate(zip(_state(dq, links), S0)):
            Jv[k][:, i] = (p1 - p0)/np.radians(h)
            dR = R1 @ R0.T
            Jw[k][:, i] = np.array([dR[2, 1] - dR[1, 2], dR[0, 2] - dR[2, 0], dR[1, 0] - dR[0, 1]])/(2*np.radians(h))
    for k, l in enumerate(links):
        R0 = S0[k][0]
        M += l['m']*Jv[k].T @ Jv[k] + Jw[k].T @ (R0 @ l['I'] @ R0.T) @ Jw[k]
    return M


def gravity(q, links, h=1e-3):
    g = np.zeros(6)
    for i in range(6):
        a = np.array(q, float); b = a.copy(); a[i] += h; b[i] -= h
        g[i] = (potential(a, links) - potential(b, links))/np.radians(2*h)
    return g


def inverse_dynamics(q, qd, qdd, links, h=0.05):
    """joint torques (Nm) for q (deg), qd (deg/s), qdd (deg/s2)"""
    w, a = np.radians(qd), np.radians(qdd)
    M = mass_matrix(q, links); c = np.zeros(6)
    if np.any(np.abs(w) > 1e-9):
        dM = []
        for i in range(6):
            qa = np.array(q, float); qb = qa.copy(); qa[i] += h; qb[i] -= h
            dM.append((mass_matrix(qa, links) - mass_matrix(qb, links))/np.radians(2*h))
        Md = sum(dM[i]*w[i] for i in range(6))
        c = Md @ w - 0.5*np.array([w @ dM[i] @ w for i in range(6)])
    return M @ a + c + gravity(q, links)


def balance(tau, q):
    """joint torques left for the motors once the spring balancer has done its part (gravity gives tau[1] < 0 for q2 > 0)"""
    t = np.array(tau, float); t[1] += BALANCER*np.sin(np.radians(q[1]))
    return t


def motor_torques(tau, qd, qdd):
    """motor torque demand (Nm): joint torques through the transmissions, losses, and rotor acceleration"""
    tm = np.linalg.solve(JM.T, tau)                      # ideal: tau = JM^T tau_m
    wm_dd = JM @ np.radians(qdd)
    return np.array([t/MOTOR[n]['eta'] + MOTOR[n]['rotor']*a for t, a, n in zip(tm, wm_dd, MOTORS)])


def motor_speed(qd):
    return JM @ np.asarray(qd)/360.0                     # rev/s


def available(speed_rev_s):
    return np.array([MOTOR[n]['torque']*min(1.0, MOTOR[n]['corner']/max(abs(s), 1e-9)) for n, s in zip(MOTORS, speed_rev_s)])


def utilisation(q, qd, qdd, links):
    tau = balance(inverse_dynamics(q, qd, qdd, links), q)
    return np.abs(motor_torques(tau, qd, qdd))/available(motor_speed(qd)), tau


# --- trajectories -------------------------------------------------------------
def quintic(qa, qb, T, n=40):
    """rest-to-rest joint move: positions, speeds, accelerations (degrees)"""
    qa, qb = np.asarray(qa, float), np.asarray(qb, float)
    for s in np.linspace(0, 1, n):
        p = 10*s**3 - 15*s**4 + 6*s**5; v = (30*s**2 - 60*s**3 + 30*s**4)/T; a = (60*s - 180*s**2 + 120*s**3)/T**2
        yield qa + (qb - qa)*p, (qb - qa)*v, (qb - qa)*a


def move_peak(qa, qb, T, links):
    u = np.zeros(6); tq = np.zeros(6)
    for q, qd, qdd in quintic(qa, qb, T):
        ui, ti = utilisation(q, qd, qdd, links); u = np.maximum(u, ui); tq = np.maximum(tq, np.abs(ti))
    return u, tq


def fastest(qa, qb, links, limit=SAFE):
    lo, hi = 0.15, 20.0
    if move_peak(qa, qb, hi, links)[0].max() > limit: return None
    for _ in range(18):
        mid = (lo + hi)/2
        if move_peak(qa, qb, mid, links)[0].max() > limit: lo = mid
        else: hi = mid
    return hi


def static_scan(links, step=15):
    """worst static motor load over the joint ranges"""
    worst = np.zeros(6); at = [None]*6; tq = np.zeros(6); z = np.zeros(6)
    q2lo, q2hi = K.LIMITS['q2']
    for q2 in range(int(q2lo), int(q2hi) + 1, step):
        for q3 in range(-60, 106, step):
            for q4 in (-90, 0, 90):
                for q5 in (-90, -45, 0, 45, 90):
                    q = [0, q2, q3, q4, q5, 0]
                    if K.fk(q)[2, 3] + K.SHOULDER_H < 20: continue          # below the desk
                    tau = balance(gravity(q, links), q); u = np.abs(motor_torques(tau, z, z))/available(z)
                    tq = np.maximum(tq, np.abs(tau))
                    for i in range(6):
                        if u[i] > worst[i]: worst[i], at[i] = u[i], q
    return worst, at, tq


VARIANTS = [   # (name, shoulder motor torque, elbow motor torque, balancer torque in Nm)
    ('arm v0.50 as designed: two NEMA23 + 10:1 on the steel bones, 48 mm elbow NEMA17 outside the cheeks, two NEMA17 on the base, balance springs outside (about 80 % of the arm weight)', 1.70, 0.50, 10.0),
    ('v0.50 without the balance springs', 1.70, 0.50, 0.0),
    ('one shoulder motor only, with the springs', 0.85, 0.50, 10.0),
]


def apply_variant(v, payload, at=None):
    global BALANCER
    name, sh, el, BALANCER = v
    extra = 0.0
    MOTOR['shoulder']['torque'] = sh
    for k in ('elbow A', 'elbow B'): MOTOR[k]['torque'] = el
    links = load_model(payload, at=at)
    if extra:                                             # heavier elbow motors, 83 mm from the shoulder axis
        l = links[1]; c = np.array([0, 0, 0.083]); m2 = l['m'] + extra; l['c'] = (l['m']*l['c'] + extra*c)/m2; l['m'] = m2
    return links


def report(payloads=((0.0, None, '0 g'), (0.5, None, '500 g at the grip'), (1.0, FLANGE, '1 kg at the flange (spec)'), (1.0, None, '1 kg at the grip'))):
    L = ['# Dynamics and motor sizing (estimate)', '',
         'Generated by `control/dynamics.py` from the CAD masses (`control/mass_model.json`). Nothing here is measured.', '']
    d = json.load(open(os.path.join(HERE, 'mass_model.json')))['links']
    L += ['## Moving masses', '', '| Link | Mass (kg) | Heaviest parts |', '|---|---|---|']
    names = ('turret (turns with the base)', 'upper arm', 'elbow cradle', 'forearm', 'wrist flange', 'hand')
    for k in range(1, 7):
        heavy = ', '.join('%s %d g' % (n[2:] if n.startswith('W_') else n, g) for n, g in d[str(k)]['heaviest'][:3])
        L.append(f"| {names[k-1]} | {d[str(k)]['mass']:.2f} | {heavy} |")
    L += ['', f"Mass carried by the shoulder: **{sum(d[str(k)]['mass'] for k in range(2, 7)):.2f} kg** plus the payload.", '']
    p_pick = np.array([0.0, 400.0, 60.0 - K.SHOULDER_H]); q_ready = [0, 10, 95, 0, -75, 0]
    q_pick = K.ik(p_pick, K.hand_frame(p_pick, 0.0), q_prev=q_ready)
    p_pl = np.array([-400.0, 0.0, 60.0 - K.SHOULDER_H]); q_place = K.ik(p_pl, K.hand_frame(p_pl, 0.0), q_prev=q_pick)
    moves = [('ready → pick on the desk, 400 mm in front', q_ready, q_pick), ('pick → place, base turned 90°', q_pick, q_place),
             ('elbow 0 → 90°, upper arm vertical', [0, 0, 0, 0, 0, 0], [0, 0, 90, 0, 0, 0])]
    z = np.zeros(6); res = {}
    for v in VARIANTS:
        L += [f'## Variant: {v[0]}', '',
              f'Share of the available motor torque. Above {SAFE:.0%} is not safe for a stepper: if it stalls it loses its position.', '',
              '| Payload | Shoulder, worst pose | Elbow, worst pose | Wrist, worst pose | Shoulder, holding at the pick point | Shoulder motor torque left (Nm at the joint) | Elbow torque (Nm) |', '|---|---|---|---|---|---|---|']
        for pl, at_, lab_ in payloads:
            links = apply_variant(v, pl, at_); w, at, tq = static_scan(links)
            hp = np.abs(motor_torques(balance(gravity(q_pick, links), q_pick), z, z))/available(z)
            L.append(f'| {lab_} | {w[1]:.0%} | {max(w[2], w[3]):.0%} | {max(w[4], w[5]):.0%} | {hp[1]:.0%} | {tq[1]:.1f} | {tq[2]:.1f} |')
            res[(v[0], lab_)] = w
        links = apply_variant(v, 0.2)
        L += ['', f'Shortest rest-to-rest move with 200 g that keeps every motor under {SAFE:.0%}:', '', '| Move | Time (s) | Limiting motor |', '|---|---|---|']
        for name, qa, qb in moves:
            T = fastest(qa, qb, links)
            if T is None: L.append(f'| {name} | not possible within the margin | |'); continue
            u, _ = move_peak(qa, qb, T*1.001, links)
            L.append(f'| {name} | {T:.2f} | {MOTORS[int(np.argmax(u))]} |')
        L.append('')
    apply_variant(VARIANTS[0], 0.0)
    L += ['## The drives (v0.47)', '',
          '- Shoulder: two 56 mm NEMA23 + 10:1 planetaries under the axis, 20T steel pinions on the 56 teeth cut in the steel bones (28:1), preloaded against each other (no backlash). Two extension springs outside (one per side, from an anchor on the cheek support 55 mm above the axis to a pin in the bone 100 mm up the arm) balance most of the arm weight with a torque ~ sin(shoulder); dynamic braking (phases shorted by a relay at power loss) lets the arm settle slowly. No brake.',
          '- Elbow: two new 48 mm NEMA17 (closed loop) outside the cheeks on the shoulder axis, fixed to the turret: Ø8 shafts on 688 through the stubs, stage 1 (20T -> 110T) inside the bones to one fixed Ø10 axle, stage 2 (20T -> 160T) to the side bevels: 44:1. Their weight does not load the shoulder.',
          '- Base: two salvaged NEMA17 + 5:1 planetaries on opposite sides of the 146T ring, preloaded; a 20T encoder pinion on the ring reads the absolute yaw.',
          '- Wrist: two salvaged NEMA17, 100/20 belts, 1:1 bevels: both motors share the pitch load.', '']
    L += ['## Assumptions', '',
          '- Printed parts: PETG, 4 walls, 40% infill (55% of solid mass); gears 85%. Motors: NEMA17 280 g (salvaged) / 390 g (48 mm elbow), 5:1 gearbox 300 g, NEMA23 56 mm 0.70 kg + 10:1 planetary 0.75 kg (on the turntable), hand servo STS3215 55 g.',
          '- Usable motor torque: 48 mm NEMA17 0.50 Nm each at the elbow; salvaged 40 mm NEMA17 0.40 Nm (wrist), 0.35 Nm each (base, two motors); NEMA23 0.85 Nm each (56 mm, about 1.26 Nm holding). Corner speed 10 / 8 / 6 rev/s at 24 V.',
          '- Payload: "at the flange" = hung 64 mm from the wrist centre on the hand mounting face (the spec, like a hook, plus the hand); "at the grip" = 194 mm from the wrist centre.',
          '- Springs: modelled as an ideal sin(shoulder) balancer of 10 Nm; real extension springs give less near the vertical. Measure on the build.',
          '- Efficiency: shoulder planetary + spur gears 85%, elbow two belts + bevels 80%, wrist belt + bevels 85%, base planetary + ring gear 85%. Friction helps when holding still; it is counted against the motor anyway.',
          '- Worst pose: every combination of shoulder, elbow, pronation and wrist pitch in 15° steps, hand above the desk.',
          '- The pick point is on the desk, 400 mm in front of the base, fingers down. Moves are quintic profiles in joint space.', '']
    return '\n'.join(L), res


if __name__ == '__main__':
    txt, res = report()
    open(os.path.join(HERE, 'dynamics_report.md'), 'w').write(txt)
    print(txt)
