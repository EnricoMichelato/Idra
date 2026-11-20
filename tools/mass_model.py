"""Mass model of the arm from the CAD: mass, centre of mass and inertia of each moving link, in its own joint frame.
Writes control/mass_model.json (used by control/dynamics.py, which needs only numpy).

    python3 tools/mass_model.py

Densities are estimates (printed parts are not solid); motors use catalogue masses. Weigh the real parts and edit OVERRIDE."""
import os, sys, json
import numpy as np
ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, 'mechanical/cad/arm_v0')); sys.path.insert(0, os.path.join(ROOT, 'mechanical/cad/wrist_v0'))
import cadquery as cq
import arm_v0 as A

RHO = dict(printed=1.27*0.55, gear=1.27*0.85, tpu=1.21*0.45, steel=7.85, alu=2.70)     # g/cm3 (printed: 4 walls + 40% infill)
OVERRIDE = [  # (substring of the part name, grams) for bought parts modelled as plain blocks; first match wins
    ('NEMA23', 700.0), ('gearbox_shoulder_A_planetary', 750.0), ('gearbox_shoulder_B_planetary', 750.0), ('foot_yaw_gearbox', 400.0),
    ('gearbox_elbow', 300.0), ('elbow_gearbox_A_planetary', 300.0), ('elbow_gearbox_B_planetary', 300.0), ('NEMA17', 280.0), ('flange_hub', 150.0),
    ('hand_gearmotor', 91.0), ('hand_shaft_hub', 30.0), ('camera_xiao', 8.0),
    ('mid_pulley_A_110T', 110.0), ('mid_pulley_B_110T', 110.0), ('mid_pulley_A_20T', 15.0), ('mid_pulley_B_20T', 15.0), ('elbow_mpulley_A_20T', 15.0), ('elbow_mpulley_B_20T', 15.0),   # bought aluminium GT2 pulleys
]
EXACT = {'W_hand_worm': 38.0, 'W_hand_worm_hub': 5.0}       # the bought steel worm, 43 g with its hub
def density(n):
    """g/cm3 for parts that have no catalogue mass"""
    m = A.material(n)
    if n.endswith('_alu') or 'hub_alu' in n: return 2.7
    if 'brg' in n: return 5.5                     # a bearing is not a solid ring
    if m: return RHO[m]
    if 'ferrite' in n: return 3.2                 # flexible sintered ferrite sheet with its PET film and adhesive
    if 'coil' in n: return 3.1                    # PCB coil: FR-4 with 4 x 70 um copper
    if 'magnet' in n: return 7.5
    if 'belt' in n: return 1.2
    if 'cable' in n: return 1.6
    return 1.8                                     # boards, sensors, small parts

def link_of(n):
    g = A.group(n)
    if n.startswith(('wrist_ghost', 'W_apple')) or 'object' in n or 'ghost' in n: return None
    if g in ('foot', 'yawpin'): return None
    if g in ('yaw', 'shpinA', 'shpinB'): return 1
    if g in ('arm', 'sideA', 'sideB'): return 2                 # side bevels stay on the elbow axis: mass of the upper arm
    if g in ('cradle', 'idle'): return 3
    if g in ('fore', 'W_fixed', 'W_A', 'W_B'): return 4
    if g in ('W_pitch', 'W_idle'): return 5
    if g in ('W_roll', 'W_worm', 'W_finger'): return 6
    raise ValueError((n, g))

def main():
    parts, info = A.build(A.P)
    ze, wz = info['ze'], info['wrist_z']
    origin = {1: (0, 0, 0), 2: (0, 0, 0), 3: (0, 0, ze), 4: (0, 0, ze), 5: (0, 0, wz), 6: (0, 0, wz)}
    acc = {k: dict(m=0.0, mc=np.zeros(3), items=[]) for k in origin}
    for n, s in parts.items():
        k = link_of(n)
        if k is None: continue
        v = s.Volume()/1000.0                                  # cm3
        fixed = EXACT.get(n, next((g for sub, g in OVERRIDE if sub in n), None))
        m = fixed if fixed is not None else v*density(n)       # grams
        c = s.Center(); c = np.array([c.x, c.y, c.z])
        if n.startswith('W_'): c = c + np.array([0, 0, wz])       # wrist parts are built about the wrist centre
        I = np.array(cq.Shape.matrixOfInertia(s)).reshape(3, 3)*(m/(v*1000.0))     # g mm2 about the part's own centre
        acc[k]['m'] += m; acc[k]['mc'] += m*c; acc[k]['items'].append((n, m, c, I))
    out = dict(source='arm v0.28 / wrist v1.24', units='kg, m, kg m2; com and inertia in the link frame, inertia about the com', links={})
    for k, a in acc.items():
        com = a['mc']/a['m']; I = np.zeros((3, 3))
        for n, m, c, Ip in a['items']:
            d = c - com; I += Ip + m*((d @ d)*np.eye(3) - np.outer(d, d))
        out['links'][str(k)] = dict(mass=round(a['m']/1000, 4), com=[round(float(x)/1000, 5) for x in com - np.array(origin[k])],
                                    inertia=[[float('%.4g' % (x*1e-9)) for x in r] for r in I],
                                    heaviest=[(n, round(m)) for n, m, _, _ in sorted(a['items'], key=lambda t: -t[1])[:6]])
        print(k, out['links'][str(k)]['mass'], 'kg  com', out['links'][str(k)]['com'], out['links'][str(k)]['heaviest'][:4])
    json.dump(out, open(os.path.join(ROOT, 'control/mass_model.json'), 'w'), indent=1)

if __name__ == '__main__':
    main()
