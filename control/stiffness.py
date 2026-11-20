"""
Wireless Robot Arm: static stiffness budget (estimate).

    python3 control/stiffness.py           # prints the report and writes control/stiffness_report.md

How far the grip drops when a payload is added, arm stretched out horizontally (the worst case), and which part of the
chain gives. Each element is a simple spring: a torsional stiffness at a joint, or a beam. The figures marked "guess" are
order-of-magnitude values for printed seats and belts: measure them on the first build.
"""
import os, math
import kinematics as K

HERE = os.path.dirname(os.path.abspath(__file__))
G = 9.81
LU, LF, D6 = K.LU/1000, K.LF/1000, K.D6/1000

E = dict(PETG=2.0e9, PETG_CF=5.0e9, PA_CF=7.0e9, aluminium=69e9, steel=205e9)       # Pa
SEAT = 1.0e7           # N/m, radial stiffness of a ball bearing in a printed seat (guess)
BELT_EA = dict(gt2_10=2.0e4, gt2_15=3.0e4, gt2_10_steel=6.0e4)        # N, belt tensile stiffness (guess: fibreglass GT2 about 2 kN per mm of width)
STEPPER_POLES = 50     # a 1.8 deg stepper holds like a spring: torque = T_hold * sin(50 * angle)


def beam_tip(F, M, L, EI):
    """deflection and slope at the end of a cantilever with end force F and end moment M"""
    return F*L**3/(3*EI) + M*L**2/(2*EI), F*L**2/(2*EI) + M*L/EI


def budget(payload=0.5, bone='steel', bone_t=0.004, fore='PETG_CF', fore_t=0.010, belt='gt2_10_steel', pron_span=0.019, rollers=True, wrist_belt=None):
    F = payload*G
    reach = LU + LF + D6
    rows = []                                              # (element, drop at the grip in mm, note)

    # --- base: tilting of the turntable -----------------------------------------------------
    M0 = F*reach
    if rollers: k_tilt = 2.0*2.0e6*0.145**2               # four rollers at 145 mm, 2e6 N/m each on printed posts (guess)
    else: k_tilt = SEAT*0.025**2/2                         # only the two spindle bearings, 25 mm apart
    rows.append(('base tilt', M0/k_tilt*reach, 'rim rollers' if rollers else 'spindle bearings only'))

    # --- shoulder: two steppers acting as springs, through steel spur gears ---------------------
    k_mesh = 2*8.0e7*0.1056**2                             # two steel meshes, 6 mm face (about 13 N/um per mm of face; guess)
    k_shaft = 2*3.0e6*0.1056**2                            # pinion shaft + support puck + cheek, per side (guess)
    k_m = 2*1.26*STEPPER_POLES*K.R_SH**2                   # two 56 mm NEMA23, 1.26 Nm holding each (gearbox stiffness not counted)
    rows.append(('shoulder gears + two motors', M0*(1/k_mesh + 1/k_shaft + 1/k_m)*reach, '10:1 planetary + 56/20 steel spur, two NEMA23'))

    # --- upper arm: both bones are driven (one motor each) ------------------------------------
    EI = E[bone]*bone_t*0.075**3/12                        # one plate, 75 mm (conservative: the strip is 66 wide, the root much wider), bending in its own plane
    Me = F*(LF + D6)
    d, s = beam_tip(F, Me, LU, EI*2.0)                     # two plates, each driven at its root
    rows.append(('upper arm bones', d + s*(LF + D6), f'{bone}, {bone_t*1000:.0f} mm'))

    # --- elbow: belts, steppers, bevel teeth ---------------------------------------------------
    r_p = 160*0.002/(2*math.pi)                            # pitch radius of the 160-tooth pulley
    k_belt = 2*BELT_EA[belt]/0.097*r_p**2                          # stage 2: two equal belts, 125 mm centre distance (intermediate shaft -> elbow, v0.40)
    k_belt1 = (160/20)**2*2*BELT_EA[belt]/0.132*(110*0.002/(2*math.pi))**2   # stage 1 (20T -> 110T outside the bones), seen at the elbow
    k_m = 2*0.30*STEPPER_POLES*K.R_EL**2                   # steppers straight on the belts (no gearbox since v0.40)
    k_teeth = 2*2.0e6*0.040**2                             # printed module 3 teeth, two meshes (guess)
    rows.append(('elbow belts, stage 2', Me/k_belt*(LF + D6), belt))
    rows.append(('elbow belts, stage 1', Me/k_belt1*(LF + D6), belt))
    rows.append(('elbow motors + bevel teeth', Me*(1/k_m + 1/k_teeth)*(LF + D6), ''))

    # --- pronation bearings in the cradle ----------------------------------------------------
    Mp = F*(LF + D6 - 0.06)
    rows.append(('pronation bearing pair', Mp/(SEAT*pron_span**2/2 + 3000.0)*(LF + D6 - 0.06), f'{pron_span*1000:.0f} mm apart'))   # + the 6810 on its own (guess)

    # --- forearm bones --------------------------------------------------------------------------
    EI = 2*E[fore]*fore_t*0.060**3/12                       # two printed plates, about 60 mm wide on average
    d, s = beam_tip(F, F*D6, 0.202, EI)                    # v1.35: longer forearm bones
    rows.append(('forearm bones', d + s*D6, f'{fore}, {fore_t*1000:.0f} mm'))

    # --- wrist: belts, steppers -----------------------------------------------------------------
    wb = wrist_belt or belt
    r_w = K.R_WR*20*0.002/(2*math.pi)
    Mw = F*D6
    rows.append(('wrist belts', Mw/(2*BELT_EA[wb]/0.137*r_w**2)*D6, wb))
    rows.append(('wrist motors', Mw/(2*0.40*STEPPER_POLES*K.R_WR**2)*D6, ''))

    # --- hand on its bearings -------------------------------------------------------------------
    rows.append(('hand roll bearings', F*(D6 - 0.03)/(SEAT*0.022**2/2)*(D6 - 0.03), '6810 ring + shaft bearings'))
    rows = [(n, d*1000, note) for n, d, note in rows]
    return rows, sum(d for _, d, _ in rows)


CASES = [
    ('as designed (v0.46): steel upper-arm bones, printed PETG-CF forearm bones, two-stage steel-cord elbow belts, two geared NEMA23', dict()),
    ('with ordinary fibreglass GT2 belts', dict(belt='gt2_10')),
    ('with aluminium 6 mm forearm bones instead of printed', dict(fore='aluminium', fore_t=0.006)),
    ('without the rim rollers', dict(rollers=False)),
]


def report(payload=0.5):
    rows, tot = budget(payload)
    L = ['# Stiffness budget (estimate)', '',
         'Generated by `control/stiffness.py`. Arm stretched out horizontally, %.0f g added at the grip (%.0f mm from the shoulder). '
         'The seat and belt stiffnesses are order-of-magnitude guesses: the ranking is more reliable than the totals.' % (payload*1000, (LU+LF+D6)*1000), '',
         '## Where the grip drops, as designed', '', '| Element | Drop at the grip (mm) | Share |', '|---|---|---|']
    for n, d, note in sorted(rows, key=lambda r: -r[1]):
        L.append(f'| {n}{" (" + note + ")" if note else ""} | {d:.2f} | {d/tot:.0%} |')
    L += [f'| **Total** | **{tot:.1f}** | |', '', '## What each change is worth', '', '| Change | Total drop (mm) | Against as designed |', '|---|---|---|']
    for name, kw in CASES:
        _, t = budget(payload, **kw)
        L.append(f'| {name} | {t:.1f} | {t/tot - 1:+.0%} |' if kw else f'| {name} | {t:.1f} | |')
    L += ['', '## Reading it', '',
          '- The belts are the largest single item even with steel cords. Their stiffness at the joint depends on the big pulley radius and on the belt, not on the ratio.',
          '- The bones are a small part of the vertical drop. Aluminium matters more for sideways and twisting loads and for creep, which this budget does not cover.',
          '- The elbow flexion and the wrist pitch have an encoder on the joint, after the belt. The firmware can read the sag and correct it, so a static belt stretch does not have to become a position error.',
          '- Backlash is not in this budget: under gravity the gears stay loaded on one flank. It shows when the load reverses.', '']
    return '\n'.join(L), rows, tot


if __name__ == '__main__':
    txt, rows, tot = report(); open(os.path.join(HERE, 'stiffness_report.md'), 'w').write(txt); print(txt)
