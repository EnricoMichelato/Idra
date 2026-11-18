"""
Wireless Robot Arm - base + cycloidal shoulder + upper arm + differential elbow + forearm (v0.28: yaw on two 6810 in a cartridge, steel-pin cycloid with 6802, bones bolted to the forearm base, printable foot; v0.27, rounded flanges, chunky cross blocks, original colours; v0.26: matte black PLA, flush idle satellite, spider stub under the forearm satellite, v0.25: one fit file + unified hardware (6 bearing types, M3/M5), 6 DOF, round lightening holes, retro 7-segment display, cables, complete with wrist and hand; elbow: module 3 spiral bevels, idle satellite, 2 AS5600, preload, back-side tensioners)
Parametric CadQuery generator. Run:  python3 arm_v0.py   (build, interference check, export to ./out)
Frame: origin = shoulder axis, X = lateral (shoulder / elbow axes), Y = front, Z = up, units mm.

Concept (user decisions):
  * 6 DOF: base yaw, shoulder, elbow flexion + forearm pronation (bevel differential), wrist pitch, endless wrist roll.
  * ELBOW = the wrist architecture scaled up: two 30T side bevels (module 3, spiral 20 deg, the two side gears are the
    SAME printed part) each in ONE piece with a 160T GT2 pulley a little bigger than the bevel, on one static pin clamped in the
    common pin; the cradle (carrier) sits OUTSIDE the plates and holds the 15T spiral bevel + hub of the forearm.
    The pin is clamped in the cradle walls and turns in a 6801 in each plate; a spider keyed on the pin carries an
    IDLE 15T satellite (shaft + 2x 688) opposite the forearm one -> half the tooth load, balanced separating forces.
    AS5600 encoders: flexion = magnet in the pin end + chip on a bracket of the right plate; pronation = magnet in the
    idle shaft end + chip on a small PCB inside the spider.
    Axial preload: a wave spring between the inner ring of the plate 6801 and the inner ring of the pulley-end 6801
    (both turn with the pin) presses each side gear into the satellites (zero backlash, no rubbing). Phase dimples on the toe faces.
    Two NEMA17 back to back inside the upper arm, ONE 10 mm GT2 belt each (8.9:1), two 20T idlers per belt on
    slotted M5 bolts, adjusted by an M3 pull screw in a housing on the outer face of each plate.
    same direction -> flexion ; opposite -> pronation (1:1 in the bevels).
  * SHOULDER (symmetric): NEMA23 + printed CYCLOIDAL reducer (21 ring pins, 20:1) coaxial on the shoulder axis INSIDE
    the two humerus plates, held by a central pedestal on the turntable (no side uprights). The reducer output drum is
    bolted to the right plate; the left plate turns on a 6810 around the static hollow hub (cable entry).
  * BASE: turntable (internal 100T ring gear, 15T pinion on a NEMA17 in the foot, 6.7:1) on a printed slewing ring.
    Foot with electronics (ESP32-S3 + 6x TMC2209, buck, fan, connector panel, e-stop in the front wall, Hall homing),
    desk lugs printed separately. Every printed part fits the 256 x 256 bed.
"""
import os, sys, itertools, math
import cadquery as cq
from cq_gears import SpurGear, RingGear, BevelGearPair
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'wrist_v0'))
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'common'))
from fit import FIT, seat as fseat, boss as fboss, R_INS3, R_INS5, R_M3, R_M5, R_TAP3

P = dict(
    # elbow differential (30T side bevels on X, 15T on the forearm axis), module 3, spiral (helix) 20 deg
    module=3.0, z_side=30, z_out=15, face=14.0, pressure=20.0, helix=20.0, backlash=0.2,
    # axial preload of the side gears: shim on the 6801 inner ring + wave spring in a counterbore of the plate
    pre_shim=(15.2, 0.8), pre_spring=(18.0, 2.2), pre_cb=(18.8, 2.0),
    # adjustable belt tensioners: 20T GT2 idler on an M5 bolt in a slot of the plate, both strands of each belt
    # (outside the loop, on the smooth back of the belt, pushing each strand IN: more wrap on the 16T pulley)
    idl_r=6.0, idl_d0=6.0, idl_adj=3.0,
    # belts: motor 16T -> elbow pulley (as big as the side bevel), one belt per motor
    t_motor=20, t_elb=160, belt_teeth_A=190, belt_teeth_B=230, belt_w=10.0, belt_t=1.5, mp_w=12.0, mp_od=12.2, mp_flange_od=16.0,
    # elbow motors: NEMA17 + bolt-on 5:1 planetary gearbox, staggered along the arm (A above B), in a printed carrier
    el_gb=(42.0, 41.0, 5), el_motor_len=39.0, el_shaft=(8.0, 23.5), el_wall=4.0, half_min=65.5,
    # upper arm (humerus): two plates, motors back to back
    n17_sq=42.3, n17_hole=31.0,
    plate_t=6.0, pulley_gap=2.5,                # bones: laser-cut 6 mm aluminium
    sh_r=40.0, el_r=33.0, pin_d=12.0,
    cover=(90.0, 4.0),                         # one cover part for the whole arm: length, thickness (2x arm, 4x forearm)
    # cradle (outside the humerus plates)
    wall_gap=1.0, wall_t=7.5, ear_r=26.0, cr_fillet=5.0, cr_y=42.0, cr_plate_t=26.0, b61810=(65.0, 50.0, 7.0),
    hub_d=30.0, tube_od=38.0, tube_id=20.0, brg_od=42.0, brg_id=30.0, brg_w=7.0,
    Lf=240.0,                                  # forearm: elbow axis -> wrist apex (about as long as the upper arm)
    # shoulder: NEMA23 with integrated steel planetary gearbox
    n23_sq=57.0, n23_len=56.0, gb_d=60.0, gb_len=61.65, gb_pilot=(40.0, 3.0), gb_shaft=(14.0, 34.0), gb_ratio=20,
    sh_hub_fl=(50.0, 6.0), sh_hub_neck=28.0, sh_hub_pcd=38.0,          # 14 mm keyed flange hub (check the part you buy)
    # base
    up_t=8.0, up_hw=35.0, z_tt=-65.0, tt_t=10.0, tt_r=115.0,
    yaw_m=1.5, yaw_ring=100, yaw_pin=15, yaw_pin_phase=6.0, yaw_w=12.0, slew=(86.0, 106.0),
    yaw_motor_len=39.0, yaw_gb=(42.0, 41.0, 5), yaw_shaft=(8.0, 23.5),   # NEMA17 with a 5:1 high-precision planetary (17HS15-1684S-HG5 class)
    # shoulder gravity balancer: cable from a lobe of the left bone, over two U-groove bearings, to a spring on the turntable
    bal=dict(a=54.0, b=40.0, torque=7.0, pulley=(13.0, 4.0, 7.0), spring=(20.0, 2.5, 52.0), y=-62.0, z=-54.5, x_anchor=79.5),
    roller_r=98.0, roller_az=(45.0, 165.0, 270.0),                       # three 625 bearings under the turntable rim
    foot_r=118.0, foot_h=96.0, ctrl=(110.0, 75.0, 25.0), buck=(45.0, 25.0, 15.0), fan=40.0, estop_d=40.0,
)

O3 = cq.Vector(0, 0, 0)
XV, YV, ZV = cq.Vector(1, 0, 0), cq.Vector(0, 1, 0), cq.Vector(0, 0, 1)

def bx(x0, x1, y0, y1, z0, z1):
    x0, x1 = sorted((x0, x1)); y0, y1 = sorted((y0, y1)); z0, z1 = sorted((z0, z1))
    return cq.Workplane('XY').box(x1-x0, y1-y0, z1-z0, centered=False).translate((x0, y0, z0))
def cyl_x(x0, x1, r, ri=0.0):
    x0, x1 = min(x0, x1), max(x0, x1)
    w = cq.Workplane('YZ').workplane(offset=x0).circle(r)
    if ri: w = w.circle(ri)
    return w.extrude(x1-x0)
def cyl_y(y0, y1, r, cx=0.0, cz=0.0, ri=0.0):
    y0, y1 = min(y0, y1), max(y0, y1)
    w = cq.Workplane('XZ').workplane(offset=-y1).center(cx, cz).circle(r)
    if ri: w = w.circle(ri)
    return w.extrude(y1-y0)
def cyl_z(z0, z1, r, ri=0.0, cx=0.0, cy=0.0):
    w = cq.Workplane('XY').workplane(offset=z0).center(cx, cy).circle(r)
    if ri: w = w.circle(ri)
    return w.extrude(z1-z0)
def prism_yz(pts, x0, x1):
    x0, x1 = sorted((x0, x1))
    return cq.Workplane('YZ').workplane(offset=x0).polyline(pts).close().extrude(x1-x0)
def prism_xy_r(x0, x1, y0, y1, z0, z1, r):
    """box with rounded vertical edges (radius r)"""
    return cq.Workplane('XY').workplane(offset=z0).center((x0+x1)/2, (y0+y1)/2).rect(x1-x0, y1-y0).extrude(z1-z0).edges('|Z').fillet(min(r, (x1-x0)/2-0.01, (y1-y0)/2-0.01))
def prism_xy_slot(cx, cy, r, half, z0, z1):
    """vertical slot (elongated along Y by +-half) for an adjustable screw"""
    return (cq.Workplane('XY').workplane(offset=z0).center(cx, cy).slot2D(2*half+2*r, 2*r, 90).extrude(z1-z0))
def prism_xz(pts, y0, y1):
    y0, y1 = sorted((y0, y1))
    return cq.Workplane('XZ').workplane(offset=-y1).polyline(pts).close().extrude(y1-y0)
def xr(a, b): return sorted((a, b))
def pitch_d(teeth): return teeth*2.0/math.pi            # GT2 pitch diameter
def _circ(cy, cz, r, n=72):
    return [(cy+r*math.cos(2*math.pi*k/n), cz+r*math.sin(2*math.pi*k/n)) for k in range(n)]
def _hull(pts):
    pts = sorted(set(pts))
    cr = lambda o, a, b: (a[0]-o[0])*(b[1]-o[1])-(a[1]-o[1])*(b[0]-o[0])
    lo, up = [], []
    for q in pts:
        while len(lo) >= 2 and cr(lo[-2], lo[-1], q) <= 0: lo.pop()
        lo.append(q)
    for q in reversed(pts):
        while len(up) >= 2 and cr(up[-2], up[-1], q) <= 0: up.pop()
        up.append(q)
    return lo[:-1] + up[:-1]
def pulley_gen(x0, r_body, r_fl, w, r_bore, fl=1.0):
    pu = cyl_x(x0, x0+fl, r_fl).union(cyl_x(x0+fl, x0+w-fl, r_body)).union(cyl_x(x0+w-fl, x0+w, r_fl))
    return pu.cut(cyl_x(x0-1, x0+w+1, r_bore))
def belt_loop(xc, r1, r2, cd, w, t):
    """closed belt loop in the YZ plane at x=xc; big pulley (r1) at the origin, small (r2) cd below it"""
    outer = _hull(_circ(0, 0, r1+t) + _circ(0, -cd, r2+t))
    inner = _hull(_circ(0, 0, r1) + _circ(0, -cd, r2))
    mk = lambda pts: cq.Workplane('YZ').workplane(offset=xc-w/2).polyline(pts).close().extrude(w)
    return mk(outer).cut(mk(inner))
def belt_loop_c(xc, circles, w, t):
    """closed belt loop in the YZ plane at x=xc around convex circles [(cy, cz, r_inner)], thickness t"""
    outer = _hull(sum((_circ(cy, cz, r+t) for cy, cz, r in circles), []))
    inner = _hull(sum((_circ(cy, cz, r) for cy, cz, r in circles), []))
    mk = lambda pts: cq.Workplane('YZ').workplane(offset=xc-w/2).polyline(pts).close().extrude(w)
    return mk(outer).cut(mk(inner))
def hull_len(circles, n=1500):
    h = _hull(sum((_circ(cy, cz, r, n) for cy, cz, r in circles), []))
    return sum(math.dist(h[i], h[(i+1) % len(h)]) for i in range(len(h)))
BT_IN, BT_OUT = 0.38, 1.12        # model belt: tooth side 0.38 inside the pitch line, back 1.12 outside
def belt_path(circ):
    """closed belt pitch path, circles [(cy, cz, r, s)] in CCW order in (y, z); s = +1 pulley (belt around it),
    s = -1 back-side idler outside the loop. Returns (length, points, outward normals)."""
    n_ = len(circ); tang = []
    for i in range(n_):
        (y1, z1, r1, s1), (y2, z2, r2, s2) = circ[i], circ[(i+1) % n_]
        D = (y2-y1, z2-z1); L = math.hypot(*D); a = s1*r1 - s2*r2; pD = math.atan2(D[1], D[0])
        for sgn in (1, -1):
            ph = pD + sgn*math.acos(max(-1.0, min(1.0, a/L))); nv = (math.cos(ph), math.sin(ph))
            if -nv[1]*D[0] + nv[0]*D[1] > 0: break
        tang.append((ph, (y1+s1*r1*nv[0], z1+s1*r1*nv[1]), (y2+s2*r2*nv[0], z2+s2*r2*nv[1])))
    total, pts, nrm = 0.0, [], []
    for i in range(n_):
        y, z, r, s = circ[i]; phin, phout = tang[i-1][0], tang[i][0]
        ang = ((phout-phin) if s > 0 else (phin-phout)) % (2*math.pi); total += r*ang
        m = max(2, int(ang*r/0.8)+2)
        for k in range(m+1):
            ph = phin + (ang if s > 0 else -ang)*k/m; nv = (math.cos(ph), math.sin(ph))
            pts.append((y+s*r*nv[0], z+s*r*nv[1])); nrm.append(nv)
        total += math.dist(tang[i][1], tang[i][2])
    return total, pts, nrm
def belt_solid(xc, circ, w):
    _, pts, nrm = belt_path(circ)
    inner = [(y-BT_IN*a, z-BT_IN*b) for (y, z), (a, b) in zip(pts, nrm)]
    outer = [(y+BT_OUT*a, z+BT_OUT*b) for (y, z), (a, b) in zip(pts, nrm)]
    mk = lambda q: cq.Workplane('YZ').workplane(offset=xc-w/2).polyline(q).close().extrude(w)
    return mk(outer).cut(mk(inner))
def tensioner_out(C, r1, r2, reff, z_rel, d0):
    """back-side idlers (front, back strand) OUTSIDE the loop pushing each strand IN by d0 at height z_rel
    (elbow axis = 0, motor at -C); returns centres [(cy, cz)] and the outward strand normals [(ny, nz)]"""
    a = math.asin((r1 - r2)/C); ny, nz = math.cos(a), -math.sin(a)
    out, nrm = [], []
    for s in (1, -1):
        p1 = (s*r1*ny, r1*nz); p2 = (s*r2*ny, -C + r2*nz)
        t = (z_rel - p1[1])/(p2[1] - p1[1]); fy = p1[0] + t*(p2[0] - p1[0])
        out.append((fy + s*ny*(reff - d0), z_rel + nz*(reff - d0))); nrm.append((s*ny, nz))
    return out, nrm
def tensioner_centres(C, r1, r2, ri, z_rel, d0):
    """idler centres (front, back) pushing each strand OUT by d0 at height z_rel (elbow axis = 0, motor at -C);
    returns [(cy, cz)], outward unit normals [(ny, nz)]"""
    a = math.asin((r1 - r2)/C); ny, nz = math.cos(a), -math.sin(a)
    out, nrm = [], []
    for s in (1, -1):
        p1 = (s*r1*ny, r1*nz); p2 = (s*r2*ny, -C + r2*nz)
        t = (z_rel - p1[1])/(p2[1] - p1[1]); fy = p1[0] + t*(p2[0] - p1[0])
        out.append((fy + s*ny*(d0 - ri), z_rel + nz*(d0 - ri))); nrm.append((s*ny, nz))
    return out, nrm
def cyl_dir(p0, d, h, r):
    return cq.Workplane('XY').newObject([cq.Solid.makeCylinder(r, h, cq.Vector(*p0), cq.Vector(*d))])
def centre_distance(L, D, d):
    q = L - math.pi*(D+d)/2
    return (q + math.sqrt(q*q - 2*(D-d)**2))/4
def cycloid_pts(N, R, rp, e, n=720):
    """cycloidal disc (N ring pins, N-1 lobes) centred on the origin, (y, z) points"""
    pts = []
    for k in range(n):
        t = 2*math.pi*k/n
        psi = math.atan2(math.sin((1-N)*t), R/(e*N) - math.cos((1-N)*t))
        y = R*math.cos(t) - rp*math.cos(t+psi) - e*math.cos(N*t)
        z = -R*math.sin(t) + rp*math.sin(t+psi) + e*math.sin(N*t)
        pts.append((y, z))
    return pts


def cover_drill(*a, **k):
    import wrist_v0 as W
    return W.cover_drill(*a, **k)
def cover_part(*a, **k):
    """the ribbed cover with tabs is defined once, in wrist_v0"""
    import wrist_v0 as W
    return W.cover_part(*a, **k)

def cable(pts, r):
    """flexible cable / harness: circle swept along a 3D spline through pts"""
    vs = [cq.Vector(*q) for q in pts]
    e = cq.Edge.makeSpline(vs)
    t = e.tangentAt(0).normalized()
    ref = cq.Vector(0, 0, 1) if abs(t.z) < 0.9 else cq.Vector(1, 0, 0)
    xd = t.cross(ref).normalized()
    path = cq.Workplane('XY').add(cq.Wire.assembleEdges([e]))
    for fr in (True, False):
        try:
            prof = cq.Workplane(cq.Plane(origin=vs[0], xDir=xd, normal=t)).circle(r)
            return prof.sweep(path, isFrenet=fr, transition='round').val()
        except Exception:
            pass
    return cable_poly(pts, r)
def cable_poly(pts, r):
    """cable laid along a polyline (tidy routing in grooves / clips): cylinders + spheres at the bends"""
    sol = None
    for p0, p1 in zip(pts[:-1], pts[1:]):
        d = cq.Vector(*p1) - cq.Vector(*p0)
        if d.Length < 1e-6: continue
        seg = cq.Solid.makeCylinder(r, d.Length, cq.Vector(*p0), d.normalized())
        sol = seg if sol is None else sol.fuse(seg)
    for q in pts[1:-1]:
        sol = sol.fuse(cq.Solid.makeSphere(r, cq.Vector(*q), angleDegrees1=-90, angleDegrees2=90))
    return sol.clean()

def round_plan(solid, hx, hy, R, axis_r, z0, z1):
    """round the corners of a flange seen from above (rectangle 2hx x 2hy, radius R), keeping the full ring around
    the joint axis (X) of radius axis_r"""
    keep = (cq.Workplane('XY').workplane(offset=z0).rect(2*hx, 2*hy).extrude(z1 - z0).edges('|Z').fillet(R)
            .union(cq.Workplane('YZ').workplane(offset=-hx-2).circle(axis_r).extrude(2*hx+4)))
    return solid.intersect(keep)

def build(p=P):
    parts = {}
    mw, bt = p['mp_w'], p['belt_t']

    # ================= ELBOW DIFFERENTIAL geometry (wrist style) =================
    pair = BevelGearPair(p['module'], p['z_side'], p['z_out'], p['face'], axis_angle=90.0,
                         pressure_angle=p['pressure'], helix_angle=p['helix'], backlash=p['backlash'])
    asm = pair.assemble(gear_build_args={'bore_d': p['pin_d']+0.4}, pinion_build_args={'bore_d': p['tube_id']})
    kids = {c.name: c for c in asm.children}
    hg = float(pair.gear.cone_h)
    to_frame = lambda s_: (s_.translate(cq.Vector(0, 0, -hg)).rotate(O3, XV, 180).rotate(O3, YV, 90))
    SB = to_frame(kids['gear'].obj.moved(kids['gear'].loc))          # 36T side bevel on +X (back plane at x = hg)
    OUT = to_frame(kids['pinion'].obj.moved(kids['pinion'].loc))     # 18T on the forearm axis +Z
    a_off = 180.0/p['z_side'] if p['z_out'] % 2 else 0.0               # odd satellite: half a side tooth of phase
    to_A = lambda s_: s_.rotate(O3, ZV, 180).rotate(O3, XV, a_off)     # side gear A = SAME part as B, turned over
    from_A = lambda s_: s_.rotate(O3, XV, -a_off).rotate(O3, ZV, 180)
    Rh = SB.BoundingBox().ymax                                        # side bevel heel radius
    toe = SB.BoundingBox().xmin
    z_out_back = OUT.BoundingBox().zmax
    # EXOSKELETON layout: the bones are the outermost parts. From the centre: bevel, pulley, cradle wall, bone.
    xc = max(50.0, hg + p['belt_w']/2 + 0.5)                             # a geared motor (80 long) must end short of the other belt
    pul_out = xc + mw/2
    u0 = max(pul_out + p['pulley_gap'], p['half_min']); u1 = u0 + p['plate_t']   # humerus plates (the shoulder unit sets the minimum)
    wall_out = u0 - p['wall_gap']; wall_in = wall_out - p['wall_t']      # cradle walls INSIDE the bones, between pulley and bone
    assert wall_in >= pul_out + 0.8, 'no room for the cradle wall between pulley and bone'
    D_e = pitch_d(p['t_elb']); D_m = pitch_d(p['t_motor'])
    rb_e = (D_e - 0.76)/2; rf_e = max(rb_e + 1.2, Rh)
    Lu = 205.0                                                           # shoulder axis -> elbow axis
    # one belt per motor, of DIFFERENT length: motor A sits above motor B on the arm centre line. Each centre distance
    # comes from its belt (pitch length = 2 * teeth) with the two tensioner idlers at their nominal position.
    r1p, r2p, reff = D_e/2, D_m/2, p['idl_r'] + BT_OUT
    DRV = {}
    for side_, sg_ in (('A', -1), ('B', 1)):
        teeth_ = p[f'belt_teeth_{side_}']
        zi_ = -(r1p + 8.0) if side_ == 'A' else -93.0                     # idler height from the elbow axis: A just under the big pulley, B below motor A's cover zone
        def bcirc(C_, d0):
            ic, _ = tensioner_out(C_, r1p, r2p, reff, zi_, d0)
            (fy, fz), (by, bz) = ic
            return [(0, 0, r1p, 1), (by, bz, reff, -1), (0, -C_, r2p, 1), (fy, fz, reff, -1)]
        blen = lambda C_, d0: belt_path(bcirc(C_, d0))[0]
        lo_, hi_ = 62.0, 170.0
        for _ in range(40):
            mid_ = (lo_ + hi_)/2
            if blen(mid_, p['idl_d0']) > 2.0*teeth_: hi_ = mid_
            else: lo_ = mid_
        C_s = mid_
        ic_, in_ = tensioner_out(C_s, r1p, r2p, reff, zi_, p['idl_d0'])
        DRV[side_] = dict(sg=sg_, C=C_s, z=Lu - C_s, idl_c=ic_, idl_n=in_, circ=bcirc(C_s, p['idl_d0']),
                          dL=[blen(C_s, p['idl_d0'] + s_*p['idl_adj']) - 2.0*teeth_ for s_ in (-1, 1)])
    zA, zB = DRV['A']['z'], DRV['B']['z']
    assert zA - zB >= p['n17_sq'] + 1.5, 'elbow motors overlap: change the belt lengths'
    C, z_m, idl_c, dL = DRV['B']['C'], zB, DRV['B']['idl_c'], DRV['B']['dL']
    ze = Lu
    plate_r_el = p['el_r']                                               # humerus end radius at the elbow (narrower than the pulley -> more flexion)
    # flex-encoder bracket straight along the humerus (toward the shoulder), its foot 44-54 mm from the elbow axis where
    # the cradle (now with R26 ears around the bearing ends) never sweeps; 2 M3 inserts side by side
    enc_pts = [(-5.5, -49.0), (5.5, -49.0)]

    # ================= UPPER ARM: flat aluminium bones, geared motors in a printed carrier, belts =================
    sq, sx = p['n17_sq'], p['n17_hole']/2
    gq, gl_, _ = p['el_gb']; ml_ = p['el_motor_len']; wt_ = p['el_wall']
    xf = xc - mw/2 - 0.5 - wt_                                           # gearbox face (the carrier wall is between it and the pulley)
    R_TAP5 = 2.1
    prof = _hull(_circ(0, 0, p['sh_r']) + _circ(0, ze, plate_r_el))      # tapered plate, round ends: a 2D outline for laser cutting
    yb1 = -(sq/2 + 1.0); yb0 = yb1 - 4.0                                 # carrier back plate, behind the motors
    zc_0, zc_1 = zB - sq/2 - 1.0, zA + sq/2 + 1.0
    car_L = [(-17.0, zc_0+8.0), (-17.0, zA-sq/2-10.0)]                   # carrier bolts through the left bone (y, z)
    car_R = [(-16.0, zc_0+6.0), (12.0, zc_0+6.0)]                        # ... and the right bone, under motor B's pulley
    for sg, nm, side_ in ((1, 'R', 'B'), (-1, 'L', 'A')):
        x0, x1 = xr(sg*u0, sg*u1)
        pl = prism_yz(prof, x0, x1)
        pl = pl.cut(cyl_x(x0-1, x1+1, fseat(21.0)).translate((0, 0, ze)))                      # 6801 in a through bore (ream; Loctite 638)
        if sg > 0:
            for ey, ez in enc_pts: pl = pl.cut(cyl_x(x0-1, x1+1, R_TAP3).translate((0, ey, ze+ez)))   # tapped M3, flex-encoder bracket
            pl = pl.cut(cyl_x(x0-1, x1+1, p['gb_shaft'][0]/2+1.0))
            for k in range(4):
                a = math.pi/4 + k*math.pi/2                              # 4x tapped M5 for the flange hub on the gearbox shaft
                pl = pl.cut(cyl_x(x0-1, x1+1, R_TAP5).translate((0, p['sh_hub_pcd']/2*math.cos(a), p['sh_hub_pcd']/2*math.sin(a))))
        else:                                                              # 6810 bearing around the static hub; lobe for the balancer cable
            pl = pl.union(prism_yz(_hull(_circ(0, -p['bal']['b'], 9.0) + _circ(0, -20.0, 22.0)), x0, x1))
            pl = pl.cut(cyl_x(x0-1, x1+1, 32.5)).cut(cyl_x(x0-1, x1+1, R_TAP5).translate((0, 0, -p['bal']['b'])))
        for cy_, cz_ in (car_R if sg > 0 else car_L): pl = pl.cut(cyl_x(x0-1, x1+1, R_M5).translate((0, cy_, cz_)))   # motor carrier
        for sy_ in (-1, 1): pl = pl.cut(cyl_x(x0-1, x1+1, R_M3).translate((0, sy_*17.0, DRV[side_]['z'])))    # 2x M3 for the bearing block of the motor shaft
        adj = p['idl_adj']                                                 # tensioner slots (M5 idler bolt)
        for (cy_, cz_), (ny_, nz_) in zip(DRV[side_]['idl_c'], DRV[side_]['idl_n']):
            sl = _hull(_circ(cy_-ny_*adj, ze+cz_-nz_*adj, R_M5) + _circ(cy_+ny_*adj, ze+cz_+nz_*adj, R_M5))
            pl = pl.cut(prism_yz(sl, x0-1, x1+1))
        parts[f'arm_humerus_{nm}'] = pl.val()
    # motor carrier (printed, PETG-CF): one piece between the bones. A wall per gearbox face, a back plate, a block on each bone.
    car = bx(-(xf+wt_), xf+wt_, yb0, yb1, zc_0, zc_1)
    for side_, d_ in DRV.items():
        sg = d_['sg']; zc = d_['z']
        wall = bx(*xr(sg*xf, sg*(xf+wt_)), yb0, sq/2+1.0, zc-sq/2-1.0, zc+sq/2+1.0).cut(cyl_x(*xr(sg*(xf-1), sg*(xf+wt_+1)), 11.5).translate((0, 0, zc)))
        for yy, zz in ((sx, sx), (-sx, sx), (sx, -sx), (-sx, -sx)):
            wall = wall.cut(cyl_x(*xr(sg*(xf-1), sg*(xf+wt_+1)), R_M3).translate((0, yy, zc+zz)))
        car = car.union(wall)
    blkL = bx(-(u0-0.3), -(xf+wt_)+0.01, yb0, -8.0, zc_0, zA-sq/2-1.0)
    blkR = bx(xf+wt_-0.01, u0-0.3, yb0, sq/2+1.0, zc_0, min(zc_0+12.0, zB-12.5))
    car = car.union(blkL).union(blkR)
    for cy_, cz_ in car_L: car = car.cut(cyl_x(-(u0+1), -(u0-11.0), R_INS5).translate((0, cy_, cz_)))
    for cy_, cz_ in car_R: car = car.cut(cyl_x(u0-11.0, u0+1, R_INS5).translate((0, cy_, cz_)))
    for d_ in DRV.values():                                                # clearance around the idler bolt heads
        for cy_, cz_ in d_['idl_c']:
            car = car.cut(cyl_x(*xr(d_['sg']*(xc-mw/2-6.0), d_['sg']*(xc+mw/2+1.5)), 9.0).translate((0, cy_, ze+cz_)))
    parts['arm_motor_carrier'] = car.val()
    for side, d_ in DRV.items():
        sg = d_['sg']; zc = d_['z']
        parts[f'arm_gearbox_elbow_{side}_planetary_5'] = bx(*xr(sg*xf, sg*(xf-gl_)), -gq/2, gq/2, zc-gq/2, zc+gq/2).val()
        parts[f'arm_motor_elbow_{side}_NEMA17'] = bx(*xr(sg*(xf-gl_), sg*(xf-gl_-ml_)), -sq/2, sq/2, zc-sq/2, zc+sq/2).val()
        parts[f'arm_motor_elbow_{side}_shaft'] = cyl_x(*xr(sg*xf, sg*(xf+p['el_shaft'][1])), p['el_shaft'][0]/2).translate((0, 0, zc)).val()
        mp = pulley_gen(sg*xc-mw/2, p['mp_od']/2, p['mp_flange_od']/2, mw, p['el_shaft'][0]/2)
        parts[f'arm_mpulley_{side}_20T'] = mp.translate((0, 0, zc)).val()
        # the shaft end runs in a 688 held by a block screwed to the inner face of the bone: the belt pull goes into the bone
        xe = xf + p['el_shaft'][1]                                         # shaft end
        blk = prism_yz(_hull(_circ(0, zc, 11.5) + _circ(-17.0, zc, 5.5) + _circ(17.0, zc, 5.5)), *xr(sg*(xe-5.5), sg*u0))
        blk = blk.cut(cyl_x(*xr(sg*(xe-6.0), sg*(xe+0.3)), fseat(16.0)).translate((0, 0, zc))).cut(cyl_x(*xr(sg*(xe-1.0), sg*(u0+1)), 5.0).translate((0, 0, zc)))
        for sy_ in (-1, 1): blk = blk.cut(cyl_x(*xr(sg*(u0-6.2), sg*(u0+1)), R_INS3).translate((0, sy_*17.0, zc)))
        parts[f'arm_shaft_block_{side}'] = blk.val()
        parts[f'arm_brg_motor_{side}_688'] = cyl_x(*xr(sg*(xe-5.2), sg*(xe-0.2)), 8.0, 4.0).translate((0, 0, zc)).val()
        bw2 = p['belt_w']/2; ri_b = p['idl_r']
        # tensioners: smooth idler on the BACK of the belt, outside the loop, on an M5 bolt in a slot of the bone + flange nut
        for (cy_, cz_), tag in zip(d_['idl_c'], ('front', 'back')):
            yc_, zc_ = cy_, ze + cz_
            idl = (cyl_x(sg*xc-bw2, sg*xc+bw2, ri_b).union(cyl_x(sg*xc-bw2-1.0, sg*xc-bw2, ri_b+1.5))
                   .union(cyl_x(sg*xc+bw2, sg*xc+bw2+1.0, ri_b+1.5)).cut(cyl_x(sg*xc-bw2-2, sg*xc+bw2+2, R_M5)))
            parts[f'arm_idler_{side}_{tag}_smooth'] = idl.translate((0, yc_, zc_)).val()
            parts[f'arm_spacer_idler_{side}_{tag}'] = cyl_x(*xr(sg*(xc+bw2+1.0), sg*u0), 4.5, 2.6).translate((0, yc_, zc_)).val()
            bolt = cyl_x(*xr(sg*(xc-bw2-1.0), sg*(u1+5.0)), 2.5).union(cyl_x(*xr(sg*(xc-bw2-3.75), sg*(xc-bw2-1.0)), 4.75))   # button head (ISO 7380): a socket head would touch the other motor
            parts[f'arm_tensioner_{side}_{tag}_screw_M5'] = bolt.translate((0, yc_, zc_)).val()
            parts[f'arm_tensioner_{side}_{tag}_nut_M5'] = cyl_x(*xr(sg*u1, sg*(u1+4.7)), 4.6, 2.5).translate((0, yc_, zc_)).val()   # flange nut
        parts[f'belt_elbow_{side}_GT2'] = belt_solid(sg*xc, d_['circ'], p['belt_w']).translate((0, 0, ze)).val()
    # torsion box: the common ribbed cover part on the front/back EDGES of the humerus plates (tilted to follow the
    # taper), screwed into M3 inserts in the plate edges, outside the radius of the static shoulder unit
    wz = lambda z: p['sh_r'] + (p['el_r'] - p['sh_r'])*z/ze
    Lc, tcv = p['cover']; zc0 = 48.0
    dy = wz(zc0+Lc) - wz(zc0); th_ = -math.degrees(math.asin(dy/Lc))
    cvz = (8.0, 45.0, 82.0)
    cva = cover_part(u0, u1, Lc, tcv, zs=cvz).val().rotate(O3, XV, th_).translate(cq.Vector(0, wz(zc0)+0.2, zc0))
    parts['arm_cover_front'] = cva
    parts['arm_cover_back'] = cva.mirror('XZ', (0, 0, 0))
    drl = cover_drill(u0, u1, Lc, R_M3, zs=cvz).val().rotate(O3, XV, th_).translate(cq.Vector(0, wz(zc0)+0.2, zc0))   # screw holes for the cover tabs
    for bn in ('arm_humerus_R', 'arm_humerus_L'):
        v0 = parts[bn].Volume(); parts[bn] = parts[bn].cut(drl).cut(drl.mirror('XZ', (0, 0, 0)))
        assert abs(v0 - parts[bn].Volume() - 6*math.pi*R_M3**2*p['plate_t']) < 15.0, 'cover screw hole meets another feature in ' + bn
    x_pe = u1 + 1.2                                                      # pin end, just proud of the bones
    parts['cradle_pin_steel'] = (cyl_x(-x_pe, x_pe, p['pin_d']/2).cut(cyl_x(x_pe-2.5, x_pe+1, 3.05))
                                 .translate((0, 0, ze)).val())
    parts['cradle_magnet_flex_6x2.5'] = cyl_x(x_pe-2.5, x_pe, 3.0).translate((0, 0, ze)).val()
    # flex encoder: AS5600 on a bracket screwed to the right plate, facing the magnet in the pin end
    c1, c2 = enc_pts
    xc0 = x_pe + 1.1; xc1 = xc0 + 1.2; xp1 = xc1 + 1.6; xb1 = xp1 + 3.0
    brk = prism_yz(_hull(_circ(0, 0, 11.0) + _circ(*c1, 6.0) + _circ(*c2, 6.0)), xp1, xb1).union(prism_yz(_hull(_circ(*c1, 5.0) + _circ(*c2, 5.0)), u1, xp1+0.01))
    for cc in (c1, c2):
        brk = brk.cut(cyl_x(u1-1, xb1+1, R_M3).translate((0,) + cc)).cut(cyl_x(xb1-2.0, xb1+1, 3.1).translate((0,) + cc))
    parts['arm_encoder_flex_bracket'] = brk.translate((0, 0, ze)).val()
    parts['arm_as5600_flex_pcb'] = bx(xc1, xp1, -8.0, 8.0, ze-8.0, ze+8.0).val()
    parts['arm_as5600_flex_chip'] = bx(xc0, xc1, -2.5, 2.5, ze-3.0, ze+3.0).val()

    # side bevel + pulley, ONE printed part each (like the wrist), 6801 bearings at the toe and at the pulley end
    b_od, b_id, b_w = 21.0, 12.0, 5.0
    # phase dimples on the toe face of the (identical) side part: 1 dimple = mesh mark used as gear B,
    # 2 dimples = mesh mark used as gear A. The satellite heel has the same marks toward +X (B) and -X (A).
    mk_B = cyl_x(toe-1.0, toe+1.0, 1.0).translate((0, 0, 22.0))
    mk_A = cyl_x(-toe-1.0, -toe+1.0, 1.0).translate((0, 0, 19.0)).union(cyl_x(-toe-1.0, -toe+1.0, 1.0).translate((0, 0, 25.0)))
    SBc = cq.Workplane('XY').newObject([SB]).cut(mk_B, clean=False).cut(cq.Workplane('XY').newObject([from_A(mk_A.val())]), clean=False).val()
    SA = to_A(SBc)
    shim_od, shim_t = p['pre_shim']; spr_od = p['pre_spring'][0]
    xsp = shim_t + p['pre_spring'][1] - (wall_in - pul_out)              # how deep shim and spring sit inside the pulley bore
    for nm, gear, sg in (('A', SA, -1), ('B', SBc, 1)):
        pu = cyl_x(*xr(sg*hg, sg*(pul_out-1.0)), rb_e).union(cyl_x(*xr(sg*(pul_out-1.0), sg*pul_out), rf_e))
        body = cq.Workplane('XY').newObject([gear]).union(pu, clean=False).cut(cyl_x(-90, 90, p['pin_d']/2+0.2), clean=False)
        body = body.cut(cyl_x(*xr(sg*(toe-0.2), sg*(toe+b_w+0.2)), fseat(b_od)), clean=False)
        body = body.cut(cyl_x(*xr(sg*(pul_out-xsp-b_w-0.2), sg*(pul_out+0.2)), fseat(b_od)), clean=False)
        parts[f"elbow_side_{nm}_{p['z_side']}T_{p['t_elb']}T"] = body.translate((0, 0, ze)).val()
        # axial preload, closed inside the cradle: shim + wave spring sit in the pulley-end bore, between the 6801 and the cradle wall
        parts[f'arm_elbow_preload_{nm}_shim_steel'] = cyl_x(*xr(sg*(pul_out-xsp), sg*(pul_out-xsp+shim_t)), shim_od/2, 6.1).translate((0, 0, ze)).val()
        parts[f'arm_elbow_preload_{nm}_wave_spring_steel'] = cyl_x(*xr(sg*(pul_out-xsp+shim_t), sg*wall_in), spr_od/2, 6.1).translate((0, 0, ze)).val()
        # the pin is clamped in the cradle walls and turns in a 6801 in each bone; a thin ring keeps wall and bearing apart
        parts[f'arm_brg_elbow_{nm}_6801'] = cyl_x(*xr(sg*(u0+0.5), sg*(u0+5.5)), 10.5, 6.0).translate((0, 0, ze)).val()
        parts[f'cradle_spacer_{nm}'] = cyl_x(*xr(sg*wall_out, sg*(u0+0.5)), 7.4, 6.05).translate((0, 0, ze)).val()
        parts[f'elbow_brg_{nm}_toe_6801'] = cyl_x(*xr(sg*toe, sg*(toe+b_w)), b_od/2, b_id/2).translate((0, 0, ze)).val()
        parts[f'elbow_brg_{nm}_pulley_6801'] = cyl_x(*xr(sg*(pul_out-xsp-b_w), sg*(pul_out-xsp)), b_od/2, b_id/2).translate((0, 0, ze)).val()

    # ================= CRADLE inside the bones, in THREE pieces that all print flat =================
    # plate (bearing seats printed as true circles) + two walls (pin bore printed as a true circle), 4 countersunk M3 per wall
    cy2 = p['cr_y']; er = p['ear_r']
    pz0 = max(z_out_back + 3.0, plate_r_el + 3.5, rf_e + 1.5); pz1 = pz0 + p['cr_plate_t']     # the plate clears the pulley flanges
    cr = bx(-wall_in, wall_in, -cy2, cy2, pz0, pz1)
    bo, bi, bw61 = p['b61810']
    cr = cr.cut(cyl_z(pz0-1, pz0+p['brg_w'], fseat(p['brg_od']))).cut(cyl_z(pz0-1, pz1+1, p['hub_d']/2+2.0))
    cr = cr.cut(cyl_z(pz1-bw61, pz1+1, fseat(bo)))
    cr = round_plan(cr, wall_in, cy2, 12.0, er+0.01, -er-5.0, pz1+5.0)
    wall_scr = [(sy_*(cy2-12.0), zz_) for sy_ in (-1, 1) for zz_ in (pz0+6.5, pz1-6.5)]
    # lightening: four through pockets (vertical, free to print) between the ring around the bearings and the outer frame;
    # a rib on the x axis ties the ring straight to each wall, so the bearing spacing and the load path stay the same
    rr_ = fseat(bo) + 4.0; px1_ = wall_in - 8.5; py_ = cy2 - 6.0
    for sx_ in (-1, 1):
        for sy_ in (-1, 1):
            pk_ = bx(*xr(sx_*22.0, sx_*px1_), *xr(sy_*4.0, sy_*py_), pz0-1, pz1+1).cut(cyl_z(pz0-2, pz1+2, rr_))
            try: pk_ = pk_.edges('|Z').fillet(2.5)
            except Exception: pass
            cr = cr.cut(pk_)
    for sg in (-1, 1):
        for yy_, zz_ in wall_scr: cr = cr.cut(cyl_x(*xr(sg*(wall_in-6.5), sg*(wall_in+1)), R_INS3).translate((0, yy_, zz_)))
    parts['cradle_plate'] = cr.translate((0, 0, ze)).val()
    wprof = _hull(_circ(0.0, 0.0, er) + [(cy2-10.0, pz0-6.0), (cy2-10.0, pz1), (-(cy2-10.0), pz1), (-(cy2-10.0), pz0-6.0)])
    for sg, nm in ((-1, 'L'), (1, 'R')):
        wl = prism_yz(wprof, *xr(sg*wall_in, sg*wall_out))
        wl = wl.cut(cyl_x(*xr(sg*(wall_in-1), sg*(wall_out+1)), p['pin_d']/2))                       # pin clamped in the wall
        wl = wl.cut(cyl_y(5.0, er+1.0, R_TAP3, cx=sg*(wall_in+wall_out)/2, cz=0.0))                    # M3 grub on the pin flat
        for yy_, zz_ in wall_scr:                                                                     # countersunk M3 x 12 into the plate inserts
            wl = wl.cut(cyl_x(*xr(sg*(wall_in-1), sg*(wall_out+1)), R_M3).translate((0, yy_, zz_)))
            wl = wl.cut(cq.Workplane('XY').polyline([(wall_out-1.8, 0), (wall_out-1.8, R_M3), (wall_out+0.01, 3.3), (wall_out+0.01, 0)]).close()
                        .revolve(360, (0, 0, 0), (1, 0, 0)).mirror('YZ', (0, 0, 0)).translate((0, yy_, zz_)) if sg < 0 else
                        cq.Workplane('XY').polyline([(wall_out-1.8, 0), (wall_out-1.8, R_M3), (wall_out+0.01, 3.3), (wall_out+0.01, 0)]).close()
                        .revolve(360, (0, 0, 0), (1, 0, 0)).translate((0, yy_, zz_)))
        parts[f'cradle_wall_{nm}'] = wl.translate((0, 0, ze)).val()
    parts['cradle_brg_hub_low_6806'] = cyl_z(ze+pz0, ze+pz0+p['brg_w'], p['brg_od']/2-0.05, p['brg_id']/2).val()
    parts['cradle_brg_forearm_6810'] = cyl_z(ze+pz1-bw61, ze+pz1, bo/2, bi/2).val()

    # ================= FOREARM: base plate right on the elbow (no tube), long ulna / radius of the wrist on top =================
    import wrist_v0 as W
    z_bp1 = p['Lf'] - W.P['yoke_len']                                       # top of the forearm base = bottom of ulna / radius
    z_bp0 = z_bp1 - 8.0
    assert z_bp0 > pz1 + 0.3, 'forearm base would touch the carrier'
    hub = cq.Workplane('XY').newObject([OUT]).union(cyl_z(z_out_back-0.01, z_bp0, p['hub_d']/2))
    # hex drive (25 across flats, passes through the 6806) into a pocket of the forearm base: torque by shape, 3 M3 hold it down
    hub = hub.union(cq.Workplane('XY').workplane(offset=z_bp0-0.01).polygon(6, 25.0/math.cos(math.pi/6)).extrude(5.01))
    hub = hub.cut(cyl_z(0, z_bp0-10.0, p['tube_id']/2)).cut(cyl_z(z_bp0-10.5, z_bp0+6, 7.0))
    hub = hub.cut(cq.Workplane('XZ').polyline([(0, z_bp0-10.01), (p['tube_id']/2, z_bp0-10.01), (7.0, z_bp0-7.0), (0, z_bp0-7.0)]).close().revolve(360, (0, 0, 0), (0, 1, 0)))
    for k in range(3):
        a = 2*math.pi*k/3
        hub = hub.cut(cyl_z(z_bp0-2.0, z_bp0+6, R_INS3, cx=11.0*math.cos(a), cy=11.0*math.sin(a)))
    z_ot = OUT.BoundingBox().zmin                                                       # toe of the forearm satellite
    hub = hub.cut(cyl_z(z_ot-1.0, z_ot+5.2, fseat(21.0)), clean=False)                    # 6801 at the toe: on the spider stub
    for mx, my in ((17.2, 0.0), (-17.2, 1.7), (-17.2, -1.7)):            # phase dimples on the satellite heel
        hub = hub.cut(cyl_z(z_out_back-1.0, z_out_back+1.0, 0.8, cx=mx, cy=my), clean=False)
    parts['forearm_bevel_hub'] = hub.translate((0, 0, ze)).val()
    # second (idle) 15T satellite opposite the forearm one: on a steel shaft turning in two 688 in a spider keyed on the
    # pin (the pin turns with the carrier); diametric magnet in the shaft end, AS5600 on a 16 x 18 PCB in the spider
    asm2 = pair.assemble(gear_build_args={'bore_d': p['pin_d']+0.4}, pinion_build_args={'bore_d': 8.0})   # glued + pressed on the shaft
    k2 = {c.name: c for c in asm2.children}
    IDL = to_frame(k2['pinion'].obj.moved(k2['pinion'].loc)).rotate(O3, XV, 180)        # axis -Z, toe at -31.5
    z_it = -OUT.BoundingBox().zmin; z_ih = -z_out_back                                   # idle toe / heel (negative)
    parts['idle_satellite_15T'] = IDL.translate(cq.Vector(0, 0, ze))                    # pressed on the shaft, no hub: flush
    parts['idle_shaft_steel'] = cyl_z(z_ih, -14.6, 4.0).translate((0, 0, ze)).val()     # ends flush with the heel
    parts['idle_magnet_6x2.5'] = cyl_z(-14.6, -12.1, 3.0).translate((0, 0, ze)).val()
    zs0 = z_it + 1.0                                                                      # spider end, 1 mm from the toe
    # spider: ring on the pin + housing of the idle shaft (down) + hollow stub (up) carrying a 6801 in the toe of the
    # forearm satellite, like the T fitting of the wrist: the satellite is held at both ends; cables go through the stub
    spd = cyl_x(-(toe-0.7), toe-0.7, 12.0).union(cyl_z(zs0, -6.0, 11.0))
    spd = spd.union(bx(-(toe-0.7), toe-0.7, -20.0, 20.0, zs0, 28.0).edges('|Z').fillet(5.0))   # chunky cross block
    spd = spd.union(cyl_z(7.0, z_ot-0.5, 9.0)).union(cyl_z(z_ot-0.51, z_ot+5.0, fboss(12.0)))
    spd = spd.cut(cyl_z(6.5, z_ot+6.0, 4.5)).cut(bx(-3.8, 3.8, 0.0, 21.0, 14.0, 24.0))       # cable bore + side window (+Y)
    spd = spd.cut(cyl_x(-20, 20, p['pin_d']/2)).cut(cyl_z(5.0, 13.0, R_TAP3, cx=11.0))        # pin bore, M3 grub on the flat
    spd = spd.cut(cyl_z(zs0-1, -15.0, fseat(16.0))).cut(cyl_z(-15.1, -11.2, 4.2))                   # 2x 688 + spacer, magnet room
    spd = spd.cut(bx(-8.5, 8.5, -8.6, 21.0, -11.2, -8.2))                                   # PCB pocket, open toward +Y
    parts['cradle_spider'] = spd.translate((0, 0, ze)).val()
    parts['cradle_brg_satellite_6801'] = cyl_z(z_ot, z_ot+5.0, 10.5, 6.0).translate((0, 0, ze)).val()
    parts['cradle_brg_idle_688_1'] = cyl_z(zs0, zs0+5.0, 8.0, 4.0).translate((0, 0, ze)).val()
    parts['cradle_spacer_idle'] = cyl_z(zs0+5.0, -20.0, 7.95, 6.0).translate((0, 0, ze)).val()
    parts['cradle_brg_idle_688_2'] = cyl_z(-20.0, -15.0, 8.0, 4.0).translate((0, 0, ze)).val()
    parts['cradle_as5600_pron_pcb'] = bx(-8.0, 8.0, -8.0, 10.0, ze-9.8, ze-8.2).val()
    parts['cradle_as5600_pron_chip'] = bx(-2.5, 2.5, -3.0, 3.0, ze-11.0, ze-9.8).val()
    import wrist_v0 as W
    wp, winfo = W.build(W.P)                                                 # wrist + hand (also gives the bone positions)
    wu0 = winfo['yoke_x'][0]
    base = bx(-wu0, wu0, -25.4, 25.4, z_bp0, z_bp1).union(cyl_z(z_bp0, z_bp1, 40.0))   # ulna / radius flank its ends
    for sx_ in (-1, 1):
        for yy in (-19.0, -9.0, 9.0, 19.0):                                  # M3 heat-set inserts, screws through the bones
            base = base.cut(cyl_x(*xr(sx_*(wu0-7.0), sx_*(wu0+1)), R_INS3).translate((0, yy, z_bp0+4.0)))
    base = base.union(cyl_z(pz1-bw61, z_bp0+0.01, bi/2, bi/2-3.5))                 # spigot in the 61810 inner ring
    base = base.union(cyl_z(z_bp0, z_bp1, 21.0))                                  # solid centre: it carries the hex pocket
    base = base.cut(cyl_z(pz1-bw61-1, z_bp0+0.01, p['hub_d']/2+0.2)).cut(cyl_z(z_bp0-1, z_bp1+1, 7.0))      # hub passage, cable hole
    base = base.cut(cq.Workplane('XY').workplane(offset=z_bp0-0.01).polygon(6, 25.3/math.cos(math.pi/6)).extrude(5.15))
    for k in range(3):
        a = 2*math.pi*k/3
        base = base.cut(cyl_z(z_bp0-1, z_bp1+1, R_M3, cx=11.0*math.cos(a), cy=11.0*math.sin(a)))   # 3x M3 into the inserts of the hub
    parts['forearm_base_plate'] = base.translate((0, 0, ze)).val()

    # ================= SHOULDER: NEMA23 with an integrated steel planetary gearbox (20:1), coaxial with the joint =================
    # (23HS22-2804S-HG20 class: 57 mm square motor 56 long + gearbox Ø60 x 61.65, Ø14 x 34 keyed shaft. MEASURE the unit you
    #  buy and set the n23_* / gb_* parameters before printing the pedestal.)
    z_tt = p['z_tt']
    q23 = p['n23_sq']; gbr = p['gb_d']/2; gbl = p['gb_len']
    xg = u0 - p['gb_pilot'][1] - 0.5                                       # gearbox front face, its pilot just clear of the right bone
    xgm = xg - gbl; xm0 = xgm - p['n23_len']                              # gearbox / motor joint, motor back
    xw1 = xm0 - 0.5; xw0 = -(u0 - 1.5)                                    # left wall (carries the left hub)
    assert xw1 - xw0 >= 5.0, 'left shoulder wall thinner than 5 mm: widen the arm (pulley_gap)'
    parts['yaw_motor_shoulder_NEMA23'] = bx(xm0, xgm, -q23/2, q23/2, -q23/2, q23/2).val()
    parts['yaw_shoulder_gearbox_planetary_20'] = (cyl_x(xgm, xg, gbr).union(bx(xgm, xgm+6.0, -q23/2-1.5, q23/2+1.5, -q23/2-1.5, q23/2+1.5))
                                                  .union(cyl_x(xg-0.01, xg+p['gb_pilot'][1], p['gb_pilot'][0]/2))).val()
    sr = p['gb_shaft'][0]/2; xse = xg + p['gb_shaft'][1]
    parts['arm_shoulder_shaft_steel_14'] = cyl_x(xg+p['gb_pilot'][1], xse, sr).val()
    # the gearbox shaft carries the right bone through a keyed steel flange hub on the OUTER face of the bone (4x M5 into inserts)
    fo_, ft_ = p['sh_hub_fl']
    hubR = cyl_x(u1, u1+ft_, fo_/2).union(cyl_x(u1+ft_-0.01, xse, p['sh_hub_neck']/2)).cut(cyl_x(u1-1, xse+1, sr))
    for k in range(4):
        a = math.pi/4 + k*math.pi/2
        hubR = hubR.cut(cyl_x(u1-1, u1+ft_+1, R_M5).translate((0, p['sh_hub_pcd']/2*math.cos(a), p['sh_hub_pcd']/2*math.sin(a))))
    parts['arm_shoulder_flange_hub_steel'] = hubR.val()
    # left side: fixed hollow hub on a wall of the pedestal, the left bone turns on it in a 6810
    xs0 = xw0
    hubL = cyl_x(-(u1+4.0), xw0, 25.0, 15.0).union(cyl_x(xw0-0.01, xw0+1.5, 18.0, 15.0))         # 1.5 mm register in the wall
    parts['arm_brg_shoulder_L_6810'] = cyl_x(-u1, -(u1-7.0), 32.45, 25.0).val()
    # pedestal: one block from the turntable, a square channel for the motor, a round one for the gearbox, the left wall on top
    pw = 44.0
    xp1 = xg - 1.0
    ped = bx(xw0, xp1, -pw, pw, z_tt, 0.0).union(cyl_x(xw0, xw1, 34.0)).union(bx(xw0, xw1, -34.0, 34.0, z_tt, 0.0))
    for sy in (-1, 1):                                                     # low gussets (the arm covers pass above at shoulder 90 deg)
        for xg_ in (xw0+4.0, xm0+12.0, xgm+8.0, xp1-4.0):
            if sy < 0: continue                                            # the balancer spring lies along the back of the pedestal
            ped = ped.union(prism_yz([(sy*pw, z_tt), (sy*(pw+16.0), z_tt), (sy*pw, z_tt+16.0)], xg_-3.0, xg_+3.0))
    ped = ped.cut(bx(xw1, xgm+6.3, -q23/2-1.8, q23/2+1.8, -q23/2-1.8, 1.0))                       # motor + gearbox flange: keyed by the square
    ped = ped.cut(cyl_x(xgm, xp1+1, gbr+0.3)).cut(bx(xgm, xp1+1, -gbr-0.3, gbr+0.3, 0.0, 1.0))
    ped = ped.cut(cyl_x(xw0-1, xw1+1, 18.15)).cut(bx(xw1-0.01, xw1+3.0, -8.0, 8.0, -q23/2-1.8, 1.0))   # hub register; rear cable notch
    for k in range(4):                                                    # 4 countersunk M3 x 12 from inside the wall into the hub
        a = math.pi/4 + k*math.pi/2; hy, hz = 21.5*math.cos(a), 21.5*math.sin(a)
        hubL = hubL.cut(cyl_x(xw0-10.0, xw0+1, R_TAP3).translate((0, hy, hz)))
        ped = ped.cut(cyl_x(xw0-1, xw1+1, R_M3).translate((0, hy, hz)))
        ped = ped.cut(cq.Workplane('XY').polyline([(xw1-1.7, 0), (xw1-1.7, R_M3), (xw1+0.01, 3.2), (xw1+0.01, 0)]).close().revolve(360, (0, 0, 0), (1, 0, 0)).translate((0, hy, hz)))
    parts['yaw_shoulder_hub_L'] = hubL.val()
    # two straps hold the unit down: 2x M5 each into inserts in the pedestal top
    strap_x = ((xm0+14.0, xm0+30.0), (xgm+18.0, xgm+34.0))
    for k, (xa, xb) in enumerate(strap_x, start=1):
        if k == 1:                                                         # low U: its top corners stay inside R48 (the elbow motor carrier passes above)
            st = bx(xa, xb, -pw, pw, 0.0, 7.0).union(bx(xa, xb, -q23/2-4.0, q23/2+4.0, 0.0, q23/2+7.0)).cut(bx(xa-1, xb+1, -q23/2-0.3, q23/2+0.3, -1.0, q23/2+0.3))
        else: st = bx(xa, xb, -pw, pw, 0.0, 7.0).union(cyl_x(xa, xb, gbr+6.5).cut(bx(xa-1, xb+1, -pw-1, pw+1, -gbr-8, 0.0))).cut(cyl_x(xa-1, xb+1, gbr+0.3))
        for sy in (-1, 1):
            cx_ = (xa+xb)/2; cy_ = sy*(pw-7.0)
            st = st.cut(cyl_z(-1.0, 60.0, R_M5, cx=cx_, cy=cy_)).cut(cyl_z(7.0, 60.0, 5.0, cx=cx_, cy=cy_))
            ped = ped.cut(cyl_z(-10.5, 1.0, R_INS5, cx=cx_, cy=cy_))
        parts[f'yaw_shoulder_strap_{k}'] = st.val()
    ped = ped.cut(bx(0.0, xp1+1.0, -10.0, 10.0, z_tt-1.0, z_tt+12.0))                             # cable tunnel from the spindle bore to the right end (the balancer is on the left and at the back)
    ped_pts = tuple((px_, sy*32.0) for px_ in (xw0+16.0, -18.0, 18.0, xp1-10.0) for sy in (-1, 1))
    for px_, py_ in ped_pts: ped = ped.cut(cyl_z(z_tt-1, z_tt+10.5, R_INS5, cx=px_, cy=py_))    # 6x M5 from under the turntable
    hb_pts = ()
    parts['yaw_shoulder_pedestal'] = ped.val()

    # ================= BASE: turntable, yaw drive, foot with electronics =================
    tt = p['tt_t']; zg1 = z_tt - tt; zg0 = zg1 - p['yaw_w']; z_f0 = zg0 - p['foot_h']
    yr_pd = p['yaw_m']*p['yaw_ring']; yp_pd = p['yaw_m']*p['yaw_pin']; a_yaw = (yr_pd - yp_pd)/2
    # yaw bearing: the turntable carries a hollow Ø50 spindle that turns in two 6810 bearings, 25 mm apart, held in one
    # printed cartridge (coaxial seats) that drops into the foot deck. Cables go up through the spindle.
    T = zg1; r_sp, r_ct, r_bs = 25.0, 38.5, fseat(65.0)
    # ---- gravity balancer of the shoulder (acts like a zero-free-length spring: torque = k * a * b * sin(shoulder angle)) ----
    # Up to 220 N in the cable, all the time the arm is forward: thick sections, wide feet, big fillets, 3 bolts per part.
    B_ = p['bal']; po_, pb_, pw_ = B_['pulley']; xs_b = -(u1 + 10.5)          # cable plane: the fork stays clear of the left bone
    parts['arm_balancer_pin_steel_M5'] = cyl_x(xs_b-3.5, -u1, 2.5).union(cyl_x(xs_b-6.5, xs_b-3.5, 4.25)).translate((0, 0, -B_['b'])).val()
    p1 = (xs_b, 0.0, -B_['a'])                                                 # pulley 1, axis along X: the "anchor" under the shoulder axis
    rg_ = po_/2 - 1.5                                                          # groove radius
    zc_b = p1[2] - po_/2 + 1.0                                                 # cable height after pulley 1 (groove bottom)
    p2 = (xs_b + po_/2 - 0.3, B_['y'] + po_/2 - 0.3, zc_b + 2.5)               # pulley 2, axis along Z: turns the cable along the pedestal
    parts['yaw_brg_balancer_1_U624ZZ'] = cyl_x(p1[0]-pw_/2, p1[0]+pw_/2, rg_, pb_/2).translate((0, p1[1], p1[2])).val()
    parts['yaw_brg_balancer_2_U624ZZ'] = cyl_z(p2[2]-pw_/2, p2[2]+pw_/2, rg_, pb_/2, cx=p2[0], cy=p2[1]).val()
    ck = 6.0; hw = pw_/2 + 0.4 + ck; zb1 = z_tt + 6.0                          # cheek thickness, fork half width, top of the base plate
    def filletY(x0, x1, y0, sgn, r):                                           # concave fillet between the base plate and a wall across Y
        return (bx(x0, x1, *sorted((y0, y0+sgn*r)), zb1-0.01, zb1+r)
                .cut(cq.Workplane('YZ').workplane(offset=x0-1).center(y0+sgn*r, zb1+r).circle(r).extrude(x1-x0+2)))
    # pulley 1: a fork with two 6 mm cheeks rounded about the axle, R8 fillets down to a 6 mm base
    fork = prism_yz(_hull(_circ(0.0, p1[2], 8.5) + [(-11.0, z_tt), (11.0, z_tt)]), xs_b-hw, xs_b+hw)
    fork = fork.cut(bx(xs_b-pw_/2-0.4, xs_b+pw_/2+0.4, -20.0, 20.0, p1[2]-po_/2-1.5, p1[2]+20.0))
    foot = prism_xy_r(xs_b-hw-17.0, xs_b+hw, -30.0, 30.0, z_tt, zb1, 8.0)
    bb = foot.union(fork).union(filletY(xs_b-hw, xs_b-pw_/2-0.4, 11.0, 1, 8.0)).union(filletY(xs_b-hw, xs_b-pw_/2-0.4, -11.0, -1, 8.0))
    bb = bb.union(filletY(xs_b+pw_/2+0.4, xs_b+hw, 11.0, 1, 8.0)).union(filletY(xs_b+pw_/2+0.4, xs_b+hw, -11.0, -1, 8.0))
    # pulley 2: held from below AND from above by a C-shaped housing (pillar on the corner the cable never reaches)
    h2 = prism_xy_r(p2[0]-10.0, p2[0]+11.0, p2[1]-10.0, p2[1]+11.0, z_tt, p2[2]+pw_/2+6.5, 5.0)
    keep = bx(p2[0]-3.5, p2[0]+13.0, p2[1]-3.5, p2[1]+13.0, z_tt-1, p2[2]+20.0).cut(cyl_z(z_tt-2, p2[2]+21.0, po_/2+0.9, cx=p2[0], cy=p2[1]))
    slot2 = bx(p2[0]-11.0, p2[0]+13.0, p2[1]-11.0, p2[1]+13.0, p2[2]-pw_/2-0.4, p2[2]+pw_/2+0.4).cut(keep)     # slot for pulley and cable; an L wall is left on the far corner
    foot2 = prism_xy_r(xs_b-hw, p2[0]+11.0, p2[1]-10.0, -29.0, z_tt, zb1, 8.0).union(prism_xy_r(xs_b-hw-9.5, xs_b-hw+4.0, -52.0, -29.0, z_tt, zb1, 6.0))
    bb = bb.union(foot2).union(h2)
    bal_pts = ((xs_b-hw-8.5, 20.0), (xs_b-hw-8.5, -20.0), (xs_b-hw-2.0, -43.0))
    for bx_, by_ in bal_pts: bb = bb.union(cyl_z(z_tt, z_tt+8.0, 7.5, cx=bx_, cy=by_))
    for bx_, by_ in bal_pts: bb = bb.cut(cyl_z(z_tt-1, z_tt+7.6, R_INS5, cx=bx_, cy=by_))
    bb = bb.cut(slot2).cut(bx(xs_b-pw_/2-0.4, xs_b+pw_/2+0.4, -12.0, 12.0, p1[2]-po_/2-1.5, p1[2]+20.0))
    bb = bb.cut(cyl_x(xs_b-hw-1, xs_b+hw+1, 2.1).translate((0, 0, p1[2])))                          # M4 axle of pulley 1, nut outside
    bb = bb.cut(cyl_z(z_tt+2.0, p2[2]+pw_/2+8.0, 2.1, cx=p2[0], cy=p2[1])).cut(cyl_z(z_tt-1, z_tt+4.2, 4.1, cx=p2[0], cy=p2[1]))   # M4 axle of pulley 2, nut sunk underneath
    bb = bb.cut(bx(xs_b-1.7, xs_b+1.7, p2[1]+po_/2+1.0, -11.0, zc_b-1.8, zc_b+12.0))                # cable channel from pulley 1 to pulley 2
    parts['yaw_balancer_bracket'] = bb.val()
    parts['yaw_balancer_axle_1_steel_M4'] = cyl_x(xs_b-hw-3.0, xs_b+hw+3.5, 2.0).translate((0, 0, p1[2])).val()
    parts['yaw_balancer_axle_2_steel_M4'] = cyl_z(z_tt+2.2, p2[2]+pw_/2+9.0, 2.0, cx=p2[0], cy=p2[1]).val()
    so_, sd_, sb_ = B_['spring']; xa_ = B_['x_anchor']
    x_hook = xa_ - (sb_ + 2*(so_-sd_) + 7.0)                                   # moving hook, arm upright
    parts['yaw_balancer_spring_steel'] = cyl_x(xa_-so_-sb_, xa_-so_, so_/2, so_/2-sd_).translate((0, B_['y'], B_['z'])).val()
    parts['yaw_balancer_hook_a_steel'] = cyl_x(x_hook, xa_-so_-sb_, sd_/2).translate((0, B_['y'], B_['z'])).val()
    parts['yaw_balancer_hook_b_steel'] = cyl_x(xa_-so_, xa_-1.0, sd_/2).translate((0, B_['y'], B_['z'])).val()
    rc_ = rg_ + 1.2; q_ = rc_*0.7071
    parts['yaw_balancer_cable'] = cable_poly([(p1[0], -2.0, p1[2]-rc_), (p2[0]-rc_, p2[1]+2.0, p2[2]), (p2[0]-rc_, p2[1], p2[2]), (p2[0]-q_, p2[1]-q_, p2[2]),
                                         (p2[0], p2[1]-rc_, p2[2]), (p2[0]+6.0, p2[1]-rc_, p2[2]), (x_hook-0.5, B_['y'], B_['z'])], 1.0)
    # spring anchor: a 12 mm wall across the spring, on a long foot along the rim with three bolts; R8 fillet on the loaded side
    aw = 12.0; ya0, ya1 = B_['y']-10.0, -8.0
    anc = prism_xy_r(xa_, xa_+aw, ya0, B_['y']+12.0, z_tt, B_['z']+11.0, 4.0).union(prism_xy_r(xa_-10.0, xa_+aw, ya0, ya1, z_tt, zb1+2.0, 5.0))
    anc = anc.union(bx(xa_-8.0, xa_+0.01, ya0+4.0, B_['y']+12.0, zb1+1.99, zb1+10.0)
                    .cut(cq.Workplane('XZ').workplane(offset=-(B_['y']+13.0)).center(xa_-8.0, zb1+10.0).circle(8.0).extrude(B_['y']+13.0-ya0+1)))
    anc = anc.cut(bx(xa_-11.0, xa_+0.02, B_['y']-so_/2-1.0, B_['y']+so_/2+1.0, z_tt+1.0, B_['z']+30.0))    # the spring end sits in a notch of the foot
    anc = anc.intersect(cyl_z(z_tt-1, 0.0, p['tt_r']-0.5))                                           # follows the rim of the turntable
    anc = anc.cut(cyl_x(xa_-1.0, xa_+aw+1.0, R_M5).translate((0, B_['y'], B_['z'])))                  # eye bolt + 2 nuts: sets the preload
    anc_pts = ((xa_+aw/2, B_['y']+22.0), (xa_+aw/2, B_['y']+37.0), (xa_+aw/2, ya1-7.0))
    for bx_, by_ in anc_pts: anc = anc.cut(cyl_z(z_tt-1, z_tt+7.6, R_INS5, cx=bx_, cy=by_))
    parts['yaw_balancer_anchor'] = anc.val()
    parts['yaw_balancer_eyebolt_steel_M5'] = cyl_x(xa_-0.9, xa_+aw+8.0, 2.5).translate((0, B_['y'], B_['z'])).val()
    tt_ = (cyl_z(zg1, z_tt, p['tt_r']).union(cyl_z(T-4.5, T+0.01, 27.5)).union(cyl_z(T-36.2, T-4.4, r_sp+FIT['shaft']/2))
           .cut(cyl_z(T-41, z_tt+1, 19.0)))
    for k in range(4):
        a = math.pi/4 + k*math.pi/2
        tt_ = tt_.cut(cyl_z(T-37, T-26, R_TAP3, cx=22.0*math.cos(a), cy=22.0*math.sin(a)))       # clamp ring screws
    for px_, py_ in ped_pts + hb_pts + bal_pts + anc_pts:
        tt_ = tt_.cut(cyl_z(zg1-1, z_tt+1, R_M5, cx=px_, cy=py_)).cut(cyl_z(zg1-1, zg1+5.0, 4.7, cx=px_, cy=py_))   # M5, head sunk
    ring = RingGear(module=p['yaw_m'], teeth_number=p['yaw_ring'], width=p['yaw_w'], rim_width=8.0).build()
    r_rg = ring.BoundingBox().xmax - 4.0
    rg = cq.Workplane('XY').newObject([ring.translate(cq.Vector(0, 0, zg0))])
    for k in range(6):
        a = math.radians(15 + 60*k); cx_, cy_ = r_rg*math.cos(a), r_rg*math.sin(a)
        rg = rg.cut(cyl_z(zg1-9.0, zg1+1, R_TAP3, cx=cx_, cy=cy_))                                 # 6x M3 from the top of the turntable
        tt_ = tt_.cut(cyl_z(zg1-1, z_tt+1, R_M3, cx=cx_, cy=cy_)).cut(cyl_z(z_tt-3.5, z_tt+1, 3.2, cx=cx_, cy=cy_))
    parts['yaw_turntable'] = tt_.val()
    parts['yaw_ring_gear_100T'] = rg.val()
    ctg = cyl_z(T-36.5, T-5.5, r_ct, r_bs).union(cyl_z(T-5.5, T-2.5, 49.5, r_bs)).union(cyl_z(T-4.5, T-2.5, r_bs+0.1, 30.0))
    ret = cyl_z(T-39.5, T-36.5, r_ct, 30.0); clamp = cyl_z(T-40.0, T-36.5, 27.4, 19.0)
    for k in range(4):
        a = math.pi/4 + k*math.pi/2; ca, sa = math.cos(a), math.sin(a)
        ctg = ctg.cut(cyl_z(T-6.5, T-1.5, R_M3, cx=45.0*ca, cy=45.0*sa)).cut(cyl_z(T-37.5, T-28.5, R_TAP3, cx=35.5*ca, cy=35.5*sa))
        ret = ret.cut(cyl_z(T-40.5, T-35.5, R_M3, cx=35.5*ca, cy=35.5*sa))
        clamp = clamp.cut(cyl_z(T-41, T-35.5, R_M3, cx=22.0*ca, cy=22.0*sa))
    parts['foot_yaw_cartridge'] = ctg.val()
    parts['foot_yaw_retainer'] = ret.val()
    parts['yaw_spindle_clamp_ring'] = clamp.val()
    parts['foot_brg_yaw_upper_6810'] = cyl_z(T-11.5, T-4.5, 32.5, 25.0).val()
    parts['foot_brg_yaw_lower_6810'] = cyl_z(T-36.5, T-29.5, 32.5, 25.0).val()
    parts['foot_yaw_spacer_outer'] = cyl_z(T-29.5, T-11.5, 32.3, 30.4).val()
    parts['yaw_spacer_inner'] = cyl_z(T-29.5, T-11.5, 27.2, 25.2).val()
    rmag = 110.0
    parts['yaw_home_magnet'] = cyl_z(zg1-3.0, zg1, 3.0, cx=rmag*math.cos(math.radians(-60)), cy=rmag*math.sin(math.radians(-60))).val()
    parts['foot_home_hall_sensor'] = bx(rmag-4.0, rmag+3.0, -4.0, 4.0, zg0+4.0, zg0+8.0).rotate(O3, ZV, -60.0).val()   # on a post of the deck
    pin = SpurGear(module=p['yaw_m'], teeth_number=p['yaw_pin'], width=p['yaw_w'], bore_d=p['yaw_shaft'][0]).build()
    parts['foot_yaw_pinion_15T'] = pin.rotate(O3, ZV, p['yaw_pin_phase']).translate(cq.Vector(0, -a_yaw, zg0))
    fr = p['foot_r']; fi = fr - 4.0; zm = (z_f0 + zg0)/2
    # foot: floor + wall in one piece, open at the top (prints upright with no supports); 7 pilasters carry the deck
    foot = cyl_z(z_f0, zg1-1.5, fr, fi).union(cyl_z(z_f0, z_f0+4.0, fr))
    deck_az = (22.5, 67.5, 147.0, 212.0, 240.0, 300.0, 337.5)
    for az in deck_az:
        ca, sa = math.cos(math.radians(az)), math.sin(math.radians(az))
        foot = foot.union(cyl_z(z_f0+3.9, zg0-10.0, 7.0, cx=fi*ca, cy=fi*sa).intersect(cyl_z(z_f0, zg0, fi+0.5)))
        foot = foot.cut(cyl_z(zg0-18.0, zg0-9.0, R_INS3, cx=(fi-3.5)*ca, cy=(fi-3.5)*sa))
    foot = foot.cut(cyl_y(fi-2, fr+2, 11.2, 0.0, zm))                                          # e-stop, front wall
    foot = foot.cut(cyl_x(-fr-2, -fi+2, 6.0).translate((0, -20.0, z_f0+20.0)))                   # DC jack (left)
    foot = foot.cut(bx(-fr-2, -fi+2, -5.0, 5.0, z_f0+38.0, z_f0+42.0))                           # USB-C
    foot = foot.cut(bx(-fr-2, -fi+2, 14.0, 34.0, z_f0+14.0, z_f0+27.0))                          # rocker switch
    foot = foot.cut(cyl_x(fi-2, fr+2, p['fan']/2-3.0).translate((0, 0.0, zm)))                   # fan (right)
    rf_ = (p['fan']/2-3.0)*0.7071
    foot = foot.cut(prism_yz([(-rf_, zm+rf_), (0.0, zm+2*rf_), (rf_, zm+rf_)], fi-2, fr+2))       # 45 deg roof: the hole prints without support
    for k in range(6):
        foot = foot.cut(bx(-40.0+k*14.0, -32.0+k*14.0, -fr-2, -fi+2, z_f0+12.0, zg0-12.0))       # rear vents
    foot = foot.cut(cyl_z(z_f0-1, z_f0+5, 18.0))
    lug_az = (45.0, 135.0, 225.0, 315.0)                                                          # clear of fan, e-stop, panel, vents
    for az in lug_az:                                                                             # M4 into the desk lugs
        for yy in (-22.0, 22.0):
            lx, ly = 90.0, yy
            ca, sa = math.cos(math.radians(az)), math.sin(math.radians(az))
            foot = foot.cut(cyl_z(z_f0-1, z_f0+5, R_M5, cx=lx*ca-ly*sa, cy=lx*sa+ly*ca))
    # retro 4-digit 7-segment display (0.36", TM1637 module) in the front-left wall, upper band above the desk lug
    dax = math.radians(120.0); dn = (math.cos(dax), math.sin(dax), 0.0); dt = (-math.sin(dax), math.cos(dax), 0.0); dzc = zg0 - 15.0
    def dloc(u0_, u1_, v0_, v1_, w0_, w1_):
        pts_ = [(u, v) for u, v in ((u0_, v0_), (u1_, v0_), (u1_, v1_), (u0_, v1_))]
        q = [cq.Vector(dn[0]*(fr+w0_) + dt[0]*u, dn[1]*(fr+w0_) + dt[1]*u, dzc + v) for u, v in pts_]
        face = cq.Face.makeFromWires(cq.Wire.makePolygon(q + [q[0]]))
        return cq.Solid.extrudeLinear(face, cq.Vector(dn[0]*(w1_-w0_), dn[1]*(w1_-w0_), 0))
    foot = foot.cut(cq.Workplane('XY').newObject([dloc(-15.5, 15.5, -7.5, 7.5, -6.0, 3.0)]))
    parts['foot_housing'] = foot.val()
    parts['elec_display_7seg_pcb'] = dloc(-21.0, 21.0, -10.0, 10.0, -7.6, -6.0)
    parts['elec_display_7seg_body'] = dloc(-15.0, 15.0, -7.0, 7.0, -6.0, -0.6)
    SEG = dict(a=(-2.0, 2.0, 4.0, 5.0), d=(-2.0, 2.0, -5.0, -4.0), g=(-2.0, 2.0, -0.5, 0.5),
               f=(-2.8, -1.8, 0.4, 4.2), b=(1.8, 2.8, 0.4, 4.2), e=(-2.8, -1.8, -4.2, -0.4), c=(1.8, 2.8, -4.2, -0.4))
    for d_ in range(4):
        uc = (d_ - 1.5)*7.0
        for sn, (a0, a1, b0, b1) in SEG.items():
            parts[f'elec_seg_{d_}_{sn}'] = dloc(uc+a0, uc+a1, b0, b1, -0.61, -0.3)
    # desk lugs (separate parts, 4x): wide pad under the foot rim, concave fillets in plan, curved web against the
    # foot wall with a centre rib; M6 (or desk clamp) at the end, 2x M4 into the foot floor
    lt, Rf = 8.0, 14.0
    for i, az in enumerate(lug_az, start=1):
        lug = bx(70.0, fr+6.0, -34.0, 34.0, z_f0-lt, z_f0).union(bx(fr, 162.0, -20.0, 20.0, z_f0-lt, z_f0)).union(cyl_z(z_f0-lt, z_f0, 20.0, cx=162.0))
        for sy in (-1, 1):
            fil = bx(fr+6.0-0.01, fr+6.0+Rf, sy*20.0, sy*(20.0+Rf), z_f0-lt, z_f0).cut(cyl_z(z_f0-lt-1, z_f0+1, Rf, cx=fr+6.0+Rf, cy=sy*(20.0+Rf)))
            lug = lug.union(fil)
        web = cyl_z(z_f0-0.01, z_f0+36.0, fr+6.0, fr+0.3).intersect(bx(fr-2, fr+8, -20.0, 20.0, z_f0-1, z_f0+40))
        rib = prism_xz([(fr+5.0, z_f0-0.01), (fr+5.0, z_f0+34.0), (fr+30.0, z_f0-0.01)], -3.0, 3.0)
        lug = lug.union(web).union(rib)
        lug = lug.cut(cyl_z(z_f0-lt-1, z_f0+1, 3.3, cx=162.0)).cut(cyl_z(z_f0-3.0, z_f0+1, 5.6, cx=162.0))   # M6 + counterbore
        for yy in (-22.0, 22.0):
            lug = lug.cut(cyl_z(z_f0-lt-1, z_f0+1, R_M5, cx=90.0, cy=yy)).cut(cyl_z(z_f0-lt-1, z_f0-lt+4.0, 4.7, cx=90.0, cy=yy))   # M5 + nut trap
        parts[f'foot_desk_lug_{i}'] = lug.rotate(O3, ZV, az).val()
    # deck: structural top of the foot (carries the yaw cartridge and the yaw motor), screwed on the 7 pilasters
    deck = cyl_z(zg0-10.0, zg0-4.0, fi-0.4).union(cyl_z(zg0-4.01, T-5.5, 49.5)).cut(cyl_z(zg0-11, T, r_ct+0.2))
    deck = deck.cut(cyl_z(zg0-11, zg0+13, 11.5, cy=-a_yaw).union(bx(-11.5, 11.5, -a_yaw-1.0, -a_yaw+1.0, zg0-11, zg0+13)))
    for sx_ in (-15.5, 15.5):
        for sy_ in (-15.5, 15.5):                                           # gearbox face, 4x M3 in slots: push the pinion into the ring, then tighten
            deck = deck.cut(prism_xy_slot(sx_, -a_yaw+sy_, R_M3, 1.0, zg0-11, zg0-3))
    rr = p['roller_r']
    for k, az in enumerate(p['roller_az'], start=1):                       # rim rollers: 625 on a radial M5, in two slotted posts of the deck
        posts = bx(rr-8.5, rr-3.5, -7.0, 7.0, zg0-4.01, zg1-3.0).union(bx(rr+3.5, rr+7.5, -7.0, 7.0, zg0-4.01, zg1-3.0))
        for x0_, x1_ in ((rr-9.5, rr-3.5), (rr+3.5, rr+7.5)):                 # thicker inner post, buttresses on both sides of each post
            posts = posts.union(bx(x0_, x1_, -7.0, 7.0, zg0-4.01, zg1-3.0))
            for sy_ in (-1, 1):
                posts = posts.union(prism_yz([(sy_*6.9, zg0-4.01), (sy_*16.0, zg0-4.01), (sy_*6.9, zg1-5.0)], x0_, x1_))
        posts = posts.cut(prism_yz(_hull(_circ(0.0, zg1-8.7, R_M5) + _circ(0.0, zg1-7.3, R_M5)), rr-9.5, rr+9.5))    # vertical slots: set the roller height
        deck = deck.union(posts.rotate(O3, ZV, az)).cut(bx(rr-3.2, rr+3.2, -8.5, 8.5, zg0-5.2, zg0-3.0).rotate(O3, ZV, az))
        parts[f'foot_brg_roller_{k}_625'] = cyl_x(rr-2.5, rr+2.5, 8.0, 2.5).translate((0, 0, zg1-8.0)).rotate(O3, ZV, az).val()
        parts[f'foot_roller_axle_{k}_steel_M5x25'] = cyl_x(rr-9.0, rr+11.0, 2.5).translate((0, 0, zg1-8.0)).rotate(O3, ZV, az).val()
    for az in deck_az:
        ca, sa = math.cos(math.radians(az)), math.sin(math.radians(az))
        deck = deck.cut(cyl_z(zg0-11, zg0-3, R_M3, cx=(fi-3.5)*ca, cy=(fi-3.5)*sa))
    for k in range(4):
        a = math.pi/4 + k*math.pi/2
        deck = deck.cut(cyl_z(T-13.0, T-5.0, R_INS3, cx=45.0*math.cos(a), cy=45.0*math.sin(a)))  # cartridge flange, 4x M3
    deck = deck.union(bx(rmag-4.0, rmag+3.0, -4.0, 4.0, zg0-4.01, zg0+4.0).rotate(O3, ZV, -60.0))  # Hall sensor post
    deck = deck.cut(cyl_z(zg0-11, zg0-3, 9.0, cx=-70.0, cy=40.0))
    deck = deck.cut(cq.Workplane('XY').newObject([dloc(-23.0, 23.0, -12.0, 12.0, -9.5, -1.0)]))   # notch for the display module                                 # cable pass to the drivers
    parts['foot_deck'] = deck.val()
    q17 = p['n17_sq']
    gq, gl_, _ = p['yaw_gb']
    parts['foot_yaw_gearbox_planetary_5'] = bx(-gq/2, gq/2, -a_yaw-gq/2, -a_yaw+gq/2, zg0-10.0-gl_, zg0-10.0).val()
    parts['foot_yaw_motor_NEMA17'] = bx(-q17/2, q17/2, -a_yaw-q17/2, -a_yaw+q17/2, zg0-10.0-gl_-p['yaw_motor_len'], zg0-10.0-gl_).val()
    parts['foot_yaw_motor_shaft'] = cyl_z(zg0-10.0, min(zg1-0.5, zg0-10.0+p['yaw_shaft'][1]), p['yaw_shaft'][0]/2, cy=-a_yaw).val()
    cw, cd, chh = p['ctrl']; zb = z_f0 + 6.0
    parts['elec_controller_esp32s3_6xTMC2209'] = bx(-cw/2, cw/2, 20.0, 20.0+cd, zb, zb+chh).val()
    bw_, bd_, bh_ = p['buck']
    parts['elec_buck_24v_5v'] = bx(-100.0, -100.0+bw_, -40.0, -40.0+bd_, zb, zb+bh_).val()
    fn = p['fan']
    parts['elec_fan_40mm'] = bx(fi-17.0, fi-4.0, -fn/2, fn/2, zm-fn/2, zm+fn/2).val()
    parts['elec_estop_button'] = cyl_y(fr, fr+14.0, p['estop_d']/2, 0.0, zm).union(cyl_y(fr-18.0, fr, 11.0, 0.0, zm)).val()

    # ================= CABLES (rigid in the viewer; they cross every joint ON its axis or inside the mechanism) =========
    # cables on the RIGHT (the balancer is on the left): out of the pedestal tunnel under the gearbox shaft, onto the shoulder
    # axis beyond the flange hub, up the outer face of the right bone, round its back edge above the cover, into the elbow.
    hb = xse + 6.0
    parts['yaw_cable_base'] = cable_poly([(0.0, 0.0, z_tt-9.0), (8.0, 0.0, z_tt+6.0), (xp1+6.0, 0.0, z_tt+6.0), (hb, 0.0, -34.0), (hb, 0.0, 0.0)], 3.5)
    xo = u1 + 4.5
    parts['arm_cable_main'] = cable([(hb, 0.0, 0.0), (hb, 0.0, 12.0), (hb-4.0, 0.0, 30.0), (xo+1.5, 0.0, 46.0), (xo, 0.0, 62.0), (xo, -6.0, 108.0), (xo, -26.0, 132.0),
                                     (xo-2.0, -41.0, 146.0), (u0-6.0, -42.0, 150.0), (50.0, -40.0, 153.0), (30.0, -33.0, 158.0), (10.0, -27.0, 161.0)], 3.5)

    # ================= WRIST + HAND (existing design, ghost only) =================
    for n, s in wp.items():                     # complete wrist + hand, kept in wrist coordinates (apex at the origin)
        parts['W_' + n] = s

    info = dict(drive={k_: dict(C=v_['C'], z=v_['z'], dL=v_['dL']) for k_, v_ in DRV.items()}, dL_tension=dL, idl_c=idl_c, a_off=a_off, z_bp1=z_bp1, ze=ze, Lu=Lu, z_m=z_m, C=C, hg=hg, Rh=Rh, xc=xc, u0=u0, u1=u1, wall_out=wall_out,
                ratio_belt=p['t_elb']/p['t_motor'], kp=p['z_out']/p['z_side'], ratio_cyc=p['gb_ratio'],
                ratio_yaw=p['yaw_ring']/p['yaw_pin'], ratio_yaw_motor=p['yaw_ring']/p['yaw_pin']*p['yaw_gb'][2], yaw_pin_xy=(0.0, -a_yaw), Lf=p['Lf'],
                shoulder_height=-z_f0, foot_d=2*fr, tt_d=2*p['tt_r'], wrist_tip=ze+p['Lf']+winfo['grip_top'],
                width_shoulder=2*(u1+4.0), wrist=winfo, wrist_z=ze+p['Lf'])
    return parts, info

GUNMETAL, GUNMETAL_GEAR, STEEL, MOTOR_DARK = '#222428', '#33373d', '#c9cdd3', '#3a3d42'   # matte black PLA, brushed steel
GUN_D, GUN_L, PCB_DARK = '#1a1c1f', '#2a2d32', '#1f2226'
def tone(n):
    if n.startswith(('arm_humerus', 'foot_housing', 'foot_desk_lug', 'yaw_shoulder_pedestal', 'yaw_n23_clamp', 'foot_motor_shelf',
                     'foot_cable_tube')): return GUN_D
    if n.startswith(('cradle_plate', 'cradle_wall', 'arm_tensioner', 'arm_encoder_flex_bracket', 'arm_spacer')): return GUN_L
    return GUNMETAL

def material(n):
    if n.startswith('W_'):
        import wrist_v0 as W
        return W.material(n[2:])
    if ('steel' in n or 'screw' in n or 'nut_M5' in n or 'brg' in n or n.endswith('_shaft') or 'mpulley' in n or 'idler' in n
            or n.startswith(('cradle_spacer', 'arm_spacer_idler'))) and 'gearbox' not in n: return 'steel'
    if n.startswith(('elbow_side', 'idle_satellite', 'forearm_bevel_hub', 'yaw_ring_gear', 'foot_yaw_pinion')) or 'cycloid_disc' in n:
        return 'gear'
    if n.startswith('arm_humerus'): return 'alu'
    if n.startswith(('arm_motor_carrier', 'arm_shaft_block', 'arm_cover', 'cradle_plate', 'cradle_wall', 'cradle_spider', 'forearm_base_plate', 'yaw_turntable',
                     'yaw_shoulder_pedestal', 'yaw_shoulder_hub', 'yaw_shoulder_strap', 'yaw_balancer_bracket', 'yaw_balancer_anchor',
                     'foot_housing', 'foot_desk_lug', 'foot_deck', 'foot_yaw_cartridge', 'foot_yaw_retainer', 'foot_yaw_spacer', 'yaw_spacer_inner', 'yaw_spindle_clamp',
                     'arm_encoder_flex_bracket')): return 'printed'
    return None
def spec_of(n):
    return dict(steel=1.0, alu=0.8, printed=0.12, gear=0.25, tpu=0.05).get(material(n), 0.3 if ('gearbox' in n or 'NEMA' in n) else 0.15)
MATERIAL_COLORS = False      # True = black PLA / brushed steel palette; False = the original colour-coded view
def color_of(n):
    m = material(n) if MATERIAL_COLORS else None
    if m:
        if n.startswith('W_'):
            import wrist_v0 as W
            return W.color_of(n[2:])
        return dict(printed=tone(n), gear=GUNMETAL_GEAR, tpu='#2b2e33', steel=STEEL, alu='#d3d7dc')[m]
    if n.startswith('W_'):
        import wrist_v0 as W
        return W.color_of(n[2:])
    if 'NEMA' in n and MATERIAL_COLORS: return MOTOR_DARK
    if n.startswith('wrist_ghost'): return '#b9bec7'
    if n.startswith('belt'): return '#222222'
    if n.startswith('elec_estop'): return '#d63a2f'
    if n.startswith('elec_seg_'): return '#ff3b2a'
    if n == 'elec_display_7seg_body': return '#1a0d0d'
    if 'cable_motor' in n: return '#5a5f66'
    if 'cable' in n: return '#151515'
    if n.startswith('elec_') or 'as5600' in n: return PCB_DARK
    if 'hall' in n or 'magnet' in n: return '#2b2b2b'
    if 'cable_duct' in n or 'cable_tube' in n: return '#3a3d44'
    if '_brg' in n or 'brg_' in n or 'slew' in n: return '#c9ced6'
    if 'NEMA' in n: return '#4a525c'
    if 'shaft' in n or 'steel' in n or 'screw' in n: return '#8a8f98'
    # "war machine" palette: gunmetal printed parts, brass accents, carbon gears; bones (aluminium) and motors as they are
    if 'gearbox' in n: return '#7a5a2e'                       # bought steel planetary gearboxes: dark bronze
    if n.startswith('arm_humerus'): return '#d3d7dc'          # laser-cut aluminium
    if 'gear' in n or 'pinion' in n or 'elbow_side' in n or 'forearm_bevel' in n or 'satellite' in n: return '#17181a'   # PETG-CF / PA-CF gears
    if 'pulley' in n or 'idler' in n or 'spacer' in n or 'tensioner' in n: return '#3a3d44'
    if n.startswith(('yaw_shoulder_strap', 'yaw_balancer_bracket', 'yaw_balancer_anchor', 'arm_encoder', 'forearm_base')): return BRASS
    if n.startswith('arm_cover'): return GUN_3
    if n.startswith(('arm_motor_carrier', 'arm_shaft_block', 'yaw_turntable', 'foot_desk_lug', 'cradle_spider', 'yaw_shoulder_hub')): return GUN_2
    if n.startswith('foot_deck'): return GUN_3
    if n.startswith(('foot', 'yaw_', 'arm_', 'cradle')): return GUN_1
    return '#999999'
GUN_1, GUN_2, GUN_3, BRASS = '#2b2e33', '#3d4148', '#1f2125', '#a9853f'

def category(n):
    if 'cable' in n: return 'electronics'
    if n.startswith('W_hand_') or n in ('W_coil_RX', 'W_ferrite_RX'): return 'hand'
    if n.startswith('W_'): return 'wrist'
    if n.startswith('wrist_ghost'): return 'wrist'
    if n.startswith('belt') or 'pulley' in n or 'idler' in n or 'tensioner' in n: return 'belts'
    if 'yaw_shoulder' in n or 'NEMA23' in n or 'arm_shoulder' in n: return 'shoulder'
    if n.startswith('elec_') or 'hall' in n or 'magnet' in n or 'cable_duct' in n or 'cable_tube' in n: return 'electronics'
    if n.startswith(('foot', 'yaw_')): return 'base'
    if n.startswith('arm_'): return 'upper_arm'
    if n.startswith(('cradle', 'elbow_', 'idle_')) or 'as5600' in n or 'encoder' in n: return 'elbow'
    if n.startswith('forearm'): return 'forearm'
    return 'upper_arm'

def group(n):
    if n.startswith('W_'):
        import wrist_v0 as W
        return 'W_' + W.group(n[2:])
    if n.startswith('wrist_ghost') or n.startswith('forearm'): return 'fore'
    if n == 'foot_yaw_pinion_15T': return 'yawpin'
    if n.startswith(('foot', 'elec_')): return 'foot'
    if n.startswith('yaw_cable'): return 'yaw'
    if n.startswith('elbow_side_A') or n.startswith('elbow_brg_A'): return 'sideA'
    if n.startswith('elbow_side_B') or n.startswith('elbow_brg_B'): return 'sideB'
    if n.startswith('cradle'): return 'cradle'
    if n.startswith('idle_'): return 'idle'
    if n.startswith(('arm_', 'belt')): return 'arm'
    return 'yaw'

PRON_SIGN = 1.0      # side bevel A turns by a_e + PRON_SIGN*phi*KP, B by a_e - PRON_SIGN*phi*KP (checked by the mesh test)
IDLE_SIGN = -1.0     # idle satellite (axis -Z) turns by IDLE_SIGN*phi about +Z

def pose(parts, info, sigma=0.0, eps=0.0, phi=0.0, psi=0.0, wpitch=0.0, wroll=0.0, theta=None):
    """sigma: shoulder lean toward +Y, eps: elbow flexion toward +Y, phi: forearm pronation, psi: base yaw,
    wpitch / wroll: wrist pitch and endless roll, theta: finger wheel angle (degrees)"""
    import wrist_v0 as W
    ze = info['ze']
    O, X, Z = (0, 0, 0), (1, 0, 0), (0, 0, 1)
    E0, E1 = (0, 0, ze), (1, 0, ze)
    a_s, a_e = -sigma, -eps
    kp = info['kp']                                    # side bevel angle per degree of pronation (z_out / z_side)
    px, py = info['yaw_pin_xy']
    out = {}
    for n, s in parts.items():
        g = group(n)
        if g == 'foot': out[n] = s
        elif g == 'yawpin': out[n] = s.rotate((px, py, 0), (px, py, 1), psi*info['ratio_yaw'])
        elif g == 'yaw': out[n] = s.rotate(O, Z, psi)
        elif g == 'arm': out[n] = s.rotate(O, X, a_s).rotate(O, Z, psi)
        elif g == 'cradle': out[n] = s.rotate(E0, E1, a_e).rotate(O, X, a_s).rotate(O, Z, psi)
        elif g == 'idle': out[n] = s.rotate(E0, (0, 0, ze+1), IDLE_SIGN*phi).rotate(E0, E1, a_e).rotate(O, X, a_s).rotate(O, Z, psi)
        elif g == 'sideA': out[n] = s.rotate(E0, E1, a_e + PRON_SIGN*kp*phi).rotate(O, X, a_s).rotate(O, Z, psi)
        elif g == 'sideB': out[n] = s.rotate(E0, E1, a_e - PRON_SIGN*kp*phi).rotate(O, X, a_s).rotate(O, Z, psi)
        else:
            if n.startswith('W_'):
                s = W.pose({n[2:]: s}, wpitch, wroll, theta, info['wrist'])[n[2:]].translate(cq.Vector(0, 0, info['wrist_z']))
            out[n] = s.rotate(E0, (0, 0, ze+1), phi).rotate(E0, E1, a_e).rotate(O, X, a_s).rotate(O, Z, psi)
    return out

def interference(parts, tol=0.5, skip=lambda a, b: False):
    names = list(parts); bad = []
    for a, b in itertools.combinations(names, 2):
        if skip(a, b): continue
        ba, bb = parts[a].BoundingBox(), parts[b].BoundingBox()
        if (ba.xmax < bb.xmin or bb.xmax < ba.xmin or ba.ymax < bb.ymin or bb.ymax < ba.ymin
                or ba.zmax < bb.zmin or bb.zmax < ba.zmin): continue
        try: v = parts[a].intersect(parts[b]).Volume()
        except Exception: v = float('nan')
        if not (v < tol): bad.append((a, b, round(v, 2)))
    return bad

SKIP = lambda a, b: (any(k in a or k in b for k in ('belt', 'wrist_ghost', 'cycloid_disc', 'cycloid_input'))
                     or (a.startswith('W_') and b.startswith('W_'))      # wrist internals are checked in wrist_v0
                     or ('tensioner' in a and 'tensioner' in b and 'screw_M3' in a+b and 'slider' in a+b)    # thread in the insert
                     or ('cable' in a and 'cable' in b))                                  # leads merging into the harness

def export_all(parts, outdir='out'):
    os.makedirs(outdir + '/step', exist_ok=True)
    asm = cq.Assembly(name='arm_v015')
    for n, s in parts.items():
        cq.exporters.export(cq.Workplane('XY').newObject([s]), f'{outdir}/step/{n}.step')
        asm.add(s, name=n, color=cq.Color(color_of(n)))
    asm.save(f'{outdir}/arm_assembly.step')

if __name__ == '__main__':
    parts, info = build(P)
    print('built', len(parts), 'parts;', {k: (round(v, 2) if isinstance(v, float) else v) for k, v in info.items()})
    bad = interference(parts, skip=SKIP)
    print('interferences (belts / ghost / cycloid internals excluded):', bad if bad else 'none')
    export_all(parts)
