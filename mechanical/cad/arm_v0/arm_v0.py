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
    idl_teeth=20, idl_adj=3.0, idl_zrel=-150.0,         # toothed idlers inside the belt loops, mid-way up the upper arm
    # belts: motor 16T -> elbow pulley (as big as the side bevel), one belt per motor
    t_motor=20, t_elb=160, belt_teeth=200, belt_w=10.0, belt_t=1.5, mp_w=12.0, mp_od=12.2, mp_flange_od=16.0,
    # elbow motors: NEMA17 + bolt-on 5:1 planetary gearbox in a printed carrier, side by side ("shoulder to shoulder") at
    # the same height, facing opposite ways: the two belts are equal
    el_gb=(42.0, 41.0, 5), el_motor_len=39.0, el_shaft=(8.0, 23.5), el_wall=4.0, half_min=65.5, el_y=22.5,
    # v0.40: the two elbow NEMA17 (no gearbox) INSIDE the upper arm, back to back on the shoulder axis, held by the stubs; each
    # drives a shaft through its stub to a 16T pulley OUTSIDE the cheek; stage 1 (16T -> 80T, outside the bone) to an
    # intermediate shaft half-way up the bone, stage 2 (20T -> 160T, inside) to the elbow: 5 x 8 = 40:1
    el_back_gap=1.5, el_plate=5.0, s1=dict(t_small=20, t_big=110, teeth=200, adj=4.0, hub=(22.0, 14.0)), idl_above_i=25.0, Lu=229.0,   # v0.45: all six belts 200T (400 mm) steel cord, 20T minimum
    # upper arm (humerus): two identical teardrop bones, waterjet cut
    n17_sq=42.3, n17_hole=31.0,
    plate_t=4.0, pulley_gap=2.5,               # upper-arm bones: waterjet-cut 4 mm steel, the shoulder teeth cut in the same outline
    bone=dict(fillet=8.0, strip=40.0, taper=(55.0, 15.0), truss_ang=40.0),   # taper: from ze-55 to ze-15   # strip half-width, where it narrows to the R33 end
    low_bolts=((14.0, 62.0), (-14.0, 62.0)),   # (y, z) the two through-bolts that are not tensioners (one per side)
    sh_r=40.0, el_r=33.0, pin_d=12.0,
    cover=(90.0, 4.0),                         # one cover part for the whole arm: length, thickness (2x arm, 4x forearm)
    # cradle (outside the humerus plates)
    wall_gap=1.0, wall_t=7.5, ear_r=26.0, cr_fillet=5.0, cr_y=42.0, cr_plate_t=26.0, b61810=(65.0, 50.0, 7.0),
    hub_d=30.0, tube_od=38.0, tube_id=20.0, brg_od=42.0, brg_id=30.0, brg_w=7.0,
    Lf=266.0,                                  # forearm: elbow axis -> wrist apex (v0.45: 266, upper arm 229, reach still 700 mm)
    # shoulder: TWO short NEMA23 + 5:1 planetary under the shoulder axis, side by side and facing opposite ways; each drives ITS
    # bone through a 14T steel pinion on the 82T teeth cut in the steel bone (module 2): 5 x 82/14 = 29.3:1. No spring, no brake yet.
    # The gear is sized so the elbow motors (beside the shoulder bearings) clear the shoulder gearboxes over +-90 deg.
    # The teeth arc is the same on both bones (symmetric, plain toward the arm); teeth_margin in degrees.
    sh=dict(m=2.0, z_gear=56, z_pin=20, gb_ratio=10, pin_w=6.0, pin_bore=14.0, pin_hub=(24.0, 6.0), y=34.0,
            n23_sq=57.0, n23_len=56.0, gb_sq=60.0, gb_len=60.0, gb_shaft=(14.0, 25.0), gb_pilot=(40.0, 2.0), gb_holes=50.0,
            teeth_margin=7.0, q_range=(-90.0, 90.0), web=(16.0, -52.0), cheek_r=53.0,
            cheek_t=4.0, cheek_gap=1.5, stub=(50.0, 40.0), stub_fl=(70.0, 5.0), brg=(65.0, 50.0, 7.0), slab_h=30.0),   # v0.42: the gearboxes stand on the turntable; the block is a frame round them
    # base: Ø320, turntable and foot each in four identical quadrants; ring gear and deck one piece each (they fit the bed)
    up_t=8.0, up_hw=35.0, z_tt=None, tt_t=10.0, tt_r=160.0, lap=12.0, home_r=95.0, deck_r=124.0,
    yaw_m=1.5, yaw_ring=146, yaw_pin=15, yaw_pin_phase=18.0, yaw_w=12.0, slew=(86.0, 106.0),
    yaw_motor_len=39.0, yaw_gb=(42.0, 41.0, 5), yaw_shaft=(8.0, 23.5),   # NEMA17 with a 5:1 high-precision planetary (17HS15-1684S-HG5 class)
    roller_r=145.0,                                                      # four 625 bearings under the turntable rim, one per foot quadrant
    foot_r=160.0, foot_wall=4.0, foot_floor=6.0, foot_h=96.0, ctrl=(110.0, 75.0, 25.0), buck=(45.0, 25.0, 15.0), fan=40.0, estop_d=40.0,
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
def teardrop_x(x0, x1, r, cy, cz):
    """horizontal hole along X with a 45 deg roof: prints in a vertical wall without supports"""
    q = r*math.sqrt(0.5)
    return cyl_x(x0, x1, r).translate((0, cy, cz)).union(prism_yz([(cy - q, cz + q), (cy, cz + r*math.sqrt(2.0)), (cy + q, cz + q)], x0, x1))
def pitch_d(teeth): return teeth*2.0/math.pi            # GT2 pitch diameter
def _circ(cy, cz, r, n=72):
    return [(cy+r*math.cos(2*math.pi*k/n), cz+r*math.sin(2*math.pi*k/n)) for k in range(n)]
def mech_slot(p0, p1, w):
    """2D window with pointed (chevron) ends instead of round ones: the 'mech' look (v0.46 / wrist v1.38)"""
    dx, dz = p1[0] - p0[0], p1[1] - p0[1]; L = math.hypot(dx, dz) or 1e-9
    ux, uz = dx/L, dz/L; nx, nz = -uz, ux; h = w/2
    return [(p0[0] - ux*h, p0[1] - uz*h), (p0[0] + nx*h, p0[1] + nz*h), (p1[0] + nx*h, p1[1] + nz*h),
            (p1[0] + ux*h, p1[1] + uz*h), (p1[0] - nx*h, p1[1] - nz*h), (p0[0] - nx*h, p0[1] - nz*h)]
def octagon(cy, cz, r_in, a0=22.5):
    """regular octagon with inscribed radius r_in, flat facets at 0/45/90... deg (a0 = 22.5: vertices between them)"""
    rc = r_in/math.cos(math.radians(22.5))
    return [(cy + rc*math.cos(math.radians(a0 + 45*k)), cz + rc*math.sin(math.radians(a0 + 45*k))) for k in range(8)]

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
    xc = max(50.0, hg + p['belt_w']/2 + 0.5)                             # stage-2 belt plane (inside the bones, at the side gears)
    pul_out = xc + mw/2
    u0 = max(pul_out + p['pulley_gap'], p['half_min']); u1 = u0 + p['plate_t']   # humerus plates (the shoulder unit sets the minimum)
    wall_out = u0 - p['wall_gap']; wall_in = wall_out - p['wall_t']      # cradle walls INSIDE the bones, between pulley and bone
    assert wall_in >= pul_out + 0.8, 'no room for the cradle wall between pulley and bone'
    ml_e = p['el_motor_len']; x_mf = p['el_back_gap'] + ml_e            # elbow motor face (shaft outward)
    x_o = u1 + p['sh']['cheek_gap'] + p['sh']['cheek_t'] + p['sh']['stub_fl'][1] + 1.0 + mw/2   # stage-1 belt plane (outside the cheek)
    D_e = pitch_d(p['t_elb']); D_m = pitch_d(p['t_motor'])
    rb_e = (D_e - 0.76)/2; rf_e = max(rb_e + 1.2, Rh)
    Lu = p['Lu']                                                         # shoulder axis -> elbow axis (255 since v0.39: 700 mm reach)
    # elbow drive (v0.40): motor (on the shoulder axis, fixed) -> 16T outside the cheek -> stage-1 belt outside the bone ->
    # 80T on the intermediate shaft half-way up the bone (it slides in slots to tension stage 1) -> 20T inside -> stage-2 belt
    # -> 160T side pulley of the elbow. The 16T does not move with the arm, so turning the shoulder turns the elbow pulleys:
    # the firmware adds -1 x shoulder to both elbow motors (control/kinematics.py). Stage 2 is tensioned by two through-bolts,
    # each carrying two toothed idlers inside the loops (one per belt), 40 mm above the intermediate shaft.
    S1 = p['s1']; D1b, D1s = pitch_d(S1['t_big']), pitch_d(S1['t_small'])
    z_i = centre_distance(2.0*S1['teeth'], D1b, D1s)                     # intermediate shaft height (stage-1 centre distance)
    C2 = Lu - z_i
    r1p, r2p, rid = D_e/2, D_m/2, pitch_d(p['idl_teeth'])/2
    teeth_, zrel_ = p['belt_teeth'], -C2 + p['idl_above_i']
    def bcirc(d0):
        ic_, nn_ = tensioner_centres(C2, r1p, r2p, rid, zrel_, d0)
        (fy_, fz_), (by_, bz_) = ic_
        return [(0, 0, r1p, 1), (by_, bz_, rid, 1), (0, -C2, r2p, 1), (fy_, fz_, rid, 1)], ic_, nn_
    blen = lambda d0: belt_path(bcirc(d0)[0])[0]
    lo_, hi_ = 0.5, 12.0
    assert blen(lo_) < 2.0*teeth_ < blen(hi_), ('elbow belt: no idler position gives that tooth count', blen(lo_)/2, blen(hi_)/2)
    for _ in range(50):
        mid_ = (lo_ + hi_)/2
        if blen(mid_) < 2.0*teeth_: lo_ = mid_
        else: hi_ = mid_
    d0_ = mid_
    cc_, ic_, nn_ = bcirc(d0_)
    DRV = {}
    for side_, sg_ in (('A', -1), ('B', 1)):
        DRV[side_] = dict(sg=sg_, C=C2, z=0.0, y=0.0, idl_c=ic_, idl_n=nn_, circ=cc_, d0=d0_,
                          dL=[blen(max(0.3, d0_ + s_*p['idl_adj'])) - 2.0*teeth_ for s_ in (-1, 1)])
    zA = zB = zm = 0.0
    C, z_m, idl_c, dL = C2, 0.0, DRV['B']['idl_c'], DRV['B']['dL']
    ze = Lu
    plate_r_el = p['el_r']                                               # humerus end radius at the elbow (narrower than the pulley -> more flexion)
    # flex-encoder bracket straight along the humerus (toward the shoulder), its foot 44-54 mm from the elbow axis where
    # the cradle (now with R26 ears around the bearing ends) never sweeps; 2 M3 inserts side by side
    enc_pts = [(-5.5, -49.0), (5.5, -49.0)]

    # ---- shoulder gear geometry (used by the bones): pinion centres and the arc of the teeth, in the bone frame ----
    S_ = p['sh']; a_sh = (S_['z_gear'] + S_['z_pin'])*S_['m']/2; R_sec = S_['z_gear']*S_['m']/2
    zmot = -math.sqrt(a_sh**2 - S_['y']**2)                               # pinion / gearbox axis height
    PIN = {1: (S_['y'], zmot), -1: (-S_['y'], zmot)}                      # right bone <- motor A (front), left bone <- motor B (back)
    psp_ = math.degrees(math.atan2(S_['y'], zmot))                        # pinion of the right bone, psi = atan2(y, z) in degrees
    Q_ = max(abs(v_) for v_ in S_['q_range'])
    def arc(margin):
        """ONE arc of teeth for both bones, symmetric about the downward direction: the right pinion uses psp - Q .. psp + Q,
        the left one the mirror image, so the two bones are the same part and the plain edge faces the arm"""
        return psp_ - Q_ - margin, 360.0 - psp_ + Q_ + margin
    def fan(r, a0, a1, r0=0.0, n=120):                                    # 2D annular sector in the (y, z) plane
        out_ = [(r*math.sin(math.radians(a0 + (a1-a0)*i/n)), r*math.cos(math.radians(a0 + (a1-a0)*i/n))) for i in range(n+1)]
        inn_ = [(r0*math.sin(math.radians(a1 - (a1-a0)*i/n)), r0*math.cos(math.radians(a1 - (a1-a0)*i/n))) for i in range(n+1)] if r0 else [(0.0, 0.0)]
        return out_ + inn_

    # ================= UPPER ARM: two identical 4 mm STEEL bones with the shoulder teeth cut in; only belts inside ===========
    R_TAP5 = 2.1
    B_ = p['bone']
    R_root = R_sec - 1.25*S_['m']; R_tip = R_sec + S_['m']
    ta0, ta1 = arc(S_['teeth_margin'])
    # outline: the toothed disk on the shoulder axis, a concave fillet, a straight strip as wide as the elbow end (R33)
    w_, Rf_ = B_['strip'], B_['fillet']
    cfy, cfz = -(w_ + Rf_), math.sqrt((R_root + Rf_)**2 - (w_ + Rf_)**2)
    psi_t = math.degrees(math.atan2(w_ + Rf_, cfz))
    assert psi_t < ta0 - 1.0, 'bone fillet runs into the teeth'
    # v0.46 'mech' outline: half an octagon round the elbow, straight facets down to the strip, a straight chamfer onto the
    # toothed disk (no fillets, no curves except the disk and its teeth)
    rco_ = plate_r_el/math.cos(math.radians(22.5))
    half_ = [(0.0, ze + plate_r_el), (rco_*math.cos(math.radians(112.5)), ze + rco_*math.sin(math.radians(112.5))),
             (rco_*math.cos(math.radians(157.5)), ze + rco_*math.sin(math.radians(157.5))), (-plate_r_el, ze)]
    tz0_, tz1_ = ze - B_['taper'][0], ze - B_['taper'][1]
    half_ += [(-plate_r_el, tz1_), (-w_, tz0_), (-w_, cfz + 12.0)]          # straight taper facet, the strip, then a chamfer onto the disk
    half_ += [(R_root*math.sin(math.radians(-psi_t - (180.0 - psi_t)*i/60)), R_root*math.cos(math.radians(-psi_t - (180.0 - psi_t)*i/60))) for i in range(0, 61)]
    prof = half_ + [(-y_, z_) for y_, z_ in reversed(half_[1:-1])]
    adj = p['idl_adj']
    T_ = {'front': (DRV['A']['idl_c'][0], DRV['A']['idl_n'][0]), 'back': (DRV['A']['idl_c'][1], DRV['A']['idl_n'][1])}
    lowb = [tuple(b_) for b_ in p['low_bolts']]
    hl_ = [(ey_, ze + ez_, R_TAP3, 'encoder') for ey_, ez_ in enc_pts]   # tapped M3 for the flex-encoder bracket (used on the right bone)
    hl_ += [(y_, z_, R_M5, 'through-bolt') for y_, z_ in lowb]
    keep_ = [(y_, z_, r_ + 5.0) for y_, z_, r_, _ in hl_] + [(c_[0], ze + c_[1], adj + R_M5 + 5.0) for c_, _ in T_.values()]
    keep_ += [(-y_, z_, r_) for y_, z_, r_ in keep_] + [(0.0, 0.0, fseat(S_['brg'][0]) + 4.0), (0.0, ze, 10.5 + 6.0)]   # fseat() gives a radius
    adj1 = S1['adj']; hy_, hz_ = S1['hub']                                # intermediate shaft: slots for the shaft and the 2 hub screws
    keep_ += [(sy_*hy_*0.6, z_i + sz_*adj1, hz_ + 4.0) for sy_ in (-1, 1) for sz_ in (-1, 1)]
    def _dseg(p_, a_, b_):
        ab = (b_[0]-a_[0], b_[1]-a_[1]); t_ = max(0.0, min(1.0, ((p_[0]-a_[0])*ab[0] + (p_[1]-a_[1])*ab[1])/(ab[0]**2 + ab[1]**2 + 1e-12)))
        return math.hypot(p_[0] - a_[0] - t_*ab[0], p_[1] - a_[1] - t_*ab[1])
    edge_ = lambda q_: min(_dseg(q_, prof[i], prof[(i+1) % len(prof)]) for i in range(len(prof)))
    for (cy_, cz_), (ny_, nz_) in T_.values():
        for s_ in (-1, 1):
            assert edge_((cy_ + s_*ny_*adj, ze + cz_ + s_*nz_*adj)) > R_M5 + 3.0, 'bone too narrow at the tensioner slot'
    for y_, z_ in lowb: assert edge_((y_, z_)) > R_M5 + 3.0, 'bone too narrow at a through-bolt'
    slots_ = []                                                           # diagonal truss slots (p0, p1, width)
    def try_slot(c_, ang_, lens_, w_):
        for L_ in lens_:
            d_ = (math.sin(math.radians(ang_))*L_/2, math.cos(math.radians(ang_))*L_/2)
            p0_, p1_ = (c_[0]-d_[0], c_[1]-d_[1]), (c_[0]+d_[0], c_[1]+d_[1])
            smp_ = [(p0_[0] + (p1_[0]-p0_[0])*i/8, p0_[1] + (p1_[1]-p0_[1])*i/8) for i in range(9)]
            if any(edge_(q_) < w_/2 + 4.5 for q_ in smp_): continue
            if any(_dseg((y_, z_), p0_, p1_) < w_/2 + r_ for y_, z_, r_ in keep_): continue
            if any(min(_dseg(q_, s0_, s1_) for q_ in smp_) < w_/2 + sw_/2 + 5.0 for s0_, s1_, sw_ in slots_): continue
            slots_.append((p0_, p1_, w_)); return True
        return False
    fl_ = 1
    for z_ in range(int(ze - 30), 40, -1):                                # strip: a zigzag (Warren truss) of diagonal slots
        if try_slot((0.0, float(z_)), fl_*B_['truss_ang'], (44.0, 36.0, 30.0, 24.0), 9.0): fl_ = -fl_
    for a_ in (180.0, 145.0, 110.0):                                      # disk: slanted slots round the bearing
        for sgn_ in ((1,) if a_ == 180.0 else (1, -1)):
            c_ = (sgn_*41.0*math.sin(math.radians(a_)), 41.0*math.cos(math.radians(a_)))
            try_slot(c_, sgn_*(a_ + 90.0), (16.0, 12.0), 7.0)
    light_ = slots_
    gear_full = SpurGear(module=S_['m'], teeth_number=S_['z_gear'], width=p['plate_t'], bore_d=20.0, backlash=0.06).build()
    reg = cq.Workplane('XY').polyline([(-zz, yy) for yy, zz in fan(R_tip + 3.0, ta0, ta1, r0=R_root - 2.0)]).close().extrude(p['plate_t'])
    teeth_ = cq.Workplane('XY').newObject([gear_full]).intersect(reg).val()          # local (xl, yl) -> bone frame (y = yl, z = -xl)
    teeth_ = teeth_.rotate(O3, YV, 90)
    for sg, nm in ((1, 'R'), (-1, 'L')):
        x0, x1 = xr(sg*u0, sg*u1)
        mir = (lambda y_: -y_) if sg > 0 else (lambda y_: y_)
        pl = prism_yz(prof, x0, x1).union(teeth_.translate(cq.Vector(x0, 0, 0)))
        pl = pl.cut(cyl_x(x0-1, x1+1, fseat(21.0)).translate((0, 0, ze)))                      # 6801 in a through bore (ream; Loctite 638)
        pl = pl.cut(cyl_x(x0-1, x1+1, fseat(S_['brg'][0])))                                    # 6810 on the fixed stub of the cheek
        for y_, z_, r_, _ in hl_: pl = pl.cut(cyl_x(x0-1, x1+1, r_).translate((0, mir(y_), z_)))
        for s0_, s1_, w2_ in light_:                                       # mech windows with pointed ends (v0.46)
            pl = pl.cut(prism_yz(mech_slot((mir(s0_[0]), s0_[1]), (mir(s1_[0]), s1_[1]), w2_), x0-1, x1+1))
        for (cy_, cz_), (ny_, nz_) in T_.values():                         # tensioner slots, both bones
            sl = _hull(_circ(cy_-ny_*adj, ze+cz_-nz_*adj, R_M5) + _circ(cy_+ny_*adj, ze+cz_+nz_*adj, R_M5))
            pl = pl.cut(prism_yz(sl, x0-1, x1+1))
        for cy_, r_ in ((0.0, 5.0), (16.0, 2.2), (-16.0, 2.2)):            # intermediate shaft + hub screws (M4), slots along the bone
            pl = pl.cut(prism_yz(_hull(_circ(cy_, z_i - adj1, r_) + _circ(cy_, z_i + adj1, r_)), x0-1, x1+1))
        parts[f'arm_humerus_{nm}_steel'] = pl.val()
    bw2 = p['belt_w']/2
    def through_bolt(tag_, cy_, zt_, sg, idlers=()):
        """M5 through-bolt from bone to bone: button head on side sg, flange nut on the other, tubes between the bones,
        split round the idlers it carries"""
        cuts_ = sorted([(ix_ - bw2 - 1.5, ix_ + bw2 + 1.5) for ix_ in idlers])
        x_ = -u0
        for k_, (a_, b_) in enumerate(cuts_ + [(u0, u0)]):
            if a_ - x_ > 0.5: parts[f'arm_spacer_bolt_{tag_}_{k_}'] = cyl_x(x_, a_, 4.5, 2.6).translate((0, cy_, zt_)).val()
            x_ = b_
        for ix_ in idlers:
            nm_ = 'A' if ix_ < 0 else 'B'
            idl = (cyl_x(ix_-bw2, ix_+bw2, rid - 0.4).union(cyl_x(ix_-bw2-1.5, ix_-bw2, rid+2.0))
                   .union(cyl_x(ix_+bw2, ix_+bw2+1.5, rid+2.0)).cut(cyl_x(ix_-bw2-2, ix_+bw2+2, R_M5)))
            parts[f'arm_idler_{tag_}_{nm_}_20T'] = idl.translate((0, cy_, zt_)).val()
        bolt = cyl_x(*xr(sg*(u1+2.75), -sg*(u1+6.0)), 2.5).union(cyl_x(*xr(sg*u1, sg*(u1+2.75)), 4.75))
        parts[f'arm_bolt_{tag_}_screw_M5'] = bolt.translate((0, cy_, zt_)).val()
        parts[f'arm_bolt_{tag_}_nut_M5'] = cyl_x(*xr(-sg*u1, -sg*(u1+4.7)), 4.6, 2.5).translate((0, cy_, zt_)).val()
    for tag_, ((cy_, cz_), _) in T_.items():
        through_bolt(f'tensioner_{tag_}', cy_, ze + cz_, 1 if cy_ > 0 else -1, idlers=(-xc, xc))
    for y_, z_ in lowb:
        through_bolt(f'low_{"front" if y_ > 0 else "back"}', y_, z_, 1 if y_ > 0 else -1)
    for side, d_ in DRV.items():
        parts[f'belt_elbow_{side}_GT2'] = belt_solid(d_['sg']*xc, d_['circ'], p['belt_w']).translate((0, 0, ze)).val()
        parts[f'belt_stage1_{side}_GT2'] = belt_solid(d_['sg']*x_o, [(0.0, z_i, D1b/2, 1), (0.0, 0.0, D1s/2, 1)], p['belt_w']).val()
    # intermediate shaft, one per side: printed hub on the OUTER face of the bone with two 688, Ø8 shaft through a slot of the
    # bone, 80T outside (stage 1), 20T inside (stage 2); two M4 through slots clamp the hub (slide it to tension stage 1)
    for sg, nm in ((-1, 'A'), (1, 'B')):
        xh1_ = x_o - mw/2 - 0.5
        hub_ = prism_yz(_hull(_circ(-hy_ + hz_, z_i, hz_) + _circ(hy_ - hz_, z_i, hz_)), *xr(sg*u1, sg*xh1_))
        hub_ = hub_.cut(cyl_x(*xr(sg*(u1-1), sg*(xh1_+1)), 4.6).translate((0, 0, z_i)))
        for x0_ in (u1 - 0.01, xh1_ - 5.0):
            hub_ = hub_.cut(cyl_x(*xr(sg*x0_, sg*(x0_ + 5.01)), fseat(16.0)).translate((0, 0, z_i)))
        for cy_ in (-16.0, 16.0):
            hub_ = hub_.cut(cyl_x(*xr(sg*(u1-1), sg*(xh1_+1)), 2.2).translate((0, cy_, z_i))).cut(cyl_x(*xr(sg*(xh1_-4.0), sg*(xh1_+1)), 4.2).translate((0, cy_, z_i)))
        parts[f'arm_mid_hub_{nm}'] = hub_.val()
        for k_, x0_ in enumerate((u1, xh1_ - 5.0)):
            parts[f'arm_mid_brg_{nm}_688_{k_+1}'] = cyl_x(*xr(sg*x0_, sg*(x0_ + 5.0)), 8.0, 4.0).translate((0, 0, z_i)).val()
        parts[f'arm_mid_{nm}_shaft'] = cyl_x(*xr(sg*(xc - mw/2 - 0.5), sg*(x_o + mw/2 + 0.5)), 4.0).translate((0, 0, z_i)).val()
        parts[f'arm_mid_pulley_{nm}_{S1["t_big"]}T'] = pulley_gen(sg*x_o - mw/2, D1b/2 - 0.38, D1b/2 + 2.0, mw, 4.0).translate((0, 0, z_i)).val()
        parts[f'arm_mid_pulley_{nm}_{p["t_motor"]}T'] = pulley_gen(sg*xc - mw/2, p['mp_od']/2, p['mp_flange_od']/2, mw, 4.0).translate((0, 0, z_i)).val()
        for cy_ in (-16.0, 16.0):                                          # M4 x 30 + nut inside
            parts[f'arm_mid_screw_{nm}_{"f" if cy_ > 0 else "b"}_M4'] = cyl_x(*xr(sg*(xh1_ - 4.0), sg*(u0 - 3.0)), 2.0).translate((0, cy_, z_i)).val()
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
        body = cq.Workplane('XY').newObject([gear]).union(pu, clean=False).cut(cyl_x(-200, 200, p['pin_d']/2+0.2), clean=False)
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

    # ================= SHOULDER: two short NEMA23 with 5:1 planetaries, steel pinions on the teeth of the steel bones ==========
    # 57 mm NEMA23, 56 long, behind a 5:1 planetary (60 square, Ø14 output). Both under the axis, side by side, facing opposite
    # ways; each gearbox face is held by a wall of the printed motor block with 4 slotted M5 (slide toward the bone to set the
    # backlash). The pinion meshes in the plane of the bone; its shaft end passes through the cheek. No brake for now
    # (a mechanical brake on the bones is to come). The bones turn on 6806 bearings around fixed hollow stubs held by two
    # printed PETG-CF cheeks; the cables enter the arm through the right stub.
    z_tt = p['z_tt'] if p['z_tt'] is not None else zmot - S_['gb_sq']/2   # v0.42: turntable top = underside of the shoulder gearboxes (as low as it goes)
    q23 = S_['gb_sq']; h23 = S_['gb_holes']/2
    x_teeth = (u0 - 1.0, u0 + p['plate_t'] + 1.0)                       # pinion teeth, 1 mm wider than the bone on each side
    x_hub0 = x_teeth[0] - S_['pin_hub'][1]
    x_face = x_hub0 - 0.5 - S_['gb_pilot'][1]                             # gearbox face (its pilot stands proud of it)
    x_gb = x_face - S_['gb_len']; x_back = x_gb - S_['n23_len']
    xs_end = x_face + S_['gb_shaft'][1]
    xch0 = u1 + S_['cheek_gap']; xch1 = xch0 + S_['cheek_t']              # cheeks, just outside the bones
    assert x_back > -xch0 + 1.0, 'motor + gearbox reach the other cheek'
    assert zmot - q23/2 > z_tt - 0.01, 'shoulder motors below the turntable'
    def to_x(sh_, x0):
        return sh_.rotate(O3, YV, 90).translate(cq.Vector(x0, 0, 0))
    pin_g = SpurGear(module=S_['m'], teeth_number=S_['z_pin'], width=S_['pin_w'], bore_d=S_['pin_bore'], backlash=0.06).build()
    for sg, nm, tag in ((1, 'R', 'A'), (-1, 'L', 'B')):
        yp_, zp_ = PIN[sg]
        bone_ = parts[f'arm_humerus_{nm}_steel']
        def pinion_at(ph):                                                # built in the local frame, then put at x_teeth
            pg = pin_g.rotate(O3, ZV, ph).translate(cq.Vector(-zp_, yp_, 0))
            return to_x(pg, x_teeth[0] if sg > 0 else -x_teeth[1])
        best = None                                                       # each side has its own phase (smallest overlap)
        for k_ in range(30):
            ph = k_*(360.0/S_['z_pin'])/30
            v_ = bone_.intersect(pinion_at(ph)).Volume()
            if best is None or v_ < best[0]: best = (v_, ph)
        pt = pinion_at(best[1])
        hub = cyl_x(*xr(sg*x_hub0, sg*(x_teeth[0]+0.6)), S_['pin_hub'][0]/2).translate((0, yp_, zp_))
        pin_s = cq.Workplane('XY').newObject([pt]).union(hub, clean=True).cut(cyl_x(-300, 300, S_['pin_bore']/2 + 0.02).translate((0, yp_, zp_)))
        parts[f'yaw_shoulder_pinion_{tag}_{S_["z_pin"]}T_steel'] = pin_s.val()
        # motor, gearbox, output shaft
        parts[f'yaw_gearbox_shoulder_{tag}_planetary_5'] = (bx(*xr(sg*x_gb, sg*x_face), yp_-q23/2, yp_+q23/2, zp_-q23/2, zp_+q23/2)
                                                       .union(cyl_x(*xr(sg*x_face, sg*(x_face+S_['gb_pilot'][1])), S_['gb_pilot'][0]/2).translate((0, yp_, zp_)))).val()
        parts[f'yaw_motor_shoulder_{tag}_NEMA23'] = bx(*xr(sg*x_back, sg*x_gb), yp_-S_['n23_sq']/2, yp_+S_['n23_sq']/2, zp_-S_['n23_sq']/2, zp_+S_['n23_sq']/2).val()
        parts[f'yaw_gearbox_shoulder_{tag}_shaft'] = cyl_x(*xr(sg*(x_face+S_['gb_pilot'][1]), sg*xs_end), S_['gb_shaft'][0]/2).translate((0, yp_, zp_)).val()
        # bone bearing, fixed stub with flange on the cheek, spacer between the bearing inner ring and the cheek
        bo_, bi_, bw_ = S_['brg']
        parts[f'arm_brg_shoulder_{nm}_6810'] = cyl_x(*xr(sg*(u1 - bw_), sg*u1), bo_/2, bi_/2).val()
        so_, sb_ = S_['stub']; fo_, ft_ = S_['stub_fl']; rfh_ = fo_/2 - 5.0
        stub = cyl_x(*xr(sg*(u1 - bw_ - 2.0), sg*xch1), so_/2, sb_/2).union(cyl_x(*xr(sg*xch1, sg*(xch1+ft_)), fo_/2, sb_/2))
        for k in range(4):
            a = math.pi/4 + k*math.pi/2
            stub = stub.cut(cyl_x(*xr(sg*(xch1-1), sg*(xch1+ft_+1)), 2.2).translate((0, rfh_*math.cos(a), rfh_*math.sin(a))))
        stub = stub.cut(cyl_x(*xr(sg*(u1-bw_-0.8), sg*(u1-bw_-2.1)), so_/2+1, so_/2-1.0))      # groove for the retaining ring
        for k in range(4):                                                 # tapped M3 in the inner end: elbow motor bracket
            a = math.pi/4 + k*math.pi/2
            stub = stub.cut(cyl_x(*xr(sg*(u1 - bw_ - 2.5), sg*(u1 - bw_ + 6.0)), R_TAP3).translate((0, 22.5*math.cos(a), 22.5*math.sin(a))))
        parts[f'yaw_shoulder_stub_{nm}_alu'] = stub.val()
        parts[f'yaw_shoulder_ring_{nm}_steel'] = cyl_x(*xr(sg*(u1-bw_-2.1), sg*(u1-bw_-1.0)), so_/2+2.0, so_/2-1.0).val()   # DIN 471 retaining ring
        parts[f'yaw_shoulder_spacer_{nm}_steel'] = cyl_x(*xr(sg*u1, sg*xch0), bi_/2 + 3.0, so_/2+0.05).val()
        # cheek: waterjet-cut 4 mm steel (v0.41), left and right the SAME part (both gearbox shaft holes), from the turntable to
        # the axis, screwed to the end face of the motor block; M4 tapped for the stub flange.
        # Keyhole outline: a slim web under the stub, then a concave flare onto the block
        # v0.46 'mech' cheek: a chunky faceted shield, an octagon round the stub on a full-width faceted foot
        rch_ = S_['cheek_r']
        # the flanks start below z = -20: above that the heads of the low through-bolts sweep past at R59 when the arm is at +-90
        ch = prism_yz(octagon(0.0, 0.0, rch_), *xr(sg*xch0, sg*xch1)).union(prism_yz(
            [(-48.0, -20.0), (48.0, -20.0), (78.0, z_tt + 26.0), (78.0, z_tt), (-78.0, z_tt), (-78.0, z_tt + 26.0)], *xr(sg*xch0, sg*xch1)))
        for sy_ in (-1, 1):                                                # two pointed windows in the flanks
            ch = ch.cut(prism_yz(mech_slot((sy_*58.0, z_tt + 34.0), (sy_*46.0, -34.0), 11.0), *xr(sg*(xch0-1), sg*(xch1+1))))
        ch = ch.cut(cyl_x(*xr(sg*(xch0-1), sg*(xch1+1)), so_/2+0.15))
        for k in range(4):
            a = math.pi/4 + k*math.pi/2
            ch = ch.cut(cyl_x(*xr(sg*(xch0-1), sg*(xch1+1)), 1.65).translate((0, rfh_*math.cos(a), rfh_*math.sin(a))))   # M4 tapped (Ø3.3 cut), stub flange
        for yy_ in (yp_, -yp_):                                            # gearbox shaft end passes through (both holes: L = R)
            ch = ch.cut(cyl_x(*xr(sg*(xch0-1), sg*(xch1+1)), S_['gb_shaft'][0]/2 + 1.5).translate((0, yy_, zp_)))
        for cy_ in (-62.0, -12.0, 12.0, 62.0): ch = ch.cut(cyl_x(*xr(sg*(xch0-1), sg*(xch1+1)), R_M5).translate((0, cy_, z_tt + 6.0)))
        parts[f'yaw_cheek_{nm}_steel'] = ch.val()
        # printed foot (v0.42) outside the bottom of the steel cheek: screwed to the turntable (2 x M5 from below) and to the
        # cheek (4 x M4 into inserts), it turns the cheek-to-base joint into a wide footing
        cf_ = prism_xz([(sg*xch1, z_tt), (sg*(xch1+14.0), z_tt), (sg*(xch1+14.0), z_tt+5.0), (sg*(xch1+4.0), z_tt+22.0), (sg*xch1, z_tt+22.0)], -56.0, 56.0)
        for cy_ in (-30.0, 30.0): cf_ = cf_.cut(cyl_z(z_tt-1, z_tt+10.0, R_INS5, cx=sg*(xch1+8.0), cy=cy_))
        for cy_ in (-46.0, -28.0, 28.0, 46.0):
            cf_ = cf_.cut(cyl_x(*xr(sg*(xch1-0.1), sg*(xch1+9.0)), 2.8).translate((0, cy_, z_tt+9.0)))
            parts[f'yaw_cheek_{nm}_steel'] = parts[f'yaw_cheek_{nm}_steel'].cut(cyl_x(*xr(sg*(xch0-1), sg*(xch1+1)), 2.2).translate((0, cy_, z_tt+9.0)).val())
        parts[f'yaw_cheek_foot_{nm}'] = cf_.val()
        # elbow motor of this side (v0.40): NEMA17 without gearbox INSIDE the arm, coaxial with the shoulder, back to back with
        # the other one, shaft outward. A printed bracket (face plate + tube) is screwed to the inner end of the stub (4 x M3)
        # and carries a 688 in a spider; jaw coupling to a Ø8 drive shaft through the stub; a second 688 in a printed bushing
        # at the outer end of the stub; 16T pulley outside the cheek (stage 1 runs outside the bone)
        es = 'A' if sg < 0 else 'B'
        sq_e = p['n17_sq']; xsi_ = u1 - bw_ - 2.0                         # stub inner end
        xp1_ = x_mf + p['el_plate']
        parts[f'yaw_elbow_motor_{es}_NEMA17'] = bx(*xr(sg*p['el_back_gap'], sg*x_mf), -sq_e/2, sq_e/2, -sq_e/2, sq_e/2).val()
        parts[f'yaw_elbow_motor_{es}_shaft'] = cyl_x(*xr(sg*x_mf, sg*(x_mf + 6.0)), 2.5).val()
        brk = bx(*xr(sg*x_mf, sg*xp1_), -sq_e/2, sq_e/2, -sq_e/2, sq_e/2).edges('|X').fillet(4.0)      # face plate on the motor
        brk = brk.cut(cyl_x(*xr(sg*(x_mf-1), sg*(xp1_+1)), 11.5))                                       # motor boss, coupling
        for k in range(4):
            a = math.pi/4 + k*math.pi/2
            brk = brk.cut(cyl_x(*xr(sg*(x_mf-1), sg*(xp1_+1)), R_M3).translate((0, 21.9*math.cos(a), 21.9*math.sin(a))))
        brk = brk.union(cyl_x(*xr(sg*(xp1_-0.01), sg*(xsi_-0.2)), so_/2, sb_/2 + 0.5))                   # tube onto the stub end
        win_ = cq.Workplane('YZ').workplane(offset=min(sg*(xp1_+0.8), sg*(xp1_+8.8))).polyline(
            [(0.0, 0.0)] + [(30.0*math.cos(math.radians(a_)), 30.0*math.sin(math.radians(a_))) for a_ in range(-20, 201, 10)]).close().extrude(8.0)
        brk = brk.cut(win_)                                                # cable window: the arm cable leaves here, any shoulder angle
        winb_ = cq.Workplane('YZ').workplane(offset=min(sg*(xp1_+0.8), sg*(xp1_+8.8))).polyline(
            [(0.0, 0.0)] + [(30.0*math.cos(math.radians(a_)), 30.0*math.sin(math.radians(a_))) for a_ in range(200, 266, 5)]).close().extrude(8.0)
        brk = brk.cut(winb_)                                               # and the cable comes in here, from the motor block below
        spd = cyl_x(*xr(sg*(xsi_-6.2), sg*(xsi_-0.2)), 10.5, 8.0 - 0.01)
        for a_ in (30.0, 150.0, 270.0):                                    # three spokes, the top left free for the cable
            spd = spd.union(bx(*xr(sg*(xsi_-6.2), sg*(xsi_-0.2)), 9.0, sb_/2 + 1.0, -2.5, 2.5).rotate(O3, XV, a_))
        brk = brk.union(spd).cut(cyl_x(*xr(sg*(xsi_-7), sg*xsi_), 8.0))
        for k in range(4):
            a = math.pi/4 + k*math.pi/2
            brk = brk.cut(cyl_x(*xr(sg*(xsi_-9.0), sg*(xsi_+1)), R_M3).translate((0, 22.5*math.cos(a), 22.5*math.sin(a))))
        parts[f'yaw_elbow_bracket_{es}'] = brk.val()
        parts[f'yaw_elbow_brg_{es}_688_in'] = cyl_x(*xr(sg*(xsi_-6.0), sg*(xsi_-1.0)), 8.0, 4.0).val()
        parts[f'yaw_elbow_coupling_{es}_steel'] = cyl_x(*xr(sg*(x_mf + 0.5), sg*(x_mf + 6.5)), 9.0, 2.5).union(cyl_x(*xr(sg*(x_mf + 6.5), sg*(x_mf + 12.5)), 9.0, 4.0)).val()
        xbo_ = xch1 + ft_                                                  # outer end of the stub
        x_o = xbo_ + 1.0 + mw/2                                            # stage-1 belt plane, outside the cheek and flange
        parts[f'yaw_elbow_drive_{es}_shaft'] = cyl_x(*xr(sg*(x_mf + 7.0), sg*(x_o + mw/2 + 0.5)), 4.0).val()
        bsh = cyl_x(*xr(sg*(xbo_ - 9.0), sg*xbo_), sb_/2 - 0.1, 4.5).cut(cyl_x(*xr(sg*(xbo_ - 5.5), sg*(xbo_+1)), fseat(16.0)))
        parts[f'yaw_elbow_bushing_{es}'] = bsh.union(cyl_x(*xr(sg*xbo_, sg*(xbo_ + 0.8)), sb_/2 + 3.0, 8.5)).val()   # lip on the stub end
        parts[f'yaw_elbow_brg_{es}_688_out'] = cyl_x(*xr(sg*(xbo_ - 5.0), sg*xbo_), 8.0, 4.0).val()
        mp = pulley_gen(sg*x_o - mw/2, pitch_d(S1['t_small'])/2 - 0.38, pitch_d(S1['t_small'])/2 + 2.5, mw, 4.0)
        parts[f"yaw_elbow_mpulley_{es}_{S1['t_small']}T"] = mp.val()
    # printed motor block: a slab under the motors (bolted to the turntable, the cheeks screwed to its end faces) and a face
    # wall for each gearbox with the pilot bore and 4 slots (along the line pinion -> gear centre)
    sh_ = S_['slab_h']; zs1 = z_tt + sh_
    blk = prism_xy_r(-xch0, xch0, -78.0, 78.0, z_tt, zs1, 6.0)
    for sg, tag in ((1, 'A'), (-1, 'B')):
        yp_, zp_ = PIN[sg]
        blk = blk.cut(bx(*xr(sg*(x_back-1.0), sg*(x_face+1.0)), yp_-q23/2-1.0, yp_+q23/2+1.0, zp_-q23/2-0.5, zs1+1))   # pocket under the motor
    blk = blk.cut(bx(-xch0-1, xch0+1, -5.5, 5.0, z_tt-1, zs1+1.0))                                   # cable channel, open at the top
    for sg, tag in ((1, 'A'), (-1, 'B')):
        yp_, zp_ = PIN[sg]
        wy0, wy1 = sorted((yp_ - sg*(q23/2 + 3.0), yp_ + sg*(q23/2 + 3.0)))
        wy0, wy1 = (max(wy0, 5.0), wy1) if sg > 0 else (wy0, min(wy1, -5.0))
        wall = bx(*xr(sg*x_face, sg*(x_face+6.0)), wy0, wy1, z_tt, zp_ + q23/2 + 3.0)       # in front of the gearbox face
        wall = wall.cut(teardrop_x(*xr(sg*(x_face-1), sg*(x_face+7)), S_['gb_pilot'][0]/2 + 0.6, yp_, zp_))
        d_ = (-yp_/a_sh, -zp_/a_sh)                                       # toward the gear centre
        for hy_, hz_ in ((h23, h23), (-h23, h23), (h23, -h23), (-h23, -h23)):
            cy_, cz_ = yp_ + hy_, zp_ + hz_
            sl = _hull(_circ(cy_ - 1.2*d_[0], cz_ - 1.2*d_[1], 2.75) + _circ(cy_ + 1.2*d_[0], cz_ + 1.2*d_[1], 2.75))
            wall = wall.cut(prism_yz(sl, *xr(sg*(x_face-1), sg*(x_face+7))))
        for gy_ in (wy0 + 4.0, wy1 - 4.0):                                # gussets from the wall out onto the end of the slab
            wall = wall.union(prism_xz([(sg*(x_face+5.99), zs1-0.01), (sg*(x_face+5.99), zs1 + 12.0), (sg*(x_face+14.0), zs1-0.01)], gy_-3.0, gy_+3.0))
        blk = blk.union(wall)
    for sg in (1, -1):
        for cy_ in (-62.0, -12.0, 12.0, 62.0):                            # cheek screws: M5 inserts in the end faces
            blk = blk.cut(cyl_x(*xr(sg*(xch0-11.0), sg*(xch0+1)), R_INS5).translate((0, cy_, z_tt + 6.0)))
    for sg in (1, -1):                                                    # v0.42: clear the pinions and the toothed bone disks
        blk = blk.cut(bx(*xr(sg*(x_face + 6.0), sg*(xch0 + 1.0)), -80.0, 80.0, z_tt + 12.0, 0.0))
        ypp_, zpp_ = PIN[sg]                                               # v0.44: the 20T pinion reaches lower
        blk = blk.cut(cyl_x(*xr(sg*(x_face + 6.0), sg*(xch0 + 1.0)), S_['m']*(S_['z_pin'] + 2)/2 + 2.0).translate((0, ypp_, zpp_)))
    for sg in (1, -1):                                                    # pilot bore through wall AND slab
        yp_, zp_ = PIN[sg]
        blk = blk.cut(teardrop_x(*xr(sg*(x_face-1), sg*(x_face+7)), S_['gb_pilot'][0]/2 + 0.6, yp_, zp_))
    blk_pts = tuple((px_, py_) for px_ in (-62.0, 62.0) for py_ in (-70.0, 70.0)) + ((0.0, 70.0), (0.0, -70.0), (66.0, -12.0), (-66.0, 12.0))
    for px_, py_ in blk_pts: blk = blk.cut(cyl_z(z_tt-1, z_tt+10.5, R_INS5, cx=px_, cy=py_))       # M5 from under the turntable
    # the open cable channel splits the block in two: one half per motor, each with 4 bolts down and 2 per cheek
    parts['yaw_shoulder_motor_block_A'] = blk.intersect(bx(-200, 200, 0.0, 200, z_tt-5, 50)).val()
    parts['yaw_shoulder_motor_block_B'] = blk.intersect(bx(-200, 200, -200, 0.0, z_tt-5, 50)).val()
    ped_pts = blk_pts

    # ================= BASE: turntable and foot in FOUR IDENTICAL QUADRANTS round a one-piece ring gear and a central hub ======
    # Ø320 for stability. Each quadrant prints alone (160 x 160). Joints: half-lap along each edge (the quadrant's tongue on
    # its counter-clockwise edge sits in the recess of the next one) with M3 into inserts; every hole pattern is 4-fold
    # symmetric, so the four quadrants are the same part. The turntable is tied by the hub, the ring gear and the shoulder
    # motor block bolts; the foot by the floor laps, a tab on each wall joint and the desk lugs, which straddle the joints.
    tt = p['tt_t']; zg1 = z_tt - tt; zg0 = zg1 - p['yaw_w']; z_f0 = zg0 - p['foot_h']
    yr_pd = p['yaw_m']*p['yaw_ring']; yp_pd = p['yaw_m']*p['yaw_pin']; a_yaw = (yr_pd - yp_pd)/2
    T = zg1; r_sp, r_ct, r_bs = 25.0, 38.5, fseat(65.0)
    lap = p['lap']; R4 = lambda q_, k_: (q_[0]*math.cos(k_*math.pi/2) - q_[1]*math.sin(k_*math.pi/2), q_[0]*math.sin(k_*math.pi/2) + q_[1]*math.cos(k_*math.pi/2))
    sym4 = lambda pts_: [R4(q_, k_) for q_ in pts_ for k_ in range(4)]
    Q1 = lambda z0_, z1_, r_=500.0: bx(0.0, r_, 0.0, r_, z0_, z1_)       # the first quadrant (x >= 0, y >= 0)
    # --- hub (one piece): spindle in the two 6810, flange under the quadrants
    zh = zg1 + tt/2
    hub = (cyl_z(zg1, z_tt, 45.0).union(cyl_z(zg1, zh, 62.0)).union(cyl_z(T-4.5, T+0.01, 27.5)).union(cyl_z(T-36.2, T-4.4, r_sp+FIT['shaft']/2))
           .cut(cyl_z(T-41, z_tt+1, 19.0)))
    for k in range(4):
        a = math.pi/4 + k*math.pi/2
        hub = hub.cut(cyl_z(T-37, T-26, R_TAP3, cx=22.0*math.cos(a), cy=22.0*math.sin(a)))       # clamp ring screws
    hub_pts = sym4([(54.0*math.cos(math.radians(22.5)), 54.0*math.sin(math.radians(22.5))), (54.0*math.cos(math.radians(67.5)), 54.0*math.sin(math.radians(67.5)))])
    for cx_, cy_ in hub_pts: hub = hub.cut(cyl_z(zh-4.5, zh+1, R_INS3, cx=cx_, cy=cy_))          # quadrants screwed down onto the flange
    parts['yaw_turntable_hub'] = hub.val()
    # --- ring gear (one piece, as large as the bed allows), 8 x M3 up into the quadrants
    ring = RingGear(module=p['yaw_m'], teeth_number=p['yaw_ring'], width=p['yaw_w'], rim_width=8.0).build()
    r_rg = ring.BoundingBox().xmax - 4.0
    assert 2*ring.BoundingBox().xmax < 250.0, 'ring gear no longer fits the bed'
    rg = cq.Workplane('XY').newObject([ring.translate(cq.Vector(0, 0, zg0))])
    rg_pts = [(r_rg*math.cos(math.radians(22.5 + 45*k)), r_rg*math.sin(math.radians(22.5 + 45*k))) for k in range(8)]
    for cx_, cy_ in rg_pts: rg = rg.cut(cyl_z(zg1-9.0, zg1+1, R_TAP3, cx=cx_, cy=cy_))
    parts[f"yaw_ring_gear_{p['yaw_ring']}T"] = rg.val()
    # --- turntable quadrant
    tr = p['tt_r']
    q = cyl_z(zg1, z_tt, tr).intersect(Q1(zg1-1, z_tt+1))
    q = q.cut(bx(-1.0, tr+1, -1.0, lap, zh, z_tt+1))                      # recess (upper half) along the y = 0 edge
    q = q.union(cyl_z(zh, z_tt, tr).intersect(bx(-lap, 0.0, 0.0, tr+1, zh, z_tt)))   # tongue (upper half) past the x = 0 edge
    q = q.cut(cyl_z(zg1-1, z_tt+1, 45.2)).cut(cyl_z(zg1-1, zh, 62.2))     # round the hub
    lap_pts = sym4([(-lap/2, 80.0), (-lap/2, 140.0)])
    for cx_, cy_ in lap_pts:
        q = q.cut(cyl_z(zh, z_tt+1, R_M3, cx=cx_, cy=cy_)).cut(cyl_z(z_tt-3.0, z_tt+1, 3.2, cx=cx_, cy=cy_)).cut(cyl_z(zg1+1.0, zh+0.1, R_INS3, cx=cx_, cy=cy_))
    for cx_, cy_ in sym4(list(ped_pts) + [(xch1 + 8.0, 30.0), (xch1 + 8.0, -30.0)]):   # motor block and cheek-foot bolts (M5 from below, head sunk)
        q = q.cut(cyl_z(zg1-1, z_tt+1, R_M5, cx=cx_, cy=cy_)).cut(cyl_z(zg1-1, zg1+5.0, 4.7, cx=cx_, cy=cy_))
    for cx_, cy_ in rg_pts + hub_pts:                                     # M3 from the top into the ring gear and the hub flange
        q = q.cut(cyl_z(zg1-1, z_tt+1, R_M3, cx=cx_, cy=cy_)).cut(cyl_z(z_tt-3.0, z_tt+1, 3.2, cx=cx_, cy=cy_))
    for k in range(4): parts[f'yaw_turntable_q{k+1}'] = q.rotate(O3, ZV, 90*k).val()
    # --- yaw bearing cartridge (unchanged), home sensor, pinion
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
    rmag = p['home_r']                                                    # inside the ring teeth, away from the pinion
    parts['yaw_home_magnet'] = cyl_z(zg1-3.0, zg1, 3.0, cx=rmag*math.cos(math.radians(-60)), cy=rmag*math.sin(math.radians(-60))).val()
    parts['foot_home_hall_sensor'] = bx(rmag-4.0, rmag+3.0, -4.0, 4.0, zg0+4.0, zg0+8.0).rotate(O3, ZV, -60.0).val()   # on a post of the deck
    pin = SpurGear(module=p['yaw_m'], teeth_number=p['yaw_pin'], width=p['yaw_w'], bore_d=p['yaw_shaft'][0]).build()
    parts[f"foot_yaw_pinion_{p['yaw_pin']}T"] = pin.rotate(O3, ZV, p['yaw_pin_phase']).translate(cq.Vector(0, -a_yaw, zg0))
    # --- foot quadrant: floor + wall, one window in the middle of its wall (a panel goes in it), two pilasters for the deck,
    # one rim roller on the wall top
    fr = p['foot_r']; fi = fr - p['foot_wall']; ft = p['foot_floor']; zfh = z_f0 + ft/2; zm = (z_f0 + zg0)/2
    fq = cyl_z(z_f0, zg1-1.5, fr, fi).union(cyl_z(z_f0, z_f0+ft, fr)).intersect(Q1(z_f0-1, zg1))
    fq = fq.cut(cyl_z(zfh, z_f0+ft+1, fi-0.2).intersect(bx(-1.0, fi, -1.0, lap, zfh, z_f0+ft+1)))     # floor recess along y = 0
    fq = fq.union(cyl_z(zfh, z_f0+ft, fi-0.2).intersect(bx(-lap, 0.0, 0.0, fi, zfh, z_f0+ft)))         # floor tongue past x = 0
    fq = fq.union(cyl_z(z_f0+ft-0.01, zg1-12.0, fi, fi-5.0).intersect(bx(-lap, 0.0, 0.0, fr, z_f0, zg1)))   # wall tab, inside the next wall
    fq = fq.cut(cyl_z(z_f0-1, z_f0+ft+1, 18.0))                                                          # cable hole in the middle
    for cx_, cy_ in sym4([(-lap/2, 70.0), (-lap/2, 130.0)]):                                             # floor lap screws
        fq = fq.cut(cyl_z(zfh, z_f0+ft+1, R_M3, cx=cx_, cy=cy_)).cut(cyl_z(z_f0+0.8, zfh+0.1, R_INS3, cx=cx_, cy=cy_))
    for k in range(4):                                                                                   # wall tab screws (radial)
        for zz in (z_f0 + 30.0, z_f0 + 80.0):
            fq = fq.cut(cyl_y(fi-0.2, fr+1.0, R_M3, -lap/2, zz).rotate(O3, ZV, 90*k)).cut(cyl_y(fi-5.5, fi+0.2, R_INS3, -lap/2, zz).rotate(O3, ZV, 90*k))
    wz0, wz1, ww = z_f0 + 14.0, z_f0 + 70.0, 35.0                         # window, in the local frame of azimuth 45
    win = bx(fi-12, fr+2, -ww, ww, wz0, wz1)
    for sy_ in (-1, 1):
        for zz in (wz0 - 3.0, wz1 + 3.0): win = win.union(cyl_x(fi-1, fr+1, R_M3).translate((0, sy_*(ww+3.0), zz)))   # panel screws
    fq = fq.cut(win.rotate(O3, ZV, 45.0))
    deck_pts = [(118.0*math.cos(math.radians(a_)), 118.0*math.sin(math.radians(a_))) for a_ in (22.5, 67.5)]
    for cx_, cy_ in deck_pts:
        fq = fq.union(cyl_z(z_f0+ft-0.1, zg0-10.0, 7.0, cx=cx_, cy=cy_)).cut(cyl_z(zg0-18.0, zg0-9.0, R_INS3, cx=cx_, cy=cy_))
    rr = p['roller_r']                                                    # rim roller: 625 on a radial M5 in two slotted posts
    posts = bx(rr-9.5, rr-3.5, -7.0, 7.0, zg1-24.0, zg1-3.0).union(bx(rr+3.5, fi+0.5, -7.0, 7.0, zg1-24.0, zg1-3.0)).union(bx(rr-9.5, fi+0.5, -7.0, 7.0, zg1-24.0, zg1-18.0))
    posts = posts.union(prism_xz([(rr-9.5, zg1-23.99), (fi+0.5, zg1-23.99), (fi+0.5, zg1-30.0)], -7.0, 7.0))
    posts = posts.cut(prism_yz(_hull(_circ(0.0, zg1-8.7, R_M5) + _circ(0.0, zg1-7.3, R_M5)), rr-10.0, fi+1.0))
    posts = posts.cut(bx(rr-3.4, rr+3.4, -9.0, 9.0, zg1-17.9, zg1+1))     # room for the roller
    fq = fq.union(posts.rotate(O3, ZV, 45.0))
    lug_pts = sym4([(90.0, 22.0), (90.0, -22.0)])
    for cx_, cy_ in lug_pts: fq = fq.cut(cyl_z(z_f0-1, z_f0+ft+1, R_M5, cx=cx_, cy=cy_))
    fq = fq.intersect(cyl_z(z_f0-1, zg1, fr))
    for k in range(4): parts[f'foot_housing_q{k+1}'] = fq.rotate(O3, ZV, 90*k).val()
    for k, az in enumerate((45.0, 135.0, 225.0, 315.0), start=1):
        parts[f'foot_brg_roller_{k}_625'] = cyl_x(rr-2.5, rr+2.5, 8.0, 2.5).translate((0, 0, zg1-8.0)).rotate(O3, ZV, az).val()
        parts[f'foot_roller_axle_{k}_steel_M5x25'] = cyl_x(rr-9.0, rr+11.0, 2.5).translate((0, 0, zg1-8.0)).rotate(O3, ZV, az).val()
    # --- the four panels in the windows (all different): front = e-stop + display, I/O, vents, fan
    wc = (wz0 + wz1)/2
    def panel(cuts_, az_):
        pl_ = cyl_z(wz0+0.2, wz1-0.2, fr, fi+0.2).intersect(bx(0.0, fr+1, -ww+0.2, ww-0.2, wz0, wz1))
        pl_ = pl_.union(cyl_z(wz0-7.0, wz1+7.0, fi, fi-2.5).intersect(bx(0.0, fi+1, -ww-7.0, ww+7.0, wz0-8, wz1+8)))   # flange inside the wall
        for sy_ in (-1, 1):
            for zz in (wz0 - 3.0, wz1 + 3.0): pl_ = pl_.cut(cyl_x(fi-3.0, fi+0.1, R_INS3).translate((0, sy_*(ww+3.0), zz)))
        for c_ in cuts_: pl_ = pl_.cut(c_)
        return pl_.rotate(O3, ZV, az_).val()
    dzc = wc - 16.0
    parts['foot_panel_front'] = panel([cyl_x(fi-8, fr+2, 11.2).translate((0, 0, wc+14.0)), bx(fi-8, fr+2, -15.5, 15.5, dzc-7.5, dzc+7.5)], 45.0)
    parts['foot_panel_io'] = panel([cyl_x(fi-3, fr+2, 6.0).translate((0, -18.0, wc-12.0)), bx(fi-3, fr+2, -5.0, 5.0, wc+14.0, wc+18.0),
                                     bx(fi-3, fr+2, 6.0, 26.0, wc-18.0, wc-5.0)], 135.0)
    parts['foot_panel_vent'] = panel([bx(fi-3, fr+2, -26.0+k*10.0, -22.0+k*10.0, wz0+8.0, wz1-8.0) for k in range(6)], 225.0)
    rf_ = (p['fan']/2-3.0)*0.7071
    parts['foot_panel_fan'] = panel([cyl_x(fi-3, fr+2, p['fan']/2-3.0).translate((0, 0, wc)),
                                      prism_yz([(-rf_, wc+rf_), (0.0, wc+2*rf_), (rf_, wc+rf_)], fi-3, fr+2)], 315.0)
    # display (TM1637, 0.36") behind the front panel, e-stop in it, fan behind its panel
    A45 = lambda s_: s_.rotate(O3, ZV, 45.0)
    parts['elec_display_7seg_pcb'] = A45(bx(fi-6.0, fi-4.5, -21.0, 21.0, dzc-10.0, dzc+10.0)).val()
    parts['elec_display_7seg_body'] = A45(bx(fi-4.5, fr-0.6, -15.0, 15.0, dzc-7.0, dzc+7.0)).val()
    SEG = dict(a=(-2.0, 2.0, 4.0, 5.0), d=(-2.0, 2.0, -5.0, -4.0), g=(-2.0, 2.0, -0.5, 0.5),
               f=(-2.8, -1.8, 0.4, 4.2), b=(1.8, 2.8, 0.4, 4.2), e=(-2.8, -1.8, -4.2, -0.4), c=(1.8, 2.8, -4.2, -0.4))
    for d_ in range(4):
        uc = (d_ - 1.5)*7.0
        for sn, (a0, a1, b0, b1) in SEG.items():
            parts[f'elec_seg_{d_}_{sn}'] = A45(bx(fr-0.61, fr-0.3, uc+a0, uc+a1, dzc+b0, dzc+b1)).val()
    parts['elec_estop_button'] = A45(cyl_x(fr, fr+14.0, p['estop_d']/2).union(cyl_x(fr-18.0, fr, 11.0)).translate((0, 0, wc+14.0))).val()
    fn = p['fan']
    parts['elec_fan_40mm'] = bx(fi-18.0, fi-5.0, -fn/2, fn/2, wc-fn/2, wc+fn/2).rotate(O3, ZV, 315.0).val()
    # desk lugs (4x, the same part) on the quadrant joints: wide pad under the foot rim, curved web against the wall, centre
    # rib; M6 (or a desk clamp) at the end, 2 x M5 into the floors of the two quadrants it joins
    lt, Rf = 8.0, 14.0; xl_ = fr + 37.0
    for i, az in enumerate((0.0, 90.0, 180.0, 270.0), start=1):
        lug = bx(70.0, fr+6.0, -34.0, 34.0, z_f0-lt, z_f0).union(bx(fr, xl_, -20.0, 20.0, z_f0-lt, z_f0)).union(cyl_z(z_f0-lt, z_f0, 20.0, cx=xl_))
        for sy in (-1, 1):
            fil = bx(fr+6.0-0.01, fr+6.0+Rf, sy*20.0, sy*(20.0+Rf), z_f0-lt, z_f0).cut(cyl_z(z_f0-lt-1, z_f0+1, Rf, cx=fr+6.0+Rf, cy=sy*(20.0+Rf)))
            lug = lug.union(fil)
        web = cyl_z(z_f0-0.01, z_f0+36.0, fr+6.0, fr+0.3).intersect(bx(fr-2, fr+8, -20.0, 20.0, z_f0-1, z_f0+40))
        rib = prism_xz([(fr+5.0, z_f0-0.01), (fr+5.0, z_f0+34.0), (fr+30.0, z_f0-0.01)], -3.0, 3.0)
        lug = lug.union(web).union(rib)
        lug = lug.cut(cyl_z(z_f0-lt-1, z_f0+1, 3.3, cx=xl_)).cut(cyl_z(z_f0-3.0, z_f0+1, 5.6, cx=xl_))
        for yy in (-22.0, 22.0):
            lug = lug.cut(cyl_z(z_f0-lt-1, z_f0+1, R_M5, cx=90.0, cy=yy)).cut(cyl_z(z_f0-lt-1, z_f0-lt+4.0, 4.7, cx=90.0, cy=yy))   # M5 + nut trap
        parts[f'foot_desk_lug_{i}'] = lug.rotate(O3, ZV, az).val()
    # deck (one piece, Ø248): carries the yaw cartridge, the yaw motor and the home sensor; screwed on the 8 pilasters
    dr = p['deck_r']
    deck = cyl_z(zg0-10.0, zg0-4.0, dr).union(cyl_z(zg0-4.01, T-5.5, 49.5)).cut(cyl_z(zg0-11, T, r_ct+0.2))
    deck = deck.cut(cyl_z(zg0-11, zg0+13, 11.5, cy=-a_yaw).union(bx(-11.5, 11.5, -a_yaw-1.0, -a_yaw+1.0, zg0-11, zg0+13)))
    for sx_ in (-15.5, 15.5):
        for sy_ in (-15.5, 15.5):                                           # gearbox face, 4x M3 in slots: push the pinion into the ring, then tighten
            deck = deck.cut(prism_xy_slot(sx_, -a_yaw+sy_, R_M3, 1.0, zg0-11, zg0-3))
    for cx_, cy_ in sym4(deck_pts): deck = deck.cut(cyl_z(zg0-11, zg0-3, R_M3, cx=cx_, cy=cy_))
    for k in range(4):
        a = math.pi/4 + k*math.pi/2
        deck = deck.cut(cyl_z(T-13.0, T-5.0, R_INS3, cx=45.0*math.cos(a), cy=45.0*math.sin(a)))  # cartridge flange, 4x M3
    deck = deck.union(bx(rmag-4.0, rmag+3.0, -4.0, 4.0, zg0-4.01, zg0+4.0).rotate(O3, ZV, -60.0))  # Hall sensor post
    for cx_, cy_ in ((-70.0, 40.0), (70.0, 40.0)): deck = deck.cut(cyl_z(zg0-11, zg0-3, 9.0, cx=cx_, cy=cy_))   # cable passes
    parts['foot_deck'] = deck.val()
    q17 = p['n17_sq']
    gq, gl_, _ = p['yaw_gb']
    parts['foot_yaw_gearbox_planetary_5'] = bx(-gq/2, gq/2, -a_yaw-gq/2, -a_yaw+gq/2, zg0-10.0-gl_, zg0-10.0).val()
    parts['foot_yaw_motor_NEMA17'] = bx(-q17/2, q17/2, -a_yaw-q17/2, -a_yaw+q17/2, zg0-10.0-gl_-p['yaw_motor_len'], zg0-10.0-gl_).val()
    parts['foot_yaw_motor_shaft'] = cyl_z(zg0-10.0, min(zg1-0.5, zg0-10.0+p['yaw_shaft'][1]), p['yaw_shaft'][0]/2, cy=-a_yaw).val()
    cw, cd, chh = p['ctrl']; zb = z_f0 + ft + 2.0
    parts['elec_controller_esp32s3_6xTMC2209'] = bx(-cw/2, cw/2, 20.0, 20.0+cd, zb, zb+chh).val()
    bw_, bd_, bh_ = p['buck']
    parts['elec_buck_24v_5v'] = bx(-100.0, -100.0+bw_, -40.0, -40.0+bd_, zb, zb+bh_).val()

    # ================= CABLES (rigid in the viewer; they cross every joint near its axis or inside the mechanism) =========
    # up through the yaw spindle, along the channel between the two shoulder motors, through the right elbow motor bracket
    # into the arm (10-17 mm from the shoulder axis), then up the middle to the elbow.
    xcc_ = xch0 + 7.0                                                    # cable channel in the right cheek, then into the stub from below
    xk_ = x_mf + p['el_plate'] + 4.8
    # v0.41: up the open channel of the shoulder motor block, between the two NEMA23, into the bottom window of the right elbow
    # motor bracket, up inside it beside the coupling and out of its top window (the cheeks and stubs carry no cable now)
    parts['yaw_cable_base'] = cable_poly([(0.0, 0.0, z_tt-9.0), (0.0, 0.0, z_tt+4.0), (xk_, 0.0, z_tt+4.0), (xk_, -1.0, z_tt+10.0), (xk_, -1.0, -31.0),
                                          (xk_, -14.0, -26.0), (xk_, -14.0, 10.0), (xk_, -6.0, 16.0), (xk_, 0.0, 17.0)], 3.5)
    # v0.40: out of the window in the top of the right elbow motor bracket (10-16 mm from the axis), up the arm beside the
    # motor and over to the middle, below the intermediate shafts, then up the middle to the elbow
    parts['arm_cable_main'] = cable_poly([(xk_, 0.0, 17.0), (xk_, 0.0, 44.0), (14.0, 0.0, 50.0), (10.0, 0.0, ze-52.0),
                                          (10.0, -30.0, ze-50.0), (30.0, -38.0, ze-47.0), (10.0, -27.0, ze-44.0)], 3.5)

    # ================= WRIST + HAND (existing design, ghost only) =================
    for n, s in wp.items():                     # complete wrist + hand, kept in wrist coordinates (apex at the origin)
        parts['W_' + n] = s

    info = dict(drive={k_: dict(C=v_['C'], z=v_['z'], dL=v_['dL']) for k_, v_ in DRV.items()}, dL_tension=dL, idl_c=idl_c, a_off=a_off, z_bp1=z_bp1, ze=ze, Lu=Lu, z_m=z_m, C=C, hg=hg, Rh=Rh, xc=xc, u0=u0, u1=u1, wall_out=wall_out,
                ratio_belt=p['t_elb']/p['t_motor'], kp=p['z_out']/p['z_side'], ratio_cyc=S_['z_gear']/S_['z_pin'], ratio_sh=S_['z_gear']/S_['z_pin'], sh_motor_z=zmot,
                ratio_yaw=p['yaw_ring']/p['yaw_pin'], ratio_yaw_motor=p['yaw_ring']/p['yaw_pin']*p['yaw_gb'][2], yaw_pin_xy=(0.0, -a_yaw), Lf=p['Lf'],
                shoulder_height=-z_f0, foot_d=2*fr, tt_d=2*p['tt_r'], wrist_tip=ze+p['Lf']+winfo['grip_top'],
                width_shoulder=2*(x_o+mw/2), width_arm=2*u1, z_mid=z_i, belt1_teeth=S1['teeth'], x_stage1=x_o, wrist=winfo, wrist_z=ze+p['Lf'],
                q_shoulder=S_['q_range'], belt_teeth=p['belt_teeth'], el_motor_z=0.0, belt_d0=DRV['A']['d0'], light_holes=len(light_), slots=[(tuple(round(v,1) for v in a), tuple(round(v,1) for v in b), w) for a, b, w in light_], teeth_arc=(ta0, ta1), ratio_sh_total=S_['z_gear']/S_['z_pin']*S_['gb_ratio'], ring_od=2*ring.BoundingBox().xmax)
    return parts, info

GUNMETAL, GUNMETAL_GEAR, STEEL, MOTOR_DARK = '#222428', '#33373d', '#c9cdd3', '#3a3d42'   # matte black PLA, brushed steel
GUN_D, GUN_L, PCB_DARK = '#1a1c1f', '#2a2d32', '#1f2226'
def tone(n):
    if n.startswith(('arm_humerus', 'foot_housing', 'foot_panel', 'foot_desk_lug', 'yaw_shoulder_motor_block', 'yaw_n23_clamp', 'foot_motor_shelf',
                     'foot_cable_tube')): return GUN_D
    if n.startswith(('cradle_plate', 'cradle_wall', 'arm_tensioner', 'arm_encoder_flex_bracket', 'arm_spacer')): return GUN_L
    return GUNMETAL

def material(n):
    if n.startswith('W_'):
        import wrist_v0 as W
        return W.material(n[2:])
    if ('steel' in n or 'screw' in n or 'nut_M5' in n or 'brg' in n or n.endswith('_shaft') or 'mpulley' in n or 'idler' in n or 'mid_pulley' in n
            or n.startswith(('cradle_spacer', 'arm_spacer_idler'))) and 'gearbox' not in n: return 'steel'
    if n.startswith(('elbow_side', 'idle_satellite', 'forearm_bevel_hub', 'yaw_ring_gear', 'foot_yaw_pinion')) or 'cycloid_disc' in n:
        return 'gear'
    if n.startswith(('arm_spacer_bolt',)) or n.endswith('_alu'): return 'alu'
    if n.startswith(('arm_motor_carrier', 'arm_shaft_block', 'arm_cover', 'cradle_plate', 'cradle_wall', 'cradle_spider', 'forearm_base_plate', 'yaw_turntable',
                     'yaw_shoulder_motor_block', 'yaw_pinion_support', 'yaw_elbow_bracket', 'yaw_cheek_foot', 'yaw_elbow_bushing', 'arm_mid_hub',
                     'foot_housing', 'foot_panel', 'foot_desk_lug', 'foot_deck', 'foot_yaw_cartridge', 'foot_yaw_retainer', 'foot_yaw_spacer', 'yaw_spacer_inner', 'yaw_spindle_clamp',
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
    if n.startswith('arm_humerus') or (n.startswith('yaw_cheek') and n.endswith('_steel')): return '#b4b9c0'   # waterjet-cut steel bones and cheeks
    if 'shaft' in n or 'steel' in n or 'screw' in n: return '#8a8f98'
    # "war machine" palette: gunmetal printed parts, brass accents, carbon gears; bones (aluminium) and motors as they are
    if 'gearbox' in n: return '#7a5a2e'                       # bought steel planetary gearboxes: dark bronze
    if n.endswith('_alu'): return '#d3d7dc'                                # machined aluminium
    if 'gear' in n or 'pinion' in n or 'elbow_side' in n or 'forearm_bevel' in n or 'satellite' in n: return '#17181a'   # PETG-CF / PA-CF gears
    if 'pulley' in n or 'idler' in n or 'spacer' in n or 'tensioner' in n: return '#3a3d44'
    if n.startswith(('yaw_pinion_support', 'arm_encoder', 'forearm_base')): return BRASS
    if n.startswith('arm_cover'): return GUN_3
    if n.startswith(('arm_motor_carrier', 'arm_shaft_block', 'yaw_turntable', 'foot_desk_lug', 'cradle_spider', 'yaw_shoulder_motor_block')): return GUN_2
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
    if 'yaw_shoulder' in n or 'yaw_gearbox_shoulder' in n or 'NEMA23' in n or 'arm_shoulder' in n or 'arm_brg_shoulder' in n or n.startswith(('yaw_cheek', 'yaw_pinion', 'yaw_brg_pinion', 'arm_gear_sector')): return 'shoulder'
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
    if n.startswith('yaw_shoulder_pinion_A'): return 'shpinA'
    if n.startswith('yaw_shoulder_pinion_B'): return 'shpinB'
    if n.startswith(('foot', 'elec_')): return 'foot'
    if n.startswith('yaw_cable'): return 'yaw'
    if n.startswith(('elbow_side_A', 'elbow_brg_A', 'elbow_pulley_A')): return 'sideA'
    if n.startswith(('elbow_side_B', 'elbow_brg_B', 'elbow_pulley_B')): return 'sideB'
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
        elif g in ('shpinA', 'shpinB'):                    # the pinions turn 13x the shoulder, opposite to the sectors
            yp_ = P['sh']['y']*(1 if g == 'shpinA' else -1); zp_ = info['sh_motor_z']
            out[n] = s.rotate((0, yp_, zp_), (1, yp_, zp_), sigma*info['ratio_sh']).rotate(O, Z, psi)
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
