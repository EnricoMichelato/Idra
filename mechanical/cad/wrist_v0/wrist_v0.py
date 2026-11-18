"""
Wireless Robot Arm - wrist + hand (v1.24: steel roll shaft, bones bolted to the forearm base, stand-offs printed with the bones, idler tensioners = M5 + nut in a slot; v1.23: passive hand, no MCU (frequency-commanded DRV8871); v1.22: rounded flange, chunky cross; v1.21: matte black PLA, flush idle bevel on a pin, roll encoder at the roll-shaft end; v1.20: one fit file, 12 mm pitch axle on 6801, 688 idler like the elbow; v1.16: round lightening holes; v1.15: cables; v1.14: one back-side tensioner per belt, flange ears wrap the bearing ends; v1.13: wider flange over the hand, one tapered slot per plate; v1.12: spiral bevels, idle 4th bevel, AS5600 on pitch and roll,
axle turning with the flange, side-gear preload, pot-core style inductive link raised above the 6810, TX driver on the flange;
10 mm ulna/radius, ribbed covers, NEMA17 wrist motors near the elbow, Fin Ray hand v2.2, XIAO camera)
Parametric CadQuery generator. Run:  python3 wrist_v0.py   (build, interference check, export to ./out)
Frame: origin = apex of the three bevel gears (crossing of pitch and roll axes)
  X = pitch axis, Z = roll axis pointing to the hand, forearm is toward -Z, units mm.
"""
import os, itertools, math
import cadquery as cq
from cq_gears import BevelGearPair
from hand_v2 import build_hand, H as HAND
import sys as _sys
_sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'common'))
from fit import FIT, seat as fseat, boss as fboss, R_INS3, R_INS5, R_M3, R_M5, R_TAP3

P = dict(
    motor_gap=1.0,                                           # gap between the two back-to-back NEMA17 motors
    roll_shaft_d=12.0, rbo=21.0, rbw=5.0,                    # roll shaft 12 mm on 6801 bearings (12x21x5)
    ring_id=50.0, ring_od=65.0, ring_w=7.0, ring_shoulder=0.6,   # 6810 ring bearing around the coils
    motor_shaft_len=20.0,
    # bevel differential (1:1), spiral (helix) 20 deg; side gears A/B are the same part
    module=2.0, teeth=24, face=11.0, pressure=20.0, helix=20.0, backlash=0.15,
    # idle 4th bevel D opposite the output C: on 2x 686 (6x13x5) on a 6 x 1 steel tube pressed in the T fitting
    idle_brg=(16.0, 8.0, 5.0), idle_pin=(8.0, 0.0),          # 2x 688 on a solid Ø8 steel pin (same bearing as the elbow idler)
    # axial preload of the side gears (as in the elbow): shim on the pulley-end 688 inner ring + 8x12 wave spring
    pre_shim=(15.2, 0.8), pre_spring=(18.0, 2.2),             # same shim / 12x18 wave spring as the elbow
    shaft_d=12.0, bore_d=12.2, hub_d=20.0, hub_len=5.0,      # pitch axle 12 mm like the elbow pin
    # 688ZZ bearings on the static axle
    brg_od=21.0, brg_id=12.0, brg_w=5.0,                    # 6801 everywhere on the pitch axle (was 688)
    # carrier (flange), outside ulna and radius
    wall_t=8.0, wall_gap=1.0, carrier_y=76.0, plate_t=10.0, wall_down=14.0,     # 76 wide: covers the Ø70 hand
    ear_r=22.0, car_fillet=8.0,                              # ear wraps the bearing end of ulna/radius (R25)                              # flange walls: round 'ear' around the bearing + R8 inner fillet
    # eye-in-hand camera: Seeed XIAO ESP32S3 Sense (21 x 17.5 x 15 mm stack, OV3660/OV2640), WiFi stream, 5 V from the forearm
    cam_x=54.0, cam_tilt=15.0, cam_board=(21.0, 17.5, 11.0),
    pulley_gap=2.5,                                          # room for the wave spring between pulley and the 6 mm plate
    # ulna / radius plates, belts, motors
    yoke_t=6.0, yoke_len=150.0, belt_teeth=170, cover=(90.0, 4.0),       # long ulna/radius down to the forearm base on the elbow
    belt_w=10.0, belt_t=1.5, bore_clear=12.6, motor_sq=42.3, motor_len=40.0, motor_shaft_d=5.0, motor_hole=31.0,   # NEMA17 (recycled)
   
    mp_od=8.4, mp_flange_od=13.0, mp_w=12.0,                 # 14T GT2 motor pulley (78/14 = 5.57)
    # one adjustable tensioner per belt: smooth idler on the belt back, outside the loop, on the front strand
    tens_r=6.0, tens_z=-92.0, tens_d0=5.0, tens_adj=2.5,
    # inductive link: PCB coils (electronics/coil_pcb: 4-layer ring boards, Ø43.6 / Ø15.6 x 1.6) face to face, 1 mm gap,
    # each backed by 2-3 rings of flexible ferrite sheet (0.6 mm) on 0.2 mm adhesive, in a pocket of the flange boss and of
    # the hand bottom; raised 3 mm above the 6810 so the steel bearing is out of the field.
    # Pads at +-20 deg from the board reference; TX pads point to the driver (180 deg), RX pads to the wire hole (90 deg).
    coil_od=43.6, coil_id=15.6, coil_t=1.6, ferrite_od=44.0, ferrite_id=15.0,
    ferrite_t=0.6, glue_t=0.2, coil_gap=1.0, coil_lift=3.0, pad_r=20.85, tx_pad_az=180.0, rx_pad_az=110.0, tx_board=(30.0, 20.0, 9.0),
    pitch_deg=0.0,
)

O3 = (0, 0, 0)

def bx(x0, x1, y0, y1, z0, z1):
    return cq.Workplane('XY').box(x1-x0, y1-y0, z1-z0, centered=False).translate((x0, y0, z0))
def cyl_x(x0, x1, r, ri=0.0):
    w = cq.Workplane('YZ').workplane(offset=x0).circle(r)
    if ri: w = w.circle(ri)
    return w.extrude(x1-x0)
def prism_yz(pts, x0, x1):
    x0, x1 = sorted((x0, x1))
    return cq.Workplane('YZ').workplane(offset=x0).polyline(pts).close().extrude(x1-x0)
def prism_xz(pts, y0, y1):
    y0, y1 = sorted((y0, y1))
    return cq.Workplane('XZ').workplane(offset=-y1).polyline(pts).close().extrude(y1-y0)
def cyl_z(z0, z1, r, ri=0.0, cx=0.0, cy=0.0):
    w = cq.Workplane('XY').workplane(offset=z0).center(cx, cy).circle(r)
    if ri: w = w.circle(ri)
    return w.extrude(z1-z0)

def _circ_xz(cx, cz, r, n=64):
    return [(cx + r*math.cos(2*math.pi*k/n), cz + r*math.sin(2*math.pi*k/n)) for k in range(n)]

def pulley_gen(x0, r_body, r_fl, w, r_bore, fl=1.0):
    pu = cyl_x(x0, x0+fl, r_fl).union(cyl_x(x0+fl, x0+w-fl, r_body)).union(cyl_x(x0+w-fl, x0+w, r_fl))
    return pu.cut(cyl_x(x0-1, x0+w+1, r_bore))
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
def _circ(cy, cz, r, n=72):
    return [(cy+r*math.cos(2*math.pi*k/n), cz+r*math.sin(2*math.pi*k/n)) for k in range(n)]
def belt_loop(xc, r1, r2, cd, w, t, ym=0.0):
    outer = _hull(_circ(0, 0, r1+t) + _circ(ym, -cd, r2+t))
    inner = _hull(_circ(0, 0, r1) + _circ(ym, -cd, r2))
    mk = lambda pts: cq.Workplane('YZ').workplane(offset=xc-w/2).polyline(pts).close().extrude(w)
    return mk(outer).cut(mk(inner))

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
def cyl_dir(p0, d, h, r):
    return cq.Workplane('XY').newObject([cq.Solid.makeCylinder(r, h, cq.Vector(*p0), cq.Vector(*d))])

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
    """round the corners of a flange seen from above, keeping the full ring around the pitch axis (X)"""
    keep = (cq.Workplane('XY').workplane(offset=z0).rect(2*hx, 2*hy).extrude(z1 - z0).edges('|Z').fillet(R)
            .union(cq.Workplane('YZ').workplane(offset=-hx-2).circle(axis_r).extrude(2*hx+4)))
    return solid.intersect(keep)

def build(p=P):
    parts = {}
    # ---------- bevel gears ----------
    pair = BevelGearPair(p['module'], p['teeth'], p['teeth'], p['face'], axis_angle=90.0,
                         pressure_angle=p['pressure'], helix_angle=p['helix'], backlash=p['backlash'])
    asm = pair.assemble(gear_build_args={'bore_d': p['bore_d']}, pinion_build_args={'bore_d': p['bore_clear']})
    kids = {c.name: c for c in asm.children}
    h = float(pair.gear.cone_h)                    # apex distance of the heel back plane
    C0 = kids['gear'].obj.moved(kids['gear'].loc)
    A0 = kids['pinion'].obj.moved(kids['pinion'].loc)
    B0 = A0.rotate(cq.Vector(0, 0, 0), cq.Vector(0, 0, 1), 180)
    def to_frame(s):                              # apex -> origin, output gear -> +Z
        return s.translate(cq.Vector(0, 0, -h)).rotate(cq.Vector(0, 0, 0), cq.Vector(1, 0, 0), 180)
    C, A, B = to_frame(C0), to_frame(A0), to_frame(B0)
    Rh = A.BoundingBox().ymax                      # outer radius of the bevel at its heel
    rb_p, rf_p = Rh - 1.0, Rh                      # belt pulley body and flange as big as the bevel
    Dp, dp = 2*rb_p + 0.76, 8.91                   # GT2 pitch diameters (big pulley, 14T motor pulley)
    Nb = p['belt_teeth']; Lb = 2.0*Nb               # GT2 10 mm closed loop, whole number of teeth (210 -> 420 mm)
    q_ = Lb - math.pi*(Dp+dp)/2
    Cb = (q_ + math.sqrt(q_**2 - 2*(Dp-dp)**2))/4  # centre distance for an exact whole number of belt teeth (no idler)
    # with the back-side tensioner pushing the front strand in by d0, solve the centre distance again
    r1p_, r2p_, reff_ = Dp/2, dp/2, p['tens_r'] + BT_OUT
    def tens_c(C_, d0):
        a_ = math.asin((r1p_ - r2p_)/C_); ny_, nz_ = math.cos(a_), -math.sin(a_)
        p1 = (r1p_*ny_, r1p_*nz_); p2 = (r2p_*ny_, -C_ + r2p_*nz_)
        t_ = (p['tens_z'] - p1[1])/(p2[1] - p1[1]); fy = p1[0] + t_*(p2[0] - p1[0])
        return (fy + ny_*(reff_ - d0), p['tens_z'] + nz_*(reff_ - d0)), (ny_, nz_)
    def bcirc(C_, d0):
        (ty, tz), _ = tens_c(C_, d0)
        return [(0, 0, r1p_, 1), (0, -C_, r2p_, 1), (ty, tz, reff_, -1)]
    lo_, hi_ = 80.0, 160.0
    for _ in range(40):
        mid_ = (lo_ + hi_)/2
        if belt_path(bcirc(mid_, p['tens_d0']))[0] > Lb: hi_ = mid_
        else: lo_ = mid_
    Cb = mid_
    tens_cn, tens_n = tens_c(Cb, p['tens_d0'])
    dL_t = [belt_path(bcirc(Cb, p['tens_d0'] + s_*p['tens_adj']))[0] - Lb for s_ in (-1, 1)]
    pulley_teeth = round(math.pi*Dp/2.0)
    hl, hd, bd = p['hub_len'], p['hub_d']/2, p['bore_d']/2
    def with_hub(shape, hub):
        return cq.Workplane('XY').newObject([shape]).union(hub, clean=False).val()
    gC = with_hub(C, cyl_z(h, h+hl, hd, bd))
    zc_lo = gC.BoundingBox().zmin
    dfill = bx(4.15, 7.0, -7.0, 7.0, zc_lo+0.01, h+hl).intersect(cyl_z(zc_lo, h+hl+1, bd+0.4))   # D bore: drives on a 2 mm flat of the shaft
    parts['gear_C_output'] = cq.Workplane('XY').newObject([gC]).union(dfill, clean=False).val()

    # ---------- derived positions ----------
    pw, g2 = p['pulley_w'] if 'pulley_w' in p else 0.0, p['pulley_gap']
    half_gap = p['motor_gap']/2
    sd, bo, bw, seat = p['shaft_d']/2, p['brg_od']/2, p['brg_w'], fseat(p['brg_od'])
    rsd, rbo, rbw = p['roll_shaft_d']/2, p['rbo']/2, p['rbw']; rseat = fseat(p['rbo'])
    xr = lambda a, b: sorted((a, b))
    xc = max(half_gap + p['motor_len'] + 2.0 + p['mp_w']/2, h + p['belt_w']/2 + 0.5)   # belt plane
    pul_in = h                                     # pulley starts right at the gear heel (no neck)
    pul_out = xc + p['mp_w']/2
    # EXOSKELETON layout: ulna and radius are the outermost parts. From the centre: bevel, pulley, flange wall, bone.
    wall_in = pul_out + 1.0; wall_out = wall_in + p['wall_t']   # flange walls INSIDE ulna and radius
    u0 = wall_out + p['wall_gap']; u1 = u0 + p['yoke_t']       # ulna / radius plates
    plate_z0 = h + hl
    plate_z1 = plate_z0 + p['plate_t']
    cy2 = p['carrier_y']/2

    # ---------- side gear + GT2 pulley printed as ONE part (A on -X, B on +X) ----------
    fl, rb, rf = 1.0, rb_p, rf_p
    xsp = p['pre_shim'][1] + p['pre_spring'][1] - (wall_in - pul_out)    # shim and wave spring sit this deep inside the pulley-end bore
    def side_part(shape, sg):
        bb = shape.BoundingBox()
        toe = min(abs(bb.xmin), abs(bb.xmax))
        pu = cyl_x(*xr(sg*pul_in, sg*(pul_out-fl)), rb).union(cyl_x(*xr(sg*(pul_out-fl), sg*pul_out), rf))
        body = cq.Workplane('XY').newObject([shape]).union(pu, clean=False)
        body = body.cut(cyl_x(-80, 80, p['bore_clear']/2), clean=False)
        body = body.cut(cyl_x(*xr(sg*(toe-0.2), sg*(toe+bw+0.2)), seat), clean=False)
        body = body.cut(cyl_x(*xr(sg*(pul_out-xsp-bw-0.2), sg*(pul_out+0.2)), seat), clean=False)
        return body.val(), toe
    gA, toe = side_part(A, -1)
    gB, _ = side_part(B, 1)
    parts['gear_A_pulley_one_piece'] = gA
    parts['gear_B_pulley_one_piece'] = gB

    # ---------- single axle: clamped in the flange walls (M3 grubs), turns in a 688 in ulna and radius ----------
    x_pe = u1 + 1.2
    parts['pitch_axle_steel'] = cyl_x(-x_pe, x_pe, sd).val()
    shim_od, shim_t = p['pre_shim']; spr_od = p['pre_spring'][0]
    for nm, sg in (('L', -1), ('R', 1)):
        parts[f'brg_yoke_{nm}_688'] = cyl_x(*xr(sg*(u0+0.5), sg*(u0+0.5+bw)), bo, sd).val()          # in the 6 mm aluminium plate
        parts[f'carrier_spacer_{nm}'] = cyl_x(*xr(sg*wall_out, sg*(u0+0.5)), 7.4, sd+0.05).val()
        parts[f'pre_{nm}_shim_steel'] = cyl_x(*xr(sg*(pul_out-xsp), sg*(pul_out-xsp+shim_t)), shim_od/2, sd+0.1).val()
        parts[f'pre_{nm}_wave_spring_steel'] = cyl_x(*xr(sg*(pul_out-xsp+shim_t), sg*wall_in), spr_od/2, sd+0.1).val()
    # pitch encoder: diametric magnet 6 x 2.5 glued on the +X axle end, AS5600 on a bracket of the radius that comes
    # from the forearm side (the flange never sweeps there within +-90 deg of pitch)
    parts['pitch_magnet_6x2.5'] = cyl_x(x_pe, x_pe+2.5, 3.0).val()
    xc0 = x_pe + 3.5; xc1 = xc0 + 1.2; xp1 = xc1 + 1.6; xb1 = xp1 + 3.0
    c1, c2 = (0.0, -33.0), (0.0, -41.0)
    brk = prism_yz(_hull(_circ(0, 0, 10.0) + _circ(*c2, 6.5)), xp1, xb1).union(prism_yz(_hull(_circ(*c1, 5.5) + _circ(*c2, 5.5)), u1, xp1+0.01))
    for cc in (c1, c2):
        brk = brk.cut(cyl_x(u1-1, xb1+1, R_M3).translate((0,) + cc)).cut(cyl_x(xb1-2.0, xb1+1, 3.1).translate((0,) + cc))
    parts['enc_pitch_bracket'] = brk.val()
    parts['enc_pitch_as5600_pcb'] = bx(xc1, xp1, -8.0, 8.0, -8.0, 8.0).val()
    parts['enc_pitch_as5600_chip'] = bx(xc0, xc1, -2.5, 2.5, -3.0, 3.0).val()

    # ---------- T fitting between the grey gears: floats on the axle, carries the lower end of the roll shaft ----------
    toe_c = parts['gear_C_output'].BoundingBox().zmin
    z_T1 = toe_c - 0.6
    bar = toe - 0.8
    ipo, ipi = p['idle_pin'][0]/2, p['idle_pin'][1]/2
    Tf = cyl_x(-bar, bar, 10.0).union(cyl_z(0.0, z_T1, 14.0)).union(cyl_z(-z_T1, 0.0, 10.5))
    Tf = Tf.union(bx(-bar, bar, -16.0, 16.0, -z_T1, z_T1).edges('|Z').fillet(5.0))   # chunky cross block
    Tf = Tf.cut(cyl_x(-bar-1, bar+1, sd)).cut(cyl_z(z_T1-rbw-0.2, z_T1+1, rseat))
    Tf = Tf.cut(cyl_z(-z_T1-1, -sd-1.2, ipo))                                      # press fit of the idle pin
    Tf = Tf.cut(cyl_z(sd-0.5, z_T1-rbw+0.01, 6.4)).cut(bx(-2.0, 2.0, 0.0, 17.0, sd+0.4, sd+3.0))   # roll-encoder PCB + wires (+Y)
    Tf = Tf.cut(cq.Workplane('XZ').workplane(offset=-17).center(10.0, 0).circle(R_TAP3).extrude(17-sd+0.5))   # M3 grub on the axle
    parts['T_fitting'] = Tf.val()
    # idle bevel D (axis -Z): same gear as C turned over, on 2x 686 on the steel tube; magnet in its closed hub,
    # AS5600 on a round 12 mm PCB on the tube end (wires inside the tube) -> absolute roll angle, endless
    ibo, ibi, ibw = p['idle_brg']
    D0 = cq.Workplane('XY').newObject([C]).union(cyl_z(h-0.01, toe_c+2*ibw+0.6, 9.5), clean=False)    # <1 mm lip: flush
    D0 = D0.cut(cyl_z(toe_c-1, toe_c+2*ibw+1.0, fseat(ibo)), clean=False).val()
    D = D0.rotate(cq.Vector(0, 0, 0), cq.Vector(1, 0, 0), 180)                       # toe at -toe_c, hub end at -(h+12)
    parts['idle_gear_D_24T'] = D
    parts['T_idle_pin_steel'] = cyl_z(-(toe_c + 2*ibw + 0.6), -sd-1.2, ipo).val()   # Ø8 pin pressed in the T, flush
    parts['T_brg_idle_688_1'] = cyl_z(-toe_c-ibw, -toe_c, ibo/2, ipo).val()
    parts['T_brg_idle_688_2'] = cyl_z(-toe_c-2*ibw, -toe_c-ibw, ibo/2, ipo).val()
    # roll encoder: magnet in the bottom end of the roll shaft (turns 1:1 with the hand), AS5600 on a Ø12 PCB lying on
    # top of the pitch axle inside the T stem -> no hub on the idle bevel any more
    zr0 = z_T1 - rbw
    parts['T_enc_roll_as5600_pcb'] = cyl_z(sd, sd+1.6, 6.0).val()
    parts['T_enc_roll_as5600_chip'] = bx(-2.5, 2.5, -3.0, 3.0, sd+1.6, sd+2.8).val()
    parts['roll_magnet_6x2.5'] = cyl_z(zr0, zr0+2.5, 3.0).val()
    parts['brg_T_stem'] = cyl_z(z_T1-rbw, z_T1, rbo, rsd).val()

    # ---------- carrier / flange: outside ulna and radius, pivots on the axle, boss inside the ring bearing ----------
    # THREE pieces that print flat: the plate (ring-bearing boss, coil pocket) and two walls (axle bore as a true circle)
    er = p['ear_r']; cxm = p['cam_x']
    car = bx(-wall_in, wall_in, -cy2, cy2, plate_z0, plate_z1)
    wall_scr = [(-(cy2-10.0), (plate_z0+plate_z1)/2), (cy2-10.0, (plate_z0+plate_z1)/2), (0.0, (plate_z0+plate_z1)/2)]
    wprof = _hull(_circ(0.0, 0.0, er) + [(cy2-6.0, plate_z0-6.0), (cy2-6.0, plate_z1), (-(cy2-6.0), plate_z1), (-(cy2-6.0), plate_z0-6.0)])
    for sg, nm in ((-1, 'L'), (1, 'R')):
        wl = prism_yz(wprof, *xr(sg*wall_in, sg*wall_out)).cut(cyl_x(*xr(sg*(wall_in-1), sg*(wall_out+1)), sd))
        wl = wl.cut(cq.Workplane('XZ').workplane(offset=-(er+1)).center(sg*(wall_in+wall_out)/2, 0).circle(R_TAP3).extrude(er+1-sd+0.5))   # M3 grub on the axle flat
        for yy_, zz_ in wall_scr:                                                          # 3 countersunk M3 x 12 into inserts in the plate
            wl = wl.cut(cyl_x(*xr(sg*(wall_in-1), sg*(wall_out+1)), R_M3).translate((0, yy_, zz_)))
            cs = cq.Workplane('XY').polyline([(wall_out-1.8, 0), (wall_out-1.8, R_M3), (wall_out+0.01, 3.3), (wall_out+0.01, 0)]).close().revolve(360, (0, 0, 0), (1, 0, 0))
            wl = wl.cut((cs if sg > 0 else cs.mirror('YZ', (0, 0, 0))).translate((0, yy_, zz_)))
            car = car.cut(cyl_x(*xr(sg*(wall_in-6.5), sg*(wall_in+1)), R_INS3).translate((0, yy_, zz_)))
        parts[f'carrier_wall_{nm}'] = wl.val()
    sh, rw, ri_ = p['ring_shoulder'], p['ring_w'], p['ring_id']/2
    bt = plate_z1 + sh + rw + p['coil_lift']       # top of the boss: coils 3 mm above the ring bearing
    car = car.union(cyl_z(plate_z1-0.01, bt, fboss(p['ring_id']))).union(cyl_z(plate_z1-0.01, plate_z1+sh, ri_+3.0))
    car = car.cut(cyl_z(plate_z0-1, bt+1, rsd+0.2)).cut(cyl_z(plate_z0, plate_z0+rbw, rseat))
    pocket = p['ferrite_t'] + p['glue_t'] + p['coil_t']                      # 2.4 mm: coil face flush with the boss top
    car = car.cut(cyl_z(bt-pocket, bt+1, p['ferrite_od']/2+0.1, ri=p['ferrite_id']/2-0.1))   # TX coil board + ferrite sheet flush in the boss
    for d_ in (-20.0, 20.0):                                                             # TX coil leads: one hole under each pad -> driver
        lx_, ly_ = (p['pad_r']+0.6)*math.cos(math.radians(p['tx_pad_az']+d_)), (p['pad_r']+0.6)*math.sin(math.radians(p['tx_pad_az']+d_))
        car = car.cut(cyl_z(plate_z0-1, bt-pocket+0.5, 1.3, cx=lx_, cy=ly_))
    for yy in (-7.0, 7.0):
        car = car.cut(cyl_z(plate_z1-6.0, plate_z1+1, 2.0, cx=-cxm, cy=yy))              # M3 inserts of the TX driver board
    for yy in (-15.0, 15.0):
        car = car.cut(cyl_z(plate_z1-6.0, plate_z1+1, 2.0, cx=cxm, cy=yy))        # M3 heat-set inserts for the camera mount
    # camera power (5 V from the TX driver board): a 3.5 mm groove in the top of the flange along the front edge,
    # the cable lies flush in it from the driver to the camera mount (no loose wire)
    cam_groove = [(-49.6, 11.6), (-41.0, cy2-3.6), (36.0, cy2-3.6), (39.8, 8.0)]
    for (ga, gb), (gc, gd) in zip(cam_groove[:-1], cam_groove[1:]):
        car = car.cut(cq.Workplane('XY').workplane(offset=plate_z1-3.0).polyline(_hull(_circ(ga, gb, 2.0, 24) + _circ(gc, gd, 2.0, 24))).close().extrude(5.0))
    car = round_plan(car, wall_in, cy2, 10.0, er+0.01, -er-5.0, bt+5.0)             # rounded corners seen from above
    parts['carrier_flange'] = car.val()

    # ---------- bearings ----------
    def brg_x(a, b): return cyl_x(a, b, bo, sd).val()
    parts['brg_roll_lower'] = cyl_z(plate_z0, plate_z0+rbw, rbo, rsd).val()
    parts['brg_ring_6810'] = cyl_z(plate_z1+sh, plate_z1+sh+rw, p['ring_od']/2-0.1, ri_).val()   # big thin ring bearing around the coils
    parts['brg_A_toe'] = brg_x(*xr(-toe, -(toe+bw)))
    parts['brg_A_pulley'] = brg_x(*xr(-(pul_out-xsp), -(pul_out-xsp-bw)))
    parts['brg_B_toe'] = brg_x(*xr(toe, toe+bw))
    parts['brg_B_pulley'] = brg_x(*xr(pul_out-xsp, pul_out-xsp-bw))

    # ---------- wrist motors back to back, clamped between ulna and radius by 4 through screws ----------
    sq = p['motor_sq']; zc_m = -Cb; sx = p['motor_hole']/2
    scr = [(sx, zc_m+sx), (-sx, zc_m+sx), (sx, zc_m-sx), (-sx, zc_m-sx)]
    mface = half_gap + p['motor_len']                                         # motors back to back; the shaft end runs in a bearing block on the bone
    def plate(x0, x1, sg):
        y = bx(x0, x1, -Rh, Rh, -p['yoke_len']-8.0, 18.0)       # the foot flanks the 8 mm forearm base                  # uniform width = pulley diameter
        y = y.cut(bx(x0-1, x1+1, -Rh-1, Rh+1, 0.0, 20.0).cut(cyl_x(x0-2, x1+2, Rh)))   # top rounded about the axle (flange fillet clears)
        y = y.cut(cyl_x(x0-2, x1+2, 11.0).translate((0, 0, -60.0)))       # round hole Ø22, >= 6 mm from every screw / slot
        if sg < 0:
            y = y.cut(bx(x0-1, x1+1, -4.2, 4.2, -p['yoke_len']-9, -p['yoke_len']+13.0))         # cable notch at the foot of the ulna
        (tcy, tcz), (tny, tnz) = tens_cn, tens_n; tty, ttz = -tnz, tny; ta = p['tens_adj']
        y = y.cut(prism_yz(_hull(_circ(tcy-tny*ta, tcz-tnz*ta, R_M5) + _circ(tcy+tny*ta, tcz+tnz*ta, R_M5)), x0-1, x1+1))   # M5 slot
        y = y.cut(cyl_x(x0-1, x1+1, seat))                                # 6801 in a through bore (laser cut, reamed; Loctite 638)
        if sg > 0:
            for zz in (-33.0, -41.0):
                y = y.cut(cyl_x(u0-1, u1+1, R_TAP3).translate((0, 0, zz)))               # tapped M3, pitch-encoder bracket
        for yy, zz in scr: y = y.cut(cyl_x(*xr(sg*(mface-1), sg*(u1+1)), R_M3).translate((0, yy, zz)))
        for yy in (-19.0, -9.0, 9.0, 19.0):                            # 4x M3 into inserts in the side of the forearm base
            y = y.cut(cyl_x(x0-1, x1+1, R_M3).translate((0, yy, -p['yoke_len']-4.0)))
        pk = cyl_x(x0-1, x1+1, R_M3).translate((0, 0, zc_m-14.5)).union(cyl_x(x0-1, x1+1, R_M3).translate((0, 0, zc_m+14.5)))   # 2x M3 for the bearing block of the motor shaft
        return y.cut(pk).val()
    parts['ulna'] = plate(-u1, -u0, -1)
    parts['radius'] = plate(u0, u1, 1)
    # one cover per side in front of the motors = the same ribbed part as the upper-arm covers
    Lc, tcv = p['cover']
    zc0 = max(-p['yoke_len'] + 4.0, zc_m - Lc/2)
    cvf = cover_part(u0, u1, Lc, tcv).val().translate(cq.Vector(0, Rh, zc0))
    parts['forearm_cover_front'] = cvf
    parts['forearm_cover_back'] = cvf.mirror('XZ', (0, 0, 0))
    drl = cover_drill(u0, u1, Lc, R_M3).val().translate(cq.Vector(0, Rh, zc0))        # 12 screw holes per bone for the cover tabs
    for bn in ('ulna', 'radius'):
        v0 = parts[bn].Volume(); parts[bn] = parts[bn].cut(drl).cut(drl.mirror('XZ', (0, 0, 0)))
        assert abs(v0 - parts[bn].Volume() - 6*math.pi*R_M3**2*p['yoke_t']) < 12.0, 'cover screw hole meets another feature in ' + bn
        for k, (yy, zz) in enumerate(scr, start=1):                    # motor stand-offs: bought M3 spacers (the bones are flat plates)
            sgb = -1 if bn == 'ulna' else 1
            parts[f'spacer_{bn}_{k}_alu'] = cyl_x(*xr(sgb*mface, sgb*u0), 3.5, 1.7).translate((0, yy, zz)).val()
    for side, sg in (('A', -1), ('B', 1)):
        xm = xr(sg*(mface-p['motor_len']), sg*mface)
        mot = bx(xm[0], xm[1], -sq/2, sq/2, zc_m-sq/2, zc_m+sq/2)
        for yy, zz in scr: mot = mot.cut(cyl_x(xm[0]-1, xm[1]+1, 1.65).translate((0, yy, zz)))
        parts[f'motor_{side}_NEMA17'] = mot.val()
        xs = xr(sg*mface, sg*(mface + p['motor_shaft_len']))
        xe = mface + p['motor_shaft_len']                                  # shaft end
        blk = prism_yz(_hull(_circ(0, zc_m, 11.5) + _circ(0, zc_m-14.5, 4.8) + _circ(0, zc_m+14.5, 4.8)), *xr(sg*(xe-5.5), sg*u0))
        blk = blk.cut(cyl_x(*xr(sg*(xe-6.0), sg*(xe+0.3)), fseat(16.0)).translate((0, 0, zc_m))).cut(cyl_x(*xr(sg*(xe-1.0), sg*(u0+1)), 4.0).translate((0, 0, zc_m)))
        for sy_ in (-1, 1): blk = blk.cut(cyl_x(*xr(sg*(u0-6.2), sg*(u0+1)), R_INS3).translate((0, 0, zc_m+sy_*14.5)))
        parts[f'motor_block_{side}'] = blk.val()                           # printed: holds the 625 of the motor shaft, 2x M3 through the bone
        parts[f'brg_motor_{side}_625'] = cyl_x(*xr(sg*(xe-5.2), sg*(xe-0.2)), 8.0, 2.5).translate((0, 0, zc_m)).val()
        parts[f'motor_{side}_shaft'] = cyl_x(xs[0], xs[1], p['motor_shaft_d']/2).translate((0, 0, zc_m)).val()
        mp = pulley_gen(sg*xc-p['mp_w']/2, p['mp_od']/2, p['mp_flange_od']/2, p['mp_w'], p['motor_shaft_d']/2)
        parts[f'mpulley_{side}_14T'] = mp.translate((0, 0, zc_m)).val()
        bw2 = p['belt_w']/2
        # adjustable tensioner: smooth idler on the belt back (front strand), M5 bolt in a slot of the plate, printed
        # slider with captive nut on the outer face, pushed inward by an M3 screw in a small housing (2 M3 inserts)
        (tcy, tcz), (tny, tnz) = tens_cn, tens_n; tty, ttz = -tnz, tny
        g = lambda u, v: (tcy + u*tny + v*tty, tcz + u*tnz + v*ttz)
        ri_t = p['tens_r']
        idl = (cyl_x(sg*xc-bw2, sg*xc+bw2, ri_t).union(cyl_x(sg*xc-bw2-1.0, sg*xc-bw2, ri_t+1.5))
               .union(cyl_x(sg*xc+bw2, sg*xc+bw2+1.0, ri_t+1.5)).cut(cyl_x(sg*xc-bw2-2, sg*xc+bw2+2, R_M5)))
        parts[f'idler_{side}_smooth'] = idl.translate((0, tcy, tcz)).val()
        parts[f'idler_{side}_spacer'] = cyl_x(*xr(sg*(xc+bw2+1.0), sg*u0), 4.5, 2.6).translate((0, tcy, tcz)).val()
        bolt = cyl_x(*xr(sg*(xc-bw2-1.0), sg*(u1+5.0)), 2.5).union(cyl_x(*xr(sg*(xc-bw2-4.5), sg*(xc-bw2-1.0)), 4.25))
        parts[f'tens_{side}_screw_M5'] = bolt.translate((0, tcy, tcz)).val()
        parts[f'tens_{side}_nut_M5'] = cyl_x(*xr(sg*u1, sg*(u1+4.7)), 4.6, 2.5).translate((0, tcy, tcz)).val()   # flange nut: slide the idler in its slot, then tighten
        rbm = Dp/2 - BT_IN
        parts[f'belt_{side}_GT2'] = belt_solid(sg*xc, bcirc(Cb, p['tens_d0']), p['belt_w']).val()
    for sd_, sg in (('L', -1), ('R', 1)):          # 4x M3x30 per motor, from the outer face into the motor face
        for k, (yy, zz) in enumerate(scr, start=1):
            parts[f'screw_M3x35_{sd_}_{k}'] = (cyl_x(*xr(sg*u1, sg*(mface-4.0)), 1.5)
                                               .union(cyl_x(*xr(sg*u1, sg*(u1+3.0)), 2.75)).translate((0, yy, zz)).val())

    # ---------- roll shaft, coils flush in the boss and the hand plate, camera, hand ----------
    z1 = plate_z1
    ft, ct, g = p['ferrite_t'], p['coil_t'], p['coil_gap']
    cbx, cby, cbz = p['cam_board']; tilt = -p['cam_tilt']; cxm = p['cam_x']
    def cam_place(sh, zt):        # camera frame: board bottom at z=0, lens up (+Z), tilted toward the hand axis
        return sh.rotate(O3, (0, 1, 0), tilt).translate(cq.Vector(cxm, 0, zt))
    tray = bx(-cbx/2-2, cbx/2+2, -cby/2-2, cby/2+2, -4.0, 4.0).cut(bx(-cbx/2-0.2, cbx/2+0.2, -cby/2-0.2, cby/2+0.2, 0.0, 5.0))
    tray = tray.cut(bx(-7, 7, -5, 5, -5, 1)).cut(bx(cbx/2-1, cbx/2+3, -5, 5, 0.5, 5))          # vent under the ESP32 + USB-C slot
    ca, sa = math.cos(math.radians(p['cam_tilt'])), math.sin(math.radians(p['cam_tilt']))
    zt = z1 + 1.0 + (cbx/2+2)*sa + 4.0*ca                                                        # lowest tray corner 1 mm above the plate
    ped = bx(cxm-12.0, cxm+12.0, -cby/2-2, cby/2+2, z1+0.02, zt+10)
    ped = ped.cut(cam_place(bx(-60, 60, -60, 60, -4.0, 60), zt))
    for yy in (-1, 1):                                                                           # screw ears, M3 into the flange inserts
        ear_ = bx(cxm-4, cxm+4, *sorted((yy*(cby/2+1), yy*18.5)), z1+0.02, z1+4.0)
        ped = ped.union(ear_.cut(cyl_z(z1-1, z1+5, 1.7, cx=cxm, cy=yy*15.0)))
    mount = ped.union(cam_place(tray, zt))
    mount = mount.cut(cyl_x(cxm-13.0, cxm-4.0, 1.6).translate((0, 8.0, z1+3.0)))                  # camera power lead into the mount
    parts['camera_mount'] = mount.val()
    board = bx(-cbx/2, cbx/2, -cby/2, cby/2, 0.0, cbz).union(bx(cbx/2-1, cbx/2+0.8, -4.5, 4.5, 0.6, 3.8))    # XIAO + Sense board, USB-C
    parts['camera_xiao_esp32s3_sense'] = cam_place(board, zt).val()
    lens = bx(-4.25, 4.25, -4.25, 4.25, cbz, cbz+2.5).union(cyl_z(cbz+2.5, cbz+7.0, 4.0))
    parts['camera_lens_ov3660'] = cam_place(lens, zt).val()
    fo, fi_ = p['ferrite_od']/2, p['ferrite_id']/2; pr_ = p['pad_r']
    def sheet(z0_, az):                          # flexible ferrite rings, with a notch under the two coil pads for the leads
        f_ = cyl_z(z0_, z0_+ft, fo-0.1, fi_+0.1)
        for d_ in (-20.0, 20.0):
            f_ = f_.cut(bx(pr_-2.2, fo+1, -2.2, 2.2, z0_-1, z0_+ft+1).rotate(O3, cq.Vector(0, 0, 1), az+d_))
        return f_
    def coil(z0_, az):                           # PCB coil: ring board, two pads on the rim (both faces)
        c_ = cyl_z(z0_, z0_+ct, p['coil_od']/2, p['coil_id']/2)
        return c_
    parts['ferrite_TX'] = sheet(bt-pocket, p['tx_pad_az']).val()
    tbw, tbd, tbh = p['tx_board']
    parts['tx_driver_board'] = bx(-cxm-tbw/2, -cxm+tbw/2, -tbd/2, tbd/2, plate_z1+3.0, plate_z1+3.0+tbh).val()
    parts['coil_TX'] = coil(bt-ct, p['tx_pad_az']).val()
    z_hb0 = bt + g
    parts['coil_RX'] = coil(z_hb0, p['rx_pad_az']).val()
    parts['ferrite_RX'] = sheet(z_hb0+ct+p['glue_t'], p['rx_pad_az']).val()
    parts['roll_shaft_steel_12'] = (cyl_z(z_T1-rbw, z_hb0+HAND['bot_t']+HAND['hub_fl'][1], rsd).cut(cyl_z(z_T1-rbw-1, z_T1-rbw+2.6, 3.05))   # magnet pocket
                                    .cut(bx(4.0, 7.0, -7.0, 7.0, toe_c-0.5, h+hl+0.5)).cut(bx(5.0, 7.0, -7.0, 7.0, z_hb0+HAND['bot_t']-(HAND['hub_h']-HAND['hub_fl'][1])+1.0, z_hb0+HAND['bot_t']+HAND['hub_fl'][1]+1)).val())   # flats: 2 mm under gear C, 1 mm for the hub set screws
    skirt_depth = z_hb0 - (plate_z1 + sh + 0.0)
    hparts, hinfo, apple = build_hand(z_hb0, rx=dict(depth=pocket, wire=(p['pad_r']+0.6, p['rx_pad_az']), r_out=p['ferrite_od']/2+0.1, r_in=p['ferrite_id']/2-0.1,
                                      shaft_r=rsd+0.1, plate_r=p['ring_od']/2+2.5, skirt_in=fseat(p['ring_od']), skirt_depth=skirt_depth))
    parts.update(hparts)

    # ---------- cables: harness out of the forearm base (centre hole), up the ulna, across the pitch axis ON the axle
    # end (service loop), over the flange to the TX driver and the camera; motor and pitch-encoder leads ----------
    zb = -p['yoke_len']; xo = -(u1 + 5.0); xl = -(x_pe + 6.0)
    parts['cable_fore_main'] = cable([(0.0, 0.0, zb-8.0), (0.0, 0.0, zb-1.0), (-8.0, 0.0, zb+3.1), (-20.0, 0.0, zb+3.1),
                                      (-35.0, 0.0, zb+3.1), (-50.0, 0.0, zb+3.1), (-62.0, 0.0, zb+4.3), (xo, 0.0, zb+10.0),
                                      (xo, 0.0, zb+24.0), (xo, 0.0, -60.0), (xo, 0.0, -42.0),
                                      (xl, 0.0, -24.0), (xl, 0.0, -6.0), (xl, 0.0, 0.0)], 3.0)
    zmt = zc_m + sq/2
    parts['cable_motor_A'] = cable([(-20.0, 0.0, zmt+0.3), (-20.0, 0.0, zmt+4.0), (-24.0, 0.0, zmt+12.0), (-44.0, 0.0, -72.0), (-58.0, -2.5, -65.0), (-66.5, -2.5, -59.0)], 2.0)
    parts['cable_motor_B'] = cable([(20.0, 0.0, zmt+0.3), (20.0, 0.0, zmt+4.0), (16.0, 0.0, zmt+10.0), (-10.0, 0.0, -82.0), (-44.0, 2.5, -66.0), (-58.0, 2.5, -60.0), (-66.5, 2.5, -55.0)], 2.0)
    parts['cable_enc_pitch'] = cable([(xc1-0.2, -9.6, -3.0), (xc1, -12.5, -10.0), (xc1+0.5, -13.0, -26.0), (u1+6.0, -10.0, -50.0), (u1+2.5, -5.0, -58.0), (u0+3.0, -4.0, -61.0)], 1.5)
    tbw, tbd, tbh = p['tx_board']; ztb = plate_z1 + 3.0 + tbh/2
    parts['cable_flange_main'] = cable([(xl, 0.0, 0.0), (xl, 0.0, 8.0), (xl, 0.0, plate_z1+2.0), (-cxm-tbw/2-5.5, 0.0, ztb), (-cxm-tbw/2-0.6, 0.0, ztb)], 3.0)
    zg_ = plate_z1 - 1.6                                                                  # cable centre in the groove
    gpts = []
    for (ga, gb), (gc, gd) in zip(cam_groove[:-1], cam_groove[1:]):
        L_ = math.hypot(gc-ga, gd-gb); n_ = max(2, int(L_/4.0))
        for k in range(n_):
            t_ = (k + 0.5)/n_; gpts.append((ga + (gc-ga)*t_, gb + (gd-gb)*t_, zg_))
    (sa, sb), (ea, eb) = cam_groove[0], cam_groove[-1]
    parts['cable_camera'] = cable_poly([(sa, sb, ztb-1.0), (sa, sb, zg_)] + [(cg[0], cg[1], zg_) for cg in cam_groove[1:-1]] +
                                       [(ea, eb, zg_), (ea, eb, z1+3.0), (45.0, eb, z1+3.0)], 1.25)

    info = dict(cone_h=h, wall_in=wall_in, wall_out=wall_out, plate=(plate_z0, plate_z1), grip_top=hinfo['tip_z'],
                yoke_x=(u0, u1), mface=mface, zc_m=zc_m, yoke_len=p['yoke_len'], width=2*wall_out, hand=hinfo, apple_shape=apple,
                pulley_teeth=pulley_teeth, belt_teeth=Nb, belt_c=Cb, tens_dL=dL_t, tens_c=tens_cn, ratio=Dp/dp, Rh=Rh, module=p['module'], face=p['face'])
    return parts, info

COLORS = {
    'gear_C_output': '#d9a21b', 'carrier_flange': '#1d9e75', 'axle_static_steel': '#8a8f98',
    'roll_shaft_steel_12': '#e3d9c4', 'coil_TX': '#c2562f', 'coil_RX': '#c2562f', 'ferrite_TX': '#55585f', 'ferrite_RX': '#55585f',
}

COVER_Z = (25.0, 67.0, 85.5)                   # default tab heights along the cover: clear of the motor screws and of the belt idlers
def cover_drill(u0, u1, L, r, zs=COVER_Z):
    """M3 clearance holes through the bones for the cover screws: they go into inserts in the tabs of cover_part"""
    d = None
    for zz in zs:
        for s in (-1, 1):
            c = cq.Workplane('YZ').workplane(offset=min(s*(u0-1.0), s*(u1+1.0))).center(-5.0, zz).circle(r).extrude(abs(u1-u0)+2.0)
            d = c if d is None else d.union(c)
    return d

def cover_part(u0, u1, L, t, rib=0.0, rw=3.0, zs=COVER_Z):
    """common cover (same printed part for upper arm and forearm): local frame x across, z along (0..L),
    plate y 0..t (flat outer face), tabs with M3 inserts on the inner side; optional ribs on the outer face if rib > 0"""
    W = u1; xs_ = (u0+u1)/2; zm = L/2
    def box(x0, x1, y0, y1, z0, z1):
        return cq.Workplane('XY').box(x1-x0, y1-y0, z1-z0, centered=False).translate((x0, y0, z0))
    def cylY(r, cx, cz, y0, y1):
        return cq.Workplane('XZ').workplane(offset=-y1).center(cx, cz).circle(r).extrude(y1-y0)
    c = box(-W, W, 0, t, 0, L)
    rb = box(-W, W, t-0.01, t+rib, 0, L).cut(box(-W+rw, W-rw, t-1, t+rib+1, rw, L-rw))          # frame
    for xx in (-27.0, 27.0):
        rb = rb.union(cylY(17.0+rw, xx, zm, t-0.01, t+rib).cut(cylY(17.0, xx, zm, t-1, t+rib+1)))  # rings
    rb = rb.union(box(-rw/2, rw/2, t-0.01, t+rib, 0, L))                                         # centre rib
    for s in (-1, 1):
        rb = rb.union(box(*sorted((s*(27.0+17.0+rw-0.5), s*(xs_-3.0))), t-0.01, t+rib, zm-rw/2, zm+rw/2))
        rb = rb.union(box(*sorted((s*(rw/2-0.01), s*(27.0-17.0-rw+0.5))), t-0.01, t+rib, zm-rw/2, zm+rw/2))
    if rib > 0: c = c.union(rb)                             # no outer ribs by default: the flat outer face goes on the print bed, tabs up
    for xx in (-27.0, 27.0):
        c = c.cut(cylY(17.0, xx, zm, -1, t+rib+1))
    for zz in zs:                                           # tabs against the inner face of each bone: M3 insert, screw from outside the bone
        for s in (-1, 1):
            tab = box(*sorted((s*(u0-8.0), s*(u0-0.2))), -9.0, 0.01, zz-4.0, zz+4.0)
            ins = cq.Workplane('YZ').workplane(offset=min(s*(u0-7.0), s*(u0+0.1))).center(-5.0, zz).circle(R_INS3).extrude(7.1)
            c = c.union(tab).cut(ins)
    return c

GUNMETAL, GUNMETAL_GEAR, TPU_DARK, STEEL, MOTOR_DARK = '#222428', '#33373d', '#141517', '#c9cdd3', '#3a3d42'   # matte black PLA
GUN_D, GUN_L, PCB_DARK = '#1a1c1f', '#2a2d32', '#1f2226'
ALU, CARBON = '#d3d7dc', '#17181a'
GUN_1, GUN_2, GUN_3, BRASS = '#2b2e33', '#3d4148', '#1f2125', '#a9853f'   # gunmetal printed parts, brass accents      # laser-cut aluminium bones; PETG-CF / PA-CF gears
def tone(n):
    """three gunmetal tones so neighbouring printed parts read apart: bones dark, covers mid, flanges / small parts light"""
    if n.startswith(('ulna', 'radius', 'hand_head', 'camera_mount')): return GUN_D
    if n.startswith(('carrier_flange', 'carrier_wall', 'motor_block', 'enc_pitch_bracket', 'tens_', 'roll_shaft', 'spacer_', 'idler_', 'hand_finger')): return GUN_L
    return GUNMETAL

def material(n):
    """'printed' (gunmetal PETG/PA-CF), 'gear' (printed gears/pulleys, a shade lighter), 'tpu', 'steel' (brushed), or None"""
    if n.startswith('hand_finger_flex'): return 'tpu'
    if n.startswith(('brg', 'T_brg', 'pitch_axle', 'pre_', 'T_idle_tube', 'carrier_spacer', 'screw', 'mpulley', 'hand_pin', 'hand_thrust',
                     'hand_motor_shaft', 'roll_shaft', 'idler_', 'hand_worm', 'hand_shaft_hub', 'hand_brg', 'spacer_')) or n.endswith('_smooth') or 'nut_M5' in n or (n.startswith('motor_') and n.endswith('_shaft')) or 'screw' in n:
        return 'steel'
    if n.startswith(('gear_', 'idle_gear')): return 'gear'
    if n.startswith(('ulna', 'radius')): return 'alu'
    if n.startswith(('T_fitting', 'carrier_flange', 'carrier_wall', 'motor_block', 'forearm_cover', 'camera_mount', 'enc_pitch_bracket', 'tens_',
                     'hand_body', 'hand_head', 'hand_finger', 'hand_palm')): return 'printed'
    return None
SPEC = dict(steel=1.0, alu=0.8, printed=0.07, gear=0.09, tpu=0.05)
def spec_of(n):
    return SPEC.get(material(n), 0.15)
MATERIAL_COLORS = False      # True = black PLA / brushed steel palette; False = the original colour-coded view
def color_of(n):
    m = material(n) if MATERIAL_COLORS else None
    if m: return dict(printed=tone(n), gear=GUNMETAL_GEAR, tpu=TPU_DARK, steel=STEEL, alu=ALU)[m]
    if n.startswith('motor_') and 'NEMA' in n and MATERIAL_COLORS: return MOTOR_DARK
    return _base_color(n)
def _base_color(n):
    if material(n) == 'gear': return CARBON                  # PETG-CF / PA-CF gears
    if n.startswith('hand_worm'): return STEEL               # bought steel worm
    if n.startswith('cable_motor'): return '#5a5f66'
    if n.startswith('cable_enc') or n == 'cable_camera': return '#b03a2e'
    if n.startswith('cable'): return '#151515'
    if n.startswith('forearm_cover'): return GUN_3
    if n.startswith(('brg', 'T_brg')): return '#c9ced6'
    if 'as5600' in n or n == 'tx_driver_board': return PCB_DARK
    if 'magnet' in n: return '#2b2b2b'
    if n == 'idle_gear_D_24T': return '#d9a21b'
    if n.startswith(('pre_', 'T_idle_tube', 'pitch_axle', 'carrier_spacer')): return '#8a8f98'
    if n == 'enc_pitch_bracket': return BRASS
    if n.startswith('belt'): return '#222222'
    if n.startswith('motor_block'): return GUN_2
    if n.startswith('motor'): return '#5b6470'
    if n == 'T_fitting': return BRASS
    if n == 'carrier_flange' or n.startswith(('carrier_wall', 'motor_block')): return GUN_1
    if n == 'hand_palm': return BRASS
    if n == 'camera_xiao_esp32s3_sense': return '#2b2b2b'
    if n == 'camera_lens_ov3660': return '#0d0d0d'
    if n == 'camera_mount': return GUN_2
    if n.startswith('screw'): return '#8a8f98'
    if n.startswith('spacer'): return '#a9afb8'
    if n.startswith('idler'): return '#c9ced6'
    if n.startswith('tens_'): return '#8a8f98' if 'screw' in n else '#3a3d44'
    if n.startswith(('ulna', 'radius')): return ALU
    if n.startswith('mpulley'): return '#3a3d44'
    if n.startswith('hand_finger_flex'): return '#9c1f1a'      # TPU fingers: dark red
    if n.startswith('hand_finger'): return GUN_3
    if n == 'hand_limit_switch': return '#1e1e1e'
    if n.startswith('hand_worm'): return '#d9a21b'
    if n in ('hand_gearmotor_25mm', 'hand_motor_shaft'): return '#5b6470'
    if n == 'hand_rx_driver_board': return PCB_DARK
    if n == 'hand_bulk_cap': return '#2a2a2a'
    if n == 'hand_body': return GUN_1
    if n == 'hand_head': return GUN_2
    if n.startswith(('hand_pin', 'hand_thrust')): return '#8a8f98'
    return COLORS.get(n, '#999999')

def interference(parts, tol=0.5):
    names = list(parts)
    bad = []
    for a, b in itertools.combinations(names, 2):
        ba, bb = parts[a].BoundingBox(), parts[b].BoundingBox()
        if (ba.xmax < bb.xmin or bb.xmax < ba.xmin or ba.ymax < bb.ymin or bb.ymax < ba.ymin
                or ba.zmax < bb.zmin or bb.zmax < ba.zmin):
            continue
        try:
            v = parts[a].intersect(parts[b]).Volume()
        except Exception:
            v = float('nan')
        if not (v < tol):
            bad.append((a, b, round(v, 2)))
    return bad

def group(n):
    if n.startswith('idle_'): return 'idle'
    if n.startswith(('pitch_', 'carrier_spacer', 'pre_')): return 'pitch'
    if n.startswith('brg_yoke'): return 'fixed'
    if 'axle' in n or n.startswith(('ulna', 'radius', 'forearm_cover', 'belt', 'motor', 'mpulley', 'screw', 'spacer', 'idler', 'tens_', 'enc_', 'brg_motor', 'cable_fore', 'cable_motor', 'cable_enc')): return 'fixed'
    if n in ('gear_A_pulley_one_piece', 'brg_A_toe', 'brg_A_pulley'): return 'A'
    if n in ('gear_B_pulley_one_piece', 'brg_B_toe', 'brg_B_pulley'): return 'B'
    if n.startswith('hand_worm'): return 'worm'
    if n.startswith('hand_finger'): return 'finger'
    if n in ('gear_C_output', 'roll_shaft_steel_12', 'coil_RX', 'ferrite_RX', 'roll_magnet_6x2.5') or n.startswith('hand_'): return 'roll'
    return 'pitch'
ROLL_SIGN = 1.0   # side gear A turns by +ROLL_SIGN*roll, B by -ROLL_SIGN*roll (verified numerically)
IDLE_SIGN = -1.0  # idle bevel D (axis -Z) turns by IDLE_SIGN*roll about +Z (verified numerically)

def pose(parts, pitch_deg=0.0, roll_deg=0.0, theta=None, info=None):
    """theta: finger opening angle in degrees (larger = more open). Worm turns s_w*(theta-theta0)*14 degrees."""
    O, X, Z = (0, 0, 0), (1, 0, 0), (0, 0, 1)
    hi = (info or {}).get('hand', {})
    if theta is None: theta = hi.get('theta0', 0.0)
    out = {}
    for n, s in parts.items():
        g = group(n)
        if g == 'fixed': out[n] = s
        elif g == 'A': out[n] = s.rotate(O, X, pitch_deg + ROLL_SIGN*roll_deg)
        elif g == 'B': out[n] = s.rotate(O, X, pitch_deg - ROLL_SIGN*roll_deg)
        elif g == 'worm': out[n] = s.rotate(O, Z, hi['s_w']*(theta-hi['theta0'])*hi['zt'] + roll_deg).rotate(O, X, pitch_deg)
        elif g == 'finger':
            pv, ax = hi['fingers'][n]; e = (pv[0]+ax[0], pv[1]+ax[1], pv[2]+ax[2])
            out[n] = s.rotate(pv, e, theta-hi['theta0']).rotate(O, Z, roll_deg).rotate(O, X, pitch_deg)
        elif g == 'roll': out[n] = s.rotate(O, Z, roll_deg).rotate(O, X, pitch_deg)
        elif g == 'idle': out[n] = s.rotate(O, Z, IDLE_SIGN*roll_deg).rotate(O, X, pitch_deg)
        else: out[n] = s.rotate(O, X, pitch_deg)
    return out

def export_all(parts, outdir='out'):
    os.makedirs(outdir + '/step', exist_ok=True)
    asm = cq.Assembly(name='wrist_hand')
    for n, s in parts.items():
        cq.exporters.export(cq.Workplane('XY').newObject([s]), f'{outdir}/step/{n}.step')
        asm.add(s, name=n, color=cq.Color(color_of(n)))
    asm.save(f'{outdir}/wrist_hand_assembly.step')
    comp = cq.Compound.makeCompound(list(parts.values()))
    cq.exporters.export(cq.Workplane('XY').newObject([comp]), f'{outdir}/wrist_hand_assembly.stl',
                        tolerance=0.1, angularTolerance=0.2)

if __name__ == '__main__':
    parts, info = build(P)
    print('built', len(parts), 'parts')
    bad = [b for b in interference(parts) if 'belt' not in b[0] and 'belt' not in b[1]]
    print('interferences (belts excluded):', bad if bad else 'none')
    export_all(pose(parts, P['pitch_deg'], info=info))
    cq.exporters.export(cq.Workplane('XY').newObject([info['apple_shape']]), 'out/step/apple_reference_80mm.step')
    print('exported')
