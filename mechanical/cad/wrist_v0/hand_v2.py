"""Hand v2.6 (14 mm shorter): the roll-shaft hub is upside down with its neck sunk in the 12 mm bottom, the worm thread is 16 mm
(cut from 22) and centred on the three wheels, no washer under the worm hub (the thrust goes up).
Hand v2.5 (turret head + dome palm): the three wheels turn in radial slots of one turret (three hollow pillars, 9 mm cheeks),
plain 5 x 32 dowel pins; the palm is a dome dropped on from above, 6x M3 into the cheek tops beside the wheels; its plugs lock the pins.
Hand v2.4 (strong grip): bought steel worm (module 1, PD 16, right hand, like KHK SW1-R1) held at BOTH ends: the grip load
pushes the worm up onto an F6-12M thrust ball bearing in the printed palm cap; 25 mm metal gearmotor (Pololu 25D 75:1 class); metal flange
hub on the roll shaft. Hand v2.3 (passive electronics, no MCU in the hand). Hand v2.2: motor -> worm (axis up, top FREE, thrust on a washer) -> three worm wheels -> three straight Fin Ray fingers.
The worm/wheel pair is mirrored (left-hand thread) so grip load pushes the worm DOWN onto the thrust washer.
v2.2: robust PETG finger roots fused to the wheels (Fin Ray TPU plugs in with a tenon + 2x M3), full travel from
'tips touching' to wide open, tapered cheeks with R10 fillets, wide vents, printable split (body / head) with M3 inserts,
motor / switch screw holes, wire holes, board rails, motor cradle, e-clips and a grub screw on the worm hub.
Angles: theta = wheel angle; finger axis from vertical = theta - cant (0 = fingers parallel, < 0 = closing in). Units mm."""
import math
import cadquery as cq
from cq_gears import Worm, CrossedHelicalGear

H = dict(
    worm_m=1.0, worm_z=28, worm_pd=16.0, worm_len=16.0, wheel_w=10.0, gear_backlash=0.1, phase=8.35, pin_d=5.0,
    right_hand=True,                                                    # bought worms are right-handed: nothing is mirrored
    theta0=46.0, theta_min=11.0, theta_max=80.0, theta_grasp=46.0,      # theta_min: Fin Ray tips touch (closing sweep)
    # finger = PETG root (one piece with the wheel) + TPU Fin Ray plugged in with a tenon and 2x M3 cross screws
    cant=20.0, root_z=24.0, root_w=20.0, root_in=7.0,
    fr_L=68.0, fr_wb=20.0, fr_wt=6.0, fr_wall=2.8, fr_rib=2.4, fr_pitch=10.0, fr_thick=18.0, fr_solid=12.0,
    tenon=(14.0, 9.0), tenon_clr=0.2, m3_x=3.5,
    lobe_r=18.2, lobe_w=2.0, sw=(12.8, 5.8, 6.5), sw_holes=6.5, blade_w=1.0,     # limit switch (KW10-size micro switch)
    motor_d=25.0, motor_len=54.0, shaft_d=4.0, shaft_len=12.5, motor_screw_r=8.5,      # 25D metal gearmotor: 54 long, 4 mm D shaft, 2x M3 17 mm apart
    hub_h=12.0, hub_fl=(32.0, 4.0), hub_neck=22.0, hub_pcd=26.0,       # 12 mm flange hub on the roll shaft (check the part you buy)
    frame_r=35.0, wall_t=2.5, bot_t=12.0, head_t=4.0, head_r=35.0, lip=(1.2, 1.8),
    rxb=(20.0, 25.0, 6.0), cap=(8.0, 12.0),     # passive RX driver board (width, height, depth) standing on the wall; bulk cap
    washer=(10.0, 4.4, 1.0), hub_d=13.0, hub_len=10.0,                 # worm hub (part of the bought worm), M4 set screw
    palm_t=7.0, top_brg=(12.0, 6.0, 4.5),                              # palm cap with an F6-12M thrust ball bearing on the worm top pin
    cheek_t=9.0, cheek_gap=0.4, turret_bore=12.0, turret_wall=3.5, turret_top=8.0, pin_len=32.0, dome_rim=3.5, dome_scr=(29.5, 10.2),
    boss_az=(60.0, 180.0, 300.0), boss_r=4.5, insert_d=4.0, insert_len=6.0,      # M3 heat-set inserts
    vent_az=(30.0, 120.0, 150.0, 200.0, 280.0, 330.0), vent_w=7.0,
    rxb_az=0.0, cap_az=90.0,
    apple_r=40.0, apple_dz=60.0,
)
O, X, Y, Z = (0, 0, 0), (1, 0, 0), (0, 1, 0), (0, 0, 1)

def bx(x0, x1, y0, y1, z0, z1):
    x0, x1 = sorted((x0, x1)); y0, y1 = sorted((y0, y1)); z0, z1 = sorted((z0, z1))
    return cq.Workplane('XY').box(x1-x0, y1-y0, z1-z0, centered=False).translate((x0, y0, z0))
def cyl_z(z0, z1, r, ri=0.0, cx=0.0, cy=0.0):
    w = cq.Workplane('XY').workplane(offset=z0).center(cx, cy).circle(r)
    if ri: w = w.circle(ri)
    return w.extrude(z1-z0)
def cyl_y(y0, y1, r, cx, cz):
    return cq.Workplane('XZ').workplane(offset=-y1).center(cx, cz).circle(r).extrude(y1-y0)
def cyl_x(x0, x1, r, cy, cz):
    return cq.Workplane('YZ').workplane(offset=x0).center(cy, cz).circle(r).extrude(x1-x0)
def prism_xz(pts, y0, y1):          # polygon in the XZ plane, extruded along Y from y0 to y1
    return cq.Workplane('XZ').workplane(offset=-y1).polyline(pts).close().extrude(y1-y0)
def prism_yz(pts, x0, x1):          # polygon in the YZ plane, extruded along X from x0 to x1
    return cq.Workplane('YZ').workplane(offset=x0).polyline(pts).close().extrude(x1-x0)
def mirror(shape):
    return shape if H['right_hand'] else shape.mirror('XZ', (0, 0, 0))
def polar(r, az):
    return r*math.cos(math.radians(az)), r*math.sin(math.radians(az))


def finray(h):
    """TPU Fin Ray: profile in the XZ plane (finger axis +Z, contact face -X), thickness fr_thick along Y.
    z = 0..fr_L, solid base 0..fr_solid with a socket that slips over the PETG tenon. Printed flat (Y on the bed)."""
    L, wb, wt, tw, rib, pit, T = h['fr_L'], h['fr_wb'], h['fr_wt'], h['fr_wall'], h['fr_rib'], h['fr_pitch'], h['fr_thick']
    ext = lambda pts: cq.Workplane('XZ').polyline(pts).close().extrude(T/2, both=True)
    outer = ext([(-wb/2, 0.0), (wb/2, 0.0), (wt/2, L), (-wt/2, L)])
    k = (wb - wt)/2.0/L
    dw = tw*math.sqrt(1.0 + k*k)
    zb, zt_c = h['fr_solid'], L - 2.5*tw
    cavity = ext([(-(wb/2-dw) + k*zb, zb), ((wb/2-dw) - k*zb, zb), ((wb/2-dw) - k*zt_c, zt_c), (-(wb/2-dw) + k*zt_c, zt_c)])
    ribs = None
    for i in range(int((zt_c - zb)/pit) + 2):
        z0 = zb + 0.6*pit + i*pit
        r = ext([(-wb, z0), (-wb, z0+rib*1.4), (wb, z0 + 0.9*wb + rib*1.4), (wb, z0 + 0.9*wb)])
        ribs = r if ribs is None else ribs.union(r)
    fin = outer.cut(cavity.cut(ribs))
    tx, tz = h['tenon']; c = h['tenon_clr']; Wr = h['wheel_w']
    fin = fin.cut(bx(-tx/2-c, tx/2+c, -Wr/2-c, Wr/2+c, -1.0, tz+c))                 # socket for the PETG tenon
    for sx in (-h['m3_x'], h['m3_x']):
        fin = fin.cut(cyl_y(-T-1, T+1, 1.6, sx, tz/2))                                 # 2x M3 + nut, washers on the TPU
    return fin

def root(h, W):
    """PETG finger root in the wheel frame (finger axis +Z before the cant): same width as the wheel (prints flat with
    the wheel, layers in the bending plane, no supports), wide in X, inner edge tapered near the worm, male tenon on top."""
    rz, rw, ri = h['root_z'], h['root_w']/2, h['root_in']
    r = prism_xz([(-ri, -3.0), (rw, -3.0), (rw, rz), (-rw, rz), (-rw, 12.0)], -W/2, W/2)
    tx, tz = h['tenon']
    r = r.union(bx(-tx/2, tx/2, -W/2, W/2, rz-0.01, rz+tz))
    for sx in (-h['m3_x'], h['m3_x']):
        r = r.cut(cyl_y(-W, W, 1.6, sx, rz+tz/2))
    return r

def build_hand(z0, h=H, rx=None):
    parts, info = {}, {}
    m, zt = h['worm_m'], h['worm_z']
    lam = math.degrees(math.atan(m/h['worm_pd'])); lead = math.pi*m
    worm = Worm(module=m, lead_angle=lam, n_threads=1, length=h['worm_len'], backlash=h['gear_backlash'])
    wh = CrossedHelicalGear(module=m, teeth_number=zt, width=h['wheel_w'], helix_angle=lam, backlash=h['gear_backlash'])
    a, W = worm.r0 + wh.r0, h['wheel_w']
    # ---- vertical stack ----
    z_b1 = z0 + h['bot_t']; z_m0 = z_b1 + h['hub_fl'][1] + 0.5; z_mt = z_m0 + h['motor_len']
    z_bd0 = z_b1 + 0.5
    z_h0 = max(z_mt + 0.3, z_bd0 + h['rxb'][1] + 0.5); z_h1 = z_h0 + h['head_t']
    wsh_od, wsh_id, wsh_t = h['washer']
    z_w1 = z_h1 + 0.3; z_hub1 = z_w1 + h['hub_len']
    alphas = (0.0, 120.0, 240.0)
    shifts = [lead/3.0 - lead*al/360.0 for al in alphas]                   # the three wheels sit centred on the short worm
    zc = z_hub1 + h['worm_len']/2
    Rw = h['frame_r']; Ri = Rw - h['wall_t']

    # ---- motor, shaft, boards standing on the inner wall ----
    parts['hand_gearmotor_25mm'] = cyl_z(z_m0, z_mt, h['motor_d']/2).val()
    fo_, ft_ = h['hub_fl']
    nk_ = h['hub_h'] - ft_                                              # hub upside down: flange on the bottom plate, neck sunk in it
    hubm = cyl_z(z_b1, z_b1+ft_, fo_/2).union(cyl_z(z_b1-nk_, z_b1+0.01, h['hub_neck']/2)).cut(cyl_z(z_b1-nk_-1, z_b1+ft_+1, (rx or {}).get('shaft_r', 6.1)))
    for k in range(4):
        hx, hy = polar(h['hub_pcd']/2, 45.0+90.0*k); hubm = hubm.cut(cyl_z(z_b1-1, z_b1+ft_+1, 1.7, cx=hx, cy=hy))
    parts['hand_shaft_hub_alu'] = hubm.val()
    parts['hand_motor_shaft'] = cyl_z(z_mt, z_mt+h['shaft_len'], h['shaft_d']/2).val()
    # passive hand (no MCU, no radio): RX coil -> bridge rectifier -> DRV8871 -> gearmotor; two LC detectors pick the
    # carrier frequency (100 kHz = close, 200 kHz = open) and drive IN1 / IN2; the NC limit switch cuts IN2 at full open
    es = h['rxb']
    dE = math.sqrt(Ri**2 - (es[0]/2)**2) - 0.1; pcb_t = 1.6
    comp = bx(dE-pcb_t, dE, -es[0]/2, es[0]/2, z_bd0, z_bd0+es[1])      # (y0, y1, z0, z1, height) on the inner face: DRV8871, 4x SS24, 2x LC detectors, zeners, ILIM resistor
    for y0, y1, c0, c1, ht in ((-4.0, 1.0, 15.0, 21.0, 1.75), (-9.0, -4.7, 2.5, 5.1, 2.3), (-4.0, 0.3, 2.5, 5.1, 2.3),
                               (-9.0, -4.7, 7.0, 9.6, 2.3), (-4.0, 0.3, 7.0, 9.6, 2.3), (1.8, 6.3, 2.5, 5.7, 3.2),
                               (1.8, 6.3, 7.0, 10.2, 3.2), (7.0, 8.6, 2.5, 5.2, 1.1), (7.0, 8.6, 7.0, 9.7, 1.1),
                               (2.5, 4.5, 15.5, 16.8, 0.6), (-8.5, -6.5, 15.5, 20.5, 2.5)):
        b = bx(dE-pcb_t-ht, dE-pcb_t+0.01, y0, y1, z_bd0+c0, z_bd0+c1)
        comp = comp.union(b)
    parts['hand_rx_driver_board'] = comp.rotate(O, Z, h['rxb_az']).val()
    cx_, cy_ = polar(Ri-5.0, h['cap_az'])
    parts['hand_bulk_cap'] = cyl_z(z_bd0, z_bd0+h['cap'][1], h['cap'][0]/2, cx=cx_, cy=cy_).val()

    # ================= HAND BODY (printed upright: bottom plate on the bed) =================
    fr = cyl_z(z0, z_b1, (rx or {}).get('plate_r', Rw))
    if rx and rx.get('skirt_in'): fr = fr.union(cyl_z(z0-rx['skirt_depth'], z0+0.01, rx['plate_r'], ri=rx['skirt_in']))
    fr = fr.union(cyl_z(z_b1-0.01, z_h0, Rw, ri=Ri))
    # M3 heat-set insert bosses under the head (45 deg cone underneath = no supports)
    for az in h['boss_az']:
        bxy = polar(Ri-2.5, az); br = h['boss_r']
        boss = cyl_z(z_h0-10.0, z_h0, br, cx=bxy[0], cy=bxy[1])
        cone = cq.Solid.makeCone(0.5, br, br-0.5, cq.Vector(bxy[0], bxy[1], z_h0-10.0-(br-0.5)), cq.Vector(0, 0, 1))
        fr = fr.union(boss).union(cq.Workplane('XY').newObject([cone]))
        fr = fr.cut(cyl_z(z_h0-h['insert_len'], z_h0+1, h['insert_d']/2, cx=bxy[0], cy=bxy[1]))
    # board rails (boards slide in from the top before the head goes on)
    for az, (bw_, bh_, bd_), d_ in ((h['rxb_az'], es, dE),):
        for s in (-1, 1):
            rail = bx(d_-bd_-1.8, Ri, s*(bw_/2+0.25), s*(bw_/2+2.25), z_bd0, z_bd0+bh_-2.0)
            lip = bx(d_-bd_-1.8, d_-bd_-0.2, s*(bw_/2-1.5), s*(bw_/2+2.25), z_bd0, z_bd0+bh_-2.0)
            fr = fr.union(rail.union(lip).rotate(O, Z, az))
    # wide vents with a 45 deg pointed roof (prints without supports)
    vw, vz0, vz1 = h['vent_w'], z_b1+3.0, z_h0-4.0
    for az in h['vent_az']:
        slot = prism_yz([(-vw/2, vz0), (vw/2, vz0), (vw/2, vz1-vw/2), (0.0, vz1), (-vw/2, vz1-vw/2)], Ri-2.0, Rw+1.0)
        fr = fr.cut(slot.rotate(O, Z, az))
    lt, lh = h['lip']
    lip = cyl_z(z_h0-0.01, z_h0+lh, Ri+lt, ri=Ri)                                   # locating lip, fits a groove in the head
    for az in h['boss_az']:
        bxy = polar(Ri-2.5, az); lip = lip.cut(cyl_z(z_h0-1, z_h0+lh+1, h['insert_d']/2+0.8, cx=bxy[0], cy=bxy[1]))
    fr = fr.union(lip)
    if rx: fr = fr.cut(cyl_z(z0-1, z0+rx['depth'], rx['r_out'], ri=rx['r_in']))
    fr = fr.cut(cyl_z(z0-1, z_b1+1, (rx or {}).get('shaft_r', 4.1)))            # the shaft goes through into the metal hub
    for k in range(4):                                                          # 4 M3 inserts under the hub flange
        hx, hy = polar(h['hub_pcd']/2, 45.0+90.0*k); fr = fr.cut(cyl_z(z_b1-6.2, z_b1+1, h['insert_d']/2, cx=hx, cy=hy))
    fr = fr.cut(cyl_z(z_b1-nk_-0.3, z_b1+1, h['hub_neck']/2+0.2))               # pocket for the hub neck
    for az in (0.0, 270.0):                                                     # key access to the two hub set screws
        fr = fr.cut(cyl_x(h['hub_neck']/2-1, (rx or {}).get('plate_r', Rw)+1, 1.6, 0, z_b1-nk_/2).rotate(O, Z, az))
    wr_, wa_ = (rx or {}).get('wire', (15.0, 90.0))
    for d_ in (-20.0, 20.0):                                                   # RX coil leads: one hole under each pad of the PCB coil
        wx, wy = polar(wr_, wa_ + d_)
        fr = fr.cut(cyl_z(z0-1, z_b1+1, 1.3, cx=wx, cy=wy))
    parts['hand_body'] = fr.val()

    # ================= HAND HEAD (head plate + cheeks, printed plate-down, groove on the bed side) =================
    hd = cyl_z(z_h0, z_h1, h['head_r'])
    hd = hd.cut(cyl_z(z_h0-1, z_h0+h['lip'][1]+0.2, Ri+h['lip'][0]+0.2, ri=Ri-0.2))      # groove for the body lip
    hd = hd.cut(cyl_z(z_h0-3, z_h1+1, 3.75))                                      # Ø7 boss of the gearbox face
    for az in h['boss_az']:                                                       # M3 screws into the inserts
        bxy = polar(Ri-2.5, az)
        hd = hd.cut(cyl_z(z_h0-3, z_h1+1, 1.7, cx=bxy[0], cy=bxy[1])).cut(cyl_z(z_h1-1.8, z_h1+1, 3.1, cx=bxy[0], cy=bxy[1]))
    for az in (90.0, 270.0):                                                      # gearmotor face screws (M3, check motor)
        mx, my = polar(h['motor_screw_r'], az)
        hd = hd.cut(cyl_z(z_h0-3, z_h1+1, 1.7, cx=mx, cy=my)).cut(cyl_z(z_h1-1.8, z_h1+1, 3.1, cx=mx, cy=my))
    swl, sww, swh = h['sw']; z_sw0 = z_h1 - 3.0
    hd = hd.cut(bx(a-swl/2-0.2, a+swl/2+0.2, -sww/2-0.2, sww/2+0.2, z_sw0-0.2, z_h1+1.0))     # switch pocket
    for sx in (-h['sw_holes']/2, h['sw_holes']/2):
        hd = hd.cut(cyl_z(z_h0-3, z_h1+1, 1.1, cx=a+sx))                                      # M2 switch screws
    hd = hd.cut(cyl_z(z_h0-3, z_h1+1, 1.6, cx=a-swl/2-2.5))                                   # switch wires
    # turret: one ring with three radial slots for the wheels; each pillar between two slots is hollow (open on top, closed by
    # the dome) and its two flanks are the 9 mm cheeks that carry the pins. Printed plate-down, no supports.
    yi = W/2 + h['cheek_gap']; yo = yi + h['cheek_t']; Rt = h['head_r']; z_p = zc + h['turret_top']
    zis = [zc + sh for sh in shifts]; dsx, dsy = h['dome_scr']
    tur = cyl_z(z_h1-0.01, z_p, Rt, ri=h['turret_bore'])
    pock = cyl_z(z_h1, z_p+1, Rt-h['turret_wall'], ri=h['turret_bore']+2.5)
    for al in alphas:
        tur = tur.cut(bx(0, Rt+5, -yi, yi, z_h1, z_p+1).rotate(O, Z, al))
        pock = pock.cut(bx(0, Rt+5, -yo, yo, z_h1-1, z_p+2).rotate(O, Z, al))
    tur = tur.cut(pock)
    for al, zi in zip(alphas, zis):
        tur = tur.cut(cyl_y(-(yo+1), Rt+1, h['pin_d']/2, a, zi).rotate(O, Z, al))            # pin: pushed in from outside through the +y wall
        for s_ in (-1, 1):                                                                  # M3 inserts for the dome, in the cheek tops
            tur = tur.cut(cyl_z(z_p-h['insert_len']-0.2, z_p+1, h['insert_d']/2, cx=dsx, cy=s_*dsy).rotate(O, Z, al))
    hd = hd.union(tur)
    for az in h['boss_az']:                                                       # head screws sit on the pocket floor: key access from above
        bxy = polar(Ri-2.5, az)
        hd = hd.cut(cyl_z(z_h1-1.8, z_p+1, 3.3, cx=bxy[0], cy=bxy[1]))
    hd = hd.cut(cyl_x(-(Rt+1), -(h['turret_bore']-1), 2.3, 0, z_w1+5.0))                   # key access to the worm set screw (turn the worm to face it)
    hd = hd.cut(bx(a-swl/2-0.2, a+swl/2+0.2, -sww/2-0.2, sww/2+0.2, z_sw0-0.2, z_h1+1.0))
    parts['hand_head'] = hd.val()

    # ---- worm: hub on the motor shaft (M3 grub screw), thread, top end FREE ----
    ws = worm.build().rotate(O, Y, -90).translate(cq.Vector(0, 0, zc))
    zt0 = zc - h['worm_len']/2
    hub = cyl_z(z_w1, zt0, h['hub_d']/2).cut(cyl_z(z_w1-1, zt0+1, h['shaft_d']/2))
    hub = hub.cut(cyl_x(0, h['hub_d'], 1.65, 0, z_w1+5.0))                         # M4 set screw on the D flat
    worm_solid = ws.rotate(O, Z, h['theta0']*zt + 120.0).cut(cyl_z(zt0-1, zc+h['worm_len'], 3.0).val())      # 6 mm through bore

    # ---- fingers: wheel + root (PETG, one piece), Fin Ray (TPU), pins with e-clips ----
    psi, th0 = h['cant'], h['theta0']
    lobe_ang = 180.0 - h['theta_max']                    # lobe points straight down at the switch at full open
    wheel_c = wh.build(bore_d=h['pin_d']+0.2).translate(cq.Vector(0, 0, -W/2)).rotate(O, X, 90).rotate(O, Y, h['phase'])
    rt = root(h, W).val().rotate(O, Y, -psi)
    lobe = bx(-h['lobe_w']/2, h['lobe_w']/2, -4.0, 4.0, 13.0, h['lobe_r']).rotate(O, Y, lobe_ang)
    rigid0 = cq.Workplane('XY').newObject([wheel_c]).union(cq.Workplane('XY').newObject([rt])).cut(cyl_y(-W, W, h['pin_d']/2+0.1, 0, 0))
    rigid1 = rigid0.union(lobe).val().rotate(O, Y, th0)
    rigid = rigid0.val().rotate(O, Y, th0)
    fin_local = finray(h).val().translate(cq.Vector(0, 0, h['root_z']+0.05)).rotate(O, Y, -psi).rotate(O, Y, th0)
    info['fingers'] = {}
    sph = cq.Workplane('XY').sphere(h['apple_r']+0.3).translate((0, 0, zc+h['apple_dz'])).val()
    for i, (al, sh) in enumerate(zip(alphas, shifts), start=1):
        zi = zc + sh
        fi = (rigid1 if i == 1 else rigid).translate(cq.Vector(a, 0, zi)).rotate(O, Z, al)
        ff = fin_local.translate(cq.Vector(a, 0, zi)).rotate(O, Z, al)
        ca0, sa0 = math.cos(math.radians(al)), math.sin(math.radians(al))
        pv0 = (a*ca0, a*sa0, zi); pe0 = (pv0[0]-sa0, pv0[1]+ca0, zi)
        dg = h['theta_grasp'] - th0
        ff = ff.rotate(pv0, pe0, dg).cut(sph).rotate(pv0, pe0, -dg)               # flex part trimmed to the apple at the grasp pose
        parts[f'hand_finger_{i}'] = mirror(fi)
        parts[f'hand_finger_flex_{i}'] = mirror(ff)
        pl = h['pin_len']/2
        pin = cyl_y(-pl, pl, h['pin_d']/2, a, zi)                                 # plain dowel pin; the dome plugs stop it at both ends
        parts[f'hand_pin_{i}'] = mirror(pin.rotate(O, Z, al).val())
        am = al if h['right_hand'] else -al
        ca, sa = math.cos(math.radians(am)), math.sin(math.radians(am))
        info['fingers'][f'hand_finger_{i}'] = ((a*ca, a*sa, zi), (-sa, ca, 0.0) if h['right_hand'] else (-sa, ca, 0.0))   # axis: + rotation = fingers OPEN
        info['fingers'][f'hand_finger_flex_{i}'] = info['fingers'][f'hand_finger_{i}']
    # ---- limit switch (body in the pocket, actuator blade up to the lobe tip at full open) ----
    z_tip = zc - h['lobe_r']
    sw_body = bx(a-swl/2, a+swl/2, -sww/2, sww/2, z_sw0, z_h1+0.8)
    for sx in (-h['sw_holes']/2, h['sw_holes']/2):
        sw_body = sw_body.cut(cyl_z(z_sw0-1, z_h1+2, 1.1, cx=a+sx))
    blade = bx(a-h['blade_w']/2, a+h['blade_w']/2, -2.0, 2.0, z_h1+0.8, z_tip)
    parts['hand_limit_switch'] = sw_body.union(blade).val()
    parts['hand_worm'] = mirror(worm_solid)
    parts['hand_worm_hub'] = hub.val()
    # ---- palm dome: takes the worm thrust (the grip load pushes the worm UP) on an F6-12M thrust bearing and puts it straight into the turret ----
    zt1 = zc + h['worm_len']/2; bo_, bi_, bw_ = h['top_brg']; pt = h['palm_t']
    parts['hand_worm_top_pin_steel_6x16'] = cyl_z(zt1-9.0, zt1+0.3+pt, bi_/2).val()          # also runs in the palm roof (radial guide)
    parts['hand_brg_worm_top_F6-12M'] = cyl_z(zt1+0.5, zt1+0.5+bw_, bo_/2, bi_/2+0.01).val()
    z_top = zt1 + 0.3 + pt; rim = h['dome_rim']; ch_ = z_top - z_p - rim
    palm = cq.Workplane('XZ').polyline([(0, z_p), (Rt, z_p), (Rt, z_p+rim), (Rt-0.85*ch_, z_top), (0, z_top)]).close().revolve(360, (0, 0, 0), (0, 1, 0))
    palm = palm.cut(cyl_z(z_p-1, zt1+0.3, h['turret_bore']-1.0))                  # room for the worm
    plug = cyl_z(min(zis)-h['pin_d']/2-1.0, z_p+0.01, Rt-h['turret_wall']-0.3, ri=h['turret_bore']+2.8)   # plugs into the pillar pockets
    for al in alphas:
        palm = palm.cut(bx(bo_/2+1.6, Rt+5, -(yi+0.2), yi+0.2, z_p-1, z_top+1).rotate(O, Z, al))      # slots for wheels and finger roots
        plug = plug.cut(bx(0, Rt+5, -(h['pin_len']/2+0.4), h['pin_len']/2+0.4, z_p-30, z_p+1).rotate(O, Z, al))
    palm = palm.union(plug)
    palm = palm.cut(cyl_z(zt1, zt1+0.5+bw_+0.2, bo_/2+0.05)).cut(cyl_z(zt1, zt1+pt+1.0, bi_/2+0.1))
    for al in alphas:                                                             # 6x M3 x 10 into the cheek tops, beside the wheels
        for s_ in (-1, 1):
            palm = palm.cut(cyl_z(z_p-1, z_top+1, 1.7, cx=dsx, cy=s_*dsy).rotate(O, Z, al))
            palm = palm.cut(cyl_z(z_p+4.3, z_top+1, 3.1, cx=dsx, cy=s_*dsy).rotate(O, Z, al))
    parts['hand_palm'] = mirror(palm.val())
    apple = cq.Workplane('XY').sphere(h['apple_r']).translate((0, 0, zc+h['apple_dz'])).val()
    # ---- printable parts in print orientation (used by export_print.py) ----
    flat = lambda sh: sh.rotate(O, X, -90)                                     # wheel / finger thickness axis (Y) -> Z
    info['print'] = {
        'hand_body_PETG': (parts['hand_body'], 1),
        'hand_head_PETG': (parts['hand_head'], 1),
        'finger_root_with_lobe_PETG': (flat(rigid0.union(lobe).val()), 1),
        'finger_root_PETG': (flat(rigid0.val()), 2),
        'finray_TPU': (flat(finray(h).val()), 3),
        'palm_PETG': (palm.val().rotate(O, X, 180), 1),
    }
    tipL = h['root_z'] + h['fr_L']
    info.update(dict(s_w=1.0 if h['right_hand'] else -1.0, a=a, zc=zc, zt=zt, theta0=th0, theta_min=h['theta_min'], theta_max=h['theta_max'], cant=psi,
                     apple=(h['apple_r'], zc+h['apple_dz']), tip_z=zc+tipL*math.cos(math.radians(th0-psi)),
                     z_hub1=z_hub1, z_h1=z_h1, z_h0=z_h0))
    return parts, info, apple
