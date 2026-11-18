"""PCB coils for the wrist link: flat spirals on a 4-layer ring board, written as Gerber + Excellon, KiCad board, DXF and SVG.

    python3 electronics/coil_pcb.py            # writes electronics/coil_pcb/<variant>/...

Board: a ring that drops into the Ø44 / Ø15 pocket of the flange (TX) or of the hand bottom (RX), in place of coil + cup rims.
Winding: layers 1+2 in parallel spiral inwards, layers 3+4 in parallel spiral back outwards in the same sense, so both
terminals are on the rim. The inner ends are joined by vias. Terminals: one solder pad each on both faces, 40 deg apart.

Everything is drawn from one list of primitives (tracks, vias, pads, circles), so the Gerbers and the KiCad board agree.
Starting values, not tested: see mechanical/cad/WRIST_BUILD.md.
"""
import math, os, uuid, json

HERE = os.path.dirname(os.path.abspath(__file__))
OUT = os.path.join(HERE, 'coil_pcb')

BOARD = dict(r_out=21.8, r_in=7.8,          # ring Ø43.6 / Ø15.6 (pocket Ø44.2 / Ø14.8)
             w_out=20.0, w_in=9.2,          # copper of the winding stays between these radii
             gap=0.25,                      # track to track (2 oz copper: fab minimum is about 0.2)
             r_pad=20.85, pad=(3.0, 1.2),   # terminal pads: centre radius, (tangential, radial) size
             r_mid=8.55,                    # radius of the vias that join the two halves at the inner end
             via=(0.6, 0.3),                # via pad, drill
             ang_a=20.0, ang_b=-20.0,       # terminal angles (deg)
             cu_um=70.0, thickness=1.6)
VARIANTS = {                                # name: (label on the silkscreen, turns per layer pair)
    'idra_coil_TX20': ('TX20', 10),
    'idra_coil_RX24': ('RX24', 12),
    'idra_coil_RX28': ('RX28', 14),
}
CU = ('F.Cu', 'In1.Cu', 'In2.Cu', 'B.Cu')
RHO = 1.72e-8                               # ohm m, copper at 20 C
STEP = math.radians(3.0)


def pol(r, a): return (r*math.cos(a), r*math.sin(a))


def design(label, n, b=BOARD):
    """-> dict(tracks=[(layer, width, [(x, y), ...])], vias=[(x, y)], pads=[(layer, cx, cy, rot_deg, sx, sy)], silk=..., info)"""
    p = (b['w_out'] - b['w_in'] + b['gap'])/(n + 1); w = p - b['gap']
    rco, rci = b['w_out'] - w/2, b['w_in'] + w/2
    ta, tb = math.radians(b['ang_a']), math.radians(b['ang_b'])
    d = (ta - tb) % (2*math.pi)
    tracks, vias, pads = [], [], []

    def spiral(t0, t1, r0, sign):
        k = max(2, int(math.ceil((t1 - t0)/STEP))); pts = []
        for i in range(k + 1):
            t = t0 + (t1 - t0)*i/k; pts.append(pol(r0 + sign*p*(t - t0)/(2*math.pi), t))
        return pts
    s_in = spiral(ta, ta + 2*math.pi*n, rco, -1)               # pair A: from terminal A inwards, counter-clockwise
    s_out = spiral(ta, ta + 2*math.pi*n - d, rci, +1)          # pair B: from the inner end outwards, same sense, ends at terminal B
    r_end = math.hypot(*s_out[-1])
    for ly in CU[:2]:
        tracks.append((ly, w, s_in)); tracks.append((ly, w, [pol(b['r_pad'], ta), pol(rco, ta)]))
    for ly in CU[2:]:
        tracks.append((ly, w, s_out)); tracks.append((ly, w, [pol(r_end, tb), pol(b['r_pad'], tb)]))
    wv = b['via'][0]
    arc = [pol(b['r_mid'], ta + math.radians(x)) for x in (-8, -4, 0, 4, 8)]
    for ly in CU:                                              # inner end: all four layers meet
        tracks.append((ly, wv, [pol(rci, ta), pol(b['r_mid'], ta)])); tracks.append((ly, wv, arc))
    for x in (-8, 0, 8): vias.append(pol(b['r_mid'], ta + math.radians(x)))
    for t in (ta, tb):                                         # terminals: pad on both faces, three vias in it
        for x in (-3, 0, 3): vias.append(pol(b['r_pad'], t + math.radians(x)))
        for ly in ('F.Cu', 'B.Cu'): pads.append((ly,) + pol(b['r_pad'], t) + (math.degrees(t) + 90.0,) + b['pad'])
    # which physical net each piece belongs to, for the clearance check
    length = sum(math.dist(a, c) for pts in (s_in, s_out) for a, c in zip(pts, pts[1:]))
    area = w*1e-3 * 2*b['cu_um']*1e-6
    do, di = 2*rco, 2*rci; rho_ = (do - di)/(do + di); dav = (do + di)/2e3
    L_air = 4e-7*math.pi*(2*n)**2*dav/2*(math.log(2.46/rho_) + 0.2*rho_**2)
    info = dict(label=label, turns=round(2*n - d/(2*math.pi), 2), per_pair=n, pitch=round(p, 3), track=round(w, 3), gap=b['gap'],
                length_m=round(length/1000, 3), r_dc=round(RHO*length/1000/area, 3),
                r_dc_inner_1oz=round(RHO*length/1000/(w*1e-3*1.5*b['cu_um']*1e-6), 3), L_air_uH=round(L_air*1e6, 1))
    return dict(tracks=tracks, vias=vias, pads=pads, silk=silk_text(label, b), info=info, s_in=s_in, s_out=s_out, w=w)


GLYPH = {'T': [[(0, 1), (0.6, 1)], [(0.3, 1), (0.3, 0)]], 'X': [[(0, 0), (0.6, 1)], [(0, 1), (0.6, 0)]],
         'R': [[(0, 0), (0, 1), (0.6, 1), (0.6, 0.5), (0, 0.5)], [(0.2, 0.5), (0.6, 0)]],
         '0': [[(0, 0), (0, 1), (0.6, 1), (0.6, 0), (0, 0)]], '2': [[(0, 1), (0.6, 1), (0.6, 0.5), (0, 0.5), (0, 0), (0.6, 0)]],
         '4': [[(0, 1), (0, 0.5), (0.6, 0.5)], [(0.6, 1), (0.6, 0)]], '8': [[(0, 0), (0, 1), (0.6, 1), (0.6, 0), (0, 0)], [(0, 0.5), (0.6, 0.5)]]}


def silk_text(label, b, h=1.0, r0=20.3, a0=180.0):
    """label along the rim, opposite the terminals; -> {layer: [polyline, ...]} (the bottom one reads correctly from below)"""
    out = {'F.SilkS': [], 'B.SilkS': []}
    adv = math.degrees(0.95*h/r0)
    for ly, mirror in (('F.SilkS', False), ('B.SilkS', True)):
        for i, ch in enumerate(label):
            k = i - (len(label) - 1)/2
            a = math.radians(a0 - k*adv if not mirror else a0 + k*adv)        # top: text runs clockwise so it reads from outside
            for st in GLYPH[ch]:
                pts = []
                for gx, gy in st:
                    u = (gx - 0.3)*h*(-1 if mirror else 1); v = gy*h
                    # local frame: v radial outwards, u along the reading direction (clockwise seen from the top)
                    pts.append(((r0 + v)*math.cos(a) + u*math.sin(a), (r0 + v)*math.sin(a) - u*math.cos(a)))
                out[ly].append(pts)
    return out


# ---------------------------------------------------------------- checks
def check(d, b=BOARD):
    """turn-to-turn gap and clearance between each terminal and the winding of the other pair, on every layer"""
    w = d['w']; res = {}
    def rad_at(pts, ang):     # radii at which a spiral crosses a given direction
        out = []
        for a, c in zip(pts, pts[1:]):
            a0, a1 = math.atan2(a[1], a[0]), math.atan2(c[1], c[0])
            dd = (ang - a0) % (2*math.pi); span = (a1 - a0) % (2*math.pi)
            if dd <= span and span > 0:
                f = dd/span; out.append(math.hypot(a[0] + f*(c[0]-a[0]), a[1] + f*(c[1]-a[1])))
        return sorted(out)
    for name, pts in (('pair_A', d['s_in']), ('pair_B', d['s_out'])):
        g = min(min(y - x for x, y in zip(rr, rr[1:])) for rr in (rad_at(pts, math.radians(t + 2.5)) for t in range(0, 360, 5)) if len(rr) > 1)
        res[name + '_gap'] = round(g - w, 3)
    half_t = math.degrees(b['pad'][0]/2/b['r_pad']) + 0.5
    for term, ang, other in (('A', b['ang_a'], d['s_out']), ('B', b['ang_b'], d['s_in'])):
        rmax = max(max(rad_at(other, math.radians(ang + x)) or [0]) for x in (-half_t, 0, half_t))
        res[f'pad_{term}_to_other_pair'] = round(b['r_pad'] - b['pad'][1]/2 - (rmax + w/2), 3)
    res['pad_to_edge'] = round(b['r_out'] - b['r_pad'] - b['pad'][1]/2, 3)
    res['mid_via_to_edge'] = round(b['r_mid'] - b['via'][0]/2 - b['r_in'], 3)
    return res


# ---------------------------------------------------------------- Gerber / Excellon
def _g(v): return f'{round(v*1e6):d}'


def gerber(path, function, tracks=(), flashes=(), polys=(), circles=(), polarity='Positive'):
    L = ['%TF.GenerationSoftware,IDRA,coil_pcb.py,1*%', f'%TF.FileFunction,{function}*%', f'%TF.FilePolarity,{polarity}*%',
         '%FSLAX46Y46*%', '%MOMM*%', '%LPD*%', 'G01*', 'G75*']
    aps = {}
    def ap(dia):
        k = round(dia, 4)
        if k not in aps:
            aps[k] = 10 + len(aps); L.append(f'%ADD{aps[k]}C,{k:.4f}*%')
        return aps[k]
    for wd in sorted({round(t[0], 4) for t in tracks} | {round(f[2], 4) for f in flashes} | {round(c[2], 4) for c in circles}): ap(wd)
    for wd, pts in tracks:
        L.append(f'D{ap(wd)}*'); L.append(f'X{_g(pts[0][0])}Y{_g(pts[0][1])}D02*')
        L += [f'X{_g(x)}Y{_g(y)}D01*' for x, y in pts[1:]]
    for x, y, dia in flashes:
        L.append(f'D{ap(dia)}*'); L.append(f'X{_g(x)}Y{_g(y)}D03*')
    for pts in polys:
        L.append('G36*'); L.append(f'X{_g(pts[0][0])}Y{_g(pts[0][1])}D02*')
        L += [f'X{_g(x)}Y{_g(y)}D01*' for x, y in pts[1:] + pts[:1]]; L.append('G37*')
    for r, _, wd in circles:                                   # full circle about the origin
        L.append(f'D{ap(wd)}*'); L.append(f'X{_g(r)}Y0D02*'); L.append(f'G03X{_g(r)}Y0I{_g(-r)}J0D01*'); L.append('G01*')
    L.append('M02*')
    open(path, 'w').write('\n'.join(L) + '\n')


def rect(cx, cy, rot, sx, sy, grow=0.0):
    c, s = math.cos(math.radians(rot)), math.sin(math.radians(rot)); hx, hy = sx/2 + grow, sy/2 + grow
    # the pad's long side (sx) is tangential: rot is the direction of that side
    return [(cx + u*c - v*s, cy + u*s + v*c) for u, v in ((-hx, -hy), (hx, -hy), (hx, hy), (-hx, hy))]


def write_gerbers(d, name, folder, b=BOARD):
    os.makedirs(folder, exist_ok=True)
    ext = {'F.Cu': ('F_Cu.gtl', 'Copper,L1,Top'), 'In1.Cu': ('In1_Cu.g2', 'Copper,L2,Inr'), 'In2.Cu': ('In2_Cu.g3', 'Copper,L3,Inr'),
           'B.Cu': ('B_Cu.gbl', 'Copper,L4,Bot')}
    for ly, (e, fn) in ext.items():
        gerber(os.path.join(folder, f'{name}-{e}'), fn, tracks=[(w, pts) for l, w, pts in d['tracks'] if l == ly],
               flashes=[(x, y, b['via'][0]) for x, y in d['vias']], polys=[rect(*p[1:]) for p in d['pads'] if p[0] == ly])
    for ly, e, fn in (('F.Cu', 'F_Mask.gts', 'Soldermask,Top'), ('B.Cu', 'B_Mask.gbs', 'Soldermask,Bot')):
        gerber(os.path.join(folder, f'{name}-{e}'), fn, polys=[rect(*p[1:], grow=0.05) for p in d['pads'] if p[0] == ly], polarity='Negative')
    for ly, e, fn in (('F.SilkS', 'F_Silkscreen.gto', 'Legend,Top'), ('B.SilkS', 'B_Silkscreen.gbo', 'Legend,Bot')):
        gerber(os.path.join(folder, f'{name}-{e}'), fn, tracks=[(0.15, pts) for pts in d['silk'][ly]])
    gerber(os.path.join(folder, f'{name}-Edge_Cuts.gm1'), 'Profile,NP', circles=[(b['r_out'], 0, 0.1), (b['r_in'], 0, 0.1)])
    L = ['M48', '; DRILL file, plated through holes', 'FMAT,2', 'METRIC', f'T1C{b["via"][1]:.3f}', '%', 'G90', 'G05', 'T1']
    L += [f'X{x:.3f}Y{y:.3f}' for x, y in d['vias']] + ['T0', 'M30']
    open(os.path.join(folder, f'{name}-PTH.drl'), 'w').write('\n'.join(L) + '\n')


# ---------------------------------------------------------------- KiCad board (version 7 format; KiCad 7, 8 and 9 open it)
def write_kicad(d, name, path, b=BOARD, ox=100.0, oy=100.0):
    def P(x, y): return f'{ox + x:.4f} {oy - y:.4f}'           # KiCad y points down
    def ts(): return f'(tstamp {uuid.uuid4()})'
    L = ['(kicad_pcb (version 20221018) (generator idra_coil_pcb)', f'  (general (thickness {b["thickness"]}))', '  (paper "A4")',
         '  (layers', '    (0 "F.Cu" signal)', '    (1 "In1.Cu" signal)', '    (2 "In2.Cu" signal)', '    (31 "B.Cu" signal)',
         '    (36 "B.SilkS" user "B.Silkscreen")', '    (37 "F.SilkS" user "F.Silkscreen")', '    (38 "B.Mask" user)', '    (39 "F.Mask" user)',
         '    (44 "Edge.Cuts" user)', '    (49 "F.Fab" user)', '  )', '  (setup (pad_to_mask_clearance 0.05))', '  (net 0 "")', '  (net 1 "COIL")']
    for i, t in enumerate((b['ang_a'], b['ang_b'])):           # terminals as one footprint each, pad on both faces
        cx, cy = pol(b['r_pad'], math.radians(t)); rot = (t + 90.0) % 360
        ref = 'AB'[i]
        L += [f'  (footprint "idra:coil_terminal" (layer "F.Cu") {ts()} (at {P(cx, cy)} {rot:.2f})', '    (attr smd exclude_from_pos_files exclude_from_bom)',
              f'    (fp_text reference "{ref}" (at 0 0 {rot:.2f}) (layer "F.Fab") hide (effects (font (size 0.8 0.8) (thickness 0.12))) {ts()})',
              f'    (fp_text value "coil terminal" (at 0 0 {rot:.2f}) (layer "F.Fab") hide (effects (font (size 0.8 0.8) (thickness 0.12))) {ts()})',
              f'    (pad "1" smd rect (at 0 0 {rot:.2f}) (size {b["pad"][0]} {b["pad"][1]}) (layers "F.Cu" "F.Mask") (net 1 "COIL") {ts()})',
              f'    (pad "1" smd rect (at 0 0 {rot:.2f}) (size {b["pad"][0]} {b["pad"][1]}) (layers "B.Cu" "B.Mask") (net 1 "COIL") {ts()})', '  )']
    for ly, w, pts in d['tracks']:
        for a, c in zip(pts, pts[1:]):
            L.append(f'  (segment (start {P(*a)}) (end {P(*c)}) (width {w:.4f}) (layer "{ly}") (net 1) {ts()})')
    for x, y in d['vias']:
        L.append(f'  (via (at {P(x, y)}) (size {b["via"][0]}) (drill {b["via"][1]}) (layers "F.Cu" "B.Cu") (net 1) {ts()})')
    for ly, strokes in d['silk'].items():
        for pts in strokes:
            for a, c in zip(pts, pts[1:]):
                L.append(f'  (gr_line (start {P(*a)}) (end {P(*c)}) (stroke (width 0.15) (type solid)) (layer "{ly}") {ts()})')
    for r in (b['r_out'], b['r_in']):
        L.append(f'  (gr_circle (center {P(0, 0)}) (end {P(r, 0)}) (stroke (width 0.1) (type solid)) (fill none) (layer "Edge.Cuts") {ts()})')
    L.append(')')
    open(path, 'w').write('\n'.join(L) + '\n')


# ---------------------------------------------------------------- DXF (R12) and SVG
def write_dxf(d, path, b=BOARD):
    L = ['0', 'SECTION', '2', 'ENTITIES']
    for r in (b['r_out'], b['r_in']): L += ['0', 'CIRCLE', '8', 'OUTLINE', '10', '0', '20', '0', '40', f'{r:.4f}']
    for x, y in d['vias']: L += ['0', 'CIRCLE', '8', 'DRILL', '10', f'{x:.4f}', '20', f'{y:.4f}', '40', f'{b["via"][1]/2:.4f}']
    for ly, w, pts in d['tracks']:
        L += ['0', 'POLYLINE', '8', ly.replace('.', '_'), '66', '1', '40', f'{w:.4f}', '41', f'{w:.4f}']
        for x, y in pts: L += ['0', 'VERTEX', '8', ly.replace('.', '_'), '10', f'{x:.4f}', '20', f'{y:.4f}']
        L += ['0', 'SEQEND']
    for p in d['pads']:
        q = rect(*p[1:]); L += ['0', 'POLYLINE', '8', p[0].replace('.', '_') + '_PAD', '66', '1', '70', '1']
        for x, y in q: L += ['0', 'VERTEX', '8', p[0].replace('.', '_') + '_PAD', '10', f'{x:.4f}', '20', f'{y:.4f}']
        L += ['0', 'SEQEND']
    L += ['0', 'ENDSEC', '0', 'EOF']
    open(path, 'w').write('\n'.join(L) + '\n')


def write_svg(d, path, layers, title, b=BOARD):
    col = {'F.Cu': '#c0392b', 'In1.Cu': '#d98a1f', 'In2.Cu': '#2a7f9e', 'B.Cu': '#2c5aa0'}
    R = b['r_out'] + 1.5
    S = [f'<svg xmlns="http://www.w3.org/2000/svg" viewBox="{-R} {-R} {2*R} {2*R}" width="900" height="900">',
         f'<title>{title}</title>', f'<rect x="{-R}" y="{-R}" width="{2*R}" height="{2*R}" fill="#f4f1ea"/>',
         f'<path fill="#0f3d2e" fill-rule="evenodd" d="M{b["r_out"]},0 A{b["r_out"]},{b["r_out"]} 0 1 0 {-b["r_out"]},0 A{b["r_out"]},{b["r_out"]} 0 1 0 {b["r_out"]},0 Z '
         f'M{b["r_in"]},0 A{b["r_in"]},{b["r_in"]} 0 1 0 {-b["r_in"]},0 A{b["r_in"]},{b["r_in"]} 0 1 0 {b["r_in"]},0 Z"/>']
    for ly in layers:
        op = 0.9 if len(layers) == 1 else 0.6
        for l, w, pts in d['tracks']:
            if l == ly:
                S.append(f'<polyline fill="none" stroke="{col[ly]}" stroke-opacity="{op}" stroke-width="{w:.3f}" stroke-linecap="round" stroke-linejoin="round" points="'
                         + ' '.join(f'{x:.3f},{-y:.3f}' for x, y in pts) + '"/>')
        for p in d['pads']:
            if p[0] == ly: S.append(f'<polygon fill="#d9b45a" points="' + ' '.join(f'{x:.3f},{-y:.3f}' for x, y in rect(*p[1:])) + '"/>')
    for x, y in d['vias']:
        S.append(f'<circle cx="{x:.3f}" cy="{-y:.3f}" r="{b["via"][0]/2}" fill="#d9b45a"/><circle cx="{x:.3f}" cy="{-y:.3f}" r="{b["via"][1]/2}" fill="#111"/>')
    if 'F.Cu' in layers:
        for pts in d['silk']['F.SilkS']:
            S.append('<polyline fill="none" stroke="#fff" stroke-width="0.15" points="' + ' '.join(f'{x:.3f},{-y:.3f}' for x, y in pts) + '"/>')
    S.append('</svg>')
    open(path, 'w').write('\n'.join(S) + '\n')


def main():
    summary = {}
    for name, (label, n) in VARIANTS.items():
        d = design(label, n); folder = os.path.join(OUT, name)
        write_gerbers(d, name, os.path.join(folder, 'gerber'))
        write_kicad(d, name, os.path.join(folder, f'{name}.kicad_pcb'))
        write_dxf(d, os.path.join(folder, f'{name}.dxf'))
        write_svg(d, os.path.join(folder, f'{name}_all_layers.svg'), CU, name)
        for ly in CU: write_svg(d, os.path.join(folder, f'{name}_{ly.replace(".", "_")}.svg'), (ly,), f'{name} {ly}')
        d['info']['checks'] = check(d); summary[name] = d['info']
        print(name, json.dumps(d['info']))
    json.dump(summary, open(os.path.join(OUT, 'summary.json'), 'w'), indent=1)


if __name__ == '__main__':
    main()
