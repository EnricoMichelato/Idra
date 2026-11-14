"""
Fit coupon for the Wireless Robot Arm: print it once, find on each row the column that fits, copy the value into fit.py.
Columns: -1 / 0 / +1 = the current fit.py value minus / equal / plus one step. Run: python3 fit_test.py -> out/fit_test.stl
Print flat, same material and settings as the robot parts (PETG-CF or PA-CF, 0.2 mm, 4 perimeters).
"""
import os, cadquery as cq
from fit import FIT

T = 7.0                                       # plate thickness = width of the widest bearing (6806)
COLS = (-1, 0, 1)
# (label, kind, nominal, fit key, step) ; kind: seat = bearing OD press seat, hole = plain hole, boss = printed boss into a bearing bore
ROWS = [
    ('6806', 'seat', 42.0, 'brg', 0.10),
    ('6801', 'seat', 21.0, 'brg', 0.10),
    ('688/625', 'seat', 16.0, 'brg', 0.10),
    ('6801 IN', 'boss', 12.0, 'shaft', 0.10),
    ('PIN 12', 'hole', 12.0, 'rot', 0.20),
    ('PIN 8', 'hole', 8.0, 'rot', 0.20),
    ('INS M5', 'hole', 0.0, 'ins5', 0.20),
    ('INS M3', 'hole', 0.0, 'ins3', 0.20),
    ('M5', 'hole', 0.0, 'm5', 0.20),
    ('M3', 'hole', 0.0, 'm3', 0.20),
]
LABEL_W = 24.0
THICK = {'6806': 7.0, '6801': 5.0, '688/625': 5.0}          # seats as deep as the bearing; the rest 6 mm (insert length)

def build():
    plate, y = None, 0.0
    for label, kind, nom, key, step in ROWS:
        d_max = (nom + FIT[key] + step) if nom else FIT[key] + step
        cw = d_max + (16.0 if kind == 'boss' else 9.0)              # column width
        h = max(d_max + (16.0 if kind == 'boss' else 8.0), 15.0)
        t = THICK.get(label, 6.0)
        strip = cq.Workplane('XY').box(LABEL_W + 3*cw, h, t, centered=False).translate((0, y, 0)).edges('|Z').fillet(1.5)
        for ci, c in enumerate(COLS):
            v = FIT[key] + c*step
            d = nom + v if nom else v
            cx, cy = LABEL_W + cw*(ci+0.5), y + h/2 + 1.5
            if kind == 'boss':      # pocket with a printed boss in the middle: the bearing bore slides on it
                strip = strip.cut(cq.Workplane('XY').workplane(offset=1.0).center(cx, cy).circle(d/2 + 5.5).extrude(t))
                strip = strip.union(cq.Workplane('XY').workplane(offset=0.5).center(cx, cy).circle(d/2).extrude(t - 0.5))
            else:
                strip = strip.cut(cq.Workplane('XY').workplane(offset=-1).center(cx, cy).circle(d/2).extrude(t + 2))
            txt = f'{c:+d}' if c else '0'
            strip = strip.cut(cq.Workplane('XY').workplane(offset=t-0.5).center(cx, y + 1.2).text(txt, 2.6, 1.0, halign='center', valign='bottom'))
        strip = strip.cut(cq.Workplane('XY').workplane(offset=t-0.5).center(2.0, y + h/2).text(label, 3.0, 1.0, halign='left', valign='center'))
        plate = strip if plate is None else plate.union(strip)
        y += h - 0.01
    return plate

if __name__ == '__main__':
    os.makedirs('out', exist_ok=True)
    p = build()
    bb = p.val().BoundingBox()
    cq.exporters.export(p, 'out/fit_test.stl', tolerance=0.05, angularTolerance=0.1)
    print('fit_test.stl', round(bb.xlen, 1), 'x', round(bb.ylen, 1), 'x', round(bb.zlen, 1), 'mm, valid', p.val().isValid())
