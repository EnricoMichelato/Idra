"""Collect everything needed to build the wrist and the hand alone (bench build) into mechanical/cad/out/wrist_kit.

    python3 tools/export_print.py      # first: oriented STLs of the whole arm
    python3 tools/export_dxf.py        # and the DXFs of the bones
    python3 tools/wrist_kit.py

It also copies the PCB coil files (Gerber zips, KiCad, pictures). Besides the normal printed parts it exports the forearm bones (ulna, radius) as STL, so the wrist can be tried
with printed bones before the aluminium ones are cut, and the small spacers that can be printed instead of bought.
"""
import os, sys, shutil, glob
ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, 'mechanical/cad/wrist_v0'))
import cadquery as cq
import wrist_v0 as W

CAD = os.path.join(ROOT, 'mechanical/cad')
OUT = os.path.join(CAD, 'out/wrist_kit')
WRIST = ('carrier_flange', 'carrier_wall_', 'gear_A_pulley', 'gear_C_output', 'idle_gear_D', 'T_fitting', 'motor_block_', 'enc_pitch_bracket',
         'camera_mount', 'forearm_base_plate', 'ulna', 'radius', 'tube_bolt', 'standoff_')
EXTRA = {  # name in the wrist model -> (file name, copies, normal that goes up on the bed)
    'idler_A_spacer': ('idler_sleeve_9x10_x2.stl', (1, 0, 0)),
    'carrier_spacer_L': ('axle_spacer_ring_1.5_x2.stl', (1, 0, 0)),
}


def flat(shape, up):
    if abs(up[0]) > 0.5: shape = shape.rotate(cq.Vector(0, 0, 0), cq.Vector(0, 1, 0), -90.0)
    b = shape.BoundingBox()
    return shape.translate(cq.Vector(-(b.xmin+b.xmax)/2, -(b.ymin+b.ymax)/2, -b.zmin))


def main():
    shutil.rmtree(OUT, ignore_errors=True)
    for d in ('print/wrist', 'print/hand', 'print/temporary', 'laser', 'electronics'): os.makedirs(os.path.join(OUT, d))
    n = 0
    for f in sorted(glob.glob(os.path.join(CAD, 'out/print/*.stl'))):
        b = os.path.basename(f)
        dst = 'print/hand' if b.startswith('hand_') else 'print/wrist' if b.startswith(WRIST) else None
        if dst: shutil.copy(f, os.path.join(OUT, dst, b)); n += 1
    parts, _ = W.build(W.P)
    for name, (fn, up) in EXTRA.items():
        sh = flat(parts[name], up); b = sh.BoundingBox()
        cq.exporters.export(cq.Workplane('XY').newObject([sh]), os.path.join(OUT, 'print/temporary', fn), tolerance=0.03, angularTolerance=0.2)
        print(f'{fn:34s} {b.xlen:6.1f} x {b.ylen:6.1f} x {b.zlen:5.1f}'); n += 1
    for f in (): shutil.copy(os.path.join(CAD, 'out/laser', f), os.path.join(OUT, 'laser', f))
    for f in ('hand_passive.md', 'rx_hand.svg', 'tx_flange.svg'): shutil.copy(os.path.join(ROOT, 'electronics', f), os.path.join(OUT, 'electronics', f))
    cp = os.path.join(ROOT, 'electronics/coil_pcb')                      # PCB coils: README, ready-to-upload Gerber zips, pictures, KiCad
    os.makedirs(os.path.join(OUT, 'electronics/coil_pcb'))
    shutil.copy(os.path.join(cp, 'README.md'), os.path.join(OUT, 'electronics/coil_pcb/README.md'))
    for f in sorted(glob.glob(os.path.join(cp, '*/*_gerber.zip')) + glob.glob(os.path.join(cp, '*/*.kicad_pcb')) + glob.glob(os.path.join(cp, '*/*_all_layers.svg'))):
        shutil.copy(f, os.path.join(OUT, 'electronics/coil_pcb', os.path.basename(f)))
    for src, dst in (('WRIST_BUILD.md', 'WRIST_BUILD.md'), ('wrist_v0/hand_print_notes.md', 'hand_print_notes.md'), ('wrist_v0/wrist_3d.html', 'wrist_3d.html'),
                     ('common/fit_test.md', 'fit_test.md')):
        p = os.path.join(CAD, src)
        if os.path.exists(p): shutil.copy(p, os.path.join(OUT, dst))
    print(n, 'STL files in', OUT)


if __name__ == '__main__':
    main()
