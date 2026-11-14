"""Export the printable hand parts as STL in print orientation (lowest point on z = 0) to ./out/print."""
import os, sys
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import cadquery as cq

def main(outdir='out/print'):
    os.makedirs(outdir, exist_ok=True)
    import wrist_v0 as W
    _, winfo = W.build(W.P)                       # hand built in the wrist: includes the RX coil recess and the skirt
    pr = dict(winfo['hand']['print'])
    worm, hub = pr.pop('worm_LH_PETG')[0], pr.pop('worm_hub_PETG')[0]
    wh = cq.Workplane('XY').newObject([worm]).union(cq.Workplane('XY').newObject([hub])).val()
    ok = wh.isValid() and wh.Volume() > 0.98*(worm.Volume() + hub.Volume()) - 50
    pr['worm_LH_with_hub_PETG'] = (wh if ok else cq.Compound.makeCompound([worm, hub]), 1)
    for name, (sh, qty) in pr.items():
        bb = sh.BoundingBox()
        sh = sh.translate(cq.Vector(-(bb.xmin+bb.xmax)/2, -(bb.ymin+bb.ymax)/2, -bb.zmin))
        fn = f'{outdir}/{name}_x{qty}.stl'
        cq.exporters.export(cq.Workplane('XY').newObject([sh]), fn, tolerance=0.02, angularTolerance=0.15)
        bb = sh.BoundingBox()
        print(f'{name:28s} x{qty}  {bb.xlen:6.1f} x {bb.ylen:6.1f} x {bb.zlen:6.1f} mm  vol {sh.Volume()/1000:6.1f} cm3  valid {sh.isValid()}')

if __name__ == '__main__':
    main(sys.argv[1] if len(sys.argv) > 1 else 'out/print')
