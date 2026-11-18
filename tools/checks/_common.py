"""Shared helpers for the interference checks: repo paths and a pairwise solid intersection test."""
import os, sys, time, itertools
ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
for sub in ('mechanical/cad/arm_v0', 'mechanical/cad/wrist_v0', 'mechanical/cad/common', 'control'):
    sys.path.insert(0, os.path.join(ROOT, sub))
LIMIT = 0.5          # mm3: anything above is reported as a hit


def apart(a, b):
    return a.xmax < b.xmin or b.xmax < a.xmin or a.ymax < b.ymin or b.ymax < a.ymin or a.zmax < b.zmin or b.zmax < a.zmin


def clashes(q, want, skip=lambda a, b: False, label=''):
    """q: {name: shape} in one pose. want(a, b): pair worth testing. Prints and returns the hits."""
    t = time.time(); names = list(q); bbs = {n: q[n].BoundingBox() for n in names}; hits = []
    for a, b in itertools.combinations(names, 2):
        if not want(a, b) or skip(a, b) or apart(bbs[a], bbs[b]): continue
        try: vol = q[a].intersect(q[b]).Volume()
        except Exception: vol = float('nan')
        if not vol < LIMIT:
            hits.append((a, b, round(vol, 2))); print(f'  hit  {a}  {b}  {vol:.2f} mm3', flush=True)
    print(f'{label}: {len(hits)} hits, {time.time()-t:.0f} s', flush=True)
    return hits
