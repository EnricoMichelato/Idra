"""Printability audit: for every printed part, bed fit, best print orientation (least unsupported overhang), mass and
thin spots. Writes out/print_audit.json and prints a table.   python3 tools/print_audit.py"""
import os, sys, json, math, time
import numpy as np
ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, 'mechanical/cad/arm_v0')); sys.path.insert(0, os.path.join(ROOT, 'mechanical/cad/wrist_v0'))
import arm_v0 as A

BED, ZMAX = 256.0, 256.0
DIRS = {'+z': (0, 0, 1), '-z': (0, 0, -1), '+x': (1, 0, 0), '-x': (-1, 0, 0), '+y': (0, 1, 0), '-y': (0, -1, 0)}   # "up" axis

def mesh(s):
    v, t = s.tessellate(0.25, 0.3)
    return np.array([[p.x, p.y, p.z] for p in v]), np.array(t)

def fits(P2):
    """smallest square-bed margin over in-plane rotations (deg): returns (fits, best angle, extents)"""
    best = None
    for a in range(0, 90, 3):
        c, s = math.cos(math.radians(a)), math.sin(math.radians(a))
        u = P2[:, 0]*c + P2[:, 1]*s; w = -P2[:, 0]*s + P2[:, 1]*c
        e = (u.max()-u.min(), w.max()-w.min())
        if best is None or max(e) < max(best[1]): best = (a, e)
    return max(best[1]) <= BED - 4, best[0], best[1]

def orient(V, T, up):
    up = np.array(up, float)
    a, b, c = V[T[:, 0]], V[T[:, 1]], V[T[:, 2]]
    n = np.cross(b-a, c-a); ar = np.linalg.norm(n, axis=1)/2; ok = ar > 1e-9
    n = n[ok]/(2*ar[ok, None]); ar = ar[ok]; cen = ((a+b+c)/3)[ok]
    h = V @ up; h0 = h.min(); hc = cen @ up - h0
    down = -(n @ up)                                   # 1 = facing straight down
    bed = (down > 0.99) & (hc < 0.15)
    over = (down > math.cos(math.radians(45))) & ~bed  # steeper than 45 deg from vertical, not on the bed
    flat = (down > 0.985) & ~bed                       # horizontal ceilings: bridges or supports
    # in-plane coordinates
    ref = np.array([1, 0, 0.]) if abs(up[0]) < 0.9 else np.array([0, 1, 0.])
    e1 = np.cross(up, ref); e1 /= np.linalg.norm(e1); e2 = np.cross(up, e1)
    f, ang, ext = fits(np.c_[V @ e1, V @ e2])
    return dict(height=float(h.max()-h0), bed_area=float(ar[bed].sum()), over=float(ar[over].sum()), flat=float(ar[flat].sum()),
                fits=bool(f and h.max()-h0 <= ZMAX), rot=ang, ext=[round(float(x), 1) for x in ext])

def candidates(V, T):
    """up-axes to try: the 6 axis directions plus 'largest flat faces on the bed' (covers parts built at an angle)"""
    a, b, c = V[T[:, 0]], V[T[:, 1]], V[T[:, 2]]
    n = np.cross(b-a, c-a); ar = np.linalg.norm(n, axis=1)/2; ok = ar > 1e-9; n = n[ok]/(2*ar[ok, None]); ar = ar[ok]
    key = np.round(n, 2); uniq, inv = np.unique(key, axis=0, return_inverse=True); inv = inv.ravel()
    area = np.bincount(inv, weights=ar)
    out = dict(DIRS)
    for i in np.argsort(-area)[:4]:
        u = -(n[inv == i]*ar[inv == i, None]).sum(axis=0); u = u/np.linalg.norm(u)       # exact mean normal of that face
        if not any(np.dot(u, np.array(d)) > 0.99999 for d in out.values()):
            out['face%d' % i] = tuple(float(x) for x in u)
    return out

def best_orientation(s):
    V, T = mesh(s); cand = candidates(V, T)
    res = {k: orient(V, T, d) for k, d in cand.items()}
    ok = [k for k in res if res[k]['fits']] or list(res)
    best = min(ok, key=lambda k: res[k]['over'] - 0.2*res[k]['bed_area'])
    return best, cand[best], res

def chirality(V, T):
    """mirror-sensitive signature, so that left/right parts are not merged as copies"""
    cen = (V[T[:, 0]] + V[T[:, 1]] + V[T[:, 2]])/3; cen = cen - cen.mean(axis=0)
    w, E = np.linalg.eigh(cen.T @ cen)
    if np.linalg.det(E) < 0: E[:, 0] = -E[:, 0]
    s3 = ((cen @ E)**3).mean(axis=0); pr = float(s3[0]*s3[1]*s3[2])
    scale = float(np.prod(np.abs(w))**0.5/len(cen)**1.5) + 1e-9
    return 0 if abs(pr) < 1e-4*scale else (1 if pr > 0 else -1)

def shape_key(s, V, T):
    """orientation-independent signature of a part: equal for copies, different for mirror images"""
    I = s.matrixOfInertia(s) if False else None
    cen = (V[T[:, 0]] + V[T[:, 1]] + V[T[:, 2]])/3; cen = cen - cen.mean(axis=0)
    w = np.linalg.eigvalsh(cen.T @ cen/len(cen))
    bb = s.BoundingBox()
    return (round(s.Volume(), -1), round(s.Area(), -1), tuple(np.round(np.sqrt(np.abs(w)), 0)), chirality(V, T))

def main():
    t0 = time.time(); parts, info = A.build(A.P); print('built', len(parts), 'parts in', round(time.time()-t0), 's', flush=True)
    rows = []
    for n, s in parts.items():
        m = A.material(n)
        if m not in ('printed', 'gear', 'tpu'):
            continue
        best, up, res = best_orientation(s); r = res[best]; vol = s.Volume()/1000
        rows.append(dict(name=n, material=m, solids=len(s.Solids()), valid=bool(s.isValid()), vol_cm3=round(vol, 1),
                         up=best, fits=r['fits'], ext=r['ext'], height=round(r['height'], 1), bed_cm2=round(r['bed_area']/100, 1),
                         overhang_cm2=round(r['over']/100, 1), ceiling_cm2=round(r['flat']/100, 1)))
    rows.sort(key=lambda r: -r['overhang_cm2'])
    os.makedirs(os.path.join(ROOT, 'mechanical/cad/arm_v0/out'), exist_ok=True)
    json.dump(rows, open(os.path.join(ROOT, 'mechanical/cad/arm_v0/out/print_audit.json'), 'w'), indent=1)
    print(f"{'part':38s} mat     vol   up     fits  ext(mm)        h    bed  overh  ceil  solids")
    for r in rows:
        print(f"{r['name'][:38]:38s} {r['material'][:6]:6s} {r['vol_cm3']:6.1f}  {r['up'][:5]:5s}  {'ok ' if r['fits'] else 'NO '}  {str(r['ext']):14s} {r['height']:5.0f} {r['bed_cm2']:6.1f} {r['overhang_cm2']:6.1f} {r['ceiling_cm2']:5.1f}  {r['solids']}{'' if r['valid'] else ' INVALID'}")
    print('printed parts', len(rows), 'total volume', round(sum(r['vol_cm3'] for r in rows)), 'cm3')

if __name__ == '__main__':
    main()
