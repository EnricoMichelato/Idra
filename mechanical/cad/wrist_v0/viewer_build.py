import json, base64, numpy as np, wrist_v0 as W
parts, info = W.build(W.P)
def cat(n):
    if n.startswith('cable'): return 'cables'
    if n.startswith(('ulna', 'radius', 'forearm_cover')): return 'bones'
    if n.startswith(('belt', 'mpulley', 'idler', 'tens_')): return 'belts'
    if n.startswith(('motor', 'screw', 'spacer', 'brg_motor')): return 'motors'
    if n.startswith('enc_'): return 'bones'
    if n == 'apple_ref': return 'apple'
    if n.startswith('hand_') or n in ('coil_RX', 'ferrite_RX'): return 'hand'
    if n == 'carrier_flange' or n.startswith(('camera', 'brg_ring', 'tx_driver')): return 'flange'
    return 'core'
data, tris = [], 0
parts = dict(parts); parts['apple_ref'] = info['apple_shape']
for n, s in parts.items():
    v, t = s.tessellate(0.8, 0.6) if n.startswith('cable') else s.tessellate(0.12, 0.25)
    if not t: continue
    V = np.array([[p.x, p.y, p.z] for p in v], dtype='<f4'); T = np.array(t, dtype='<u4')
    tris += len(T)
    h = ('c4452a' if n == 'apple_ref' else W.color_of(n).lstrip('#')); rgb = [round(int(h[i:i+2], 16)/255, 3) for i in (0, 2, 4)]
    ex = {}
    if n in info['hand']['fingers']:
        pv, ax = info['hand']['fingers'][n]; ex = dict(pv=[round(x, 4) for x in pv], ax=[round(x, 4) for x in ax])
    data.append(dict(n=n, g=W.group(n) if n != 'apple_ref' else 'roll', k=cat(n), c=rgb, m=(0.25 if n == 'apple_ref' else W.spec_of(n)), **ex,
                     p=base64.b64encode(V.tobytes()).decode(), i=base64.b64encode(T.tobytes()).decode()))
html = open('viewer_tpl.html').read().replace('__DATA__', json.dumps(data, separators=(',', ':')))
open('wrist_3d.html', 'w').write(html)
print(len(data), 'parts', tris, 'triangles', round(len(html)/1e6, 2), 'MB')
