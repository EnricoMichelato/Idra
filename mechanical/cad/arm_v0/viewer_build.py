"""Builds arm_3d.html from the CAD. `python3 viewer_build.py --tpl` reuses the cached meshes (out/viewer_cache.json)
and only re-applies the template, which takes seconds instead of rebuilding the whole model."""
import sys, os; sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import json, base64, numpy as np
HERE = os.path.dirname(os.path.abspath(__file__)); CACHE = os.path.join(HERE, 'out', 'viewer_cache.json')
if '--tpl' in sys.argv and os.path.exists(CACHE):
    cache = json.load(open(CACHE)); data, K = cache['data'], cache['K']
    if '--recolor' in sys.argv:                      # colours changed in the CAD scripts: update the cache without re-meshing
        import arm_v0 as A
        for d in data:
            h = '#4b4d51' if d['n'] == 'desk' else None if d['n'] == 'obj' else A.color_of(d['n'])
            if h: d['c'] = [round(int(h.lstrip('#')[i:i+2], 16)/255, 3) for i in (0, 2, 4)]
            if d['n'] not in ('desk', 'obj'): d['m'] = A.spec_of(d['n'])
        json.dump({'data': data, 'K': K}, open(CACHE, 'w'))
else:
    import arm_v0 as A, cadquery as cq
    parts, info = A.build(A.P)
    data, tris = [], 0
    for n, s in parts.items():
        wr_ = n.startswith('W_')
        v, t = s.tessellate(0.8, 0.6) if 'cable' in n else s.tessellate(0.2 if wr_ else 0.15, 0.3)
        if not t: continue
        V = np.array([[p.x, p.y, p.z] for p in v], dtype='<f4'); T = np.array(t, dtype='<u4'); tris += len(T)
        h = A.color_of(n).lstrip('#'); rgb = [round(int(h[i:i+2], 16)/255, 3) for i in (0, 2, 4)]
        ex = {}
        if wr_ and n[2:] in info['wrist']['hand']['fingers']:
            pv, ax = info['wrist']['hand']['fingers'][n[2:]]; ex = dict(pv=[round(x, 4) for x in pv], ax=[round(x, 4) for x in ax])
        if n.startswith('elec_seg_'):
            _, _, d_, sn = n.split('_'); ex['sg'] = [int(d_), 'abcdefg'.index(sn)]
        data.append(dict(n=n, g=A.group(n), k=A.category(n), c=rgb, m=A.spec_of(n), **ex,
                         p=base64.b64encode(V.tobytes()).decode(), i=base64.b64encode(T.tobytes()).decode()))
    # extras: the 80 mm reference apple (hand frame) and a desk under the foot
    extra = [('obj', 'object', '#c0392b', info['wrist']['apple_shape']),
             ('desk', 'desk', '#4b4d51', cq.Workplane('XY').circle(430).extrude(8).translate((0, 120, -info['shoulder_height']-8)).val())]
    for g_, k_, col, s in extra:
        v, t = s.tessellate(0.3, 0.3)
        V = np.array([[p.x, p.y, p.z] for p in v], dtype='<f4'); T = np.array(t, dtype='<u4')
        h = col.lstrip('#'); rgb = [round(int(h[i:i+2], 16)/255, 3) for i in (0, 2, 4)]
        data.append(dict(n=g_, g=g_, k=k_, c=rgb, p=base64.b64encode(V.tobytes()).decode(), i=base64.b64encode(T.tobytes()).decode()))
    K = {'__ZE__': round(info['ze'], 3), '__RB1__': round(info['ratio_belt'], 4), '__RB2__': 1.0, '__RCAP__': info['ratio_cyc'],
         '__RYAW__': round(info['ratio_yaw'], 4), '__KP__': info['kp'], '__YPX__': round(info['yaw_pin_xy'][0], 3),
         '__YPY__': round(info['yaw_pin_xy'][1], 3), '__YP2X__': round(info['yaw_pin2_xy'][0], 3), '__YP2Y__': round(info['yaw_pin2_xy'][1], 3), '__YEX__': round(info['yaw_enc_xy'][0], 3), '__YEY__': round(info['yaw_enc_xy'][1], 3), '__RYE__': round(info['ratio_yaw_enc'], 4), '__SH__': info['shoulder_height'], '__WZ__': round(info['wrist_z'], 3),
         '__SPY__': round(A.P['sh']['y'], 3), '__SPZ__': round(info['sh_motor_z'], 3), '__D6__': round(info['wrist']['hand']['apple'][1], 3), '__TIP__': round(info['wrist']['grip_top'], 3)}
    os.makedirs(os.path.dirname(CACHE), exist_ok=True); json.dump({'data': data, 'K': K}, open(CACHE, 'w'))
    print(len(data), 'parts', tris, 'triangles')
html = open(os.path.join(HERE, 'viewer_tpl.html')).read().replace('__DATA__', json.dumps(data, separators=(',', ':')))
for k, v in K.items():
    html = html.replace(k, repr(v))
open(os.path.join(HERE, 'arm_3d.html'), 'w').write(html)
print('arm_3d.html', round(len(html)/1e6, 2), 'MB')
