"""Kinematics self-test: viewer formula vs fk, closed-form wrist centre, IK round trips on random poses, motor mixing.

    python3 tools/checks/check_kinematics.py
"""
import sys; import os; sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))), 'control'))
import numpy as np, kinematics as K
from kinematics import Rx,Rz,Tz
rng=np.random.default_rng(1)
# 1) viewer replica: piv(m)=T(ZE) m T(-ZE); fore=Yw Ms Ec piv(rz(ph)); MW=fore T(WZ); MH=MW rx(wp) rz(wr)
def viewer(q):
    ps,sg,ep,ph,wp,wr=q; ZE=K.LU; WZ=K.LU+K.LF
    piv=lambda m: Tz(ZE)@m@Tz(-ZE)
    fore=Rz(ps)@Rx(-sg)@piv(Rx(-ep))@piv(Rz(ph))
    return fore@Tz(WZ)@Rx(wp)@Rz(wr)@Tz(K.D6)
e=max(np.abs(viewer(q)-K.fk(q)).max() for q in rng.uniform(-170,170,(500,6)))
print('viewer vs fk max err',e)
# 2) closed-form wrist centre
e=max(np.abs(K.wrist_center(*q[:3])-K.fk_frames(q)['wrist'][:3,3]).max() for q in rng.uniform(-170,170,(500,6)))
print('wrist centre err',e)
# 3) IK round trip on all solutions, random poses within limits
lo=np.array([v[0] for v in K.LIMITS.values()][:5]); hi=np.array([v[1] for v in K.LIMITS.values()][:5])
worst=0; nf=0; nsol=[]
for _ in range(3000):
    q=np.r_[rng.uniform(lo,hi), rng.uniform(-720,720)]
    T=K.fk(q); S=K.ik_all(T[:3,3],T[:3,:3]); nsol.append(len(S))
    for s in S: worst=max(worst,np.abs(K.fk(s)-T).max())
    b=K.ik(T[:3,3],T[:3,:3],q_prev=q+rng.normal(0,3,6))
    if b is None or np.abs(K.fk(b)-T).max()>1e-6: nf+=1
    elif np.abs(b-q).max()>1e-6 and abs(q[4])>1: nf+=0  # other branch allowed
print('ik_all worst residual',worst,'solutions/min/max',min(nsol),max(nsol),'ik fails',nf)
# 4) best solution recovers original when started close
rec=0
for _ in range(1000):
    q=np.r_[rng.uniform(lo+5,hi-5), rng.uniform(-720,720)]
    if abs(q[4])<3: continue
    T=K.fk(q); b=K.ik(T[:3,3],T[:3,:3],q_prev=q+rng.normal(0,2,6))
    rec+= np.abs(b-q).max()<1e-6
print('recovered same branch',rec)
# 5) singular wrist
q=np.array([20,30,40,25,0,100.]); T=K.fk(q); b=K.ik(T[:3,3],T[:3,:3],q_prev=[20,30,40,10,0,0])
print('singular',b.round(4),np.abs(K.fk(b)-T).max())
# 6) mixing round trip + mixing vs viewer side gear angles
for q in rng.uniform(-170,170,(200,6)):
    assert np.allclose(K.motors_to_joints(K.joints_to_motors(q)),q)
q=[0,0,10,40,15,30]; m=K.joints_to_motors(q)
print('side A elbow', m[2]/K.R_EL, 'viewer', -10+0.5*40, ' wrist A', m[4]/K.R_WR,'viewer',15+30)
print('steps/deg', [round(r*K.STEPS_REV/360,2) for r in (K.R_YAW,K.R_SH,K.R_EL,K.R_WR)])
# 7) hand_frame orthonormal & tilt
R=K.hand_frame([100,300,-100],30,20); print('orth',np.abs(R.T@R-np.eye(3)).max(),np.linalg.det(R))
# workspace
print('reach wrist',K.LU+K.LF,'grip max',K.LU+K.LF+K.D6)
q=[-50,34.73,93.67,0,-51.59,0]; T=K.fk(q); print('anim pick grip',T[:3,3].round(2), 'z desk',round(T[2,3]+K.SHOULDER_H,1), 'zaxis',T[:3,2].round(3))
