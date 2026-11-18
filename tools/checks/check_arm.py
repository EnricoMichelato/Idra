"""Interference check of the whole arm in one or more poses (tests every pair that involves a moving part).

    python3 tools/checks/check_arm.py                       # the standard set of poses (slow: about 5 min per pose)
    python3 tools/checks/check_arm.py 60,90,105,120,-60,45  # yaw, shoulder, elbow, pronation, wrist pitch, wrist roll (deg)

Exit code 1 if anything clashes. Cables are drawn approximately and are left out.
"""
import sys
from _common import clashes
import arm_v0 as A

POSES = ('0,0,0,0,0,0', '60,90,105,120,-60,45', '-30,-60,90,-170,90,0', '45,30,-60,90,-90,170', '90,60,0,-170,45,0')
MOVING = ('foot_', 'yaw_', 'arm_', 'cradle_', 'forearm_', 'elbow_', 'idle_', 'W_')


def want(a, b):
    if 'cable' in a or 'cable' in b or a.startswith('elec_seg') or b.startswith('elec_seg'): return False
    if a.startswith('yaw_cyc_pin') and b.startswith('yaw_cyc_pin'): return False
    return a.startswith(MOVING) or b.startswith(MOVING) or 'nut_M5' in a + b


def main(poses):
    parts, info = A.build(A.P); n = 0
    for arg in poses:
        q = A.pose(parts, info, *[float(x) for x in arg.split(',')])
        n += len(clashes(q, want, A.SKIP, label=f'pose {arg}'))
    return n


if __name__ == '__main__':
    sys.exit(1 if main(sys.argv[1:] or POSES) else 0)
