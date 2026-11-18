"""Interference check of the wrist (forearm, differential, flange, coils, hand body) at several pitch / roll angles.

    python3 tools/checks/check_wrist.py              # standard set
    python3 tools/checks/check_wrist.py 90,45        # pitch, roll (deg)
"""
import sys
from _common import clashes
import wrist_v0 as W

POSES = ('0,0', '90,45', '-90,170', '45,-90')


def want(a, b): return 'belt' not in a + b and 'cable' not in a + b


def main(poses):
    parts, info = W.build(W.P); n = 0
    for arg in poses:
        pd, rd = [float(x) for x in arg.split(',')]
        n += len(clashes(W.pose(parts, pd, rd, info=info), want, label=f'pitch {pd:+.0f} roll {rd:+.0f}'))
    return n


if __name__ == '__main__':
    sys.exit(1 if main(sys.argv[1:] or POSES) else 0)
