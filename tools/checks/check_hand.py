"""Hand check: every solid valid, and no interference with the fingers closed, half open and fully open."""
import sys
from _common import clashes
import wrist_v0 as W


def main():
    parts, info = W.build(W.P); hi = info['hand']
    bad = [n for n, s in parts.items() if not s.isValid() or len(s.Solids()) != 1]
    print('parts', len(parts), 'invalid', bad, '| theta min/open/max', hi['theta_min'], hi['theta0'], hi['theta_max'], '| tip z', round(hi['tip_z'], 1))
    want = lambda a, b: (a.startswith('hand') or b.startswith('hand') or 'roll_shaft' in a + b) and 'cable' not in a + b
    tips = lambda a, b: a.startswith('hand_finger_flex') and b.startswith('hand_finger_flex')   # closed: the soft tips touch on purpose
    n = len(bad)
    for th in (hi['theta_min'], hi['theta0'], hi['theta_max']):
        n += len(clashes(W.pose(parts, 0, 30, theta=th, info=info), want, tips, label=f'fingers at {th:.1f} deg'))
    return n


if __name__ == '__main__':
    sys.exit(1 if main() else 0)
