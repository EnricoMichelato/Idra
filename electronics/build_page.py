"""Inserts the SVG schematics into the hand_passive.html page."""
import re
def svg(path, pad=60):
    s = open(path, encoding='utf-8').read()
    s = s[s.index('<svg'):]
    m = re.search(r'viewBox="([-\d.e]+) ([-\d.e]+) ([-\d.e]+) ([-\d.e]+)"', s)
    x, y, w, h = map(float, m.groups())
    s = s.replace(m.group(0), f'viewBox="{x-pad} {y-10} {w+2*pad} {h+20}"')
    s = re.sub(r'\s(width|height)="[^"]*"', '', s, count=2)
    s = s.replace('fill:false', 'fill:none').replace('fill:white', 'fill:var(--panel)')
    s = s.replace('font-family="sans"', '')
    return s
t = open('hand_passive_tpl.html', encoding='utf-8').read()
t = t.replace('__RX_SVG__', svg('rx_hand.svg')).replace('__TX_SVG__', svg('tx_flange.svg'))
open('hand_passive.html', 'w', encoding='utf-8').write(t)
print('ok', len(t))
