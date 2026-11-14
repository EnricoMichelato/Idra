"""Schematics of the passive hand (RX in the hand) and of the TX driver on the wrist flange.
Writes rx_hand.svg and tx_flange.svg. Starting values, to be confirmed by the bench test."""
import schemdraw
import schemdraw.elements as elm

schemdraw.use('svg')

schemdraw.config(fontsize=11, lw=1.3, font='sans-serif')


def rx_hand(path='rx_hand.svg'):
    with schemdraw.Drawing(file=path, show=False) as d:
        d.config(unit=2.0)
        # ---- bridge rectifier: W = GND (anodes), E = VM (cathodes), N/S = AC from the coil ----
        R = d.add(elm.Rectifier(labels=['SS24', 'SS24', 'SS24', 'SS24']).at((0, 0)))
        N, S, E, W = R.N, R.S, R.E, R.W
        xc = W[0] - 2.4                                      # coil column
        d.add(elm.Line().at(N).to((xc, N[1])))
        coil = d.add(elm.Inductor2(loops=4).at((xc, N[1])).to((xc, S[1]))
                     .label('RX coil\nLitz, Ø44 cup', loc='top', ofst=0.3))
        d.add(elm.Line().at(S).to((xc, S[1])))
        d.add(elm.Dot().at((xc, N[1])))
        # GND rail from W, down to y_g
        y_g = S[1] - 2.4
        d.add(elm.Line().at(W).to((W[0], y_g)))
        d.add(elm.Dot().at((W[0], y_g)))
        # VM rail from E
        x = E[0]
        d.add(elm.Line().at(E).right(1.0)); vm = d.here
        d.add(elm.Label().at(vm).label('VM  10–15 V', loc='top', ofst=0.1))
        for kind, lab in (('cp', '47 µF\n50 V'), ('tvs', 'TVS\nSMBJ30A'), ('c', '1 µF')):
            d.add(elm.Dot().at(vm))
            if kind == 'tvs':     # drawn from GND up to VM: anode on GND, cathode on VM
                d.add(elm.Zener().at((vm[0], y_g)).to(vm).label(lab, loc='bottom'))
            else:
                d.add((elm.Capacitor(polar=True) if kind == 'cp' else elm.Capacitor()).at(vm).to((vm[0], y_g)).label(lab, loc='bottom'))
            d.add(elm.Line().at(vm).right(2.0)); vm = d.here
        d.add(elm.Line().at(vm).right(0.6)); vm = d.here
        ic = d.add(elm.Ic(pins=[elm.IcPin(name='ILIM', side='left', slot='1/4'),
                                elm.IcPin(name='IN2', side='left', slot='2/4'),
                                elm.IcPin(name='IN1', side='left', slot='3/4'),
                                elm.IcPin(name='VM', side='left', slot='4/4'),
                                elm.IcPin(name='OUT2', side='right', slot='2/4'),
                                elm.IcPin(name='OUT1', side='right', slot='3/4'),
                                elm.IcPin(name='GND', side='bottom')],
                           size=(3.0, 4.4), leadlen=0.5).theta(0).label('DRV8871', loc='top', fontsize=12)
                   .anchor('VM').at(vm))
        d.add(elm.Line().at(ic.GND).to((ic.GND[0], y_g)))
        d.add(elm.Line().at((W[0], y_g)).to((ic.GND[0], y_g)))
        d.add(elm.Ground().at((W[0] - 0.0, y_g)))
        d.add(elm.Line().at(ic.ILIM).left(0.5)); p = d.here
        d.add(elm.Resistor().at(p).to((p[0], y_g)).label('R_ILIM\n82 kΩ', loc='top'))
        d.add(elm.Dot().at((p[0], y_g)))
        d.add(elm.Line().at(ic.OUT1).right(1.0)); o1 = d.here
        d.add(elm.Line().at(ic.OUT2).right(1.0)); o2 = d.here
        d.add(elm.Motor().at(o1).to(o2).label('Ø22 gear motor', loc='bottom', ofst=0.4))

        # ---- frequency detectors, fed from the top coil node (it swings 0..VM against GND) ----
        def detector(y, lv, cv, rv, txt, switch):
            d.add(elm.Line().at((xc, N[1])).to((xc, y)))
            if y != max(yA, yB):
                d.add(elm.Dot().at((xc, y)))
            d.add(elm.Inductor().at((xc, y)).right(1.6).label(lv, loc='top'))
            d.add(elm.Capacitor().right(1.4).label(cv, loc='top')); n = d.here
            d.add(elm.Dot().at(n))
            d.add(elm.Resistor().at(n).down(1.4).label(rv, loc='bottom')); d.add(elm.Ground())
            d.add(elm.Diode().at(n).right(2.0).label('1N4148W', loc='top')); n = d.here
            d.add(elm.Dot().at(n))
            d.add(elm.Capacitor().at(n).down(1.4).label('4.7 nF', loc='bottom')); d.add(elm.Ground())
            d.add(elm.Line().at(n).right(1.9)); n = d.here
            d.add(elm.Dot().at(n))
            d.add(elm.Resistor().at(n).down(1.4).label('10 kΩ', loc='bottom')); d.add(elm.Ground())
            d.add(elm.Zener().at(n).right(1.6).reverse().label('3.3 V', loc='top')); n = d.here
            if switch:
                d.add(elm.Switch(nc=True).at(n).right(1.6).label('NC limit switch', loc='top')); n = d.here
            d.add(elm.Line().at(n).right(0.6)); n = d.here
            d.add(elm.Dot().at(n))
            d.add(elm.Resistor().at(n).down(1.4).label('47 kΩ', loc='bottom')); d.add(elm.Ground())
            d.add(elm.Line().at(n).right(1.9)); n = d.here
            d.add(elm.Dot().at(n))
            d.add(elm.Zener().at(n).down(1.4).reverse().label('5.6 V', loc='bottom')); d.add(elm.Ground())
            d.add(elm.Line().at(n).right(0.6)); n = d.here
            d.add(elm.Label().at(n).label(txt, loc='right', ofst=0.1))
            return n
        yB = N[1] + 3.2
        yA = yB + 3.4
        pB = detector(yB, '2.2 mH', '270 pF', '1 kΩ', 'IN2  open  (≈ 207 kHz)', True)
        pA = detector(yA, '2.2 mH', '1.2 nF', '470 Ω', 'IN1  close  (≈ 98 kHz)', False)
        # off-page references to the DRV inputs
        d.add(elm.Label().at(ic.IN1).label('IN1 ', loc='left', halign='right', ofst=0.0))
        d.add(elm.Label().at(ic.IN2).label('IN2 ', loc='left', halign='right', ofst=0.0))


def tx_flange(path='tx_flange.svg'):
    with schemdraw.Drawing(file=path, show=False) as d:
        d.config(unit=2.0)
        yT, yG, xQ, xT = 12.0, 2.0, 10.0, 13.0              # 24 V rail, supply GND rail, FET column, tank column
        d.add(elm.Line().at((8.0, yT)).to((16.5, yT)))
        d.add(elm.Label().at((8.0, yT)).label('24 V  ', loc='left', halign='right'))
        d.add(elm.Line().at((3.0, yG)).to((16.5, yG)))
        d.add(elm.Ground().at((12.0, yG)))
        for x, el, lab in ((15.0, elm.Capacitor(polar=True), '100 µF\n35 V'), (16.5, elm.Capacitor(), '1 µF')):
            d.add(elm.Dot().at((x, yT))); d.add(elm.Dot().at((x, yG)))
            d.add(el.at((x, yT)).to((x, yG)).label(lab, loc='bottom'))
        # half bridge (gate on the left)
        d.add(elm.Dot().at((xQ, yT)))
        d.add(elm.Line().at((xQ, yT)).to((xQ, 9.6)))
        q1 = d.add(elm.NFet().theta(0).reverse().at((xQ, 9.6)).anchor('drain').label('Q1', loc='bottom', ofst=-0.2))
        sw = (xQ, q1.source[1] - 0.6)
        d.add(elm.Line().at(q1.source).to(sw)); d.add(elm.Dot().at(sw))
        d.add(elm.Line().at(sw).to((xQ, sw[1] - 0.6)))
        q2 = d.add(elm.NFet().theta(0).reverse().at((xQ, sw[1] - 0.6)).anchor('drain').label('Q2', loc='bottom', ofst=-0.2))
        yL = q2.source[1] - 0.8
        d.add(elm.Line().at(q2.source).to((xQ, yL)))
        # tank: SW -> film cap -> TX coil -> local ground
        d.add(elm.Line().at(sw).to((xT, sw[1])))
        d.add(elm.Label().at(((xQ + xT) / 2, sw[1])).label('SW', loc='top', ofst=0.05))
        d.add(elm.Capacitor().at((xT, sw[1])).down(1.4).label('1 µF film\n100 V', loc='bottom'))
        d.add(elm.Inductor2(loops=4).to((xT, yL)).label('TX coil', loc='bottom'))
        d.add(elm.Line().at((xQ, yL)).to((xT, yL)))
        d.add(elm.Dot().at((xQ, yL)))
        d.add(elm.Label().at(((xQ + xT) / 2, yL)).label('local ground', loc='bottom', ofst=0.1))
        d.add(elm.Resistor().at((xQ, yL)).to((xQ, yG)).label('0.1 Ω\nshunt', loc='bottom'))
        d.add(elm.Dot().at((xQ, yG)))
        # gate driver
        ic = d.add(elm.Ic(pins=[elm.IcPin(name='COM', side='left', slot='1/4'),
                                elm.IcPin(name='SD', side='left', slot='2/4'),
                                elm.IcPin(name='IN', side='left', slot='3/4'),
                                elm.IcPin(name='VCC', side='left', slot='4/4'),
                                elm.IcPin(name='LO', side='right', slot='1/4'),
                                elm.IcPin(name='VS', side='right', slot='2/4'),
                                elm.IcPin(name='HO', side='right', slot='3/4'),
                                elm.IcPin(name='VB', side='right', slot='4/4')],
                           size=(2.2, 4.4), leadlen=0.6).theta(0).label('IR2104', loc='bottom', fontsize=12)
                   .anchor('HO').at((6.0, q1.gate[1])))
        d.add(elm.Resistor().at(ic.HO).to(q1.gate).label('10 Ω', loc='top'))
        d.add(elm.Line().at(ic.LO).to((7.4, ic.LO[1])))
        d.add(elm.Line().at((7.4, ic.LO[1])).to((7.4, q2.gate[1])))
        d.add(elm.Resistor().at((7.4, q2.gate[1])).to(q2.gate).label('10 Ω', loc='bottom'))
        d.add(elm.Line().at(ic.VS).right(0.5)); d.add(elm.Label().label(' SW', loc='right', halign='left'))
        d.add(elm.Line().at(ic.VB).up(0.9)); vb = d.here
        d.add(elm.Dot().at(vb))
        d.add(elm.Diode().at(vb).left(1.8).reverse().label('US1D', loc='top'))
        d.add(elm.Label().label('12 V ', loc='left', halign='right'))
        d.add(elm.Capacitor().at(vb).right(1.4).label('1 µF', loc='top'))
        d.add(elm.Label().label(' SW', loc='right', halign='left'))
        d.add(elm.Line().at(ic.VCC).left(0.6)); d.add(elm.Label().label('12 V (78L12 from 24 V) ', loc='left', halign='right'))
        d.add(elm.Line().at(ic.IN).left(0.6)); d.add(elm.Label().label('XIAO: PWM 100 / 200 kHz, 50 % ', loc='left', halign='right'))
        d.add(elm.Line().at(ic.SD).left(0.6)); d.add(elm.Label().label('XIAO: enable ', loc='left', halign='right'))
        d.add(elm.Line().at(ic.COM).left(0.6)); d.add(elm.Label().label('local ground ', loc='left', halign='right'))
        # current sense across the shunt
        amp = d.add(elm.Ic(pins=[elm.IcPin(name='IN+', side='right', slot='2/2'), elm.IcPin(name='IN−', side='right', slot='1/2'),
                                 elm.IcPin(name='OUT', side='left')],
                           size=(1.8, 1.6), leadlen=0.4).theta(0).label('INA180A2 ×50', loc='top', fontsize=10)
                    .anchor('IN+').at((8.2, yG + 1.5)))
        p = amp['IN+']; d.add(elm.Line().at(p).to((9.4, p[1]))); d.add(elm.Line().at((9.4, p[1])).to((9.4, yL)))
        d.add(elm.Line().at((9.4, yL)).to((xQ, yL)))
        p = amp['IN−']; d.add(elm.Line().at(p).to((9.0, p[1]))); d.add(elm.Line().at((9.0, p[1])).to((9.0, yG)))
        d.add(elm.Dot().at((9.0, yG)))
        d.add(elm.Resistor().at(amp.OUT).left(1.4).label('1 kΩ', loc='top')); n = d.here
        d.add(elm.Dot().at(n))
        d.add(elm.Capacitor().at(n).to((n[0], yG)).label('100 nF', loc='bottom'))
        d.add(elm.Dot().at((n[0], yG)))
        d.add(elm.Line().at(n).left(0.6)); d.add(elm.Label().label('XIAO ADC: current ', loc='left', halign='right'))


def themable(path):
    """Black strokes and text -> currentColor, so the page can theme the drawing."""
    import re
    s = open(path, encoding='utf-8').read()
    s = re.sub(r'(?i)(stroke|fill)="(black|#000000|#000)"', r'\1="currentColor"', s)
    s = re.sub(r'(?i)(stroke|fill):\s*(black|#000000|#000)', r'\1:currentColor', s)
    open(path, 'w', encoding='utf-8').write(s)


if __name__ == '__main__':
    rx_hand()
    tx_flange()
    for p in ('rx_hand.svg', 'tx_flange.svg'):
        themable(p)
