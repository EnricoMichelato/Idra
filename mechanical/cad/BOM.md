# Wireless Robot Arm: hardware list (arm v0.32, wrist v1.29, hand v2.6)

Bought parts are modelled from catalogue figures. **Buy the three geared units and the two flange hubs first, measure them, set their parameters in the scripts, then print the parts around them** (pedestal, deck, hand body).

Dimensions are indicative: check them when buying.

## Geared drives (bought, steel)
| Part | Qty | Where | Figures used in the CAD |
|---|---|---|---|
| NEMA23 2.8 A with integrated 20:1 high-precision planetary gearbox (23HS22-2804S-HG20 class) | 1 | Shoulder | motor 57 × 57 × 56, gearbox Ø60 × 61.65, shaft Ø14 × 34 with key, 25 Nm, backlash ≤ 15–25 arcmin, 1.8 kg |
| NEMA17 with integrated 5:1 high-precision planetary gearbox (17HS15-1684S-HG5 class) | 1 | Base | motor 39, gearbox 42 × 41, shaft Ø8 × 23.5 with key, backlash ≤ 15 arcmin. **Not the cheaper PG5 version: its backlash is up to 3°** |
| Bolt-on 5:1 planetary gearbox for NEMA17, high precision, 5 mm input bore (EG17-G5 class) | 2 | Elbow, on the salvaged motors | 42 × 42 × 41, shaft Ø8 × 23.5, face holes 31 × 31 M3 (assumed: measure) |
| 25 mm metal gear motor, 12 V, 75:1 (Pololu 25D MP class) | 1 | Hand | Ø25 × 54, 4 mm D shaft 12.5 long, 2 × M3 17 mm apart, 0.83 Nm stall, 100 rpm |
| Steel worm, module 1, single start, right hand, pitch Ø16 (KHK SW1-R1 class) | 1 | Hand | Ø18 × 32 with hub, 6 mm bore, M4 set screw. Cut the thread to 16 mm. Needs a 4/6 mm sleeve on the motor shaft, or buy one with a 4 mm bore |

## Other motors and drivers
| Part | Qty | Notes |
|---|---|---|
| NEMA17, 40 mm, 5 mm shaft (the four salvaged ones: measure them) | 4 | Elbow 2 (behind the 5:1 gearboxes), wrist 2 |
| Driver for 2.8 A (TMC5160 board or external DM542-type) | 1 | Shoulder: a TMC2209 cannot give the NEMA23 its full current |
| ESP32-S3 board + 5 × TMC2209 | 1 | In the foot: base, elbow A/B, wrist A/B |

## Bearings
| Type | Size (mm) | Qty | Where |
|---|---|---|---|
| 6801-2RS | 12 × 21 × 5 | 15 | Elbow: 4 in the side bevels, 2 in the bones, 1 in the tip of the forearm satellite. Wrist: 4 in the side bevels, 2 in ulna and radius, 2 on the roll shaft |
| 688-2RS | 8 × 16 × 5 | 6 | Elbow (2) and wrist (2) idler bevels, elbow motor shaft supports (2) |
| 625-2RS | 5 × 16 × 5 | 5 | Wrist motor shaft supports, in blocks on ulna and radius (2), rollers under the turntable rim (3) |
| 6806-2RS | 30 × 42 × 7 | 1 | Elbow cradle, under the satellite hub |
| 6810 (61810) | 50 × 65 × 7 | 5 | Base yaw spindle (2), forearm base on the cradle, left shoulder bone, ring around the wrist coils |
| F6-12M thrust ball bearing | 6 × 12 × 4.5 | 1 | Hand: takes the worm thrust in the palm dome |

The 6802 and 61814 of the printed cycloid are gone.

## Laser-cut aluminium (6 mm)
| Part | Size | Qty | Notes |
|---|---|---|---|
| Humerus, left and right | 80 × 278 | 1 + 1 | DXF files and finishing notes in `LASER_CUT.md` |
| Ulna and radius | 51 × 176 | 1 + 1 | as above |

## Shafts, hubs, pins, springs, magnets
| Part | Size | Qty | Notes |
|---|---|---|---|
| Spacer, aluminium, M3 clearance | Ø7 × 24 | 8 | Wrist motor stand-offs between motor face and bone (25 mm is the common length: file 1 mm off, or print them) |
| Flange hub, steel, 14 mm bore with keyway | flange Ø50, 4 holes on Ø38, about 28 long | 1 | Shoulder: on the gearbox shaft, bolted to the outer face of the right bone |
| Flange hub, aluminium, 12 mm bore, set screws | flange Ø32, 4 holes on Ø26, 12 high | 1 | Hand: on the roll shaft, screwed to the bottom of the hand |
| Ground shaft h6 | Ø12 × 145 | 1 | Elbow: clamped in the cradle walls with 2 grub screws, 1 mm flat |
| Ground shaft h6 | Ø12 × 143 | 1 | Wrist pitch axle, as above |
| Shaft, **non-magnetic** (304 stainless or 7075 aluminium) | Ø12 × 59 | 1 | Wrist roll shaft. A 2 mm flat 22 mm long under bevel C, a 1 mm flat at the hub end. The encoder magnet is glued on its lower end |
| Shaft, non-magnetic (304 stainless) | Ø8 × 32 | 1 | Elbow idler bevel; carries the pronation encoder magnet |
| Steel pin | Ø8 × 20 | 1 | Wrist idler bevel pin |
| Dowel pin | Ø6 × 16 | 1 | Worm top pin (Loctite in the worm bore) |
| Dowel pins | Ø5 × 32 | 3 | Hand fingers (held by the dome plugs, no clips) |
| Wave spring | 12 × 18, about 50 N at 2 mm | 4 | Side bevel preload |
| Spacer ring | 12 × 15 × 0.8 | 4 | On the inner ring of the pulley-side bearing |
| Spacer ring | 12 × 14.8 × 1.5 | 4 | Between each cradle or flange wall and the bearing in the bone (elbow 2, wrist 2); can be printed |
| Sleeve, M5 clearance | Ø9 × 10 | 6 | Between bone and belt idler (wrist 2 at 10 mm, elbow 4 at 9.5 mm); can be printed |
| Shim washers | DIN 988 12 × 18, 0.1–0.5 | assortment | Preload tuning |
| Diametric magnet | 6 × 2.5 | 4 | Encoders |

## Shoulder gravity balancer
| Part | Qty | Notes |
|---|---|---|
| Extension spring, wire Ø2.5, outer Ø20, body about 52 mm (20–22 coils), rate about 3.2–3.5 N/mm, max load ≥ 220 N, hooks no longer than 18 mm each | 1 | Along the back of the pedestal. A rate between 2.9 and 3.6 N/mm is fine: the balancing torque changes in proportion |
| U-groove bearing U624ZZ, 4 × 13 × 7 | 2 | Cable pulleys |
| Cable: Dyneema cord Ø2 or steel rope Ø1.5, about 300 mm, with two loops or ferrules | 1 | Load up to 220 N |
| M5 eye bolt, 30 mm, + 2 nuts | 1 | Spring anchor: sets the cable length |
| M5 × 12 shoulder or button-head screw + washer | 1 | Cable pin in the tapped lobe of the left bone |
| M4 × 30 + nyloc nut | 2 + 2 | Pulley axles, each held on both sides of the pulley |

## Transmission
| Part | Qty | Notes |
|---|---|---|
| Closed GT2 belt 10 mm, **steel cord**, 380 mm (190 teeth) | 1 | Elbow motor A (upper) |
| Closed GT2 belt 10 mm, **steel cord**, 460 mm (230 teeth) | 1 | Elbow motor B (lower). The two elbow motors sit one above the other, so their belts differ |
| Closed GT2 belt 10 mm, **steel cord**, 340 mm (170 teeth) | 2 | Wrist |
| GT2 pulley 20 teeth, 8 mm bore, for 10 mm belt | 2 | On the elbow gearbox shafts |
| GT2 pulley 14 teeth, 5 mm bore, for 10 mm belt | 2 | On the wrist NEMA17s (78/14 = 5.57:1). Less common than 16 teeth: check that the one you buy has a 5 mm bore and two set screws |

Steel-cord closed GT2 loops are less common than fibreglass ones. If a length is not available, change the tooth count in the script (it recomputes the centre distance), or fall back to genuine Gates fibreglass belts: the arm then sags about 3.4 mm instead of 1.9 mm under 500 g (`control/stiffness_report.md`).

| Part | Qty | Notes |
|---|---|---|
| Smooth flanged idler, 5 mm bore, for 10 mm belt (about Ø12) | 6 | Tensioners: M5 in a slot of the bone + flange nut |

## Electronics
| Part | Qty | Notes |
|---|---|---|
| 24 → 5 V step-down | 1 | |
| 24 V 200 W power supply, DC socket, switch, panel USB-C | 1 + 1 + 1 + 1 | 200 W because of the 2.8 A shoulder |
| Ø22 mushroom emergency button | 1 | Cuts the 24 V to the drivers |
| 40 mm 24 V fan | 1 | |
| 4-digit 0.36" 7-segment display (TM1637) | 1 | In the foot |
| AS5600 on a small PCB (16 × 18 or Ø12) | 4 | Small custom PCBs are needed |
| Hall sensor + magnet for base homing | 1 + 1 | |
| Seeed XIAO ESP32S3 Sense (camera) | 1 | On the wrist flange; also drives the TX bridge |
| PCB coils `idra_coil_TX20` + `idra_coil_RX24` (4 layers, 2 oz, Ø43.6 / Ø15.6 × 1.6), files in `electronics/coil_pcb/` | 1 + 1 | Wrist inductive link. Add `idra_coil_RX28` to the same order as a spare option |
| Flexible sintered ferrite sheet for wireless charging, 0.3 mm, with adhesive | 4 rings Ø44 / Ø15 | Two behind each coil board; cut from 50 × 50 mm sheets |
| Passive hand board: DRV8871, 4 × SS24, 47 µF 50 V, SMBJ30A TVS, 1 µF | 1 | Schematic in `electronics/hand_passive.md` |
| Hand filters: 2 × 2.2 mH SMD inductors, C0G 1.2 nF and 270 pF, 1206 resistors 470 Ω and 1 kΩ, 2 × 1N4148W, 2 × 4.7 nF, 2 × 10 kΩ, 2 × 3.3 V zeners, 2 × 47 kΩ, 2 × 5.6 V zeners, **R_ILIM 82 kΩ** | 1 set | R_ILIM sets the grip force (0.78 A) |
| KW10 micro switch (NC contact) | 1 | Hand limit switch |
| Flange TX bridge: IR2104, 2 × N-MOSFET ≥ 40 V, 78L12, US1D, 1 µF 100 V film, 0.1 Ω shunt, INA180A2 | 1 | Must deliver about 10 W for the stall of the hand: bench test |

## Screws and inserts
| Part | Qty (approx.) | Where |
|---|---|---|
| M3 heat-set inserts (Ø4.0 hole) | ~93 | Hand turret 6, Cover tabs 24, cradle plate 8, wrist flange plate 6, motor shaft blocks 8, forearm base 8, foot pilasters 7, deck 4, encoder brackets 4, camera and TX driver 4, forearm hub 3, hand head 3, hand hub 4 |
| M5 heat-set inserts (Ø6.4 hole) | 22 | Pedestal bottom 8, pedestal top (straps) 4, elbow motor carrier 4, balancer bracket 3 and anchor 3 |
| M3 × 10 socket head | ~74 | Hand dome 6, Covers 24 (through the bones into the cover tabs), motor shaft blocks 8 (through the bones), elbow gearboxes on the carrier 8, foot deck 7, yaw cartridge 4, yaw gearbox 4, brackets, camera and driver |
| M3 × 16 socket head | 8 | Ulna and radius on the forearm base |
| M3 × 12 self-tapping | 18 | Ring gear 6, yaw retainer 4, spindle clamp ring 4, left shoulder hub 4 (countersunk) |
| M3 × 12 countersunk | 14 | Cradle walls on the plate 8, wrist flange walls on the plate 6 |
| M3 × 35 + washer | 8 | Wrist NEMA17s, through bone and 24 mm spacer (about 4.5 mm of thread in the motor: check the depth of your motors) |
| M3 × 8 | 10 | Forearm base on the satellite hub 3, hand hub 4, hand head on the body 3 |
| M3 × 6 grub screw | 7 | Cradle 2, spider 1, wrist flange 2, T fitting 1, yaw pinion 1 |
| M5 × 20 | 8 | Pedestal, from under the turntable |
| M5 × 12 | 6 | Balancer bracket 3 and anchor 3, from under the turntable |
| M5 × 16 | 8 | Flange hub into the tapped right bone 4, elbow motor carrier through the bones 4 |
| M5 × 45 and M5 × 16 | 2 + 2 | Shoulder straps (motor strap, gearbox strap) |
| M5 × 35 + flange nut | 6 + 6 | Belt idler axles. The 4 at the elbow must be button head (ISO 7380): a socket head touches the opposite motor |
| M5 × 25 + nut | 3 + 3 | Rim roller axles |
| M5 × 20 + nut | 8 + 8 | Foot wings |
| M6 or desk clamp | 4 | Fixing the wings to the desk |
| M2 × 10 | 2 | Hand micro switch |
| M3 × 25 + nut + large washers | 6 | Fin Ray on the finger roots |

## Printed materials
- **Gears, bevels and pulleys, finger roots:** PETG-CF or PA-CF.
- **Structural parts** (cradle, flange, pedestal, carrier, covers, foot): PETG or PETG-CF.
- **Fingers:** TPU 95A.

## Still to complete in the CAD
- **Bought-part dimensions:** gearbox mounting faces, flange hub hole patterns and the worm are from catalogue figures, not measured.
- **Cable connectors** at the shoulder and at the forearm base.
- **Small-part fixings:** boards, sensors, connectors and cable clips have no modelled screws.
- **Not built yet:** nothing here has been printed. Start with the fit coupon and the bench tests in `ASSEMBLY.md`.
