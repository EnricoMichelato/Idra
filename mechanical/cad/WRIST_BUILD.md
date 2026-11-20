# Wrist and hand: bench build (wrist v1.38, hand v2.6)

Everything needed to build the forearm, the wrist differential and the hand alone, on the bench, before the rest of the arm exists. Since wrist v1.32 the whole forearm and wrist structure is printed (PETG-CF): only bearings, shafts, belts, pulleys, motors and screws are bought.
`python3 tools/wrist_kit.py` collects the files into `mechanical/cad/out/wrist_kit/`.

Nothing here has been built yet. Bought-part sizes are catalogue figures: **buy first, measure, then print the parts around them.**

## What exists and what does not

| Item | State |
|---|---|
| Printed parts, bones, fasteners | modelled, checked for interference, STLs oriented for printing |
| Hand electronics (RX) and flange driver (TX) | schematic with starting values (`electronics/`), **no PCB layout**: build them on prototyping board |
| Coils | **4-layer PCB rings** (`electronics/coil_pcb/`), ready to order; turn counts are a first calculation, to be confirmed on the bench |
| Firmware | **none yet**: for the first motion use any stepper test firmware, or an old printer board (below) |
| AS5600 encoder boards | need small boards (16 × 18 mm and Ø12); the common 23 mm modules do not fit. Leave the encoders out of the first build |

## Work in four stages

1. **Fit coupon.** Print `common/fit_test` first and put the measured clearances in `common/fit.py` (`fit_test.md`). Bearing seats and insert holes depend on it.
2. **Wrist mechanics.** Bones, two motors, belts, differential, flange, roll shaft. Goal: pitch ±90° and endless roll by hand and with the motors, belts that do not jump, backlash measured.
3. **Hand mechanics.** Motor fed straight from a 12 V bench supply with a current limit of 0.8 A. Goal: grip force on a kitchen scale (about 16 N per finger expected), limit switch, self-locking.
4. **Inductive link.** PCB coils and both boards on the bench first, with a 15 Ω load, then in the wrist. Order the coil boards together with the electronics.

Stages 2 and 3 need no custom electronics, so they can start as soon as the mechanical parts arrive.

## Print

Settings as in `PRINT_LIST.md`: structural parts in PETG or PETG-CF (0.2 mm, 4 perimeters, 40% gyroid), gears in PETG-CF or PA-CF (6 perimeters, 60–100%), fingers in TPU 95A.

### Wrist (`print/wrist/`)
| File | Qty | Class | Notes |
|---|---|---|---|
| `carrier_flange_x1.stl` | 1 | structural | flange plate: ring-bearing boss, TX coil pocket (2.4 mm) with two lead holes, 6 inserts in its sides |
| `carrier_wall_L_x1.stl`, `carrier_wall_R_x1.stl` | 1 + 1 | structural | flange walls, 3 countersunk M3 each |
| `gear_A_pulley_one_piece_x2.stl` | 2 | gear | side bevel 24T + 100T pulley |
| `gear_C_output_x1.stl` | 1 | gear | roll bevel, D bore on the roll shaft |
| `idle_gear_D_24T_x1.stl` | 1 | gear | idler bevel |
| `T_fitting_x1.stl` | 1 | structural | carries the idler pin and the lower roll bearing |
| `motor_block_A_x1.stl`, `motor_block_B_x1.stl` | 1 + 1 | structural | 625 bearing for each motor shaft, on the inner face of the bone |
| `forearm_base_plate_x1.stl` | 1 | structural | the bones screw onto it; on the bench it is the stand (screw it to a board through its three M3 holes, or clamp it) |
| `enc_pitch_bracket_x1.stl` | 1 | structural | only when the pitch encoder goes on (the camera and the TX driver board are left off the flange for now, v1.36) |
| `ulna_x1.stl`, `radius_x1.stl` | 1 + 1 | structural | the forearm bones: ulna and radius are the same part turned over (either file twice), 176 mm long since v1.35, wider since v1.36 to carry the side-by-side motors, 10 mm thick with diagonal truss slots. **PETG-CF, 6 perimeters, 100% infill**, flat on the bed |
| `standoff_ulna_1_x8.stl` | 8 | structural | motor stand-offs, 100% infill |
| `tube_bolt_fore_1_x4.stl` | 4 | structural | tubes of the four through-bolts that tie the bones, 100% infill |

### Hand (`print/hand/`)
`hand_body`, `hand_head`, `hand_palm` (dome), `hand_finger_1`, `hand_finger_2` (× 2), and the three TPU `hand_finger_flex_*`. Details in `hand_print_notes.md`.

### Temporary or optional (`print/temporary/`)
| File | Qty | Notes |
|---|---|---|
| `idler_sleeve_9x10_x2.stl` | 2 | between bone and belt idler |
| `axle_spacer_ring_1.5_x2.stl` | 2 | on the pitch axle, between each flange wall and the bearing in the bone |

**Printed bones (the design since wrist v1.32, no aluminium version any more):**
- 10 mm of PETG-CF is still several times less stiff than 6 mm aluminium, so the belt pull bends them a little. Keep the belt tension moderate and always fit the four through-bolts. The whole arm sags about 0.1 mm more under 500 g than with aluminium bones (`control/stiffness_report.md`).
- The two Ø2.5 holes for the pitch-encoder bracket: an M3 screw cuts its own thread in the plastic.
- The 6801 seats go through the 10 mm plate: glue the bearings with a drop of Loctite 638 or cyanoacrylate.

## Buy: mechanical

| Part | Qty | Notes |
|---|---|---|
| NEMA17, about 40 mm, 5 mm shaft | 2 | the salvaged ones; measure body length and shaft length (the CAD assumes 39 and 20 mm) |
| GT2 pulley, 20 teeth, 5 mm bore, for 10 mm belt | 2 | check bore and that it has two set screws |
| Closed GT2 belt, 10 mm wide, **steel cord**, 400 mm (200 teeth) | 2 | the same belt as the four elbow belts; the 20-tooth motor pulleys are big enough for steel cords |
| Smooth idler, 5 mm bore, for 10 mm belt, about Ø12, with bearings | 2 | belt tensioners |
| 6801-2RS, 12 × 21 × 5 | 8 | 4 in the side bevels, 2 in the bones, 2 on the roll shaft |
| 688-2RS, 8 × 16 × 5 | 2 | idler bevel |
| 625-2RS, 5 × 16 × 5 | 2 | motor shaft blocks |
| 6810 (61810), 50 × 65 × 7 | 1 | ring bearing between flange and hand |
| F6-12M thrust ball bearing, 6 × 12 × 4.5 | 1 | worm thrust, in the palm dome |
| Ground steel shaft Ø12 h6 × 119 | 1 | pitch axle; file three 1 mm flats for the grub screws (two walls, T fitting) |
| **Non-magnetic** shaft Ø12 × 59 (304 stainless or 7075 aluminium) | 1 | roll shaft: 2 mm flat 22 mm long under bevel C, 1 mm flat at the hub end |
| Steel pin Ø8 × 20 | 1 | idler bevel pin |
| Dowel pins Ø5 × 32 | 3 | finger pins |
| Dowel pin Ø6 × 16 | 1 | worm top pin |
| Wave spring 12 × 18, about 50 N at 2 mm | 2 | side bevel preload |
| Spacer ring 12 × 15 × 0.8 and DIN 988 shims 12 × 18 (0.1–0.5) | 2 + assortment | preload tuning |
| Flange hub, aluminium, 12 mm bore, flange Ø32, 4 holes on Ø26, 12 mm high | 1 | hand on the roll shaft; measure it before printing `hand_body` |
| 25 mm metal gear motor, 12 V, 75:1, 4 mm D shaft (Pololu 25D class) | 1 | Ø25 × 54 |
| Steel worm, module 1, single start, right hand, pitch Ø16, with hub (KHK SW1-R1 class) | 1 | thread cut to 16 mm; 6 mm bore needs a 4/6 sleeve, or buy a 4 mm bore |
| KW10 micro switch, NC contact | 1 | finger limit switch |
| Diametric magnets 6 × 2.5 | 2 | encoders, later |
| Loctite 638 and 243, dry PTFE spray | | no grease on the worm: it must stay self-locking |

### Screws and inserts
| Part | Qty | Where |
|---|---|---|
| M3 heat-set inserts (Ø4.0 hole) | 34 | forearm base 8, flange plate 6, hand turret 6, motor blocks 4, hand hub seat 4, hand body 3 |
| M3 × 40 + washer | 8 | motors, through the 10 mm bone and the 24 mm stand-off (about 4.5 mm of thread in the motor: check the hole depth) |
| M3 × 20 | 8 | bones on the forearm base |
| M3 × 12 countersunk | 6 | flange walls on the flange plate |
| M3 × 10 | 14 | hand dome 6, motor blocks 4, encoder bracket 2 |
| M3 × 8 | 7 | hand hub 4, hand head on body 3 |
| M3 × 6 | 2 | gear motor face |
| M3 × 6 grub screws | 3 | flange walls 2, T fitting 1 |
| M5 × 35 + flange nut | 2 + 2 | belt idlers, in the slots of the bones |
| M5 × 130 button head (or M5 threaded rod) + flange nut | 4 + 4 | through-bolts between ulna and radius, with the printed tubes |
| M2 × 10 | 2 | micro switch |
| M3 × 25 + nut + large washers | 6 | Fin Ray fingers on their roots |

## Buy: electronics

### To move the wrist (stage 2)
| Part | Qty | Notes |
|---|---|---|
| Stepper drivers (TMC2209 modules) | 2 | about 1 A is enough on the bench |
| Controller: ESP32-S3 dev board, or any board that makes step/dir pulses | 1 | the final arm uses an ESP32-S3 |
| 24 V supply, 3 A or more (12 V works for slow tests) | 1 | plus a 100 µF capacitor at each driver |

An old 3D-printer mainboard with its supply can do this job on day one: with CoreXY kinematics selected, the two motors behave exactly like this differential (one axis is pitch, the other roll).

### To close the hand directly (stage 3)
A 12 V bench supply with a current limit, or a DRV8871 breakout with its current-limit resistor changed to 82 kΩ (0.78 A).

### Inductive link (stage 4)
| Block | Parts |
|---|---|
| Coils | PCB coils `idra_coil_TX20` and `idra_coil_RX24` (plus `RX28` as a spare), 4 layers, 2 oz copper; flexible sintered ferrite sheet for wireless charging, 0.3 mm, at least 50 × 50 mm, 4 sheets (2 per coil) |
| Flange driver (TX) | Seeed XIAO ESP32S3 Sense; IR2104; 2 × N-MOSFET (≥ 40 V, ≤ 50 mΩ, gate charge ≤ 20 nC); 78L12; US1D; 1 µF 100 V film capacitor (polypropylene); 2 × 10 Ω; 0.1 Ω 1 W shunt; INA180A2; 1 kΩ + 100 nF; 100 µF 35 V + 100 nF on the 24 V |
| Hand board (RX) | DRV8871; 4 × SS24; 47 µF 50 V; SMBJ30A; 1 µF 50 V; 2 × 2.2 mH inductors; C0G 1.2 nF and 270 pF; 470 Ω and 1 kΩ; 2 × 1N4148W; 2 × 4.7 nF; 2 × 10 kΩ; 2 × 3.3 V zeners; 2 × 47 kΩ; 2 × 5.6 V zeners; 82 kΩ (R_ILIM) |
| Bench | 15 Ω 10 W resistor (stands in for the stalled motor); an oscilloscope is close to essential; an LCR meter helps |

Schematics and the test plan are in `electronics/hand_passive.md`, `rx_hand.svg`, `tx_flange.svg`.

## Coils

The coils are **printed circuit boards**: 4-layer rings made by `electronics/coil_pcb.py`, with Gerbers, KiCad and DXF files in `electronics/coil_pcb/` (ordering notes in its README). They replace hand winding: same size, same turns, every piece the same.

| Board | Where | Turns | DC resistance (calculated, 2 oz copper) |
|---|---|---|---|
| `idra_coil_TX20` | flange | 20 | 0.30 Ω |
| `idra_coil_RX24` | hand | 24 | 0.45 Ω |
| `idra_coil_RX28` | hand, spare option | 28 | 0.65 Ω |

**The stack in each pocket (Ø44.2 / Ø14.8, 2.4 mm deep, both sides):**
1. on the floor, 2 rings of flexible sintered ferrite sheet, 0.3 mm each (Ø44 / Ø15, cut with scissors, two notches under the pads for the leads);
2. 0.2 mm of adhesive (most ferrite sheets come with it);
3. the coil board, 1.6 mm, copper face flush with the pocket rim. The two boards face each other across a 1 mm gap.

**Leads:** solder two short wires to the pads on the back of the board (the ferrite side), and pass them through the two Ø2.6 holes under the pads: down through the flange plate to the driver on the TX side, up into the hand on the RX side. Twist each pair.

**Starting values (first calculation, not tested):**
- **TX20 / RX24.** The bridge puts a ±12 V square wave on the TX coil; the 1.2 turns ratio is there to give 10–15 V after the rectifier under load.
- **Inductance:** about 15 µH (TX) and 21 µH (RX) in air, from the flat-spiral formula; the ferrite sheet raises it by an amount to be measured. Measure it at the real gap before the first power-up (step 1 of the bench test).
- **If the hand voltage under the 15 Ω load is below 9 V:** use `RX28`. If the idle current of the bridge is too high, the coils need more turns: change `VARIANTS` in `coil_pcb.py` and order again.
- **Resistance:** about twice that of a Litz coil of the same size. At about 1 A that is 0.3–0.5 W per coil from the DC resistance alone, more at 100–200 kHz by an amount not calculated; check that the boards stay below 60 °C in the bench test.

**Litz fallback:** if you want to try before ordering boards, wind the same coils by hand: Litz about 30 × 0.1 mm (outer Ø about 0.8 mm), 20 and 24 turns in two layers, 18 mm inside, 40 mm outside, 1.6 mm thick at most so they fit the same pocket. Wind on a jig, fix with thin cyanoacrylate, tin the ends until every strand is wetted.

**Things to watch on the bench:**
- the roll shaft passes through the field: if a stainless shaft gets warm, switch to 7075 aluminium or shorten the bridge on-time;
- the 6810 ring bearing is steel and sits 3 mm below the coils: check that it stays cold;
- unloaded, the rectified voltage must stay below 30 V.

## Assembly order

1. Inserts everywhere (list above). Bearings into the bones, the side bevels and the motor blocks.
2. Motors on the bones with stand-offs and M3 × 40; 20-tooth pulleys; motor blocks on the inner face of the bones over the shaft ends.
3. Wrist stack on the Ø12 × 119 axle: T fitting with idler bevel in the middle, the two side bevels with bearings, shims and wave springs, then the two flange walls, the 1.5 mm rings, and the bones outside. Belts on before the walls close.
4. Bones onto the forearm base (M3 × 16), the four through-bolts with their tubes, idlers into their slots.
5. TX coil stack into the flange plate pocket (ferrite, adhesive, board), leads through the two holes. Flange plate between the walls (3 countersunk M3 per side). Roll shaft up through the T fitting bearing and the flange, bevel C on its flat.
6. Ring bearing on the flange boss; hand as in `hand_print_notes.md`; hand hub clamped on the roll shaft.
7. Belt tension: the strand deflects about 2 mm under a firm finger push.

## Bench tests

1. **By hand, no belts:** pitch and roll turn freely, no tight spots over a full roll turn. Adjust shims until the bevels have just perceptible backlash.
2. **Motors:** both the same way gives pitch, opposite ways gives roll (5 motor turns per joint turn). Check ±90° pitch and several roll turns in each direction.
3. **Backlash:** block the motor pulleys and measure the play at the fingertip (205 mm from the wrist centre). This is the number that decides whether the printed bevels are good enough.
4. **Belts:** stall the flange by hand and check that no belt jumps a tooth.
5. **Hand:** grip force on a scale, then lift a full 0.5 l bottle and shake it with the power off.
6. **Inductive link:** the eight steps in `electronics/hand_passive.md`.
