# Wireless Robot Arm: hardware list (arm v0.50, wrist v1.40, hand v3.0)

Bought parts are modelled from catalogue figures. The base is Ø300: turntable and foot each in four identical printed quadrants, the 146-tooth ring gear and the deck in one piece each. **Buy the motors, gearboxes, pinions and the hand flange hub first, measure them, set their parameters in the scripts, then print the parts around them** (shoulder motor block, deck, hand body).

Dimensions are indicative: check them when buying.

## Geared drives (bought, steel)
| Part | Qty | Where | Figures used in the CAD |
|---|---|---|---|
| 5:1 high-precision planetary gearbox for NEMA17 (17HS15-1684S-HG5 class, or the gearbox alone for the salvaged motors) | 2 | Base, one per yaw motor (v0.47: two drives on opposite sides of the ring, preloaded against each other) | gearbox 42 × 41, shaft Ø8 × 23.5 with key, backlash ≤ 15 arcmin |
| Feetech STS3215 serial-bus servo, **12 V version** (C018 / C047, 30 kg·cm, 345:1 metal gears, 12-bit encoder) | 1 | Hand (v3.0) | 45.2 × 24.7 × 35, output 11.3 mm from the near end (check the drawing), used in motor mode, multi-turn |
| Steel worm, **module 1.5, single start**, right hand, pitch Ø20 (KHK SW1.5-R1 class), with hub | 1 | Hand | Ø23, thread 20 mm, hub Ø16 × 10, 6 mm bore, M4 set screw; lead angle 4.3°: self-locking |
| Servo-to-shaft coupler, aluminium, 25T spline → 6 mm stub (Ø14) | 1 | Hand | Clamps the STS3215 spline; the worm hub sits on its 6 mm stub |

## Shoulder drive (two geared motors on the teeth of the steel bones, balance springs, no brake)
| Part | Qty | Notes |
|---|---|---|
| NEMA23, 57 × 57 × 56, about 1.26 Nm holding, 2.8 A (23HS22 class), shaft to suit the gearbox | 2 | Under the shoulder axis, side by side, facing opposite ways. No brake: four balance springs carry most of the arm weight and dynamic braking (phases shorted by a relay) makes it settle slowly when the power goes |
| Planetary gearbox **10:1** (single stage) for NEMA23, high precision (≤ 15 arcmin), flange about 60 × 60, output shaft Ø14 × 25 | 2 | On the motors. Measure flange, pilot and hole pattern and set `gb_*` in the `sh` parameters before printing the motor block. The two motors driven slightly against each other take out the gearbox backlash too |
| Steel spur pinion, module 2, 20 teeth (pitch Ø40), face 6 mm or more, 14 mm bore with key or set screw, hub Ø24 × 6 | 2 | On the gearbox shafts; it meshes with the 56 teeth cut in the 4 mm steel bone (40:1 in all). Loctite 638 as well as the set screw |
| Cheeks, waterjet-cut steel, 4 mm | 2 | One file, quantity 2 (left = right, `LASER_CUT.md`), cut with the upper-arm bones. M4 tapped: 4 for the stub flange + 5 on R46 for the cheek support; 4 × Ø4.4 for the support screws from inside |
| Fixed stub, aluminium, Ø50 × 15 with Ø40 bore, flange Ø70 × 5 with 4 × M4 on Ø60, groove for a DIN 471 50 ring, 4 × M3 tapped in the inner end | 2 | A small lathe job. The bones turn on them (6810); the elbow drive shaft runs through on two 688, the elbow motor flange sits on the outer flange |
| DIN 471 retaining ring 50 + spacer ring 50 × 56 × 1.5 | 2 + 2 | On each stub: the ring inside, the spacer between bearing and cheek |
| Driver for 3 A (TMC5160 board or external DM542-type) | 2 | One per NEMA23, both on the same step and direction signals |

## Other motors and drivers
| Part | Qty | Notes |
|---|---|---|
| NEMA17, 40 mm, 5 mm shaft (the four salvaged ones: measure them) | 4 | Base 2 (with the 5:1 gearboxes), wrist 2 |
| NEMA17, **48 mm**, 0.59 Nm, 2 A, **long or double shaft** (17HS19-2004 class), ideally the closed-loop version with encoder | 2 | Elbow (v0.47): outside the cheeks, on printed flanges, coaxial with the shoulder |
| Closed-loop driver boards for NEMA17 / NEMA23 (MKS SERVO42D / SERVO57D class, CAN) | as many as the axes you want closed loop | Recommended on every axis (no lost steps) |
| ESP32-S3 board + 6 × TMC2209 | 1 | In the foot: base A/B, elbow A/B, wrist A/B (or the closed-loop boards above) |

## Bearings
| Type | Size (mm) | Qty | Where |
|---|---|---|---|
| 6801-2RS | 12 × 21 × 5 | 15 | Elbow: 4 in the side bevels, 2 in the bones, 1 in the tip of the forearm satellite. Wrist: 4 in the side bevels, 2 in ulna and radius, 2 on the roll shaft |
| 688-2RS | 8 × 16 × 5 | 9 | Elbow (2) and wrist (2) idler bevels, elbow drive shafts (2 per stub, in the bushing), yaw encoder lever (1) |
| 6800-2RS | 10 × 19 × 5 | 4 | Elbow intermediate pulley pairs on the common Ø10 axle (2 per side) |
| 625-2RS | 5 × 16 × 5 | 6 | Wrist motor shaft supports, in blocks on ulna and radius (2), rollers under the turntable rim (4, one per foot quadrant) |
| 6806-2RS | 30 × 42 × 7 | 1 | Elbow cradle plate, under the satellite hub; preloaded upward against the thrust bearing (v0.50) |
| Needle thrust bearing AXK5578 + 2 washers AS5578 | 55 × 78 × 5 (stack) | 1 | Sunk in the top of the elbow cradle plate: the forearm base plate sits on it (v0.50, replaces the 61810 at the pronation) |
| 6810 (61810) | 50 × 65 × 7 | 5 | Base yaw spindle (2), shoulder, one in each upper-arm bone (2), ring around the wrist coils |
| F6-12M thrust ball bearing | 6 × 12 × 4.5 | 1 | Hand: takes the worm thrust in the palm dome |

The 6802 and 61814 of the printed cycloid are gone.

## Waterjet-cut plates
| Part | Material | Size | Qty | Notes |
|---|---|---|---|---|
| Humerus, with the shoulder teeth cut in (56 teeth module 2 on the lower arc) | steel, 4 mm | 116 × 320 | 2 (one file) | The left and right bones are the same part, the second turned over. DXF files and finishing notes in `LASER_CUT.md` |
| Shoulder cheek | steel, 4 mm | 156 × 151 | 2 (one file) | Left and right are the same part (both gearbox shaft holes); M4 tapped for the stub flange |

## Shafts, hubs, pins, springs, magnets
| Part | Size | Qty | Notes |
|---|---|---|---|
| Flange hub, aluminium, 12 mm bore, set screws | flange Ø32, 4 holes on Ø26, 12 high | 1 | Hand: on the roll shaft, screwed to the bottom of the hand |
| Ground shaft h6 | Ø12 × 145 | 1 | Elbow: clamped in the cradle walls with 2 grub screws, 1 mm flat |
| Ground shaft h6 | Ø8 × about 55 | 2 | Elbow drive shafts, through the stubs on two 688: jaw coupling to the motor outside, 20T pulley inside the bones |
| Ground shaft h6 | Ø10 × 152 | 1 | Common intermediate axle across the upper arm, clamped on the outer faces of the bones (slide it in the slots to tension both stage-1 belts) |
| Ground shaft h6 | Ø12 × 119 | 1 | Wrist pitch axle, as above |
| Shaft, **non-magnetic** (304 stainless or 7075 aluminium) | Ø12 × 59 | 1 | Wrist roll shaft. A 2 mm flat 22 mm long under bevel C, a 1 mm flat at the hub end. The encoder magnet is glued on its lower end |
| Shaft, non-magnetic (304 stainless) | Ø8 × 32 | 1 | Elbow idler bevel; carries the pronation encoder magnet |
| Steel pin | Ø8 × 20 | 1 | Wrist idler bevel pin |
| Dowel pin | Ø6 × 16 | 1 | Worm top pin (Loctite in the worm bore) |
| Dowel pins | Ø6 × 36 | 3 | Hand finger wheels (held by the palm plugs) |
| Extension spring, about 0.9 N/mm, **Ø10**, about 45 mm free length (eye to eye), loop ends, initial tension as high as offered | 2 | Shoulder balance (v0.49): outside, one per side, from the anchor pin on the cheek support to the pin in the bone; about 100 N at full stretch (114 mm); check the rate on the build |
| Steel pin | Ø6 × 30 | 2 | Spring anchor pins, pressed in the cheek supports, groove for an e-clip |
| Steel pin (or M10 shoulder bolt) | Ø10 × 40, M10 thread on the inner end, collar Ø16 | 2 | Spring pins through the bones, 100 mm up the arm; M10 nut inside, e-clip groove at the spring |
| Wave spring | 12 × 18, about 50 N at 2 mm | 4 | Side bevel preload |
| Spacer ring | 12 × 15 × 0.8 | 4 | On the inner ring of the pulley-side bearing |
| Spacer ring | 12 × 14.8 × 1.5 | 4 | Between each cradle or flange wall and the bearing in the bone (elbow 2, wrist 2); can be printed |
| Sleeve, M5 clearance | Ø9 × 10 | 2 | Wrist: between bone and belt idler; can be printed |
| Spacer tube, aluminium, M5 clearance (Ø9 / Ø5.2) | cut to length | 6 | Upper-arm tensioner through-bolts (v0.50: one for stage 1, one for stage 2), each split round its two idlers |
| Shim washers | DIN 988 12 × 18, 0.1–0.5 | assortment | Preload tuning |
| Diametric magnet | 6 × 2.5 | 4 | Encoders |

## Transmission
| Part | Qty | Notes |
|---|---|---|
| Closed GT2 belt 10 mm, **steel cord**, 400 mm (200 teeth) | 6 | **All six belts are the same, all steel cord** (since v0.44 every small pulley has 20 teeth, Ø12.7, for the steel cords): elbow stage 1 (20T → 110T, inside the bones since v0.47) 2, elbow stage 2 (20T → 130T, inside; v0.50) 2, wrist (20T → 100T) 2. Two kits of four give two spares |
| GT2 pulley 20 teeth, 8 mm bore, for 10 mm belt | 2 | On the elbow drive shafts, inside the bones (stage-1 plane) |
| Intermediate pulley pair, 110T + 20T in one piece (printed PA-CF, or two aluminium pulleys on a sleeve), 19 mm seats for two 6800 | 2 | On the common Ø10 axle, inside the bones |
| Jaw coupling 5 → 8 mm, Ø18 | 2 | Elbow motor shaft to the drive shaft, inside the stub mouth / flange spigot |
| 20T steel-free spur pinion, module 1.5, printed PA-CF, 5 mm bore | 1 | Yaw encoder, on a spring-loaded lever under the ring gear |
| GT2 pulley 20 teeth, 5 mm bore, for 10 mm belt | 2 | On the wrist NEMA17s (100/20 = 5:1) |

Steel-cord closed GT2 loops are less common than fibreglass ones: buy the six 400 mm (200T) loops together, from one supplier (400 mm is the most common closed GT2 length; ask the seller to confirm the steel cord). No pulley in the arm is smaller than 20 teeth, so they are not bent tighter than about Ø12.7. If 400 mm is not available, the tooth count is one parameter in each script, but then the arm lengths change too (they were chosen so that all six are the same).

| Part | Qty | Notes |
|---|---|---|
| Smooth flanged idler, 5 mm bore, for 10 mm belt (about Ø12) | 2 | Wrist tensioners: M5 in a slot of the bone + flange nut |
| Smooth flanged idler Ø18, 5 mm bore, for 10 mm belt | 4 | Elbow tensioners, OUTSIDE the belt loops on the belt back (v0.50): two on the stage-1 bolt (back strand), two on the stage-2 bolt (front strand) |

## Electronics
| Part | Qty | Notes |
|---|---|---|
| 24 → 5 V step-down | 1 | |
| 24 V 350 W power supply, DC socket, switch, panel USB-C | 1 + 1 + 1 + 1 | 350 W because of the two 2.8 A shoulder motors |
| Ø22 mushroom emergency button | 1 | Cuts the 24 V to the drivers |
| 40 mm 24 V fan | 1 | |
| 4-digit 0.36" 7-segment display (TM1637) | 1 | In the foot |
| Magnetic encoder, **14 bit** (MT6701 or AS5047P) on a small PCB | 6 | Base (on the encoder pinion), shoulder, elbow flexion and pronation, wrist pitch and roll (AS5600 12-bit is too coarse at 50 cm) |
| Hall sensor + magnet for base homing | 1 + 1 | Optional now that the yaw encoder is absolute |
| Relay (or MOSFETs) for dynamic braking of the shoulder motors | 1 | Shorts the NEMA23 phases when the 24 V drops: the arm settles slowly |
| Seeed XIAO ESP32S3 Sense (camera) | 1 | Later: camera and TX driver are left off the wrist flange for now (wrist v1.36) |
| PCB coils `idra_coil_TX20` + `idra_coil_RX24` (4 layers, 2 oz, Ø43.6 / Ø15.6 × 1.6), files in `electronics/coil_pcb/` | 1 + 1 | Wrist inductive link. Add `idra_coil_RX28` to the same order as a spare option |
| Flexible sintered ferrite sheet for wireless charging, 0.3 mm, with adhesive | 4 rings Ø44 / Ø15 | Two behind each coil board; cut from 50 × 50 mm sheets |
| Active hand board (to be drawn): ESP32-S3 module (BLE / WiFi), bridge rectifier 4 × SS34, 470 µF 25 V, SMBJ15A TVS, 12 → 3.3 V buck, half-duplex servo bus buffer (74LVC1G125 or similar), limit-switch input | 1 + 1 | Replaces the passive DRV8871 board (hand v3.0); the hand is still powered only through the coils |
| KW10 micro switch | 1 | Hand homing switch (the servo encoder is single-turn) |
| Flange TX bridge: IR2104, 2 × N-MOSFET ≥ 40 V, 78L12, US1D, 1 µF 100 V film, 0.1 Ω shunt, INA180A2 | 1 | Fixed frequency now (no command in the frequency); must deliver about 12–15 W: bench test |

## Screws and inserts
| Part | Qty (approx.) | Where |
|---|---|---|
| M3 heat-set inserts (Ø4.0 hole) | ~113 | Base: turntable laps 8, hub flange 8, foot floor laps 8, foot wall tabs 8, panels 16, foot pilasters 8, deck 4; hand turret 6, cradle plate 8, wrist flange plate 6, wrist motor shaft blocks 4, forearm base 8, encoder brackets 4, camera and TX driver 4, forearm hub 3, hand head 3, hand hub 4 |
| M5 heat-set inserts (Ø6.4 hole) | 24 | Shoulder motor block: bottom 8, end faces for the cheeks 8; cheek supports 8 |
| M4 heat-set inserts (Ø5.6 hole) | 16 | Cheek supports (screws from inside the steel cheeks) 8; forearm base plate, sides (v0.50) 8 |
| M3 × 10 socket head | ~95 | Base: turntable laps 8, quadrants onto the hub 8, foot floor laps 8, foot wall tabs 8, panels 16, deck on the pilasters 8, yaw cartridge 4, yaw gearbox 4; hand palm 6, wrist motor shaft blocks 4 (through the bones), elbow motors onto their flanges 8, brackets, camera and driver |
| M3 × 12 socket head | 8 | Spring anchors onto the stub inner ends |
| M4 × 20 | 8 | Cheek supports, through the stub flanges into the tapped cheeks |
| M4 × 16 | 10 | Cheek supports, round the edge into the tapped cheeks (5 per side) |
| M4 × 20 socket head | 8 | Ulna and radius (10 mm printed) on the forearm base, into M4 inserts (v0.50) |
| M3 × 12 self-tapping | 8 | Yaw retainer 4, spindle clamp ring 4 |
| M3 × 20 self-tapping | 8 | Ring gear, from the top of the turntable quadrants (two per quadrant) |
| M3 × 12 countersunk | 14 | Cradle walls on the plate 8, wrist flange walls on the plate 6 |
| M3 × 40 + washer | 8 | Wrist NEMA17s, through the 10 mm bone and the printed 24 mm stand-off (about 4.5 mm of thread in the motor: check the depth of your motors) |
| M3 × 8 | 10 | Forearm base on the satellite hub 3, hand hub 4, hand head on the body 3 |
| M3 × 6 grub screw | 7 | Cradle 2, spider 1, wrist flange 2, T fitting 1, yaw pinion 1 |
| M5 × 20 | 16 | Shoulder motor block (8) and cheek supports (8), from under the turntable |
| M4 × 12 | 8 | From inside the steel cheeks into the cheek supports |
| M5 × 16 | 8 | Cheeks (4 mm steel) into the motor block |
| M5 × 16 + washer | 8 | Shoulder gearbox faces in the slots of the motor block |
| M4 × 8 | 0 | (v0.47: the stub flanges are held by the M4 × 20 of the elbow motor flanges) |
| M5 × 35 + flange nut | 2 + 2 | Wrist belt idler axles |
| M5 × 150 button head (ISO 7380, or threaded rod) + flange nut | 2 + 2 | Upper-arm tensioner through-bolts (v0.50: the two plain ones are gone) |
| M4 × 30 + nut | 4 | Intermediate axle clamps, through the slots of the bones |
| M5 × 25 + nut | 4 + 4 | Rim roller axles |
| M5 × 20 + nut | 8 + 8 | Foot wings |
| M6 or desk clamp | 4 | Fixing the wings to the desk |
| M2 × 10 | 2 | Hand micro switch |
| M3 × 25 + nut + large washers | 6 | Fin Ray on the finger roots |

## Printed materials
- **Gears, bevels and pulleys, finger roots:** PETG-CF or PA-CF.
- **Structural parts** (cradle, flange, shoulder motor block, carrier, foot): PETG or PETG-CF.
- **Forearm bones, motor stand-offs and through-bolt tubes** (the whole forearm and wrist structure is printed since wrist v1.32): PETG-CF, 6 perimeters, 100% infill. No covers for now (planned): the elbow and wrist flanges tie the bones (v0.50: no forearm through-bolts).
- **Fingers:** TPU 95A.

## Still to complete in the CAD
- **Bought-part dimensions:** NEMA23 face and shaft, gearbox mounting faces, pinions, flange hub hole patterns and the worm are from catalogue figures, not measured.
- **Cable connectors** at the shoulder and at the forearm base.
- **Small-part fixings:** boards, sensors, connectors and cable clips have no modelled screws.
- **Not built yet:** nothing here has been printed. Start with the fit coupon and the bench tests in `ASSEMBLY.md`.
