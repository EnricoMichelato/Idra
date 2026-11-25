# Assembly guide (arm v0.50, wrist v1.40, hand v3.0)

Nothing in this guide has been built yet: it is the order the CAD was designed for. Where a step is a guess, it says so.

## Before printing
1. **Buy and measure the bought units first:** the two NEMA23 and their 5:1 gearboxes, the shoulder pinions, the geared NEMA17, the hand flange hub, the STS3215 servo, its coupler and the worm. Set their parameters in `arm_v0.py` (`sh` for the shoulder, `yaw_gb`, `yaw_shaft`) and `hand_v3.py` (`servo`, `servo_out`, `spline`, `coupler_d`, `hub_*`, `worm_*`), then rebuild.
2. Print the fit coupon (`common/fit_test.md`), put the measured values in `common/fit.py`, rebuild, then run `python3 tools/export_print.py`.
3. Order the two upper-arm bones and the two cheeks, all 4 mm steel, from the DXF files (`LASER_CUT.md`): ream the bearing bores and tap the marked holes.
4. Print from `out/print/`: every STL already lies in its print orientation. `PRINT_LIST.md` says which parts need supports.
5. Press all heat-set inserts before assembling anything (list in `BOM.md`).

## Fasteners, joint by joint
| Joint | How it is held | Access |
|---|---|---|
| Foot → desk | 4 lugs, 2 × M5 + nut each into the foot floor, M6 or clamp to the desk | from above |
| Deck → foot | 7 × M3 × 10 into inserts in the pilasters | from above, turntable off |
| Yaw cartridge → deck | 4 × M3 × 10 through the cartridge flange into deck inserts | from above |
| Yaw bearings in the cartridge | upper 6810, two spacers, lower 6810; retainer ring with 4 × M3 × 12 | from below the deck |
| Turntable spindle → bearings | clamp ring on the spindle end, 4 × M3 × 12 | from below the deck |
| Ring gear → turntable | 6 × M3 × 12 from the top of the turntable | from above |
| Yaw motors (two) → deck | 4 × M3 × 10 each from above the deck into the gearbox face, in slots | from above, turntable off |
| Yaw encoder | 20T pinion on a Ø5 shaft in a 688 in the printed lever; lever on an M3 shoulder screw into a deck insert, pulled outward by a small spring; magnet on the shaft end, 14-bit encoder board under the deck | from above and below the deck |
| Rim rollers | 625 on an M5 × 25 through two slotted posts of the deck, nut outside | from above, turntable off |
| Shoulder motor block (two halves, one per motor) → turntable | 8 × M5 × 20 into inserts in the block bottom | from under the turntable, before it goes on the base |
| Shoulder gearbox → motor block | 4 × M5 × 16 + washers in the slots of each face wall (gearbox output face) | from the motor side, before the cheeks |
| Cheeks (4 mm steel) → motor block | 4 × M5 × 16 each into inserts in the end faces | from outside |
| Cheek supports → turntable | the big printed support outside each cheek (v0.49, foot + motor sleeve in one part): 4 × M5 × 20 from under the turntable into inserts in its foot, 4 × M4 × 12 from inside the cheek into inserts in the support | from below and from inside the cheek |
| Stub → cheek | 4 × M4 × 8 into the tapped cheek | from outside |
| Upper-arm bones → stubs | 6810 in each bone, DIN 471 ring on the stub inside, spacer ring between bearing and cheek | — |
| Elbow motors → cheek supports → cheeks | each 48 mm NEMA17 slides 24 mm into the square sleeve of the support and is held by 4 × M3 × 10 from the inner face (before the support goes on); the support covers the whole steel cheek down to the turntable, its spigot goes into the stub mouth, 4 × M4 × 20 through the stub flange and 5 × M4 × 16 round the edge into the tapped cheek | from outside |
| Elbow drive shafts | Ø8 shaft from inside the arm through the bushing (two 688) in the stub; jaw coupling to the motor in the flange spigot; 20T pulley on the inner end (set screws on a flat) | from inside the arm, before the bones close |
| Balance springs | one per side, outside: Ø6 anchor pin pressed in the tower of the cheek support (55 mm above the axis), Ø10 steel pin through the bone 100 mm up the arm (M10 nut inside, collar outside); spring eyes on both pins, e-clips | from outside, arm upright |
| Intermediate axle | one Ø10 axle across the arm through the bone slots; printed clamp on the outer face of each bone, 2 × M4 × 30 + nut through the slots and an M3 grub on the axle; one pulley pair (110T + 20T) per side on two 6800, spacer to the bone | from outside the bones; slide the axle along the slots to tension both stage-1 belts |
| Upper-arm tensioners | 2 × M5 × 150 through both bones (v0.50): one for stage 1 (back strand, about two thirds up to the intermediate axle), one for stage 2 (front strand); each carries two smooth Ø18 idlers outside the belt loops, between split tubes; flange nut on the far bone | from outside, slid along the slots |
| Cradle walls → cradle plate, flange walls → flange plate | 4 (elbow) and 3 (wrist) countersunk M3 × 12 per wall into inserts in the plate sides | from outside the wall, before the bones go on |
| Wrist motor shaft blocks → bones | 2 × M3 × 10 each through the bone into inserts in the block (625) | from outside the bone |
| Elbow shaft → cradle walls | 2 × M3 grubs on a flat | from the front, through the gap between pulley and bone |
| Spider → elbow shaft | M3 grub on a flat | before the forearm hub goes in |
| Forearm base → bevel hub | the base sits on the needle thrust bearing sunk in the cradle plate; hub hex 25 mm in the pocket under the base + 3 × M3 × 8 from above into inserts in the hub end: tightening them pulls the hub up and preloads the 6806 against the thrust bearing (shims under the hex set the preload: no tilt play, still turns freely) | from above the base, before the bones |
| Ulna, radius → forearm base | 4 × M4 × 20 each into inserts in the base sides | from outside |
| Wrist NEMA17s → ulna, radius | 4 × M3 × 40 each, through the 10 mm bone and a printed stand-off | from outside |
| Wrist axle → flange, T fitting → axle | M3 grubs on flats | from outside |
| Bevel C → roll shaft | D-bore on a 2 mm flat; trapped between the T-fitting bearing and the flange bearing | none needed |
| Hand → roll shaft | aluminium flange hub: set screws on the shaft flat, 4 × M3 × 8 into inserts in the hand bottom | from inside the hand body, before the head |
| Servo → hand body | STS3215 sits on the hub flange and a pad, located by two ribs; the head plate clamps it | from above, before the head |
| Worm → servo | aluminium coupler on the spline (clamp screw), worm hub on its 6 mm stub (M4 set screw); top pin in a thrust bearing in the palm | before the palm |
| Hand head → hand body | 3 × M3 × 8 into inserts, on the floor of the turret pillars | from above, before the dome |
| Palm → head | 6 × M3 × 10 into inserts in the cheek tops, beside the wheels; its plugs hold the finger pins | from above |
| Wrist belt idlers | M5 × 35 in a slot of the bone + flange nut | from outside |

## Order
### Base
The base is Ø320. Turntable and foot are each four identical quadrants: every quadrant has a half-thickness tongue on one edge that lies in the recess of the next one, so they go together only one way round.
1. **Foot:** lay the four foot quadrants out, tongues on the recesses, and screw the floor laps (2 × M3 per joint) and the wall tabs (2 × M3 per joint, from outside). Bolt the four desk lugs across the joints (2 × M5 each).
2. Panels into the four windows from inside (4 × M3 each): front with the e-stop and the display, I/O, vents, fan.
3. Press the two 6810 with their spacers into the cartridge, screw on the retainer. Screw the two yaw motors with their gearboxes (pinions on, grubs on the key flats) and the cartridge to the deck, motor screws loose; fit the encoder lever and its spring. Fit the controller, the two shoulder drivers, the step-down and the fan, then screw the deck on the eight pilasters.
4. Fit the four rollers in their posts on the foot walls, low in the slots.
5. **Turntable:** put the hub flange down, lay the four turntable quadrants round it (tongues on recesses), screw the laps (2 × M3 per joint) and the quadrants onto the hub flange (8 × M3). Screw the ring gear underneath with 8 × M3 × 20 from the top.
### Shoulder
1. Screw each NEMA23 to its 5:1 gearbox and the gearbox faces to the face walls of the motor block, screws loose in their slots; pinions on the gearbox shafts (Loctite 638 + set screw), the hub toward the gearbox.
2. Put the two halves of the motor block on the turntable and fit the 8 M5 from below (4 per half); they also tie the turntable quadrants together. The gearboxes stand directly on the turntable.
3. Press a 6810 into each upper-arm bone (they stand a little proud of the 4 mm steel). The shoulder teeth are part of the bones.
4. Stubs on the cheeks. Build the upper arm (next section) before closing the shoulder.
5. Slide the arm between the cheeks with the bones on the stubs, fit the retaining rings from inside, then screw the cheeks to the motor block. The shoulder gearbox shaft ends pass through the holes in the cheeks.
6. Elbow motors, from outside: screw each 48 mm NEMA17 into the sleeve of its cheek support (4 M3 from the inner face), coupling on the motor shaft; push the support spigot into the stub mouth so the coupling takes the drive shaft, then the 9 M4 into the cheek, the 4 M4 from inside the cheek and the 4 M5 from under the turntable; hook the balance spring on its two pins with the arm upright.
7. Backlash: with the arm upright, push each gearbox toward the bone teeth in its slots until the teeth just have play, and tighten. Check over the whole ±90° range: the play must stay small but never zero. **There is no brake: until the springs are hooked and the dynamic braking is wired, support the arm whenever the power is off.**
8. Drop the hub spindle into the yaw bearings and fit the clamp ring from below.
9. Push the first yaw pinion into the ring gear until the play is gone and tighten its gearbox screws; push the second one in and tighten it with the two motors preloaded against each other (firmware: a few tenths of a degree). Check that the encoder pinion is pressed into the ring by its spring. Raise each roller until it just touches the turntable and tighten its nut. Turn the base by hand through the whole range: it must not bind.

### Upper arm and elbow
The upper arm holds the common intermediate axle with the two pulley pairs, both belt stages (stage 2 inward, a smooth neck away from the side-bevel heels; stage 1 outward, right inside the stubs, v0.49), the pins of the two balance springs and the through-bolts. The elbow motors are outside the cheeks, fixed to the turret.

1. Fit the 6801 and 6810 bearings in the bones (Loctite 638).
2. Build the elbow stack on the Ø12 shaft: spider in the middle, the two side bevels with their bearings, shims and wave springs, then the two cradle walls, then the bones outside them. Screw the walls to the cradle plate. Belts go on before the walls close.
3. Fit the drive shafts through the stubs with their 20T pulleys. Fit the two spring pins through the bones (nut inside) before the bones close. Fit the intermediate axle with its two pulley pairs; stage-1 belts over the 20T at the shoulder, stage-2 belts over the 20T of the pairs. Fit the two low through-bolts with their split tubes. The springs go on last, from outside (see the shoulder table).
4. Fit the forearm bevel hub through the cradle 6806 with timing marks facing; lay the AXK5578 with its two washers in the pocket on top of the cradle plate; put the forearm base on the hex and set the preload with the three M3 (shims under the hex). Route the cable up the hub and along the groove in the top of the base, out through the hole in the ulna.
5. Fit the two tensioner through-bolts (stage 1 low on the back strands, stage 2 high on the front strands; split tubes, smooth idlers outside the loops) and tension the belts (below). The intermediate axle no longer slides. No covers for now.

### Forearm, wrist, hand
1. Screw the wrist motors to ulna and radius, build the wrist stack like the elbow one, screw the bones to the forearm base, (no through-bolts: the flanges tie the bones; covers later).
2. Roll shaft: bevel C on its flat, up through the flange bearings.
3. Hand v3.0: the hub in the hand bottom clamps on the roll shaft; servo on the hub flange, boards on the wall rails, head plate, coupler and worm, finger wheels with their pins (timing: all three fingers at the same angle), palm. Notes for v2.6 printing still apply (`wrist_v0/hand_print_notes.md`).
4. PCB coils with their ferrite sheet (`WRIST_BUILD.md`, section Coils) and the hand board. Camera and TX driver come later.

## Belt tension
- Push each idler against the back of the belt until the strand deflects about 2 mm under a firm finger push at mid span, then tighten the flange nut. At the elbow the idler rides on a through-bolt: loosen its nut, slide the bolt along the two slots, tighten; the bolt and its tubes also tie the bones together, so tighten it firmly.
- A GT2 belt does not slip, it jumps teeth when it is slack. With the idlers the small pulleys have 7 or more teeth engaged. Holding 1 kg at the flange needs about 70 N per stage-2 belt. A stalled 48 mm NEMA17 (0.59 Nm) can pull about 90 N on stage 1 and about 500 N on stage 2: set the elbow driver current (or the closed-loop torque limit) so a jammed arm cannot overload the belts.
- If a belt ever jumps: more tension first; if that is not enough, 20-tooth motor pulleys (the script recomputes the centre distance).

## Bench tests before building the whole arm
1. **Shoulder:** both geared NEMA23 in the motor block with the cheeks and one bone, drivers at 2.8 A on the same step signal. Hang 3 kg at 300 mm from the axis (about 9 Nm) on that bone: it must lift it without losing steps. Turn the bone through ±90°: the teeth must stay in mesh to the ends. Without power the arm drops: keep a hand under it.
2. **One elbow belt stage:** one bone, one motor, one side bevel. Stall the pulley by hand and check that the belt does not jump.
3. **Hand:** squeeze a kitchen scale between two fingers with the servo current limited to 0.6 A (expected about 18 N per finger). Then lift a full 0.5 l bottle and shake it. Check that the worm holds with the servo off.
4. **Inductive link:** 12 Ω, 12–15 W load on the hand rectifier: the 12 V bus must stay between 10 and 13 V (servo 4–14 V); then the real hand board with the ESP32-S3 and the servo stalling at the current limit.
5. **Shoulder springs:** with the motors off, the arm leaned to 45° and to 90° should move only slowly; measure the torque left with a luggage scale and adjust the spring rate.

## What is not verified
- No part has been printed; clearances come from `common/fit.py` defaults.
- Bought-part dimensions are catalogue figures: mounting faces and hole patterns must be measured.
- Tool access was reasoned from the model, not tried.
- Boards, sensors and connectors have no modelled screws.
- Cable routes need real cables.
