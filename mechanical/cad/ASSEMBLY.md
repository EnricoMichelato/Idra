# Assembly guide (arm v0.46, wrist v1.38, hand v2.6)

Nothing in this guide has been built yet: it is the order the CAD was designed for. Where a step is a guess, it says so.

## Before printing
1. **Buy and measure the bought units first:** the two NEMA23 and their 5:1 gearboxes, the shoulder pinions, the geared NEMA17, the hand flange hub, the hand gearmotor and the worm. Set their parameters in `arm_v0.py` (`sh` for the shoulder, `yaw_gb`, `yaw_shaft`) and `hand_v2.py` (`motor_*`, `hub_*`, `worm_*`), then rebuild.
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
| Geared yaw motor → deck | 4 × M3 × 10 from above the deck into the gearbox face, in slots | from above, turntable off |
| Rim rollers | 625 on an M5 × 25 through two slotted posts of the deck, nut outside | from above, turntable off |
| Shoulder motor block (two halves, one per motor) → turntable | 8 × M5 × 20 into inserts in the block bottom | from under the turntable, before it goes on the base |
| Shoulder gearbox → motor block | 4 × M5 × 16 + washers in the slots of each face wall (gearbox output face) | from the motor side, before the cheeks |
| Cheeks (4 mm steel) → motor block | 4 × M5 × 16 each into inserts in the end faces | from outside |
| Cheek feet | printed foot outside each cheek: 2 × M5 × 20 from under the turntable, 4 × M4 × 12 through the cheek into inserts in the foot | from below and from outside |
| Stub → cheek | 4 × M4 × 8 into the tapped cheek | from outside |
| Upper-arm bones → stubs | 6810 in each bone, DIN 471 ring on the stub inside, spacer ring between bearing and cheek | — |
| Elbow motors → stubs | motor face on the printed bracket with 4 × M3 × 10, jaw coupling on the motor shaft; bracket (with its 688) onto the inner end of the stub with 4 × M3 × 10 into the tapped holes | from inside the arm, through the gap between the bones |
| Elbow drive shafts | Ø8 shaft from outside through the outer bushing (688) and the stub into the bracket 688 and the coupling; 16T pulley on the outer end (set screws on a flat) | from outside the cheek |
| Intermediate shafts | printed hub with two 688 on the outer face of each bone, 2 × M4 × 30 + nut through the slots; Ø8 shaft through the bone, 80T outside, 20T inside (set screws on flats) | from both sides of the bone; slide the hub along the slots to tension stage 1 |
| Upper-arm through-bolts | 2 plain ones near the shoulder: M5 × 150 through both bones and a Ø9 × 131 tube; flange nut on the far bone | from outside |
| Upper-arm tensioners | 2 more M5 × 150 through both bones, each with two toothed idlers (one per belt) between three tubes; flange nut on the far bone | from outside, slid along the slots |
| Cradle walls → cradle plate, flange walls → flange plate | 4 (elbow) and 3 (wrist) countersunk M3 × 12 per wall into inserts in the plate sides | from outside the wall, before the bones go on |
| Wrist motor shaft blocks → bones | 2 × M3 × 10 each through the bone into inserts in the block (625) | from outside the bone |
| Elbow shaft → cradle walls | 2 × M3 grubs on a flat | from the front, through the gap between pulley and bone |
| Spider → elbow shaft | M3 grub on a flat | before the forearm hub goes in |
| Forearm base → bevel hub | hex 25 mm in a pocket + 3 × M3 × 8 into inserts in the hub end | from above the base, before the bones |
| Ulna, radius → forearm base | 4 × M3 × 20 each into inserts in the base sides | from outside |
| Forearm through-bolts | 4 × M5 × 130 through ulna and radius, above and below the wrist motors, printed tube; flange nut on the far bone | from outside |
| Wrist NEMA17s → ulna, radius | 4 × M3 × 40 each, through the 10 mm bone and a printed stand-off | from outside |
| Wrist axle → flange, T fitting → axle | M3 grubs on flats | from outside |
| Bevel C → roll shaft | D-bore on a 2 mm flat; trapped between the T-fitting bearing and the flange bearing | none needed |
| Hand → roll shaft | aluminium flange hub: set screws on the shaft flat, 4 × M3 × 8 into inserts in the hand bottom | from inside the hand body, before the head |
| Worm → gearmotor | M4 set screw on the D flat; top pin in a thrust bearing in the palm dome | before the dome |
| Hand head → hand body | 3 × M3 × 8 into inserts, on the floor of the turret pillars | from above, before the dome |
| Palm dome → head | 6 × M3 × 10 into inserts in the cheek tops, beside the wheels; its plugs hold the finger pins | from above |
| Wrist belt idlers | M5 × 35 in a slot of the bone + flange nut | from outside |

## Order
### Base
The base is Ø320. Turntable and foot are each four identical quadrants: every quadrant has a half-thickness tongue on one edge that lies in the recess of the next one, so they go together only one way round.
1. **Foot:** lay the four foot quadrants out, tongues on the recesses, and screw the floor laps (2 × M3 per joint) and the wall tabs (2 × M3 per joint, from outside). Bolt the four desk lugs across the joints (2 × M5 each).
2. Panels into the four windows from inside (4 × M3 each): front with the e-stop and the display, I/O, vents, fan.
3. Press the two 6810 with their spacers into the cartridge, screw on the retainer. Screw the geared yaw motor (pinion on, grub on the key flat) and the cartridge to the deck, motor screws loose. Fit the controller, the two shoulder drivers, the step-down and the fan, then screw the deck on the eight pilasters.
4. Fit the four rollers in their posts on the foot walls, low in the slots.
5. **Turntable:** put the hub flange down, lay the four turntable quadrants round it (tongues on recesses), screw the laps (2 × M3 per joint) and the quadrants onto the hub flange (8 × M3). Screw the ring gear underneath with 8 × M3 × 20 from the top.
### Shoulder
1. Screw each NEMA23 to its 5:1 gearbox and the gearbox faces to the face walls of the motor block, screws loose in their slots; pinions on the gearbox shafts (Loctite 638 + set screw), the hub toward the gearbox.
2. Put the two halves of the motor block on the turntable and fit the 8 M5 from below (4 per half); they also tie the turntable quadrants together. The gearboxes stand directly on the turntable.
3. Press a 6810 into each upper-arm bone (they stand a little proud of the 4 mm steel). The shoulder teeth are part of the bones.
4. Stubs on the cheeks. Build the upper arm (next section) before closing the shoulder.
5. Slide the arm between the cheeks with the bones on the stubs, fit the retaining rings from inside, then screw the cheeks to the motor block. The shoulder gearbox shaft ends pass through the holes in the cheeks.
6. Elbow motors, from inside the arm: each motor with its bracket and coupling goes in between the bones and the bracket is screwed to its stub end (4 × M3). The two motors end up back to back on the shoulder axis; they do not touch. Then, from outside, push each drive shaft through the outer bushing into the coupling, fit the 16T pulley and the stage-1 belt over the 80T of the intermediate shaft, and slide the intermediate hub up the bone until the belt is tight (2 mm deflection at mid span).
7. Backlash: with the arm upright, push each gearbox toward the bone teeth in its slots until the teeth just have play, and tighten. Check over the whole ±90° range: the play must stay small but never zero. **There is no brake yet: support the arm whenever the power is off.**
8. Drop the hub spindle into the yaw bearings and fit the clamp ring from below.
9. Push the yaw pinion into the ring gear until the play is gone and tighten the gearbox screws. Raise each roller until it just touches the turntable and tighten its nut. Turn the base by hand through the whole range: it must not bind.

### Upper arm and elbow
The upper arm holds the two intermediate shafts, the stage-2 belts with their four idlers, and the through-bolts; the stage-1 belts run outside the bones. The elbow motors sit inside it, on the shoulder axis, but they are fixed to the stubs (turret): the arm turns round them. The bones are 4 mm steel with the shoulder teeth cut in; the right one is the left one turned over.

1. Fit the 6801 and 6810 bearings in the bones (Loctite 638).
2. Build the elbow stack on the Ø12 shaft: spider in the middle, the two side bevels with their bearings, shims and wave springs, then the two cradle walls, then the bones outside them. Screw the walls to the cradle plate. Belts go on before the walls close.
3. Fit the two low through-bolts (inside the belt loops). Fit the intermediate hubs and shafts (80T outside, 20T inside); the stage-2 belts go over the 20T pulleys.
4. Fit the forearm bevel hub through the cradle 6806 with timing marks facing, then the forearm base on the hex.
5. Fit the two tensioner through-bolts (short tube, idler, long tube) and tension the belts (below). No covers for now.

### Forearm, wrist, hand
1. Screw the wrist motors to ulna and radius, build the wrist stack like the elbow one, screw the bones to the forearm base, then fit the four through-bolts with their tubes (no covers).
2. Roll shaft: bevel C on its flat, up through the flange bearings.
3. Hand as in `wrist_v0/hand_print_notes.md`: the hub in the hand bottom clamps on the roll shaft.
4. PCB coils with their ferrite sheet (`WRIST_BUILD.md`, section Coils) and the hand board. Camera and TX driver come later.

## Belt tension
- Push each idler against the back of the belt until the strand deflects about 2 mm under a firm finger push at mid span, then tighten the flange nut. At the elbow the idler rides on a through-bolt: loosen its nut, slide the bolt along the two slots, tighten; the bolt and its tubes also tie the bones together, so tighten it firmly.
- A GT2 belt does not slip, it jumps teeth when it is slack. With the idlers the small pulleys have 7 or more teeth engaged. Holding 500 g needs about 60 N per elbow belt. Behind the 5:1 gearboxes a stalled motor could pull over 250 N, which is too much for a 10 mm belt: set the elbow driver current so the motor gives about 0.2 Nm.
- If a belt ever jumps: more tension first; if that is not enough, 20-tooth motor pulleys (the script recomputes the centre distance).

## Bench tests before building the whole arm
1. **Shoulder:** both geared NEMA23 in the motor block with the cheeks and one bone, drivers at 2.8 A on the same step signal. Hang 3 kg at 300 mm from the axis (about 9 Nm, half the joint load) on that bone: it must lift it without losing steps. Turn the bone through ±90°: the teeth must stay in mesh to the ends. Without power the arm drops: keep a hand under it.
2. **One elbow belt stage:** one bone, one motor, one side bevel. Stall the pulley by hand and check that the belt does not jump.
3. **Hand:** squeeze a kitchen scale between two fingers and read the force at R_ILIM 82 kΩ (expected about 16 N per finger). Then lift a full 0.5 l bottle and shake it.
4. **Inductive link:** 15 Ω, 10 W load on the hand rectifier: VM must stay between 10 and 15 V (`electronics/hand_passive.md`).

## What is not verified
- No part has been printed; clearances come from `common/fit.py` defaults.
- Bought-part dimensions are catalogue figures: mounting faces and hole patterns must be measured.
- Tool access was reasoned from the model, not tried.
- Boards, sensors and connectors have no modelled screws.
- Cable routes need real cables.
