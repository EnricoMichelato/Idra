# Assembly guide (arm v0.32, wrist v1.29, hand v2.6)

Nothing in this guide has been built yet: it is the order the CAD was designed for. Where a step is a guess, it says so.

## Before printing
1. **Buy and measure the bought units first:** geared NEMA23, geared NEMA17, the two flange hubs, the hand gearmotor and the worm. Set their parameters in `arm_v0.py` (`n23_*`, `gb_*`, `sh_hub_*`, `yaw_gb`, `yaw_shaft`) and `hand_v2.py` (`motor_*`, `hub_*`, `worm_*`), then rebuild.
2. Print the fit coupon (`common/fit_test.md`), put the measured values in `common/fit.py`, rebuild, then run `python3 tools/export_print.py`.
3. Order the four bones from the DXF files (`LASER_CUT.md`): ream the bearing bores and tap the marked holes.
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
| Pedestal → turntable | 8 × M5 × 20 into inserts in the pedestal bottom | from under the turntable, before it goes on the base |
| Geared NEMA23 → pedestal | square pocket (torque) + 2 straps, 2 × M5 each into inserts in the pedestal top | from above |
| Balancer bracket and anchor → turntable | 3 + 3 × M5 × 12 into inserts | from under the turntable, with the pedestal |
| Left hub → pedestal wall | 4 countersunk M3 × 12 from inside the wall, self-tapping | before the motor goes in |
| Right bone → gearbox shaft | keyed steel flange hub, 4 × M5 × 16 into tapped holes of the bone, set screws on the key | from outside the bone |
| Elbow motors → gearboxes → carrier | motor on its 5:1 gearbox; gearbox face on the carrier wall with 4 × M3 × 10 | before the carrier goes between the bones |
| Motor carrier → bones | 2 × M5 × 16 through each bone into inserts in the carrier blocks | from outside the bones |
| Covers → bones | 6 × M3 × 10 each, through the bones into inserts in the cover tabs | from outside the bones |
| Cradle walls → cradle plate, flange walls → flange plate | 4 (elbow) and 3 (wrist) countersunk M3 × 12 per wall into inserts in the plate sides | from outside the wall, before the bones go on |
| Motor shaft blocks → bones | 2 × M3 × 10 each through the bone into inserts in the block (688 at the elbow, 625 at the wrist) | from outside the bone |
| Elbow shaft → cradle walls | 2 × M3 grubs on a flat | from the front, through the gap between pulley and bone |
| Spider → elbow shaft | M3 grub on a flat | before the forearm hub goes in |
| Forearm base → bevel hub | hex 25 mm in a pocket + 3 × M3 × 8 into inserts in the hub end | from above the base, before the bones |
| Ulna, radius → forearm base | 4 × M3 × 16 each into inserts in the base sides | from outside |
| Wrist NEMA17s → ulna, radius | 4 × M3 × 30 each, through the bone and a Ø7 × 15.5 spacer | from outside |
| Wrist axle → flange, T fitting → axle | M3 grubs on flats | from outside |
| Bevel C → roll shaft | D-bore on a 2 mm flat; trapped between the T-fitting bearing and the flange bearing | none needed |
| Hand → roll shaft | aluminium flange hub: set screws on the shaft flat, 4 × M3 × 8 into inserts in the hand bottom | from inside the hand body, before the head |
| Worm → gearmotor | M4 set screw on the D flat; top pin in a thrust bearing in the palm dome | before the dome |
| Hand head → hand body | 3 × M3 × 8 into inserts, on the floor of the turret pillars | from above, before the dome |
| Palm dome → head | 6 × M3 × 10 into inserts in the cheek tops, beside the wheels; its plugs hold the finger pins | from above |
| Belt idlers | M5 × 35 in a slot of the bone + flange nut (button head at the elbow) | from outside |

## Order
### Base
1. Press the two 6810 with their spacers into the cartridge, screw on the retainer.
2. Screw the geared yaw motor (pinion on, grub on the key flat) and the cartridge to the deck. Leave the motor screws loose.
3. Fit the three rollers in their posts, low in the slots.
4. Fit the controller, the shoulder driver, the step-down, the fan, the display and the panel parts in the foot; screw the deck on.

### Shoulder
1. Screw the left hub to the pedestal wall.
2. Lay the geared NEMA23 in the pedestal channel, motor against the wall, and fit the two straps.
3. Put the pedestal on the turntable and fit the 8 M5 from below. Screw the ring gear under the turntable.
4. Drop the spindle into the yaw bearings and fit the clamp ring from below.
5. Push the yaw pinion into the ring gear until the play is gone and tighten the gearbox screws. Raise each roller until it just touches the turntable and tighten its nut. Turn the base by hand through the whole range: it must not bind.

### Upper arm and elbow
1. Fit the 6801 bearings in the bones (Loctite 638). Screw the two geared motors to the carrier, upper one facing left, lower one facing right; fit the 20-tooth pulleys.
2. Build the elbow stack on the Ø12 shaft: spider in the middle, the two side bevels with their bearings, shims and wave springs, then the two cradle walls, then the bones outside them. Screw the walls to the cradle plate. Belts go on before the walls close.
3. Slide the right bone over the gearbox shaft and the left bone with its 6810 over the hub. Fit the key and the flange hub from outside and bolt it to the bone. Put the carrier between the bones, belts over the motor pulleys, and bolt it through both bones.
4. Fit the forearm bevel hub through the cradle 6806 with timing marks facing, then the forearm base on the hex.
5. Fit the idlers, tension the belts (below), screw on the two covers.

### Forearm, wrist, hand
1. Screw the wrist motors to ulna and radius, build the wrist stack like the elbow one, screw the bones to the forearm base.
2. Roll shaft: bevel C on its flat, up through the flange bearings.
3. Hand as in `wrist_v0/hand_print_notes.md`: the hub in the hand bottom clamps on the roll shaft.
4. PCB coils with their ferrite sheet (`WRIST_BUILD.md`, section Coils), boards, camera.

## Shoulder balancer
1. Fit the two U-groove bearings in the bracket (M4 × 30 with nyloc nuts; the nut of the second one sits in the pocket under the bracket, so fit it before the bracket goes on the turntable) and the eye bolt in the anchor, nut loose.
2. Hook the spring on the eye bolt. Tie the cable to the spring, lead it round the pulley behind the pedestal, under the pulley below the shoulder, and up to the screw in the lobe of the left bone.
3. Set the length with the arm **upright**: the cable must be just taut, with the spring pulled about 7 mm. Lock the eye bolt with its second nut.
4. Check with the motors off: the upper arm with the forearm hanging should stay roughly where you leave it between upright and horizontal. If it falls forward, shorten the cable a little; if it springs back, lengthen it.
5. **Warning:** the spring is loaded with up to 220 N when the arm is forward. Bring the arm upright before touching the cable.

## Belt tension
- Push each idler against the back of the belt until the strand deflects about 2 mm under a firm finger push at mid span, then tighten the flange nut.
- A GT2 belt does not slip, it jumps teeth when it is slack. With the idlers the small pulleys have 7 or more teeth engaged. Holding 500 g needs about 60 N per elbow belt. Behind the 5:1 gearboxes a stalled motor could pull over 250 N, which is too much for a 10 mm belt: set the elbow driver current so the motor gives about 0.2 Nm.
- If a belt ever jumps: more tension first; if that is not enough, 20-tooth motor pulleys (the script recomputes the centre distance).

## Bench tests before building the whole arm
1. **Shoulder unit:** geared NEMA23 strapped in the pedestal, driver at 2.8 A. Hang 5 kg at 250 mm (12 Nm) on a bar bolted to the flange hub: it must lift it without losing steps.
2. **One elbow belt stage:** one bone, one motor, one side bevel. Stall the pulley by hand and check that the belt does not jump.
3. **Hand:** squeeze a kitchen scale between two fingers and read the force at R_ILIM 82 kΩ (expected about 16 N per finger). Then lift a full 0.5 l bottle and shake it.
4. **Inductive link:** 15 Ω, 10 W load on the hand rectifier: VM must stay between 10 and 15 V (`electronics/hand_passive.md`).

## What is not verified
- No part has been printed; clearances come from `common/fit.py` defaults.
- Bought-part dimensions are catalogue figures: mounting faces and hole patterns must be measured.
- Tool access was reasoned from the model, not tried.
- Boards, sensors and connectors have no modelled screws.
- Cable routes need real cables.
