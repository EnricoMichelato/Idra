# Arm v0.32 (exposed-mechanics style): shoulder, elbow, wrist, base and electronics

## Architecture (6 axes)
1. **Base (yaw):** a 100-tooth internal ring gear (module 1.5) under the Ø230 turntable and a 15-tooth pinion on a NEMA17 with a 5:1 high-precision planetary gearbox, fixed under the deck: 33.3:1 in total. Yaw is limited to ±170°.
   - **Bearing:** the turntable carries a hollow Ø50 spindle (Ø38 bore for the cables) that turns in two 6810 bearings, 25 mm apart.
   - **Cartridge:** both bearings sit in one printed cartridge, so their seats are coaxial by construction. The cartridge drops into the foot deck and is held by 4 M3 through its flange.
   - **Retention:** a printed spacer between the outer rings and one between the inner rings; a retainer ring under the cartridge (4 M3) and a clamp ring on the end of the spindle (4 M3).
   - **Rim rollers:** three 625 bearings on radial M5 screws, in slotted posts of the deck at 98 mm radius (6 mm inner post, buttresses on both sides of each post), carry the underside of the turntable. The tilting moment of the arm goes onto a 196 mm circle instead of the 25 mm between the spindle bearings. Raise each roller until it touches, then tighten its nut.
   - **Mesh adjustment:** the gearbox screws sit in slots of the deck. Push the pinion into the ring gear until the play is gone, then tighten. I used a fixed adjustment and not a spring, because a spring lets the pinion climb out of the teeth under load.
   - **Height:** the foot is 96 mm high (was 62) to take the motor with its gearbox; the shoulder axis is 183 mm above the desk.
   - **Deck:** the structural top of the foot. It carries the cartridge, the yaw motor (4 M3) and the Hall sensor post, and is screwed onto 7 pilasters of the foot wall.
2. **Shoulder:** a NEMA23 with an integrated steel planetary gearbox (20:1, 23HS22-2804S-HG20 class) lies on the shoulder axis, between the two bones. It replaces the printed cycloid of v0.28: about 90% efficient, 25 Nm, backlash 15–25 arcmin, no printed wear parts.
   - **Fixed unit:** motor and gearbox lie in a channel of the pedestal. The square motor body and gearbox flange sit in a square pocket, which takes the reaction torque (up to about 25 Nm) by shape. Two printed straps hold the unit down, each with 2 × M5 into inserts in the pedestal top.
   - **Pedestal:** 88 mm wide, one printed piece with the left wall. 8 × M5 from under the turntable go into inserts in its bottom. A tunnel under the motor takes the cables from the hollow spindle to the back.
   - **Right side:** the gearbox shaft (Ø14, keyed) passes through the right bone. A steel flange hub on the outer face of the bone is bolted to it with 4 × M5 into inserts. The shaft carries the right bone directly: about half the arm weight, far below its 400 N radial rating. The hub stands 22 mm proud of the bone, on the axis.
   - **Left side:** the bone turns on a 6810 bearing around a fixed hollow hub Ø50/30, screwed to the left wall of the pedestal with 4 countersunk M3 and centred by a register.
   - **Torsion box:** two covers are screwed onto the outer edges of the bones, from 48 to 138 mm from the axis. They carry the torque to the left bone.
   - **Gravity balancer:** a cable from a lobe of the left bone (40 mm from the axis, on the side opposite the arm) runs over a U-groove bearing 54 mm under the axis, round a second one, and on to an extension spring lying along the back of the pedestal. Set so that the spring force is proportional to the lobe-to-pulley distance, it gives 7 Nm × sin(shoulder angle), the same shape as gravity. The load goes into the left 6810, not into the gearbox shaft. The cable carries up to 220 N for as long as the arm is forward, so the printed parts are heavy: both pulleys are held on both sides (a fork with 6 mm cheeks and R8 fillets, and a C-shaped housing), the spring anchor is a 12 mm wall on a 64 mm foot with an R8 fillet, and each part has three M5 bolts. With 500 g the shoulder motor goes from 70% to 39% of its torque in the worst pose (`control/dynamics_report.md`).
   - **Driver:** the NEMA23 needs its full 2.8 A (TMC5160 or an external driver); on a TMC2209 the shoulder goes over the safety margin (`control/dynamics_report.md`).
   - **Width:** the arm is 143 mm wide: the shoulder unit needs 131 mm between the bones, and the bones are 6 mm aluminium.
   - **Measure first:** the pedestal channel and the bone bolt pattern come from catalogue figures. Measure the unit and the hub you buy and set `n23_*`, `gb_*` and `sh_hub_*` before printing.
3. **Elbow (flexion and pronation):** a differential like the wrist one, scaled up.
   - **Spiral bevels, module 3, 20° spiral angle:**
     - two 30-tooth side gears and a 15-tooth satellite, ratio 2:1, 90 mm diameter;
     - the two side gears are the same printed part twice. One is flipped and offset by half a tooth, because the satellite has an odd tooth count;
     - each side gear is printed together with a 160-tooth GT2 pulley (102 mm, a little bigger than the bevel), on a common Ø12 shaft.
   - **Shaft fixed to the cradle:** the shaft is clamped in the cradle walls (flat and M3 grub screw) and turns in a 6801 in each bone. The side bevels turn on the shaft on their own 6801s.
   - **Idler satellite:**
     - a second 15-tooth satellite, opposite the forearm one, sits inside the upper arm;
     - it is pressed on a Ø8 steel shaft that ends flush with its heel (no protruding hub);
     - it turns on two 688s in a printed spider, keyed to the shaft between the tips of the side gears.
   - **Forearm satellite supported at both ends:**
     - **Bottom:** a hollow stub rises from the spider, with a 6801 that fits into the tip of the satellite (like the wrist T fitting).
     - **Top:** the cradle holds the satellite with a 6806 and a 6810.
     - **Cables:** they enter the hollow stub through a side window and go up through the satellite bore.
     - **Effect:** the tooth load halves and the separating forces on shaft and cradle cancel out.
     - **Timing:** the idler meshes in any position because (30 + 30)/2 is an integer, so it needs no timing marks.
   - **Spiral thrust:** with 20° the side gears are always pushed outwards, in both directions of rotation. The satellite gets thrust in both directions, taken by the 61810 and the 6806.
   - **Axial preload:**
     - **Parts:** a wave spring (12 × 18, about 40–60 N) sits between the inner ring of the bone 6801 and a Ø15 × 0.8 shim on the inner ring of the pulley-side 6801.
     - **No rubbing:** both rings turn with the shaft.
     - **Closed load path:** a spacer carries the thrust to the cradle wall, so the load closes entirely inside the cradle.
     - **Tuning:** the preload is set with DIN 988 12 × 18 shim washers.
   - **Timing marks:**
     - Each side gear has one dot (position like B) and two dots (position like A) on its tip face.
     - The satellite heel has one dot towards B and two dots towards A.
     - Assemble with matching marks facing each other, elbow straight and pronation centred.
   - **Cradle:**
     - It sits **inside** the bones (the bones are the outermost parts, like an exoskeleton): two 7.5 mm walls between the big pulleys and the bones, and a 26 mm plate between the walls.
     - **Lightened plate:** four through pockets between the ring around the bearings and the outer frame, with a rib on the axis tying the ring to each wall (about 97 g instead of 141 g).
     - **Three pieces, each printed flat:** the plate (bearing seats as true circles) and two walls (shaft bore as a true circle). Each wall is fixed to the side of the plate with 4 countersunk M3 into heat-set inserts.
     - It is 84 mm wide, so it covers the whole forearm base (Ø80).
     - The plate holds a 6806 at the bottom, around the satellite hub, and a 61810 (50 × 65 × 7) at the top.
   - **Forearm base:** the base plate sits directly on the 61810 and has a hexagonal pocket that takes the hex end of the bevel hub (25 mm across flats): the pronation torque goes through the hex, and 3 × M3 into heat-set inserts in the hub end hold it down. There is no tube. Ulna and radius flank the two ends of the base plate and are screwed to it with 4 × M3 each, into heat-set inserts in its sides. They are 176 mm long. Each has a cover on its outer edge, in front of the wrist motors (torsion box).
   - **Motors:**
     - two salvaged 40 mm NEMA17s, each on a bolt-on 5:1 planetary gearbox. A geared motor is 80 mm long, so the two lie across the arm one above the other, 73 and 118 mm from the shoulder axis;
     - both sit in one printed carrier bolted through the two bones, which also ties the bones together;
     - one 10 mm steel-cord GT2 belt each, of different length (230 and 190 teeth), on 20-tooth pulleys: 5 × 160/20 = 40:1. The belts run right next to the bones (2.5 mm), so the pull of each belt is taken close to the bone bearing;
     - limit the elbow driver current: behind the gearbox a stalled motor would overload the belt.
   - **Adjustable tensioners, outside the belt:**
     - **Layout:** each belt has two smooth Ø12 flanged idlers, one per strand, outside the loop.
     - **Effect:** they press on the back of the belt and push the strand inwards by about 6 mm. The belt wraps more of the 16-tooth pulley: 163° instead of 142°, about 7 teeth engaged instead of 6.
     - **Adjustment:** each idler turns on an M5 screw in a slot in the bone (±3 mm). Push the idler against the belt and tighten the flange nut on the outer face. There are no sliders or holders any more.
     - **Why not slotted motors:** each motor shaft ends in a 688 bearing, in a printed block screwed to the inner face of the bone (2 M3), which carries the belt pull into the bone, so the motor cannot slide.
     - **Range:** the travel takes up −1.1 to +1.9 mm of belt length.
   - **Motion:** motors turning the same way give flexion; opposite ways give pronation (1:1 in the bevels, so side gear angle = pronation/2).
   - **Range:** elbow −60° to +105°, pronation about ±180° (limited by the cables).
   - **Encoders (2 × AS5600):**
     - **Flexion:**
       - a 6 × 2.5 diametric magnet sits in the right end of the shaft;
       - the chip is on a bracket screwed to the right bone (2 M3 inserts), coming from the shoulder side where the cradle never passes;
       - the bracket runs straight along the bone, with its foot 44–54 mm from the axis (2 M3 inserts side by side);
       - the wires stay still on the arm.
     - **Pronation:**
       - the magnet is in the tip of the idler satellite shaft;
       - the chip is on a 16 × 18 mm PCB in a side-open pocket of the spider;
       - 1:1 with pronation: absolute angle over ±180°;
       - it needs a small custom PCB, because common AS5600 modules (about 23 mm) do not fit.
     - **Wrist cables:** they pass beside the spider, at the front, and enter the Ø20 bore of the forearm satellite. The route needs checking with real cables.
4. **Wrist (v1.14, 76 mm flange that covers the Ø70 hand, R22 ears around the ulna and radius heads):** differential and endless roll with an inductive link. The wrist motors sit near the elbow (170-tooth belts, 14-tooth motor pulleys, 5.57:1), so there is less weight at the tip.
   - **Adjustable tensioner, one per belt:**
     - **Layout:** a smooth Ø12 idler presses on the back of the front strand, outside the loop, 92 mm below the wrist axis.
     - **Adjustment:** it turns on an M5 in a slot in the bone (±2.5 mm), locked by a flange nut on the outer face.
     - **Effect:** it pushes the strand inwards by 5 mm, and the travel takes up −0.4 to +0.7 mm of belt, enough for print tolerances.
   - **Spiral bevels:** 24/24 teeth, module 2, 20° spiral angle. A and B are the same part. With a 1:1 ratio and 20° of spiral, the side gears are always pushed outwards, as at the elbow.
   - **Fourth bevel, idler D:**
     - the same gear as output C, turned towards the forearm;
     - it turns on two 688s mounted on a Ø8 steel pin pressed into the T fitting;
     - everything ends flush with the heel, with less than 1 mm of lip;
     - the tooth load halves and the thrusts balance;
     - D's hub is closed.
   - **Flange inside the bones, in three pieces:** as at the elbow, a plate and two 8 mm walls printed flat, 3 countersunk M3 per wall. Ulna and radius are outside the walls (141 mm over the bones). Each wrist motor shaft ends in a 625 in a printed block screwed to the inner face of its bone.
   - **Axle fixed to the flange:** the Ø12 axle is clamped in the flange walls with M3 grub screws and turns in one 6801 per bone, as at the elbow. The T fitting is keyed to the axle with an M3 grub screw.
   - **Side gear preload:** identical to the elbow. A Ø15 × 0.8 shim sits on the inner ring of the pulley-side 6801, and a 12 × 18 wave spring presses on the inner ring of the bone 6801. A spacer carries the thrust to the flange wall.
   - **Encoders (2 × AS5600):**
     - **Pitch:**
       - a 6 × 2.5 magnet is glued on the right end of the axle;
       - the chip is on a bracket screwed to the radius (2 M3 inserts), coming from the forearm side where the flange never passes within ±90°.
     - **Roll:**
       - the magnet is in the lower tip of the roll shaft, which turns 1:1 with the hand;
       - the chip is on a round Ø12 PCB resting on the pitch axle, inside the T fitting, with the wires leaving towards +Y;
       - it measures the hand directly, not through the idler bevel;
       - absolute angle within one turn; the firmware counts turns, because the roll is endless.
   - **Improved inductive link:**
     - **PCB coils (since wrist v1.29):** each coil is a 4-layer ring board, Ø43.6 / Ø15.6 × 1.6 mm (`electronics/coil_pcb/`), backed by two rings of 0.3 mm flexible ferrite sheet, in a 2.4 mm pocket. 1 mm air gap between the two boards. They replace the Litz coils in ferrite cups of the earlier versions: a cup of that size with a hole for the shaft is not a catalogue part.
     - **Height:** the coils are 3 mm above the 6810 bearing, so the steel bearing stays out of the field. The hand rises by 3 mm.
     - **TX driver:** a 30 × 20 board on the flange, opposite the camera, with 2 M3 inserts. The coil leads are short and pass through two holes under the pads of the coil board.
   - **Roll shaft:** a Ø12 × 48 metal shaft; it used to be printed. Use **non-magnetic** material (304 stainless or 7075 aluminium), because the encoder magnet sits in its end. It needs two 2 mm flats: one under bevel C (D-shaped bore, slides on) and a 6 mm one at the hand end (D-shaped bore, epoxied).
   - **Passive hand (wrist v1.23 / hand v2.3):** there is no microcontroller in the hand any more.
     - **Flange side:** the camera's XIAO drives the TX bridge (IR2104 + 2 MOSFETs) and reads its current through a shunt and an INA180.
     - **Commands:** the frequency is the command. 98 kHz closes, 207 kHz opens, bridge off holds: the steel worm has a 3.6° lead angle and a 75:1 gearmotor behind it, so it cannot be back-driven.
     - **In the hand:** a rectifier, a DRV8871 and two LC filters that raise IN1 or IN2.
     - **End of travel:** the NC limit switch is in series with IN2 and stops the opening by itself. A grasp is detected by the stall, that is by the current rising on the TX side.
     - **Wiring:** the cable from the flange to the camera goes from 2 to 5 wires (5 V, ground, PWM, enable, current), in the same groove.
     - **Details:** schematic, values and bench test in `electronics/hand_passive.md`.

## Arm v0.46 / wrist v1.38: 'mech' bones
- **Mech styling** on every bone, no inserts: straight facets instead of fillets and curves, windows with pointed (chevron) ends instead of round slots (`mech_slot()`, `octagon()` in both scripts).
  - Humerus: half-octagon round the elbow, straight taper facet, straight strip, a chamfer onto the toothed disk; the teeth and the disk stay round.
  - Cheeks: bigger and chunkier, an octagon round the stub on a faceted full-width foot (the flanks start 20 mm below the axis, out of the sweep of the low through-bolt heads) (octagon inscribed R53, corners at R57.4; was a R48 circle), with a pointed window in each flank. Still 4 mm steel, left = right.
  - Ulna/radius (printed, flat): straight facets (no smoothing), faceted top inside the old radius round the axle, pointed windows.
- **Wrist motors touching** (centres 21.5 mm either side, 0.7 mm apart): the forearm bones are ±45 mm in Y (were ±50); lower forearm through-bolts at ±36 mm.

## Wrist v1.37: upright wrist motors
- The wrist motors stand upright (square to the forearm), only moved 25 mm to the side; the motor screws, bearing blocks and stand-offs follow. The belts still run about 10° off the forearm axis, from the side-shifted motor pulley to the wrist pulley.

## Wrist v1.36: wrist motors side by side
- The two wrist NEMA17 no longer sit back to back: they sit **side by side**, centres 25 mm either side of the forearm plane, each screwed to its own bone through four printed stand-offs. They overlap in X, so the belt planes come in to the bevel heels: the 100T pulley starts right at the heel (no flat sleeve any more) and the forearm is **116 mm across the bones instead of 149** (flange walls 94 mm). Each belt runs about 10° off the forearm axis; motor A (ulna) at +Y, motor B (radius) at −Y, B's drive the mirror of A's, so ulna and radius stay one part.
- The bones are a little wider in Y (±50 mm) to carry the motor screws; the lower through-bolts moved below the motors (±40, 169 mm below the wrist axis). Pitch axle Ø12 × 119, forearm through-bolts M5 × 130.
- **Camera, XIAO and TX driver board removed from the flange** (to be added later); the TX coil and its lead holes stay.

## Arm v0.45 / wrist v1.35: 400 mm belts
- The six belts are now **400 mm (200 teeth)**, the most common closed GT2 length (380 mm is hard to find). Lengths re-chosen for equal belts and 700 mm reach: **upper arm 229 mm, forearm 266 mm** (ulna and radius 176 mm). Intermediate shafts 131.9 mm up the bone, stage-2 idlers bend the belt 10 mm (adjustment −2.5…+3.2 mm); wrist motors 137 mm below the wrist axis.

## Arm v0.44 / wrist v1.34: steel cord everywhere, 20T small pulleys
- **Every small pulley has 20 teeth** (Ø12.7), so all six belts can be steel cord: elbow drive shafts 20T, intermediate shafts 20T inside, wrist motors 20T.
- **Elbow:** the intermediate pulley grows to 110T (Ø70, outside the bone): 110/20 × 160/20 = **44:1** (+10% torque). Stage 1 is exact at 121.6 mm, stage 2 has 85.5 mm between centres, idlers bend it 10 mm.
- **Wrist v1.34:** 100T pulley on the side bevel with the 20T motor pulley: **5:1** (was 6:1; the motors were already at 36%). Ulna and radius 198 mm; the motors sit 127 mm below the wrist axis.
- **Shoulder:** 20T steel pinions on the 56 bone teeth: **28:1** (was 40:1), faster and still well within the torque. The pinions are bigger, so the NEMA23 sit 7 mm lower and the shoulder axis is 216 mm above the desk (was 209).
- **Lengths** re-chosen so the six belts stay equal and the reach stays 700 mm: upper arm 207 mm, forearm 288 mm.

## Arm v0.43 / wrist v1.33: six equal belts
- **All six belts are GT2 10 mm, 190 teeth (380 mm):** elbow stage 1 and stage 2 on both sides, and the two wrist belts. One spare fits everywhere.
- To get there the upper arm is shorter and the forearm longer, with the reach unchanged at 700 mm: **upper arm 226 mm (was 255), forearm 269 mm (was 240)**. The intermediate shafts sit 140.5 mm up the bone (stage 1 exact with 190T); stage 2 has 85.5 mm between centres and its idlers, now 25 mm above the intermediate shaft, bend the belt about 10 mm (adjustment −2.6…+3.4 mm).
- **Wrist v1.33:** ulna and radius 179 mm (were 150) to make room for the longer belt; the wrist motors sit 139 mm below the wrist axis (were 122). The belt pulley is 2 mm bigger in radius than the bevel heel: 84 teeth, wrist ratio 6:1 (was 5.57:1, +8% torque). The back-side tensioners push 7 mm (were 5). Lower forearm through-bolts moved to 164 mm below the wrist axis.
- Viewer keyframes re-solved for the same targets.

## Arm v0.42: shoulder as low as it goes
- **The shoulder gearboxes stand straight on the turntable** (turntable top = underside of the 60 mm gearbox flanges). The printed motor block is now a 30 mm frame round the motors and gearboxes (face walls with the backlash slots, bolts down, end faces for the cheeks); its top stays below the reach of the toothed bone disks. Shoulder axis 209 mm above the desk (was 240), cheeks 30 mm shorter.
- **Printed cheek feet:** a wedge-shaped foot outside the bottom of each steel cheek, 2 × M5 from below into the turntable and 4 × M4 through the cheek into inserts: a wide footing for the thin plate.
- Viewer pick-and-place keyframes re-solved for the same targets above the desk.

## Arm v0.41: steel cheeks, cable through the motor block
- **Cheeks waterjet-cut from the same 4 mm steel as the upper-arm bones**, left and right the same part (each has both gearbox shaft holes), M4 tapped for the stub flanges. Much stiffer in their plane than the 15 mm printed ones and no creep; the shoulder is 186 mm across the stage-1 pulleys (was 208). Stubs 15 mm shorter.
- **Cable:** it no longer goes through the cheek and the stub. It rises from the yaw spindle along the open channel of the shoulder motor block, between the two NEMA23, into a window in the bottom of the right elbow motor bracket, up inside the bracket beside the coupling, and out of the top window into the arm as before (10–17 mm from the shoulder axis).
- Waiting for Enrico: common intermediate axle and tensioning with the four through-bolts (belts inside the bolts), which depend on whether stage 1 stays outside the bones.

## Arm v0.40: two belt stages instead of the elbow gearboxes, narrow upper arm again
- **Elbow motors without gearbox**, still inside the upper arm, back to back on the shoulder axis and fixed to the turret: a printed bracket (face plate + tube with a 688 in a spider) screwed with 4 × M3 to the inner end of the stub. A jaw coupling drives a Ø8 shaft through the stub; a second 688 sits in a printed bushing at the outer end of the stub; the 16T pulley is outside the cheek.
- **Stage 1 outside the bone:** 16T → 80T (5:1), 180-tooth belt (360 mm), to an intermediate shaft 130 mm up the bone. The shaft turns in two 688 in a printed hub on the outer face of the bone, clamped by 2 × M4 in slots: sliding the hub tensions stage 1.
- **Stage 2 inside:** 20T on the intermediate shaft → 160T side pulley (8:1), 224-tooth belt (448 mm), tensioned by the two through-bolts with two toothed idlers each, 40 mm above the intermediate shaft (adjustment −1.5…+2.1 mm of belt). 5 × 8 = 40:1 as before. The motor mixing now adds −1 × shoulder angle (the 16T is on the motor shaft).
- **Narrow again:** without the gearboxes the motors take ±40.5 mm, so the bones go back to the minimum set by the shoulder unit: 139 mm across (v0.39: 235). The elbow is the v0.38 one again (side bevel and 160T in one piece, pin Ø12 × 145). Shoulder 208 mm across the stage-1 pulleys.
- **Cable:** up the right cheek into the stub, along the top of the bore past the spider (its three spokes leave the top free), out of a window in the top half of the right motor bracket 10–17 mm from the axis, up beside the motor and over to the middle of the arm, below the intermediate shafts.
- No elbow gearboxes: about 0.6 kg off the turret, no gearbox backlash; the elbow encoder is on the joint anyway.

## Arm v0.39: elbow motors inside the arm, wide upper arm, 700 mm reach
- **Elbow motors inside the upper arm**, back to back on the shoulder axis with the shafts outward, still fixed to the turret: each gearbox face is screwed to a printed bracket that runs under the 20T pulley (where the belts never go: the arm swings ±90°, the belts stay in the upper half) and is screwed with 3 × M3 to the bottom of the inner end of the fixed stub. The pulley sits straight on the gearbox shaft: the v0.38 couplings, Ø8 shafts, spiders and 688 are gone. The shoulder coupling in the motor mixing stays (−5 × shoulder angle on both elbow motors).
- **Wide upper arm:** the motor stack (39 + 41 mm, plate, pulley) puts the belt planes at ±93.5 mm, so the bones are 227 mm apart inside (235 outside, was 139). The elbow is as wide: the 160T pulleys move out to the belt planes on a long sleeve of the side gear (one printed part with the bevel), and are separate printed parts screwed to a collar of the sleeve (3 × M3), so both print flat. Elbow shaft Ø12 × 238. The shoulder is 278 mm across the stub flanges (was about 360 with the motors outside).
- **Upper arm 255 mm** (was 205): reach 700 mm to the grip centre. Elbow belts 350 teeth (700 mm). The bone taper now follows the elbow (`taper` measured from the elbow axis). The tensioner bolts moved to 150 mm below the elbow (mid-span): there the idlers push each strand out by 11.6 mm for 350 teeth, and sliding the bolt ±3 mm changes the belt by −1.0…+1.3 mm (v0.38 had only −0.2…+0.6).
- **Cable:** up the right cheek into the right stub as before, then out of the stub end into the gap left on purpose between the right pulley and the bone (5.5 mm), up to 44 mm above the axis, across the belt plane inside its loop and up the middle of the arm. It crosses the joint 10–14 mm from the axis.
- Pick-and-place keyframes of the viewer animation re-solved with the inverse kinematics for the longer arm (same grip targets).
- **Numbers:** see STATUS (shoulder 28% at 500 g, 56% on one motor; elbow and wrist unchanged).

## Arm v0.38: elbow motors on the shoulder axis, slim upper arm, printed cheeks
- **Elbow motors outside the cheeks, coaxial with the shoulder** (on the turret, they do not move with the arm). Each geared NEMA17 sits on a printed mount on its cheek; a coupling and a Ø8 shaft through the hollow fixed stub (688 in a printed spider at its inner end) carry a 20T pulley in the belt plane of its side, on the shoulder axis. One belt per side straight to the 160T elbow pulley: 300 teeth (600 mm), both the same. Ratio still 40:1. The pulleys do not turn with the arm, so a shoulder move turns the elbow side gears: the motor mixing adds −5 × shoulder angle to both elbow motors (`control/kinematics.py`, `K_ES`).
- Considered and dropped: elbow motors in front of and behind the turret with a two-stage belt (the pulley on the shoulder axis must clear the Ø50 stub, which leaves little ratio for the belts and keeps the shoulder tall), and internal teeth in a window of the bone (the pinion moves toward the axis, so the ring would have to be about Ø200 to keep the shoulder gearboxes out of the arm's way).
- **Upper arm:** only the two belts, four toothed idlers and four through-bolts. The two tensioner bolts (one per side, mid-way up) each carry two idlers inside the belt loops, one per belt, so sliding a bolt tensions both belts equally. The two plain bolts sit near the shoulder, inside the loops.
- **Shoulder:** 10:1 planetaries on the short NEMA23 and 56 teeth on the bones (Ø112): 40:1. The gearboxes sit just below the reach of the belts when the arm swings ±90°. Shoulder axis 240 mm above the desk. The bones are a straight 80 mm strip with a toothed foot (R8 fillet) and the R33 elbow end; truss slots: an X of two diagonal slots between the tensioners, three slanted slots round the bearing. Bearings 6810 on Ø50 hollow stubs.
- **Cheeks printed in PETG-CF**, 15 mm, with M4 inserts for the stub flange and the elbow motor mount; the right one has the cable channel inside, from the bottom to the stub. Only the two steel upper-arm bones are still cut.
- **Numbers:** moving mass on the shoulder 4.6 kg (was 6.4). With 500 g: shoulder 24% of its torque (48% with one motor), elbow 41%, wrist 38%. Sag 1.6 mm. The shoulder is about 36 cm wide across the elbow motors.
- **Checked:** arm interference in 8 poses; belts and cables, including the arm cable against the turret-side drive parts at ±90°; kinematics with the new coupling.

## Arm v0.37: proportions, truss slots, printed forearm (wrist v1.32)
- The mechanics stay as in v0.36 (Enrico chose to keep the elbow motors low and the teeth in the bones); this pass is about looks.
- **Upper-arm bones, true teardrop:** the outline leaves the gear disk just before the first tooth and narrows in one smooth curve (a cubic tangent to the disk and to the R33 elbow end) instead of a disk, a strip and a small fillet.
- **Truss slots:** diagonal elongated slots instead of round holes. Upper arm: a zigzag of two slots along the neck and a ring of five slanted slots in the gear disk; forearm: a V of two slots under the pitch bearing and a slot each side of the motors. Every slot keeps 5 mm from the edge and from every hole; the two bones of each pair are still one part.
- **Shoulder lifted from the base:** the printed motor block has a 30 mm taller pedestal, so the shoulder axis is 268.5 mm above the desk (was 238.5). The cheeks are slimmer: a 28 mm web down to 75 mm below the axis, then a concave flare onto the block.
- **Wrist fully printed (v1.32):** ulna and radius are now 10 mm PETG-CF plates (were 6 mm aluminium), with the motor stand-offs and the through-bolt tubes printed too. Only bearings, shafts, belts, pulleys, motors and screws are bought. The forearm through-bolts become M5 × 160, the motor screws M3 × 40.
- **Through-bolts of the upper arm:** the plain pair moved to (±41, 70) to fit inside the narrower neck.
- **Checked:** arm interference in 8 poses, wrist in 4 poses, kinematics.

## Arm v0.36: elbow motors back beside the shoulder bearings, Ø320 base in quadrants
- **Elbow motors next to the shoulder bearings** (Enrico did not like them high up): side by side again, 32 mm above the shoulder axis, as in v0.34; equal belts of 270 teeth (540 mm). For them to clear the shoulder gearboxes over ±90°, the gearboxes moved 24 mm further from the axis: the teeth cut in the steel bones went from 60 to **82** (Ø164), shoulder ratio **29.3:1**, shoulder **238.5 mm** above the desk. The bones (874 g each) are wider at the shoulder (strip 112 mm at the gear, fillet R10) to hold the motors' carrier feet and the low through-bolts.
- Upper-arm through-bolts: the two tensioners as before, the two plain ones moved beside the belts (±46, 66 mm), clear of the motors and of the cheeks.
- **Base Ø320** (was 250): turntable and foot each in **four identical quadrants** (160 × 160 on the bed), joined by half-laps with M3 into inserts; every hole pattern is 4-fold symmetric. The turntable quadrants sit on a printed hub (spindle + flange) and are tied by the hub, the one-piece ring gear (8 screws, two per quadrant) and the shoulder motor block bolts. The foot quadrants are tied by the floor laps, a tab on each wall joint and the four desk lugs, which now sit on the joints. Each foot quadrant has one window in the middle of its wall; four different printed panels go in them (front: e-stop + display; I/O; vents; fan). Four rim rollers, one on the wall top of each quadrant, at R145. The deck stays one piece (Ø248) on eight pilasters.
- **Ring gear 146 teeth** (module 1.5, Ø239 printed in one piece, still fits the bed): base ratio 48.7:1 (was 40). The home sensor moved inside the ring teeth (R95).
- **Numbers:** shoulder at 37% of its torque with 500 g (was 54%: more ratio, elbow motors back near the axis), elbow 43%, wrist 39%. Moving mass on the shoulder 6.6 kg. Sag 1.6 mm.
- **Checked:** arm interference in 8 poses (shoulder ±90°); quadrants against each other, the hub and the panels; kinematics.

## Arm v0.35: geared shoulder, steel upper-arm bones, through-bolts (wrist v1.31)
- **Shoulder range ±90°** (Enrico meant ±90, not ±180).
- **Geared shoulder motors:** two short NEMA23 (56 mm) each behind a 5:1 planetary (60 square, Ø14 output), side by side under the axis, facing opposite ways. 14-tooth module 2 steel pinion on 60 teeth: 5 × 60/14 = 21.4:1. The gear is now Ø120 instead of Ø312, the motors sit 74 mm from the axis and the **shoulder is 215 mm above the desk** (was 312). The gearbox output bearings take the tooth force, so the pinion supports on the cheeks are gone. **No brake for now**: motor, gearbox and brake would not fit between the cheeks; a mechanical brake on the bones is to come. The two motors driven slightly against each other also take out the gearbox backlash.
- **Upper-arm bones in 4 mm steel with the teeth cut in:** no separate sectors and no sector screws; the teeth are concentric with the 6806 bore because they come from the same cut. 618 g each (was 959 g aluminium + 889 g sector). The two bones are still one part. Outline: the Ø120 gear (teeth only on the lower arc, plain toward the arm), an R10 fillet into a strip 86 mm wide that widens to 100 mm at the tensioners, then the R33 elbow end as before. 24 round lightening holes in mirror pairs.
- **Elbow motors higher:** side by side at 108 mm from the shoulder axis, out of the reach of the shoulder motors when the arm swings ±90°. The two elbow belts are 200 teeth (400 mm), a common length, 107 mm centre distance.
- **Through-bolts instead of covers:** 8 in all, M5 × 150 with aluminium tubes between the bones: in the upper arm the two tensioners (one per side, each carrying its belt idler) and two plain ones low down; in the forearm four, two per side beside the wrist motors. All covers are gone for now.
- **Forearm bones** widen to 74 mm below the pulleys to carry the through-bolts (171 g each); still one part for both sides.
- **Numbers:** moving mass on the shoulder 6.1 kg (was 8.5). With 500 g the shoulder uses 54% of its torque in the worst pose (was 43%: the big sectors used to balance part of the arm, and the elbow motors moved up the arm), elbow 43%, wrist 39%. One shoulder motor alone is no longer enough (108%). Sag under 500 g 1.7 mm.
- **Checked:** interference in 8 arm poses including the shoulder at +90°, −90° and −60°; belts and the arm cable against everything; wrist and kinematics checks.

## Arm v0.34: symmetry pass (teardrop bones, module-2 sectors, equal belts)
- **Elbow motors side by side** ("shoulder to shoulder"), both 32 mm above the shoulder axis, 22.5 mm either side of the arm centre line, facing opposite ways. The whole upper arm is symmetric under a half turn about its own axis, so:
  - the two elbow belts are the same: 270 teeth (540 mm) each, 174 mm centre distance (were 268 and 336);
  - the two upper-arm bones are **one part**, the right one is the left one turned over (one DXF, quantity 2);
  - the two gear sectors are **one part** too.
- **Shoulder gears, module 2:** 12-tooth steel pinion on a 156-tooth waterjet-cut steel sector (pitch Ø312), still 13:1. One arc of 295° for both sectors, symmetric about the downward direction and open toward the arm, 9 × M5 countersunk into each bone.
- **Shoulder range ±130°** (was −30…90). ±180° is not possible: past about ±135° the elbow reaches the turntable and the tensioner heads reach the cheek. Above 130° the sector would also have to close its gap, where the bone runs into the strip.
- **Teardrop bones:** a Ø302 disk around the shoulder axis that carries the sector, a 66 mm strip to the elbow end (R33, as before, for the elbow range), and an R40 concave fillet between them. 16 round lightening holes; 959 g each.
- **Shoulder 55 mm higher:** 312 mm above the desk (was 257), because the module-2 pinions sit 168 mm from the axis. Turntable top unchanged in shape, moved down with the motors.
- **Through-bolts instead of covers:** five Ø12 aluminium standoffs (countersunk M5 at both ends) and two M5 through-bolts that are also the belt tensioners: each carries one smooth idler between two tubes and slides in a slot cut in both bones. The printed upper-arm covers are gone for now.
- **Countersunk everything** through the upper-arm bones except the two tensioner bolts, so nothing stands proud where the bones turn past the cheeks. The cheek web is narrow down to 80 mm below the axis for the same reason.
- **Ulna and radius are one part** (wrist v1.30): both get the cable notch and the encoder holes.
- **Numbers:** moving mass on the shoulder 8.5 kg with the forearm (upper arm 6.0 kg, of which 1.9 kg bones and 1.8 kg steel sectors, mostly round the axis and opposite the arm). Shoulder at 43% of its torque with 500 g, elbow 42%, wrist 39%. Sag under 500 g 1.7 mm (elbow belts 0.54).
- **Checked:** interference in 8 arm poses including the shoulder at +130°, −130° and −100°; belts and the arm cable against everything; wrist and kinematics checks.

## Arm v0.33: symmetric shoulder with two motors, no spring
- **Two NEMA23 under the shoulder axis** (76 mm, 8 mm shaft, power-off brake), side by side and facing opposite ways. Each drives its own bone: a 13-tooth steel pinion on a 169-tooth laser-cut steel gear sector, 6 mm, bolted to the inner face of the bone with 5 × M5 countersunk. Ratio 13:1. Both bones get the same torque.
- **No spring:** the balancer (cable, pulleys, spring, bracket, anchor) is gone. With 500 g the shoulder uses 44% of the available torque in the worst pose, because the gear sectors and their bone lobes sit opposite the arm and the elbow motors are balanced about the axis. A brake on each motor holds the arm when the power goes.
- **Backlash:** each motor face sits in four slots along the line pinion → gear centre. Set the mesh with the arm upright. Driving the two motors a few tenths of a degree against each other removes the play.
- **Pinion support:** the far end of each motor shaft runs in a 688 in a printed puck on the cheek, so the tooth force (about 160 N at full motor torque) does not hang on the motor bearings.
- **Bones on fixed stubs:** each upper-arm bone turns on a 6806 around a hollow aluminium stub (Ø30, bore 20) held by a 6 mm aluminium cheek. The cheeks are laser cut like the bones and screwed to the end faces of the printed motor block (two halves, one per motor, split by the open cable channel). The cables enter the arm through the right stub, on the axis.
- **Shoulder 74 mm higher:** the axis is now 257 mm above the desk (was 183), the room needed for the motors under the gear sectors. The width over the stub flanges is 168 mm.
- **Elbow motors one above and one below the shoulder axis** (+33 and −37 mm along the arm, were +118 and +73). Belts 268 and 336 teeth (were 190 and 230). Their B idlers moved down to 35 mm from the shoulder, under the covers. The longer belts are softer: the elbow belts now give 0.63 mm of the 1.8 mm sag under 500 g (0.39 before), while the shoulder gives 0.10 mm (0.32 before).
- **Base as large as the bed allows:** turntable Ø248 (was 230), foot Ø250 (was 236), 120-tooth ring gear (was 100): base ratio 40:1 (was 33.3:1). Rim rollers at 110 mm (were 98).
- **Checked:** gear mesh in four shoulder angles (the pinions turn 13 times the shoulder in the model and the viewer), interference in 5 arm poses, cables at three shoulder angles.

## Wrist v1.29: PCB coils
- **Coils on printed circuit boards:** 4-layer ring boards Ø43.6 / Ø15.6 × 1.6 mm (`electronics/coil_pcb.py`: TX20, RX24, spare RX28), each backed by two rings of 0.3 mm flexible ferrite sheet. They replace the hand-wound Litz coils in ferrite cups: a cup of that size with a hole for the shaft is not a catalogue part, and boards are the same every time.
- **Pockets** in the flange boss and the hand bottom are now 2.4 mm deep (were 3.5); the coil faces stay 1 mm apart and 3 mm above the 6810.
- **Leads:** two holes under the two pads of each board (TX towards the driver, RX into the hand) instead of one.
- **Checks** moved into the repository (`tools/checks/`), and `STATUS.md` added as the handoff document.

## Hand v2.6
- **13.5 mm shorter**, same motor and grip: hub upside down with its neck in a 12 mm bottom, worm thread cut to 16 mm, no washer under the worm hub. Wrist centre to grip centre is now 205.4 mm.

## Hand v2.5
- **Finger supports:** one turret with three wheel slots and hollow pillars (9 mm cheeks) replaces the six free-standing cheeks.
- **Palm:** a dome screwed into the cheek tops with 6 M3; plugs under it hold the Ø5 × 32 finger pins (no e-clips). Same height as before.

## Exoskeleton pass (arm v0.32, wrist v1.27)
- **Cradle and wrist flange inside the bones:** the bones are now the outermost parts at both joints. Elbow: 143 mm over the bones, where the cradle was 168 mm wide. Wrist: 141 mm over the bones.
- **Both flanges in three flat-printed pieces** (plate + two walls), so every bearing seat and shaft bore prints as a true circle in the layer plane.
- **Elbow shaft** Ø12 × 145 (was 168). The preload shim and wave spring now sit inside the bore of the big pulley, so the wall can run 1 mm from it.
- **Motor shafts stay supported in the bones:** bearing blocks on the inner face of each bone (688 at the elbow, 625 at the wrist).
- **Elbow belt planes** 50 mm from the centre (were 51.5); gearbox wall of the carrier 4 mm; the four elbow idler bolts are button-head M5 so they pass the end of the opposite motor (0.75 mm).
- **Checked:** no interference in 5 poses over the joint ranges, cables excluded.

## Stiffness pass (v0.30)
- **Goal:** 500 g payload and less sag. Numbers in `control/stiffness_report.md` and `control/dynamics_report.md`.
- **Bones:** humerus, ulna and radius are flat 6 mm aluminium plates, laser cut (`LASER_CUT.md`). Every feature is a through cut; motor stand-offs are bought spacers again.
- **Covers:** each cover has six tabs with M3 inserts; the screws go through the bones from outside. No holes in the plate edges.
- **Belts:** steel cord instead of fibreglass, at elbow and wrist.
- **Elbow drive:** geared motors, 40:1 (see above).
- **Pronation bearings:** the cradle plate is 26 mm thick (was 16), so the 6806 and the 6810 are 19 mm apart instead of 9. The forearm is 10 mm longer (240 mm).
- **Sag under 500 g, arm stretched:** about 1.9 mm, against about 3.9 mm for v0.29 (estimates).
- **Parts:** 48 printed parts.

## Overbuilt pass (v0.29)
- **Goal:** reliable and effective rather than cheapest. Every drive with real load now has a bought steel gearbox, and the motor sizes follow the dynamics model (`control/dynamics_report.md`) and the grip model (`control/hand_grip_report.md`).
- **Shoulder:** geared NEMA23 instead of the printed cycloid; 2.8 A driver.
- **Elbow:** 48 mm NEMA17s and 160-tooth pulleys (10:1).
- **Base:** 5:1 planetary on the yaw motor, rim rollers, adjustable mesh.
- **Hand v2.4:** 25 mm metal gearmotor, bought steel worm held at both ends, thrust bearing in a palm cap, metal hub on the roll shaft; about 16 N per finger.
- **Margins with 200 g:** shoulder 55%, elbow 60%, wrist 27% of the available motor torque in the worst pose.
- **Parts:** 51 printed parts in 45 files.

## Print-ready pass (v0.28)
- **Parts:** 58 printed parts in 48 different files, down from 86 (51 parts in 45 files after v0.29). The 16 loose spacers became bosses on the bones; the 12 tensioner sliders and holders and the two slewing rings are gone.
- **STLs:** `tools/export_print.py` writes every part already lying in its print orientation, and `PRINT_LIST.md` lists file, quantity, material class and supports.
- **Audit:** `tools/print_audit.py` checks bed fit and unsupported overhang for every part.
- **Fasteners:** every joint between printed parts now has its screws or a shape lock in the CAD (see `ASSEMBLY.md`). Added in this pass: hex drive of the forearm hub, insert holes for the 4 covers in the bone edges, D-bores on the roll shaft.

## Easy printing and assembly (v0.25)
- **One clearance file:** `common/fit.py` holds all bearing seats, insert holes and through holes. Print the `common/fit_test.stl` coupon, pick the right column for each row and update the values. Instructions are in `common/fit_test.md`.
- **Unified hardware:**
  - Bearings: 6 types (6801, 688, 625, 6806, 6810, and one F6-12M thrust bearing in the hand).
  - Wrist axle Ø12 on 6801s, like the elbow, with the same springs and shims.
  - Elbow and wrist idler bevels both on 688s.
  - Screws: only M3 and M5, plus M2 for the micro switch and M6 for the desk. The M4s are gone: the shoulder drum and the base wings now use M5, and the cradle grub screw is M3.
  - Full list in `BOM.md`.
- **Motors mounted properly:** each NEMA17 is screwed to its bone with 4 × M3 × 30, from the outside through the bone and its stand-off bosses into the motor's threaded holes. The stand-offs are printed with the bone, not loose spacers. Earlier there were through screws across both motors, which cannot work because NEMA17 holes are blind.

## Shapes (v0.27)
- **Rounded flanges:** seen from above, the elbow cradle and the wrist flange have rounded corners (R28 at the elbow, R25 at the wrist). They still cover the forearm base, the hand, the camera and the TX driver. The ring around the axis stays solid.
- **Stronger spiders:** the elbow spider and the wrist T fitting are now solid blocks between the tips of the side bevels, with R5 rounded edges: 27 × 40 mm at the elbow, 31 × 32 mm at the wrist, instead of the thin cylinders used before.

## Style
- **3D viewer:** back to the original colour coding (blue arm, green cradle and flange, yellow gears), which reads better. Matte black stays the choice for printing and comes back with `MATERIAL_COLORS = True` in the scripts.
- **Printed parts:**
  - **Colour:** matte black, in three slightly different tones in the viewer only to tell them apart.
  - **Covers and cosmetic parts:** matte black PLA is fine.
  - **Structural parts:** bones, cradle, flange and parts near motors or gears are better in matte black PLA-CF or PETG. Plain PLA softens around 55 °C, and NEMA17s at full current get hotter than that.
  - **Gears and pulleys:** printed slightly lighter.
  - **Fin Ray fingers:** black TPU.
- **Electronics:** black solder mask (TX driver, AS5600, ESP32, hand board, fan), so no more green.
- **Metal parts** (shafts, screws, bearings, pins, idlers, motor pulleys): brushed steel. Black motors.
- **Accents:** red 7-segment display, red emergency button, black cables, red cables for camera and encoders.

## Stiffness
- **Thickness:** upper-arm and forearm bones are 10 mm.
- **Covers:**
  - **Part:** a single 131 × 90 × 4 mm part with 2 Ø34 portholes, used 4 times (2 on the upper arm, 2 on the forearm).
  - **Outer face:** a frame, rings around the portholes, a central rib and cross ribs (3 mm high, 3 mm wide), with recessed screw heads.
  - **Mounting:** it screws onto the outer edges of the bones with M3 screws in heat-set inserts. On the upper arm the edge tapers slightly, so the cover is tilted by about 2°.
- **Recommended material:** PETG-CF or PA-CF for structural parts.

## Lightening
- **Upper-arm bones:** one round Ø28 hole 134 mm from the shoulder, at least 6 mm from every screw, insert or slot (encoder bracket, tensioners, covers). The cable bundle runs through it.
- **Ulna and radius:** one round Ø22 hole 60 mm below the wrist axis, at least 6 mm from the encoder bracket screws and the covers. The wrist motor cables run through it.
- **Covers:** two Ø34 portholes each, which also cool the motors.

## Printability
Every printed part fits the 256 × 256 mm bed:
- **Upper-arm bones:** 278 × 80 mm, diagonally.
- **Base:** foot Ø236, turntable Ø230, deck Ø227.
- **Desk clamping wings:**
  - 4 separate parts, at 45°, 135°, 225° and 315°, so they do not cover the fan, button, panel or vents;
  - each has a 68 mm wide pad under the foot edge, R14 fillets in plan where it widens, and a 36 mm high curved wall against the foot with a central rib;
  - a countersunk M6 at the tip and 2 × M5 into the bottom of the foot.

## Base and electronics
- **Retro 7-segment display:** a 4-digit 0.36" module (TM1637 type, 2 wires to the ESP32), in the front-left wall of the foot, in the top band above the wing. In the 3D viewer it shows the base angle; on the real robot it can show angles, status, load or errors.
- **Cables (also in the 3D viewer):** a Ø7 black sleeve for the main bundle, grey Ø4 cables for the motors, red Ø3 for encoders and camera. Each joint is crossed on its axis or inside the mechanism, so the cable twists instead of being pulled.
  - **Base → shoulder:** the cable leaves a grommet in the turret, rises and enters the fixed left shoulder hub axially. From there, on the axis, it runs along the outer face of the left bone with a service loop.
  - **Upper arm:** the cable runs up the left bone and enters the round hole. Inside it picks up the cables of the two elbow NEMA17s and goes to the centre of the elbow, then through the spider and the satellite into the forearm.
  - **Forearm:**
    - The cable leaves the central hole of the forearm base, passes under the wrist motors and exits through a notch at the base of the ulna.
    - It runs up the ulna and picks up the wrist motor cables, which come out of the round hole.
    - It crosses the pitch joint on the axis, outside the head of the left axle, with a loop, and reaches the TX driver on the flange.
    - From there, the red camera power cable (5 V, 2 wires: the camera streams video over WiFi) runs in a 4 × 3 mm groove in the top face of the flange, along the front edge, and enters the camera mount through a hole. Nothing dangles.
  - **Pitch encoder:** its cable runs down the radius and enters the round hole.
- **Fixed foot:** floor and wall are one part, open at the top, so it prints upright with no supports. The wall rises to just under the turntable and hides the ring gear.
  - **Front:** emergency button in the wall, which cuts the 24 V to the drivers.
  - **Left side:** 24 V DC socket, USB-C and switch.
  - **Right side:** 40 mm fan. Vents at the back.
- **Inside, under the deck:** ESP32-S3 board with 5 TMC2209 drivers (base, elbow A and B, wrist A and B), a 2.8 A driver for the shoulder and a 24 → 5 V step-down.
- **Homing:** Hall sensor on a post of the deck and a magnet under the turntable.
- **Power:** external 24 V, 200 W supply. No 230 V mains inside the printed base.
- **Cables:** from the foot they rise through the hollow yaw spindle and leave the pedestal through a tunnel at its right end, under the gearbox shaft. They reach the shoulder axis beyond the flange hub and run up the outer face of the right bone. The balancer (spring, cable, pulleys) is on the left and at the back, so the two never cross.
- **TX coil driver:** near the wrist, with short wires.

## Numbers (estimates, to be checked with real masses)
- **Shoulder axis:** 183 mm above the desk.
- **Lengths and reach:** upper arm 205 mm, forearm 240 mm. The fingertips reach about 683 mm.
- **Shoulder:** NEMA23 at 2.8 A (1.26 Nm) × 20 × 0.9 gives about 22.7 Nm. 12.4 Nm is needed in the worst pose with 200 g.
- **Elbow:** 2 × 0.30 Nm × 40 × 0.80 gives about 19 Nm. About 6 Nm is needed in the worst pose with 500 g.

## Arm v0.47 / wrist v1.39 / hand v3.0: new spec, elbow motors outside, active hand
New spec (Enrico, 23 Nov 2026): pick objects up to 50 cm from the base, **1 kg at the wrist flange** (like a hook, plus the hand), as precise and robust as possible.
- **Hand v3.0 (`wrist_v0/hand_v3.py`, active):** Feetech STS3215 12 V servo standing under the worm, output coaxial with the hand, aluminium coupler (25T spline → 6 mm stub) carrying the worm hub. Steel worm module 1.5, single start, PD 20 (lead 4.3°, self-locking). Three **sector** wheels, 18 teeth module 1.5 in PA-CF: only the arc that ever meets the worm is toothed, so the bigger teeth do not make the hand wider. ESP32-S3 board and power board on the inner wall opposite the servo body; power only through the coils (the frequency command is gone). Limit switch kept for homing (the servo encoder is single-turn). Ø80 (the STS3215 output sits 11 mm from one end of its 45 mm body: it does not fit in Ø70). Grip centre 194.3 mm from the wrist centre (was 205.4), fingertips at 217. Wheel phase 3.0° (smallest overlap with the worm). Camera left out for now. `hand_v2.py` stays for reference.
- **Two flanges per joint (asked):** not possible at the elbow or the wrist: on the idle-bevel side the belts run down to the motors, a second flange would cut them. The T fitting (wrist) and the spider (elbow) already carry the idle bevel.
- **Elbow motors outside:** two new 48 mm NEMA17 (0.59 Nm, closed loop recommended) outside the cheeks, coaxial with the shoulder, each on a printed octagonal flange screwed over the stub flange into the cheek (4 × M4) with a spigot in the stub bore. Jaw coupling 5 → 8 in the spigot; Ø8 drive shaft on two 688 in a bushing inside the stub; 20T pulley on the inner end, in the stage-1 plane **inside** the bones.
- **Stage 1 inside the bones, common axle:** stage 1 runs inward of stage 2 (x = ±36.5, stage 2 at ±50). One fixed Ø10 axle across the arm at the intermediate height, in the existing bone slots, clamped on the outer faces by printed clamps (2 × M4 + an M3 grub); sliding it tensions both stage-1 belts at once. One pulley pair per side (110T + 20T in one piece) on two 6800. The stage-2 tensioners moved from 25 to 36 mm above the axle so their bolts clear the 110T pulleys. The plain through-bolts near the shoulder have their tubes split round the stage-1 belts.
- **Bone width:** kept at 139 mm (Enrico's choice "a", 24 Nov): the two NEMA23 with their gearboxes (about 130 mm long) between the bones set it, not the elbow. Moving them outside the cheeks would give about 125 mm.
- **Shoulder springs (instead of a brake):** four extension springs inside the arm, two per side in the plane x = ±50 (between stage 1 and the bone), from a printed anchor on each stub inner end (pin 30 mm above the axis) to hooks on the two low through-bolts (front and back, 63.6 mm from the axis at ±12.7°). The front + back pair gives 2·k·a·b·cos12.7°·sin(shoulder), like gravity. About 1.6 N/mm each for ~6 Nm per side (about 80 % of the arm weight with real springs). Plus dynamic braking: a relay shorts the NEMA23 phases when the 24 V drops.
- **Base:** two salvaged NEMA17 with 5:1 planetaries on opposite sides of the 146T ring (at ±Y), preloaded against each other in firmware: no yaw backlash, twice the torque. The controller moved to the +X side. A 20T printed pinion on a spring-loaded lever reads the ring (az 180°), magnet + 14-bit encoder under the deck: absolute yaw while the cables keep the hollow centre.
- **Cables:** up the yaw spindle, along the open channel between the two NEMA23, into the arm right on the shoulder axis (service loop), then up 12 mm behind the intermediate axle.
- **Pronation bearings:** cradle plate 26 → 50 mm thick: the 6806 and the 61810 are now 43 mm apart (were 19). This pushes the forearm base up: **forearm 266 → 290 mm**, which also gives the reach for fingers-down picks at 50 cm (shoulder → wrist centre 519 mm; IK solves 500 mm at desk height fingers down with the elbow at 31°).
- **Checks:** hand (3 finger angles), wrist (4 poses) and arm (8 standard poses plus elbow −60 / 105 and shoulder ±90) without hits. Dynamics with 1 kg at the flange: shoulder 24 % (49 % without springs), elbow 28 %, wrist 33 %; 1 kg in the hand at the grip would put the wrist at 71 % (out of spec). Sag 1.2 mm under 500 g at full reach.

## Arm v0.48: pulley planes swapped, big motor flanges
- **Intermediate pulley pairs turned round (Enrico):** stage 2 (20T on the pair → 160T on the side bevel) now runs in the INNER plane, right at the heel of the side bevel (x = ±31.3; it was held at ≥ 50, so an 18 mm hub sat between bevel and pulley); stage 1 (20T on the drive shaft → 110T on the pair) runs in the OUTER plane (x = ±44.8). The drive shafts are shorter (pulley 15 mm beyond the inner 688 instead of 25). Belt lengths unchanged: all six are still 400 mm.
- **Cradle walls** moved in with the pulleys (right outside them, 1 mm gap, like the wrist); longer spacers on the pin out to the bones. The bones stay 139 mm apart (NEMA23 between them).
- **Big printed flanges** over the whole steel cheek above its foot, one per side: a pocket for the stub flange, the motor 24 mm deep in a square sleeve, 4 M4 through the stub flange + 5 M4 on R46 into the cheek (5 new tapped holes in the cheek DXF). Plate 12 mm, the same pointed windows as the cheek. Shoulder 270 mm across the motors.
- **Springs** moved to x = ±57 (between the stage-1 pulleys and the bones), Ø8; the anchor has a fork for the spring eye.
- Checks: arm, 9 poses, no hits.

## Arm v0.49: springs outside, one big cheek support, shorter drive shafts, stepped cradle plate
- **Balance springs moved outside (one per side).** The four springs inside the arm (and their anchors on the stub ends and the hooks on the low through-bolts) are gone. Each side now has one extension spring in a plane 6 mm in front of the cheek support (x = ±93): from a Ø6 anchor pin pressed in a tower of the support, a = 55 mm above the shoulder axis, to a Ø10 steel pin through the bone, b = 100 mm up the arm (nut inside, collar outside; same hole in both bones). Zero-length layout: torque = k·a·b·sin(q), about 10 Nm for the pair with k ≈ 0.9 N/mm each. The spring line never comes closer than 48 mm to the axis, clear of the motor sleeve.
- **Stage 1 moved out, drive shafts shorter.** With the springs gone from the inside, the stage-1 plane goes as far out as the stub allows (x = ±53.5, 1 mm inside the stub inner end); stage 2 sits right inward of it (x = ±40). The 20T pulley now overhangs the inner 688 by about 7 mm (was 15); each Ø8 drive shaft is about 9 mm shorter.
- **Smooth neck on the elbow side bevels.** Between the bevel heel (x = 25.8) and the 160T pulley (now from x = 34) there is an 8 mm smooth Ø44 neck; the pulley has its own inner flange. The cradle walls moved out with the pulleys (x = 47 to 54.5).
- **One big printed cheek support per side (PETG-CF)**, replacing both the cheek foot and the motor flange: it covers the whole outer face of the steel cheek down to the turntable, stands on a 10 mm foot slab (34 × 144 mm) bolted with 4 × M5 from under the turntable (holes at 8 and 16 mm out from the cheek face), and has three ribs: two side gussets at y = ±66 and a centre web from the foot up under the motor sleeve. It still takes the stub flange in a pocket, the motor in a 24 mm square sleeve, 4 + 5 M4 into the cheek, and the 4 M4 from inside the cheek into inserts (the old foot screws). Counterbores clear the heads of the cheek-to-block M5. Turntable: two new M5 holes per side.
- **Cradle plate stepped:** a 26 mm frame (walls screwed at 6.5 and 19.5 mm) and a round boss up to 40 mm that carries the 61810. Pronation bearings 33 mm apart (was 43). Steel would not help here: the thickness is there for the bearing spacing, not for strength. Forearm 280 mm (was 290); the 50 cm reach constraint is dropped (Enrico).

## Arm v0.50 / wrist v1.40 (25 Nov 2026)
- **Elbow differential:** 28T side bevels on 15T satellites, module 3, spiral 35° (was 30/15, 20°). Each side bevel and its stage-2 pulley are one printed part: the pulley is now 130T (was 160T), as big as the bevel, and the teeth run on past the heel into the pulley (the heel cross-section carried on for 3 mm, then a ramp closes the gaps): no flat cut at the tooth ends. Elbow ratio 110/20 × 130/20 = 35.75:1 (was 44:1). With the same 400 mm belts the upper arm grows to 249 mm.
- **Pronation:** the cradle plate is a flat 18 mm plate: the 6806 at its bottom, right above the satellite, and an AXK5578 needle thrust bearing (2 × AS5578) sunk in its top. The forearm base plate (10 mm, a rectangle as big as the cradle) sits on the thrust bearing, 0.2 mm above the plate, and the satellite hub is screwed into it from above (3 × M3): the screws pull the hub up, its collar loads the 6806 upward, so the thrust bearing is preloaded and takes the tilting moment (shims under the hub hex set it). No 61810 any more. Slim spider (no cross block).
- **Cradle walls** 90 mm deep, as deep as the forearm, and ending where ulna and radius end (x = ±63); three screws per wall in one row at mid-thickness.
- **Tensioners:** one per belt stage, smooth Ø18 idlers outside the loops on the belt back: stage 1 on its back strand (the intermediate axle is now fixed), stage 2 on its front strand. No other through-bolts in the upper arm.
- **Bones, for stiffness:** humerus with one straight taper from R33 at the elbow to 84 mm at the root and round holes on the neutral axis only (no 'mech' facets or windows). Ulna and radius with the same shape (round top, one flare to the full 90 mm width, straight to the foot), round holes on the centre line (the one between the motor screws moved onto the axis of the other motor), no through-bolts (the two flanges tie the pair; covers later), a Ø8 cable exit at the foot. They are 161 mm long: the wrist motors stand right on the forearm base plate. Forearm 236 mm.
- **Cheeks and cheek supports:** the same smooth outline (a circle round the stub, tangents to the foot), no windows.
- **Wrist v1.40:** 32T side bevels (module 2, face 13, spiral 35°) as big as the 100T pulleys (5:1 again), teeth running on into the pulleys; 24T roll gear, so the wrist stays narrow (the roll turns 32/24 times the side-gear difference). Roll-gear hub 1 mm and an 8 mm flange plate: the hand flange is 6 mm lower. Flange 80 mm deep with walls 10 mm thick, shaped like the elbow walls.
- **Cable path** modelled: up the arm, a service loop round the elbow axis between the side bevels, into the spider window, up the satellite hub, through the forearm base plate and out along a groove under the wrist motors to the ulna.
- **Base Ø300** (was 320): rim rollers at R135, outside the ring gear.
- **Parking ("Bed" in the viewer):** shoulder +38°, elbow folded back to −130° (free up to there; at −140° the forearm meets the intermediate pulleys), wrist −90°.
