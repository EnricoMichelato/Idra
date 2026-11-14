# Arm v0.31 (exposed-mechanics style): shoulder, elbow, wrist, base and electronics

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
     - It sits outside the bones, with 10 mm walls (R26 ears that wrap the bone head where the bearing is) and a 16 mm plate.
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
     - **Why not slotted motors:** each motor shaft ends in a 625 bearing in the bone, which carries the belt pull, so the motor cannot slide.
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
4. **Wrist (v1.14, 76 mm flange that covers the Ø70 hand, R22 ears around the ulna and radius heads):** differential and endless roll with an inductive link. The wrist motors sit near the elbow (170-tooth belts, 4.86:1), so there is less weight at the tip.
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
     - **Ferrite cups:** each coil sits in a ferrite cup, with a 1.5 mm bottom plus inner and outer rims flush with the coil face, like the two halves of a pot core. The field stays closed between the cups: more coupling and less leakage towards metal parts.
     - **Sizes:** ferrite outer diameter 44 mm, inner 15 mm. Litz coils 40/18 × 2 mm, 1 mm air gap.
     - **Height:** the coils are 3 mm above the 6810 bearing, so the steel bearing stays out of the field. The hand rises by 3 mm.
     - **TX driver:** a 30 × 20 board on the flange, opposite the camera, with 2 M3 inserts. The coil leads are short and pass through a hole next to the seat.
   - **Roll shaft:** a Ø12 × 48 metal shaft; it used to be printed. Use **non-magnetic** material (304 stainless or 7075 aluminium), because the encoder magnet sits in its end. It needs two 2 mm flats: one under bevel C (D-shaped bore, slides on) and a 6 mm one at the hand end (D-shaped bore, epoxied).
   - **Passive hand (wrist v1.23 / hand v2.3):** there is no microcontroller in the hand any more.
     - **Flange side:** the camera's XIAO drives the TX bridge (IR2104 + 2 MOSFETs) and reads its current through a shunt and an INA180.
     - **Commands:** the frequency is the command. 98 kHz closes, 207 kHz opens, bridge off holds: the steel worm has a 3.6° lead angle and a 75:1 gearmotor behind it, so it cannot be back-driven.
     - **In the hand:** a rectifier, a DRV8871 and two LC filters that raise IN1 or IN2.
     - **End of travel:** the NC limit switch is in series with IN2 and stops the opening by itself. A grasp is detected by the stall, that is by the current rising on the TX side.
     - **Wiring:** the cable from the flange to the camera goes from 2 to 5 wires (5 V, ground, PWM, enable, current), in the same groove.
     - **Details:** schematic, values and bench test in `electronics/hand_passive.md`.

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
