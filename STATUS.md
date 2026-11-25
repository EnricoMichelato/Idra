# IDRA: project status and handoff

Read this first when picking the project up in a new session. It says where things stand, how the repository is organised, how to regenerate and check everything, and what is still open.

Last updated: 25 November 2026 · arm v0.50 · wrist v1.40 · hand v3.0 · nothing built yet; the wrist is being built first.

**Git:** v0.46 is on GitHub (commit d39c9eb). The v0.47 to v0.50 changes are in the working tree: `git add -A` and commit.

## The robot in one paragraph

Six-axis desk arm, picks on the desk fingers-down up to about 48 cm from the base (about 680 mm to the grip; the 50 cm target was dropped in v0.49), spec **1 kg at the wrist flange** plus the hand. Base yaw (Ø300, turntable and foot in four identical printed quadrants; **two** salvaged NEMA17 + 5:1 planetaries on opposite sides of a 146T ring, preloaded; 20T encoder pinion on a spring lever; rim rollers), shoulder (two short NEMA23 with 10:1 planetaries under the axis, each driving one bone through a 20T steel pinion on 56 teeth cut in the 4 mm steel bone, 28:1, ±90°; **two balance springs outside the cheek supports** + dynamic braking, no brake), elbow bevel differential (flexion + pronation; 28T side bevels on 15T satellites, module 3, spiral 35°; two **48 mm NEMA17 outside the cheeks** on the shoulder axis, Ø8 shafts through the stubs, stage 1 20T→110T inside the bones to one fixed Ø10 axle, stage 2 20T→130T on pulleys printed in one piece with the side bevels, 36:1; one tensioner per stage, smooth idlers outside the loops; motors held in one big printed support over each steel cheek; the forearm sits on a preloaded needle thrust bearing + one 6806), wrist bevel differential (pitch ±90° + endless roll; 32T side bevels in one piece with the 100T pulleys, 24T roll gear; two salvaged NEMA17, 5:1), **active three-finger Fin Ray hand** (Feetech STS3215 servo, module 1.5 steel worm, 18-tooth sector wheels, ESP32-S3 in the hand). The hand gets its power through two PCB coils across the roll joint. Bones on the outside of every joint ("exoskeleton"), in identical pairs shaped for stiffness (one straight taper, round holes on the neutral axis), tied only by the joints and the two tensioner bolts (covers planned): 4 mm steel in the upper arm with the shoulder teeth cut in, 10 mm printed PETG-CF in the forearm; the shoulder cheeks are 4 mm steel too, printed parts in PETG / PETG-CF, gears PETG-CF or PA-CF, fingers TPU.

## Ground rules

- **Language:** everything in the repository is in English. Enrico writes in Italian; replies to him are in Italian.
- **Public pages:** the README and the site (`docs/`) say the wrist roll is endless, **without explaining how** (no coils, no induction, no frequency command there). The method is documented only in `electronics/`, `mechanical/cad/` and the CAD scripts.
- **Licence:** `LICENSE` is "all rights reserved". GitHub: `enricomichelato/Idra` (capital I), site `https://enricomichelato.github.io/Idra/`.
- **Commits:** Enrico commits and pushes himself. Stage with `git add -A`; do not commit unless he asks.
- **Approvals:** Enrico approves publishing the 3D viewer without being asked. He wants changed files written straight into his project folder when the session can reach it, not a zip to copy by hand. Some changes he wants discussed first ("don't start yet"): ask before large redesigns.
- **Style of the model:** bones aluminium, motors as they are, everything else dark "Doctor Octopus" (gunmetal, brass, carbon, dark red TPU). Colours live in `color_of()` of each CAD script.

## Repository map

| Path | What |
|---|---|
| `mechanical/cad/arm_v0/arm_v0.py` | the whole arm (imports the wrist and hand); `build(P)` returns `(parts, info)`, `pose()`, `material()`, `color_of()`, `group()`, `category()`, `SKIP` |
| `mechanical/cad/wrist_v0/wrist_v0.py` | forearm, wrist differential, flange, coils; `hand_v3.py` the hand (`hand_v2.py`: the old passive hand, kept for reference) |
| `mechanical/cad/common/fit.py` | print clearances (`seat`, `boss`, insert and screw radii); calibrate with `fit_test.py` |
| `mechanical/cad/arm_v0/viewer_build.py`, `viewer_tpl.html` | 3D viewer of the arm (`arm_3d.html`, about 9 MB); `--tpl` reuses cached meshes, `--recolor` only recolours |
| `mechanical/cad/wrist_v0/viewer_build.py` | 3D viewer of the wrist and hand alone (`wrist_3d.html`) |
| `mechanical/cad/*.md` | `BOM`, `ASSEMBLY`, `PRINT_LIST` (generated), `LASER_CUT` (generated), `WRIST_BUILD` (bench build of the wrist) |
| `mechanical/cad/arm_v0/base_notes.md` | design log: every change and why, newest passes near the end |
| `control/` | kinematics (forward, inverse, motor mixing), dynamics (motor sizing), hand grip, stiffness; each writes a `*_report.md` |
| `electronics/` | hand board (RX) and flange driver (TX): `hand_passive.md`, schematics, page; `coil_pcb.py` + `coil_pcb/` the PCB coils |
| `tools/` | `export_print.py` (oriented STLs + `PRINT_LIST.md`), `export_dxf.py` (cut plates + `LASER_CUT.md`, checks that paired parts are identical), `wrist_kit.py`, `mass_model.py`, `print_audit.py`, `build_site.py`, `preview.js` |
| `tools/checks/` | interference and kinematics checks (below) |
| `docs/` | GitHub Pages site, built by `tools/build_site.py` |

Generated files go to `out/` folders and are not in git. `arm_3d.html`, `wrist_3d.html` and `hand_passive.html` are ignored too: the published copies are in `docs/`.

## Regenerate everything

```bash
pip install -r requirements.txt                      # cadquery, cq_gears, numpy, schemdraw
python3 tools/export_print.py                        # out/print/*.stl + PRINT_LIST.md   (~10 min)
python3 tools/export_dxf.py                          # out/laser/*.dxf + LASER_CUT.md
python3 tools/wrist_kit.py                           # out/wrist_kit/: everything for the bench wrist
python3 tools/mass_model.py && python3 control/dynamics.py && python3 control/stiffness.py
cd mechanical/cad/arm_v0 && python3 viewer_build.py  # arm_3d.html                         (~10 min)
cd ../wrist_v0 && python3 viewer_build.py            # wrist_3d.html
cd ../../../electronics && python3 hand_passive_schematic.py && python3 build_page.py && python3 coil_pcb.py
cd .. && python3 tools/build_site.py --preview       # docs/ (preview image needs Playwright)
```

## Checks (run after every geometry change)

```bash
python3 tools/checks/check_hand.py          # valid solids + fingers closed / half / open   (~1 min)
python3 tools/checks/check_wrist.py         # 4 pitch/roll poses                             (~1 min)
python3 tools/checks/check_arm.py           # 8 arm poses (shoulder up to ±90), 1–2 min each after a 2 min build; or pass poses: 60,90,105,120,-60,45
python3 tools/checks/check_kinematics.py    # FK/IK round trips, viewer vs fk, mixing
node tools/checks/screenshot.js mechanical/cad/arm_v0/arm_3d.html shot.png
```

Every check exits with code 1 if something clashes. Accepted on purpose: closed fingertips touching (skipped); belts, cables and the shoulder springs are left out of the checks (drawn approximately). Last full run (25 Nov 2026, v0.50 / v1.40 / v3.0): **no hits** (arm: elbow −130 to 120 and shoulder ±90; wrist 4 poses).

After a change, also: bump the version labels (`grep -rn "wrist v1.29"`), re-export STLs and DXFs, rebuild the viewers and the site, update `base_notes.md`, `BOM.md`, `ASSEMBLY.md` as needed, and the copies of the docs in the claude.ai project.

## Key numbers

| | |
|---|---|
| Link lengths | shoulder height 216, upper arm 249, forearm 236, wrist centre → grip centre 196 mm (hand v3.0; flange 58 mm) |
| Ratios | yaw 48.7 (5 × 146/15), shoulder 28 (10 × 56/20, two geared motors), elbow 35.75 (110/20 × 130/20, two belt stages), wrist 5 (100/20); bevels: elbow 28/15, wrist 32/24 (roll 24/32 of the side-gear difference) |
| Joint limits | yaw ±170, shoulder ±90, elbow −60…105, pronation ±180, wrist pitch ±90, roll endless |
| Motor load, 1 kg at the flange (worst pose) | shoulder 20% with the springs (44% without), elbow 28%, wrist 32% of the available torque (safe limit 60%). 1 kg held at the grip: wrist 71% (out of spec). Base 90° turn 0.67 s |
| Grip | about 18 N per finger at 0.6 A servo current (about 17% of its torque), worm self-locking |
| Sag under 500 g at full reach | about 1.1 mm (belts dominate; joint encoders after the belts can correct it) |
| Moving mass on the shoulder | 4.8 kg with the forearm (elbow motors on the turret) |
| Widths | upper arm and elbow 139 mm across the bones (set by the NEMA23 between them), forearm 126 mm across ulna and radius (= the elbow walls), shoulder 270 mm across the elbow motors (one big printed support over each cheek, down to the turntable) |

## Open items

**Not started**
- Firmware (stepper control with closed loop, the two differentials, homing, preload of the paired motors at base and shoulder, the STS3215 over the hand radio, keyboard / Xbox control, link to the web simulator).
- **Hand board (active, v3.0):** schematic and layout (ESP32-S3, rectifier, bulk capacitor, 3.3 V buck, half-duplex servo bus). The passive DRV8871 board in `electronics/hand_passive.md` is superseded; the TX driver now runs at a fixed frequency and must give 12–15 W.
- Encoder boards (14 bit, MT6701 / AS5047P) small enough for the joints; shoulder encoder not modelled yet.
- Dynamic-braking relay in the base electronics.
- Cable connectors and small-part fixings in the CAD.

**Calculated but never tested** (check on the bench)
- Coil turn counts and the hand voltage under load (now 12–15 W with the servo and the ESP32-S3); heating near the field.
- KiCad files of the coils: never opened in KiCad. Run the DRC before ordering.
- Backlash of the printed bevels at the fingertip; belt tooth jump under stall.
- Shoulder springs: rate and geometry from a simple model (two outside, about 0.9 N/mm each, a = 55, b = 100 mm, ~80 % of the arm weight); real extension springs are not zero-length. Measure the torque left at 45° and 90°.
- Shoulder: tooth quality on the 4 mm steel bones, mesh with the 20T pinions, the two motors sharing the load.
- STS3215: output position on its body (11.3 mm from the near end assumed) and the spline coupler: measure before printing the hand.
- Common intermediate axle: Ø10, belt loads near the bones; check that the clamps do not slip.
- Elbow drive shafts: 20T pulley about 7 mm beyond the inner 688 (v0.49).
- Pronation on a preloaded needle thrust bearing + one 6806 (v0.50): check the tilt play with 1 kg at the flange; more shims if the thrust bearing unloads on one side.
- Teeth running on into the pulleys (v0.50): print a pair first and check the mesh near the heel.
- Quadrant base, fixed shoulder stubs (lathe job), printed forearm bones (creep), printed cradle plate (flat 18 mm, thrust bearing sunk in its top).

**Decided against / not possible**
- Two flanges per joint (elbow and wrist): the belts run where the second flange would be.
- Narrower upper arm: would need the NEMA23 outside the cheeks (Enrico chose to keep them inside).

**Proposed and put on hold by Enrico** (do not start without asking)
- Staggered wrist motors; steel bevels later. (Next: wrist without flanges, Source Robotics style, bigger roll gear and module.)
- Planetary 5:1 on the wrist motors (not needed for the 1 kg flange spec).
- Restyling the kinematics and electronics pages in the dark look.

## Build plan (as agreed)

1. Fit coupon → `fit.py`.
2. Wrist and hand mechanics on the bench, printed bones (`WRIST_BUILD.md`, kit from `tools/wrist_kit.py`).
3. Hand driven straight from a 12 V supply.
4. Inductive link: PCB coils and the two boards on the bench with a 15 Ω load, then in the wrist (`electronics/hand_passive.md`, 8 steps).
5. The rest of the arm.

## History in brief

v0.28 print-ready pass → v0.29 overbuilt (steel gearboxes, bearings, worm hand) → v0.30 stiffness (aluminium bones, geared elbow, steel-cord belts) → v0.31 shoulder spring balancer, name IDRA, site with keyboard control → v0.32 cradle and wrist flange inside the bones, each in three flat-printed pieces; hand v2.5 turret + dome palm, v2.6 13.5 mm shorter; 14-tooth wrist pulleys; lightened cradle plate → wrist v1.29 PCB coils replace Litz coils in ferrite cups → v0.33 symmetric shoulder: two NEMA23 with brakes on steel gear sectors (13:1), no spring, shoulder 74 mm higher, elbow motors above and below the shoulder axis, base Ø250 with a 120T ring → v0.34 symmetry: elbow motors side by side with equal belts, identical teardrop bones and identical module-2 sectors, shoulder ±130° and 55 mm higher, through-bolt tensioners and standoffs instead of covers; ulna and radius one part (wrist v1.30) → v0.35: shoulder ±90° with two short NEMA23 + 5:1 planetaries on teeth cut in 4 mm steel upper-arm bones (no sectors, no brake yet), shoulder 215 mm above the desk, elbow motors higher with 400 mm belts, 8 through-bolts and no covers (wrist v1.31) → v0.36: elbow motors back beside the shoulder bearings, 82-tooth bones (29.3:1), shoulder 238.5 mm; base Ø320 with turntable and foot in four identical quadrants, 146T one-piece ring gear (48.7:1), four panels in the foot windows → v0.37: teardrop upper-arm bones and diagonal truss slots, shoulder on a taller pedestal (268.5 mm), slimmer cheeks; wrist v1.32 fully printed (10 mm PETG-CF forearm bones) → v0.38: elbow motors outside the cheeks on the shoulder axis (upper arm holds only belts), 56-tooth bones with 10:1 planetaries (40:1), shoulder 240 mm, printed PETG-CF cheeks → v0.39: elbow motors inside the upper arm, back to back on the shoulder axis (brackets on the stubs), bones 235 mm apart, upper arm 255 mm for a 700 mm reach → v0.40: no elbow gearboxes, two belt stages (16→80 outside the bone to an intermediate shaft, 20→160 inside), bones back to 139 mm → v0.41: steel cheeks (left = right), cable up through the shoulder motor block → v0.42: shoulder gearboxes straight on the turntable, shoulder 209 mm above the desk, printed feet for the steel cheeks → v0.43 / wrist v1.33: all six belts the same (190T): upper arm 226, forearm 269 (reach still 700), wrist pulley 84T (6:1) → v0.44 / wrist v1.34: steel cord everywhere with 20T small pulleys; 110T intermediate (elbow 44:1), wrist 100T (5:1), 20T shoulder pinions (28:1); upper arm 207, forearm 288 → v0.45 / wrist v1.35: belts 400 mm (200T), the common size: upper arm 229, forearm 266 → wrist v1.36: wrist motors side by side (pulleys right at the bevel heels, forearm 116 mm across instead of 149), camera and TX driver off the flange for now → wrist v1.37: the wrist motors stand upright (only the belts run slanted) → v0.46 / wrist v1.38: 'mech' styling (faceted outlines, pointed windows) on humerus, cheeks and forearm bones; bigger faceted cheeks; wrist motors touching, forearm bones ±45 mm → v0.47 / wrist v1.39 / hand v3.0: spec 1 kg at the flange and picks at 50 cm; active hand (STS3215, module 1.5 worm, sector wheels); 48 mm elbow motors outside the cheeks, stage 1 inside the bones on a common axle; four shoulder balance springs; two yaw motors + encoder pinion; pronation bearings 43 mm apart, forearm 290 → v0.48: intermediate pulley pairs turned round (stage 2 inner at the bevel heels, stage 1 outer: short drive shafts), cradle walls in, big printed motor flanges over the steel cheeks, springs at x = ±57 → v0.49: springs outside (one per side, anchor on the support, pin in the bone), stage 1 out to the stubs (shorter drive shafts), smooth neck on the elbow side bevels, cheek foot + motor flange merged into one big ribbed cheek support, stepped cradle plate, forearm 280 → v0.50 / wrist v1.40: 28/15 elbow bevels and 32/24 wrist bevels (spiral 35°, teeth running into the pulleys), 130T elbow pulleys (36:1, upper arm 249), forearm on a needle thrust bearing (no 61810), one tensioner per belt stage outside the loops, bones and cheeks reshaped for stiffness without through-bolts, forearm 236, base Ø300, Bed pose. Details in `mechanical/cad/arm_v0/base_notes.md`.
