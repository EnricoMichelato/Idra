# IDRA: project status and handoff

Read this first when picking the project up in a new session. It says where things stand, how the repository is organised, how to regenerate and check everything, and what is still open.

Last updated: 19 November 2026 · arm v0.46 · wrist v1.38 · hand v2.6 · nothing built yet; the wrist is being built first.

**Git:** v0.32 is staged in the index and not committed yet; the v0.33–v0.45 changes are in the working tree only (each version alone is in `idra-repo-v0.3x.zip`). Commit first (that records v0.32), then `git add -A` and commit again.

## The robot in one paragraph

Six-axis desk arm, about 700 mm reach, 500 g payload target. Base yaw (Ø320, turntable and foot in four identical printed quadrants; NEMA17 + 5:1 planetary on a 146T ring, rim rollers), shoulder (two short NEMA23 with 10:1 planetaries under the axis, each driving one bone through a 20T steel pinion on 56 teeth cut in the 4 mm steel bone, 28:1, ±90°, no spring, no brake yet), elbow bevel differential (flexion + pronation, two salvaged NEMA17 without gearbox inside the upper arm, back to back on the shoulder axis and fixed to the turret, two belt stages per side: 16T→80T outside the bone to an intermediate shaft half-way up, 20T→160T inside; the shoulder coupling is in the motor mixing), wrist bevel differential (pitch ±90° + endless roll, two salvaged NEMA17, belts 14T→78T), passive three-finger Fin Ray hand (steel worm, Ø25 75:1 gear motor). The hand gets its power through two PCB coils across the roll joint; the grip command is the frequency of that power. Bones on the outside of every joint ("exoskeleton"), in identical pairs with diagonal truss slots, tied by through-bolts (no covers): 4 mm steel in the upper arm with the shoulder teeth cut in, 10 mm printed PETG-CF in the forearm; the shoulder cheeks are 4 mm steel too (one sheet for all cut parts), printed parts in PETG / PETG-CF, gears PETG-CF or PA-CF, fingers TPU.

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
| `mechanical/cad/wrist_v0/wrist_v0.py` | forearm, wrist differential, flange, coils; `hand_v2.py` the hand |
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

Every check exits with code 1 if something clashes. Accepted on purpose: closed fingertips touching (skipped); belts and cables are left out of the checks (drawn approximately). Last full run (20 Nov 2026, v0.46 / v1.38 / v2.6): **no hits** in any check.

After a change, also: bump the version labels (`grep -rn "wrist v1.29"`), re-export STLs and DXFs, rebuild the viewers and the site, update `base_notes.md`, `BOM.md`, `ASSEMBLY.md` as needed, and the copies of the docs in the claude.ai project.

## Key numbers

| | |
|---|---|
| Link lengths | shoulder height 216, upper arm 229, forearm 266, wrist centre → grip centre 205.4 mm |
| Ratios | yaw 48.7 (5 × 146/15), shoulder 28 (10 × 56/20, two geared motors), elbow 44 (110/20 × 160/20, two belt stages), wrist 5 (100/20) |
| Joint limits | yaw ±170, shoulder ±90, elbow −60…105, pronation ±180, wrist pitch ±90, roll endless |
| Motor load at 500 g (worst pose) | shoulder 40% (no spring), elbow 39%, wrist 43% of the available torque (safe limit 60%); with one shoulder motor only 81% (no longer a safe fallback since the 20T pinions). Payload limit for 60%: about 0.8 kg, set by the wrist |
| Grip | about 16 N per finger (current limit 0.78 A, R_ILIM 82 kΩ), worm self-locking |
| Sag under 500 g at full reach | about 1.4 mm (belts dominate; joint encoders after the belts can correct it) |
| Moving mass on the shoulder | 5.0 kg with the forearm (the elbow motors are fixed to the turret since v0.38) |
| Widths | upper arm and elbow 139 mm across the bones, shoulder 186 mm across the stage-1 pulleys |

## Open items

**Not started**
- Firmware (stepper control, the two differentials, homing, hand frequency command, keyboard / Xbox control, link to the web simulator).
- PCB layout of the hand board (RX) and of the flange driver (TX): only schematics exist.
- AS5600 encoder boards small enough for the joints (16 × 18 mm and Ø12).
- Cable connectors and small-part fixings in the CAD.

**Calculated but never tested** (check on the bench)
- Coil turn counts (TX20 / RX24, spare RX28) and the hand voltage under load; coil inductance with ferrite sheet; heating of the coil boards, the roll shaft and the 6810 near the field.
- KiCad files of the coils: parsed by a script, never opened in KiCad. Run the DRC before ordering.
- Backlash of the printed bevels at the fingertip; belt tooth jump under stall.
- Steel-cord belts everywhere: no pulley is smaller than 20 teeth (Ø12.7); no datasheet minimum was found for the cheap steel-cord GT2 loops, 20T is the usual conservative choice.
- Strength of the hand turret and dome, and of the screwed joints between cradle walls and plate (no dowels; adding two Ø3 dowels per wall was offered, not done).
- Shoulder: waterjet tooth quality on the 4 mm steel bones, mesh with the 20T module-2 pinions, backlash setting in the slots, the two motors sharing the load, gearbox backlash taken out by preloading the motors against each other. **No brake: the arm drops with the power off** until the mechanical brake on the bones is designed.
- All six belts are 200 teeth (400 mm), steel cord. Stage-2 idlers bend the belt about 10 mm and give −2.6…+3.4 mm; stage 1 is set by sliding the intermediate hub; the wrist tensioners give only −0.5…+0.7 mm.
- Elbow motors inside the arm hang from the stub ends on printed brackets (about 0.3 kg each): check that the bracket does not creep; the cable leaves through a window in the right bracket, check it with the real harness.
- Intermediate shafts: printed hubs on the steel bones with two 688, stage-1 pulleys 28 mm outside the bone; check the hub does not tilt under belt tension (about 60 N stage 1, 200 N stage 2).
- Quadrant base: the half-lap joints and wall tabs are untested; check that the turntable stays flat once the four quadrants, hub and ring gear are screwed together.
- Fixed shoulder stubs need a small lathe job (Ø30 tube, flange, ring groove).
- Printed forearm bones (10 mm PETG-CF) are much less stiff than aluminium; creep under constant belt tension to be watched.
- All printed covers were removed (v0.34 upper arm, v1.31 forearm); 8 through-bolts tie the bones. New panels are left for later.
- Mechanical brake on the shoulder bones: wanted by Enrico, not designed yet.

**Proposed and put on hold by Enrico** (do not start without asking)
- Common fixed axle for the two intermediate pulley pairs, and tensioning both elbow belt stages with the four through-bolts, two by two, belts running inside the bolts (back-side idlers). Needs stage 1 moved inside the bones (or outer C-brackets): Enrico is thinking about it (19 Nov).
- Staggered wrist motors for a slimmer forearm; wrist bevel module 2.5–3; elbow bevel face 16 mm and lightened side gears; one-piece hand cage.
- Planetary 5:1 on the wrist motors (only if more payload is wanted).
- Restyling the kinematics and electronics pages in the dark look; encoders on shoulder and base.

## Build plan (as agreed)

1. Fit coupon → `fit.py`.
2. Wrist and hand mechanics on the bench, printed bones (`WRIST_BUILD.md`, kit from `tools/wrist_kit.py`).
3. Hand driven straight from a 12 V supply.
4. Inductive link: PCB coils and the two boards on the bench with a 15 Ω load, then in the wrist (`electronics/hand_passive.md`, 8 steps).
5. The rest of the arm.

## History in brief

v0.28 print-ready pass → v0.29 overbuilt (steel gearboxes, bearings, worm hand) → v0.30 stiffness (aluminium bones, geared elbow, steel-cord belts) → v0.31 shoulder spring balancer, name IDRA, site with keyboard control → v0.32 cradle and wrist flange inside the bones, each in three flat-printed pieces; hand v2.5 turret + dome palm, v2.6 13.5 mm shorter; 14-tooth wrist pulleys; lightened cradle plate → wrist v1.29 PCB coils replace Litz coils in ferrite cups → v0.33 symmetric shoulder: two NEMA23 with brakes on steel gear sectors (13:1), no spring, shoulder 74 mm higher, elbow motors above and below the shoulder axis, base Ø250 with a 120T ring → v0.34 symmetry: elbow motors side by side with equal belts, identical teardrop bones and identical module-2 sectors, shoulder ±130° and 55 mm higher, through-bolt tensioners and standoffs instead of covers; ulna and radius one part (wrist v1.30) → v0.35: shoulder ±90° with two short NEMA23 + 5:1 planetaries on teeth cut in 4 mm steel upper-arm bones (no sectors, no brake yet), shoulder 215 mm above the desk, elbow motors higher with 400 mm belts, 8 through-bolts and no covers (wrist v1.31) → v0.36: elbow motors back beside the shoulder bearings, 82-tooth bones (29.3:1), shoulder 238.5 mm; base Ø320 with turntable and foot in four identical quadrants, 146T one-piece ring gear (48.7:1), four panels in the foot windows → v0.37: teardrop upper-arm bones and diagonal truss slots, shoulder on a taller pedestal (268.5 mm), slimmer cheeks; wrist v1.32 fully printed (10 mm PETG-CF forearm bones) → v0.38: elbow motors outside the cheeks on the shoulder axis (upper arm holds only belts), 56-tooth bones with 10:1 planetaries (40:1), shoulder 240 mm, printed PETG-CF cheeks → v0.39: elbow motors inside the upper arm, back to back on the shoulder axis (brackets on the stubs), bones 235 mm apart, upper arm 255 mm for a 700 mm reach → v0.40: no elbow gearboxes, two belt stages (16→80 outside the bone to an intermediate shaft, 20→160 inside), bones back to 139 mm → v0.41: steel cheeks (left = right), cable up through the shoulder motor block → v0.42: shoulder gearboxes straight on the turntable, shoulder 209 mm above the desk, printed feet for the steel cheeks → v0.43 / wrist v1.33: all six belts the same (190T): upper arm 226, forearm 269 (reach still 700), wrist pulley 84T (6:1) → v0.44 / wrist v1.34: steel cord everywhere with 20T small pulleys; 110T intermediate (elbow 44:1), wrist 100T (5:1), 20T shoulder pinions (28:1); upper arm 207, forearm 288 → v0.45 / wrist v1.35: belts 400 mm (200T), the common size: upper arm 229, forearm 266 → wrist v1.36: wrist motors side by side (pulleys right at the bevel heels, forearm 116 mm across instead of 149), camera and TX driver off the flange for now → wrist v1.37: the wrist motors stand upright (only the belts run slanted) → v0.46 / wrist v1.38: 'mech' styling (faceted outlines, pointed windows) on humerus, cheeks and forearm bones; bigger faceted cheeks; wrist motors touching, forearm bones ±45 mm. Details in `mechanical/cad/arm_v0/base_notes.md`.
