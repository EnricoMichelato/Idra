# IDRA: project status and handoff

Read this first when picking the project up in a new session. It says where things stand, how the repository is organised, how to regenerate and check everything, and what is still open.

Last updated: 17 November 2026 · arm v0.32 · wrist v1.29 · hand v2.6 · nothing built yet; the wrist is being built first.

## The robot in one paragraph

Six-axis desk arm, about 650 mm reach, 500 g payload target. Base yaw (NEMA17 + 5:1 planetary, rim rollers), shoulder (geared NEMA23 20:1 between the two upper-arm bones, spring balancer), elbow bevel differential (flexion + pronation, two salvaged NEMA17 with 5:1 gearboxes, belts 20T→160T), wrist bevel differential (pitch ±90° + endless roll, two salvaged NEMA17, belts 14T→78T), passive three-finger Fin Ray hand (steel worm, Ø25 75:1 gear motor). The hand gets its power through two PCB coils across the roll joint; the grip command is the frequency of that power. Aluminium 6 mm bones on the outside of every joint ("exoskeleton"), printed parts in PETG / PETG-CF, gears PETG-CF or PA-CF, fingers TPU.

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
| `tools/` | `export_print.py` (oriented STLs + `PRINT_LIST.md`), `export_dxf.py` (bones + `LASER_CUT.md`), `wrist_kit.py`, `mass_model.py`, `print_audit.py`, `build_site.py`, `preview.js` |
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
python3 tools/checks/check_arm.py           # 5 arm poses, 1–2 min each after a 2 min build; or pass poses: 60,90,105,120,-60,45
python3 tools/checks/check_kinematics.py    # FK/IK round trips, viewer vs fk, mixing
node tools/checks/screenshot.js mechanical/cad/arm_v0/arm_3d.html shot.png
```

Every check exits with code 1 if something clashes. Accepted on purpose: closed fingertips touching (skipped); belts and cables are left out of the checks (drawn approximately). Last full run (17 Nov 2026, v0.32 / v1.29 / v2.6): **no hits** in any check.

After a change, also: bump the version labels (`grep -rn "wrist v1.29"`), re-export STLs and DXFs, rebuild the viewers and the site, update `base_notes.md`, `BOM.md`, `ASSEMBLY.md` as needed, and the copies of the docs in the claude.ai project.

## Key numbers

| | |
|---|---|
| Link lengths | shoulder height 183, upper arm 205, forearm 240, wrist centre → grip centre 205.4 mm |
| Ratios | yaw 33.3, shoulder 20, elbow 40 (5 × 160/20), wrist 5.57 (78/14) |
| Joint limits | yaw ±170, shoulder −30…90, elbow −60…105, pronation ±180, wrist pitch ±90, roll endless |
| Motor load at 500 g (worst pose) | shoulder 38%, elbow 42%, wrist 39% of the available torque (safe limit 60%) |
| Grip | about 16 N per finger (current limit 0.78 A, R_ILIM 82 kΩ), worm self-locking |
| Sag under 500 g at full reach | about 1.8 mm (belts dominate; joint encoders after the belts can correct it) |
| Moving mass on the shoulder | 5.4 kg |

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
- Steel-cord belts on the 14-tooth wrist pulleys (tight bend): fibreglass is the safer first choice.
- Strength of the hand turret and dome, and of the screwed joints between cradle walls and plate (no dowels; adding two Ø3 dowels per wall was offered, not done).
- The elbow idler bolts must be button head: 0.75 mm from the opposite motor.
- Printed bones (for the bench wrist) are much less stiff than aluminium.

**Proposed and put on hold by Enrico** (do not start without asking)
- Staggered wrist motors for a slimmer forearm; wrist bevel module 2.5–3; elbow bevel face 16 mm and lightened side gears; one-piece hand cage.
- Planetary 5:1 on the wrist motors (only if more payload is wanted).
- Shorter shoulder shaft protrusion; restyling the kinematics and electronics pages in the dark look; encoders on shoulder and base.

## Build plan (as agreed)

1. Fit coupon → `fit.py`.
2. Wrist and hand mechanics on the bench, printed bones (`WRIST_BUILD.md`, kit from `tools/wrist_kit.py`).
3. Hand driven straight from a 12 V supply.
4. Inductive link: PCB coils and the two boards on the bench with a 15 Ω load, then in the wrist (`electronics/hand_passive.md`, 8 steps).
5. The rest of the arm.

## History in brief

v0.28 print-ready pass → v0.29 overbuilt (steel gearboxes, bearings, worm hand) → v0.30 stiffness (aluminium bones, geared elbow, steel-cord belts) → v0.31 shoulder spring balancer, name IDRA, site with keyboard control → v0.32 cradle and wrist flange inside the bones, each in three flat-printed pieces; hand v2.5 turret + dome palm, v2.6 13.5 mm shorter; 14-tooth wrist pulleys; lightened cradle plate → wrist v1.29 PCB coils replace Litz coils in ferrite cups. Details in `mechanical/cad/arm_v0/base_notes.md`.
