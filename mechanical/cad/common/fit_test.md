# Fit test coupon (fit_test.stl)

All the robot's bearing seats, insert holes and through holes read their values from `common/fit.py`. Print the coupon once, pick the column that fits on each row, copy the value into `fit.py` and regenerate the parts: they come out right for your printer.

**Printing:** flat, with the same material and settings as the robot parts (PETG-CF or PA-CF, 0.2 mm layers, 4 perimeters, 20–40% infill), no supports. It is about 105 cm³, so 40–60 g.

**Columns:** −1 / 0 / +1 are the current `fit.py` value minus one step, equal, and plus one step.

| Row | What you test | It fits when | Key in fit.py | Step |
|---|---|---|---|---|
| 6806, 6801, 688/625 | bearing seat | it goes in with thumb pressure or a vice, and does not fall out when turned over | `brg` | 0.1 |
| 6801 IN | printed pin inside the bearing bore | the bearing slides on with little force and does not wobble | `shaft` | 0.1 |
| PIN 12, PIN 8 | hole turning on a steel shaft | the shaft turns freely with no visible play | `rot` | 0.2 |
| INS M5, INS M3 | heat-set insert hole | the insert goes in with the iron and the melted material fills the knurls | `ins5`, `ins3` | 0.2 |
| M5, M3 | through hole | the screw passes without forcing | `m5`, `m3` | 0.2 |

**Updating:** if the +1 column fits the bearings, add one step in `fit.py` (for example `brg=0.10` becomes `brg=0.20`). Then run the scripts again (`arm_v0.py`, `wrist_v0.py`, `export_print.py`), and reprint the coupon only if you want a confirmation.

The large seats (6810, 61814) follow the same rule: in practice the deviation of a printed hole is almost constant with diameter.
