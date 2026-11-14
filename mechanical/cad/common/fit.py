"""
Printer fit parameters, shared by every part of the Wireless Robot Arm (arm_v0, wrist_v0, the fit coupon).
Print fit_test.stl once, check which column fits on each row, copy the matching values here and rebuild:
every bearing seat, insert hole and clearance hole of the robot follows.
All values are DIAMETERS in mm.
"""
FIT = dict(
    brg=0.10,    # added to a bearing OD for its seat in a printed part (press fit; larger = looser)
    shaft=-0.10, # added to a bearing BORE for a printed boss that goes inside it (smaller = looser)
    rot=0.40,    # clearance of a printed bore turning on a steel pin
    ins3=4.0,    # hole for an M3 heat-set insert (5.7 mm long)
    ins5=6.4,    # hole for an M5 heat-set insert (9.5 mm long)
    m2=2.2, m3=3.4, m5=5.5,   # clearance holes
    tap3=2.5,    # M3 thread cut by the screw itself (grub screws in printed parts)
)

def seat(od):
    """radius of a printed seat for a bearing of outer diameter od"""
    return od/2 + FIT['brg']/2

def boss(bore):
    """radius of a printed boss inside a bearing of inner diameter bore"""
    return bore/2 + FIT['shaft']/2

R_INS3, R_INS5 = FIT['ins3']/2, FIT['ins5']/2
R_M2, R_M3, R_M5 = FIT['m2']/2, FIT['m3']/2, FIT['m5']/2
R_TAP3 = FIT['tap3']/2

# hardware standard of the whole robot (one list, used by the BOM)
BEARINGS = {'6801': (12, 21, 5), '688': (8, 16, 5), '625': (5, 16, 5), '6802': (15, 24, 5), '6806': (30, 42, 7), '6810': (50, 65, 7), '61814': (70, 90, 10)}
