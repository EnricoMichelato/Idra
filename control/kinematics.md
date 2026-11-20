# Arm kinematics (control/kinematics.py)

## Conventions
- The base frame has its origin on the shoulder axis, which crosses the yaw axis. z points up and the desk is at z = −216.
- With q = 0 the arm stands vertical; a positive shoulder angle leans it towards +y.
- Angles are in degrees, with the same signs as the 3D viewer.

| Joint | Rotation | Limits |
|---|---|---|
| q1 base | Rz(q1) | ±170 |
| q2 shoulder | Rx(−q2) | ±90 |
| q3 elbow | Rx(−q3) | −60…105 |
| q4 pronation | Rz(q4) | ±180 |
| q5 wrist pitch | Rx(q5) | ±90 |
| q6 roll | Rz(q6) | endless |

Dimensions: Lu 229, Lf 266, d6 205.4 (from the wrist centre to the grip centre; the fingertips are at about 228).

## Forward kinematics
T = Rz(q1)·Rx(−q2)·Tz(Lu)·Rx(−q3)·Rz(q4)·Tz(Lf)·Rx(q5)·Rz(q6)·Tz(d6)

The wrist is spherical (the axes of q4, q5 and q6 meet at the wrist centre), so:
- r = Lu sin q2 + Lf sin(q2+q3)
- z = Lu cos q2 + Lf cos(q2+q3)
- p_wrist = (−r sin q1, r cos q1, z)

## Inverse kinematics (8 solutions)
1. **Wrist centre:** p_w = p − d6·a, where a is the hand z axis.
2. **Base:** q1 = atan2(−x, y). The alternative is q1 + 180 with the arm leaning backwards.
3. **Shoulder and elbow:** cos q3 = (r² + z² − Lu² − Lf²)/(2 Lu Lf), with elbow up or down. Then q2 = atan2(r, z) − atan2(Lf sin q3, Lu + Lf cos q3).
4. **Wrist:** W = R03ᵀ·R with R03 = Rz(q1)·Rx(−(q2+q3)). W gives the ZXZ Euler angles (q4, q5, q6):
   - q5 = atan2(±√(W02² + W12²), W22);
   - q4 = atan2(W02, −W12);
   - q6 = atan2(W20, W21).
5. **Choosing a solution:** `ik()` picks the one inside the limits closest to the previous pose. q6 is moved to the nearest turn, because the roll is endless. With q5 = 0 it keeps q4 still and puts all the rotation on q6.

## Singularities
- **q5 = 0:** q4 and q6 are aligned.
- **q3 = 0:** arm straight, at the 495 mm edge.
- **Wrist centre on the yaw axis:** q1 is undefined.
- The rest pose (all zero) has all three at once.

## Motors
| Motor | Rotor angle | Steps/degree (200 × 16) |
|---|---|---|
| Base | 48.67·q1 | 432.59 |
| Shoulder | 28·q2 (two motors with 10:1 gearboxes, 20T pinions, same signal) | 248.89 |
| Elbow A | 44·(−q3 + q4/2) − q2 | 391.11 |
| Elbow B | 44·(−q3 − q4/2) − q2 | 391.11 |
| Wrist A | 5·(q5 + q6) | 44.44 |
| Wrist B | 5·(q5 − q6) | 44.44 |

The elbow motors are fixed to the turret, coaxial with the shoulder (inside the upper arm since v0.39): their pulleys do not turn with the upper arm, so a shoulder move turns the elbow side gears unless the elbow motors follow it (the −q2 term: the 16T pulleys on the shoulder axis are on the motor shafts).

The real direction of each motor is set once in the firmware (DIR pin) at homing.

The AS5600 encoders read the joints directly:
- q3 on the elbow shaft;
- −q4 on the idler satellite;
- q5 on the wrist axle;
- q6 in the T fitting.

## Checks
- Forward kinematics matches the 3D viewer matrices within 1e-13 mm.
- On 3000 random poses, every inverse solution puts the grip back where it was (error below 1e-12 mm).
- The JavaScript copy in the interactive page gives the same results as Python.

## Cartesian control in the 3D viewer
Turn it on with the C key or the button. There is also an on-screen pad for touch screens.

**Modes** (switch with M):

| Mode | ← → | ↑ ↓ | W S |
|---|---|---|---|
| Cylindrical (default) | orbit around the base | reach out and pull in | up and down |
| View | move on the plane, relative to the view | as ← → | up and down |
| Hand | sideways, relative to the view | forward and back along the fingers | sideways, relative to the view |
| Joints | base | shoulder | elbow |

In joints mode, R/F move pronation, A/D the wrist pitch and Q/E the roll.

**Orientation** (switch with O):
- **Hand follows the arm (default).** Pronation, wrist pitch and roll stay still. Forearm and hand act as one rigid link, and the position is solved in closed form with 3 joints. If a move is blocked, the wrist bends by itself as little as needed (2°, 5°, 10° or 20°). A/D move the wrist.
- **Hand fixed in space.** The orientation stays fixed in the world and the 6-joint inverse is used. If a move is blocked (usually by the wrist at ±90° when the forearm points up), the hand tilts about the horizontal as little as needed (2°, 5° or 10°) and says so.

**How it moves:**
- The keys move a target. The joints chase it with a synchronised move, a straight line in joint space, with top speeds of 90/90/120/240/240/400 °/s.
- Near singularities the arm slows down instead of stopping. If it falls more than 20° behind, the target waits.
- Rejected: jumps of more than 25° on base, shoulder or elbow (a branch change), and solutions outside the limits, out of reach or touching the desk.

**Ready pose** (P): joints 0 / 10 / 95 / 0 / −75 / 0, fingers vertical, 276 mm from the axis and 193 mm above the desk. Entering Cartesian control from a singular pose (elbow straight or wrist above the base), the arm goes there by itself.

## Reachable space in the 3D viewer
The "Reachable space" box draws a point cloud, in three modes:
- **All poses:** 900,000 random joint sets inside the limits. One point is kept per 15 mm cube, coloured by height.
- **Top grasp:** the inverse kinematics with the fingers vertical, computed on a grid in the arm plane and swept by ±170°. Forwards, the hand can grasp from above between about 300 and 490 mm from the base axis (grip centre 20–60 mm above the desk, joint limits only).
- **Section:** an 18 mm thick slice on the arm plane, following the base rotation.

All three modes leave out poses where the fingertips, wrist or elbow would touch the desk.
