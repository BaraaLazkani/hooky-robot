# 2026-05-19: position control

**Dates:** 2026-05-19 for the Y-pair and the 30/-20 sketch. 2026-06-01 for `xy_position` (modification times).
**Original folders:** `19-5/two motor go to position`, `19-5/one motor go to position`, `19-5/x y position code` (each contained `sketch_may19a`).

## What changed

Control moves from velocity to position. The sketches convert a target in centimetres to a step count, keep a software step counter, and pulse a motor until the counter matches. Roller radius is now 30.5 mm (`r = 0.0305`), recalibrated from the earlier 30 mm.

## Sketches

| Sketch | Original folder | What it does |
|---|---|---|
| `y_axis_dual_motor_position` | `two motor go to position` | The two Y motors, STEP 23 and 22, DIR 18 and 19, move to `target_x = 5.0` cm. Prints the position on every loop. |
| `xy_position_test_30_-20` | `one motor go to position` | Despite the folder name, this drives the X motor (STEP 24, DIR 35) and the two Y motors to (30, -20) cm. |
| `xy_position` | `x y position code` | Same code with the target set to (10, 5) cm. |

## How they work

- `cm_to_steps()` and `steps_to_cm()` convert between centimetres and steps.
- Each `loop()` iteration issues one step in the direction of the error, then waits 500 microseconds high and 500 low. Both axes are blocking, so they do not run at independent speeds.
- Position is purely open-loop, with the origin at power-up.
- A `Serial.print` on every iteration slows the stepping.

## Notes

The two X/Y sketches differ only in their target values. The original folder called one of them "one motor", which does not match its code, so the new name describes what the code does.
