# 2026-04-14: velocity control

**Dates:** 2026-04-14 for `single_motor_velocity_simple` and `single_motor_velocity_timer_isr`. 2026-05-19 for the other two (modification times). The sketches were named `sketch_apr14a/b`, so April 14 is when they were written.
**Original folders:** everything under `14-4/` except `motor_control/` (see the 2025 stages).

## What changed

The sketches take a velocity in cm/s over serial at 115200 baud and convert it to a step frequency with the 30 mm roller radius and 8x microstepping. Pulses come from `delayMicroseconds` in `loop()`, or from a timer in the ISR variant. Enter `10` for forward at 10 cm/s, `-10` for reverse and `0` to stop.

## Sketches

| Sketch | Original folder | What it does |
|---|---|---|
| `single_motor_velocity_simple` | `the simple code of baraa code for arduinouno` | One motor, STEP 26, DIR 35. Pulse train in `loop()`. Direction from the sign of the input. |
| `single_motor_velocity` | `the code for the individual motor` | One motor, STEP 24, DIR 35. Adds a minimum half-period of 100 microseconds as a speed limit. |
| `dual_motor_velocity` | `the code for the both motor` | The two Y motors together: STEP 23 and 22, DIR 18 and 19. Both are stepped in the same `loop()` iteration. Minimum half-period 200 microseconds. A short delay after a direction change gives the driver time to settle. |
| `single_motor_velocity_timer_isr` | `RC_CODES` | Timer1 in CTC mode toggles the STEP bit in an interrupt. Uses direct port writes with the Uno's port mapping, so it targets pins 12 and 13 of an Uno. Prints a debug line each second. The meaning of "RC" in the original name is not documented. |

## Notes

- The original folder name says "arduinouno", but pin 26 does not exist on an Uno. The `simple` sketch was probably run on the Mega.
- This is the first stage that uses the final gantry pin assignment (X on 24/35, Y on 23/18 and 22/19). See [hardware.md](../../docs/hardware.md).
