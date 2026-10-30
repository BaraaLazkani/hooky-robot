# 2025-08-27: timer-interrupt motor control

**Dates:** 2025-08-27 to 2025-08-29 (file modification times).
**Original folders:** `src/motor_control/` and `14-4/motor_control/motor_control/` (identical contents; the `14-4` copy is a later snapshot).

## What this stage did

The first firmware. Instead of pulsing a pin with `delayMicroseconds`, the sketches use Arduino Mega hardware timers in CTC mode. The timer interrupt toggles the STEP pin, so pulses keep coming while `loop()` reads serial commands. The ramps and the step frequency are calculated from a linear speed, using a 30 mm roller radius and 8x microstepping.

## Sketches

| Sketch | Date | What it does |
|---|---|---|
| `one_motor_control_with_timer` | 08-28 | One motor on Timer1. STEP pin 5, DIR pin 4. Send a speed in cm/s (positive is backward, negative forward, 0 stops). Prints the computed half-period `dt` and the OCR value. |
| `three_motors_control_using_timers` | 08-28 | X and Y with direct port manipulation. Send `v_x v_y target_x target_y` (cm/s and cm). Position limits stop each axis at its target. |
| `run_motors_using_timers` | 08-27 | Position control with acceleration. Send `Y(mm) X(mm)`. Timer1 drives both Y motors from one STEP pin, Timer4 drives X, and Timer5 runs a 1 kHz tick that ramps the pulse rate over 200 steps up to about 1000 mm/s. Uses 9600 baud. |
| `motors_test_4x_microstepping` | 08-29 | Velocity test with the `PA0/PA1/PA2` STEP and `PC0/PC1/PC2` DIR pins (see [hardware.md](../../docs/hardware.md)). Send `Y_velocity X_velocity` in cm/s. A code comment marks the timer constant as corrected from 4.0 to 2.0. |

## Notes

- The original `motors_test_go2pos_with_speed` is byte-identical to `motors_test_4x_microstepping` despite its name, so it is not duplicated here.
- The pin assignment differs between sketches. These were separate experiments, not a single wiring.
- Sketch names keep their original spelling, apart from the stray double underscore in `run_motors_using__timers`.
