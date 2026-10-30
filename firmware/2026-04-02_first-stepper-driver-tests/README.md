# 2026-04-02: first stepper driver tests

**Dates:** 2026-04-02 for `accelstepper_back_and_forth`. 2026-04-07 for the other two (file modification times).
**Original folders:** `sketch_apr2a`, `code_with_enable`, `code_without_accelstepper` (loose sketches next to the dated folders).

## What changed

A restart with the simplest possible code, to check the TB6600 driver and the motor before building anything larger. The timer-interrupt work from 2025 is not used here.

## Sketches

| Sketch | What it does |
|---|---|
| `accelstepper_back_and_forth` | AccelStepper in `DRIVER` mode on STEP 24 and DIR 35. Max speed 1000, acceleration 500. Moves to +1000 steps, waits one second, then moves to -1000. The `#define STEP_PIN 26` at the top is unused. |
| `accelstepper_with_enable_pin` | The same idea on STEP 5, DIR 4 and an ENA pin 8. Max speed 2000, acceleration 500. The comment says LOW enables most TB6600 boards, so test it. It calls `moveTo(1000)` and `moveTo(400)` in every loop, so the target is effectively always 400. It reads as unfinished. |
| `plain_pulse_stepping` | No library. The Makerguides example: STEP 5, DIR 4, 500 microseconds high and 500 low, one direction forever (1000 steps/s). |

## Requirements

The **AccelStepper** library for the first two sketches.
