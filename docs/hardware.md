# Hardware

## Bill of parts (as inferred from the code)

| Part | Notes |
|---|---|
| Arduino Mega 2560 | Mega pin numbers and `PORTA/PORTC/PORTE` registers are used. The ROS bridge logs it as "Arduino Mega" and opens `/dev/ttyACM*`. |
| TB6600 stepper drivers (x3) | The sketches name the TB6600 explicitly. DIR and STEP lines are driven directly. |
| Stepper motors (x3) | 200 steps/rev (1.8 deg), 8x microstepping, so 1600 steps per revolution. |
| Gantry | One motor drives the **X** axis. Two motors drive the **Y** axis together, wired to step in lockstep. |
| Rollers | Radius 30 mm (`r = 0.03`) until May 2026, then 30.5 mm (`r = 0.0305`) after calibration. |
| USB camera | Mounted above the table, 1280x720, MJPG, 30 fps. Read through V4L2 at index 2. |

## Pin assignments

From the end of August 2025 the X axis stays on pins 24/35. The Y wiring changed between 2025 and 2026.

| Axis | STEP | DIR | Used in |
|---|---|---|---|
| X | 24 (PA2) | 35 (PC2) | `motors_test_4x_microstepping`, trapezoid sketch (2025-09-07), all 2026 sketches |
| Y1 | 22 (PA0) | 37 (PC0) | same 2025 sketches |
| Y2 | 23 (PA1) | 36 (PC1) | same 2025 sketches |
| Y1 | 23 | 18 | 2026 sketches |
| Y2 | 22 | 19 | 2026 sketches |

The earliest 2025 timer sketches and the single-motor experiments used other pins (`2/4/5`, `7/6`, `10/11`, `12/13`, `26/35`). They are listed in each stage README. The first AccelStepper test also used an enable pin (`ENA`, pin 8).

## Unit conversions

With `r` the roller radius in metres, one revolution moves the belt or carriage `2*pi*r`.

```
steps_per_rev = 200 * microsteps = 1600
cm_per_step   = 2*pi*r*100 / steps_per_rev
```

| r | cm per step | steps per mm |
|---|---|---|
| 30.0 mm | 0.01178 | 8.49 |
| 30.5 mm | 0.01198 | 8.35 |

The speed-to-interval helper used from June 2026 converts a speed in cm/s into the time between two step pulses:

```
steps_per_second = speed_cm_s / cm_per_step
interval_us      = 1_000_000 / steps_per_second
```

## Coordinate frames

- **Camera pixels.** Origin at the image corner, 1280x720.
- **Table millimetres.** The calibration script plots a 800 x 1600 mm rectangle with the origin at one corner and the X axis reversed. Calibration points sit 33.5 mm inside the border.
- **Arduino centimetres.** The firmware works in cm. The ROS bridges divide the millimetre values by 10 before sending.
- **Start position.** From June 2026 the firmware assumes the striker starts at a fixed spot: (40.7, 8.8) cm, later (40.8, 9.2) cm in the 4-6 sketch. It has no limit switches, so the position is only valid if the striker was parked there at power-up.

## Wiring reference

The wiring for a single TB6600 and an Arduino was taken from a public tutorial (`PUL+`, `DIR+`, `ENA+` to digital pins, `PUL-`, `DIR-`, `ENA-` to GND, 9-40 V supply on the driver). The diagram is not redistributed here because it is third-party material. Search for "TB6600 Arduino wiring" on any maker site to find it.

Check the DIP switches on each driver. The sketches assume 8x microstepping, and a different setting scales every speed and distance.
