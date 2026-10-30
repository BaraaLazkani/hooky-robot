# 2026-06-01: serial targets and speed control

**Date:** 2026-06-01 (modification times). Two of the three original folders were last touched later (the "final one" on 06-04, the "perfect code" on 09-16), so their exact day is uncertain.
**Original folders:** `1-6/position_speed_first_code`, `1-6/the final one/the_speed_code`, `1-6/the_perfect_code_taking_me_to_a_specifc_location_by_camera`.

## What changed

This stage replaced the blocking pulse loop with **non-blocking stepping**: each axis has its own `micros()` timer and step interval, so X and Y move at independent speeds. The target now arrives over serial as `x,y\n` in centimetres, which lets the camera drive the robot. The ROS bridge for this protocol is [`2026-06-01_ros-to-arduino-bridge-basic.py`](../../ros2_ws/src/yolo_pkg/yolo_pkg/codes/bridge_versions/2026-06-01_ros-to-arduino-bridge-basic.py).

## Sketches

| Sketch | Original folder | What it does |
|---|---|---|
| `xy_position_with_speed` | `position_speed_first_code` | Target (10, 5) cm and speeds (5, 2) cm/s fixed in code. Steps are issued when the per-axis interval has elapsed, with a 2 microsecond pulse. Prints the position every 100 ms. |
| `proportional_speed_control` | `the final one/the_speed_code` | Starts at (40.7, 8.8) cm. Reads `x,y` from serial. Speed is proportional to the error (gain 3.0 for X, 1.0 for Y), capped at 30 and 20 cm/s, with a 1 cm dead zone. The X direction is flipped to match the machine. |
| `serial_goto_xy_with_arrival` | `the_perfect_code_taking_me_to_a_specifc_location_by_camera` | Reads `x,y` from serial and moves at a fixed 5 cm/s. Uses "arrived" flags per axis and a 5-step tolerance. Prints "New Target" and "X arrived" / "Y arrived". |

## Serial protocol

```
x_cm,y_cm\n        e.g. "30.50,12.00\n"
```

115200 baud. The Arduino replies with a status line every 100 ms.

## Notes

- The original folder names ("the final one", "the perfect code") were not final. The next stage retuned and extended `proportional_speed_control`.
- A screenshot saved in `1-6/` (not included) shows an editor with `x_speed_cm_s = 40.0` and `y_speed_cm_s = 25.0`, which match the speed caps of the 2026-09-16 sketches (40 and 25 cm/s).
- The start position is hard-coded, and the sketch assumes the striker is parked there.
