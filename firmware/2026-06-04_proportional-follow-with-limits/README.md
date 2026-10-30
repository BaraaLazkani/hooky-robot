# 2026-06-04: proportional follow with limits

**Date:** 2026-06-04.
**Original folder:** `4-6/motor_control`.
**Host side:** [`2026-06-04_ros-to-arduino-bridge-intercept.py`](../../ros2_ws/src/yolo_pkg/yolo_pkg/codes/bridge_versions/2026-06-04_ros-to-arduino-bridge-intercept.py) (original `final-4-6_ros_to_arduino.py`) and [`2026-06-04_ros-to-arduino-bridge-pixel-feedback.py`](../../ros2_ws/src/yolo_pkg/yolo_pkg/codes/bridge_versions/2026-06-04_ros-to-arduino-bridge-pixel-feedback.py) (original `4-6_ros_to_arduino.py`).

## What changed

The same proportional controller as the previous stage, tuned for a moving target:

- Much higher gains: X gain 100, Y gain 3. Speed caps raised to 100 cm/s (X) and 60 cm/s (Y).
- **Y limit.** The requested Y is clamped to 30 cm so the striker never leaves its half of the table.
- **Home trigger.** If the requested Y exceeds 130 cm (the puck is far away), the target is reset to the start position (40.8, 9.2) cm.
- Dead zone of 1 cm on both axes.

## Sketch: `proportional_follow_with_limits`

Serial input is `x,y\n` in centimetres, like the previous stage. The messages in the code ("Goal Y > 80", "clamped to 35 cm") are out of date, because the constants are 130 and 30.

## Host-side bridges

| Script | What it does |
|---|---|
| `...-bridge-intercept.py` | Subscribes to `pred_points` and `present_points`. Takes the predicted point closest to `DEFENSE_Y_CM = 30` and sends its X with the Y fixed to the defence line. Falls back to chasing the current puck position if the prediction is older than 0.3 s. Rate limit 25 Hz. A background thread drains the Arduino's output so the serial buffer never fills. |
| `...-bridge-pixel-feedback.py` | Also subscribes to `pixel_points`. Sends four fields `x_cm,y_cm,pixel_y,pixel_x`, with pixels divided by the calibration constants 9.8 and 10.23. Rate limit 0.3 s. This protocol belongs with the **2026-06-08** sketch. The file was last modified on 2026-09-16 but keeps its `4-6` label. |

## Notes

The intercept bridge and this sketch use the same two-field protocol, so they are a pair.
