# 2026-09-16: ball chasing

**Date:** 2026-09-16 (modification times; the second sketch is the later one, saved about 15 minutes after the first).
**Original folders:** `16-9/hockey_robot_final` and `16-9/hockey_robot_final(1)`. The third folder, `16-9/hockey_robot`, is identical to the 2026-06-08 sketch.
**Host side:** [`2026-09-16_ros-to-arduino-bridge-timer-based.py`](../../ros2_ws/src/yolo_pkg/yolo_pkg/codes/bridge_versions/2026-09-16_ros-to-arduino-bridge-timer-based.py) (original `16-9_ros_to_arduino.py`).

## What changed

The latest stage. It undoes the camera-as-position idea from 06-08 and keeps the lessons:

- **Position is tracked internally** by counting the steps the sketch has issued. It is noise-free, so it can drive the controller. The robot fields in the serial line are ignored.
- **The target comes from the camera.** Only the last two of the four fields (`ball_x`, `ball_y`) are used.
- **Arrival flags.** Each axis stops when it reaches the target and starts again only when a new target arrives.
- **Gentler tuning.** Gains 6.0 on both axes, caps 40 cm/s (X) and 25 cm/s (Y), 0.3 cm dead zone. Start position (40.7, 8.8) cm.

## Sketches

| Sketch | What it does |
|---|---|
| `ball_chasing_arrival_flags` | Reads `robotX,robotY,ballX,ballY`, uses the ball fields as the target, and moves with proportional speed until within 0.3 cm. Prints "New Target", "X arrived" and "Y arrived". |
| `ball_chasing_jitter_filter` | The same sketch plus a change threshold of 1 cm per axis. A new target is accepted only if it moved by more than that. This stops camera noise on a stationary ball from re-arming the motors and causing endless tiny corrections. |

## Host-side bridge

Earlier bridges sent a line when a message arrived and the robot had moved. This one **sends on a fixed timer** (every 0.3 s) with the latest values, so the Arduino always gets fresh coordinates. It writes `robot_x_cm,robot_y_cm,pixel_x,pixel_y`, with the pixel values divided by 9.8 and 10.23, and waits until it has received at least one `present_points` message. It uses `/dev/ttyACM1`.

## Notes

- The robot and ball names in the bridge may be swapped relative to what the models track. See [known-issues.md](../../docs/known-issues.md).
- This is the most recent code in the repository.
