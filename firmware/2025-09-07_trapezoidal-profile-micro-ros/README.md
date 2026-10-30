# 2025-09-07: trapezoidal profile with micro-ROS

**Date:** 2025-09-07.
**Original folders:** `src/motor_control/three_motors_trap_with_timers/` and the same sketch in `14-4/motor_control/`.

## What changed

The sketch moves the Arduino into the ROS 2 graph. It is a micro-ROS client that talks over the USB serial port, so a micro-ROS agent must run on the host. The Arduino now decides *how* to move (a trapezoidal velocity profile). The host decides *where*.

## Sketch: `three_motors_trap_with_timers`

- Uses the `PA0/PA1/PA2` STEP and `PC0/PC1/PC2` DIR pins.
- Subscribes to `go_to` (`std_msgs/Float32MultiArray`, the target position) and publishes `motor_position`.
- Builds a trapezoidal velocity profile per motor: max speed 0.40 m/s, max acceleration 4.2 m/s^2, minimum step frequency 60 Hz. For short moves where the full speed cannot be reached, it computes a peak speed and uses a triangular profile.
- Hardware timers produce the pulses.
- Waits at start-up for the micro-ROS agent (`rmw_uros_ping_agent`) and prints "Ready - Waiting for ROS commands".

## Host side

`ros2_ws/src/arduino_bridge/arduino_bridge/ROS_Arduino_bridge.py` (also copied into `yolo_pkg/.../codes/`) subscribes to `present_points`, `pred_points` and `motor_position`, and publishes `go_to`. See [architecture.md](../../docs/architecture.md#design-a-september-2025-micro-ros) for its rules.

## Requirements

- The **micro_ros_arduino** library.
- A micro-ROS agent connected to the Mega's serial port.

## Notes

This design was replaced in 2026 by plain serial text lines. The 2026 sketches do not need micro-ROS.
