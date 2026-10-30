# Firmware

Arduino Mega 2560 sketches, one folder per stage of development. Each folder has its own `README.md` that explains what changed from the previous stage and what each sketch does.

Every sketch lives in `<stage>/<sketch>/<sketch>.ino`, so the Arduino IDE opens it directly.

| Stage | Control idea | Host interface |
|---|---|---|
| [2025-08-27](2025-08-27_timer-interrupt-motor-control/) | Hardware timer interrupts generate step pulses | Serial text, velocity or position |
| [2025-09-07](2025-09-07_trapezoidal-profile-micro-ros/) | Trapezoidal velocity profile | micro-ROS topics |
| [2026-04-02](2026-04-02_first-stepper-driver-tests/) | First tests with a TB6600 driver | none |
| [2026-04-14](2026-04-14_velocity-control/) | Constant velocity, blocking pulses | Serial `velocity` |
| [2026-05-19](2026-05-19_position-control/) | Go to a position by counting steps | none (target in code) |
| [2026-06-01](2026-06-01_serial-target-and-speed/) | Non-blocking stepping, proportional speed | Serial `x,y` |
| [2026-06-04](2026-06-04_proportional-follow-with-limits/) | High-gain follow with Y limit and home | Serial `x,y` |
| [2026-06-08](2026-06-08_pixel-feedback-from-camera/) | Position taken from camera pixels | Serial 4 fields |
| [2026-09-16](2026-09-16_ball-chasing/) | Step-counted position, arrival flags, jitter filter | Serial 4 fields |

Pin numbers, unit conversions and wiring notes are in [../docs/hardware.md](../docs/hardware.md).
