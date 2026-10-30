# Known issues and things to check

These were found while documenting the project. The code was restructured but **not** changed, so everything below is still true of the files in the repo. The history is kept as it was, and this list says what to fix before relying on any part.

## Build and launch

- **ROS 2 entry points are broken.** `yolo_pkg/setup.py` registers `yolo_pkg.import_cv2_main_unit:main` and `yolo_pkg.ROS_Arduino_bridge:main`. Neither module exists in the package. The scripts live in `yolo_pkg/codes/`, which is not a Python package (no `__init__.py`). `arduino_bridge/setup.py` has an empty `console_scripts` list.
- **The launch file cannot work as written.** `launch/launch_unit.py` passes filesystem paths (`/home/jojo/htr/...`) as the `package=` argument, and refers to the executable `import_cv2_main_unit`.
- **Run the live node from `codes/`.** `main_launching_unit.py` loads `bytetrack_puck.yaml` by relative name.
- **Package metadata.** The maintainer is `jojo` with placeholder e-mail addresses. License and description fields were filled in when this repo was assembled. The maintainer fields were left alone.

## Hard-coded paths and devices

| File | Hard-coded value |
|---|---|
| `main_launching_unit.py` | model paths under `/home/humam/hooky/src/yolo_pkg/yolo_pkg/codes/` |
| `collect_trajectory_dat_points.py` | the same `/home/humam/hooky/...` model paths, and a CSV output folder under `/home/jojo/htr/...` |
| `position_model/train_to_get_mm.py` | `save_dir = /home/jojo/htr/.../position_model` |
| `full_trajectory_prediction/test_a_random_trajectory.py` | `/home/jojo/htr/...` |
| `full_trajectory_prediction/puck_trajectory_prediction.py` | Google Colab / Drive paths |
| `yolo_for_train/old/*` | `/home/jaafar/htr/srs/...` |
| Camera | `/dev/video2` (index 2, V4L2) |
| Arduino port | `/dev/ttyACM0` in some bridges, `/dev/ttyACM1` in others |

Replace these with your own paths. A future cleanup should move them into ROS parameters or a config file. The 2025-09-07 bridge already uses ROS parameters for the port.

## Code behaviour worth knowing

- **Prediction overlay draws one point.** In `main_launching_unit.py`, the `cv2.circle` / `cv2.line` calls sit after the `for i in range(PREDICTION_STEPS)` loop at the same indentation as the loop, not inside it. Only the last predicted point is drawn. Publishing to `pred_points` is not affected.
- **`main()` references `mask` after `rclpy.spin`.** `cv2.imshow("Blue Mask", mask)` runs after the spin returns, where `mask` is not defined. It would raise `NameError` on shutdown.
- **Two copies of `ROS_Arduino_bridge.py` differ.** `arduino_bridge/arduino_bridge/` uses `/dev/ttyACM0` and 400 mm/s. `yolo_pkg/.../codes/` uses `/dev/ttyACM1` and 500 mm/s. The comment in both says 500 mm/s.
- **Calibration constants are inline.** The bridges divide pixel coordinates by `9.8` and `10.23` to get centimetres. These are camera-specific.
- **Possibly swapped names in the 2026-09-16 bridge.** It stores `present_points` (the YOLO puck position) as `robot_x_cm/robot_y_cm` and `pixel_points` (the blue marker) as the ball. In the training images the green disc is the puck and the blue disc is the striker. The firmware uses the last two fields as its target, so check which object you want the striker to follow.
- **`4-6_ros_to_arduino.py` is mis-dated.** It was last modified on 2026-09-16 and its four-field protocol belongs with the 2026-06-08 sketch. It is kept under its original date, as `2026-06-04_ros-to-arduino-bridge-pixel-feedback.py`.
- **Stale comments in `proportional_follow_with_limits`.** Messages say "Goal Y > 80" and "clamped to 35 cm", but the constants are 130 and 30.
- **`accelstepper_with_enable_pin` looks unfinished.** It sets `moveTo(1000)` and `moveTo(400)` back to back in every loop, so the target is always 400.
- **`single_motor_velocity_timer_isr` targets an Uno.** Its port mapping (`PORTB` bit 4 = pin 12) is the Uno's. On a Mega pins 12 and 13 are on different bits.

## Safety and calibration

- No limit switches or homing. The firmware assumes the striker is parked at a hard-coded start position at power-up. A wrong start position means the controller moves the carriage past the physical end of the rail.
- The Y-axis limit and the "home trigger" are only in the 2026-06-04 sketch. The other sketches have no travel limits.
- Speeds, gains and dead zones were tuned by hand for one machine.

## Models and licences

- The scalers (`.pkl`) and some model files are pickled. Only load files from sources you trust. They also depend on the scikit-learn and PyTorch versions used to create them, which were not recorded.
- Recent PyTorch versions default `torch.load` to `weights_only=True`. If loading fails, check that before anything else.
- Ultralytics YOLO is AGPL-3.0, which affects redistribution of the trained detector.

## Not in the repository

Build and install output, `log/`, the `camenv` virtualenv, zip archives (duplicates of files that are tracked), a 41 MB screen recording from 2025-09-06, a third-party wiring diagram, and personal photos.
