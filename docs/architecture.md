# Architecture

The system has three layers: perception on the host PC, a ROS 2 bridge, and motion control on the Arduino. This page describes each node and model, and the two communication designs the project went through.

## Perception (`yolo_pkg`)

### `main_launching_unit.py`

A ROS 2 node named `trajectory_prediction_node`. It runs a 30 Hz timer. On every tick:

1. Reads a frame from the camera (V4L2 index 2, 1280x720, MJPG, 30 fps).
2. Runs an HSV threshold for blue (`[100,100,50]` to `[130,255,255]`), takes the centroid of the mask and publishes it on `pixel_points`.
3. Runs `YOLO.track(...)` with `puck_v2`, `imgsz=320`, `conf=0.4`, `iou=0.5`, `max_det=1`, ByteTrack and `bytetrack_puck.yaml`.
4. Feeds the box centre (as `[y, x]` in pixels) to `PositionNet`, which returns `(x_mm, y_mm)`.
5. Computes velocity from the previous position and the elapsed time, then publishes `[frame, x, y, vx, vy]` on `present_points`.
6. Appends the state to a 20-step buffer. If the puck has moved more than 10 mm from where it was first seen, and the buffer is full, it runs the LSTM and publishes up to 30 predicted points on `pred_points`. Points outside `0 <= y <= 700` mm are dropped.
7. Draws the overlay and timing statistics in an OpenCV window. Pressing `q` shuts the node down.

### Models

| Model | Architecture | Input | Output | Files |
|---|---|---|---|---|
| Puck detector | YOLOv8n | 320 px frame | puck box | `yolo_for_train/runs/detect/puck_v2/weights/best.pt` (`best.onnx` too) |
| `PositionNet` | MLP 2-16-32-2, ReLU, MSE loss, Adam | scaled `(y_pix, x_pix)` | scaled `(x_mm, y_mm)` | `position_model/puck_mm_v1.pt`, `puck_X_scaler.pkl`, `puck_Y_scaler.pkl` |
| `TrajectoryPredictor` | 3 LSTM layers (128, 64, 64), batch norm, dropout 0.2, 8-head self-attention, layer norm, residual from the last input step | 20 x `(x, y, vx, vy)` | 30 x `(x, y, vx, vy)` | `20-30-300.pth`, `20-30-300_scaler.pkl` |

Training scripts:

- `yolo_for_train/yolo_train_for_puck.py` splits the CVAT export 80/20 and trains YOLOv8n.
- `position_model/train_to_get_mm.py` trains `PositionNet` from a CSV of calibration points (`x_yolo`, `y_yolo`, `x_mm`, `y_mm`).
- `full_trajectory_prediction/puck_trajectory_prediction.py` trains the LSTM on per-track CSVs (`frame, time_delta_ms, x_mm, y_mm`) with a weighted MSE loss, Adam at 1e-4, a plateau scheduler and early stopping. Its default paths point at Google Colab, so it was trained there.
- `full_trajectory_prediction/test_a_random_trajectory.py` plots predictions against ground truth for held-out tracks.

### Calibration pipeline

The pixel-to-mm network is trained from a physical calibration:

1. A marker is placed at known table positions (`point_taken_of_workspace_for_training_position_detection.csv`, in mm).
2. `collect_data_from_yolo.py` runs the detector on the photos of those positions and writes the pixel centres (`yolo_collected_data.csv`).
3. `combine_yoloPositions_with_collectedData.py` joins both files into `final_result.csv`.
4. `train_to_get_mm.py` fits `PositionNet` to that table.

## Bridge (`arduino_bridge`)

The package holds several bridge nodes. They all turn ROS 2 messages into bytes on a serial port.

| File | Role |
|---|---|
| `arduino_bridge/ROS_Arduino_bridge.py` | September 2025 design. Decides where to send the striker (see below). |
| `arduino_bridge/communication.py` | Generic relay: `ros2ard` (String) to serial, serial to `ard2ros`. Takes the port and baud rate as arguments. |
| `arduino_bridge/test_send.py` | Publishes `"1"` and `"0"` on `ros2ard` once per second. |
| `yolo_pkg/.../codes/bridge_versions/` | The 2026 bridges, one per stage. |

## Two communication designs

### Design A, September 2025: micro-ROS

The Arduino ran a micro-ROS node (`three_motors_trap_with_timers.ino`) over its USB serial port (`set_microros_serial_transports`), so a micro-ROS agent had to run on the host. The node had one subscriber, `go_to`, and one publisher, `motor_position`, both `Float32MultiArray`. The host-side `ROS_Arduino_bridge.py` decided what to send:

- If the puck is nearly stationary (`|v| < 5 mm/s`) and inside `0 <= y <= 700` mm, send its position.
- If a predicted point is reachable (`distance / 400 mm/s <= frame * 0.033 s`), send it. Only the first such point is sent.
- Once the motors report they have arrived and the puck is moving away (`vy > 0`), send home at `(400, 0)`.

The Arduino generated a trapezoidal velocity profile (max 0.40 m/s, max 4.2 m/s^2) using hardware timers.

### Design B, 2026: plain serial text

The 2026 firmware drops micro-ROS. The host writes a newline-terminated, comma-separated line to the serial port and the Arduino parses it with `readStringUntil('\n')`. Motion is done by software stepping in `loop()` with `micros()` timers and a proportional speed law. The line format changed with each stage, see the table in the [README](../README.md#data-flow-and-message-formats).

The Arduino prints its position every 100 ms. The bridges run a background thread that reads and discards that output. Without it the OS serial buffer fills in about 18 seconds and writes start to fail (a bug noted in the 2026-06-04 bridge).

## Firmware control law (2026)

```
error_cm  = target_cm - position_cm            # position comes from counting steps
speed     = clamp(|error| * GAIN, 0, MAX_SPEED)  # cm/s
interval  = 1e6 / (speed / cm_per_step)         # microseconds between pulses
if |error| > DEADZONE: step one pulse every `interval`, in the direction of the error
```

Position is tracked by counting the steps it has issued, which is deterministic. The 2026-09-16 sketch explains why it ignores the camera reading of its own position: camera jitter changed the direction of motion every frame.
