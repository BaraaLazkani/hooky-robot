# `codes/`: vision, prediction and bridge scripts

Everything on the host side that is not a ROS package definition. The folder layout is the one the code expects, so only the absolute path prefix at the top of a few scripts needs changing. See [docs/known-issues.md](../../../../../docs/known-issues.md).

Dates are file modification times. Run scripts from this directory, since the tracker config is loaded by relative name.

## Live pipeline

| File | What it does |
|---|---|
| `main_launching_unit.py` | The ROS 2 node that runs the whole perception chain: camera, blue-marker tracker, YOLO + ByteTrack, `PositionNet`, LSTM. Publishes `present_points`, `pred_points`, `pixel_points`. See [architecture.md](../../../../../docs/architecture.md). |
| `ROS_Arduino_bridge.py` | Copy of the September 2025 micro-ROS-era bridge. The package copy is in `arduino_bridge/`. |
| `bytetrack_puck.yaml`, `custom_bytetrack.yaml` | ByteTrack configs (identical). Ultralytics defaults with a 60-frame track buffer, 2025-08-11. |
| `20-30-300.pth`, `20-30-300_scaler.pkl` | The LSTM weights and scaler that the live node loads (2025-09-02). |

## `bridge_versions/`: ROS-to-Arduino bridges, one per stage

| File | Stage | Original name | Protocol |
|---|---|---|---|
| `2026-06-01_ros-to-arduino-bridge-basic.py` | [2026-06-01](../../../../../firmware/2026-06-01_serial-target-and-speed/) | `1-6_ros_to_arduino.py` | `x_cm,y_cm` for every `present_points` message |
| `2026-06-04_ros-to-arduino-bridge-intercept.py` | [2026-06-04](../../../../../firmware/2026-06-04_proportional-follow-with-limits/) | `final-4-6_ros_to_arduino.py` | `x_cm,y_cm`, X from the predicted interception point, 25 Hz |
| `2026-06-04_ros-to-arduino-bridge-pixel-feedback.py` | 2026-06-04, but belongs with [2026-06-08](../../../../../firmware/2026-06-08_pixel-feedback-from-camera/) | `4-6_ros_to_arduino.py` | `x_cm,y_cm,pixel_y,pixel_x` |
| `2026-09-16_ros-to-arduino-bridge-timer-based.py` | [2026-09-16](../../../../../firmware/2026-09-16_ball-chasing/) | `16-9_ros_to_arduino.py` | `robot_x,robot_y,pixel_x,pixel_y` on a 0.3 s timer |

## `colour_marker/`

| File | What it does |
|---|---|
| `2026-06-08_blue-marker-detection.py` | Standalone viewer for the HSV blue-marker tracker (original `8-6blue_color.py`). |

## `tools/`

| File | What it does |
|---|---|
| `camera_read_frames.py` | Opens camera index 1, shows the live frame and saves a JPEG each time you press `s` (2026-05-25). |
| `../images_source/` | `change_brightness.py` and `split_images_from_video_and_save_as_JPEG.py` for building the image set, with the 136 raw and 136 augmented frames. |

## `collect_trajectory_dat_points.py` (2026-05-25)

Live data-collection tool. Opens camera index 2, runs the detector and the position network, and shows the annotated frame. Press `s` to start and stop a recording, which saves a video and a per-frame CSV of `(x_mm, y_mm)` (the tracks the LSTM trains on). Press `q` to quit. It uses `puck_best.pt`, the earlier position model.

## `data_collecting/`: pixel-to-mm calibration (2025-07-15)

| File | What it does |
|---|---|
| `collect_data_from_yolo.py` | Runs YOLO over a folder of numbered JPEGs and writes the pixel centres to CSV. |
| `combine_yoloPositions_with_collectedData.py` | Joins the physical positions with the YOLO pixels into the training CSV. |
| `plot_point_taken_of_workspace_for_training_position_detection.py` | Plots the calibration points over the 800 x 1600 mm table. |
| `csv_files/` | The 136 calibration points: `point_taken_of_workspace...csv` (mm), `yolo_collected_data.csv` (pixels), `final_result.csv` (joined). |

## `position_model/`: pixel to millimetres (2025-09-04, retrained 2026-05-25)

| Path | What it is |
|---|---|
| `train_to_get_mm.py` | Trains the 2-16-32-2 MLP, MSE loss, Adam. |
| `puck_mm_v1.pt`, `puck_X_scaler.pkl`, `puck_Y_scaler.pkl` | Model and scalers used by the live node (the `.pt` is the 2026-05-25 retrain). |
| `puck.pt`, `puck_best.pt` | The earlier models from 2025-09-04. |
| `puck_training_history.csv` | Train and validation loss per epoch. |
| `puck/`, `puck_mm_v1/`, `old/` | The same weights unpacked, and older copies. |

## `full_trajectory_prediction/`: the LSTM (2025-08-25 to 2025-09-06)

| Path | What it is |
|---|---|
| `puck_trajectory_prediction.py` | Training script (Colab paths). 20 input steps, 30 predicted steps, up to 300 epochs. |
| `test_a_random_trajectory.py` | Compares predictions with ground truth for held-out tracks. |
| `20-20-200.*`, `20-30-300.*` | Two model generations and their scalers. The names appear to encode input steps, prediction steps and epochs. |
| `real-time_frame_positions_in_mm/new/` | 304 recorded tracks as CSV (`frame, time_delta_ms, x_mm, y_mm`). `testing_points/` holds 26 held-out ones. |
| `plots/` | Training curves and prediction-versus-truth plots for six model variants, on the 302-track dataset. |

## `yolo_for_train/`: the puck detector (2025-07-16 to 2025-09-06)

| Path | What it is |
|---|---|
| `obj_train_data/` | 136 labelled frames exported from CVAT in YOLO format. |
| `split_data/` | The same data split 80/20 into `train/` and `val/`. |
| `yolo_train_for_puck.py` | Splits and trains YOLOv8n. |
| `runs/detect/puck_v1`, `puck_v1.1`, `puck_v2` | Training runs. `puck_v2` is the one in use: 150 epochs, batch 8, 320 px, CPU. It includes per-10-epoch checkpoints, an ONNX export, curves and sample predictions. |
| `puck_detection_final.pt` | The final export of the detector (a different file from `puck_v2/weights/best.pt`). |
| `old/` | The first dataset layout, scripts and runs. |
