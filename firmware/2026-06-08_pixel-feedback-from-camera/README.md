# 2026-06-08: pixel feedback from the camera

**Date:** 2026-06-08.
**Original folders:** `8-6/hooky.zip/hooky.zip.ino`. The same sketch is in `16-9/hockey_robot`, which is a byte-for-byte copy and is not duplicated here.
**Host side:** [`2026-06-08_blue-marker-detection.py`](../../ros2_ws/src/yolo_pkg/yolo_pkg/codes/colour_marker/2026-06-08_blue-marker-detection.py), the pixel publisher in [`main_launching_unit.py`](../../ros2_ws/src/yolo_pkg/yolo_pkg/codes/main_launching_unit.py), and the pixel-feedback bridge listed in the 2026-06-04 stage.

## What changed

The camera now also reports where the striker is. A new HSV blue-colour detector finds the blue disc, and the sketch takes its **current position straight from those pixel coordinates** instead of counting steps.

- The sketch reads four fields per line: `centre_x,centre_y,pixel_x,pixel_y`.
- `target_x/target_y` come from the first two fields. `x_pos/y_pos` come from the last two (the camera reading).
- The status line prints all six values (`CenterX`, `CenterY`, `PixelX`, `PixelY`, `x_pos`, `y_pos`) for debugging.

## Sketch: `pixel_feedback_follow`

Same proportional controller and limits as the 2026-06-04 sketch (gains 100 and 3, caps 100 and 60 cm/s, Y limit 30, home trigger 130). The Y limit and home trigger are no longer applied, since the new serial parser does not call them.

## Host-side pieces

- `colour_marker/2026-06-08_blue-marker-detection.py` is a standalone viewer. It opens camera index 2, thresholds blue in HSV (`[100,100,50]` to `[130,255,255]`), and shows the mask and the centroid.
- `main_launching_unit.py` runs the same threshold in the live node and publishes the centroid on `pixel_points`.

## Notes

This approach was reversed three months later (2026-09-16). The comment in that sketch explains why: the camera's reading of the striker jitters from frame to frame, and driving the controller with it made the direction flip erratically.
