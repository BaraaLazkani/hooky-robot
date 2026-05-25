import cv2, torch, joblib
import numpy as np
from tkinter import Tk, filedialog
from ultralytics import YOLO
from datetime import datetime
import os
from pathlib import Path
import time
from threading import Thread
from queue import Queue, Empty
from collections import deque

# Verify CUDA setup
device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
print(f"PyTorch device: {device}")
if device.type == "cuda":
    print(f"GPU: {torch.cuda.get_device_name(0)}")
    print(f"CUDA version: {torch.version.cuda}")

# --- Hide tkinter window ---
root = Tk()
root.withdraw()

# --- Paths ---
yolo_model_path = '/home/humam/hooky/src/yolo_pkg/yolo_pkg/codes/yolo_for_train/runs/detect/puck_v2/weights/best.pt'
position_model_path = '/home/humam/hooky/src/yolo_pkg/yolo_pkg/codes/position_model/puck_best.pt'
x_scaler_path = '/home/humam/hooky/src/yolo_pkg/yolo_pkg/codes/position_model/puck_X_scaler.pkl'
y_scaler_path = '/home/humam/hooky/src/yolo_pkg/yolo_pkg/codes/position_model/puck_Y_scaler.pkl'

X_scaler, Y_scaler = joblib.load(x_scaler_path), joblib.load(y_scaler_path)

# --- Load YOLO model ---
yolo = YOLO(yolo_model_path)
yolo.to(device)
print(f"YOLO device: {yolo.device}")

# --- Warm up GPU ---
print("Warming up GPU...")
dummy_frame = np.zeros((720, 1280, 3), dtype=np.uint8)
for _ in range(5):
    yolo.track(dummy_frame, imgsz=640, conf=0.25, iou=0.6, augment=False, device=device, tracker='bytetrack_puck.yaml', persist=True, max_det=1)
print("GPU warm-up complete.")

# --- Define position prediction network ---
class PositionNet(torch.nn.Module):
    def __init__(self):
        super(PositionNet, self).__init__()
        self.layers = torch.nn.Sequential(
            torch.nn.Linear(2, 16),
            torch.nn.ReLU(),
            torch.nn.Linear(16, 32),
            torch.nn.ReLU(),
            torch.nn.Linear(32, 2)
        )
    def forward(self, x):
        return self.layers(x)

# --- Load position model ---
position_model = PositionNet()
position_model.load_state_dict(torch.load(position_model_path, map_location=device))
position_model.to(device).eval()

# --- Initialize webcam ---
cap = cv2.VideoCapture(2, cv2.CAP_V4L2)
cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
cap.set(cv2.CAP_PROP_FPS, 30)

# Verify webcam settings
width = cap.get(cv2.CAP_PROP_FRAME_WIDTH)
height = cap.get(cv2.CAP_PROP_FRAME_HEIGHT)
fps = cap.get(cv2.CAP_PROP_FPS)
fourcc = cap.get(cv2.CAP_PROP_FOURCC)
print(f"Webcam: {width}x{height} at {fps} fps, FOURCC: {fourcc}")

if not cap.isOpened():
    print("Error: Could not open webcam.")
    exit()

# --- Video recording setup ---
recording = False
video_writer = None
video_output_dir = '/home/jojo/htr/src/yolo_pkg/yolo_pkg/codes/full_trajectory_prediction/real-time_frame_positions_in_mm'
csv_output_dir = '/home/jojo/htr/src/yolo_pkg/yolo_pkg/codes/full_trajectory_prediction/real-time_frame_positions_in_mm'
video_counter = 1
csv_log_path = None

# --- Frame capture thread setup ---
frame_queue = Queue(maxsize=10)
stop_capture = False

def capture_frames():
    global stop_capture
    last_time = None
    while not stop_capture:
        ret, frame = cap.read()
        current_time = time.time()
        time_delta_ms = 0.0 if last_time is None else (current_time - last_time) * 1000
        last_time = current_time
        if not ret:
            frame_queue.put((False, None, None))
            break
        frame_queue.put((ret, frame, time_delta_ms))

capture_thread = Thread(target=capture_frames, daemon=True)
capture_thread.start()

# --- Find the next available video number ---
def get_next_video_number():
    global video_counter
    while os.path.exists(os.path.join(video_output_dir, f"{video_counter}.avi")):
        video_counter += 1
    return video_counter

# --- CSV Log File Setup ---
frame_counter = 0
# Time accumulator for frames without detection
time_accumulator = 0.0

# --- Smoothing buffers ---
smooth_window = 1
x_smooth, y_smooth = deque(maxlen=smooth_window), deque(maxlen=smooth_window)

print("🎥 Live stream started (Rapoo C200 720p). Press 'p' to capture photo, 's' to toggle video recording, 'q' to quit.")

cv2.namedWindow("Live YOLO + Position Detection", cv2.WINDOW_NORMAL)
cv2.resizeWindow("Live YOLO + Position Detection", 1280, 720)

log_buffer = []

while True:
    try:
        ret, frame, time_delta_ms = frame_queue.get(timeout=0.1)
    except Empty:
        continue
    if not ret:
        print("Failed to grab frame.")
        break

    # Add time to accumulator if we're not detecting an object
    if recording:
        time_accumulator += time_delta_ms

    log_entry = [frame_counter, time_delta_ms, None, None]

    # --- Optimized ByteTrack tracking ---
    results = yolo.track(
        frame,
        imgsz=640,
        conf=0.4,
        iou=0.5,
        augment=False,
        device=device,
        tracker='bytetrack_puck.yaml',
        persist=True,
        max_det=1
    )
    boxes = results[0].boxes

    if boxes is not None and boxes.conf is not None and len(boxes) > 0:
        best_idx = boxes.conf.argmax().item()
        best_box = boxes.xywh[best_idx].tolist()
        x_center, y_center, w, h = best_box

        # Position prediction
        input_array = np.array([[y_center, x_center]])
        x_input_scaled = X_scaler.transform(input_array)
        x_tensor = torch.tensor(x_input_scaled, dtype=torch.float32).to(device)
        y_pred_scaled = position_model(x_tensor).detach().cpu().numpy()
        x_mm, y_mm = Y_scaler.inverse_transform(y_pred_scaled)[0]

        # --- Apply smoothing ---
        x_smooth.append(x_mm)
        y_smooth.append(y_mm)
        x_mm_s = np.mean(x_smooth)
        y_mm_s = np.mean(y_smooth)

        # If we have accumulated time from previous frames without detection
        if recording and time_accumulator > 0:
            log_entry[1] = time_accumulator  # Use accumulated time
            time_accumulator = 0.0  # Reset accumulator
        
        log_entry[2] = x_mm_s
        log_entry[3] = y_mm_s

        annotated_frame = frame.copy()
        left = int(x_center - w / 2)
        top = int(y_center - h / 2)
        right = int(x_center + w / 2)
        bottom = int(y_center + h / 2)
        cv2.rectangle(annotated_frame, (left, top), (right, bottom), (0, 255, 0), 2)
        text = f"x = {x_mm_s:.1f} mm, y = {y_mm_s:.1f} mm"
        cv2.putText(annotated_frame, text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
        
        # Only add to log buffer if we're recording and have a detection
        if recording:
            log_buffer.append(log_entry)
    else:
        annotated_frame = frame
        cv2.putText(annotated_frame, "No object tracked", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
        
        # If we're recording but not detecting, don't add to log yet
        # We'll accumulate the time and add it to the next detection

    frame_counter += 1

    key = cv2.waitKey(1) & 0xFF

    if key == ord('s'):
        if not recording:
            video_number = get_next_video_number()
            video_path = os.path.join(video_output_dir, f"{video_number}.avi")
            csv_log_path = os.path.join(csv_output_dir, f"{video_number}.csv")
            with open(csv_log_path, 'w') as f:
                f.write("frame,time_delta_ms,x_mm,y_mm\n")

            video_writer = cv2.VideoWriter(
                video_path,
                cv2.VideoWriter_fourcc(*'MJPG'),
                30.0,
                (int(width), int(height))
            )
            recording = True
            # Reset time accumulator when starting recording
            time_accumulator = 0.0
            print(f"Started recording: {video_path} with CSV log: {csv_log_path}")
            log_buffer = []
            frame_counter = 0
        else:
            video_writer.release()
            recording = False
            print(f"Stopped recording, saved: {video_path}")
            if log_buffer:
                with open(csv_log_path, 'a') as f:
                    for row in log_buffer:
                        x_mm_str = f"{row[2]:.3f}" if row[2] is not None else ""
                        y_mm_str = f"{row[3]:.3f}" if row[3] is not None else ""
                        f.write(f"{row[0]},{row[1]:.3f},{x_mm_str},{y_mm_str}\n")
                print(f"🧠 Position log saved to: {csv_log_path} with {len(log_buffer)} entries")
            video_writer = None
            video_counter += 1

    if recording and video_writer is not None:
        video_writer.write(annotated_frame)

    cv2.imshow("Live YOLO + Position Detection", annotated_frame)

    if key == ord('q'):
        print("Exiting program...")
        break

if recording and log_buffer:
    with open(csv_log_path, 'a') as f:
        for row in log_buffer:
            x_mm_str = f"{row[2]:.3f}" if row[2] is not None else ""
            y_mm_str = f"{row[3]:.3f}" if row[3] is not None else ""
            f.write(f"{row[0]},{row[1]:.3f},{x_mm_str},{y_mm_str}\n")
    print(f"🧠 Position log saved to: {csv_log_path} with {len(log_buffer)} entries")

print("Cleaning up resources...")
stop_capture = True
