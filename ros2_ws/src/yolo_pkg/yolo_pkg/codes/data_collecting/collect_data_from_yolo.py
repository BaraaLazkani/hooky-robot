import os
import re
import cv2
import pandas as pd
from tkinter import Tk, filedialog, simpledialog
from ultralytics import YOLO

def numerical_sort(file_list):
    """Sort filenames by first number found (e.g., img1.jpg, img2.jpg, ...)."""
    return sorted(file_list, key=lambda f: int(re.search(r'\d+', f).group()))

# --- Hide root tkinter window ---
root = Tk()
root.withdraw()

# --- Folder selection for images ---
# the file name should be (new images after some noise)
folder_path = filedialog.askdirectory(title="Select Folder Containing .jpg Images")
if not folder_path:
    raise FileNotFoundError("❌ No folder selected.")

# --- YOLO model selection ---
# after training YOLO on the taken images we save it on
# this path (/home/jaafar/htr/srs/yolo_pkg/yolo_pkg/codes/yolo_for_train/runs/detect/puck_v12/weights)
# so here we select the best model.
model_path = filedialog.askopenfilename(
    title="Select YOLO .pt Model",
    filetypes=[("YOLO model", "*.pt")]
)
if not model_path:
    raise FileNotFoundError("❌ No YOLO model selected.")

# --- Load YOLO model ---
model = YOLO(model_path)

# --- Gather .jpg files sorted numerically ---
image_files = [f for f in os.listdir(folder_path) if f.lower().endswith(".jpg")]
image_files = numerical_sort(image_files)

# --- Store detections ---
detections = []

# --- Process each image ---
for img_name in image_files:
    img_path = os.path.join(folder_path, img_name)
    image = cv2.imread(img_path)

    if image is None:
        print(f"⚠️ Could not read image: {img_name}")
        continue

    # YOLO inference
    results = model(image)
    annotated = results[0].plot()

    # Show image with detections
    cv2.imshow("YOLO Detection", annotated)

    # Select only the most confident detection
    boxes = results[0].boxes
    if boxes is not None and boxes.conf is not None and len(boxes) > 0:
        best_idx = boxes.conf.argmax().item()
        best_box = boxes.xywh[best_idx].tolist()
        x_yolo, y_yolo, _, _ = best_box

        # Save only the best detection
        detections.append({
            "image": img_name,
            "x_yolo": x_yolo,
            "y_yolo": y_yolo
        })

    # Wait until user presses 'q' to continue
    while True:
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

cv2.destroyAllWindows()

# --- Ask for output CSV name ---
output_filename = simpledialog.askstring("Save CSV", "Enter output filename (e.g., yolo_output.csv):")
if not output_filename:
    raise ValueError("❌ No filename entered.")
if not output_filename.endswith(".csv"):
    output_filename += ".csv"

# --- Ask for directory to save CSV ---
save_dir = filedialog.askdirectory(title="Select Folder to Save Output CSV")
if not save_dir:
    raise FileNotFoundError("❌ No save location selected.")

# --- Save to CSV ---
# this file name should be YOLO_detected
output_path = os.path.join(save_dir, output_filename)
df = pd.DataFrame(detections)[["image", "x_yolo", "y_yolo"]]
df.to_csv(output_path, index=False)
print(f"✅ Coordinates saved to: {output_path}")
