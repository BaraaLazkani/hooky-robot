import os
import shutil
from sklearn.model_selection import train_test_split

# Paths (adjust as needed)
# currently we are not using these because they are missing
extracted_dir = "/home/jaafar/htr/srs/yolo_pkg/yolo_pkg/codes/yolo_for_train/obj_train_data"  # From CVAT export
output_dir = "/home/jaafar/htr/srs/yolo_pkg/yolo_pkg/codes/yolo_for_train/puck_dataset"

# Create folders
os.makedirs(f"{output_dir}/images/train", exist_ok=True)
os.makedirs(f"{output_dir}/images/val", exist_ok=True)
os.makedirs(f"{output_dir}/labels/train", exist_ok=True)
os.makedirs(f"{output_dir}/labels/val", exist_ok=True)

# List and split images (80% train, 20% val)
all_images = [f for f in os.listdir(f"{extracted_dir}/images") if f.endswith(('.jpg', '.png'))]
train_images, val_images = train_test_split(all_images, test_size=0.2, random_state=42)

# Move files to train/val folders
for img in train_images:
    # Move image
    shutil.move(
        f"{extracted_dir}/images/{img}",
        f"{output_dir}/images/train/{img}"
    )
    # Move corresponding label
    label = os.path.splitext(img)[0] + ".txt"
    shutil.move(
        f"{extracted_dir}/labels/{label}",
        f"{output_dir}/labels/train/{label}"
    )

for img in val_images:
    # Move image
    shutil.move(
        f"{extracted_dir}/images/{img}",
        f"{output_dir}/images/val/{img}"
    )
    # Move corresponding label
    label = os.path.splitext(img)[0] + ".txt"
    shutil.move(
        f"{extracted_dir}/labels/{label}",
        f"{output_dir}/labels/val/{label}"
    )
    