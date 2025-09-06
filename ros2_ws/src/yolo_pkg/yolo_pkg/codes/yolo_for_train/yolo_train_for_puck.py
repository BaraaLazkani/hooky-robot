from ultralytics import YOLO
import os
import yaml
from sklearn.model_selection import train_test_split
import shutil

def prepare_data_split(data_dir, output_dir, test_size=0.2, random_state=42):
    """
    Split data into train and validation sets and create appropriate directory structure
    """
    # Create directories
    os.makedirs(os.path.join(output_dir, 'train', 'images'), exist_ok=True)
    os.makedirs(os.path.join(output_dir, 'train', 'labels'), exist_ok=True)
    os.makedirs(os.path.join(output_dir, 'val', 'images'), exist_ok=True)
    os.makedirs(os.path.join(output_dir, 'val', 'labels'), exist_ok=True)
    
    # Get all image files
    image_files = [f for f in os.listdir(os.path.join(data_dir, 'images')) 
                  if f.endswith(('.jpg', '.jpeg', '.png', '.bmp'))]
    
    # Split into train and validation
    train_files, val_files = train_test_split(
        image_files, test_size=test_size, random_state=random_state
    )
    
    # Copy files to appropriate directories
    for file in train_files:
        # Copy image
        shutil.copy2(
            os.path.join(data_dir, 'images', file),
            os.path.join(output_dir, 'train', 'images', file)
        )
        # Copy corresponding label file if exists
        label_file = os.path.splitext(file)[0] + '.txt'
        label_src = os.path.join(data_dir, 'labels', label_file)
        if os.path.exists(label_src):
            shutil.copy2(   
                label_src,
                os.path.join(output_dir, 'train', 'labels', label_file)
            )
    
    for file in val_files:
        # Copy image
        shutil.copy2(
            os.path.join(data_dir, 'images', file),
            os.path.join(output_dir, 'val', 'images', file)
        )
        # Copy corresponding label file if exists
        label_file = os.path.splitext(file)[0] + '.txt'
        label_src = os.path.join(data_dir, 'labels', label_file)
        if os.path.exists(label_src):
            shutil.copy2(
                label_src,
                os.path.join(output_dir, 'val', 'labels', label_file)
            )
    
    return output_dir, len(train_files), len(val_files)

def create_dataset_yaml(data_path, class_names, output_path='dataset.yaml'):
    """
    Create YAML dataset configuration file
    """
    dataset_config = {
        'path': data_path,
        'train': os.path.join(data_path, 'train'),
        'val': os.path.join(data_path, 'val'),
        'names': class_names
    }
    
    with open(output_path, 'w') as f:   
        yaml.dump(dataset_config, f, default_flow_style=False)
    
    return output_path

# Configuration - UPDATED PATHS FOR YOUR USER (jojo)
DATA_DIR = "/home/jojo/htr/src/yolo_pkg/yolo_pkg/codes/yolo_for_train/obj_train_data"
SPLIT_OUTPUT_DIR = "/home/jojo/htr/src/yolo_pkg/yolo_pkg/codes/yolo_for_train/split_data"  # Changed to jojo's home
CLASS_NAMES = {0: 'puck'}

# Check if data directory exists
if not os.path.exists(DATA_DIR):
    print(f"Error: Data directory '{DATA_DIR}' does not exist!")
    exit(1)

# Check if images subdirectory exists
images_dir = os.path.join(DATA_DIR, 'images')
if not os.path.exists(images_dir):
    print(f"Error: Images directory '{images_dir}' does not exist!")
    print("Please ensure your data is organized with 'images' and 'labels' subdirectories")
    exit(1)

# Prepare data split
print("Splitting data into training and validation sets...")
try:
    data_path, train_count, val_count = prepare_data_split(DATA_DIR, SPLIT_OUTPUT_DIR, test_size=0.2)
    print(f"Data split complete: {train_count} training samples, {val_count} validation samples")
except PermissionError as e:
    print(f"Permission error: {e}")
    print("Please check that you have write permissions to the output directory")
    exit(1)
except Exception as e:
    print(f"Error during data splitting: {e}")
    exit(1)

# Create dataset YAML file
yaml_path = create_dataset_yaml(SPLIT_OUTPUT_DIR, CLASS_NAMES, 'puck_dataset.yaml')

# Initialize model
model = YOLO("yolov8n.pt")  # Using nano version for CPU efficiency

# Train the model with improved parameters
results = model.train(
    data=yaml_path,
    epochs=150,
    imgsz=320,
    batch=8,
    device="cpu",
    optimizer="AdamW",
    lr0=0.001,
    lrf=0.01,
    momentum=0.9,
    weight_decay=0.0005,
    warmup_epochs=3,
    warmup_momentum=0.8,
    warmup_bias_lr=0.1,
    box=7.5,
    cls=0.5,
    dfl=1.5,
    hsv_h=0.015,
    hsv_s=0.7,
    hsv_v=0.4,
    degrees=0.0,
    translate=0.1,
    scale=0.5,
    shear=0.0,
    perspective=0.0,
    flipud=0.0,
    fliplr=0.5,
    mosaic=1.0,
    mixup=0.0,
    copy_paste=0.0,
    name="puck_v2",
    save=True,
    save_period=10,
    val=True,
    plots=True,
    exist_ok=True,
    patience=50,
    deterministic=True,
    single_cls=False,
    amp=False,
)

# The training process will automatically save the following in the 'runs/detect/puck_v2' directory:
# 1. Model weights (best.pt, last.pt)
# 2. Training arguments (args.yaml)
# 3. Training results (results.csv)
# 4. Training plots (multiple PNG files)

print("Training completed! All outputs saved in the 'runs/detect/puck_v2' directory.")