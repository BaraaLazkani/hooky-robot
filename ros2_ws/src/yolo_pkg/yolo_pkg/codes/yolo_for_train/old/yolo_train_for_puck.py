from ultralytics import YOLO

model = YOLO("yolov8n.pt")  

results = model.train(
    data="/home/jaafar/htr/srs/yolo_pkg/yolo_pkg/codes/yolo_for_train/puck.yaml",
    epochs=100,
    imgsz=320,
    batch=8,              # Reduced for CPU
    device="cpu",         # Force CPU usage
    optimizer="Adam",
    lr0=0.01,
    name="puck_v1"
)