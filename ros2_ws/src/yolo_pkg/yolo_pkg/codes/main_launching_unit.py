# Edited import cv2_main_unit.py
import cv2
import torch
import torch.nn as nn
import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from collections import deque
import time
import joblib
from ultralytics import YOLO

# Verify CUDA setup
device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
print(f"PyTorch device: {device}")
if device.type == "cuda":
    print(f"GPU: {torch.cuda.get_device_name(0)}")
    print(f"CUDA version: {torch.version.cuda}")

# --- Paths ---
yolo_model_path = '/home/humam/hooky/src/yolo_pkg/yolo_pkg/codes/yolo_for_train/runs/detect/puck_v2/weights/best.pt'
position_model_path = '/home/humam/hooky/src/yolo_pkg/yolo_pkg/codes/position_model/puck_mm_v1.pt'
x_scaler_path = '/home/humam/hooky/src/yolo_pkg/yolo_pkg/codes/position_model/puck_X_scaler.pkl'
y_scaler_path = '/home/humam/hooky/src/yolo_pkg/yolo_pkg/codes/position_model/puck_Y_scaler.pkl'

# Load scalers
X_scaler = joblib.load(x_scaler_path)
Y_scaler = joblib.load(y_scaler_path)

# --- Define LSTM trajectory prediction network ---
INPUT_STEPS = 20
PREDICTION_STEPS = 30

class TrajectoryPredictor(nn.Module):
    def __init__(self):
        super().__init__()
        # Input: 4 features (x_mm, y_mm, v_x, v_y)
        # lstm1: 4 -> 128 (hidden_size = 128)
        self.lstm1 = nn.LSTM(input_size=4, hidden_size=128, batch_first=True)
        self.bn1 = nn.BatchNorm1d(INPUT_STEPS)
        self.dropout1 = nn.Dropout(0.2)
        
        # Enhanced attention mechanism (input size = 128)
        self.attention = nn.MultiheadAttention(embed_dim=128, num_heads=8, batch_first=True, dropout=0.1)
        
        # lstm2: 128 -> 64 (hidden_size = 64)
        self.lstm2 = nn.LSTM(input_size=128, hidden_size=64, batch_first=True)
        self.bn2 = nn.BatchNorm1d(INPUT_STEPS)
        self.dropout2 = nn.Dropout(0.2)
        
        # lstm3: 64 -> 64 (hidden_size = 64)
        self.lstm3 = nn.LSTM(input_size=64, hidden_size=64, batch_first=True)
        self.bn3 = nn.BatchNorm1d(INPUT_STEPS)
        self.dropout3 = nn.Dropout(0.2)
        
        # Layer normalization (from the debug output)
        self.ln = nn.LayerNorm(64)
        
        # Residual connection
        self.residual_fc = nn.Linear(4, 64) if INPUT_STEPS > 0 else None
        
        # Output: PREDICTION_STEPS * 4 (x, y, v_x, v_y)
        self.fc = nn.Linear(64, PREDICTION_STEPS * 4)

    def forward(self, x):
        # Convert input to float32 for LSTM compatibility
        x = x.float()
        
        # Store residual
        residual = x[:, -1, :]  # Last timestep
        
        # First LSTM layer with batch norm
        x, _ = self.lstm1(x)
        x = self.bn1(x)
        x = self.dropout1(x)
        
        # Apply attention with residual connection
        attn_output, _ = self.attention(x, x, x)
        x = x + attn_output  # Residual connection
        
        # Second LSTM layer
        x, _ = self.lstm2(x)
        x = self.bn2(x)
        x = self.dropout2(x)
        
        # Third LSTM layer
        x, _ = self.lstm3(x)
        x = self.bn3(x)
        x = self.dropout3(x[:, -1, :])  # Use only last timestep
        
        # Apply layer normalization
        x = self.ln(x)
        
        # Add residual connection if applicable
        if self.residual_fc is not None:
            residual = self.residual_fc(residual)
            x = x + residual
        
        x = self.fc(x)
        return x


# ROS2 Node for trajectory prediction
class TrajectoryPredictionNode(Node):
    def __init__(self):
        super().__init__('trajectory_prediction_node')
        
        # Movement detection parameters
        self.movement_threshold = 10.0  # mm threshold for significant movement
        self.valid_y_range = (0, 700)   # Send data if puck is within this y-range
        self.initial_position = None
        self.movement_detected = False
        self.movement_start_frame = 0
        self.frame_count = 0
        self.prediction_made = False
        self.movement_buffer = deque(maxlen=20)  # Buffer to track recent movements
        
        # Timing statistics
        self.timing_stats = {
            'yolo_time': deque(maxlen=30),
            'position_time': deque(maxlen=30),
            'prediction_time': deque(maxlen=30),
            'total_time': deque(maxlen=30)
        }
        self.frame_times = deque(maxlen=30)
        self.last_frame_time = time.time()
        
        # Load YOLO model
        self.yolo = YOLO(yolo_model_path)
        self.yolo.to(device)
        print(f"YOLO device: {self.yolo.device}")
        
        # Warm up GPU
        print("Warming up GPU...")
        dummy_frame = np.zeros((720, 1280, 3), dtype=np.uint8)
        for _ in range(5):
            self.yolo.track(dummy_frame, imgsz=320, conf=0.25, iou=0.6, augment=False, 
                           device=device, tracker='bytetrack_puck.yaml', persist=True, max_det=1)
        print("GPU warm-up complete.")
        
        # Load position model
        self.position_model = self.load_position_model(position_model_path)
        
        # Load trajectory model and scaler
        trajectory_model_path = '/home/humam/hooky/src/yolo_pkg/yolo_pkg/codes/20-30-300.pth'
        trajectory_scaler_path = '/home/humam/hooky/src/yolo_pkg/yolo_pkg/codes/20-30-300_scaler.pkl'
        
        self.trajectory_model = TrajectoryPredictor().to(device)
        self.trajectory_model.load_state_dict(torch.load(trajectory_model_path, map_location=device), strict=False)
        self.trajectory_model.eval()
        
        self.trajectory_scaler = joblib.load(trajectory_scaler_path)
        
        # Create publishers
        self.present_points_pub = self.create_publisher(Float32MultiArray, 'present_points', 10)
        self.pred_points_pub = self.create_publisher(Float32MultiArray, 'pred_points', 10)
        self.pixel_points_pub = self.create_publisher(Float32MultiArray,'pixel_points',10)
        
        # Initialize webcam
        self.cap = cv2.VideoCapture(2, cv2.CAP_V4L2)
        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
        self.cap.set(cv2.CAP_PROP_FPS, 30)
        
        # Verify webcam settings
        width = self.cap.get(cv2.CAP_PROP_FRAME_WIDTH)
        height = self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT)
        fps = self.cap.get(cv2.CAP_PROP_FPS)
        print(f"Webcam: {width}x{height} at {fps} fps")
        
        if not self.cap.isOpened():
            self.get_logger().error("Could not open webcam.")
            raise RuntimeError("Could not open webcam.")
        
        # Data buffers
        self.frame_buffer = deque(maxlen=INPUT_STEPS)
        self.last_x, self.last_y = None, None
        self.last_time = None
        self.time_accumulator = 0.0
        
        # Smoothing buffers
        smooth_window = 1
        self.x_smooth, self.y_smooth = deque(maxlen=smooth_window), deque(maxlen=smooth_window)
        
        # Start processing
        self.timer = self.create_timer(1/30, self.process_frame)  # 30 FPS
    
    def load_position_model(self, model_path):
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
        
        model = PositionNet()
        model.load_state_dict(torch.load(model_path, map_location=device))
        model.to(device).eval()
        return model
    
    def check_position_valid(self, y):
        """Check if the puck is within valid y-range (0, 700)"""
        y_min, y_max = self.valid_y_range
        return y_min <= y <= y_max
    
    def check_movement(self, x, y):
        """Check if the puck has moved significantly from its initial position"""
        if self.initial_position is None:
            self.initial_position = (x, y)
            return False
        
        dx = x - self.initial_position[0]
        dy = y - self.initial_position[1]
        distance = np.sqrt(dx**2 + dy**2)
        
        return distance > self.movement_threshold
    
    def print_timing_stats(self):
        """Print timing statistics to terminal"""
        if not any(self.timing_stats.values()):
            return
            
        print("\n" + "="*60)
        print("TIMING STATISTICS (last 30 frames)")
        print("="*60)
        
        for process, times in self.timing_stats.items():
            if times:
                avg_time = sum(times) / len(times) * 1000  # Convert to ms
                max_time = max(times) * 1000
                min_time = min(times) * 1000
                print(f"{process:15s}: {avg_time:6.2f} ms (min: {min_time:6.2f}, max: {max_time:6.2f})")
        
        if self.frame_times:
            avg_fps = 1.0 / (sum(self.frame_times) / len(self.frame_times))
            print(f"{'FPS':15s}: {avg_fps:6.2f}")
        print("="*60)
    
    def process_frame(self):
        total_start_time = time.time()
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warn("Failed to grab frame.")
            return
        # --------------------------------------------------
        # BLUE OBJECT DETECTION
        # --------------------------------------------------
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        lower_blue = np.array([100, 100, 50])
        upper_blue = np.array([130, 255, 255])
        mask = cv2.inRange(hsv, lower_blue, upper_blue)
        M = cv2.moments(mask)
        
        self.frame_count += 1
        current_time = time.time()
        time_delta_ms = 0.0 if self.last_time is None else (current_time - self.last_time) * 1000
        self.last_time = current_time
        
        # Add time to accumulator
        self.time_accumulator += time_delta_ms

        # YOLO tracking
        yolo_start_time = time.time()
        results = self.yolo.track(
            frame,
            imgsz=320,
            conf=0.4,
            iou=0.5,
            augment=False,
            device=device,
            tracker='bytetrack_puck.yaml',
            persist=True,
            max_det=1
        )
        yolo_time = time.time() - yolo_start_time
        self.timing_stats['yolo_time'].append(yolo_time)
        
        boxes = results[0].boxes
        if M["m00"] > 0:
           cx = int(M["m10"] / M["m00"])
           cy = int(M["m01"] / M["m00"])
           # draw center
           cv2.circle(frame, (cx, cy), 8, (0, 0, 255), -1)

           cv2.putText(frame,
              f"({cx},{cy})",
             (cx + 10, cy - 10),
             cv2.FONT_HERSHEY_SIMPLEX,
             0.5,
             (0, 0, 255),
             2)

           # publish pixel coordinates
           pixel_msg = Float32MultiArray()
           pixel_msg.data = [float(cx), float(cy)]
           self.pixel_points_pub.publish(pixel_msg)

           xx_center = cx
           yy_center = cy

        if boxes is not None and boxes.conf is not None and len(boxes) > 0:
            best_idx = boxes.conf.argmax().item()
            best_box = boxes.xywh[best_idx].tolist()
            x_center, y_center, w, h = best_box

            # Position prediction
            position_start_time = time.time()
            input_array = np.array([[y_center, x_center]])
            x_input_scaled = X_scaler.transform(input_array)
            x_tensor = torch.tensor(x_input_scaled, dtype=torch.float32).to(device)
            y_pred_scaled = self.position_model(x_tensor).detach().cpu().numpy()
            x_mm, y_mm = Y_scaler.inverse_transform(y_pred_scaled)[0]
            position_time = time.time() - position_start_time
            self.timing_stats['position_time'].append(position_time)

            # Apply smoothing
            self.x_smooth.append(x_mm)
            self.y_smooth.append(y_mm)
            x_mm_s = np.mean(self.x_smooth)
            y_mm_s = np.mean(self.y_smooth)

            # Check if position is within valid y-range
            is_in_valid_area = self.check_position_valid(y_mm_s)

            # Check for significant movement (only if in valid area)
            if is_in_valid_area and not self.movement_detected:
                self.movement_detected = self.check_movement(x_mm_s, y_mm_s)
                if self.movement_detected:
                    self.movement_start_frame = self.frame_count
                    self.prediction_made = False
                    self.get_logger().info(f"Movement detected at frame {self.frame_count}! Position: ({x_mm_s:.1f}, {y_mm_s:.1f})")

            # Calculate velocities
            if self.last_x is not None and self.last_y is not None:
                time_delta_s = self.time_accumulator / 1000.0  # Convert to seconds
                v_x = (x_mm_s - self.last_x) / time_delta_s if time_delta_s > 0 else 0
                v_y = (y_mm_s - self.last_y) / time_delta_s if time_delta_s > 0 else 0
            else:
                v_x, v_y = 0, 0

            # Reset time accumulator
            self.time_accumulator = 0.0
            
            # Update last position
            self.last_x, self.last_y = x_mm_s, y_mm_s

            # Always publish current position and velocity to present_points
            present_msg = Float32MultiArray()
            present_msg.data = [self.frame_count, x_mm_s, y_mm_s, v_x, v_y]
            self.present_points_pub.publish(present_msg)
            
            # Always collect data for prediction accuracy
            current_state = [x_mm_s, y_mm_s, v_x, v_y]
            self.frame_buffer.append(current_state)
            
            # Check if we have enough data for prediction and movement was detected
            prediction_time = 0
            if self.movement_detected and len(self.frame_buffer) >= INPUT_STEPS:
                # Prepare input for trajectory prediction
                prediction_start_time = time.time()
                input_sequence = np.array(list(self.frame_buffer))
                input_scaled = self.trajectory_scaler.transform(input_sequence)
                input_tensor = torch.tensor(input_scaled, dtype=torch.float32).unsqueeze(0).to(device)
                
                # Make prediction
                with torch.no_grad():
                    pred_scaled = self.trajectory_model(input_tensor).cpu().numpy()
                    pred_scaled = pred_scaled.reshape(PREDICTION_STEPS, 4)
                    pred_unscaled = self.trajectory_scaler.inverse_transform(pred_scaled)
                    for i in range(PREDICTION_STEPS):
                      px = int(pred_unscaled[i, 0])
                      py = int(pred_unscaled[i, 1])
                    
                    cv2.circle(frame, (px, py), 5, (0, 0, 255), -1)
                    
                    if i > 0:
                      prev_px = int(pred_unscaled[i - 1, 0])
                      prev_py = int(pred_unscaled[i - 1, 1])
                      
                      cv2.line(frame,(prev_px, prev_py),(px, py),(255, 0, 0),2)
                
                prediction_time = time.time() - prediction_start_time
                self.timing_stats['prediction_time'].append(prediction_time)
                
                # Publish predicted positions starting from frame 41 (adjusted for user example: detect at 20, send from 41)
                if self.frame_count >= self.movement_start_frame + 20:
                    pred_msg = Float32MultiArray()
                    pred_data = []
                    
                    for i in range(PREDICTION_STEPS):
                        frame_num = self.movement_start_frame + 21 + i  # Adjusted to start from 41 if detect at 20
                        x_pos = pred_unscaled[i, 0]
                        y_pos = pred_unscaled[i, 1]
                        
                        # Only include positions within valid y-range
                        if self.check_position_valid(y_pos):
                            pred_data.extend([frame_num, x_pos, y_pos])
                    
                    if pred_data:  # Only publish if we have valid predictions
                        pred_msg.data = pred_data
                        self.pred_points_pub.publish(pred_msg)
                        self.get_logger().info(f"Published {len(pred_data)//3} predicted positions")

            # Draw detection on frame
            left = int(x_center - w / 2)
            top = int(y_center - h / 2)
            right = int(x_center + w / 2)
            bottom = int(y_center + h / 2)
            cv2.rectangle(frame, (left, top), (right, bottom), (0, 255, 0), 2)
            
            # Add timing information to the frame
            total_time = time.time() - total_start_time
            self.timing_stats['total_time'].append(total_time)
            self.frame_times.append(total_time)
            
            # Print timing stats every 30 frames
            if self.frame_count % 30 == 0:
                self.print_timing_stats()
            
            # Display timing info on frame
            timing_text = f"YOLO: {yolo_time*1000:.1f}ms, Pos: {position_time*1000:.1f}ms"
            if prediction_time > 0:
                timing_text += f", Pred: {prediction_time*1000:.1f}ms"
            timing_text += f", Total: {total_time*1000:.1f}ms"
            
            cv2.putText(frame, timing_text, (10, 70), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 0), 2)
            
            status_text = "Stationary"
            status_color = (0, 255, 0)  # Green
            if self.movement_detected:
                frames_since_movement = self.frame_count - self.movement_start_frame
                status_text = f"Moving: {frames_since_movement} frames"
                status_color = (0, 165, 255)  # Orange
                
                if frames_since_movement >= INPUT_STEPS and self.prediction_made:
                    status_text = f"Prediction made: {frames_since_movement} frames"
                    status_color = (255, 0, 0)  # Blue
                    
            if not is_in_valid_area:
                status_text = "Out of Range"
                status_color = (0, 0, 255)  # Red
                
            text = f"x = {x_mm_s:.1f} mm, y = {y_mm_s:.1f} mm, Status: {status_text}"
            cv2.putText(frame, text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, status_color, 2)
            
        else:
            # No puck detected - reset movement detection
            self.movement_detected = False
            self.initial_position = None
            cv2.putText(frame, "No object tracked", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
            
            # Still record total time
            total_time = time.time() - total_start_time
            self.timing_stats['total_time'].append(total_time)
            self.frame_times.append(total_time)

        cv2.imshow("Live YOLO + Trajectory Prediction", frame)
        
        # Check for exit key
        if cv2.waitKey(1) & 0xFF == ord('q'):
            self.destroy_node()
            rclpy.shutdown()

    def destroy_node(self):
        self.cap.release()
        cv2.destroyAllWindows()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryPredictionNode()
    
    try:
        rclpy.spin(node)
        cv2.imshow("Blue Mask", mask)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
