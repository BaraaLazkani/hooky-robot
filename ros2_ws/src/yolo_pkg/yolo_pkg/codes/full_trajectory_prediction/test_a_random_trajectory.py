import torch
import torch.nn as nn
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import os
import random
import joblib
from tkinter import Tk, filedialog
from sklearn.preprocessing import StandardScaler
import time

# Constants
INPUT_STEPS = 20
PREDICTION_STEPS = 30
MODEL_PATH = '/home/jojo/htr/src/yolo_pkg/yolo_pkg/codes/full_trajectory_prediction'
DATA_PATH = '/home/jojo/htr/src/yolo_pkg/yolo_pkg/codes/full_trajectory_prediction/real-time_frame_positions_in_mm/new/testing_points'
PLOT_PATH = '/home/jojo/htr/src/yolo_pkg/yolo_pkg/codes/full_trajectory_prediction/plots/new/testing_points'

# Enhanced Model Definition (matching the improved training model)
class EnhancedTrajectoryPredictor(nn.Module):
    def __init__(self):
        super().__init__()
        # Input: 4 features (x_mm, y_mm, v_x, v_y)
        self.lstm1 = nn.LSTM(4, 128, batch_first=True)
        self.bn1 = nn.BatchNorm1d(INPUT_STEPS)
        self.dropout1 = nn.Dropout(0.2)
        
        # Enhanced attention mechanism
        self.attention = nn.MultiheadAttention(128, 8, batch_first=True, dropout=0.1)
        
        self.lstm2 = nn.LSTM(128, 128, batch_first=True)
        self.bn2 = nn.BatchNorm1d(INPUT_STEPS)
        self.dropout2 = nn.Dropout(0.2)
        
        self.lstm3 = nn.LSTM(128, 64, batch_first=True)
        self.bn3 = nn.BatchNorm1d(INPUT_STEPS)
        self.dropout3 = nn.Dropout(0.2)
        
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
        
        # Add residual connection if applicable
        if self.residual_fc is not None:
            residual = self.residual_fc(residual)
            x = x + residual
        
        x = self.fc(x)
        return x

# Improved velocity calculation (matching training)
def calculate_velocities(df):
    df['time_delta_s'] = df['time_delta_ms'] / 1000.0
    
    # Use more robust velocity calculation with smoothing
    for coord in ['x_mm', 'y_mm']:
        # First derivative (velocity)
        df[f'v_{coord[0]}'] = df[coord].diff() / df['time_delta_s']
        
        # Apply smoothing
        df[f'v_{coord[0]}'] = df[f'v_{coord[0]}'].rolling(
            window=5, center=True, min_periods=1).mean().fillna(0)
    
    return df

def load_and_prepare_data(file_path):
    df = pd.read_csv(file_path, skiprows=1, names=['frame', 'time_delta_ms', 'x_mm', 'y_mm'])
    df = calculate_velocities(df)
    return df[['x_mm', 'y_mm', 'v_x', 'v_y']].values

def create_prediction_sequences(states, input_steps):
    X = []
    for i in range(len(states) - input_steps + 1):
        X.append(states[i:i + input_steps])
    return np.array(X)

def calculate_metrics(real, pred):
    """Calculate various performance metrics"""
    mae = np.mean(np.abs(real - pred))
    mse = np.mean((real - pred) ** 2)
    rmse = np.sqrt(mse)
    
    # Calculate R-squared
    ss_res = np.sum((real - pred) ** 2)
    ss_tot = np.sum((real - np.mean(real)) ** 2)
    r2 = 1 - (ss_res / ss_tot) if ss_tot != 0 else 0
    
    return {
        'mae': mae,
        'mse': mse,
        'rmse': rmse,
        'r2': r2
    }

def main():
    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    print(f"Using device: {device}")
    
    # Let user select model weights
    root = Tk()
    root.withdraw()
    model_file = filedialog.askopenfilename(
        initialdir=MODEL_PATH,
        title="Select Model Weights File",
        filetypes=[("PyTorch Model", "*.pth")]
    )
    
    if not model_file:
        print("No model file selected. Exiting.")
        return
    
    # Try to find corresponding scaler file
    base_name = os.path.splitext(model_file)[0]
    scaler_file = base_name + "_scaler.pkl"
    
    if not os.path.exists(scaler_file):
        print(f"Scaler file {scaler_file} not found. Trying to locate manually.")
        scaler_file = filedialog.askopenfilename(
            initialdir=MODEL_PATH,
            title="Select Scaler File",
            filetypes=[("Pickle File", "*.pkl")]
        )
        
        if not scaler_file:
            print("No scaler file selected. Exiting.")
            return
    
    # Load model and scaler
    model = EnhancedTrajectoryPredictor().to(device)
    model.load_state_dict(torch.load(model_file, map_location=device))
    model.eval()
    scaler = joblib.load(scaler_file)
    print("Model and scaler loaded successfully.")
    
    # Get all CSV files in the testing directory
    csv_files = [f for f in os.listdir(DATA_PATH) if f.endswith('.csv')]
    if not csv_files:
        print(f"No CSV files found in {DATA_PATH}")
        return
    
    print(f"Found {len(csv_files)} CSV files for testing")
    
    # Create directory for plots
    os.makedirs(PLOT_PATH, exist_ok=True)
    
    # Store metrics for overall performance
    all_metrics = []
    
    # Process each file
    for i, selected_file in enumerate(csv_files):
        print(f"\nProcessing file {i+1}/{len(csv_files)}: {selected_file}")
        file_path = os.path.join(DATA_PATH, selected_file)
        
        # Load and prepare data
        states = load_and_prepare_data(file_path)
        
        # Scale the data
        states_scaled = scaler.transform(states)
        
        # Create sequences for prediction
        X = create_prediction_sequences(states_scaled, INPUT_STEPS)
        
        # Make predictions
        predictions = []
        prediction_times = []
        
        with torch.no_grad():
            for i in range(0, len(X)):
                x_tensor = torch.tensor(X[i:i+1], dtype=torch.float32).to(device)
                
                # Time the prediction
                start_time = time.time()
                pred_scaled = model(x_tensor).cpu().numpy()
                end_time = time.time()
                
                pred_scaled = pred_scaled.reshape(PREDICTION_STEPS, 4)
                pred_unscaled = scaler.inverse_transform(pred_scaled)
                predictions.append(pred_unscaled[0])  # First prediction step
                prediction_times.append(end_time - start_time)
        
        # Convert to numpy array
        predictions = np.array(predictions)
        
        # Prepare data for plotting
        real_x = states[INPUT_STEPS:, 0]
        real_y = states[INPUT_STEPS:, 1]
        real_vx = states[INPUT_STEPS:, 2]
        real_vy = states[INPUT_STEPS:, 3]
        
        pred_x = predictions[:len(real_x), 0]
        pred_y = predictions[:len(real_y), 1]
        pred_vx = predictions[:len(real_vx), 2]
        pred_vy = predictions[:len(real_vy), 3]
        
        # Calculate metrics
        metrics = {
            'file': selected_file,
            'position_x': calculate_metrics(real_x, pred_x),
            'position_y': calculate_metrics(real_y, pred_y),
            'velocity_x': calculate_metrics(real_vx, pred_vx),
            'velocity_y': calculate_metrics(real_vy, pred_vy),
            'avg_prediction_time': np.mean(prediction_times) * 1000  # Convert to ms
        }
        all_metrics.append(metrics)
        
        # Print metrics
        print(f"  Position X - MAE: {metrics['position_x']['mae']:.2f}mm, RMSE: {metrics['position_x']['rmse']:.2f}mm, R²: {metrics['position_x']['r2']:.3f}")
        print(f"  Position Y - MAE: {metrics['position_y']['mae']:.2f}mm, RMSE: {metrics['position_y']['rmse']:.2f}mm, R²: {metrics['position_y']['r2']:.3f}")
        print(f"  Avg prediction time: {metrics['avg_prediction_time']:.2f}ms")
        
        # Create time axis
        time_axis = np.arange(len(real_x))
        
        # Create and save plots
        plt.figure(figsize=(15, 10))
        
        # Plot 1: Real vs Predicted Path
        plt.subplot(2, 3, 1)
        plt.plot(real_x, real_y, 'b-', label='Real Path', linewidth=2)
        plt.plot(pred_x, pred_y, 'g--', label='Predicted Path', linewidth=2)
        plt.xlabel('X Position (mm)')   
        plt.ylabel('Y Position (mm)')
        plt.title('Real vs Predicted Path')
        plt.legend()
        plt.grid(True)
        # Set fixed limits for x and y axes
        plt.xlim(0, 800)
        plt.ylim(0, 1600)
        
        # Plot 2: X Velocity Comparison
        plt.subplot(2, 3, 2)
        plt.plot(time_axis, real_vx, 'b-', label='Real Vx', linewidth=2)
        plt.plot(time_axis, pred_vx, 'g--', label='Predicted Vx', linewidth=2)
        plt.xlabel('Time Step')
        plt.ylabel('Velocity X (mm/s)')
        plt.title('X Velocity Comparison')
        plt.legend()
        plt.grid(True)
        
        # Plot 3: Y Velocity Comparison
        plt.subplot(2, 3, 3)
        plt.plot(time_axis, real_vy, 'b-', label='Real Vy', linewidth=2)
        plt.plot(time_axis, pred_vy, 'g--', label='Predicted Vy', linewidth=2)
        plt.xlabel('Time Step')
        plt.ylabel('Velocity Y (mm/s)')
        plt.title('Y Velocity Comparison')
        plt.legend()
        plt.grid(True)
        
        # Plot 4: Position Errors
        plt.subplot(2, 3, 4)
        x_error = np.abs(real_x - pred_x)
        y_error = np.abs(real_y - pred_y)
        plt.plot(time_axis, x_error, 'r-', label='X Position Error', linewidth=2)
        plt.plot(time_axis, y_error, 'm-', label='Y Position Error', linewidth=2)
        plt.xlabel('Time Step')
        plt.ylabel('Position Error (mm)')
        plt.title('Position Errors')
        plt.legend()
        plt.grid(True)
        
        # Plot 5: Velocity Errors
        plt.subplot(2, 3, 5)
        vx_error = np.abs(real_vx - pred_vx)
        vy_error = np.abs(real_vy - pred_vy)
        plt.plot(time_axis, vx_error, 'r-', label='Vx Error', linewidth=2)
        plt.plot(time_axis, vy_error, 'm-', label='Vy Error', linewidth=2)
        plt.xlabel('Time Step')
        plt.ylabel('Velocity Error (mm/s)')
        plt.title('Velocity Errors')
        plt.legend()
        plt.grid(True)
        
        # Plot 6: Prediction Time Distribution
        plt.subplot(2, 3, 6)
        plt.hist(prediction_times, bins=20, alpha=0.7, color='blue')
        plt.xlabel('Prediction Time (seconds)')
        plt.ylabel('Frequency')
        plt.title(f'Prediction Time Distribution\nAvg: {metrics["avg_prediction_time"]:.2f}ms')
        plt.grid(True)
        
        plt.tight_layout()
        
        # Save the plot
        plot_filename = os.path.join(PLOT_PATH, f"prediction_comparison_{selected_file.replace('.csv', '')}.png")
        plt.savefig(plot_filename, dpi=300, bbox_inches='tight')
        plt.close()
        
        print(f"✅ Plot saved: {plot_filename}")
    
    # Print overall performance summary
    print(f"\n{'='*60}")
    print("OVERALL PERFORMANCE SUMMARY")
    print(f"{'='*60}")
    
    # Calculate average metrics
    avg_metrics = {
        'position_x_mae': np.mean([m['position_x']['mae'] for m in all_metrics]),
        'position_x_rmse': np.mean([m['position_x']['rmse'] for m in all_metrics]),
        'position_x_r2': np.mean([m['position_x']['r2'] for m in all_metrics]),
        'position_y_mae': np.mean([m['position_y']['mae'] for m in all_metrics]),
        'position_y_rmse': np.mean([m['position_y']['rmse'] for m in all_metrics]),
        'position_y_r2': np.mean([m['position_y']['r2'] for m in all_metrics]),
        'avg_pred_time': np.mean([m['avg_prediction_time'] for m in all_metrics])
    }
    
    print(f"Average Position X - MAE: {avg_metrics['position_x_mae']:.2f}mm, RMSE: {avg_metrics['position_x_rmse']:.2f}mm, R²: {avg_metrics['position_x_r2']:.3f}")
    print(f"Average Position Y - MAE: {avg_metrics['position_y_mae']:.2f}mm, RMSE: {avg_metrics['position_y_rmse']:.2f}mm, R²: {avg_metrics['position_y_r2']:.3f}")
    print(f"Average Prediction Time: {avg_metrics['avg_pred_time']:.2f}ms")
    print(f"\n✅ All {len(csv_files)} files processed successfully!")

if __name__ == "__main__":
    main()