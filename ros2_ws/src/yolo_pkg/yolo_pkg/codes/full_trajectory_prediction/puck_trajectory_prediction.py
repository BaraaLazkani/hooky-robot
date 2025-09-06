import torch
import torch.nn as nn
from torch.utils.data import Dataset, DataLoader
import numpy as np
import os
import pandas as pd
from sklearn.model_selection import train_test_split
from sklearn.preprocessing import StandardScaler
import joblib
try:
    import tkinter as tk
    from tkinter import filedialog, simpledialog
    TKINTER_AVAILABLE = True
except ImportError:
    TKINTER_AVAILABLE = False
try:
    import matplotlib.pyplot as plt
    MATPLOTLIB_AVAILABLE = True
except ImportError:
    MATPLOTLIB_AVAILABLE = False
import time
import random

# Constants
INPUT_STEPS = 20
PREDICTION_STEPS = 30
BATCH_SIZE = 64
EPOCHS = 300
LEARNING_RATE = 0.0001
PATIENCE = 20  # For early stopping

# Paths
DATA_DIR = '/content/drive/MyDrive/codes/full_trajectory_prediction/real-time_frame_positions_in_mm/new'
PLOT_DIR = '/content/drive/MyDrive/codes/full_trajectory_prediction/plots/new'
MODEL_SAVE_DIR = '/content/drive/MyDrive/TrajectoryPrediction/models'

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

# Dataset Class with data augmentation
class TrajectoryDataset(Dataset):
    def __init__(self, X, y, augment=False):
        # Convert to float32 for compatibility with LSTM layers
        self.X = torch.tensor(X, dtype=torch.float32)
        self.y = torch.tensor(y, dtype=torch.float32)
        self.augment = augment

    def __len__(self):
        return len(self.X)

    def __getitem__(self, idx):
        X_item = self.X[idx]
        y_item = self.y[idx]
        
        # Data augmentation: random scaling
        if self.augment and random.random() > 0.5:
            scale_factor = random.uniform(0.9, 1.1)
            X_item = X_item * scale_factor
            # Only scale position components in y, not velocities
            y_reshaped = y_item.view(PREDICTION_STEPS, 4)
            y_reshaped[:, :2] = y_reshaped[:, :2] * scale_factor
            y_item = y_reshaped.view(-1)
            
        return X_item, y_item

# Enhanced Loss Function with position emphasis
def weighted_mse(y_true, y_pred):
    # Ensure both tensors are the same type
    if y_true.dtype != y_pred.dtype:
        y_true = y_true.to(y_pred.dtype)
    
    # Reshape to separate position and velocity
    y_true_reshaped = y_true.view(-1, PREDICTION_STEPS, 4)
    y_pred_reshaped = y_pred.view(-1, PREDICTION_STEPS, 4)
    
    # Higher weight for position errors (x, y)
    position_weights = torch.tensor([2.0, 2.0, 1.0, 1.0], device=y_true.device)
    weights = position_weights.repeat(PREDICTION_STEPS)
    
    squared_error = (y_true - y_pred) ** 2
    weighted_error = weights * squared_error
    return weighted_error.mean()

# Improved velocity calculation
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

# Data Loading with Explicit Column Names
def load_data(file_path):
    try:
        df = pd.read_csv(file_path, skiprows=1,
                         names=['frame', 'time_delta_ms', 'x_mm', 'y_mm'])
        required_columns = ['frame', 'time_delta_ms', 'x_mm', 'y_mm']
        if not all(col in df.columns for col in required_columns):
            raise ValueError(f"Missing required columns {required_columns} in {file_path}")
        
        # Calculate velocities
        df = calculate_velocities(df)
        
        numeric_cols = ['x_mm', 'y_mm', 'v_x', 'v_y']
        df[numeric_cols] = df[numeric_cols].apply(pd.to_numeric, errors='coerce')
        df_cleaned = df.dropna(subset=numeric_cols)
        
        if df_cleaned.empty:
            raise ValueError(f"No valid numeric data after cleaning in {file_path}")
        
        # Use only the 4 features for training
        states = df_cleaned[['x_mm', 'y_mm', 'v_x', 'v_y']].values
        print(f"Loaded {file_path}: {len(states)} frames after cleaning")
        return states
    except Exception as e:
        print(f"Error reading {file_path}: {e}")
        return None

# Sequence Creation for multi-step prediction
def create_training_sequences(states, input_steps, prediction_steps):
    X, y = [], []
    for i in range(len(states) - input_steps - prediction_steps + 1):
        X.append(states[i:i + input_steps])
        # Next 'prediction_steps' points as target
        y.append(states[i+input_steps:i+input_steps+prediction_steps].flatten())
    return np.array(X), np.array(y)

# Load and Split Data
def load_train_val_data(data_dir):
    files = [f for f in os.listdir(data_dir) if f.endswith('.csv')]
    if not files:
        raise ValueError(f"No .csv files found in {data_dir}")

    train_files, val_files = train_test_split(files, test_size=0.2, random_state=42)
    print(f"Train files: {len(train_files)}, Validation files: {len(val_files)}")

    scaler = StandardScaler()

    # First, fit the scaler on all training data
    all_train_states = []
    for file in train_files:
        file_path = os.path.join(data_dir, file)
        states = load_data(file_path)
        if states is None or len(states) < INPUT_STEPS + PREDICTION_STEPS:
            print(f"Skipping {file}: insufficient data")
            continue
        all_train_states.append(states)

    if not all_train_states:
        raise ValueError("No valid training data found.")

    all_train_states_concat = np.concatenate(all_train_states, axis=0)
    scaler.fit(all_train_states_concat)

    # Create datasets for each file
    train_datasets = []
    for file in train_files:
        file_path = os.path.join(data_dir, file)
        states = load_data(file_path)
        if states is None or len(states) < INPUT_STEPS + PREDICTION_STEPS:
            continue
        states = scaler.transform(states)
        X, y = create_training_sequences(states, INPUT_STEPS, PREDICTION_STEPS)
        if len(X) > 0:
            train_datasets.append(TrajectoryDataset(X, y, augment=True))  # Enable augmentation for training

    val_datasets = []
    for file in val_files:
        file_path = os.path.join(data_dir, file)
        states = load_data(file_path)
        if states is None or len(states) < INPUT_STEPS + PREDICTION_STEPS:
            continue
        states = scaler.transform(states)
        X, y = create_training_sequences(states, INPUT_STEPS, PREDICTION_STEPS)
        if len(X) > 0:
            val_datasets.append(TrajectoryDataset(X, y, augment=False))  # No augmentation for validation

    return train_datasets, val_datasets, scaler

# Main Script with enhanced training
if __name__ == "__main__":
    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    print(f"Using device: {device}")

    # Check if we're running in Colab by checking if /content exists
    IS_COLAB = os.path.exists('/content')
    
    if IS_COLAB:
        print("Running in Google Colab environment")
        # Create directories if they don't exist
        os.makedirs(DATA_DIR, exist_ok=True)
        os.makedirs(PLOT_DIR, exist_ok=True)
        os.makedirs(MODEL_SAVE_DIR, exist_ok=True)
    else:
        print("Running in local environment")
        # Use local paths if not in Colab
        DATA_DIR = './data'
        PLOT_DIR = './plots'
        MODEL_SAVE_DIR = './models'
        os.makedirs(DATA_DIR, exist_ok=True)
        os.makedirs(PLOT_DIR, exist_ok=True)
        os.makedirs(MODEL_SAVE_DIR, exist_ok=True)

    # Check if data directory exists
    if not os.path.exists(DATA_DIR):
        print(f"Data directory {DATA_DIR} does not exist. Please upload your CSV files to this folder.")
        if IS_COLAB:
            print("In Colab, you need to:")
            print("1. Mount Google Drive manually using: from google.colab import drive; drive.mount('/content/drive')")
            print("2. Update the DATA_DIR path to match your actual directory structure")
            print("3. Make sure your CSV files are in the correct directory")
        exit()

    train_datasets, val_datasets, scaler = load_train_val_data(DATA_DIR)
    print(f"Loaded {len(train_datasets)} training datasets and {len(val_datasets)} validation datasets")

    if len(train_datasets) == 0:
        print("Error: No valid training datasets found.")
        print("Please make sure:")
        print("1. Your CSV files are in the correct format")
        print("2. Files are in the directory: " + DATA_DIR)
        print("3. Each file has at least " + str(INPUT_STEPS + PREDICTION_STEPS) + " data points")
        exit()

    model = TrajectoryPredictor().to(device)
    optimizer = torch.optim.Adam(model.parameters(), lr=LEARNING_RATE)
    
    # Add learning rate scheduler
    scheduler = torch.optim.lr_scheduler.ReduceLROnPlateau(
        optimizer, mode='min', factor=0.5, patience=10)
    
    # Early stopping
    best_loss = float('inf')
    epochs_without_improvement = 0

    epochs_list, times_list, train_losses, val_losses = [], [], [], []

    # Enhanced training loop
    for epoch in range(EPOCHS):
        model.train()
        total_loss = 0
        total_samples = 0
        start_time = time.time()
        
        # Shuffle the order of training datasets (files)
        random.shuffle(train_datasets)
        
        # Process each file separately
        for dataset in train_datasets:
            # Create a DataLoader for this file with augmentation
            file_loader = DataLoader(dataset, batch_size=BATCH_SIZE, shuffle=True)
            
            for X_batch, y_batch in file_loader:
                X_batch, y_batch = X_batch.to(device), y_batch.to(device)
                
                optimizer.zero_grad()
                
                # Forward pass
                predictions = model(X_batch)
                loss = weighted_mse(y_batch, predictions)
                loss.backward()
                
                # Gradient clipping to prevent exploding gradients
                torch.nn.utils.clip_grad_norm_(model.parameters(), max_norm=1.0)
                
                optimizer.step()
                
                total_loss += loss.item() * X_batch.size(0)
                total_samples += X_batch.size(0)
        
        avg_train_loss = total_loss / total_samples if total_samples > 0 else float('inf')
        epoch_time = time.time() - start_time

        # Validation
        model.eval()
        total_val_loss = 0
        total_val_samples = 0
        with torch.no_grad():
            for dataset in val_datasets:
                file_loader = DataLoader(dataset, batch_size=BATCH_SIZE, shuffle=False)
                
                for X_batch, y_batch in file_loader:
                    X_batch, y_batch = X_batch.to(device), y_batch.to(device)
                    
                    predictions = model(X_batch)
                    loss = weighted_mse(y_batch, predictions)
                    
                    total_val_loss += loss.item() * X_batch.size(0)
                    total_val_samples += X_batch.size(0)
        
        avg_val_loss = total_val_loss / total_val_samples if total_val_samples > 0 else float('inf')
        
        # Update learning rate
        scheduler.step(avg_val_loss)
        
        print(f"Epoch {epoch+1}/{EPOCHS}, Train Loss: {avg_train_loss:.4f}, Val Loss: {avg_val_loss:.4f}, Time: {epoch_time:.2f}s")

        # Check for improvement
        if avg_val_loss < best_loss:
            best_loss = avg_val_loss
            epochs_without_improvement = 0
            # Save the best model locally first
            torch.save(model.state_dict(), 'best_model.pth')
        else:
            epochs_without_improvement += 1
            
        # Early stopping
        if epochs_without_improvement >= PATIENCE:
            print(f"Early stopping at epoch {epoch+1}")
            break

        epochs_list.append(epoch + 1)
        times_list.append(epoch_time)
        train_losses.append(avg_train_loss)
        val_losses.append(avg_val_loss)

    print("Training complete. Best validation loss:", best_loss)

    # Save model and scaler
    model_path = os.path.join(MODEL_SAVE_DIR, 'best_model.pth')
    scaler_path = os.path.join(MODEL_SAVE_DIR, 'best_model_scaler.pkl')
    
    if os.path.exists('best_model.pth'):
        model_state_dict = torch.load('best_model.pth')
        torch.save(model_state_dict, model_path)
        print(f"Model saved to: {model_path}")
    
    joblib.dump(scaler, scaler_path)
    print(f"Scaler saved to: {scaler_path}")

    # Plot training time and loss
    if MATPLOTLIB_AVAILABLE:
        # Training time plot
        plt.figure(figsize=(10, 6))
        plt.plot(epochs_list, times_list, 'b-', label='Time per Epoch')
        plt.xlabel('Epoch')
        plt.ylabel('Time (seconds)')
        plt.title('Time Taken for Each Training Epoch')
        plt.legend()
        plt.grid(True)
        time_plot_path = os.path.join(PLOT_DIR, 'training_time_per_epoch.png')
        plt.savefig(time_plot_path)
        plt.close()
        print(f"Saved training time plot to {time_plot_path}")
        
        # Loss plot
        plt.figure(figsize=(10, 6))
        plt.plot(epochs_list, train_losses, 'b-', label='Training Loss')
        plt.plot(epochs_list, val_losses, 'r-', label='Validation Loss')
        plt.xlabel('Epoch')
        plt.ylabel('Loss (Weighted MSE)')
        plt.title('Training and Validation Loss Over Epochs')
        plt.legend()
        plt.grid(True)
        loss_plot_path = os.path.join(PLOT_DIR, 'loss_over_epochs.png')
        plt.savefig(loss_plot_path)
        plt.close()
        print(f"Saved loss plot to {loss_plot_path}")
    else:
        print("Matplotlib not available, skipping plots.")