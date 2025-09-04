import os
import pandas as pd
import torch
import torch.nn as nn
import torch.optim as optim
from sklearn.preprocessing import StandardScaler
from sklearn.model_selection import train_test_split
import joblib
from tkinter import Tk, filedialog, simpledialog
import matplotlib.pyplot as plt
import numpy as np
from torch.utils.data import DataLoader, TensorDataset

# --- Hide tkinter root window ---
root = Tk()
root.withdraw()
save_dir = "/home/jojo/htr/src/yolo_pkg/yolo_pkg/codes/position_model"
model_name = "puck"
# --- Select CSV file ---
csv_path = filedialog.askopenfilename(
    title="Select CSV File for Training",
    filetypes=[("CSV files", "*.csv")]
)
if not csv_path:
    raise FileNotFoundError("❌ No CSV file selected.")

# --- Load and prepare data ---
df = pd.read_csv(csv_path, sep=';')
df.columns = df.columns.str.strip()

# Check for required columns
required_columns = ['x_yolo', 'y_yolo', 'x_mm', 'y_mm']
if not all(col in df.columns for col in required_columns):
    raise ValueError(f"❌ CSV must contain these columns: {required_columns}")

X_raw = df[['x_yolo', 'y_yolo']].to_numpy()
Y_raw = df[['x_mm', 'y_mm']].to_numpy()

# Split data into training and validation sets
X_train, X_val, Y_train, Y_val = train_test_split(
    X_raw, Y_raw, test_size=0.2, random_state=42
)

# Normalize inputs and outputs
X_scaler = StandardScaler()
Y_scaler = StandardScaler()
X_train_scaled = X_scaler.fit_transform(X_train)
X_val_scaled = X_scaler.transform(X_val)
Y_train_scaled = Y_scaler.fit_transform(Y_train)
Y_val_scaled = Y_scaler.transform(Y_val)

# Convert to tensors
device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
print(f"Using device: {device}")

X_train_tensor = torch.tensor(X_train_scaled, dtype=torch.float32).to(device)
Y_train_tensor = torch.tensor(Y_train_scaled, dtype=torch.float32).to(device)
X_val_tensor = torch.tensor(X_val_scaled, dtype=torch.float32).to(device)
Y_val_tensor = torch.tensor(Y_val_scaled, dtype=torch.float32).to(device)

# Create DataLoader for batch training
train_dataset = TensorDataset(X_train_tensor, Y_train_tensor)
train_loader = DataLoader(train_dataset, batch_size=32, shuffle=True)

# --- Define the improved network ---
class ImprovedNet(nn.Module):
    def __init__(self, dropout_rate=0.2):
        super(ImprovedNet, self).__init__()
        self.layers = nn.Sequential(
            nn.Linear(2, 64),
            nn.ReLU(),
            nn.Dropout(dropout_rate),
            nn.Linear(64, 128),
            nn.ReLU(),
            nn.Dropout(dropout_rate),
            nn.Linear(128, 64),
            nn.ReLU(),
            nn.Linear(64, 32),
            nn.ReLU(),
            nn.Linear(32, 2)
        )
        
    def forward(self, x):
        return self.layers(x)

model = ImprovedNet().to(device)

# --- Training setup ---
criterion = nn.MSELoss()
optimizer = optim.Adam(model.parameters(), lr=0.001, weight_decay=1e-5)
scheduler = optim.lr_scheduler.ReduceLROnPlateau(optimizer, mode='min', factor=0.5, patience=1000)
epochs = 50000

# Track training progress
train_losses = []
val_losses = []
best_val_loss = float('inf')
patience_counter = 0
patience_limit = 5000  # Early stopping patience

# --- Train the model ---
print("🚀 Starting training...")
for epoch in range(1, epochs + 1):
    model.train()
    epoch_train_loss = 0
    
    # Batch training
    for batch_X, batch_Y in train_loader:
        optimizer.zero_grad()
        outputs = model(batch_X)
        loss = criterion(outputs, batch_Y)
        loss.backward()
        optimizer.step()
        epoch_train_loss += loss.item()
    
    # Calculate average training loss
    avg_train_loss = epoch_train_loss / len(train_loader)
    train_losses.append(avg_train_loss)
    
    # Validation
    model.eval()
    with torch.no_grad():
        val_outputs = model(X_val_tensor)
        val_loss = criterion(val_outputs, Y_val_tensor)
        val_losses.append(val_loss.item())
    
    # Learning rate scheduling
    scheduler.step(val_loss)
    
    # Save best model
    if val_loss.item() < best_val_loss:
        best_val_loss = val_loss.item()
        torch.save(model.state_dict(), os.path.join(save_dir, f"{model_name}_best.pt"))
        patience_counter = 0
    else:
        patience_counter += 1
    
    # Early stopping
    if patience_counter >= patience_limit:
        print(f"Early stopping at epoch {epoch}")
        break
    
    if epoch % 500 == 0 or epoch == 1:
        print(f"Epoch {epoch}/{epochs}  Train Loss: {avg_train_loss:.6f}  Val Loss: {val_loss.item():.6f}")

# Load best model
model.load_state_dict(torch.load(os.path.join(save_dir, f"{model_name}_best.pt")))

# --- Ask for model name and save location ---
model_name = simpledialog.askstring("Save Model", "Enter base name for the model (e.g., model_v1):")
if not model_name:
    raise ValueError("❌ No model name entered.")

save_dir = filedialog.askdirectory(title="Select Directory to Save the Model and Scalers")
if not save_dir:
    raise FileNotFoundError("❌ No directory selected.")

# --- Save the model and scalers ---
torch.save(model.state_dict(), os.path.join(save_dir, f"{model_name}.pt"))
joblib.dump(X_scaler, os.path.join(save_dir, f"{model_name}_X_scaler.pkl"))
joblib.dump(Y_scaler, os.path.join(save_dir, f"{model_name}_Y_scaler.pkl"))

# Save training history
history = pd.DataFrame({
    'epoch': range(1, len(train_losses) + 1),
    'train_loss': train_losses,
    'val_loss': val_losses
})
history.to_csv(os.path.join(save_dir, f"{model_name}_training_history.csv"), index=False)

# Plot training history
plt.figure(figsize=(10, 6))
plt.plot(history['epoch'], history['train_loss'], label='Training Loss')
plt.plot(history['epoch'], history['val_loss'], label='Validation Loss')
plt.xlabel('Epoch')
plt.ylabel('Loss')
plt.title('Training History')
plt.legend()
plt.yscale('log')  # Use log scale for better visualization
plt.savefig(os.path.join(save_dir, f"{model_name}_training_history.png"))
plt.close()

print(f"\n✅ Training complete! Model and scalers saved to: {save_dir}")