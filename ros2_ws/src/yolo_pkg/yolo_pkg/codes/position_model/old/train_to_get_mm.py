import os
import pandas as pd
import torch
import torch.nn as nn
import torch.optim as optim
from sklearn.preprocessing import StandardScaler
import joblib
from tkinter import Tk, filedialog, simpledialog

# --- Hide tkinter root window ---
root = Tk()
root.withdraw()

# --- Select CSV file ---
# this file is the final results containing (x,y)YOLO and (x,y) in mm
csv_path = filedialog.askopenfilename(
    title="Select CSV File for Training",
    filetypes=[("CSV files", "*.csv")]
)
if not csv_path:
    raise FileNotFoundError("❌ No CSV file selected.")

# --- Load and prepare data ---
df = pd.read_csv(csv_path, sep=';')
df.columns = df.columns.str.strip()

X_raw = df[['x_yolo', 'y_yolo']].to_numpy()
Y_raw = df[['x_mm', 'y_mm']].to_numpy()

# Normalize inputs and outputs
X_scaler = StandardScaler()
Y_scaler = StandardScaler()
X = X_scaler.fit_transform(X_raw)
Y = Y_scaler.fit_transform(Y_raw)

# Convert to tensors
device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
X_tensor = torch.tensor(X, dtype=torch.float32).to(device)
Y_tensor = torch.tensor(Y, dtype=torch.float32).to(device)

# --- Define the network ---
class Net(nn.Module):
    def __init__(self):
        super(Net, self).__init__()
        self.layers = nn.Sequential(
            nn.Linear(2, 16),
            nn.ReLU(),
            nn.Linear(16, 32),
            nn.ReLU(),
            nn.Linear(32, 2)
        )

    def forward(self, x):
        return self.layers(x)

model = Net().to(device)

# --- Training setup ---
criterion = nn.MSELoss()
optimizer = optim.Adam(model.parameters(), lr=0.001)
epochs = 50000

# --- Train the model ---
print("🚀 Starting training...")
for epoch in range(1, epochs + 1):
    model.train()
    optimizer.zero_grad()
    outputs = model(X_tensor)
    loss = criterion(outputs, Y_tensor)
    loss.backward()
    optimizer.step()

    if epoch % 500 == 0 or epoch == 1:
        print(f"Epoch {epoch}/{epochs}  Loss: {loss.item():.6f}")

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

print(f"\n✅ Training complete! Model and scalers saved to: {save_dir}")
