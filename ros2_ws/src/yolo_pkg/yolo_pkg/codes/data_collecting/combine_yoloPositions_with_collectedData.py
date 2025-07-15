import pandas as pd
from tkinter import Tk, filedialog, simpledialog
import os

# Hide the root tkinter window
root = Tk()
root.withdraw()

# --- Select a.csv ---
# the points taken from workspace
file_a = filedialog.askopenfilename(
    title="Select a.csv (semicolon-delimited)",
    filetypes=[("CSV files", "*.csv")]
)
if not file_a:
    raise FileNotFoundError("❌ You did not select a.csv.")

# --- Select b.csv ---
# the points extracted from YOLO
file_b = filedialog.askopenfilename(
    title="Select b.csv (standard CSV)",
    filetypes=[("CSV files", "*.csv")]
)
if not file_b:
    raise FileNotFoundError("❌ You did not select b.csv.")

# --- Ask for output filename ---
# this file is the final result
# this file is taken to the position model
output_filename = simpledialog.askstring("Output File Name", "Enter desired output file name (e.g., result.csv):")
if not output_filename:
    raise ValueError("❌ No output filename was provided.")
if not output_filename.endswith(".csv"):
    output_filename += ".csv"

# --- Ask for output directory ---
# the path should be like (/home/jaafar/htr/srs/yolo_pkg/yolo_pkg/codes/data_collecting/csv_files)
output_dir = filedialog.askdirectory(title="Select Directory to Save the Output")
if not output_dir:
    raise FileNotFoundError("❌ No output directory selected.")

# --- Load and process b.csv ---
df_b = pd.read_csv(file_b)
df_b['x_yolo'] = pd.to_numeric(df_b['x_yolo'], errors='coerce')
df_b['y_yolo'] = pd.to_numeric(df_b['y_yolo'], errors='coerce')

# --- Load and process a.csv ---
df_a = pd.read_csv(file_a, delimiter=';')

# --- Debugging row count ---
print(f"a.csv rows: {len(df_a)}")
print(f"b.csv rows: {len(df_b)}")

# --- Validate row count ---
if len(df_a) != len(df_b):
    raise ValueError("❌ Mismatch between number of rows in a.csv and b.csv.")

# --- Merge Data ---
df_a['x_yolo'] = df_b['x_yolo']
df_a['y_yolo'] = df_b['y_yolo']

# --- Save the result ---
output_path = os.path.join(output_dir, output_filename)
df_a.to_csv(output_path, index=False, sep=';')
print(f"✅ Done! Data merged and saved as: {output_path}")
