import pandas as pd
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from tkinter import Tk, filedialog

# Open file dialog to choose CSV file
root = Tk()
root.withdraw()  # Hide the root window
file_path = filedialog.askopenfilename(
    # the file name should contain (point_taken_of_workspace)
    title="Select CSV file",
    filetypes=[("CSV files", "*.csv")]
)

# Check if a file was selected
if not file_path:
    raise FileNotFoundError("No file selected.")

# Load CSV data with semicolon delimiter
df = pd.read_csv(file_path, delimiter=';')

# Extract x and y values
if df.shape[1] >= 2:
    x_data = df.iloc[:, 0]
    y_data = df.iloc[:, 1]
else:
    raise ValueError("CSV file must contain at least two columns.")

# Define rectangle dimensions
width = 800   # set your desired width
height = 1600   # set your desired height

# Create the plot
fig, ax = plt.subplots()

# Scatter plot
ax.scatter(x_data, y_data)

# Reverse x-axis (origin on right, positive x goes left)
ax.set_xlim([width, 0])
ax.set_ylim([0, height])
ax.set_aspect('equal', adjustable='box')

# Add rectangle
rectangle = patches.Rectangle((0, 0), width, height,
                              edgecolor='red',
                              facecolor='none',
                              linewidth=2)
ax.add_patch(rectangle)

# Labels and grid
ax.set_xlabel('X-axis (positive leftward)')
ax.set_ylabel('Y-axis (positive upward)')
ax.grid(True)
plt.title('Scatter Plot with Right-to-Left X-Axis and Rectangle')

# Create annotation box
tooltip = ax.annotate('', xy=(0,0), xytext=(20,20),
                      textcoords='offset points',
                      bbox=dict(boxstyle='round', fc='yellow', alpha=0.7),
                      arrowprops=dict(arrowstyle='->'))
tooltip.set_visible(False)

# Mouse move event
def on_motion(event):
    if event.inaxes == ax and event.xdata is not None and event.ydata is not None:
        tooltip.xy = (event.xdata, event.ydata)
        tooltip.set_text(f"x: {event.xdata:.2f}, y: {event.ydata:.2f}")
        tooltip.set_visible(True)
        fig.canvas.draw_idle()
    else:
        tooltip.set_visible(False)
        fig.canvas.draw_idle()

# Connect the event to the handler
fig.canvas.mpl_connect('motion_notify_event', on_motion)

plt.show()
