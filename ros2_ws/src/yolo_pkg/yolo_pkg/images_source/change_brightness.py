import os
import random
from tkinter import Tk, filedialog
from PIL import Image, ImageEnhance

def choose_folder():
    root = Tk()
    root.withdraw()  # Hide the main tkinter window
    folder_path = filedialog.askdirectory(title="Select Folder Containing Images")
    return folder_path

def adjust_brightness(image, factor):
    enhancer = ImageEnhance.Brightness(image)
    return enhancer.enhance(factor)

def process_images(folder_path):
    if not folder_path:
        print("No folder selected.")
        return

    output_folder = os.path.join(folder_path, "new images")
    os.makedirs(output_folder, exist_ok=True)

    supported_extensions = (".jpg", ".jpeg", ".png")

    for filename in os.listdir(folder_path):
        if filename.lower().endswith(supported_extensions):
            image_path = os.path.join(folder_path, filename)

            try:
                img = Image.open(image_path)
                # Random brightness factor between 0.9 and 1.1 (±10%)
                brightness_factor = random.uniform(0.9, 1.1)
                img_adjusted = adjust_brightness(img, brightness_factor)

                # Save with the same name into 'new images' folder
                new_image_path = os.path.join(output_folder, os.path.splitext(filename)[0] + ".jpg")
                img_adjusted.save(new_image_path, "JPEG")

                print(f"Processed and saved: {new_image_path}")

            except Exception as e:
                print(f"Failed to process {filename}: {e}")

if __name__ == "__main__":
    folder = choose_folder()
    process_images(folder)
