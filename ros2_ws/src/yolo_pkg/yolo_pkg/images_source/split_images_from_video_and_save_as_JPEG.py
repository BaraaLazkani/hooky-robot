import cv2
import os

# Configuration
video_path = '/home/jaafar/htr/srs/yolo_pkg/yolo_pkg/images_source/videos_taken_with_ditection/1.avi'
output_folder = '/home/jaafar/htr/srs/yolo_pkg/yolo_pkg/images_source/splited_image_from_video/images_with_ditection'
os.makedirs(output_folder, exist_ok=True)

# Open the video
cap = cv2.VideoCapture(video_path)
fps = cap.get(cv2.CAP_PROP_FPS)
print(f"Video FPS: {fps}")

frame_count = 0
while True:
    # Read a frame
    ret, frame = cap.read()
    
    # If frame is read correctly, ret is True
    if not ret:
        print("Can't receive frame (stream end?). Exiting...")
        break
    
    # Save frame as JPEG file
    frame_filename = os.path.join(output_folder, f"frame_{frame_count:05d}.jpeg")
    cv2.imwrite(frame_filename, frame)
    print(f"Saved {frame_filename}")
    
    frame_count += 1

# Release everything
cap.release()
print(f"Extracted {frame_count} frames")
