import cv2

# Open camera
cap = cv2.VideoCapture(1)

if not cap.isOpened():
    print("Cannot open camera")
    exit()

frame_count = 0

while True:
    # Read frame
    ret, frame = cap.read()

    if not ret:
        print("Can't receive frame")
        break

    # Show frame
    cv2.imshow("USB Camera", frame)

    # Save frame every time you press 's'
    key = cv2.waitKey(1)

    if key == ord('s'):
        filename = f"frame_{frame_count}.jpg"
        cv2.imwrite(filename, frame)
        print("Saved:", filename)
        frame_count += 1

    # Quit with q
    if key == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
