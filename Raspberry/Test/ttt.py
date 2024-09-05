import cv2

# Initialize the webcam (0 is the default camera)
# cap = cv2.VideoCapture(10)
cap = cv2.VideoCapture("/dev/media0")

# Check if the webcam is opened correctly
if not cap.isOpened():
    print("Error: Could not open webcam")
    exit()

# Capture a single frame
ret, frame = cap.read()

# Check if the frame was captured correctly
if ret:
    # Save the captured image to a file
    cv2.imwrite("captured_image.jpg", frame)
    print("Image saved as 'captured_image.jpg'")
else:
    print("Error: Could not capture image")

# Release the webcam
cap.release()

# Close all OpenCV windows (if any were opened)
cv2.destroyAllWindows()
