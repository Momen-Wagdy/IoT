from picamera import PiCamera
from time import sleep

# Initialize the camera
camera = PiCamera()

# Start the camera preview
camera.start_preview()

# Wait for the camera to adjust to lighting conditions
sleep(2)

# Capture an image
camera.capture('/home/pi/Desktop/test_image.jpg')

# Stop the camera preview
camera.stop_preview()

# Release the camera resources
camera.close()

print("Image captured and saved to /home/pi/Desktop/test_image.jpg")
