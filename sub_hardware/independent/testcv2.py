import cv2
import numpy as np
import sys

print(f"--- OpenCV Test Script ---")
print(f"Python version: {sys.version}")
print(f"OpenCV version: {cv2.__version__}")

# Create a blank black image
# The data type must be unsigned 8-bit integer
image = np.zeros((480, 640, 3), dtype=np.uint8)

# Write some text on the image
cv2.putText(
    image,
    'If you see this, OpenCV is working!',
    (20, 240),                # Position
    cv2.FONT_HERSHEY_SIMPLEX, # Font
    1,                        # Scale
    (0, 255, 0),              # Color (Green)
    2                         # Thickness
)

print("--> Calling cv2.imshow(). A window should appear.")
cv2.imshow('OpenCV Test Window', image)

print("--> Calling cv2.waitKey(0). Press any key in the window to quit.")
# waitKey(0) waits indefinitely for a key press.
# This is CRITICAL for drawing the window and handling events.
cv2.waitKey(0)

print("--> Window closed. Cleaning up.")
cv2.destroyAllWindows()
print("--- Test Finished ---")
