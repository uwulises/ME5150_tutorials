import cv2
import numpy as np

# Callback function for slider changes
def on_slider_change(value):
    # Retrieve current slider values
    h = cv2.getTrackbarPos('H', 'Sliders Window')
    s = cv2.getTrackbarPos('S', 'Sliders Window')
    v = cv2.getTrackbarPos('V', 'Sliders Window')

# Create a window to display the sliders
cv2.namedWindow('Sliders Window')

# Create three sliders
cv2.createTrackbar('H', 'Sliders Window', 0, 179, on_slider_change)
cv2.createTrackbar('S', 'Sliders Window', 0, 255, on_slider_change)
cv2.createTrackbar('V', 'Sliders Window', 0, 255, on_slider_change)

# Create a blank image
image = np.zeros((200, 200, 3), dtype=np.uint8)

# Keep updating the window until the 'Esc' key is pressed
while True:
    # Get the current slider values
    h = cv2.getTrackbarPos('H', 'Sliders Window')
    s = cv2.getTrackbarPos('S', 'Sliders Window')
    v = cv2.getTrackbarPos('V', 'Sliders Window')

    # Set the color in HSV
    color_hsv = np.array([[[h, s, v]]], dtype=np.uint8)
    color_bgr = cv2.cvtColor(color_hsv, cv2.COLOR_HSV2BGR)
    color_bgr = tuple(map(int, color_bgr[0, 0]))

    # Fill the image with the specified color
    image[:] = color_bgr

    # Display the image in the window
    cv2.imshow('Sliders Window', image)

    # Wait for the 'Esc' key to be pressed
    if cv2.waitKey(1) == 27:
        break

# Close the window and release resources
cv2.destroyAllWindows()