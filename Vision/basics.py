import cv2
import numpy as np

# Load the image
img = cv2.imread('Vision/shapes.jpg')

print(img)


# Convert the image to grayscale
gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

# Apply Gaussian blur to the grayscale image
blur = cv2.GaussianBlur(gray, (7, 7), 0)

# Apply Canny edge detection to the blurred image
edges = cv2.Canny(blur, 100, 200)

cv2.imshow('shapes', edges)
cv2.waitKey(0)

# Find contours in the edges image
contours, hierarchy = cv2.findContours(edges, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

# Draw a green bounding box around each detected contour
for cnt in contours:
    area = cv2.contourArea(cnt)
    if area > 150:  # Ignore small contours
        approx = cv2.approxPolyDP(cnt, 0.02 * cv2.arcLength(cnt, True), True)
        if len(approx) == 3:
            cv2.drawContours(img, [cnt], 0, (0, 255, 0), 2)
        elif len(approx) == 4:
            x, y, w, h = cv2.boundingRect(cnt)
            aspect_ratio = float(w) / h
            if aspect_ratio >= 0.95 and aspect_ratio <= 1.05:
                cv2.drawContours(img, [cnt], 0, (255, 255, 0), 2)
            else: 
                cv2.drawContours(img, [cnt], 0, (255, 150, 0), 2)
        elif len(approx) == 5:
            cv2.drawContours(img, [cnt], 0, (0, 255, 0), 2)
        else:
            cv2.drawContours(img, [cnt], 0, (255, 0, 255), 2)

# displaying the image after drawing contours
cv2.imshow('shapes', img)
  
cv2.waitKey(0)
cv2.destroyAllWindows()