import cv2
import numpy as np
import matplotlib.pyplot as plt
import time

# Function to process each frame
def process_frame(img):

    # Convert the image to grayscale
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    k = 7
    # Apply Gaussian blur to the grayscale image
    blur = cv2.GaussianBlur(gray, (k, k), 0)

    lower_thr = 100
    upper_thr = 200
    # Apply Canny edge detection to the blurred image
    edges = cv2.Canny(blur, lower_thr, upper_thr)

    # Find contours in the edges image
    contours, _ = cv2.findContours(edges, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    # Draw a green bounding box around each detected contour
    for cnt in contours:
        area = cv2.contourArea(cnt)

        if area > 150:  # Ignore small contours

            approx = cv2.approxPolyDP(cnt, 0.1 * cv2.arcLength(cnt, True), True)
            if len(approx) == 3:
                cv2.drawContours(img, [cnt], 0, (0, 255, 0), 4)
            elif len(approx) == 4:
                x, y, w, h = cv2.boundingRect(cnt)
                aspect_ratio = float(w) / h
                if aspect_ratio >= 0.95 and aspect_ratio <= 1.05:
                    cv2.drawContours(img, [cnt], 0, (255, 255, 0), 4)
                else: 
                    cv2.drawContours(img, [cnt], 0, (255, 150, 0), 4)
            elif len(approx) == 5:
                cv2.drawContours(img, [cnt], 0, (0, 255, 0), 4)
            else:
                cv2.drawContours(img, [cnt], 0, (255, 0, 255), 4)

    # Display the processed frame
    cv2.imshow('Processed Frame', img)
    cv2.imshow('Canny Frame', edges)

    # Check for 'q' key press to exit
    if cv2.waitKey(1) & 0xFF == ord('q'):
        cv2.imwrite(f'Vision/results/detection_{int(time.time())}.png', img)
        return False
    
    return True

# Main function
def main():
    # Open the webcam
    cap = cv2.VideoCapture(1)

    while True:
        # Read a frame from the webcam
        ret, frame = cap.read()
        
        # Check if frame reading was successful
        if not ret:
            break
        
        # Process the frame
        if not process_frame(frame):
            break
    
    # Release the webcam and close windows
    cap.release()
    cv2.destroyAllWindows()

# Run the main function
if __name__ == '__main__':
    main()
    time.time()