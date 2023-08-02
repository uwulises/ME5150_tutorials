import cv2
import numpy as np

# Open the video file
video_path = "Tareas/t2_apples.mp4"
cap = cv2.VideoCapture(video_path)

# Check if the video file was successfully opened
if not cap.isOpened():
    print("Error opening video file")

# Read the first frame
ret, frame = cap.read()

    
def frame_processing(frame):
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    lower_red1 = np.array([0, 100, 200])
    upper_red1 = np.array([12, 255, 255])
    lower_red2 = np.array([170, 100, 200])
    upper_red2 = np.array([255, 255, 255])

    mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
    mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
    maskred = cv2.bitwise_or(mask1, mask2)

    # Find contours of red objects
    contours, _ = cv2.findContours(maskred, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    # Create a bounding box for each contour
    for cnt in contours:
        area = cv2.contourArea(cnt)
        if area > 200:  # Ignore small contours
            x, y, w, h = cv2.boundingRect(cnt)
            cv2.rectangle(frame, (x, y), (x+w, y+h), (0, 255, 0), 2)
    new_frame = frame
    return new_frame

new_video = []

# Loop through all frames
while ret:
    # Process the frame here    
    new_frame = frame_processing(frame)

    # Add new frame to list of new video
    new_video.append(new_frame)

    # Display the processed frame
    cv2.imshow("Processed Frame", new_frame)

    # Wait for the 'q' key to exit
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

    # Read the next frame
    ret, frame = cap.read()

# Create video
fourcc = cv2.VideoWriter_fourcc(*'mp4v')
video = cv2.VideoWriter('Tareas/t2_output.avi', fourcc, 30, (1280, 720))

for i in range(len(new_video)):
    video.write(new_video[i])

# Release the video file and close windows
cap.release()
video.release()  
cv2.destroyAllWindows()