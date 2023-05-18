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


def frame_processing(img):
        #------------  ADD YOUR CODE HERE   -----------
        new_frame = np.zeros([720,1280,3])
        

        #------------ END -----------
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