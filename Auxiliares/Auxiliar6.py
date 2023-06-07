import cv2
import numpy as np

# Open the video file
video_path = "Vision/pitufos.mp4"
cap = cv2.VideoCapture(video_path)

# Check if the video file was successfully opened
if not cap.isOpened():
    print("Error opening video file")

# Read the first frame
ret, frame = cap.read()

    
def frame_processing(frame):
    # Transformación de BGR a HSV
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    #lower_blue = np.array([90, 100, 160])
    #upper_blue = np.array([120, 255, 255])
    lower_blue = (90, 100, 160)
    upper_blue = (120, 255, 255)

    # Aplicar dilatación a imagen hsv, con un kernel a definir
    k = 13 
    
    # Define the kernel size and shape for dilatation
    kernel = np.ones((k, k), np.uint8)

    # Perform dilatation on the image
    dilated_image = cv2.dilate(hsv, kernel, iterations=1)

    # Generar máscara a partir de rango de colores
    mask_blue = cv2.inRange(dilated_image, lower_blue, upper_blue)

    # Encontrar contornos de pitufos en la máscara
    contours, _ = cv2.findContours(mask_blue, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    # Crear bounding boxes rectangulares para áreas de cierto tamaño
    for cnt in contours:
        area = cv2.contourArea(cnt)
        if area > 500:  # Ignore small contours
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
video = cv2.VideoWriter('Auxiliares/det_pitufos.mp4',fourcc, 25.0, (1920, 1080))

for i in range(len(new_video)):
    video.write(new_video[i])

# Release the video file and close windows
cap.release()
video.release()  
cv2.destroyAllWindows()