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

    # Aplicar dilatación a imagen hsv, con un kernel a definir
    dilated_image = np.zeros(hsv.shape)

    # Definir colores límites (inferior y superior) para la máscara
    lower_blue = np.array([0, 0, 0])
    upper_blue = np.array([180, 255, 255])

    # Generar máscara a partir de rango de colores
    mask_blue = cv2.inRange(dilated_image, lower_blue, upper_blue)

    # Encontrar contornos de pitufos en la máscara
    contours = []

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
video = cv2.VideoWriter('Auxiliares/det_pitufos.avi', fourcc, 30, (1280, 720))

for i in range(len(new_video)):
    video.write(new_video[i])

# Release the video file and close windows
cap.release()
video.release()  
cv2.destroyAllWindows()