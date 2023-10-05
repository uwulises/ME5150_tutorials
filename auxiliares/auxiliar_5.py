import cv2
import numpy as np
import matplotlib.pyplot as plt
import time

def find_yellow(img):
    val_contornos = []
    
    # Convertir imagen a espacio de color HSV
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

    # Definir rangos de colores (esto no es amarillo)
    lower = (90, 100, 160) # np.array([90, 100, 160])
    upper = (120, 255, 255) # np.array([120, 255, 255])

    # Generar máscara a partir de rango de colores
    mask = cv2.inRange(hsv, lower, upper)

    # Aplicar detección de bordes usnado algoritmo Canny
    lower_thr = 100
    upper_thr = 200
    edges = cv2.Canny(mask, lower_thr, upper_thr)

    # Encontrar contornos en la imagen de bordes
    contours, _ = cv2.findContours(edges, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    if len(contours) > 0:
        # Para cada contorno
        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area > 150:  # Ignore small contours
                val_contornos.append(cnt)

    return val_contornos

# Function to find colors
def find_green(img):
    ctr = []
    return ctr

def find_red(img):
    ctr = []
    return ctr

# Functions to find shapes
def find_triangle(img):
    val_contornos = []

    # Convertir imagen a escala de grises
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    # Aplicar filtro Gaussiano a la imagen, con un kernel a definir
    k = 5
    blur = cv2.GaussianBlur(gray, (k, k), 0)

    # Aplicar detección de bordes usando algoritmo Canny
    lower_thr = 100
    upper_thr = 200
    edges = cv2.Canny(blur, lower_thr, upper_thr)

    # Encontrar contornos en la imagen de bordes
    contours, _ = cv2.findContours(edges, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    # Para cada contorno
    for cnt in contours:
        # Calcular el area del contorno
        area = cv2.contourArea(cnt)
        if area > 150:  # Ignore small contours
            approx = cv2.approxPolyDP(cnt, epsilon = 0.05 * cv2.arcLength(cnt, closed = True), closed = True)
            if len(approx) == 3:
                val_contornos.append(cnt)
    return val_contornos

def find_circle(img):
    ctr = []
    return ctr

def find_square(img):
    ctr = []
    return ctr

def find_rectangle(img):
    ctr = []
    return ctr


# Function to process each frame
def process_frame(img):
    
    contours = []
    contours+=find_triangle(img)
    contours+=find_yellow(img)

    # Dibujar los contornos en la imagen
    if len(contours) > 0:
        for cnt in contours:
            cv2.drawContours(img, [cnt], contourIdx = 0, color = (0, 255, 0), thickness = 3)

    # Display the processed frame
    cv2.imshow('Processed Frame', img)

    # Check for 'q' key press to exit
    if cv2.waitKey(1) & 0xFF == ord('q'):
        cv2.imwrite(f'multimedia/detection_{int(time.time())}.png', img)
        return False
    
    return True

# Main function
def main():
    # Open the webcam
    cap = cv2.VideoCapture(0)

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