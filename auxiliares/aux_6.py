import cv2
import numpy as np
import matplotlib.pyplot as plt
import time

# Function to find colors
def find_yellow(img):
    ctr = []
    return ctr

def find_green(img):
    ctr = []
    return ctr

def find_pink(img):
    val_contornos = []
    # Convertir imagen a espacio de color HSV
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

    # Definir rangos de colores
    lower = (151, 78, 156) # np.array([90, 100, 160])
    upper = (178, 236, 255) # np.array([120, 255, 255])

    # Generar máscara a partir de rango de colores
    mask = cv2.inRange(hsv, lower, upper)
    # cv2.imshow('Mask', mask)
    # cv2.waitKey(1)
    
    # Dilatar la máscara
    dilated = cv2.dilate(mask, None, iterations = 2)
    # cv2.imshow('Dilated mask', dilated)
    # cv2.waitKey(1)

    # Aplicar detección de bordes usando algoritmo Canny
    lower_thr = 100
    upper_thr = 200
    edges = cv2.Canny(dilated, lower_thr, upper_thr)

    # cv2.imshow('Edges', edges)
    # cv2.waitKey(1)

    # Encontrar contornos en la imagen de bordes
    contours, _ = cv2.findContours(edges, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    ctr_img = img.copy()

    for cnt in contours:
        area = cv2.contourArea(cnt)
        if area > 200:  # Ignore small contours
            ctr_img = cv2.drawContours(ctr_img, [cnt], contourIdx = 0, color = (0, 255, 0), thickness = 3)
            val_contornos.append(cnt)

    # cv2.imshow('Contours', ctr_img)
    # waitKey(1)

    return val_contornos

# Functions to find shapes
def find_triangle(img):
    # Lista para almacenar contornos válidos
    valid_ctr = []

    # Convertir imagen a escala de grises
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    
    # Aplicar filtro Gaussiano a la imagen, con un kernel a definir
    k = 9
    blur = cv2.GaussianBlur(gray, (k, k), 0)

    # Aplicar detección de bordes usando algoritmo Canny
    lower_thr = 20
    upper_thr = 100
    edges = cv2.Canny(blur, lower_thr, upper_thr)
    #_, threshold = cv2.threshold(gray, 127, 255, cv2.THRESH_BINARY) 
    
    # Encontrar contornos en la imagen de bordes
    contours, _ = cv2.findContours(edges, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    # Para cada contorno
    for cnt in contours:
        # Calcular el area del contorno
        area = cv2.contourArea(cnt)

        if area > 150:  # Ignore small contours
            perimeter = cv2.arcLength(cnt, closed = True)
            approx = cv2.approxPolyDP(cnt, epsilon = 0.05 * perimeter, closed = True)

            if len(approx) == 3:
                valid_ctr.append(approx)

    return valid_ctr

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
    contours+=find_pink(img)
    # contours+=find_triangle(img)
    # contours+=find_yellow(img)

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