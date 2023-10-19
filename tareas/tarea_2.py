import cv2
import numpy as np
import os

# Importar todos los videos e imagenes de la carpeta multimedia/tarea2
input_images_path = "./multimedia/tarea2"
files_names = os.listdir(input_images_path)
#print('Archivos cargados', files_names)

def find_rectangle(img):
    ctr = []
    
    # Convertir a escala de grises
    gray_img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    # Aplicar filtro gaussiano
    blur_img = cv2.GaussianBlur(gray_img, (5,5), 0)
    
    # Aplicar detección de bordes usando algoritmo Canny
    lower_thr = 20
    upper_thr = 100
    edges = cv2.Canny(img, lower_thr, upper_thr)

    # Encontrar contornos en la imagen de bordes
    contours, _ = cv2.findContours(edges, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    # Para cada contorno
    for cnt in contours:
        # Calcular el area del contorno
        area = cv2.contourArea(cnt)        
        if area > 150:  # Ignore small contours
            perimeter = cv2.arcLength(cnt, closed = True)
            approx = cv2.approxPolyDP(cnt, epsilon = 0.01 * perimeter, closed = True)
            if len(approx) == 4 and area<1000000:
                ctr.append(approx)
    return ctr

def find_circle (img):
    ctr = []
    
    # Convertir a escala de grises
    gray_img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    # Aplicar filtro gaussiano
    blur_img = cv2.GaussianBlur(gray_img, (3, 3), 0)
    
    # Aplicar detección de bordes usando algoritmo Canny
    lower_thr = 20
    upper_thr = 100
    edges = cv2.Canny(img, lower_thr, upper_thr)

    # Encontrar contornos en la imagen de bordes
    contours, _ = cv2.findContours(edges, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    # Para cada contorno
    for cnt in contours:
        # Calcular el area del contorno
        area = cv2.contourArea(cnt)        
        if area > 150:  # Ignore small contours
            perimeter = cv2.arcLength(cnt, closed = True)
            approx = cv2.approxPolyDP(cnt, epsilon = 0.01 * perimeter, closed = True)
            if len(approx) >= 17:
                ctr.append(approx)
    return ctr

def find_pentagon (img):
    ctr = []
    
   # Convertir imagen a espacio de color HSV
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

    # Definir rangos de colores
    lower = (0, 0, 0) # np.array([90, 100, 160])
    upper = (179, 172, 152) # np.array([120, 255, 255])

    # Generar máscara a partir de rango de colores
    mask = cv2.inRange(hsv, lower, upper)

    # Encontrar contornos en la imagen de bordes
    contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    # Para cada contorno
    for cnt in contours:
        # Calcular el area del contorno
        area = cv2.contourArea(cnt)        
        if area > 150:  # Ignore small contours
            perimeter = cv2.arcLength(cnt, closed = True)
            approx = cv2.approxPolyDP(cnt, epsilon = 0.01 * perimeter, closed = True)
            if len(approx) == 5:
                ctr.append(approx)
    return ctr

def find_triangle(img):
    ctr = []
    
    # Convertir a escala de grises
    gray_img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    # Aplicar filtro gaussiano
    blur_img = cv2.GaussianBlur(gray_img, (3, 3), 0)
    
    # Aplicar detección de bordes usando algoritmo Canny
    lower_thr = 20
    upper_thr = 100
    edges = cv2.Canny(img, lower_thr, upper_thr)

    # Encontrar contornos en la imagen de bordes
    contours, _ = cv2.findContours(edges, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    # Para cada contorno
    for cnt in contours:
        # Calcular el area del contorno
        area = cv2.contourArea(cnt)        
        if area > 150:  # Ignore small contours
            perimeter = cv2.arcLength(cnt, closed = True)
            approx = np.array(cv2.approxPolyDP(cnt, epsilon = 0.09 * perimeter, closed = True))
            if len(approx) == 3:
                ctr.append(approx)
    return ctr

def process_s1(img):
    """
    Detectar triangulos, cuadrados, rectangulos y circulos, diferenciando entre cuadrados y rectangulos
    """
    ctr = []
    ctr.append(find_triangle(img))
    #ctr.append(find_rectangle(img))
    #ctr.append(find_circle(img))
    return ctr

def process_s2(img):
    """
    Detectar pentagono, cuadrado exterior y circulo
    Sugerencia: filtrar colores para encontrar el pentágono

    """
    ctr = []
    ctr.append(find_pentagon(img))
    return ctr

def process_s3(img):
    """
    Detectar cuadrado amarillo y rectangulo morado
    """
    ctr = []
    ctr.append(find_rectangle(img))
    return ctr

def process_s4(img):
    """
    Detectar cuadrado amarillo y rectangulo morado
    """
    ctr = []
    ctr.append(find_rectangle(img))
    return ctr
    
# Function to process each frame
def process_frame(img):

    contours = []
    for ctr in process_s2(img): # Cambiar según que video se está analizando
        contours+=ctr

    # Dibujar los contornos en la imagen
    if len(contours) > 0:
        for cnt in contours:
            cv2.drawContours(img, [cnt], contourIdx = 0, color = (0, 255, 0), thickness = 3)

    # Display the processed frame
    cv2.imshow('Processed Frame', img)
    cv2.waitKey(1)
    
    return img

# Main function
def main():
    # Cambiar nombre del video o imagen a procesar
    path = input_images_path+'/'+'s2.mp4'

    # Si es un video
    if path.split('.')[-1] == 'mp4':
        new_video = []

        # Open the webcam
        cap = cv2.VideoCapture(path)

        if not cap.isOpened():
            print("Error opening video file")

        while True:
            # Leer frame del video
            ret, frame = cap.read()
            # Revisar si el frame es válido
            if not ret:
                break
            
            # Procesar frame
            new_frame = process_frame(frame)
            
            # Añadir frame a la lista
            new_video.append(new_frame)

            # Cuando ya no queden frames
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

        # Crear video
        img_shape = np.flip(new_video[0].shape[:2])
        name_video = '.' + path.split('.')[-2] + '_proc.avi'
        fourcc = cv2.VideoWriter_fourcc(*'mp4v')
        video = cv2.VideoWriter(name_video, fourcc, 30, img_shape)

        for i in range(len(new_video)):
            video.write(new_video[i])

        # Release the video file and close windows
        cap.release()
        video.release()  

    # Si es una imagen
    elif path.split('.')[-1] == 'jpg':
        cap = cv2.imread(path)
        new_video = process_frame(cap)
    
    cv2.destroyAllWindows()

# Run the main function
if __name__ == '__main__':
    main()