import cv2
import numpy as np
import os

# Importar todos los videos e imagenes de la carpeta multimedia/tarea2
input_images_path = "./multimedia/tarea2"
files_names = os.listdir(input_images_path)
#print('Archivos cargados', files_names)

def find_rectangle(img):
    ctr = []
    
    # Convertir imagen a espacio de color HSV
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

    # Definir rangos de colores
    lower = (36, 69, 91)
    upper = (100, 255, 255)

    # Generar máscara a partir de rango de colores
    mask = cv2.inRange(hsv, lower, upper)
    
    # Dilate
    kernel = np.ones((5,5),np.uint8)
    mask = cv2.dilate(mask, kernel, iterations = 1)

    # Aplicar detección de bordes usando algoritmo Canny
    lower_thr = 20
    upper_thr = 100
    edges = cv2.Canny(mask, lower_thr, upper_thr)

    #blur_edges = cv2.GaussianBlur(edges, (3, 3), 0)
    # Encontrar contornos en la imagen de bordes
    contours, _ = cv2.findContours(edges, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    # Para cada contorno
    for cnt in contours:
        # Calcular el area del contorno
        area = cv2.contourArea(cnt)        
        if area > 5000:  # Ignore small contours
            perimeter = cv2.arcLength(cnt, closed = True)
            approx = cv2.approxPolyDP(cnt, epsilon = 0.01 * perimeter, closed = True)
            if len(approx) == 4:
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
    lower = (0, 0, 0)
    upper = (179, 172, 152)

    # Generar máscara a partir de rango de colores
    mask = cv2.inRange(hsv, lower, upper)

    # Encontrar contornos en la imagen de bordes
    contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    # Para cada contorno
    for cnt in contours:
        # Calcular el area del contorno
        area = cv2.contourArea(cnt)        
        if area > 5000:  # Ignore small contours
            perimeter = cv2.arcLength(cnt, closed = True)
            approx = cv2.approxPolyDP(cnt, epsilon = 0.015 * perimeter, closed = True)
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

def find_shapes (img):
    ctr = []
    gray_img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    threshold = 170
    thres_img = cv2.threshold(gray_img, threshold, 255, cv2.THRESH_BINARY)[1]

    
    # Aplicar detección de bordes usando algoritmo Canny
    lower_thr = 20
    upper_thr = 100
    edges = cv2.Canny(thres_img, lower_thr, upper_thr)

    # Encontrar contornos en la imagen de bordes
    contours, _ = cv2.findContours(edges, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    # Para cada contorno
    for cnt in contours:
        # Calcular el area del contorno
        area = cv2.contourArea(cnt)        
        if area > 150:  # Ignore small contours
            perimeter = cv2.arcLength(cnt, closed = True)
            approx = np.array(cv2.approxPolyDP(cnt, epsilon = 0.02 * perimeter, closed = True))
            if len(approx) == 3:
                ctr.append(approx)
            elif len(approx) == 4:
                ctr.append(approx)
            approx2 = np.array(cv2.approxPolyDP(cnt, epsilon = 0.005 * perimeter, closed = True))
            if len(approx2) >= 10:
                ctr.append(cnt)

    return  ctr
def process_s1(img):
    """
    Detectar triangulos, cuadrados, rectangulos y circulos.
    Sugerencia: usar threshold sobre la imagen original,
                para encontrar los circulos generar un approxPolyDP más estricto.
    """
    ctr = []
    #ctr.append(find_triangle(img))
    #ctr.append(find_rectangle(img))
    #ctr.append(find_circle(img))
    ctr.append(find_shapes(img))
    
    return ctr

def process_s2(img):
    """
    Detectar pentagono y cuadrado exterior.
    Sugerencia: filtrar por color y area para encontrar el pentágono,
                filtrar por color y dilatar para encontrar el cuadrado.

    """
    ctr = []
    ctr.append(find_pentagon(img))
    ctr.append(find_rectangle(img))
    return ctr

def process_s4(img):
    """
    Detectar cuadrado gris
    """
    ctr = []
    ctr.append(find_rectangle(img))
    return ctr


def process_s6(img):
    """
    Detectar cuadrado
    Sugerencia: usar threshold sobre la imagen.
    """
    #find black square in the image, binarising the image
    #convert to binary
    ctr = []
    #cv2.imshow("mask", res)
    gray_img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    #erode image
    gray_img = cv2.erode(gray_img, (21,21)) 
    #use gaussian blur
    gray_img = cv2.GaussianBlur(gray_img, (9,9), 0)
    ret, thresh = cv2.threshold(gray_img, 100, 255, cv2.THRESH_BINARY_INV)
    edges = cv2.Canny(thresh, 590,600)
    contorno, hierarchy = cv2.findContours(edges, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    ctr.append(contorno)

    return ctr

def process_s7(img):
    """
    Detectar circulo interno de la cinta
    """
    ctr = []
    return ctr

# Function to process each frame
def process_frame(img, process):
    img = cv2.resize(img, (640, 480))
    contours = []
    for ctr in process(img): # Cambiar según que video se está analizando
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

    """
    PASO 0: Cambiar nombre del video o imagen a procesar
    """
    name_file = 's2.mp4'

    path = input_images_path + '/' + name_file

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

            # Generar nombre de la función a ejecutar
            name_process = 'process_' + name_file.split('.')[0]
            process = eval(name_process)

            # Procesar frame
            new_frame = process_frame(frame, process)
            
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