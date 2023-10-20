import cv2
import numpy as np
import os

# Importar todos los videos e imagenes de la carpeta multimedia/tarea2
input_images_path = "./multimedia/tarea2"
files_names = os.listdir(input_images_path)
#print('Archivos cargados', files_names)

def process_s1(img):
    """
    Detectar triangulos, cuadrados, rectangulos y circulos.
    Sugerencia: usar threshold sobre la imagen original,
                para encontrar los circulos generar un approxPolyDP más estricto.
    """
    ctr = []
    return ctr

def process_s2(img):
    """
    Detectar pentagono y cuadrado exterior.
    Sugerencia: filtrar por color y area para encontrar el pentágono,
                filtrar por color y dilatar para encontrar el cuadrado.

    """
    ctr = []
    return ctr

def process_s3(img):
    """
    Detectar cuadrado gris
    Sugerencia: filtrar por color, erosionar y dilatar.
    """
    ctr = []
    return ctr

def process_s4(img):
    """
    Detectar cuadrado
    Sugerencia: usar threshold sobre la imagen.
    """
    ctr = []
    return ctr

def process_all(img):
    """
    
    """
    ctr = []
    return ctr

# Function to process each frame
def process_frame(img, process):
    # Redimencionar el frame
    img = cv2.resize(img, (640, 480))
    
    # Recolectar contornos del procesamiento del frame
    contours = []
    for ctr in process(img):
        contours+=ctr

    # Dibujar los contornos en la imagen original
    if len(contours) > 0:
        for cnt in contours:
            cv2.drawContours(img, [cnt], contourIdx = 0, color = (0, 255, 0), thickness = 3)

    # Mostrar el frame procesado
    cv2.imshow('Processed Frame', img)
    cv2.waitKey(1)
    
    return img

# Main function
def main():

    """
    PASO 0: Cambiar nombre del video o imagen a procesar
    """
    name_file = 'all.jpg'

    # Generar el path completo del video o imagen
    path = input_images_path + '/' + name_file

    # Generar nombre de la función a ejecutar
    name_process = 'process_' + name_file.split('.')[0]
    process = eval(name_process)

    # Si es un video
    if path.split('.')[-1] == 'mp4':
        new_video = []

        # Abrir el video
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
        # Abrir la imagen
        cap = cv2.imread(path)

        # Procesar la imagen
        new_img = process_frame(cap, process)

        # Guardar la imagen
        name_img = '.' + path.split('.')[-2] + '_proc.jpg'
        cv2.imwrite(name_img, new_img)
    
    cv2.destroyAllWindows()

# Run the main function
if __name__ == '__main__':
    main()