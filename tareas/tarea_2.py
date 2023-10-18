import cv2
import numpy as np
import os

# Importar todos los videos e imagenes de la carpeta multimedia/tarea2
input_images_path = "./multimedia/tarea2"
files_names = os.listdir(input_images_path)
#print(files_names)
def non_max_suppression_fast(boxes, overlapThresh):
	# if there are no boxes, return an empty list
	if len(boxes) == 0:
		return []
	# if the bounding boxes integers, convert them to floats --
	# this is important since we'll be doing a bunch of divisions
	if boxes.dtype.kind == "i":
		boxes = boxes.astype("float")
	# initialize the list of picked indexes	
	pick = []
	# grab the coordinates of the bounding boxes
	x1 = boxes[:,0]
	y1 = boxes[:,1]
	x2 = boxes[:,2]
	y2 = boxes[:,3]
	# compute the area of the bounding boxes and sort the bounding
	# boxes by the bottom-right y-coordinate of the bounding box
	area = (x2 - x1 + 1) * (y2 - y1 + 1)
	idxs = np.argsort(y2)
	# keep looping while some indexes still remain in the indexes
	# list
	while len(idxs) > 0:
		# grab the last index in the indexes list and add the
		# index value to the list of picked indexes
		last = len(idxs) - 1
		i = idxs[last]
		pick.append(i)
		# find the largest (x, y) coordinates for the start of
		# the bounding box and the smallest (x, y) coordinates
		# for the end of the bounding box
		xx1 = np.maximum(x1[i], x1[idxs[:last]])
		yy1 = np.maximum(y1[i], y1[idxs[:last]])
		xx2 = np.minimum(x2[i], x2[idxs[:last]])
		yy2 = np.minimum(y2[i], y2[idxs[:last]])
		# compute the width and height of the bounding box
		w = np.maximum(0, xx2 - xx1 + 1)
		h = np.maximum(0, yy2 - yy1 + 1)
		# compute the ratio of overlap
		overlap = (w * h) / area[idxs[:last]]
		# delete all indexes from the index list that have
		idxs = np.delete(idxs, np.concatenate(([last],
			np.where(overlap > overlapThresh)[0])))
	# return only the bounding boxes that were picked using the
	# integer data type
	return boxes[pick].astype("int")


def find_rectangle(img):
    ctr = []
    # Aplicar detección de bordes usando algoritmo Canny
    lower_thr = 20
    upper_thr = 100
    edges = cv2.Canny(img, lower_thr, upper_thr)

    #_, threshold = cv2.threshold(gray, 127, 255, cv2.THRESH_BINARY) 
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
    return np.array(ctr)

def find_circle (img):
    ctr = []
    # Aplicar detección de bordes usando algoritmo Canny
    lower_thr = 20
    upper_thr = 100
    edges = cv2.Canny(img, lower_thr, upper_thr)

    #_, threshold = cv2.threshold(gray, 127, 255, cv2.THRESH_BINARY) 
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
    return np.array(ctr)


def find_triangle(img):
    ctr = []
    # Aplicar detección de bordes usando algoritmo Canny
    lower_thr = 20
    upper_thr = 100
    edges = cv2.Canny(img, lower_thr, upper_thr)

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
                ctr.append(approx)
    return np.array(ctr)

def process_s1(img):
    ctr = []
    ctr.append(find_triangle(img))
    ctr.append(find_rectangle(img))
    ctr.append(find_circle(img))
    print(np.array(ctr)[0].shape)
    return non_max_suppression_fast(np.array(ctr), 0.5)
    
# Function to process each frame
def process_frame(img):
    contours = []

    # Convertir a escala de grises
    gray_img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    # Aplicar filtro gaussiano
    blur_img = cv2.GaussianBlur(gray_img, (3, 3), 0)
    a = process_s1(blur_img)
    
    for ctr in a:
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
    path = input_images_path+'/'+'s1.mp4'

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