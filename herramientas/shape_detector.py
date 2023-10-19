import numpy as np
import cv2

class ShapeDetector():
    def __init__(self):
        self.img = None
        self.blur = None
        self.img_detection = None
        self.corners = None
        self.contours = None
    
    def update_image(self, img):
        self.img = img
        self.img_detection = img

    def preprocess_img(self):
        # Convertir imagen a escala de grises
        gray = cv2.cvtColor(self.img, cv2.COLOR_BGR2GRAY)
        
        # Aplicar filtro Gaussiano a la imagen, con un kernel a definir
        k = 7
        self.blur = cv2.GaussianBlur(gray, (k, k), 0)

    def find_contours(self, lower_thr = 20, upper_thr = 100):
        
        # Aplicar detección de bordes usando algoritmo Canny
        edges = cv2.Canny(self.blur, lower_thr, upper_thr)

        # Encontrar contornos en la imagen de bordes
        self.contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    def detect_by_shape(self):
        self.preprocess_img()
        self.find_contours()

    def find_triangle(self):
        self.detect_by_shape()

        # Para cada contorno
        for cnt in self.contours:
            # Calcular el area del contorno
            area = cv2.contourArea(cnt)
            
            if area > 150:  # Ignore small contours
                perimeter = cv2.arcLength(cnt, closed = True)
                approx = cv2.approxPolyDP(cnt, epsilon = 0.05 * perimeter, closed = True)

                if len(approx) == 3:
                    x, y, w, h = cv2.boundingRect(approx) 
                    
                    self.img_detection = cv2.rectangle(self.img, (x, y), (x + w, y + h), (255, 255, 255), 2) 
                    M = cv2.moments(cnt)

                    if M['m00'] != 0.0:
                        x = int(M['m10']/M['m00'])
                        y = int(M['m01']/M['m00'])

    def find_rectangle(self):
        x, y = (0, 0)
        self.detect_by_shape()

        # Para cada contorno
        for cnt in self.contours:

            # Calcular el area del contorno
            area = cv2.contourArea(cnt)
            approx = cv2.approxPolyDP(cnt, 0.05 * cv2.arcLength(cnt, True), True)

            if area > 150:  # Ignore small contours
                if len(approx) == 4:
                    x, y, w, h = cv2.boundingRect(approx) 
                    
                    self.img_detection = cv2.rectangle(self.img, (x, y), (x + w, y + h), (255, 255, 255), 2) 
                    M = cv2.moments(cnt)

                    if M['m00'] != 0.0:
                        x = int(M['m10']/M['m00'])
                        y = int(M['m01']/M['m00'])
        return (x, y)

            

    