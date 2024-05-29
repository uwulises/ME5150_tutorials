import numpy as np
import cv2

class ArucoHunting():
    def __init__(self):
        self.arucos_data = []
        self.marker_length = 0.0
        self.camera_matrix = None
        self.dist_coeff = None
        self.img = None
        self.img_detection = None

    def update_image(self, img):
        self.img = img

    def set_camera_parameters(self, 
                    camera_matrix,
                    dist_coeff = [0., 0., 0., 0., 0.]):

        assert camera_matrix.shape == (3, 3), "La matriz de la cámara debe ser de 3x3"
        self.camera_matrix = camera_matrix.astype(np.float32)
        self.dist_coeff = np.array(dist_coeff)

    def set_marker_length(self, marker_length):
        assert marker_length > 0, "El largo del marcador debe ser mayor a 0 y se mide en metros"
        self.marker_length = marker_length

    def update_pose_and_ids(self):
        assert self.marker_length != 0, "Debe definir el largo del marcador"
        assert self.camera_matrix is not None, "Debe definir los parámetros de la cámara"
        assert self.img is not None, "Debe definir la imagen"
        arucos_data = []

        
        # Diccionario de marcadores
        _aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)

        # Carga parametros de detección
        _aruco_parameters = cv2.aruco.DetectorParameters()

        # Crea detector a partir del diccionario y los parametros
        _detector = cv2.aruco.ArucoDetector(_aruco_dict, _aruco_parameters)

        # Detecta marcadores
        marker_corners, ids, _ = _detector.detectMarkers(self.img)

        self.img_detection = np.copy(self.img)
        if marker_corners == ():
            pass
        
        # Crea distribución de puntos 3D de cualquier marcador (es igual para todos)
        _objPoints = np.array([[-self.marker_length/2, self.marker_length/2, 0], [self.marker_length/2, self.marker_length/2, 0],
                               [self.marker_length/2, -self.marker_length/2, 0], [-self.marker_length/2, -self.marker_length/2, 0]])
        
        for id, _marker_corner in enumerate(marker_corners):
            # Calcula pose de cada marcador detectado
            _, rvec, tvec = cv2.solvePnP(_objPoints, _marker_corner, self.camera_matrix, self.dist_coeff, flags=cv2.SOLVEPNP_IPPE_SQUARE)
            
            # Dibuja ejes de cada marcador detectado
            self.img_detection = cv2.drawFrameAxes(self.img_detection, self.camera_matrix, self.dist_coeff, rvec, tvec, self.marker_length)
            self.img_detection = cv2.aruco.drawDetectedMarkers(self.img_detection, marker_corners, borderColor=(255, 0, 0))
            
            # Se ajustan ejes de coordenadas
            tvec = np.transpose(tvec)[0]
            rvec = np.transpose(rvec)[0]
            
            aruco_data = {'id': ids[id][0], 'position': tvec, 'orientation': rvec}
            arucos_data.append(aruco_data)

        self.arucos_data = arucos_data

def main():
    cap = cv2.VideoCapture(0)
    hunter = ArucoHunting()
    camera_matrix = np.array([[691., 0. , 289.],[0., 690., 264.], [0., 0., 1.]]) 
    hunter.set_marker_length(0.06)
    hunter.set_camera_parameters(camera_matrix)

    while True:
        ret, frame = cap.read()

        if not ret:
            print("Error al abrir la cámara")
            return

        hunter.update_image(frame)
        hunter.update_pose_and_ids()

        if hunter.arucos_data != []:
            print(hunter.arucos_data)

        cv2.imshow('frame', hunter.img_detection)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()

if __name__ == "__main__":
    main()

    
    
