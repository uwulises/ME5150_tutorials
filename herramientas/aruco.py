import numpy as np
import cv2

class ArucoHunting():
    def __init__(self):
        self.pose = None
        self.corners = None
        self.marker_length = 0.0
        self.camera_matrix = np.array([[0., 0., 0.],[0., 0., 0.],[0., 0., 1.]])
        self.dist_coeff = np.array([[0.,  0., 0.,  0., 0.]])
        self.img = None
        self.img_detection = None

    def update_image(self, img):
        self.img = img

    def set_camera_parameters(self, camera_matrix, dist_coeff):
        self.camera_matrix = camera_matrix
        self.dist_coeff = dist_coeff

    def set_marker_length(self, marker_length):
        self.marker_length = marker_length

    def update_pose_and_corners(self):

        self.img_detection = np.copy(self.img)
        # Diccionario de marcadores
        _aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)

        # Carga parametros de detección
        _aruco_parameters = cv2.aruco.DetectorParameters()

        # Crea detector a partir del diccionario y los parametros
        _detector = cv2.aruco.ArucoDetector(_aruco_dict, _aruco_parameters)

        # Detecta marcadores
        marker_corners, _, _ = _detector.detectMarkers(self.img)

        if marker_corners == ():
            return None, None
        
        # Crea distribución de puntos 3D de cualquier marcador (es igual para todos)
        _objPoints = np.array([[-self.marker_length/2, self.marker_length/2, 0], [self.marker_length/2, self.marker_length/2, 0],
                               [self.marker_length/2, -self.marker_length/2, 0], [-self.marker_length/2, -self.marker_length/2, 0]])
        
        for _marker_corner in marker_corners:
            # Calcula pose de cada marcador detectado
            _, rvec, tvec = cv2.solvePnP(_objPoints, _marker_corner, self.camera_matrix, self.dist_coeff, flags=cv2.SOLVEPNP_IPPE_SQUARE)
            # Dibuja ejes de cada marcador detectado
            self.img_detection = cv2.drawFrameAxes(self.img_detection, self.camera_matrix, self.dist_coeff, rvec, tvec, self.marker_length)
            self.img_detection = cv2.aruco.drawDetectedMarkers(self.img_detection, marker_corners, borderColor=(255, 0, 0))
        # Se ajustan ejes de coordenadas
        rvec = np.transpose(rvec)
        tvec = np.transpose(tvec)[0]

        self.pose = np.array([tvec, rvec[0]])
        self.corners = _marker_corner
