import numpy as np
import cv2

class ArucoHunting():
    def __init__(self):
        self.poses = None
        self.marker_length = 0.0
        self.camera_matrix = None
        self.dist_coeff = None
        self.img = None
        self.img_detection = None
        self.aruco_ids = []

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
        poses = []
        self.img_detection = np.copy(self.img)
        # Diccionario de marcadores
        _aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)

        # Carga parametros de detección
        _aruco_parameters = cv2.aruco.DetectorParameters()

        # Crea detector a partir del diccionario y los parametros
        _detector = cv2.aruco.ArucoDetector(_aruco_dict, _aruco_parameters)

        # Detecta marcadores
        marker_corners, ids, _ = _detector.detectMarkers(self.img)
        if marker_corners == ():
            return None, None
        
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
            
            aruco_pose = {'id': ids[id][0], 'position': tvec, 'orientation': rvec}
            poses.append(aruco_pose)

        self.poses = poses
        self.aruco_ids = ids
