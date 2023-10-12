import pybullet as p
import pybullet_data
import numpy as np
import cv2

# Initialize PyBullet
physicsClient = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())

# Set gravity and time step
p.setGravity(0, 0, -9.81)
p.setTimeStep(1 / 240)
p.setRealTimeSimulation(1)

cubeStartPos = [0, 0, 1]
cubeStartOrientation = p.getQuaternionFromEuler([0, 0, 0])
cube = p.loadURDF("modelos/objetos/cubo.urdf", basePosition=cubeStartPos, useFixedBase=True)


def traslation_transform(x, y, z):
    return np.array([[1, 0, 0, x],
                     [0, 1, 0, y],
                     [0, 0, 1, z],
                     [0, 0, 0, 1]])

def get_aruco_pose(frame):
        # Parametros cámara
        camera_matrix = np.array([[0., 0., 0.],[0., 0., 0.],[0., 0., 1.]])
        dist_coeff = np.array([[0.,  0., 0.,  0., 0.]])

        # Parametros marcador
        _marker_length = 0.028 # Ajustar a la medida real del marcador
        _aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250) # Diccionario de marcadores

        _draw_img = np.copy(frame)
        _frame_gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        
        # Carga parametros de detección
        _aruco_parameters = cv2.aruco.DetectorParameters()
        # Crea detector a partir del diccionario y los parametros
        _detector = cv2.aruco.ArucoDetector(_aruco_dict, _aruco_parameters)

        # Detecta marcadores
        marker_corners, _, _ = _detector.detectMarkers(_frame_gray)

        if marker_corners == ():
            return None
        # Crea distribución de puntos 3D de cualquier marcador (es igual para todos)
        _objPoints = np.array([[-_marker_length/2, _marker_length/2, 0], [_marker_length/2, _marker_length/2, 0],
                               [_marker_length/2, -_marker_length/2, 0], [-_marker_length/2, -_marker_length/2, 0]])
        
        for _marker_corner in marker_corners:
            # Calcula pose de cada marcador detectado
            _valid, rvec, tvec = cv2.solvePnP(_objPoints, _marker_corner, camera_matrix, dist_coeff, flags=cv2.SOLVEPNP_IPPE_SQUARE)
            # Dibuja ejes de cada marcador detectado
            _draw_img = cv2.drawFrameAxes(_draw_img, camera_matrix, dist_coeff, rvec, tvec, _marker_length)

        # Se ajustan ejes de coordenadas
        rvec = np.transpose(rvec)
        tvec = np.transpose(tvec)[0]

        # Dibuja marcadores detectados sobre imagen original
        _draw_img = cv2.aruco.drawDetectedMarkers(_draw_img, marker_corners, borderColor=(255, 0, 0))
        _draw_img = cv2.cvtColor(_draw_img, cv2.COLOR_BGR2RGB)
        #cv2.imshow('frame', _draw_img)

        # Retorna pose de marcador detectado
        pose_aruco = np.array([tvec, rvec[0]])
        return pose_aruco

def perm_columns(pos):
    n_position = np.zeros(3)
    n_position[0] = pos[2] 
    n_position[1] = pos[0]
    n_position[2] = -pos[1]
    return n_position

cap = cv2.VideoCapture(1)

while cap.isOpened():
    ret, frame = cap.read()

    pose = get_aruco_pose(frame)

    if pose is not None:
        # Permutar columnas para ajustar a PyBullet
        position = perm_columns(pose[0])
        # Convertir a cuaternión
        orientation = p.getQuaternionFromEuler(pose[1])
        # Actualizar posición y orientación del cubo
        p.resetBasePositionAndOrientation(cube, position, orientation)
        p.stepSimulation()

    p.stepSimulation()
    cv2.imshow('frame', frame)

    if cv2.waitKey(1) == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()

p.disconnect()