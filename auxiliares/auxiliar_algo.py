import pybullet as p
import pybullet_data
import numpy as np
import cv2

# Initialize PyBullet
physicsClient = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())

# Set gravity and time step
p.setGravity(0, 0, -9.81)
p.setTimeStep(1 / 240) # Para el dron se recomienda trabajar con stepSimulation
p.setRealTimeSimulation(1) # 0 Desactiva la simulacion en tiempo real

# Cargar ambiente de simulacion
#planeId = p.loadURDF("plane.urdf")
cubeStartPos = [0, 0, 1]
cubeStartOrientation = p.getQuaternionFromEuler([0, 0, 0])
cube = p.loadURDF("modelos/objetos/cubo.urdf", basePosition=cubeStartPos, useFixedBase=True)


def traslation_transform(x, y, z):
    return np.array([[1, 0, 0, x],
                     [0, 1, 0, y],
                     [0, 0, 1, z],
                     [0, 0, 0, 1]])

def get_aruco_pose(frame):
        # Parametros camara
        camera_matrix = np.array([[1080., 0., 290.],[0., 1072., 250.],[0., 0., 1.]])
        dist_coeff = np.array([[-1.125,  7.71, -0.044,  0.0143, -4.105]])

        _draw_img = np.copy(frame)
        _frame_gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        _aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
        _aruco_parameters = cv2.aruco.DetectorParameters()
        _detector = cv2.aruco.ArucoDetector(_aruco_dict, _aruco_parameters)

        marker_corners, _, _ = _detector.detectMarkers(_frame_gray)

        if marker_corners == ():
            return None
        
        _marker_length = 0.028 # Ajustar a la medida real

        _objPoints = np.array([[-_marker_length/2, _marker_length/2, 0], [_marker_length/2, _marker_length/2, 0],
                               [_marker_length/2, -_marker_length/2, 0], [-_marker_length/2, -_marker_length/2, 0]])
        
        for _marker_corner in marker_corners:
            _valid, rvec, tvec = cv2.solvePnP(_objPoints, _marker_corner, camera_matrix, dist_coeff, flags=cv2.SOLVEPNP_IPPE_SQUARE)
            _draw_img = cv2.drawFrameAxes(_draw_img, camera_matrix, dist_coeff, rvec, tvec, _marker_length)

        rvec = np.transpose(rvec)
        tvec = np.transpose(tvec)[0]
        _draw_img = cv2.aruco.drawDetectedMarkers(_draw_img, marker_corners, borderColor=(255, 0, 0))
        _draw_img = cv2.cvtColor(_draw_img, cv2.COLOR_BGR2RGB)
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
        pos = perm_columns(pose[0])
        q = p.getQuaternionFromEuler(pose[1])
        print(np.array(pos, dtype = np.float16))
        p.resetBasePositionAndOrientation(cube, pos, q)
        p.stepSimulation()
    #p.setLinkState(cube, 0, pose[:3], pose[3:])

    p.stepSimulation()
    cv2.imshow('frame', frame)


    if cv2.waitKey(1) == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()

p.disconnect()