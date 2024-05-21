import os
import sys
import pybullet as p
import pybullet_data
import numpy as np
import cv2
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '../herramientas')))

from aruco_hunting import ArucoHunting

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

def perm_columns(pos):
    # TODO -------- Parte 2.c: Permutar valores ejes coordenados para ajustar a coordenadas PyBullet
    n_position = np.zeros(3)
    n_position[0] = pos[2] 
    n_position[1] = pos[0]
    n_position[2] = -pos[1]
    return n_position

cap = cv2.VideoCapture(0)

hunter = ArucoHunting()
# TODO -------- Parte 2.b: Cambiar el valor del largo del marcador
hunter.set_marker_length(0.036) # en metros

# TODO -------- Parte 2.c: Cambiar los valores de la matriz de la cámara y coeficientes de distorsión
camera_matrix = np.array([[691. ,0., 289.],[  0. ,690., 264.], [0. ,  0. ,  1.]])
dist_coeff = np.array([[-0.1951 ,  1.474  ,  0.01244 ,-0.00816, -2.896  ]])
hunter.set_camera_parameters(camera_matrix, dist_coeff)

while cap.isOpened():
    ret, frame = cap.read()
    hunter.update_image(frame)
    hunter.update_pose_and_corners()

    aruco_pose = hunter.pose

    # Si se detecta un marcador
    if aruco_pose is not None:
        # Permutar ejes coordenados para ajustar a PyBullet
        position = perm_columns(aruco_pose[0])
        # Convertir a cuaternión
        orientation = p.getQuaternionFromEuler(aruco_pose[1])
        # Actualizar posición y orientación del cubo
        p.resetBasePositionAndOrientation(cube, position, orientation)

    p.stepSimulation()
    cv2.imshow('ArUco detection', hunter.img_detection)        

    if cv2.waitKey(1) == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()

p.disconnect()