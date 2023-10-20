#Importacion de librerias
import os
import sys
import pybullet as p
import pybullet_data
import time
import numpy as np
import cv2
import matplotlib.pyplot as plt

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '../herramientas')))
from movedrone import MoveDrone
from aruco import ArucoHunting

path = 'multimedia/tello_real_cut.mp4'
"""
Parte detección de ArUco
"""


"""
Parte de simulación
"""

# Initialize PyBullet
physicsClient = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.configureDebugVisualizer(p.COV_ENABLE_KEYBOARD_SHORTCUTS, 0)

# Set gravity and time step
p.setGravity(0, 0, -9.81)
#p.setTimeStep(1 / 240)
p.setRealTimeSimulation(1)

# Cargar ambiente de simulacion
planeId = p.loadURDF("plane.urdf")

#Dron con 4 helices
#drone = p.loadURDF("modelos/drones/djitello/djitello.urdf", basePosition=[0, 0, 1], useFixedBase=True)	    
drone = p.loadURDF("modelos/drones/matrice100/matrice100.urdf", basePosition=[0, 0, 1], useFixedBase=True)
dron_pose = p.getBasePositionAndOrientation(drone)[0] + p.getEulerFromQuaternion(p.getBasePositionAndOrientation(drone)[1])
movdrone = MoveDrone(dron_pose, vel=1)


def get_img_cam(width=240, height=240, fov=60, near=0.02, far=4, camposition=[1, 0, 1.5], camorientation = [0, -90, 0], distance=0.1):
    """
    Función para obtener imagen de la cámara simulada
    """
    aspect = width / height
    yaw, pitch, roll = camorientation
    view_matrix = p.computeViewMatrixFromYawPitchRoll(camposition, distance, yaw, pitch, roll, upAxisIndex = 2)
    projection_matrix = p.computeProjectionMatrixFOV(fov, aspect, near, far)
    _, _, rgbaImg, depthImg, segImg = p.getCameraImage(width, height, view_matrix, projection_matrix, renderer=p.ER_BULLET_HARDWARE_OPENGL,flags = p.ER_SEGMENTATION_MASK_OBJECT_AND_LINKINDEX)
    rgbaImg = cv2.cvtColor(rgbaImg, cv2.COLOR_BGR2RGB)
    return rgbaImg, segImg, depthImg


# TODO: Cargar AruCo markers a la simulación y ubicar en punto fijo

# TODO: Completar lista de poses con esas poses relativas
""""
[x, y, z, roll, pitch, yaw] en [m, m, m, rad, rad, rad]
"""

list_poses= [[0, 0, 1, 0, 0, 0], [0, 0, 1, 0, 0, np.pi/2], [0, 0, 1.5, 0, 0, np.pi], [0, 0, 1, 0, 0, 3*np.pi/2]]

#ciclo basico de la simulacion
time0 = time.time()
while True:

    # Para cada pose en la lista de poses
    for pose in list_poses:

        # Setear pose objetivo
        movdrone.set_target_pose(pose)

        # Mientras no se llegue a la pose objetivo
        while not movdrone.is_on_target():

            # Mover dron un poco más cerca de la pose objetivo
            movdrone.update_pose()

            # Actualizar pose del dron en simulacion
            ori = p.getQuaternionFromEuler(movdrone.pose[3:])
            p.resetBasePositionAndOrientation(drone, movdrone.pose[:3], ori)
            
            # Obtener imagen de camara cada 0.02 segundos
            if time0 + 0.02 < time.time():
                # Pose de la camara
                camposition = movdrone.pose[:3]+[0,0,-0.2] # usar -0.1 para DJI Tello
                camorientation = [movdrone.get_pose()[5] * 180/np.pi, -90, 0]

                # Actualizar imagen de camara
                img_RGB, img_segmentada, img_depth = get_img_cam(camposition = camposition, camorientation = camorientation)
                time0 = time.time()
    