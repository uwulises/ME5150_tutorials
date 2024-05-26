#Importacion de librerias
import os
import sys
import pybullet as p
import pybullet_data
import time
import numpy as np
import cv2
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '../herramientas')))
from movedrone import MoveDrone

# Initialize PyBullet
physicsClient = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.configureDebugVisualizer(p.COV_ENABLE_KEYBOARD_SHORTCUTS, 0)

# Set gravity and time step
p.setGravity(0, 0, -9.81)
p.setTimeStep(1 / 240) # Para el dron se recomienda trabajar con stepSimulation

# Cargar ambiente de simulacion
planeId = p.loadURDF("plane.urdf")
arucoId1 = p.loadURDF("modelos/objetos/aruco_box/aruco_id1.urdf", basePosition=[0, 0, 0.5], useFixedBase=True)
drone = p.loadURDF("modelos/drones/matrice100/matrice100.urdf", basePosition=[0, 0, 1], useFixedBase=True)
dron_pose = p.getBasePositionAndOrientation(drone)[0]+p.getEulerFromQuaternion(p.getBasePositionAndOrientation(drone)[1])

movdrone = MoveDrone(dron_pose, vel = 5) # Clase para mover el drone

# Obtener imagen de camara simulada
def get_img_cam(width=240, height=240, fov=60, near=0.02, far=4, camposition=[1, 0, 1.5], camorientation = [0, -90, 0], distance=0.1):
    aspect = width / height
    yaw, pitch, roll = camorientation
    view_matrix = p.computeViewMatrixFromYawPitchRoll(camposition, distance, yaw, pitch, roll, upAxisIndex = 2)
    projection_matrix = p.computeProjectionMatrixFOV(fov, aspect, near, far)
    _, _, rgbaImg, depthImg, segImg = p.getCameraImage(width, height, view_matrix, projection_matrix, renderer=p.ER_BULLET_HARDWARE_OPENGL,flags = p.ER_SEGMENTATION_MASK_OBJECT_AND_LINKINDEX)
    rgbaImg = cv2.cvtColor(rgbaImg, cv2.COLOR_BGR2RGB)
    return rgbaImg, segImg, depthImg

# Procesamiento de imagen
def img_process(img):
    drone_next_pose = None

    return drone_next_pose

time0 = time.time()-0.2
while True:
    # Actualizar imagen de camara cada 0.1 segundos
    if time0 + 0.1 < time.time():
        # Pose de la camara
        camposition = movdrone.pose[:3] + [0, 0, -0.15] # usar z = -0.1 para DJI Tello
        camorientation = [movdrone.get_pose()[5] * 180/np.pi - 90, -30, 0]

        # Actualizar imagen de camara
        img_RGB, img_segmentada, img_depth = get_img_cam(camposition = camposition, camorientation = camorientation)
        time0 = time.time()

    next_pose = img_process(img_RGB)
    if next_pose is not None:
        movdrone.set_target_pose(next_pose)

    movdrone.update_pose()
    ori = p.getQuaternionFromEuler(movdrone.pose[3:])
    p.resetBasePositionAndOrientation(drone, movdrone.pose[:3], ori)
    p.stepSimulation()