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

### funcion para obtener imagen de camara simulada
def get_img_cam(width=240, height=240, fov=60, near=0.02, far=4, camposition=[1, 0, 1.5], camorientation = [0, -90, 0], distance=0.1):
    aspect = width / height
    yaw, pitch, roll = camorientation
    view_matrix = p.computeViewMatrixFromYawPitchRoll(camposition, distance, yaw, pitch, roll, upAxisIndex = 2)
    projection_matrix = p.computeProjectionMatrixFOV(fov, aspect, near, far)
    _, _, rgbaImg, depthImg, segImg = p.getCameraImage(width, height, view_matrix, projection_matrix, renderer=p.ER_BULLET_HARDWARE_OPENGL,flags = p.ER_SEGMENTATION_MASK_OBJECT_AND_LINKINDEX)
    rgbaImg = cv2.cvtColor(rgbaImg, cv2.COLOR_BGR2RGB)
    return rgbaImg, segImg, depthImg

# Initialize PyBullet
physicsClient = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.configureDebugVisualizer(p.COV_ENABLE_KEYBOARD_SHORTCUTS, 0)

# Set gravity and time step
p.setGravity(0, 0, -9.81)
p.setTimeStep(1 / 240) # Para el dron se recomienda trabajar con stepSimulation
#p.setRealTimeSimulation(1) # 0 Desactiva la simulacion en tiempo real

# Cargar ambiente de simulacion
planeId = p.loadURDF("plane.urdf")

#Dron con 4 helices
#drone = p.loadURDF("modelos/drones/djitello/djitello.urdf", basePosition=[0, 0, 1], useFixedBase=True)	    
drone = p.loadURDF("modelos/drones/matrice100/matrice100.urdf", basePosition=[0, 0, 1], useFixedBase=True)
dron_pose = p.getBasePositionAndOrientation(drone)[0]+p.getEulerFromQuaternion(p.getBasePositionAndOrientation(drone)[1])
movdrone = MoveDrone(dron_pose, vel=5)

#ciclo basico de la simulacion
time0 = time.time()
while True:
    # Leer teclado
    keys = p.getKeyboardEvents()

    # Mover dron
    movdrone.move_by_key(keys)
    movdrone.update_pose()
    ori = p.getQuaternionFromEuler(movdrone.pose[3:])
    p.resetBasePositionAndOrientation(drone, movdrone.pose[:3], ori)
    
    # Obtener imagen de camara
    if time0 + 0.1 < time.time():
        # Pose de la camara
        camposition = movdrone.pose[:3] + [0, 0, -0.15] # usar z = -0.1 para DJI Tello
        camorientation = [movdrone.get_pose()[5] * 180/np.pi - 90, -30, 0]

        # Actualizar imagen de camara
        img_RGB, img_segmentada, img_depth = get_img_cam(camposition = camposition, camorientation = camorientation)
        time0 = time.time()
    
    p.stepSimulation()