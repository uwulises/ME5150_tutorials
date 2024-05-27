#Importacion de librerias
import os
import sys
import pybullet as p
import pybullet_data
import time
import numpy as np
import cv2
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '../herramientas')))
from movedronev2 import MoveDrone
from aruco_huntingv2 import ArucoHunting

# Initialize PyBullet
physicsClient = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.configureDebugVisualizer(p.COV_ENABLE_KEYBOARD_SHORTCUTS, 0)
p.setGravity(0, 0, -9.81)
p.setTimeStep(1 / 240)

# Cargar ambiente de simulacion
planeId = p.loadURDF("plane.urdf")
rot_z180 = p.getQuaternionFromEuler([0, 0, np.pi])
arucoId1 = p.loadURDF("modelos/objetos/aruco_id1/aruco_id1.urdf", basePosition=[1.5, -0.2, 0.7], baseOrientation = rot_z180, useFixedBase=True)
arucoId2 = p.loadURDF("modelos/objetos/aruco_id2/aruco_id2.urdf", basePosition=[3.5, 0.2, 0.5], baseOrientation = rot_z180, useFixedBase=True)
arucoId3 = p.loadURDF("modelos/objetos/aruco_id3/aruco_id3.urdf", basePosition=[3, 0, 0.2], baseOrientation = rot_z180, useFixedBase=True)
cube = p.loadURDF("modelos/objetos/cubo.urdf", basePosition=[2.5, 0, 0.1], useFixedBase=True)
drone = p.loadURDF("modelos/drones/matrice100/matrice100.urdf", basePosition=[0, 0, 1], useFixedBase=True)
dron_pose = p.getBasePositionAndOrientation(drone)[0]+p.getEulerFromQuaternion(p.getBasePositionAndOrientation(drone)[1])

# Posicion final del drone, se sabe que está en -0.5 en x respecto al arucoId3
final_pose = p.getBasePositionAndOrientation(cube)[0]

movdrone = MoveDrone(dron_pose, vel = 3) # Clase para mover el drone
hunter = ArucoHunting() # Clase para cazar aruco

# TODO: Actualizar la matriz de la cámara usando los valores obtenidos en tu calibración
camera_matrix = np.array([[691.,0. , 289.],[0., 690., 264.], [0., 0., 1.]])
hunter.set_marker_length(0.078) # Dado por cubo simulado
hunter.set_camera_parameters(camera_matrix) # Matriz de la cámara

# Calculo de matriz de proyeccion de camara
def cvK2BulletP(width, height, cam_mat, near, far):
    fx, fy, cx, cy = cam_mat[0, 0], cam_mat[1, 1], cam_mat[0, 2], cam_mat[1, 2]
    aspect = width / height
    fov = 2 * np.arctan(height / (2 * fy)) * 180 / np.pi
    projection_matrix = p.computeProjectionMatrixFOV(fov, aspect, near, far)
    return projection_matrix

# Obtener imagen de camara simulada
def get_img_cam(width=480,
                height=480, 
                fov=None, 
                near=0.02, 
                far=5, 
                camposition=[1, 0, 1.5], 
                camorientation = [0, -90, 0], 
                distance=0.1, 
                cam_mat = None):

    aspect = width / height
    yaw, pitch, roll = camorientation
    view_matrix = p.computeViewMatrixFromYawPitchRoll(camposition, distance, yaw, pitch, roll, upAxisIndex = 2)
    if fov == None:
        projection_matrix = cvK2BulletP(width, height, cam_mat, near, far)
    else: 
        projection_matrix = p.computeProjectionMatrixFOV(fov, aspect, near, far)
    _, _, rgbaImg, depthImg, segImg = p.getCameraImage(width, height, view_matrix, projection_matrix, renderer=p.ER_BULLET_HARDWARE_OPENGL,flags = p.ER_SEGMENTATION_MASK_OBJECT_AND_LINKINDEX)
    rgbaImg = cv2.cvtColor(rgbaImg, cv2.COLOR_BGR2RGB)
    return rgbaImg, segImg, depthImg

# Procesamiento de imagen
def get_drone_next_pose(img):
    drone_next_pose = None # Siguiente pose del drone [x, y, z, roll, pitch, yaw]
    arucos_data = None # Información de los arucos detectados
    # TODO: Procesar imagen para obtener las detecciones de arucos

    # arucos_data = hunter.poses #descomentar cuando se tenga la información de los arucos
    if arucos_data is not None:
        # TODO: Calcular en base a las detecciones de arucos la siguiente pose del drone
        drone_next_pose = [1, 0, 1] + [0, 0, 0] # Modificar
    return drone_next_pose

# Sección que actualiza la posición del drone 
t0 = 0
while True:
    # Actualizar imagen de camara cada 0.1 segundos
    if t0 + 0.1 < time.time():
        # Pose de la camara
        camposition = movdrone.pose[:3] + [0, 0, -0.15] # usar z = -0.1 para DJI Tello
        camorientation = [movdrone.pose[5] * 180/np.pi - 90, -20, 0]

        # Actualizar imagen de camara
        img_RGB, img_segmentada, img_depth = get_img_cam(camposition = camposition, camorientation = camorientation, cam_mat=camera_matrix)
        t0 = time.time()
    
    """ Se puede activar el control por teclas o por detección de arucos """
    move_with_keys = False
    if move_with_keys:
        # Mover drone con teclas, solo para pruebas
        movdrone.move_by_key(p.getKeyboardEvents())
    else:
        # Mover drone con detección de arucos
        next_pose = get_drone_next_pose(img_RGB) # Procesar imagen para obtener siguiente pose del drone
        if next_pose is not None:
            movdrone.set_target_pose(next_pose)

    movdrone.update_pose() # Actualizar pose del drone
    dron_position = movdrone.pose[:3]
    dron_orientation = p.getQuaternionFromEuler(movdrone.pose[3:])
    p.resetBasePositionAndOrientation(drone, dron_position, dron_orientation)
    p.stepSimulation()

    # Si el drone está cerca de la pose pedida, terminar
    dif = np.array(final_pose) - np.array(movdrone.pose[:3])
    if np.linalg.norm(dif) < 0.25:
        break

