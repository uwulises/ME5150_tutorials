#Importacion de librerias
import os
import sys
import pybullet as p
import pybullet_data
import time
import numpy as np
import cv2
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '../herramientas')))
from aruco_huntingv2 import ArucoHunting

# Inicializar PyBullet
physicsClient = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -9.81)
p.setRealTimeSimulation(1)

# Rotaciones útiles en cuaterniones
rot_y90 = p.getQuaternionFromEuler([0, np.deg2rad(-90), 0])
rot_z90 = p.getQuaternionFromEuler([0, 0, np.deg2rad(90)])

# Cargar modelos a utilizar
planeId = p.loadURDF("plane.urdf")
arucoId1 = p.loadURDF("modelos/objetos/aruco_id1/aruco_id1.urdf", globalScaling = 0.6, basePosition=[0.5, 0.1, 0.74], baseOrientation = rot_y90, useFixedBase=True)
arucoId2 = p.loadURDF("modelos/objetos/aruco_id2/aruco_id2.urdf", globalScaling = 0.6, basePosition=[0.5, 0.2, 0.74], baseOrientation = rot_y90, useFixedBase=True)
tableId = p.loadURDF("../modelos/manipuladores/scara/base_scara.urdf", basePosition=[0, 0, 0.69], useFixedBase=True)
table2Id = p.loadURDF("../modelos/manipuladores/scara/base_scara.urdf", basePosition=[0.8, 0, 0.69], useFixedBase=True)
robotId = p.loadURDF("../modelos/manipuladores/scara/scara.urdf", basePosition=[0, 0, 0.69], baseOrientation=rot_z90,useFixedBase=True, )

# Crea sliders para controlar la posición de los arucos
x_slider = p.addUserDebugParameter("x", 0, 0.25, 0)
off_slider = p.addUserDebugParameter("offset", 0.15, 0.4, 0.15)

# Coordenadas del endeffector del robot
n_tcf = 2

# Inicializar caza de arucos
hunter = ArucoHunting()

# TODO: Actualizar la matriz de la cámara usando los valores obtenidos en tu calibración
camera_matrix = np.array([[691., 0. , 289.],[0., 690., 264.], [0., 0., 1.]])
hunter.set_marker_length(0.047) # Dado por cubo simulado
hunter.set_camera_parameters(camera_matrix) # Matriz de la cámara

# No modificar - Calculo de matriz de proyeccion de camara
def cvK2BulletP(width, height, cam_mat, near, far):
    fx, fy, cx, cy = cam_mat[0, 0], cam_mat[1, 1], cam_mat[0, 2], cam_mat[1, 2]
    aspect = width / height
    fov = 2 * np.arctan(height / (2 * fy)) * 180 / np.pi
    projection_matrix = p.computeProjectionMatrixFOV(fov, aspect, near, far)
    return projection_matrix

# No modificar - Obtener imagen de camara simulada
def get_img_cam(width=480,
                height=480, 
                fov=None, 
                near=0.02, 
                far=5, 
                camposition = [0.7, 0.2, 2], 
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

# No modificar - Mover scara a su posición de home
def scara_go_home():
    home = [0, 0, -0.17]
    t = time.time()
    while t + 0.5 > time.time():
        go_q(home)
        p.stepSimulation()
    return 

# Mover scara según posiciones articulares q
def go_q(q):
    global robotId
    t = time.time()
    dt = 0.02 # Opcional, modificar para cambiar la velocidad de movimiento
    p.setJointMotorControlArray(robotId, range(3), p.POSITION_CONTROL, targetPositions=q)
    while t + dt > time.time():
        p.stepSimulation()
    return 

# Función para obtener el siguiente path del scara
def get_scara_path(img):
    path = [] # Lista de posiciones [x, y, z]
    arucos_data = [] # Ejecutar aruco_huntingv2.py y llamar esa clase

    # TODO: Procesar imagen para obtener las detecciones de arucos

    """# Ejemplo de detección de arucos, descomentar para probar
    arucos_data = [{'id': 1, 'position': [0.4, 0, 0.8], 'orientation': [0, 0, 0]}]"""

    if arucos_data != []:
        # TODO: Calcular en base a las detecciones de arucos la siguiente pose del drone
        path = []
        """# Ejemplo de path, descomentar para probar
        p1 = [0.4, 0, 0.8]
        p2 = [0.4, 0.2, 0.8]
        p3 = [0.5, 0.2, 0.8]
        # Crea 100 puntos entre p1 y p2
        for i in range(100):
            path.append(np.sum([p1, [0, 0.2*i/100, 0]], axis=0))
        # Crea 100 puntos entre p2 y p3
        for i in range(100):
            path.append(np.sum([p2, [0.1*i/100, 0, 0]], axis=0))
        # Crea 100 puntos entre p3 y p1
        for i in range(100):
            path.append(np.sum([p3, [-0.1*i/100, -0.2*i/100, 0]], axis=0))"""

    return path

# Sección que actualiza la posición del drone 
t1 = 0
t2 = 0
path = []

while True:
    if t1 + 0.05 < time.time():
        t = [0.3, 0.1, 0.74]
        x = p.readUserDebugParameter(x_slider)
        o = p.readUserDebugParameter(off_slider)
        xyz1 = np.sum([t, [x, - o/2, 0]], axis=0)
        xyz2 = np.sum([t, [x, + o/2, 0]], axis=0)
        p.resetBasePositionAndOrientation(arucoId1, xyz1, rot_y90)
        p.resetBasePositionAndOrientation(arucoId2, xyz2, rot_y90)
        t1 = time.time()

    # Actualizar imagen de camara cada 0.05 segundos
    if t2 + 0.05 < time.time():
        img_RGB, img_segmentada, img_depth = get_img_cam(cam_mat=camera_matrix)
        t2 = time.time()

    # Mover scara a su home, si no hay path
    if len(path) == 0:
        scara_go_home()
        path = get_scara_path(img_RGB)

    # Mover scara a la siguiente pose
    if len(path) > 0:
        next_pose = path.pop(0)
        target = p.calculateInverseKinematics(robotId, endEffectorLinkIndex=n_tcf, targetPosition=next_pose)
        go_q(target)