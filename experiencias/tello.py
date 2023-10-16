#Importacion de librerias
import pybullet as p
import pybullet_data
import time
import numpy as np
import keyboard
import cv2

class MoveDrone:
    def __init__(self):
        self.pose = [0, 0, 1, 0, 0, 0]
        self.vel = 15 # velocidad de movimiento
        self.target_pose = [0, 0, 1, 0, 0, 0]

    def update_pose(self):
        dif = np.array(self.target_pose) - np.array(self.pose)
        self.pose += dif * self.vel * 1/240

    def get_pose(self):
        return self.pose
    
    def set_target_pose(self, pose):
        self.target_pose = pose

    def move_by_key(self, keys):
        d = 0.001 # variación de posición
        d_degrees = 0.001 # variación de ángulo
        if keys.get(114)==1:
            # Go up, R
            self.target_pose[2] += d
        elif keys.get(102)==1:
            # Go down, F
            self.target_pose[2] -= d

        elif keys.get(119)==1:
            # Go forward, W
            self.target_pose[0] += d * np.cos(self.pose[5])
            self.target_pose[1] += d * np.sin(self.pose[5])
        elif keys.get(115)==1:
            # Go back, S
            self.target_pose[0] -= d * np.cos(self.pose[5])
            self.target_pose[1] -= d * np.sin(self.pose[5])
        elif keys.get(97)==1:
            # Go left, A
            self.target_pose[1] += d * np.cos(self.pose[5])
            self.target_pose[0] -= d * np.sin(self.pose[5])
        elif keys.get(100)==1:
            # Go right, D
            self.target_pose[1] -= d * np.cos(self.pose[5])
            self.target_pose[0] += d * np.sin(self.pose[5])
        elif keys.get(113)==1:
            # Go counterclockwise, Q
            self.target_pose[5] += d_degrees
        elif keys.get(101)==1:
            # Go clockwise, E
            self.target_pose[5] -= d_degrees


### funcion para obtener imagen de camara simulada
def get_img_cam(width=240, height=240, fov=60, near=0.02, far=4, camposition=[1, 0, 1.5], camorientation = [0, -90, 0], distance=0.1):
    aspect = width / height
    yaw, pitch, roll = camorientation
    view_matrix = p.computeViewMatrixFromYawPitchRoll(camposition, distance, yaw, pitch, roll, upAxisIndex = 2)
    projection_matrix = p.computeProjectionMatrixFOV(fov, aspect, near, far)
    _, _, rgbaImg, depthImg, segImg = p.getCameraImage(width, height, view_matrix, projection_matrix, renderer=p.ER_BULLET_HARDWARE_OPENGL,flags = p.ER_SEGMENTATION_MASK_OBJECT_AND_LINKINDEX)
    #depth_buffer_opengl = np.reshape(depthImg, [width, height])
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
drone = p.loadURDF("modelos/drones/djitello/djitello.urdf", basePosition=[0, 0, 1], useFixedBase=True)
movdrone = MoveDrone()

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
        camposition = movdrone.pose[:3]+[0,0,-0.1]
        camorientation = [movdrone.get_pose()[5] * 180/np.pi, -90, 0]

        # Actualizar imagen de camara
        img_RGB, img_segmentada, img_depth = get_img_cam(camposition = camposition, camorientation = camorientation)
        time0 = time.time()
    
    p.stepSimulation()