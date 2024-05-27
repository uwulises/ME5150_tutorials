

# NOT READY YET

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

def cvK2BulletP(K, w, h, near, far):
    """
    cvKtoPulletP converst the K interinsic matrix as calibrated using Opencv
    and ROS to the projection matrix used in openGL and Pybullet.

    :param K:  OpenCV 3x3 camera intrinsic matrix
    :param w:  Image width
    :param h:  Image height
    :near:     The nearest objects to be included in the render
    :far:      The furthest objects to be included in the render
    :return:   4x4 projection matrix as used in openGL and pybullet
    """ 
    f_x = K[0,0]
    f_y = K[1,1]
    c_x = K[0,2]
    c_y = K[1,2]
    A = (near + far)/(near - far)
    B = 2 * near * far / (near - far)

    projection_matrix = [
                        [2/w * f_x,  0,          (w - 2*c_x)/w,  0],
                        [0,          2/h * f_y,  (2*c_y - h)/h,  0],
                        [0,          0,          A,              B],
                        [0,          0,          -1,             0]]
    return np.array(projection_matrix).T.reshape(16).tolist()

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

# Cargar ambiente de simulacion
planeId = p.loadURDF("plane.urdf")

#Dron con 4 helices
drone = p.loadURDF("modelos/drones/matrice100/matrice100.urdf", basePosition=[0, 0, 1], useFixedBase=True)
dron_pose = p.getBasePositionAndOrientation(drone)[0]+p.getEulerFromQuaternion(p.getBasePositionAndOrientation(drone)[1])
movdrone = MoveDrone(dron_pose, vel=5)

# Define the dimensions of checkerboard
CHECKERBOARD = (7, 4)
SIZE = 33 # mm

# stop the iteration when specified
# accuracy, epsilon, is reached or
# specified number of iterations are completed.
criteria = (cv2.TERM_CRITERIA_EPS + 
            cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
  
  
threedpoints = []
twodpoints = []  
#  3D points real world coordinates
objectp3d = np.zeros((1, CHECKERBOARD[0] 
                      * CHECKERBOARD[1], 
                      3), np.float32)

objectp3d[0, :, :2] = np.mgrid[0:CHECKERBOARD[0],
                               0:CHECKERBOARD[1]].T.reshape(-1, 2)*SIZE
prev_img_shape = None


def collect_points(img):
  global threedpoints
  global twodpoints
  grayColor = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
  ret, corners = cv2.findChessboardCorners(
                    grayColor, CHECKERBOARD, 
                    cv2.CALIB_CB_ADAPTIVE_THRESH 
                    + cv2.CALIB_CB_FAST_CHECK + 
                    cv2.CALIB_CB_NORMALIZE_IMAGE)
  if ret == True:
        threedpoints.append(objectp3d)
        corners2 = cv2.cornerSubPix(grayColor, corners, (11, 11), (-1, -1), criteria)
  
        twodpoints.append(corners2)
  return
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
        
        collect_points(img_RGB)
        
        if len(threedpoints) == 15:
            break
        time0 = time.time()

    p.stepSimulation()

cv2.destroyAllWindows()
p.disconnect()

# Calibracion de camara
ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(threedpoints, twodpoints, grayColor.shape[::-1], None, None)
print("Matriz de calibracion: ", mtx)
print("Distorsion: ", dist)

