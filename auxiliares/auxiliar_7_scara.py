import pybullet as p
import pybullet_data
import time
import numpy as np
import cv2
import os
import sys
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '../herramientas')))
from aruco import ArucoHunting

# Initialize PyBullet
physicsClient = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
useFixedBase = True

# Set gravity and time step
p.setGravity(0, 0, -9.81)
p.setRealTimeSimulation(1)

# Load SCARA robot arm and table
planeId = p.loadURDF("plane.urdf")
baseId = p.loadURDF("../modelos/manipuladores/scara/base_scara.urdf",
                     basePosition=[0, 0, 0.69], useFixedBase=useFixedBase)
robotId = p.loadURDF("../modelos/manipuladores/scara/scara.urdf", basePosition = [0, 0, 0.69], baseOrientation= p.getQuaternionFromEuler([np.deg2rad(0),np.deg2rad(0),np.deg2rad(180)]),useFixedBase = useFixedBase)
arucoId = p.loadURDF("../modelos/objetos/ArucoBox/model.urdf", basePosition = [-0.4, 0.2, 1])
# tool coordinate position
n_tcf = 2




def cvK2BulletP(K, w, h, near, far):
        """
        https://pybullet.org/Bullet/phpBB3/viewtopic.php?t=12901
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
        #The transpose is needed for respecting the array structure of the OpenGL
        return np.array(projection_matrix).T.reshape(16).tolist()

### funcion para obtener imagen de camara simulada
def get_img_cam(width=1920, height=1080, near=0.2, far=2, camposition=[-0.5, 0.2, 1.5],distance=0.05,yaw=0,pitch=-90,roll=0):
    K=np.array([[1080., 0., 290.],[0., 1072., 250.],[0., 0., 1.]])
    aspect = width / height
    view_matrix = p.computeViewMatrixFromYawPitchRoll(camposition,distance,yaw,pitch,roll,upAxisIndex=2)
    projection_matrix = cvK2BulletP(K, width, height, near, far)
    w_img, h_img, rgbaImg, depthImg, segImg = p.getCameraImage(width, height, view_matrix, projection_matrix, renderer=p.ER_BULLET_HARDWARE_OPENGL,flags = p.ER_SEGMENTATION_MASK_OBJECT_AND_LINKINDEX)
    depth_buffer_opengl = np.reshape(depthImg, [width, height])
    rgbaImg = cv2.cvtColor(rgbaImg, cv2.COLOR_BGR2RGB)
    return rgbaImg


def move_scara(xyz=[-0.4,0.6,0.2]):
    offset_z = 0.69 #considering the base height
    xyz[2] = xyz[2] + offset_z
    target = p.calculateInverseKinematics(robotId, endEffectorLinkIndex = n_tcf, targetPosition = xyz)
    p.setJointMotorControlArray(robotId, range(3), p.POSITION_CONTROL, targetPositions = target)


aruco_look = ArucoHunting()
aruco_look.set_marker_length(0.04)
aruco_look.camera_matrix = np.array([[1080., 0., 290.],[0., 1072., 250.],[0., 0., 1.]])
aruco_look.dist_coeff= np.array([[-1.125,  7.71, -0.044,  0.0143, -4.105]])

# Run the simulation
while True:

    img_RGB = get_img_cam()
    #resize
    img_RGB = cv2.resize(img_RGB, (640, 480))
    aruco_look.update_image(img_RGB)
    aruco_look.update_pose_and_corners()

    #pose aruco
    pose = aruco_look.pose


    move_scara()

  
    cv2.imshow("Seg", aruco_look.img_detection)
    #waitkey esc
    if cv2.waitKey(1) == 27:
        break

    