import pybullet as p
import pybullet_data
import time
import cv2
import numpy as np

# Inicializar PyBullet
physicsClient = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())  # Para encontrar los archivos URDF en pybullet_data

# Crear el suelo
floor = p.loadURDF("plane.urdf")

# Crear el robot (base móvil)
omnibase = p.loadURDF("modelos/base_movil/omnibase.urdf", [0, 0, 0.05]) 
#cubeId = p.loadURDF("Pybullet/ME5150_tutorials/modelos/base_movil/pybulletSimulation/cubo.urdf",basePosition=[0.5,0,0.1],useFixedBase=False)
# Establecer gravedad y paso de tiempo
p.setGravity(0, 0, -9.81)
p.setTimeStep(1 / 240)

def get_img_cam(width=320, height=240, fov=60, near=0.01, far=5, camposition=[1, 0, 1.5],distance=0.0035,yaw=0,pitch=-90,roll=90):
    aspect = width / height
    view_matrix = p.computeViewMatrixFromYawPitchRoll(camposition,distance,yaw,pitch,roll,upAxisIndex=1)
    projection_matrix = p.computeProjectionMatrixFOV(fov, aspect, near, far)
    rgbaImg= p.getCameraImage(width, height, view_matrix, projection_matrix, renderer=p.ER_BULLET_HARDWARE_OPENGL,flags = p.ER_SEGMENTATION_MASK_OBJECT_AND_LINKINDEX)[2]
    #revisar si resultó usar el [2] para solo acceder al rgb.
    rgbaImg = cv2.cvtColor(rgbaImg, cv2.COLOR_BGR2RGB)
    return rgbaImg


def main():
    while True:
        pitch = 260
        roll = 90
        yaw = 0
        pose_base = p.getLinkState(omnibase, 0)
        pose_base_x = float(pose_base[0][0])
        pose_base_y = float(pose_base[0][1])
        pose_base_z = float(pose_base[0][2])
        cam_pose = np.array([pose_base_x+0.1,pose_base_y - 0.1,pose_base_z+0.06])
        img_RGB= get_img_cam(camposition=cam_pose,roll=roll,pitch=pitch,yaw=yaw)
        cv2.waitKey(1)
        p.stepSimulation()
        time.sleep(0.01)

if __name__ == "__main__":
    
    main()