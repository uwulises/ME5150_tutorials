import pybullet as p
import pybullet_data
import numpy as np
import cv2
# Initialize PyBullet
physicsClient = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
useFixedBase = True

# Set gravity and time step
p.setGravity(0, 0, -9.81)
p.setRealTimeSimulation(1)
# Load SCARA robot arm and table
planeId = p.loadURDF("plane.urdf")

omnibaseId = p.loadURDF("modelos/base_movil/omnibase.urdf",
                     basePosition=[0, 0, 0.1],useFixedBase=False)

cubeId = p.loadURDF("modelos/objetos/cubo.urdf",basePosition=[0.5,0,0.1],useFixedBase=False)

# Add sliders to control the position of the cube
x_slider = p.addUserDebugParameter("x", 0, 3,0.6)
y_slider = p.addUserDebugParameter("y", -3, 3, 0)

#function to move the cubeId to the target position using sliders
def move_cube(id):
    # get the current slider values
    x = p.readUserDebugParameter(x_slider)
    y = p.readUserDebugParameter(y_slider)
    z = 0.05
    # target position
    xyz = [x, y, z]
    p.resetBasePositionAndOrientation(id, xyz, [0, 0, 0, 1])
    p.stepSimulation()


### funcion para obtener imagen de camara simulada
def get_img_cam(width=320, height=240, fov=60, near=0.01, far=5, camposition=[1, 0, 1.5],distance=0.0035,yaw=0,pitch=-90,roll=90):
    aspect = width / height
    view_matrix = p.computeViewMatrixFromYawPitchRoll(camposition,distance,yaw,pitch,roll,upAxisIndex=1)
    projection_matrix = p.computeProjectionMatrixFOV(fov, aspect, near, far)
    w_img, h_img, rgbaImg, depthImg, segImg = p.getCameraImage(width, height, view_matrix, projection_matrix, renderer=p.ER_BULLET_HARDWARE_OPENGL,flags = p.ER_SEGMENTATION_MASK_OBJECT_AND_LINKINDEX)
    depth_buffer_opengl = np.reshape(depthImg, [width, height])
    rgbaImg = cv2.cvtColor(rgbaImg, cv2.COLOR_BGR2RGB)

    return rgbaImg, segImg, depthImg

def simple_velcontrol(id, vel_cmd=[0,0,0,0]):
    p.setJointMotorControlArray(id, range(8), p.VELOCITY_CONTROL, targetVelocities=[0,vel_cmd[0],0,-vel_cmd[1],0,vel_cmd[2],0,-vel_cmd[3]])

def main():
    while True:
        move_cube(cubeId)
        pitch = -120
        roll = 90
        yaw = 0
        pose_base = p.getLinkState(omnibaseId, 0)
        pose_base_x = float(pose_base[0][0])
        pose_base_y = float(pose_base[0][1])
        pose_base_z = float(pose_base[0][2])
        cam_pose = np.array([pose_base_x+0.1,pose_base_y - 0.1,pose_base_z+0.06])
        img_RGB, img_segmentada, img_depth = get_img_cam(camposition=cam_pose,roll=roll,pitch=pitch,yaw=yaw)
        simple_velcontrol(omnibaseId, vel_cmd=[1,1,1,1])

if __name__ == "__main__":
    
    main()