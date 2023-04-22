import pybullet as p
import pybullet_data
import time
import numpy as np
from camera import get_img_rgba, pose_object, detect_borders
import cv2
# Initialize PyBullet
physicsClient = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
useFixedBase = True

# Set gravity and time step
p.setGravity(0, 0, -9.81)
p.setTimeStep(1 / 240)

# Load KR6 robot arm and table
planeId = p.loadURDF("plane.urdf")
#robotId = p.loadURDF("brazos/kuka_model/kr6_2.urdf", basePosition = [0, 0, 0], useFixedBase = useFixedBase)
tableId = p.loadURDF("table/table.urdf", basePosition = [0.3, 0, 0], useFixedBase = useFixedBase)
robotId = p.loadURDF("brazos/scara_fcfm_model/scara.urdf", basePosition = [0, 0, 0.63], useFixedBase = useFixedBase)
# tool coordinate position
n_tcf = 2


# Create sliders for X, Y, Z
x_slider = p.addUserDebugParameter("X", 0, 1, 0.6)
y_slider = p.addUserDebugParameter("Y", -1, 1, 0)
z_slider = p.addUserDebugParameter("Z", 0.6, 0.9, 0.9)

# Run the simulation
while True:

    # get the current slider values
    x = p.readUserDebugParameter(x_slider)
    y = p.readUserDebugParameter(y_slider)
    z = p.readUserDebugParameter(z_slider)
    
    # target position
    xyz = [x,y,z]
    
    # calculate joint target
    target = p.calculateInverseKinematics(robotId, endEffectorLinkIndex = n_tcf, targetPosition = xyz)
    p.setJointMotorControlArray(robotId, range(3), p.POSITION_CONTROL, targetPositions = target)
    
    imagen_RGB = get_img_rgba(camposition=[0.5, 0, 1.5])
    cv2.imshow('Pose object',pose_object(imagen_RGB))
    cv2.imshow('Edges',detect_borders(imagen_RGB))
    cv2.waitKey(1)
    p.stepSimulation()

    