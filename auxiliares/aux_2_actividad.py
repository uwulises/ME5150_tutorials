import pybullet as p
import pybullet_data
import time
import numpy as np

# Initialize PyBullet
physicsClient = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
useFixedBase = True #Para que el brazo no flote o se desplace

# Load KR6 robot arm and table
planeId = p.loadURDF("plane.urdf")
robotId = p.loadURDF("modelos/manipuladores/kuka/kr6_2.urdf", basePosition=[0, 0, 0],useFixedBase=useFixedBase)
tableId = p.loadURDF("table/table.urdf", basePosition=[1.5, 0, 0],useFixedBase=useFixedBase)
trayUid = p.loadURDF("tray/traybox.urdf",basePosition=[1.2,0,0.65])
trayUid = p.loadURDF("tray/traybox.urdf",basePosition=[-0.2,-1,0])

# Set gravity and time step
p.setGravity(0, 0, -9.81)
p.setTimeStep(1 / 240)


# Get the joints info
jointInfo0 = p.getJointInfo(robotId, 0)
jointInfo1 = p.getJointInfo(robotId, 1)
jointInfo2 = p.getJointInfo(robotId, 2)
jointInfo3 = p.getJointInfo(robotId, 3)
jointInfo4 = p.getJointInfo(robotId, 4)
jointInfo5 = p.getJointInfo(robotId, 5)

# Define the joint indices for each link of the robot
# num_joints = p.getNumJoints(robotId)
# link_indices = [joint_index for joint_index in range(num_joints)]

# # Create sliders for each joint
# sliders = [p.addUserDebugParameter(f"Link {i+1}", -np.pi, np.pi, 0) for i in range(num_joints)]

# def move_robot():
#     while True:
#         # Get the slider values
#         angles = [p.readUserDebugParameter(slider) for slider in sliders]

#         # Set the joint angles of the robot
#         for i, angle in enumerate(angles):
#             p.setJointMotorControl2(robotId, link_indices[i], p.POSITION_CONTROL, targetPosition=angle)

#         # Step the simulation
#         p.stepSimulation()
#         time.sleep(1/240)  # Control the simulation speed

# # Call the move_robot function
# move_robot()

# A1 -> A6
path_KR6_2 = [[1.65,1.16,0.33,1.52,0,0], [1.65,0.36,0.33,1.52,0,0], [0.13,0.36,0.33,1.52,0,0], [0.13,0.6,0.33,3.14,-0.43,1.32], [0.13,0.36,0.33,1.52,0,0], [1.65,0.36,0.33,1.52,0,0], [1.65,1.16,0.33,1.52,0,0] ]

# Ciclos por instrucción
rate = 2000

# Run the simulation
while True:
    p.stepSimulation()

    for i in range(len(path_KR6_2)):
        j = i*rate

        while (j/(rate*(i+1)) <= 1):
            # Set the position of the robot arm
            p.setJointMotorControlArray(robotId, range(6), p.POSITION_CONTROL, targetPositions=path_KR6_2[i])
            # Step the simulation
            p.stepSimulation()
            
            j+=1
