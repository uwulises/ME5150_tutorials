import pybullet as p
import pybullet_data
import numpy as np
# Initialize PyBullet
physicsClient = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
useFixedBase = True

# Set gravity and time step
p.setGravity(0, 0, -9.81)
p.setRealTimeSimulation(1)

# Load SCARA robot arm and table
planeId = p.loadURDF("plane.urdf")
tableId = p.loadURDF("../modelos/manipuladores/scara/base_scara.urdf",
                     basePosition=[0, 0, 0.69], useFixedBase=useFixedBase)
initialori = p.getQuaternionFromEuler([0, 0, np.deg2rad(90)]) # initial orientation of the robot
robotId = p.loadURDF("../modelos/manipuladores/scara/scara.urdf",
                     basePosition=[0, 0, 0.69], baseOrientation=initialori,useFixedBase=useFixedBase)
# tool coordinate position
n_tcf = 2


# Create sliders for each joint
q0_slider = p.addUserDebugParameter("q0", -1.57, 1.57, 0)
q1_slider = p.addUserDebugParameter("q1", -1.3, 1.57, 0)
q2_slider = p.addUserDebugParameter("q2", 0, 0.2, 0)


# Create sliders for X, Y, Z
x_slider = p.addUserDebugParameter("X", 0.15, 1.3, 0.6)
y_slider = p.addUserDebugParameter("Y", -1.3, 1.3, 0)
z_slider = p.addUserDebugParameter("Z", 0.7, 0.9, 0.8)

p.addUserDebugLine([0, 0, 0], [0.2, 0, 0], [1, 0, 0], lineWidth=5, parentObjectUniqueId=robotId, parentLinkIndex=2)
p.addUserDebugLine([0, 0, 0], [0, 0.2, 0], [0, 1, 0], lineWidth=5,parentObjectUniqueId=robotId, parentLinkIndex=2)
p.addUserDebugLine([0, 0, 0], [0, 0, 0.2], [0, 0, 1], lineWidth=5,parentObjectUniqueId=robotId, parentLinkIndex=2)

def FK():
        # get the current slider values
        q0 = p.readUserDebugParameter(q0_slider)
        q1 = p.readUserDebugParameter(q1_slider)
        q2 = -p.readUserDebugParameter(q2_slider) # negative for the simulation
        # target position
        q = [q0, q1, q2]
        p.setJointMotorControlArray(robotId, range(
            3), p.POSITION_CONTROL, targetPositions=q)


def IK():
        # get the current slider values
        x = p.readUserDebugParameter(x_slider)
        y = p.readUserDebugParameter(y_slider)
        z = p.readUserDebugParameter(z_slider)
        # target position
        xyz = [x, y, z]
        target = p.calculateInverseKinematics(
            robotId, endEffectorLinkIndex=n_tcf, targetPosition=xyz)
        p.setJointMotorControlArray(robotId, range(
            3), p.POSITION_CONTROL, targetPositions=target)



def main():
    while True:
        #Selects from Inverse or Forward Kinematics
        FK()
        #IK()


if __name__ == "__main__":
    
    main()