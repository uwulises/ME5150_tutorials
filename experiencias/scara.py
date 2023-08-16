import pybullet as p
import pybullet_data

# Initialize PyBullet
physicsClient = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
useFixedBase = True

# Set gravity and time step
p.setGravity(0, 0, -9.81)
p.setTimeStep(1 / 240)

# Load SCARA robot arm and table
planeId = p.loadURDF("plane.urdf")
tableId = p.loadURDF("table/table.urdf",
                     basePosition=[0.3, 0, 0], useFixedBase=useFixedBase)
robotId = p.loadURDF("../brazos/scara_fcfm_model/scara.urdf",
                     basePosition=[0, 0, 0.63], useFixedBase=useFixedBase)
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


def FK():
    while True:
        # get the current slider values
        q0 = p.readUserDebugParameter(q0_slider)
        q1 = p.readUserDebugParameter(q1_slider)
        q2 = p.readUserDebugParameter(q2_slider)
        # target position
        q = [q0, q1, q2]
        p.setJointMotorControlArray(robotId, range(
            3), p.POSITION_CONTROL, targetPositions=q)
        p.stepSimulation()


def IK():
    while True:
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
        p.stepSimulation()


if __name__ == "__main__":
    # IK()
    FK()
